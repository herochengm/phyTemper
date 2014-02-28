/*
 * ======== Standard MSP430 includes ========
 */
#include <msp430.h>

/*
 * ======== Grace related includes ========
 */
#include <ti/mcu/msp430/Grace.h>
#include "mlx90615.h"
#include "sendHeadphone.h"
#define LED_LOW_BAT_PIN	(1<<0)
#define LED_LOW_BAT_DIR	P2DIR
#define LED_LOW_BAT_REN	P2REN
#define LED_LOW_BAT_OUT	P2OUT

#define MLX90615_ADDR	0xb6
#define MDELAY	(3)

#define COMUNICATION_FRE	16//通信频率
#define HEADER_LEN_CLOCK	9	//协议头 9clock
#define	STOP_LEN_CLOCK		6	//协议尾5clock
#define TIMER_CNT_FRE		500//定时器分频时钟
#define HIGH_LEN_CLOCK		2
#define LOW_LEN_CLOCK		1

#define HEADER_LEN			(TIMER_CNT_FRE*HEADER_LEN_CLOCK/COMUNICATION_FRE)
#define STOP_LEN			(TIMER_CNT_FRE*STOP_LEN_CLOCK/COMUNICATION_FRE)
#define	HIGH_BIT_LEN   		(TIMER_CNT_FRE*HIGH_LEN_CLOCK/COMUNICATION_FRE)
#define	LOW_BIT_LEN   		(TIMER_CNT_FRE*HIGH_LEN_CLOCK/COMUNICATION_FRE)
#define HALF_LOW_BIT_LEN	(LOW_BIT_LEN/2)
#define MAX_TRANS_SIZE		32

#define HEADER_LEN_START	(261)//HEADER_LEN - 20
#define HEADER_LEN_END	(301)//HEADER_LEN + 20
#define STOP_LEN_START	(172)//STOP_LEN - 15
#define STOP_LEN_END	(202)//STOP_LEN + 15
#define HIGH_LEN_START	(52)//HIGH_BIT_LEN - 5
#define HIGH_LEN_END	(75)//HIGH_BIT_LEN + 5
#define MCLK	(4)

#define AUDIO_IN	(P1IN&(1<<3))

unsigned int volatile gHighCNT = 0;
unsigned char buffer[MAX_TRANS_SIZE];
unsigned char data_index = 0;
char bit_index = 0;
unsigned char find_start = 0;

static void mdelay(unsigned long mm) {
	while (--mm)
		__delay_cycles(1000 * MCLK);
}
void init_GPIO() {
	LED_LOW_BAT_DIR = LED_LOW_BAT_PIN;
	LED_LOW_BAT_REN &= ~LED_LOW_BAT_PIN;

}

#if 1
static void iic_gpio_init() {
	PIN_SDA_PORT_DIR |= PIN_SDA_PORT_DIR;
	PIN_SCL_PORT_DIR |= PIN_SCL_PORT_NUM;
	PIN_SCL_PORT_REN &= ~PIN_SCL_PORT_NUM;
	PIN_SDA_PORT_REN &= ~PIN_SDA_PORT_NUM;
	PIN_SCL_PORT_OUT |= PIN_SCL_PORT_NUM;
	PIN_SDA_PORT_OUT |= PIN_SDA_PORT_NUM;
}
static void setsda(int data) {
	if (data) {
		PIN_SDA_PORT_OUT |= PIN_SDA_PORT_NUM;

	} else {
		PIN_SDA_PORT_OUT &= ~PIN_SDA_PORT_NUM;
	}
}

static void setscl(int data) {
	if (data) {
		PIN_SCL_PORT_OUT |= PIN_SCL_PORT_NUM;

	} else {
		PIN_SCL_PORT_OUT &= ~PIN_SCL_PORT_NUM;
	}

}
unsigned char getSDA() {

	return (PIN_SDA_PORT_IN & PIN_SDA_PORT_NUM);
}
void setSDAInput() {
	//PIN_SDA_PORT_REN |= PIN_SDA_PORT_NUM;
	PIN_SDA_PORT_DIR &= ~PIN_SDA_PORT_NUM;
	//mSDA_HIGH();
}
void setSDAOutput() {
	//PIN_SDA_PORT_REN |= PIN_SDA_PORT_NUM;
	PIN_SDA_PORT_DIR |= PIN_SDA_PORT_NUM;
	//mSDA_LOW();
}
void I2C_Start() {
	mSDA_OUT;
	mSDA_HIGH();			// Set SDA line
	mSCL_HIGH();			// Set SCL line
	mSDA_LOW();				// Clear SDA line

	mSCL_LOW();				// Clear SCL line
}
static void I2C_Stop() {
	/* assert: scl is low */
	mSDA_OUT;
	mSCL_LOW();				// Clear SCL line
	mSDA_LOW();				// Clear SDA line
	mSCL_HIGH();			// Set SCL line
	mSDA_HIGH();			// Set SDA line
}

int I2C_Send(unsigned char dat, int ack) {
	unsigned char i = 0;
	unsigned char cnt = 0;
	int ret = 0;

	for (i = 0; i < 8; i++) {

		if (dat & 0x80)
			mSDA_HIGH();
		else
			mSDA_LOW();
		mSCL_HIGH();

		mSCL_LOW();
		dat <<= 1;
	}

	mSDA_IN;

	mSCL_HIGH();

	if (ack) {
		if (getSDA()) {
			ret = -1;
		}
	}
	mSCL_LOW();

	//setSDAOutput();
	//mSDA_LOW();
	mSDA_OUT;

	return ret;
}
int IIC_Receive(unsigned char ack) {
	unsigned char dat;
	unsigned char i;
	setSDAInput();
	for (i = 0; i < 8; i++) {
		dat <<= 1;
		mSCL_HIGH();
		if (getSDA())
			dat |= 1;
		mSCL_LOW();
	}
	setSDAOutput();
	if (ack)
		mSDA_LOW();
	mSCL_HIGH();
	mSCL_LOW();
	return dat;
}

unsigned long gTemp = 0;
static unsigned int readTemp() {
	//int ret;
	unsigned long temp = 0;
	int cnt = 0;
	repeat: if (cnt++ > 100) {
		return -1;
	}
	I2C_Stop();
	mdelay(1);
	I2C_Start();
	if (I2C_Send(MLX90615_ADDR, 1) == -1)
		goto repeat;
	if (I2C_Send(0x27, 1) == -1)
		goto repeat;
	//I2C_Stop();

	I2C_Start();

	if (I2C_Send(MLX90615_ADDR | 1, 1) == -1)
		goto repeat;
	temp = IIC_Receive(1) | (IIC_Receive(1) << 8);
	IIC_Receive(1);
	I2C_Stop();
	/*	gTemp = temp;
	 gTemp = temp * 2 - 27315;*/
	return temp;

}

static void EnterSleepSensor() {
	I2C_Start();
	I2C_Send(MLX90615_ADDR, 1);
	I2C_Send(0xc6, 1);
	I2C_Send(0x6d, 0);
	I2C_Stop();
	mSDA_LOW();
}
static void WakeUpSensor() {
	mSCL_LOW();
	mdelay(80);
	mSCL_HIGH();
}
#endif


void interrupt_init() {

	/*******************
	 * Enable P1.3 IO Interrupte
	 */
	P1IE |= 0x08;
	P1IFG &= ~0x8;
	/****
	 * end
	 */
}
void timer_init() {
	TAR = 0;
	TACTL = 0x2e4;
	TACTL = 0x2e0;
}
/*
 *  ======== main ========
 */
void loop()
{
	unsigned char SlaveAddress; 			//Contains device address
	unsigned char command;	  			//Contains the access command
	unsigned int data;		  			//Contains data value
           // Activate Grace-generated configuration
	headPhoneGpio_init();
	SlaveAddress = SA << 1;						//Set device address
	command = RAM_Access | RAM_To; //Form RAM access command + RAM address
	//P1DIR &= ~0x01;
	mdelay(300);
	//interrupt_init();
	iic_gpio_init();
	WakeUpSensor();
	timer_init();
	// >>>>> Fill-in user code here <<<<<
	//_BIS_SR(LPM0_bits + GIE);
	for (;;) {
		data=readTemp();//MemRead((MLX90615_ADDR>>1),0x27);
		if(data!=-1)
		{
			data=data * 2 - 27315;
			buffer[0]=TOKEN_CMD_OBJ_TMP;
			buffer[1]=(data>>8)&0xff;
			buffer[2]=(data)&0xff;

			buffer[3]=~(buffer[0]+buffer[1]+buffer[2])-1;
			sendBytes(buffer,4);
		}

		//	readTemp();
	}
	return ;
}

int main(void) {
	Grace_init();                   // Activate Grace-generated configuration
	loop();
	// >>>>> Fill-in user code here <<<<<
	while (1) {
		P2OUT |= LED_LOW_BAT_PIN;
		mdelay(300);
		P2OUT &= ~LED_LOW_BAT_PIN;
		mdelay(300);
	}
	return (0);
}
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {

	int i = 0;

	P1IFG &= ~0x8;                           // P1.4 IFG cleared
	/***
	 * P1.3  for test
	 */
//	P1IFG &= ~0x08;
//	sendBytes(buffer,10);
	/**
	 * end
	 */
	if (AUDIO_IN)				//上升沿
	{
		TAR = 0;
		//P1IES = BIT6;
	} else {
		//下降沿
		gHighCNT = TAR;
		//P1IES &= ~BIT6;
		if (gHighCNT > HEADER_LEN_START && gHighCNT < HEADER_LEN_END) {
			find_start = 1;
			data_index = 0;
			bit_index = 7;

			return;
		}
		if (find_start == 1) {

			if (gHighCNT > STOP_LEN_START && gHighCNT < STOP_LEN_END) {
				find_start = 0;
				for (i = MAX_TRANS_SIZE - 1; i >= 0; i--) {
					buffer[i] = 0;
				}
				return;
			}

			if (gHighCNT > HIGH_LEN_START && gHighCNT < HIGH_LEN_END) {
				buffer[data_index] |= (1 << bit_index);

			}
			if (bit_index-- <= 0) {
				bit_index = 7;
				data_index++;
			}

		}

	}
}

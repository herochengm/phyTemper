/*
 * mlx90615.h
 *
 *  Created on: 2014-1-21
 *      Author: Administrator
 */

#ifndef MLX90615_H_
#define MLX90615_H_

#define PIN_SCL_PORT_DIR	P1DIR
#define PIN_SCL_PORT_IN		P1IN
#define PIN_SCL_PORT_OUT	P1OUT
#define PIN_SCL_PORT_REN	P1REN
#define PIN_SCL_PORT_NUM	(1<<5)

#define PIN_SDA_PORT_DIR	P1DIR
#define PIN_SDA_PORT_IN		P1IN
#define PIN_SDA_PORT_OUT	P1OUT
#define PIN_SDA_PORT_REN	P1REN
#define PIN_SDA_PORT_NUM	(1<<4)

/*
#define mSDA_LOW    (PIN_SDA_PORT_OUT &= ~PIN_SDA_PORT_NUM)
#define mSDA_HIGH   (PIN_SDA_PORT_OUT |=PIN_SDA_PORT_NUM)
#define mSCL_LOW    (PIN_SCL_PORT_OUT &= ~PIN_SCL_PORT_NUM)
#define mSCL_HIGH   (PIN_SCL_PORT_OUT |= PIN_SCL_PORT_NUM)

#define mSDA_IN     (PIN_SDA_PORT_DIR &= ~PIN_SDA_PORT_NUM)
#define mSDA_OUT    (PIN_SDA_PORT_DIR |= PIN_SDA_PORT_NUM)
#define mSCL_IN     (PIN_SCL_PORT_DIR &= ~PIN_SCL_PORT_NUM)
#define mSCL_OUT    (PIN_SCL_PORT_DIR |= PIN_SCL_PORT_NUM)
*/

//----------------IO bits Set/Clr---------------------
#define ClrP1(n) P1OUT&=(~n)
#define SetP1(n) P1OUT|=n
#define ClrP2(n) P2OUT&=(~n)
#define SetP2(n) P2OUT|=n
#define ClrP3(n) P3OUT&=(~n)
#define SetP3(n) P3OUT|=n
#define ClrP4(n) P4OUT&=(~n)
#define SetP4(n) P4OUT|=n
#define ClrP5(n) P5OUT&=(~n)
#define SetP5(n) P5OUT|=n
#define ClrP6(n) P6OUT&=(~n)
#define SetP6(n) P6OUT|=n
//--------------------IO bits Input/Output--------------------------
#define SetInP1(n) P1DIR&=(~n)
#define SetOutP1(n) P1DIR|=n
#define SetInP2(n) P2DIR&=(~n)
#define SetOutP2(n) P2DIR|=n

//---------------------IO bits PULL Enabled/Disabled----------------
#define PullEnabledP1(n) P1REN|=n
#define PullDisabledP1(n) P1REN&=(~n)
#define PullEnabledP2(n) P2REN|=n
#define PullDisabledP2(n) P2REN&=(~n)

//---------------------IO bits PullUp/PullDown----------------------
#define PullDownP1(n) P1OUT&=(n)
#define PullUpP1(n) P1OUT|=n
#define PullDownP2(n) P2OUT&=(~n)
#define PullUpP2(n) P2OUT|=n

//------------------------IO bits Input-----------
#define P1Input(n) P1IN&n?1:0
#define P2Input(n) P2IN&n?1:0

#define mSDA_LOW()    ClrP1(PIN_SDA_PORT_NUM)
#define mSDA_HIGH()   SetP1(PIN_SDA_PORT_NUM)
#define mSCL_LOW()    ClrP1(PIN_SCL_PORT_NUM)
#define mSCL_HIGH()   SetP1(PIN_SCL_PORT_NUM)

#define mSDA_IN     SetInP1(PIN_SDA_PORT_NUM)
#define mSDA_OUT    SetOutP1(PIN_SDA_PORT_NUM)
#define mSCL_IN     SetInP1(PIN_SCL_PORT_NUM)
#define mSCL_OUT    SetOutP1(PIN_SCL_PORT_NUM)

#define mSDA_IO		P1Input(PIN_SDA_PORT_NUM)

#define ACK	  0
#define	NACK      1

//MLX90614 constants
#define SA				0x5b	// Slave address
#define DEFAULT_SA		0x5B	// Default Slave address
#define RAM_Access		0x20	// RAM access command
#define EEPROM_Access	        0x10	// EEPROM access command
#define RAM_To		0x07	// To address in the ram
//*PROTOTYPES**********************************************************************************
void START_bit(void);
void STOP_bit(void);
unsigned char TX_byte(unsigned char Tx_buffer);
unsigned char RX_byte(unsigned char ack_nack);
void send_bit(unsigned char bit_out);
unsigned char Receive_bit(void);
void MLX90615_init(void);
unsigned int MemRead(unsigned char SlaveAddress, unsigned char command);
void SendRequest(void);
void DummyCommand(unsigned char byte);
float CalcTemp(unsigned int value);
unsigned char PEC_calculation(unsigned char pec[]);

#define DCO_F 8               //定义主时钟频率，1,8,12,16(MHZ)
#define _delay_us(n) __delay_cycles((float)(n)*DCO_F)   //20us以下误差较大
#define _delay_ms(n) __delay_cycles(n*(1000.0f)*DCO_F)
//------------------------------------------------------------------------
#define delay_Tbuf __delay_cycles(4)//16MHZ,62.5ns*76=4.75us
#define delay_Thd  __delay_cycles(1)//16MHZ,62.5ns*5=312.5ns

#endif /* MLX90615_H_ */

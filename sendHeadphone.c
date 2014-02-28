#include <msp430.h>
#define HEADPHONE_NUM	(BIT6)

#define HEADPHONE_OUT	(P1DIR=(P1DIR|HEADPHONE_NUM))
#define HEADPHONE_HIGH	(P1OUT=(P1OUT|HEADPHONE_NUM))
#define HEADPHONE_LOW	(P1OUT=(P1OUT&~HEADPHONE_NUM))

#define BIT_HIGH_CNT	68*2
#define BIT_LOW_CNT		34*2
#define BIT_LOW_LOW		BIT_HIGH_CNT

#define STOP_HIGH_CNT	(BIT_HIGH_CNT*2)
#define START_HIGH_CNT	(BIT_HIGH_CNT*2)
#define START_LOW_CNT	(BIT_HIGH_CNT*2)
#define START_PRE_CNT   32
#define MCLK	(4)


static void __head_udelay(int iu)
{
	while(iu--)
		__delay_cycles(MCLK);
}

void headPhoneGpio_init()
{
	HEADPHONE_OUT;
	HEADPHONE_LOW;
}
static void start()
{

	int i=START_PRE_CNT;
	for(;i>0;i--)
	{
		HEADPHONE_HIGH;
		__head_udelay(BIT_LOW_CNT);
		HEADPHONE_LOW;
		__head_udelay(BIT_LOW_CNT);
	}
	HEADPHONE_HIGH;
	__head_udelay(START_LOW_CNT);
	HEADPHONE_LOW;
	__head_udelay(START_LOW_CNT);
}

static void stop()
{
	HEADPHONE_HIGH;
	__head_udelay(STOP_HIGH_CNT);
	HEADPHONE_LOW;
}

static void sendBit(unsigned char bit)
{
#if 1
	HEADPHONE_HIGH;
	if(bit)
	{
		__head_udelay(BIT_HIGH_CNT);
	}
	else
		__head_udelay(BIT_LOW_CNT);
	HEADPHONE_LOW;
	__head_udelay(BIT_LOW_CNT);
#else
	if(bit)
	{
		HEADPHONE_HIGH;
		__head_udelay(BIT_LOW_CNT);
		HEADPHONE_LOW;
		__head_udelay(BIT_LOW_CNT);
		/*HEADPHONE_HIGH;
		__head_udelay(BIT_LOW_CNT);
		HEADPHONE_LOW;
		__head_udelay(BIT_LOW_CNT);*/
	}
	else
	{
		HEADPHONE_HIGH;
		__head_udelay(BIT_HIGH_CNT);
		HEADPHONE_LOW;
		__head_udelay(BIT_HIGH_CNT);
	}
#endif
}
void sendTestMi()
{
	HEADPHONE_HIGH;
	__head_udelay(BIT_HIGH_CNT);
	HEADPHONE_LOW;
	__head_udelay(BIT_HIGH_CNT);
}
static void sendByte(unsigned char byte)
{
	int i=8;
	for(;i>0;i--)
	{
		sendBit(byte&0x80);
		byte<<=1;
	}
}
void sendBytes(unsigned char *buf,char len)
{

	int i=0;
	start();
	for(;i<len;i++)
	{
		sendByte(buf[i]);
	}
	stop();
}

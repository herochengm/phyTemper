/*
 * sendHeadphone.h
 *
 *  Created on: 2014-1-23
 *      Author: Administrator
 */

#ifndef SENDHEADPHONE_H_
#define SENDHEADPHONE_H_

#define TOKEN_CMD_OBJ_TMP	0XA
#define TOKEN_CMD_ENV_TMP	0X5

void sendBytes(unsigned char *buf,char len);
void headPhoneGpio_init();

typedef  struct
{
	unsigned char cmd_type;//√¸¡Ó
	unsigned char data[2];
	unsigned char sum;

}temperature;

typedef  struct
{
	unsigned char cmd_type;//√¸¡Ó
	unsigned char data[16];
	unsigned char sum;

}ProductID;



#endif /* SENDHEADPHONE_H_ */

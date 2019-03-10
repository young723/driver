#include <REGX52.H>
#include "qst_sw_i2c.h"

static unsigned char QST_SW_I2C_ADDR_W = 0x24;
static unsigned char  QST_SW_I2C_ADDR_R = 0x25;

#define I2C_SCL_PIN		P2_1
#define I2C_SDA_PIN		P2_0

#define I2C_SCL_OUTPUT()			
#define I2C_SCL_INPUT()			
#define I2C_SDA_OUTPUT()			
#define I2C_SDA_INPUT()			
#define I2C_SCL_1()  I2C_SCL_PIN=1		/* SCL = 1 */
#define I2C_SCL_0()  I2C_SCL_PIN=0		/* SCL = 0 */
#define I2C_SDA_1()  I2C_SDA_PIN=1		/* SDA = 1 */
#define I2C_SDA_0()  I2C_SDA_PIN=0		/* SDA = 0 */
#define I2C_SDA_READ()  I2C_SDA_PIN		/* SDA data */


void i2c_GPIO_Config(void);
void i2c_Ack(void);
void i2c_NAck(void);

/*
*********************************************************************************************************
*	? ? ?: i2c_Delay
*	????: I2C?????,??400KHz
*	?    ?:?
*	? ? ?: ?
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	unsigned char i;

	/* 
	 	???????????AX-Pro???????????
		CPU??72MHz?,???Flash??, MDK?????
		?????10?,SCL?? = 205KHz 
		?????7?,SCL?? = 347KHz, SCL?????1.5us,SCL?????2.87us 
	 	?????5?,SCL?? = 421KHz, SCL?????1.25us,SCL?????2.375us 
        
    IAR???????,?????7
	*/
	for (i = 0; i < 10; i++);
}

/*
*********************************************************************************************************
*	? ? ?: i2c_Start
*	????: CPU??I2C??????
*	?    ?:?
*	? ? ?: ?
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* ?SCL????,SDA?????????I2C?????? */
	
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	? ? ?: i2c_Start
*	????: CPU??I2C??????
*	?    ?:?
*	? ? ?: ?
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();

	/* ?SCL????,SDA?????????I2C?????? */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************
*	? ? ?: i2c_SendByte
*	????: CPU?I2C??????8bit??
*	?    ?:_ucByte : ???????
*	? ? ?: ?
*********************************************************************************************************
*/
void i2c_SendByte(unsigned char _ucByte)
{
	unsigned char i;

	/* ????????bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // ????
		}
		_ucByte <<= 1;	/* ????bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	? ? ?: i2c_ReadByte
*	????: CPU?I2C??????8bit??
*	?    ?:?
*	? ? ?: ?????
*********************************************************************************************************
*/
unsigned char i2c_ReadByte(unsigned char ack)
{
	unsigned char i;
	unsigned char value;

	/* ???1?bit????bit7 */
	I2C_SDA_INPUT();	// set data input	
	i2c_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		//I2C_SCL_1();
		//i2c_Delay();
		I2C_SCL_0();
		i2c_Delay();
	}
	
	I2C_SDA_OUTPUT();	// set data output	
	i2c_Delay();
	if(ack==0)
		i2c_NAck();
	else
		i2c_Ack();
	return value;
}

/*
*********************************************************************************************************
*	? ? ?: i2c_WaitAck
*	????: CPU??????,??????ACK????
*	?    ?:?
*	? ? ?: ??0??????,1???????
*********************************************************************************************************
*/
unsigned char i2c_WaitAck(void)
{
	unsigned char re;

	I2C_SDA_1();	/* CPU??SDA?? */
	I2C_SDA_INPUT();	//set data input
	i2c_Delay();
	I2C_SCL_1();	/* CPU??SCL = 1, ???????ACK?? */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU??SDA???? */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	I2C_SDA_OUTPUT();	//set data input
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	? ? ?: i2c_Ack
*	????: CPU????ACK??
*	?    ?:?
*	? ? ?: ?
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU??SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU??1??? */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU??SDA?? */
}

/*
*********************************************************************************************************
*	? ? ?: i2c_NAck
*	????: CPU??1?NACK??
*	?    ?:?
*	? ? ?: ?
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU??SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU??1??? */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************
*	? ? ?: i2c_GPIO_Config
*	????: ??I2C???GPIO,????IO?????
*	?    ?:?
*	? ? ?: ?
*********************************************************************************************************
*/
void i2c_GPIO_Config(void)
{
	i2c_Stop();
}

/*
*********************************************************************************************************
*	? ? ?: i2c_CheckDevice
*	????: ??I2C????,CPU???????,??????????????????
*	?    ?:_Address:???I2C????
*	? ? ?: ??? 0 ????, ??1??????
*********************************************************************************************************
*/
unsigned char i2c_CheckDevice(unsigned char _Address)
{
	unsigned char ucAck;

	QST_SW_I2C_ADDR_W = _Address;
	i2c_GPIO_Config();		/* ??GPIO */
	i2c_Start();		/* ?????? */
	/* ??????+????bit(0 = w, 1 = r) bit7 ?? */
	i2c_SendByte(_Address|I2C_WR);
	ucAck = i2c_WaitAck();	/* ?????ACK?? */
	i2c_Stop();			/* ?????? */

	return ucAck;
}


unsigned char qst_sw_writereg(unsigned char reg_add,unsigned char reg_dat)
{
	i2c_Start();
	i2c_SendByte(QST_SW_I2C_ADDR_W);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_dat);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_Stop();

	return 1;
}

unsigned char qst_sw_readreg(unsigned char reg_add,unsigned char *buf,unsigned char num)
{
	//unsigned char ret;
	unsigned char i;

	i2c_Start();
	i2c_SendByte(QST_SW_I2C_ADDR_W);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);
	if(i2c_WaitAck())
	{
		return 0;
	}

	i2c_Start();
	i2c_SendByte(QST_SW_I2C_ADDR_W+1);		// QST_SW_I2C_ADDR_R
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(num-1);i++){
		*buf=i2c_ReadByte(1);
		buf++;
	}
	*buf=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
}

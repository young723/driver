#ifndef _QST_SW_I2C_H
#define _QST_SW_I2C_H

#define I2C_WR	0		/* ???bit */
#define I2C_RD	1		/* ???bit */

void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(unsigned char _ucByte);
unsigned char i2c_ReadByte(unsigned char ack);
unsigned char i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
unsigned char i2c_CheckDevice(unsigned char _Address);
void i2c_GPIO_Config(void);

unsigned char qst_sw_readreg(unsigned char reg_add,unsigned char *buf,unsigned char num);
unsigned char qst_sw_writereg(unsigned char reg_add,unsigned char reg_dat);

#endif


#ifndef _BSP_I2C_H
#define _BSP_I2C_H

#include <inttypes.h>
#include "stm32f10x.h"

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

void i2c_sw_gpio_config(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(uint8_t ack);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);


uint8_t qst_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat);
uint8_t qst_sw_readreg(uint8_t slave, uint8_t reg_add,uint8_t *buf,uint8_t num);

#endif

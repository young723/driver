#ifndef FIS210X_SW_I2C_H
#define FIS210X_SW_I2C_H


#include <linux/wakelock.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#include <mach/mt_gpio_core.h>

//#define MTK_SW_IIC
//#define QST_SW_IIC

#if defined(MTK_SW_IIC)
extern unsigned char sw_iic_write(unsigned char slave_addr, unsigned char addr, unsigned char para);
extern unsigned char sw_iic_read_multi(unsigned char slave_addr, unsigned char addr, unsigned char *buf, unsigned char nb_char);
#endif
#if defined(QST_SW_IIC)
extern void qst_sw_iic_init(void);
extern uint8_t qst_iic_write(uint8_t slave,uint8_t Addr, uint8_t Data);
extern uint8_t qst_iic_read(uint8_t slave, uint8_t Addr, uint8_t *pData, uint16_t Length);
#endif

#endif

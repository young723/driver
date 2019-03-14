
#ifndef _QST_IIC_H_
#define _QST_IIC_H_

//#define QST_SW_IIC
//#define QST_SW_IIC_MTK
extern uint8_t qst_iic_write(uint8_t slave, uint8_t Addr, uint8_t Data);
extern uint8_t qst_iic_read(uint8_t slave, uint8_t Addr, uint8_t *pData, uint16_t Length);
extern uint8_t qst_iic_read_2(uint8_t slave, uint8_t Addr, uint8_t *pData, uint16_t Length);

#endif


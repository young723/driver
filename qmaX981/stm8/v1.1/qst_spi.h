
#ifndef _QST_SPI_H_
#define _QST_SPI_H_

extern uint8_t qmp6988_spi_write(uint8_t Addr, uint8_t Data);
extern uint8_t qmp6988_spi_read(uint8_t Addr, uint8_t *pData, uint16_t Length);
#if defined(QST_CONFIG_QMAX981)
extern uint8_t qst_qmaX981_spi_write(uint8_t Addr, uint8_t Data);
extern uint8_t qst_qmaX981_spi_read(uint8_t Addr, uint8_t* Buf, uint8_t len);
#endif

#endif


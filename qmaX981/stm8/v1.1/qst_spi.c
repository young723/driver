
#include "stm8s.h"
#include "qst_spi.h"
#include "delay.h"

#if defined(USE_SPI)
uint8_t spi_write_byte(uint8_t value)
{
	while(SPI_GetFlagStatus(SPI_FLAG_TXE)==RESET);
	SPI_SendData(value);
	while(SPI_GetFlagStatus(SPI_FLAG_RXNE)==RESET);
	return SPI_ReceiveData();
}

void qst_spi_cs_write(unsigned char level)
{
	if(level)
	{
#if defined(STM8S103)
		GPIO_WriteHigh(GPIOA,GPIO_PIN_3);		// CS	
#elif defined(STM8S105)
		//GPIO_WriteHigh(GPIOE,GPIO_PIN_5);		// CS	
		//GPIO_WriteHigh(GPIOC,GPIO_PIN_4);
		GPIO_WriteHigh(GPIOF,GPIO_PIN_4);
#endif	
	}
	else
	{
#if defined(STM8S103)
		GPIO_WriteLow(GPIOA,GPIO_PIN_3);		// CS	
#elif defined(STM8S105)
		//GPIO_WriteLow(GPIOE,GPIO_PIN_5);		// CS	
		//GPIO_WriteLow(GPIOC,GPIO_PIN_4);
		GPIO_WriteLow(GPIOF,GPIO_PIN_4);
#endif	
	}
}

uint8_t qmp6988_spi_write(uint8_t Addr, uint8_t Data)
{
	qst_spi_cs_write(0);
	delay_us(20);
	spi_write_byte(Addr&0x7f);		// write commond
	spi_write_byte(Data);
	delay_us(20);
	qst_spi_cs_write(1);;

	return 1;
}

uint8_t qmp6988_spi_read(uint8_t Addr, uint8_t *pData, uint16_t Length)
{
	uint8_t i = 0;

	qst_spi_cs_write(0);
	delay_us(20);
	spi_write_byte(Addr | 0x80);
	for(i =0; i < Length; i ++){
	  pData[i] = spi_write_byte(0x00);
	}  
	delay_us(20);
	qst_spi_cs_write(1);
	
	return 1;
}

#if defined(QST_CONFIG_FIS210X)
uint8_t qst_fis210x_spi_write(uint8_t Addr, uint8_t Data)
{
	qst_spi_cs_write(0);
	delay_us(20);
	spi_write_byte(Addr&0x3f);		// write commond
	spi_write_byte(Data);
	delay_us(20);
	qst_spi_cs_write(1);

	return 1;
}

uint8_t qst_fis210x_spi_read(uint8_t Addr, uint8_t *pData, uint16_t Length)
{
	uint8_t i = 0;

	qst_spi_cs_write(0);
	delay_us(20);

	if(Length == 1)
		spi_write_byte(Addr|0x80);
	else
		spi_write_byte(Addr|0xc0);
	for(i =0; i < Length; i++){
	  pData[i] = spi_write_byte(0x00);
	}  
	delay_us(20);
	qst_spi_cs_write(1);
	
	return 1;
}
#endif

#if defined(QST_CONFIG_QMAX981)
uint8_t qst_qmaX981_spi_write(uint8_t Addr, uint8_t Data)
{
	qst_spi_cs_write(0);	//SPI_CS_LOW;
	delay_us(10);
	spi_write_byte(0xae);	// IDW
	spi_write_byte(0x00);	//addr high
	spi_write_byte(Addr);	// addr low
	spi_write_byte(0x00);	// len high
	spi_write_byte(0x01);	// len low
	spi_write_byte(Data);	// data
	delay_us(10);
	qst_spi_cs_write(1);	//SPI_CS_HIGH;

	return 1;
}

uint8_t qst_qmaX981_spi_read(uint8_t Addr, uint8_t* Buf, uint8_t len)
{
	uint8_t index;

	qst_spi_cs_write(0);	//SPI_CS_LOW;
	delay_us(10);
	spi_write_byte(0xaf);	// IDR
	spi_write_byte(0x00);	// addr high
	spi_write_byte(Addr);	// addr low	
	
	spi_write_byte(0x00);	// len high
	spi_write_byte(len);	// len low
	//r_data = Spi_SendByte(0xff);
	for(index=0;index<len;index++)
	{	
		//buff[r_data] = SPI_I2S_ReceiveData(QST_SPIx);
		Buf[index] = spi_write_byte(0x00);
	}
	delay_us(10);
	qst_spi_cs_write(1);	//SPI_CS_HIGH;

	return 1;
}

#endif

#endif

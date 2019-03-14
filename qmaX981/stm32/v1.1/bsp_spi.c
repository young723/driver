 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   spi QST 底层应用函数bsp 
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./spi/bsp_spi.h"

//#define USE_SW_SPI
static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;    
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);
void spi_sw_Init(void);
void spi_struct_config(int mode);

/**
  * @brief  SPI_QST初始化
  * @param  无
  * @retval 无
  */
void Spi_Init(void)
{
#if defined(USE_SW_SPI)
	spi_sw_Init();
#else
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能SPI时钟 */
	RCC_APB2PeriphClockCmd(QST_SPI_CLK, ENABLE );

	/* 使能SPI引脚相关的时钟 */
	RCC_APB2PeriphClockCmd( QST_SPI_CS_CLK|QST_SPI_SCK_CLK|QST_SPI_MISO_PIN|QST_SPI_MOSI_PIN, ENABLE );

	/* 配置SPI的 CS引脚，普通IO即可 PA4*/
	GPIO_InitStructure.GPIO_Pin = QST_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(QST_SPI_CS_PORT, &GPIO_InitStructure);

	/* 配置SPI的 SCK引脚 PA5 */
	GPIO_InitStructure.GPIO_Pin = QST_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF_PP;
	GPIO_Init(QST_SPI_SCK_PORT, &GPIO_InitStructure);

	/* 配置SPI的 MISO引脚 PA6*/
	GPIO_InitStructure.GPIO_Pin = QST_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
	GPIO_Init(QST_SPI_MISO_PORT, &GPIO_InitStructure);

	/* 配置SPI的 MOSI引脚 PA7*/
	GPIO_InitStructure.GPIO_Pin = QST_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;	//GPIO_Mode_AF_PP;	//GPIO_Mode_Out_PP;
	GPIO_Init(QST_SPI_MOSI_PORT, &GPIO_InitStructure);

	/* 停止信号 QST: CS引脚高电平*/
	SPI_QST_CS_HIGH();

	/* SPI 模式配置 */
	// QST芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
/*
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(QST_SPIx , &SPI_InitStructure);
*/
	spi_struct_config(0);

	/* 使能 SPI  */
	SPI_Cmd(QST_SPIx , ENABLE);
#endif
}

void spi_struct_config(int mode)
{
	SPI_InitTypeDef  SPI_InitStructure;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	if(mode == 0)
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	}
	else if(mode == 1)
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	}	
	else if(mode == 3)
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	}
	else
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	}
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(QST_SPIx , &SPI_InitStructure);
}


static void spi_delay(uint32_t nCount)
{
	u16 i=0;
	while(nCount--)
	{
	  i=10;
	  while(i--);
	}
}



 /**
  * @brief  使用SPI发送一个字节的数据
  * @param  byte：要发送的数据
  * @retval 返回接收到的数据
  */
u8 Spi_SendByte(u8 byte)
{
  SPITimeout = SPIT_FLAG_TIMEOUT;
  /* 等待发送缓冲区为空，TXE事件 */
  while (SPI_I2S_GetFlagStatus(QST_SPIx , SPI_I2S_FLAG_TXE) == RESET)
	{
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
   }

  /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
  SPI_I2S_SendData(QST_SPIx , byte);

	SPITimeout = SPIT_FLAG_TIMEOUT;
  /* 等待接收缓冲区非空，RXNE事件 */
  while (SPI_I2S_GetFlagStatus(QST_SPIx , SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
   }

  /* 读取数据寄存器，获取接收缓冲区数据 */
  return SPI_I2S_ReceiveData(QST_SPIx );
}

static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* 等待超时后的处理,输出错误信息 */
  console_write("SPI 等待超时!errorCode = %d",errorCode);
  return 0;
}


#if defined(USE_SW_SPI)

#define SPI_CS_HIGH				GPIO_SetBits(GPIOA,GPIO_Pin_4) // CS----PB9
#define SPI_CS_LOW				GPIO_ResetBits(GPIOA,GPIO_Pin_4)

#define SPI_SCK_HIGH			GPIO_SetBits(GPIOA,GPIO_Pin_5) //SCLK----PA5
#define SPI_SCK_LOW				GPIO_ResetBits(GPIOA,GPIO_Pin_5)

#define SPI_MOSI_HIGH			GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define SPI_MOSI_LOW			GPIO_ResetBits(GPIOA,GPIO_Pin_7) //MOSI----PA7

#define SPI_MISO_DATA			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) //MISO----PA6

uint8_t spi_sw_write_data(uint8_t data)
{
	uint8_t RecevieData=0,i;

	for(i=0;i<8;i++)
	{

		if(data&0x80)
		{
			SPI_MOSI_HIGH;
		}
		else
		{
			SPI_MOSI_LOW;
		}
		SPI_SCK_HIGH; //我这里是选择下降沿采集数据
		//SPI_SCK_LOW;

		data<<=1;

		//__NOP();//__NOP();__NOP();__NOP();
		spi_delay(2);

		RecevieData <<= 1;
		if(SPI_MISO_DATA)
		{
			RecevieData |= 1; //Wait SDO to go Hi
		}
		//SPI_SCK_HIGH;
		SPI_SCK_LOW;
	}

	return RecevieData;
}


void spi_sw_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7; //SPI_CS and SPI_SCK and SPI_OUT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //SPI_IN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //SPI_CS and SPI_SCK and SPI_OUT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	SPI_CS_HIGH;
	SPI_SCK_LOW;
	SPI_MOSI_LOW;
}
#endif


u8 qmaX981_spi_write(u8 addr,u8 data)
{
#if defined(USE_SW_SPI)
	SPI_CS_LOW;
	spi_delay(1);

	spi_sw_write_data(0xae);	// IDW
	spi_sw_write_data(0x00);	//addr high
	spi_sw_write_data(addr);	// addr low
	spi_sw_write_data(0x00);	// len high
	spi_sw_write_data(0x01);	// len low
	spi_sw_write_data(data);	// data

	spi_delay(1);
	SPI_CS_HIGH;
#else
	SPI_QST_CS_LOW();//SPI_CS_LOW;
	spi_delay(10);
	Spi_SendByte(0xae);	// IDW
	Spi_SendByte(0x00);	//addr high
	Spi_SendByte(addr);	// addr low
	Spi_SendByte(0x00);	// len high
	Spi_SendByte(0x01);	// len low
	Spi_SendByte(data);	// data
	spi_delay(10);
	SPI_QST_CS_HIGH();
#endif

	return 1;
}


u8 qmaX981_spi_read(u8 addr, u8* buff, u8 len)
{
	u8 r_data;

#if defined(USE_SW_SPI)
	SPI_CS_LOW;//SPI_CS_LOW;
	spi_delay(1);
	spi_sw_write_data(0xaf); // IDR
	spi_sw_write_data(0x00); // addr high
	spi_sw_write_data(addr); // addr low 

	spi_sw_write_data(0x00); // len high
	spi_sw_write_data(len);	// len low
	//r_data = Spi_SendByte(0xff);
	for(r_data=0;r_data<len;r_data++)
	{	
		//buff[r_data] = SPI_I2S_ReceiveData(QST_SPIx);
		buff[r_data] = spi_sw_write_data(0x00);
	}
	spi_delay(1);
	SPI_CS_HIGH;
#else
	SPI_QST_CS_LOW();//SPI_CS_LOW;
	spi_delay(10);
	Spi_SendByte(0xaf);	// IDR
	Spi_SendByte(0x00);	// addr high
	Spi_SendByte(addr);	// addr low	
	
	Spi_SendByte(0x00);	// len high
	Spi_SendByte(len);	// len low
	//r_data = Spi_SendByte(0xff);
	for(r_data=0;r_data<len;r_data++)
	{	
		//buff[r_data] = SPI_I2S_ReceiveData(QST_SPIx);
		buff[r_data] = Spi_SendByte(0x00);
	}
	spi_delay(10);
	SPI_QST_CS_HIGH();
#endif
	return 1;
}


unsigned char bmi160_spi_write(unsigned char reg_addr, unsigned char reg_data)
{
#if defined(USE_SW_SPI)
	SPI_CS_LOW;
	spi_delay(1);
	spi_sw_write_data((0x7F&reg_addr));
	spi_sw_write_data(reg_data);
	spi_delay(1);
	SPI_CS_HIGH;
#else
	SPI_QST_CS_LOW();//SPI_CS_LOW;
	spi_delay(20);
	Spi_SendByte((0x7F&reg_addr));
	Spi_SendByte(reg_data);
	spi_delay(20);
	SPI_QST_CS_HIGH();
#endif
}

unsigned char bmi160_spi_read(unsigned char reg_addr, unsigned char* buff, unsigned char len)
{
	u8 r_data;

#if defined(USE_SW_SPI)
	SPI_CS_LOW;//SPI_CS_LOW;
	spi_delay(1);
	spi_sw_write_data((0x80|reg_addr));	// len low
	for(r_data=0;r_data<len;r_data++)
	{
		buff[r_data] = spi_sw_write_data(0x00);
	}	
	spi_delay(1);
	SPI_CS_HIGH;
#else
	SPI_QST_CS_LOW();//SPI_CS_LOW;
	spi_delay(10);
	Spi_SendByte((0x80|reg_addr));
	for(r_data=0;r_data<len;r_data++)
	{	
		//buff[r_data] = SPI_I2S_ReceiveData(QST_SPIx);
		buff[r_data] = Spi_SendByte(0xff);
	}
	spi_delay(10);
	SPI_QST_CS_HIGH();
#endif
}



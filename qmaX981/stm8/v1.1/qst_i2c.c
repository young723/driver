
#include "stm8s.h"
#include "qst_i2c.h"
#include "delay.h"

#if defined(QST_SW_IIC)
#define GPIO_PORT_I2C		GPIOB
#define I2C_SCL_PIN			GPIO_PIN_4			/* SCL GPIO */
#define I2C_SDA_PIN			GPIO_PIN_5			/* SDA GPIO */

#define I2C_SCL_OUTPUT()	GPIO_Init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUT_PP_HIGH_FAST)	
#define I2C_SCL_INPUT()			
#define I2C_SDA_OUTPUT()	GPIO_Init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUT_OD_HIZ_FAST)		
#define I2C_SDA_INPUT()		GPIO_Init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_IN_PU_NO_IT)	
#define I2C_SCL_1()  		GPIO_WriteHigh(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 1 */
#define I2C_SCL_0()  		GPIO_WriteLow(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
#define I2C_SDA_1()  		GPIO_WriteHigh(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 1 */
#define I2C_SDA_0()  		GPIO_WriteLow(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
#define I2C_SDA_READ()  	GPIO_ReadInputPin(GPIO_PORT_I2C, I2C_SDA_PIN)	/* Read SDA */


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
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
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
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

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
		//i2c_Delay();
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
uint8_t i2c_ReadByte(u8 ack)
{
	uint8_t i;
	uint8_t value;

	/* ???1?bit????bit7 */
	I2C_SDA_INPUT();	// set data input	
	i2c_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		//I2C_SCL_1();
		//i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_1();
		i2c_Delay();
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
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

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

#endif

#if defined(QST_SW_IIC_MTK)

#define GPIO_PORT_I2C		GPIOB
#define I2C_SCL_PIN			GPIO_PIN_4			/* SCL GPIO */
#define I2C_SDA_PIN			GPIO_PIN_5			/* SDA GPIO */

#define MS_CLK_PIN_GPIO_MODE
#define	MS_DATA_PIN_GPIO_MODE
#define MS_I2C_CLK_OUTPUT			GPIO_Init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUT_PP_HIGH_FAST)
#define MS_I2C_DATA_OUTPUT			GPIO_Init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUT_OD_HIZ_FAST)
#define MS_I2C_DATA_INPUT		   	GPIO_Init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_IN_PU_NO_IT)
#define MS_I2C_CLK_HIGH				GPIO_WriteHigh(GPIO_PORT_I2C, I2C_SCL_PIN)
#define MS_I2C_CLK_LOW				GPIO_WriteLow(GPIO_PORT_I2C, I2C_SCL_PIN)
#define MS_I2C_DATA_HIGH			GPIO_WriteHigh(GPIO_PORT_I2C, I2C_SDA_PIN)
#define MS_I2C_DATA_LOW				GPIO_WriteLow(GPIO_PORT_I2C, I2C_SDA_PIN)
#define MS_I2C_GET_BIT				GPIO_ReadInputPin(GPIO_PORT_I2C, I2C_SDA_PIN)


//#define SW_i2c_udelay		delay_us

static void SW_i2c_udelay(uint16_t us)
{
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

}

static void SW_i2c_start(void)
{
	MS_CLK_PIN_GPIO_MODE;
	MS_I2C_CLK_OUTPUT;

	MS_DATA_PIN_GPIO_MODE;
	MS_I2C_DATA_OUTPUT;
	
	MS_I2C_DATA_HIGH;
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(40);		//20
	MS_I2C_DATA_LOW;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_LOW;
	SW_i2c_udelay(20);		//10
}

/******************************************
	software I2C stop bit
*******************************************/
static void SW_i2c_stop(void)
{
	MS_I2C_CLK_OUTPUT;
	MS_I2C_DATA_OUTPUT;
	
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(20);		//10
	MS_I2C_DATA_HIGH;
}

/******************************************
	software I2C one clock
*******************************************/
static void SW_i2c_one_clk(void)
{
	SW_i2c_udelay(10);		//5
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_LOW;
	SW_i2c_udelay(10);		//5
}

/******************************************
	software I2C read byte with ack
*******************************************/
static uint8_t ms_ReadByteAck(void)
{
	int8_t i;
	uint8_t data;

	MS_I2C_DATA_INPUT; 
	data = 0; 
	
	for (i=7; i>=0; i--) 
	{
		if (MS_I2C_GET_BIT)
		{
			data |= (0x01<<i);
		}
		SW_i2c_one_clk();
	}			                                

	MS_I2C_DATA_OUTPUT;                    
	MS_I2C_DATA_LOW;                       
	SW_i2c_one_clk();                         

	return data;
}

/******************************************
	software I2C read byte without ack
*******************************************/
static uint8_t ms_ReadByteNAck(void)
{
	int8_t i;
	uint8_t data;

	MS_I2C_DATA_INPUT; 
	data = 0; 
	
	for (i=7; i>=0; i--) 
	{
		if (MS_I2C_GET_BIT)
		{
			data |= (0x01<<i);
		}
		SW_i2c_one_clk();
	}			                                

	MS_I2C_DATA_OUTPUT;                                           
	MS_I2C_DATA_HIGH;
	SW_i2c_one_clk();                         
	
	return data;
}

/******************************************
	software I2C send byte
*******************************************/
static void ms_SendByte(uint8_t sData) 
{
	int8_t i;
	MS_I2C_DATA_OUTPUT;       
	for (i=7; i>=0; i--) 
	{            
		if ((sData>>i)&0x01) 
		{               
			MS_I2C_DATA_HIGH;	              
		} 
		else 
		{ 
			MS_I2C_DATA_LOW;                  
		}
		SW_i2c_one_clk();                        
	}
	MS_I2C_DATA_HIGH;
}
/******************************************
	software I2C check ack bit
*******************************************/
static uint8_t ms_ChkAck(void)//
{
  	MS_I2C_DATA_HIGH;
	MS_I2C_DATA_INPUT;
	SW_i2c_udelay(10);		//5
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(10);		//5

	if(MS_I2C_GET_BIT)		//Non-ack
	{
		SW_i2c_udelay(10);	//5
		MS_I2C_CLK_LOW;
		SW_i2c_udelay(10);	//5
		MS_I2C_DATA_OUTPUT;
		MS_I2C_DATA_LOW;
		
		return 0;
	}
	else					//Ack
	{
		SW_i2c_udelay(10);	//5
		MS_I2C_CLK_LOW;
		SW_i2c_udelay(10);	//5
		MS_I2C_DATA_OUTPUT;
		MS_I2C_DATA_LOW;

		return 1;
	}
}

/******************************************
	software I2C restart bit
*******************************************/
static void ms_Restart(void)
{
	MS_I2C_CLK_OUTPUT;
	MS_I2C_DATA_OUTPUT;

	SW_i2c_udelay(40);
	MS_I2C_DATA_HIGH;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(40);
	MS_I2C_DATA_LOW;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_LOW;
	SW_i2c_udelay(20);		//10
}

#endif

static uint32_t retry_count = 0;
#define MAX_RETRY_COUNT		30000
uint8_t qst_iic_write(uint8_t slave,uint8_t Addr, uint8_t Data)
{
#if defined(QST_SW_IIC)
	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(Addr);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(Data);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_Stop();

	return 1;
#elif defined(QST_SW_IIC_MTK)
	SW_i2c_start(); 					//start bit
	ms_SendByte(slave);		//slave address|write bit
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send write id
		SW_i2c_stop();
		return 0;
	}
	ms_SendByte(Addr);				//send RegAddr
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send RegAddr
		SW_i2c_stop();
		return 0;
	}
	ms_SendByte(Data);					//send parameter
	if(0 == ms_ChkAck())
	{
		//TO_DO: display check ack fail when send data
		SW_i2c_stop();
		return 0;
	}
	SW_i2c_stop();						//stop bit

	return 1;
#else
  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
  {
  	if(retry_count++ > MAX_RETRY_COUNT)
		return 0;
  }
  /* 1.开始 */
  I2C_GenerateSTART(ENABLE);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)) //while(!(I2C->SR1&0x01));
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 2.设备地址/写 */
  I2C_Send7bitAddress(slave, I2C_DIRECTION_TX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) //while(!((I2C->SR1)&0x02));
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 3.数据地址 */
  I2C_SendData((Addr&0xFF));
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) //while(!(I2C->SR1 & 0x04));  
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 4.写一字节数据 */
  I2C_SendData(Data);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) //while(!(I2C->SR1 & 0x04));  
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 5.停止 */
  I2C_GenerateSTOP(ENABLE);

  return 1;
#endif
}

/************************************************
函数名称 ： EEPROM_WriteNByte
功    能 ： EEPROM写N字节
参    数 ： Addr ----- 地址
            pData ---- 数据
            Length --- 长度
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
uint8_t qst_iic_read(uint8_t slave, uint8_t Addr, uint8_t *pData, uint16_t Length)
{
#if defined(QST_SW_IIC)
	uint8_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(Addr);
	if(i2c_WaitAck())
	{
		return 0;
	}

	i2c_Start();
	i2c_SendByte(slave+1);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(Length-1);i++){
		*pData=i2c_ReadByte(1);
		pData++;
	}
	*pData=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
#elif defined(QST_SW_IIC_MTK)
	uint8_t* Data_ptr;
	uint16_t i;

	Data_ptr = pData;

	SW_i2c_start(); 					//start bit
	ms_SendByte(slave);		//slave address|write bit
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send write id		
		SW_i2c_stop();
		return 0;
	}
		
	ms_SendByte(Addr);				//send RegAddr
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send RegAddr		
		SW_i2c_stop();
		return 0;
	}

	ms_Restart();						//restart bit
	ms_SendByte(slave+1);		//slave address|read bit
	if(0 == ms_ChkAck())
	{
		//TO_DO: display ack check fail when send read id		
		SW_i2c_stop();
		return 0;
	}
	for(i=Length; i>1; i--)
	{
		*Data_ptr = ms_ReadByteAck();	//read byte with ack
		Data_ptr++;
	}
	*Data_ptr = ms_ReadByteNAck();		//read byte with non-ack to stop reading
	SW_i2c_stop();						//stop bit
	//TO_DO: add debug code to display the data received

	return 1;
#else
  uint16_t cnt;

  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 1.开始 */
  I2C_GenerateSTART(ENABLE);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 2.设备地址/写 */
  I2C_Send7bitAddress(slave, I2C_DIRECTION_TX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 3.数据地址 */
  I2C_SendData((Addr&0xFF));
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 4.重新开始 */
  I2C_GenerateSTART(ENABLE);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 5.设备地址/读 */
  I2C_Send7bitAddress(slave, I2C_DIRECTION_RX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 6.读多字节数据 */
  for(cnt=0; cnt<(Length-1); cnt++)
  {
    I2C_AcknowledgeConfig(I2C_ACK_CURR);                             //产生应答
	retry_count = 0;
    while(I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
	{
		if(retry_count++ > MAX_RETRY_COUNT)
			return 0;
	}
    *pData = I2C_ReceiveData();                                      //连续读取(Length-1)字节
    pData++;
  }
  I2C_AcknowledgeConfig(I2C_ACK_NONE);                               //读取最后1字节(产生非应答)
  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  *pData = I2C_ReceiveData();                                        //读取数据
  /* 7.停止 */
  I2C_GenerateSTOP(ENABLE);

  return 1;
#endif
}


uint8_t qst_iic_read_2(uint8_t slave, uint8_t Addr, uint8_t *pData, uint16_t Length)
{
#if defined(QST_SW_IIC)
	uint8_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(Addr);
	if(i2c_WaitAck())
	{
		return 0;
	}

	i2c_Start();
	i2c_SendByte(slave+1);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(Length-1);i++){
		*pData=i2c_ReadByte(1);
		pData++;
	}
	*pData=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
#elif defined(QST_SW_IIC_MTK)
	uint8_t* Data_ptr;
	uint16_t i;

	Data_ptr = pData;

	SW_i2c_start(); 					//start bit
	ms_SendByte(slave);		//slave address|write bit
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send write id		
		SW_i2c_stop();
		return 0;
	}
		
	ms_SendByte(Addr);				//send RegAddr
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send RegAddr		
		SW_i2c_stop();
		return 0;
	}

	//ms_Restart();						//restart bit
	SW_i2c_start();
	ms_SendByte(slave+1);		//slave address|read bit
	if(0 == ms_ChkAck())
	{
		//TO_DO: display ack check fail when send read id		
		SW_i2c_stop();
		return 0;
	}
	for(i=Length; i>1; i--)
	{
		*Data_ptr = ms_ReadByteAck();	//read byte with ack
		Data_ptr++;
	}
	*Data_ptr = ms_ReadByteNAck();		//read byte with non-ack to stop reading
	SW_i2c_stop();						//stop bit
	//TO_DO: add debug code to display the data received

	return 1;
#else
  uint16_t cnt;

  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 1.开始 */
  I2C_GenerateSTART(ENABLE);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 2.设备地址/写 */
  I2C_Send7bitAddress(slave, I2C_DIRECTION_TX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 3.数据地址 */
  I2C_SendData((Addr&0xFF));
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 4.重新开始 */
  I2C_GenerateSTART(ENABLE);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 5.设备地址/读 */
  I2C_Send7bitAddress(slave, I2C_DIRECTION_RX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 6.读多字节数据 */
  for(cnt=0; cnt<(Length-1); cnt++)
  {
    I2C_AcknowledgeConfig(I2C_ACK_CURR);                             //产生应答
	retry_count = 0;
    while(I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
	{
		if(retry_count++ > MAX_RETRY_COUNT)
			return 0;
	}
    *pData = I2C_ReceiveData();                                      //连续读取(Length-1)字节
    pData++;
  }
  I2C_AcknowledgeConfig(I2C_ACK_NONE);                               //读取最后1字节(产生非应答)
  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  *pData = I2C_ReceiveData();                                        //读取数据
  /* 7.停止 */
  I2C_GenerateSTOP(ENABLE);

  return 1;
#endif
}



#if defined(QST_CONFIG_JHM1200)
uint8_t jhm1200_iic_write(uint8_t Addr, uint8_t* Buff, uint8_t Len)
{
#if defined(QST_SW_IIC)
	uint8_t i;

	i2c_Start();
	i2c_SendByte(0xf0);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(Addr);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	for(i=0;i<Len;i++)
	{
		i2c_SendByte(Buff[i]);	
		if(i2c_WaitAck())
		{
			return 0;
		}
	}
	i2c_Stop();

	return 1;
#elif defined(QST_SW_IIC_MTK)
	SW_i2c_start(); 					//start bit
	ms_SendByte(0xf0);		//slave address|write bit
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send write id
		SW_i2c_stop();
		return 0;
	}
	ms_SendByte(Addr);				//send RegAddr
	if(0 == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send RegAddr
		SW_i2c_stop();
		return 0;
	}
	
	for(i=0;i<Len;i++)
	{
		ms_SendByte(Buff[i]);					//send parameter
		if(0 == ms_ChkAck())
		{
			//TO_DO: display check ack fail when send data
			SW_i2c_stop();
			return 0;
		}
	}
	SW_i2c_stop();						//stop bit

	return 1;
#else
  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
  {
  	if(retry_count++ > MAX_RETRY_COUNT)
		return 0;
  }
  /* 1.开始 */
  I2C_GenerateSTART(ENABLE);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)) //while(!(I2C->SR1&0x01));
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 2.设备地址/写 */
  I2C_Send7bitAddress(0xf0, I2C_DIRECTION_TX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) //while(!((I2C->SR1)&0x02));
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 3.数据地址 */
  I2C_SendData((Addr&0xFF));
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) //while(!(I2C->SR1 & 0x04));  
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 4.写一字节数据 */
  for(i=0;i<Len;i++)
  {
	  I2C_SendData(Buff[i]);
	  retry_count = 0;
	  while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) //while(!(I2C->SR1 & 0x04));  
	  {
		  if(retry_count++ > MAX_RETRY_COUNT)
			  return 0;
	  }
  }
  /* 5.停止 */
  I2C_GenerateSTOP(ENABLE);

  return 1;
#endif
}


uint8_t jhm1200_iic_read(uint8_t *pData, uint16_t Length)
{
#if defined(QST_SW_IIC)
	uint8_t i;

	i2c_Start();
	i2c_SendByte(0xf1);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(Length-1);i++){
		*pData=i2c_ReadByte(1);
		pData++;
	}
	*pData=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
#elif defined(QST_SW_IIC_MTK)
	uint8_t* Data_ptr;
	uint16_t i;

	Data_ptr = pData;

	ms_Restart();						//restart bit
	ms_SendByte(0xf1);		//slave address|read bit
	if(0 == ms_ChkAck())
	{
		//TO_DO: display ack check fail when send read id		
		SW_i2c_stop();
		return 0;
	}
	for(i=Length; i>1; i--)
	{
		*Data_ptr = ms_ReadByteAck();	//read byte with ack
		Data_ptr++;
	}
	*Data_ptr = ms_ReadByteNAck();		//read byte with non-ack to stop reading
	SW_i2c_stop();						//stop bit
	//TO_DO: add debug code to display the data received

	return 1;
#else
  uint16_t cnt;

  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 1.开始 */
  I2C_GenerateSTART(ENABLE);
#if 0
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 2.设备地址/写 */
  I2C_Send7bitAddress(slave, I2C_DIRECTION_TX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 3.数据地址 */
  I2C_SendData((Addr&0xFF));
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 4.重新开始 */
  I2C_GenerateSTART(ENABLE);
#endif
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 5.设备地址/读 */
  I2C_Send7bitAddress(0xf0, I2C_DIRECTION_RX);
  retry_count = 0;
  while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  /* 6.读多字节数据 */
  for(cnt=0; cnt<(Length-1); cnt++)
  {
    I2C_AcknowledgeConfig(I2C_ACK_CURR);                             //产生应答
	retry_count = 0;
    while(I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
	{
		if(retry_count++ > MAX_RETRY_COUNT)
			return 0;
	}
    *pData = I2C_ReceiveData();                                      //连续读取(Length-1)字节
    pData++;
  }
  I2C_AcknowledgeConfig(I2C_ACK_NONE);                               //读取最后1字节(产生非应答)
  retry_count = 0;
  while(I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
  {
	  if(retry_count++ > MAX_RETRY_COUNT)
		  return 0;
  }
  *pData = I2C_ReceiveData();                                        //读取数据
  /* 7.停止 */
  I2C_GenerateSTOP(ENABLE);

  return 1;
#endif
}


#endif

/**** Copyright (C)2017 strongerHuang. All Rights Reserved **** END OF FILE ****/

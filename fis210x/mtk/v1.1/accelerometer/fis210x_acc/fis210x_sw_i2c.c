
#include "fis210x_sw_i2c.h"

#if defined(MTK_SW_IIC)
#define GPIO_SDA 	GPIO49
#define GPIO_SCL	GPIO45

#define SW_IIC_CLK_MODE_GPIO					mt_set_gpio_mode(GPIO_SCL, GPIO_MODE_GPIO)
#define SW_IIC_SDA_MODE_GPIO					mt_set_gpio_mode(GPIO_SDA, GPIO_MODE_GPIO)

#define SW_IIC_CLK_OUTPUT						mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT)
#define SW_IIC_CLK_INPUT						mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_IN)
#define SW_IIC_DATA_OUTPUT						mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT)
#define SW_IIC_DATA_INPUT						mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_IN)

#define SW_IIC_CLK_HIGH							mt_set_gpio_out(GPIO_SCL, GPIO_OUT_ONE)
#define SW_IIC_CLK_LOW							mt_set_gpio_out(GPIO_SCL, GPIO_OUT_ZERO)
#define SW_IIC_DATA_HIGH						mt_set_gpio_out(GPIO_SDA, GPIO_OUT_ONE)
#define SW_IIC_DATA_LOW							mt_set_gpio_out(GPIO_SDA, GPIO_OUT_ZERO)

#define GET_SW_IIC_DATA_BIT						mt_get_gpio_in(GPIO_SDA)

#define I2C_DELAY								2

static int i2c_delay(unsigned int n)
{
#if 1
    udelay(n);
#else
	unsigned int count = 1024*n;
	asm volatile(
			"1:                                 \n\t"
			"subs   %[count], %[count], #1      \n\t"
			"bge    1b                          \n\t"
			:[count] "+r" (count)
			:
			:"memory"
			);
#endif
	return 0;
}


#define I2C_START_TRANSMISSION \
{ \
	/*volatile unsigned char j;*/ \
	SW_IIC_CLK_OUTPUT; \
	SW_IIC_DATA_OUTPUT; \
	SW_IIC_CLK_HIGH; \
	SW_IIC_DATA_HIGH; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY); 	\
	SW_IIC_DATA_LOW; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY);	\
	SW_IIC_CLK_LOW; \
}

#define I2C_STOP_TRANSMISSION \
{ \
	/*volatile unsigned char j;*/ \
	SW_IIC_CLK_OUTPUT; \
	SW_IIC_DATA_OUTPUT; \
	SW_IIC_CLK_LOW; \
	SW_IIC_DATA_LOW; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY);	\
	SW_IIC_CLK_HIGH; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY);	\
	SW_IIC_DATA_HIGH; \
}					

static void sw_iic_send_byte(unsigned char send_byte)
{
	volatile signed char i;
	//volatile unsigned int j;

	for (i=7;i>=0;i--)
	{	/* data bit 7~0 */
		if (send_byte & (1<<i))
		{
			SW_IIC_DATA_HIGH;
		}
		else
		{
			SW_IIC_DATA_LOW;
		}
		i2c_delay(I2C_DELAY);
		SW_IIC_CLK_HIGH;
		i2c_delay(I2C_DELAY);
		SW_IIC_CLK_LOW;
		i2c_delay(I2C_DELAY);
	}
	/* don't care bit, 9th bit */
	SW_IIC_DATA_LOW;
	SW_IIC_DATA_INPUT;
	SW_IIC_CLK_HIGH;
	i2c_delay(I2C_DELAY);
	SW_IIC_CLK_LOW;
	SW_IIC_DATA_OUTPUT;
}	/* sw_iic_send_byte() */

static unsigned char sw_iic_get_byte(void)
{
	volatile signed char i;
	//volatile unsigned char j;
	unsigned char get_byte=0;

	SW_IIC_DATA_INPUT;

	for (i=7;i>=0;i--)
	{	/* data bit 7~0 */
		SW_IIC_CLK_HIGH;
		//for(j=0;j<I2C_DELAY;j++);
		i2c_delay(I2C_DELAY);
		if (GET_SW_IIC_DATA_BIT)
			get_byte |= (1<<i);
		//for(j=0;j<I2C_DELAY;j++);
		i2c_delay(I2C_DELAY);
		SW_IIC_CLK_LOW;
		//for(j=0;j<I2C_DELAY;j++);
		i2c_delay(I2C_DELAY);
	}
	/* don't care bit, 9th bit */
	SW_IIC_DATA_OUTPUT;
	SW_IIC_DATA_HIGH;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SW_IIC_CLK_HIGH;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SW_IIC_CLK_LOW;

	return get_byte;
}	/* sw_iic_send_byte() */

unsigned char sw_iic_write(unsigned char slave_addr, unsigned char addr, unsigned char para)
{
	//volatile unsigned int i,j;
	mt_set_gpio_mode(GPIO_SDA, GPIO_MODE_GPIO);
	mt_set_gpio_mode(GPIO_SCL, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT);
	mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT);

	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(slave_addr);

	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(addr);

	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(para);

	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	i2c_delay(I2C_DELAY);
	i2c_delay(I2C_DELAY);
	I2C_STOP_TRANSMISSION;

	return 0;
}
EXPORT_SYMBOL(sw_iic_write);


unsigned char ssw_iic_write_multi(unsigned char slave_addr, unsigned char *addr, unsigned char nb_char)
{
	volatile unsigned int i;

	mt_set_gpio_mode(GPIO_SDA, GPIO_MODE_GPIO);
	mt_set_gpio_mode(GPIO_SCL, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT);
	mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT);

	I2C_START_TRANSMISSION;
	
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(slave_addr);

	for(i=0;i<nb_char;i++)
	{
		i2c_delay(I2C_DELAY);
		sw_iic_send_byte(addr[i]);
	}

	I2C_STOP_TRANSMISSION;

	return 0;
}
EXPORT_SYMBOL(ssw_iic_write_multi);


unsigned char sw_iic_read(unsigned char slave_addr, unsigned char addr, unsigned char *buf)
{
	unsigned int get_byte;
	//volatile unsigned int i, j;

	mt_set_gpio_mode(GPIO_SDA, GPIO_MODE_GPIO);
	mt_set_gpio_mode(GPIO_SCL, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT);
	mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT);

	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(slave_addr);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(addr);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_STOP_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(slave_addr|0x1);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	get_byte=sw_iic_get_byte();
	*buf = get_byte;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_STOP_TRANSMISSION;

	return 0;
}
EXPORT_SYMBOL(sw_iic_read);


unsigned char sw_iic_read_multi(unsigned char slave_addr, unsigned char addr, unsigned char *buf, unsigned char nb_char)
{	
	unsigned int get_byte;
	volatile unsigned int i;

	mt_set_gpio_mode(GPIO_SDA, GPIO_MODE_GPIO);
	mt_set_gpio_mode(GPIO_SCL, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT);
	mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT);

	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(slave_addr);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(addr);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_STOP_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	sw_iic_send_byte(slave_addr|0x1);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	for(i=0; i< nb_char; i++)
	{
		get_byte=sw_iic_get_byte();
		buf[i] = get_byte;
		//for(j=0;j<I2C_DELAY;j++);
		i2c_delay(I2C_DELAY);
	}
	I2C_STOP_TRANSMISSION;

	return 0;
}
EXPORT_SYMBOL(sw_iic_read_multi);


#if 0
void init_i2c(void)
{
	mt_set_gpio_mode(GPIO_SDA, GPIO_MODE_00);
	mt_set_gpio_mode(GPIO_SCL, GPIO_MODE_00);

	mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT);
	mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT);
}

void suspend_i2c(void)
{
    mt_set_gpio_pull_enable(GPIO_SDA, true);
    mt_set_gpio_pull_select(GPIO_SDA, GPIO_PULL_UP);
    //mt_set_gpio_out(GPIO187, 0);
    //mt_set_gpio_pull_enable(GPIO187, true);
    //mt_set_gpio_pull_select(GPIO187, GPIO_PULL_UP);


    mt_set_gpio_pull_enable(GPIO_SCL, true);    
    mt_set_gpio_pull_select(GPIO_SCL, GPIO_PULL_UP);    
    return TM_OK;
}

void resume_i2c(void)
{
    mt_set_gpio_pull_enable(GPIO_SDA, false);

    mt_set_gpio_pull_enable(GPIO_SCL, false);    
    return TM_OK;
}
#endif

#endif


#if defined(QST_SW_IIC)
static spinlock_t		sw_i2c_lock;

#define I2C_SCL_PIN			GPIO45			/* SCL GPIO */
#define I2C_SDA_PIN			GPIO49			/* SDA GPIO */

#if 0
#define I2C_CLK_MODE_GPIO()		mt_set_gpio_mode(I2C_SCL_PIN, GPIO_MODE_GPIO)
#define I2C_SDA_MODE_GPIO()		mt_set_gpio_mode(I2C_SDA_PIN, GPIO_MODE_GPIO)

#define I2C_SCL_OUTPUT()	mt_set_gpio_dir(I2C_SCL_PIN, GPIO_DIR_OUT)	
#define I2C_SCL_INPUT()		mt_set_gpio_dir(I2C_SCL_PIN, GPIO_DIR_IN)	
#define I2C_SDA_OUTPUT()	mt_set_gpio_dir(I2C_SDA_PIN, GPIO_DIR_OUT)		
#define I2C_SDA_INPUT()		mt_set_gpio_dir(I2C_SDA_PIN, GPIO_DIR_IN)	
#define I2C_SCL_1()  		mt_set_gpio_out(I2C_SCL_PIN, GPIO_OUT_ONE)		/* SCL = 1 */
#define I2C_SCL_0()  		mt_set_gpio_out(I2C_SCL_PIN, GPIO_OUT_ZERO)		/* SCL = 0 */
#define I2C_SDA_1()  		mt_set_gpio_out(I2C_SDA_PIN, GPIO_OUT_ONE)		/* SDA = 1 */
#define I2C_SDA_0()  		mt_set_gpio_out(I2C_SDA_PIN, GPIO_OUT_ZERO)		/* SDA = 0 */
#define I2C_SDA_READ()  	mt_get_gpio_in(I2C_SDA_PIN)	/* Read SDA */
#else
#define I2C_CLK_MODE_GPIO()		mt_set_gpio_mode_base(I2C_SCL_PIN, GPIO_MODE_GPIO)
#define I2C_SDA_MODE_GPIO()		mt_set_gpio_mode_base(I2C_SDA_PIN, GPIO_MODE_GPIO)

#define I2C_SCL_OUTPUT()	mt_set_gpio_dir_base(I2C_SCL_PIN, GPIO_DIR_OUT)	
#define I2C_SCL_INPUT()		mt_set_gpio_dir_base(I2C_SCL_PIN, GPIO_DIR_IN)	
#define I2C_SDA_OUTPUT()	mt_set_gpio_dir_base(I2C_SDA_PIN, GPIO_DIR_OUT)		
#define I2C_SDA_INPUT()		mt_set_gpio_dir_base(I2C_SDA_PIN, GPIO_DIR_IN)	
#define I2C_SCL_1()  		mt_set_gpio_out_base(I2C_SCL_PIN, GPIO_OUT_ONE)		/* SCL = 1 */
#define I2C_SCL_0()  		mt_set_gpio_out_base(I2C_SCL_PIN, GPIO_OUT_ZERO)		/* SCL = 0 */
#define I2C_SDA_1()  		mt_set_gpio_out_base(I2C_SDA_PIN, GPIO_OUT_ONE)		/* SDA = 1 */
#define I2C_SDA_0()  		mt_set_gpio_out_base(I2C_SDA_PIN, GPIO_OUT_ZERO)		/* SDA = 0 */
#define I2C_SDA_READ()  	mt_get_gpio_in_base(I2C_SDA_PIN)	/* Read SDA */
#endif


void i2c_Ack(void);
void i2c_NAck(void);

/*
*********************************************************************************************************
*	? ? ?: i2c_sw_Delay
*	????: I2C?????,??400KHz
*	?    ?:?
*	? ? ?: ?
*********************************************************************************************************
*/
static void i2c_sw_Delay(void)
{
	//volatile unsigned int j;
	udelay(2);
	//for(j=0;j<20;j++);
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
	I2C_CLK_MODE_GPIO();
	I2C_SDA_MODE_GPIO();
	udelay(10);
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	udelay(10);

	I2C_SDA_1();
	I2C_SCL_1();
	i2c_sw_Delay();
	I2C_SDA_0();
	i2c_sw_Delay();
	I2C_SCL_0();
	i2c_sw_Delay();
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
	i2c_sw_Delay();
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
		//i2c_sw_Delay();
		I2C_SCL_1();
		i2c_sw_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // ????
		}
		_ucByte <<= 1;	/* ????bit */
		i2c_sw_Delay();
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
	i2c_sw_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		//I2C_SCL_1();
		//i2c_sw_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_1();
		i2c_sw_Delay();
		I2C_SCL_0();
		i2c_sw_Delay();
	}
	
	I2C_SDA_OUTPUT();	// set data output	
	i2c_sw_Delay();
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
	i2c_sw_Delay();
	I2C_SCL_1();	/* CPU??SCL = 1, ???????ACK?? */
	i2c_sw_Delay();
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
	i2c_sw_Delay();
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
	i2c_sw_Delay();
	I2C_SCL_1();	/* CPU??1??? */
	i2c_sw_Delay();
	I2C_SCL_0();
	i2c_sw_Delay();
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
	i2c_sw_Delay();
	I2C_SCL_1();	/* CPU??1??? */
	i2c_sw_Delay();
	I2C_SCL_0();
	i2c_sw_Delay();	
}

void qst_sw_iic_init(void)
{
	I2C_CLK_MODE_GPIO();
	I2C_SDA_MODE_GPIO();
	udelay(10);
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	udelay(10);

	spin_lock_init(&sw_i2c_lock);
}
EXPORT_SYMBOL(qst_sw_iic_init);


uint8_t qst_iic_write(uint8_t slave,uint8_t Addr, uint8_t Data)
{
	unsigned long flags;
	
	spin_lock_irqsave(&sw_i2c_lock, flags);

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		spin_unlock_irqrestore(&sw_i2c_lock, flags);
		return 1;
	}
	i2c_SendByte(Addr);	
	if(i2c_WaitAck())
	{
		spin_unlock_irqrestore(&sw_i2c_lock, flags);
		return 1;
	}
	i2c_SendByte(Data);	
	if(i2c_WaitAck())
	{
		spin_unlock_irqrestore(&sw_i2c_lock, flags);
		return 1;
	}
	i2c_Stop();
	spin_unlock_irqrestore(&sw_i2c_lock, flags);

	return 0;
}
EXPORT_SYMBOL(qst_iic_write);


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
	uint8_t i;
	unsigned long flags;
	
	spin_lock_irqsave(&sw_i2c_lock, flags);

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		spin_unlock_irqrestore(&sw_i2c_lock, flags);
		return 1;
	}
	i2c_SendByte(Addr);
	if(i2c_WaitAck())
	{
		spin_unlock_irqrestore(&sw_i2c_lock, flags);
		return 1;
	}

	i2c_Start();
	i2c_SendByte(slave+1);
	if(i2c_WaitAck())
	{
		spin_unlock_irqrestore(&sw_i2c_lock, flags);
		return 1;
	}

	for(i=0;i<(Length-1);i++){
		*pData=i2c_ReadByte(1);
		pData++;
	}
	*pData=i2c_ReadByte(0);
	i2c_Stop();
	spin_unlock_irqrestore(&sw_i2c_lock, flags);

	return 0;
}
EXPORT_SYMBOL(qst_iic_read);

#endif


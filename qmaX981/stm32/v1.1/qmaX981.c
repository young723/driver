/**
  ******************************************************************************
  * @file    qma7981.c
  * @author  Yangzhiqiang@qst
  * @version V1.0
  * @date    2017-12-15
  * @brief    qma6981驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 指南者 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./qmaX981/qmaX981.h"
#include "./usart/bsp_usart.h"
#include "./i2c/bsp_i2c.h"
#include <stdbool.h>
#include <string.h>

#define QMAX981_LOG		printf
#define QMAX981_ERR		printf

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
extern void qmaX981_step_debounce_reset(void);
extern int qmaX981_step_debounce_int_work(int data, unsigned char irq_level);
extern int qmaX981_step_debounce_read_data(int result);
#endif
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
extern int qmaX981_check_abnormal_data(int data_in, int *data_out);
#endif

typedef enum
{	
	CHIP_TYPE_QMA6981 = 0,
	CHIP_TYPE_QMA7981,
	CHIP_TYPE_UNDEFINE,
	CHIP_TYPE_MAX
}qmaX981_type;

typedef struct
{
    s16 sign[3];
    u16 map[3];
}qst_convert;

typedef struct
{
	u8					chip_id;
	qmaX981_type		chip_type;
	s32					lsb_1g;
	u8					layout;
	qst_convert			cvt;
	u8					int_level;
}qmaX981_data;

static const qst_convert qst_map[] = 
{
    { { 1, 1, 1}, {0, 1, 2} },
    { {-1, 1, 1}, {1, 0, 2} },
    { {-1,-1, 1}, {0, 1, 2} },
    { { 1,-1, 1}, {1, 0, 2} },

    { {-1, 1, -1}, {0, 1, 2} },
    { { 1, 1, -1}, {1, 0, 2} },
    { { 1,-1, -1}, {0, 1, 2} },
    { {-1,-1, -1}, {1, 0, 2} }
};


static qmaX981_data g_qmaX981;

const unsigned char qma6981_init_tbl[][2] = 
{
#if defined(QMAX981_STEP_COUNTER)
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},
	{0x11, 0x80},
	{0x0f, QMAX981_RANGE_8G},
	{0x10, 0x2a},
	{0x12, 0x8f},
	{0x13, 0x10},
	{0x14, 0x14},
	{0x15, 0x10},	
	{0x16, 0x0c},
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	{0x19, 0x08},
#endif
	{0x32, 0x02},
	{0x27, QMA6981_OFFSET},
	{0x28, QMA6981_OFFSET},
	{0x29, QMA6981_OFFSET},
#else
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},
	{0x11, 0x80},
	//{0x36, 0xb6},
	{0xff, 5},
	//{0x36, 0x00},
	//{0x11, 0x80},
	{0x0f, QMAX981_RANGE_4G},
	{0x10, QMA6981_ODR_125HZ},
#endif
#if defined(QMAX981_FIFO_FUNC)
	{0x10, QMA6981_ODR_250HZ},
	{0x11, 0x8b},
	{0x3E, 0x40},
	{0x17, 0x20},
	#if defined(QMAX981_FIFO_USE_INT)
	{0x1a, 0x20},	// fifo int map to int1
	#endif
#endif
#if defined(QMAX981_TAP_FUNC)
	{0x10, 0x05},
	{0x11, 0x80},	// 0x85 {0x2a, 0x80},	
	{0x2b, 0x07},	//0x14	
	{0x16, 0x20},	
	{0x19, 0x20},
	//{0x1b, 0x20},
#endif
#if defined(QMAX981_INT_LATCH_MODE)
	{0x21, 0x01},
#endif

	{0xff, 1}
};

/*	
qma7981 odr setting
0x10<2:0>		ODR(Hz)				Time(ms)	|	RANGE 0x0f<3:0>
000				43.3125				23.088		|	0001	2g  		244ug/LSB
001				86.4453				11.568		|	0010	4g  		488ug/LSB
002				172.1763			5.808		|	0100	8g  		977ug/LSB
003				341.5300			2.928		|	1000	16g  	1.95mg/LSB
004				672.0430			1.488		|	1111	32g  	3.91mg/LSB
005				32.5013				30.768		|	Others	2g  		244ug/LSB
006				129.3995			7.728		|
007				257.2016			3.888		|
*/

const unsigned char qma7981_init_tbl[][2] = 
{
#if defined(QMAX981_STEP_COUNTER)
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},	
	{0x0f, QMAX981_RANGE_4G},	// 0.488 mg
	{0x10, 0x05},		// BW 32.5hz
	{0x4a, 0x08},		//Force I2C I2C interface.SPI is disabled,SENB can be used as ATB
	{0x20, 0x05},
	// seting 1
	#if 0
	{0x10, 0x05},		// BW 32.5hz, 30.768 ms
	{0x12, 0x8f},		// 
	{0x13, 0x22},		// STEP_PRECISION<6:0>*LSB*16 , 0.488*16*40=310mg
	{0x14, 0x0e},		// STEP_TIME_LOW<7:0>*(1/ODR)
	{0x15, 0x09},		// STEP_TIME_UP<7:0>*8*(1/ODR)
	{0x1f, 0x00},
	#endif
	// setting 2
	#if 1	
	{0x0f, QMAX981_RANGE_4G},	// 0.488 mg
	{0x10, 0x06},		// ODR: 129.3995 Hz		delay:7.728 ms
	{0x12, 0x95},
	{0x13, 0x80},		// clear step
	{0x13, 0x22},		//old 0x22 // STEP_PRECISION<6:0>*LSB*16 , 0.488*16*34=265mg
	{0x14, 0x3a},		// STEP_TIME_LOW<7:0>*(1/ODR) 7.728*58=450ms(0x3a)
	{0x15, 0x20},		// STEP_TIME_UP<7:0>*8*(1/ODR) 7.728*32*8=1978ms
	{0x1f, 0x00},		// 390.4mg 11:<30*(16*LSB)   10:<150*(16*LSB)   01:<100*(16*LSB)   00:<50*(16*LSB) 
	#endif	
	// setting 3
	#if 0
	{0x0f, QMAX981_RANGE_8G},	// 0.977 mg
	{0x10, 0x05},		// ODR: 32.5013		delay:30.768 ms
	{0x12, 0x95},
	{0x13, 0x80},		// clear step
	{0x13, 0x11},		// STEP_PRECISION<6:0>*LSB*16 , 0.488*16*34=265mg
	{0x14, 0x0b},		// STEP_TIME_LOW<7:0>*(1/ODR) 7.728*58=450ms(0x3a)
	{0x15, 0x41},		// STEP_TIME_UP<7:0>*8*(1/ODR) 7.728*32*8=1978ms
	{0x1f, 0x03},		//468.96 mg 11:<30*(16*LSB)   10:<150*(16*LSB)   01:<100*(16*LSB)   00:<50*(16*LSB) 
	#endif
	
	{0x11, 0x80},
	{0x5f, 0x80},		// enable test mode,take control the FSM
	{0x5f, 0x00},		//normal mode
#else
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},
	{0x0f, QMAX981_RANGE_4G},
	{0x10, 0x05},		// BW 32.5hz	
	{0x4a, 0x08},		//Force I2C I2C interface.SPI is disabled,SENB can be used as ATB
	{0x20, 0x05},	
	{0x11, 0x80},
	{0x5f, 0x80},		// enable test mode,take control the FSM
	{0x5f, 0x00},		//normal mode
#endif
#if defined(QMAX981_FIFO_FUNC)
	{0x10, 0x06},
	{0x3E, 0x40},
	{0x17, 0x20},
	#if defined(QMAX981_FIFO_USE_INT)
	{0x1a, 0x20},	// fifo int map to int1
	#endif
#endif

	{0xff, 1}
};


void qmaX981_delay(unsigned int delay)
{
	int i,j;
	for(i=0;i<delay;i++)
	{
		for(j=0;j<1000;j++)
		{
			;
		}
	}
}

u8 qmaX981_writereg(u8 reg_add,u8 reg_dat)
{
	u8 ret;

	ret = I2C_ByteWrite(reg_dat,reg_add);
	return ret;
}

u8 qmaX981_readreg(u8 reg_add,u8 *buf,u8 num)
{
	u8 ret;

	ret = I2C_BufferRead(buf,reg_add,(u16)num);
	return ret;
}

u8 qmaX981_chip_id()
{
	u8 chip_id = 0x00;

	qmaX981_writereg(QMAX981_REG_POWER_CTL, 0x80);
	qmaX981_delay(5);
	qmaX981_readreg(QMAX981_CHIP_ID, &chip_id, 1);
	QMAX981_LOG("qmaX981_chip_id id=0x%x \n", chip_id);

	return chip_id;
}


static s32 qma6981_initialize(void)
{
	int ret = 0;
	int index, total;
	unsigned char data[2] = {0};

	total = sizeof(qma6981_init_tbl)/sizeof(qma6981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data[0] = qma6981_init_tbl[index][0];
		data[1] = qma6981_init_tbl[index][1];
		if(data[0] == 0xff)
		{
			qmaX981_delay(data[1]);
		}
		else
		{
			if(data[0] == QMAX981_REG_RANGE)
			{
				if(data[1] == QMAX981_RANGE_4G)
					g_qmaX981.lsb_1g = 128;
				else if(data[1] == QMAX981_RANGE_8G)
					g_qmaX981.lsb_1g = 64;
				else					
					g_qmaX981.lsb_1g = 256;
			}

			ret = qmaX981_writereg(data[0],data[1]);
			if(ret == 0)
			{
				QMAX981_ERR("qma6981_initialize ret=%d reg_addr=%x \n", ret, data[0]);
				//return ret;
			}
			qmaX981_delay(2);
		}
	}

   	return ret;
}


static s32 qma7981_initialize(void)
{
	int ret = 0;
	int index, total;
	unsigned char data[2] = {0};
	// for 7981, from peili, delete later
	unsigned char r_58,r_59,r_5a,r_42,r_44,r_3d;
	// peili

	total = sizeof(qma7981_init_tbl)/sizeof(qma7981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data[0] = qma7981_init_tbl[index][0];
		data[1] = qma7981_init_tbl[index][1];
		if(data[0] == 0xff)
		{
			qmaX981_delay(data[1]);
		}
		else
		{
			if(data[0] == QMAX981_REG_RANGE)
			{
				if(data[1] == QMAX981_RANGE_4G)
					g_qmaX981.lsb_1g = 2048;
				else if(data[1] == QMAX981_RANGE_8G)
					g_qmaX981.lsb_1g = 1024;
				else if(data[1] == QMAX981_RANGE_16G)
					g_qmaX981.lsb_1g = 512;
				else if(data[1] == QMAX981_RANGE_32G)
					g_qmaX981.lsb_1g = 256;
				else
					g_qmaX981.lsb_1g = 4096;
			}
			ret = qmaX981_writereg(data[0],data[1]);
			if(ret == 0)
			{
				QMAX981_ERR("qma7981_initialize ret=%d\n", ret);
				return ret;
			}
			qmaX981_delay(2);
		}
	}
#if 1
	// for 7981, from peili, delete later
	r_58 = 0x58;	
	ret=qmaX981_readreg(0x58, &r_58, 1);
	qmaX981_delay(2);

	r_59 = 0x59;	
	ret=qmaX981_readreg(0x59, &r_59, 1);
	qmaX981_delay(2);

	r_5a = 0x5a;	
	ret=qmaX981_readreg(0x5a, &r_5a, 1);
	qmaX981_delay(2);

	r_42 = 0x42;	
	ret=qmaX981_readreg(0x42, &r_42, 1);
	qmaX981_delay(2);

	r_44 = 0x44;	
	ret=qmaX981_readreg(0x44, &r_44, 1);
	qmaX981_delay(2);

	r_3d = 0x3d;	
	ret=qmaX981_readreg(0x3d, &r_3d, 1);
	qmaX981_delay(2);
	
	//write 0x58<3> to 0x3D<7>
	data[0] = 0x3d;
	data[1] = (r_3d|0x7f)|((r_58&0x08)<<4);
	ret = qmaX981_writereg(data[0],data[1]);
	qmaX981_delay(2);
	//write 0x5a<7:5>,0x59<7:5> to 0x42<5:0>
	data[0] = 0x42;
	data[1] = (r_42&0xc0)|((r_59&0xe0)>>5)|((r_5a&0xe0)>>2);
	ret = qmaX981_writereg(data[0],data[1]);
	qmaX981_delay(2);
	//write 0x59<3:0> to 0x44<3:0>
	data[0] = 0x44;
	data[1] = (r_44&0xf0)|(r_59&0x0f);
	ret = qmaX981_writereg(data[0],data[1]);
	qmaX981_delay(2);

	// peili
#endif
   	return ret;
}



static void qmaX981_setup_irq1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;  
    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;   
      
    RCC_APB2PeriphClockCmd(QMAX981_IRQ1_RCC|RCC_APB2Periph_AFIO, ENABLE);//打开GPIO AFIO的时钟  
    GPIO_InitStructure.GPIO_Pin = QMAX981_IRQ1_PIN;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
    GPIO_Init(QMAX981_IRQ1_PORT, &GPIO_InitStructure); 
          
    EXTI_ClearITPendingBit(EXTI_Line11);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11);//PC11  为GPIOC的PIN11  
    EXTI_InitStructure.EXTI_Line= EXTI_Line11; //PC11，为：EXTI_Line11  
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising_Falling;   //中断方式为上升与下降沿  
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure); 
}

static void qmaX981_setup_irq2(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;  
    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;   
      
    RCC_APB2PeriphClockCmd(QMAX981_IRQ2_RCC|RCC_APB2Periph_AFIO, ENABLE);//打开GPIO AFIO的时钟  
    GPIO_InitStructure.GPIO_Pin = QMAX981_IRQ2_PIN;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
    GPIO_Init(QMAX981_IRQ2_PORT, &GPIO_InitStructure); 
          
    EXTI_ClearITPendingBit(EXTI_Line5);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);//PC11  为GPIOC的PIN11  
    EXTI_InitStructure.EXTI_Line= EXTI_Line5; //PC11，为：EXTI_Line11  
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;   //中断方式为上升与下降沿  
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure); 
}

void EXTI15_10_IRQHandler(void)         //这里为：EXTI15_10 (外部中断号的10~15都在这里实现）  
{
	u8 ret;
	u8 data[2];
	int step_num;
	
#if defined(QMAX981_TAP_FUNC)
	ret = qmaX981_readreg(QMAX981_INT_STAT0, data, 1);
	printf("EXTI15_10_IRQHandler value_0a=%x \r\n", data[0]);
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line11);		 //清中断  
		printf("EXTI_ClearITPendingBit\r\n");
	}
#endif
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)	
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) //这里为判断相应的中断号是否进入中断，如果有多个中断的话。  
	{
		g_qmaX981.int_level = GPIO_ReadInputDataBit(QMAX981_IRQ1_PORT,QMAX981_IRQ1_PIN);		
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
		step_num = (data[1]<<8)|data[0];
		printf("gpio level = %d step_num=%d \r\n", g_qmaX981.int_level, step_num);
		qmaX981_step_debounce_int_work(step_num, g_qmaX981.int_level);
		EXTI_ClearITPendingBit(EXTI_Line11);		 //清中断  
		printf("EXTI_ClearITPendingBit\r\n");
	}
#endif
}


void EXTI9_5_IRQHandler(void)         //这里为：EXTI15_10 (外部中断号的10~15都在这里实现）  
{
	u8 ret;
	u8 value_0a;
	
	ret = qmaX981_readreg(QMAX981_INT_STAT0, &value_0a, 1);
	printf("EXTI9_5_IRQHandler value_0a=%x\r\n", value_0a);
	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) //这里为判断相应的中断号是否进入中断，如果有多个中断的话。  
	{  
		  EXTI_ClearITPendingBit(EXTI_Line5);		 //清中断  
		  printf("EXTI_ClearITPendingBit\r\n");
	}
}


s32 qmaX981_init(void)
{
	int ret = 0;

	I2C_Bus_set_slave_addr(QMAX981_ACC_I2C_ADDRESS);
	qmaX981_delay(1000);

	memset(&g_qmaX981, 0, sizeof(g_qmaX981));
	g_qmaX981.chip_id = qmaX981_chip_id();
	if((g_qmaX981.chip_id>=0xb0) && (g_qmaX981.chip_id<=0xb6))
	{		
		QMAX981_LOG("qma6981 find \n");
		g_qmaX981.chip_type = CHIP_TYPE_QMA6981;	
	}
	else if((g_qmaX981.chip_id>=0xe0) && (g_qmaX981.chip_id<=0xe6))	
	{
		QMAX981_LOG("qma7981 find \n");		
		g_qmaX981.chip_type = CHIP_TYPE_QMA7981;	
	}	
	else	
	{		
		QMAX981_LOG("qma acc chip id not defined!!! \n");		
		g_qmaX981.chip_type = CHIP_TYPE_UNDEFINE;	
	}

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)		
		ret = qma6981_initialize();	
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)		
		ret = qma7981_initialize();
	else
		ret = 0;

	g_qmaX981.layout = 3;
	memcpy(&g_qmaX981.cvt, &qst_map[g_qmaX981.layout], sizeof(qst_convert));

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	qmaX981_setup_irq1();
	qmaX981_step_debounce_reset();
#endif
	return ret;
}


static int qma6981_read_raw_xyz(int *data)
{
	//int res;	
	unsigned char databuf[6] = {0};		
	unsigned char i;
	int ret;

	ret = qmaX981_readreg(QMAX981_XOUTL, databuf, 6);
	if(ret == 0){
		QMAX981_ERR("read xyz error!!!");
		return 0;	
	}
 	data[0]  = (short)((databuf[1]<<2) |( databuf[0]>>6));
	data[1]  = (short)((databuf[3]<<2) |( databuf[2]>>6));
	data[2]  = (short)((databuf[5]<<2) |( databuf[4]>>6));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data[i] == 0x0200 )	//so we want to calculate actual number here
			data[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data[i] & 0x0200 )	//transfor format
		{					//printk("data 0 step %x \n",data[i]);
			data[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
			data[i] = ~data[i];		//printk("data 2 step %x \n",data[i]);
			data[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
			data[i] = -data[i];		
		}
#if defined(QMAX981_STEP_COUNTER)
		data[i] -= QMA6981_OFFSET;
#endif
	}

	//printk("yzqaccraw	%d	%d	%d\n", data[0], data[1], data[2]);
	return 1;
}

static int qma7981_read_raw_xyz(int *data)
{
	unsigned char databuf[6] = {0}; 	
	int ret;
	//unsigned char i;
	//qma7981_acc_format data_14bit;

	ret = qmaX981_readreg(QMAX981_XOUTL, databuf, 6);
	if(ret == 0){
		QMAX981_ERR("read xyz error!!!");
		return 0;	
	}

	data[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data[0] = data[0]>>2;
	data[1] = data[1]>>2;
	data[2] = data[2]>>2;

	return 1;
}

s32 qmaX981_read_raw(s32 *rawData)
{
	int ret;

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)		
		ret = qma6981_read_raw_xyz(rawData);	
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)		
		ret = qma7981_read_raw_xyz(rawData);
	else
		ret = 0;

	return ret;
}


s32 qmaX981_read_acc(s32 *accData)
{
	int ret;
	s32 rawData[3];

	qmaX981_read_raw(rawData);
	accData[g_qmaX981.cvt.map[0]] = g_qmaX981.cvt.sign[0]*rawData[0];
	accData[g_qmaX981.cvt.map[1]] = g_qmaX981.cvt.sign[1]*rawData[1];
	accData[g_qmaX981.cvt.map[2]] = g_qmaX981.cvt.sign[2]*rawData[2];

	accData[0] = (accData[0]*GRAVITY_EARTH_1000)/(g_qmaX981.lsb_1g);
	accData[1] = (accData[1]*GRAVITY_EARTH_1000)/(g_qmaX981.lsb_1g);
	accData[2] = (accData[2]*GRAVITY_EARTH_1000)/(g_qmaX981.lsb_1g);

	return ret;

}

#if defined(QMAX981_STEP_COUNTER)
int qmaX981_read_stepcounter(void)
{
	int ret;
	u8 data[2];
	int step_num;

	ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
	step_num = (data[1]<<8)|data[0];
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
	ret=qmaX981_check_abnormal_data(step_num, &step_num);
	if(ret != 0)
	{
		return -1;
	}
#endif
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	step_num = qmaX981_step_debounce_read_data(step_num);
#endif

	return step_num;
}
#endif


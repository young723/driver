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
#include <REGX52.H>

#include "qmaX981.h"

//#include <stdbool.h>
#include <string.h>

extern void qst_log(const char *format, ...);
	
#define QMAX981_LOG		qst_log
#define QMAX981_ERR		qst_log
//#define QMAX981_USE_SPI

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
extern void qmaX981_step_debounce_reset(void);
extern int qmaX981_step_debounce_int_work(int data_buf, unsigned char irq_level);
extern int qmaX981_step_debounce_read_data(int result);
#endif
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
extern int qmaX981_check_abnormal_data(int data_in, int *data_out);
#endif

#if defined(QMAX981_USE_SPI)
extern u8 qmaX981_spi_read(u8 addr, u8* buff, u8 len);
extern void qmaX981_spi_write(u8 addr,u8 data);
extern void Spi_Init(void);
extern void spi_sw_Init(void);
extern void MYSPI_Init(void);
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

static const qst_convert xdata qst_map[] = 
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

//#define QMAX981_I2C_ADDR_W		QMAX981_ACC_I2C_ADDRESS
//#define QMAX981_I2C_ADDR_R		QMAX981_ACC_I2C_ADDRESS|0x01
static unsigned char QMAX981_I2C_ADDR_W	= QMAX981_ACC_I2C_ADDRESS;
static unsigned char QMAX981_I2C_ADDR_R	= QMAX981_ACC_I2C_ADDRESS|0x01;

const unsigned char xdata qma6981_init_tbl[][2] = 
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
	{0x2b, 0x07},	//0x14	125*7
	{0x16, 0x20},	
	{0x19, 0x20},
	//{0x1b, 0x20},
#endif
#if defined(QMAX981_INT_LATCH_MODE)
	{0x21, 0x01},
#endif
#if 1
	{0x20, 0x00},		// 下降沿触发
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

const unsigned char xdata qma7981_init_tbl[][2] = 
{
#if defined(QMAX981_STEP_COUNTER)
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},	
	{0x0f, QMAX981_RANGE_4G},	// 0.488 mg
	{0x10, 0x05},		// BW 32.5hz
	//{0x4a, 0x08},		//Force I2C I2C interface.SPI is disabled,SENB can be used as ATB
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
	//{0x4a, 0x08},		//Force I2C I2C interface.SPI is disabled,SENB can be used as ATB
	{0x20, 0x05},	
	{0x11, 0x80},
	{0x5f, 0x80},		// enable test mode,take control the FSM
	{0x5f, 0x00},		//normal mode
#endif
#if defined(QMAX981_FIFO_FUNC)
	{0x5b, 0x08},	// set i2c clock
	{0x10, 0x05},
	{0x3E, 0x40},
	{0x17, 0x20},
	#if defined(QMAX981_FIFO_USE_INT)
	{0x1a, 0x20},	// fifo int map to int1
	#endif
#endif
#if defined(QMA7981_IRQ_TEST)
	{0x16, 0x48},
	{0x19, 0x48},
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
	u8 ret=0;

	ret = qst_sw_writereg(reg_add,reg_dat);
	return 1;
}

u8 qmaX981_readreg(u8 reg_add,u8 *buf,u8 num)
{
	u8 ret;

	ret = qst_sw_readreg(reg_add,buf,num);
	return ret;
}


u8 qmaX981_chip_id()
{
	u8 chip_id = 0x00;
	qmaX981_writereg(QMAX981_REG_POWER_CTL, 0x80);

	qmaX981_readreg(QMAX981_CHIP_ID, &chip_id, 1);
	QMAX981_LOG("qmaX981_chip_id id=0x%x \n", (u16)chip_id);

	return chip_id;
}


static s32 qma6981_initialize(void)
{
	int ret = 0;
	int index, total;
	unsigned char data_buf[2] = {0};

	total = sizeof(qma6981_init_tbl)/sizeof(qma6981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data_buf[0] = qma6981_init_tbl[index][0];
		data_buf[1] = qma6981_init_tbl[index][1];
		if(data_buf[0] == 0xff)
		{
			qmaX981_delay(data_buf[1]);
		}
		else
		{
			if(data_buf[0] == QMAX981_REG_RANGE)
			{
				if(data_buf[1] == QMAX981_RANGE_4G)
					g_qmaX981.lsb_1g = 128;
				else if(data_buf[1] == QMAX981_RANGE_8G)
					g_qmaX981.lsb_1g = 64;
				else					
					g_qmaX981.lsb_1g = 256;
			}

			ret = qmaX981_writereg(data_buf[0],data_buf[1]);
			if(ret == 0)
			{
				QMAX981_ERR("qma6981_initialize ret=%d reg_addr=%x \n", ret, data_buf[0]);
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
	unsigned char data_buf[2] = {0};
	// for 7981, from peili, delete later
#if 0
	unsigned char r_58,r_59,r_5a,r_42,r_44,r_3d;
#endif
	// peili

	total = sizeof(qma7981_init_tbl)/sizeof(qma7981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data_buf[0] = qma7981_init_tbl[index][0];
		data_buf[1] = qma7981_init_tbl[index][1];
		if(data_buf[0] == 0xff)
		{
			qmaX981_delay(data_buf[1]);
		}
		else
		{
			if(data_buf[0] == QMAX981_REG_RANGE)
			{
				if(data_buf[1] == QMAX981_RANGE_4G)
					g_qmaX981.lsb_1g = 2048;
				else if(data_buf[1] == QMAX981_RANGE_8G)
					g_qmaX981.lsb_1g = 1024;
				else if(data_buf[1] == QMAX981_RANGE_16G)
					g_qmaX981.lsb_1g = 512;
				else if(data_buf[1] == QMAX981_RANGE_32G)
					g_qmaX981.lsb_1g = 256;
				else
					g_qmaX981.lsb_1g = 4096;
			}
			ret = qmaX981_writereg(data_buf[0],data_buf[1]);
			if(ret == 0)
			{
				QMAX981_ERR("qma7981_initialize ret=%d\n", ret);
				return ret;
			}
			qmaX981_delay(2);
		}
	}
#if 0
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
	data_buf[0] = 0x3d;
	data_buf[1] = (r_3d|0x7f)|((r_58&0x08)<<4);
	ret = qmaX981_writereg(data_buf[0],data_buf[1]);
	qmaX981_delay(2);
	//write 0x5a<7:5>,0x59<7:5> to 0x42<5:0>
	data_buf[0] = 0x42;
	data_buf[1] = (r_42&0xc0)|((r_59&0xe0)>>5)|((r_5a&0xe0)>>2);
	ret = qmaX981_writereg(data_buf[0],data_buf[1]);
	qmaX981_delay(2);
	//write 0x59<3:0> to 0x44<3:0>
	data_buf[0] = 0x44;
	data_buf[1] = (r_44&0xf0)|(r_59&0x0f);
	ret = qmaX981_writereg(data_buf[0],data_buf[1]);
	qmaX981_delay(2);

	// peili
#endif
   	return ret;
}


#if defined(QMAX981_FIFO_FUNC)
static int qmaX981_fifo_data[32][3];

static int qma6981_read_fifo_raw(int *data_raw)
{
	//int res;	
	unsigned char databuf[6] = {0};		
	unsigned char i;
	int ret;
	
	ret = qmaX981_readreg(0x3f, databuf, 6);
	qmaX981_delay(2);
	if(ret != 1)
	{
		QMAX981_LOG("qma6981_read_fifo_raw error \n");
		return ret;
	}

 	data_raw[0]  = (short)((databuf[1]<<2) |( databuf[0]>>6));
	data_raw[1]  = (short)((databuf[3]<<2) |( databuf[2]>>6));
	data_raw[2]  = (short)((databuf[5]<<2) |( databuf[4]>>6));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data_raw[i] == 0x0200 )	//so we want to calculate actual number here
			data_raw[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data_raw[i] & 0x0200 )	//transfor format
		{					//printk("data 0 step %x \n",data[i]);
			data_raw[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
			data_raw[i] = ~data_raw[i];		//printk("data 2 step %x \n",data[i]);
			data_raw[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
			data_raw[i] = -data_raw[i];		
		}
#if defined(QMAX981_STEP_COUNTER)
		data_raw[i] -= QMA6981_OFFSET;
#endif
	}
	//printk("qma6981 fifo raw: %d	%d	%d\n", data[0], data[1], data[2]);	

	return 1;	
}

static int qma7981_read_fifo_raw(int *data_raw)
{
	int res;	
	unsigned char databuf[6] = {0};
	int ret;
	
	ret = qmaX981_readreg(0x3f, databuf, 6);
	qmaX981_delay(2);
	if(ret != 1)
	{
		QMAX981_LOG("qma7981_read_fifo_raw error \n");
		return ret;
	}

	data_raw[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data_raw[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data_raw[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data_raw[0] = data_raw[0]>>2;
	data_raw[1] = data_raw[1]>>2;
	data_raw[2] = data_raw[2]>>2;

	//printk("qma7981 fifo raw: %d	%d	%d\n", data[0], data[1], data[2]);	
	return 1;
}

static int qmaX981_read_fifo_acc(int *acc_data)
{
	int ret = 0;
	int raw_data[3];

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
	{
		ret = qma6981_read_fifo_raw(raw_data);
	}
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)
	{
		ret = qma7981_read_fifo_raw(raw_data);
	}
	else
	{
		ret = 0;
	}
	
	if(1 != ret ){
		QMAX981_ERR("qmaX981_read_fifo_acc error\n");
		return ret;
	}
	
	//remap coordinate
	acc_data[g_qmaX981.cvt.map[0]] = g_qmaX981.cvt.sign[0]*raw_data[0];
	acc_data[g_qmaX981.cvt.map[1]] = g_qmaX981.cvt.sign[1]*raw_data[1];
	acc_data[g_qmaX981.cvt.map[2]] = g_qmaX981.cvt.sign[2]*raw_data[2];
	//QMAX981_LOG("qmaX981 AFTER x1:%d,y:%d,z:%d\n",data[0],data[1],data[2]);

	acc_data[0] = (acc_data[0]*9807)/(g_qmaX981.lsb_1g);
	acc_data[1] = (acc_data[1]*9807)/(g_qmaX981.lsb_1g);
	acc_data[2] = (acc_data[2]*9807)/(g_qmaX981.lsb_1g);

	return ret;
}

static int qmaX981_read_fifo(unsigned char is_raw)
{
	int ret = 0;
	unsigned char databuf[2];
	int acc_data[3];
	int icount;
	int fifo_depth = 32;

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)
		fifo_depth = 16;
	else
		fifo_depth = 32;

	ret = qmaX981_readreg(QMAX981_FIFO_STATE, databuf, 1);
	qmaX981_delay(2);

	QMAX981_LOG("fifo level = %d   %d \r\n", fifo_depth, databuf[0]&0x7f);

	if((databuf[0]&0x7f)==fifo_depth)
	{
		for(icount=0; icount<fifo_depth; icount++)
		{
			if(is_raw == 1)
			{
				if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
				{
					ret = qma6981_read_fifo_raw(acc_data);
				}
				else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)
				{
					ret = qma7981_read_fifo_raw(acc_data);
				}
				else
				{
					ret = 0;
				}
			}
			else
			{
				ret = qmaX981_read_fifo_acc(acc_data);
			}
			
			if(ret != 1)
			{
				QMAX981_ERR("read 0x3f error!\n");
				return ret;
			}
			qmaX981_fifo_data[icount][0] = acc_data[0];
			qmaX981_fifo_data[icount][1] = acc_data[1];
			qmaX981_fifo_data[icount][2] = acc_data[2];
			QMAX981_LOG("fifo_data %d: %f	%f	%f \r\n", icount, acc_data[0]/1000.00, acc_data[1]/1000.00, acc_data[2]/1000.00);
		}
		// read status reg
	}
	else
	{
		ret = 0;
	}
	ret = qmaX981_readreg(QMAX981_INT_STAT1, databuf, 1);
	qmaX981_delay(2);
	// write 0x3e
	ret = qmaX981_writereg(0x3e, 0x40);
	qmaX981_delay(2);

	return ret;
}
#endif


#if 0//defined(QMAX981_USE_IRQ1)
static void qmaX981_setup_irq1(void)
{

}

void EXTI15_10_IRQHandler(void)         //这里为：EXTI15_10 (外部中断号的10~15都在这里实现）  
{

}

#endif


#if 0//defined(QMAX981_USE_IRQ2)
static void qmaX981_setup_irq2(void)
{
}


void EXTI9_5_IRQHandler(void)
{

}
#endif


s32 qmaX981_init(void)
{
	int ret = 0;

	i2c_CheckDevice(QMAX981_I2C_ADDR_W);
#if defined(QMAX981_USE_SPI)
	Spi_Init();
#endif
	qmaX981_delay(100);

	memset(&g_qmaX981, 0, sizeof(g_qmaX981));
	g_qmaX981.chip_id = qmaX981_chip_id();
	if((g_qmaX981.chip_id>=0xa9) && (g_qmaX981.chip_id<=0xb6))
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
	// add by yangzhiqiang use another i2c addr
	if(g_qmaX981.chip_type == CHIP_TYPE_UNDEFINE)
	{
		QMAX981_LOG("qmaX981 change I2C add = 0x%x! \n", QMAX981_ACC_I2C_ADDRESS2);		
		qmaX981_delay(1000);
		QMAX981_I2C_ADDR_W = QMAX981_ACC_I2C_ADDRESS2;
		QMAX981_I2C_ADDR_R = QMAX981_ACC_I2C_ADDRESS2|0x01;
		i2c_CheckDevice(QMAX981_I2C_ADDR_W);
		qmaX981_delay(1000);
		g_qmaX981.chip_id = qmaX981_chip_id();
		if((g_qmaX981.chip_id>=0xa9) && (g_qmaX981.chip_id<=0xb6))
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
	}
	// add by yangzhiqiang
	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)		
		ret = qma6981_initialize();	
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)		
		ret = qma7981_initialize();
	else
		ret = 0;

	g_qmaX981.layout = 3;
	memcpy(&g_qmaX981.cvt, &qst_map[g_qmaX981.layout], sizeof(qst_convert));

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	qmaX981_step_debounce_reset();
#endif

#if 0//defined(QMAX981_USE_IRQ1)
	qmaX981_setup_irq1();
#endif
	return ret;
}


static int qma6981_read_raw_xyz(int *data_buf)
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
 	data_buf[0]  = (short)((databuf[1]<<2) |( databuf[0]>>6));
	data_buf[1]  = (short)((databuf[3]<<2) |( databuf[2]>>6));
	data_buf[2]  = (short)((databuf[5]<<2) |( databuf[4]>>6));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data_buf[i] == 0x0200 )	//so we want to calculate actual number here
			data_buf[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data_buf[i] & 0x0200 )	//transfor format
		{					//printk("data 0 step %x \n",data[i]);
			data_buf[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
			data_buf[i] = ~data_buf[i];		//printk("data 2 step %x \n",data[i]);
			data_buf[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
			data_buf[i] = -data_buf[i];		
		}
#if defined(QMAX981_STEP_COUNTER)
		data_buf[i] -= QMA6981_OFFSET;
#endif
	}

	//printk("yzqaccraw	%d	%d	%d\n", data[0], data[1], data[2]);
	return 1;
}

static int qma7981_read_raw_xyz(int *data_buf)
{
	unsigned char databuf[6] = {0}; 	
	int ret;
	//qma7981_acc_format data_14bit;

	ret = qmaX981_readreg(QMAX981_XOUTL, databuf, 6);
	if(ret == 0){
		QMAX981_ERR("read xyz error!!!");
		return 0;	
	}

	data_buf[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data_buf[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data_buf[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data_buf[0] = data_buf[0]>>2;
	data_buf[1] = data_buf[1]>>2;
	data_buf[2] = data_buf[2]>>2;

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



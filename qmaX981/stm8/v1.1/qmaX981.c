
#include "stm8s.h"
#include "qst_i2c.h"
#if defined(USE_SPI)
#include "qst_spi.h"
#endif
#include "bsp_timer.h"

#if defined(QST_CONFIG_QMAX981)
// for qma7981
#define QMAX981_STEPCOUNTER
//#define QMA7981_ANY_MOTION
//#define QMA7981_NO_MOTION
//#define QMA7981_SIGNIFICANT_MOTION
//#define QMA7981_INT_LATCH
//#define QMA7981_HAND_UP_DOWN
//#define QMA7981_6100_FIFO
// 7981

#define QMAX981_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))
#define GRAVITY_EARTH_1000          9807

enum 
{
	ACC_RANGE_2G,
	ACC_RANGE_4G,
	ACC_RANGE_8G,
	ACC_RANGE_16G,
	ACC_RANGE_32G,

	ACC_RANGE_TOTAL
};

extern void delay_ms(unsigned long nms);
extern void qst_printf(const char *format, ...);
extern void bsp_led_set(int enable);

#define QST_PRINTF		qst_printf
//#define QST_PRINTF

static unsigned char acc_chip_id=0;
static unsigned short acc_lsb_div = 0;

enum qmaX981_axis 
{
	QMAX981_AXIS_X = 0,
	QMAX981_AXIS_Y,
	QMAX981_AXIS_Z,
	QMAX981_AXIS_NUM
};

struct qmaX981_convert 
{
	signed char sign[3];
	unsigned char map[3];
};

static struct qmaX981_convert g_map;
static int qmaX981_layout = 0;

uint8_t qmaX981_init();

uint8_t qmaX981_write_reg(uint8_t Addr, uint8_t Data)
{
#if defined(USE_SPI)
	return qst_qmaX981_spi_write(Addr, Data);
#else
	return qst_iic_write((0x12<<1), Addr, Data);
#endif
}

uint8_t qmaX981_read_reg(uint8_t Addr, uint8_t* Buf, uint8_t Len)
{
#if defined(USE_SPI)
	return qst_qmaX981_spi_read(Addr, Buf, Len);
#else
	return qst_iic_read((0x12<<1), Addr, Buf, Len);
#endif
}

void qmaX981_set_layout(int layout)
{
	if(layout == 0)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 1)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 2)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 3)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}	
	else if(layout == 4)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 5)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 6)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 7)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else		
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
}


int qmaX981_read_xyz(int acc[3])
{
	uint8_t reg_data[6];
	int16_t raw[3];
	int32_t raw_32[3];
	uint8_t ret;

	ret = qmaX981_read_reg(0x01, reg_data, 6);
	if(ret == 0)
	{
		QST_PRINTF("qmaX981_read_xyz error! \n");
		return 0;
	}
 	raw[0] = (int16_t)((int16_t)(reg_data[1]<<8)|(int16_t)(reg_data[0]));
	raw[1] = (int16_t)((int16_t)(reg_data[3]<<8)|(int16_t)(reg_data[2]));
	raw[2] = (int16_t)((int16_t)(reg_data[5]<<8)|(int16_t)(reg_data[4]));
	
	if((acc_chip_id>=0xb0)&&(acc_chip_id<=0xb9))
	{
		raw[0] = raw[0]>>6;
		raw[1] = raw[1]>>6;
		raw[2] = raw[2]>>6;
	}
	else if((acc_chip_id>=0xe0)&&(acc_chip_id<=0xe9))
	{		
		raw[0] = raw[0]>>2;
		raw[1] = raw[1]>>2;
		raw[2] = raw[2]>>2;
	}

	//QST_PRINTF("acc raw	%d	%d	%d\n", raw[0], raw[1], raw[2]);

	raw_32[0] = g_map.sign[QMAX981_AXIS_X]*raw[g_map.map[QMAX981_AXIS_X]];
	raw_32[1] = g_map.sign[QMAX981_AXIS_Y]*raw[g_map.map[QMAX981_AXIS_Y]];
	raw_32[2] = g_map.sign[QMAX981_AXIS_Z]*raw[g_map.map[QMAX981_AXIS_Z]];

	acc[0] = (raw_32[0]*GRAVITY_EARTH_1000)/(acc_lsb_div);
	acc[1] = (raw_32[1]*GRAVITY_EARTH_1000)/(acc_lsb_div);
	acc[2] = (raw_32[2]*GRAVITY_EARTH_1000)/(acc_lsb_div);

//	QST_PRINTF("acc	%f	%f	%f\n",(float)acc[0]/1000.0,(float)acc[1]/1000.0,(float)acc[2]/1000.0);
	//QST_PRINTF("acc	%d	%d	%d\n",acc[0],acc[1],acc[2]);

	return 1;
}


#if 1 //defined(QMAX981_STEPCOUNTER)
unsigned int qmaX981_read_step(void)
{
	uint8_t reg_data[3];
	int step;

	qmaX981_read_reg(0x07, reg_data, 2);
	
	if((acc_chip_id>=0xe0)&&(acc_chip_id<=0xe9))
	{
		qmaX981_read_reg(0x0e, &reg_data[2], 1);
		step = ((int)reg_data[2]<<16)|((int)reg_data[1]<<8)|reg_data[0];
	}
	else
	{
		step = ((unsigned short)reg_data[1]<<8)|reg_data[0];
	}

	return step;
}
#endif

void qmaX981_set_range(int range)
{
	if((acc_chip_id>=0xb0)&&(acc_chip_id<=0xb9))
	{
		if(ACC_RANGE_4G == range)
		{
			qmaX981_write_reg(0x0f, 0x02);
			acc_lsb_div = 128;
		}
		else if(ACC_RANGE_8G == range)
		{
			qmaX981_write_reg(0x0f, 0x04);
			acc_lsb_div = 64;
		}
		else
		{
			qmaX981_write_reg(0x0f, 0x01);
			acc_lsb_div = 256;
		}
	}
	else if((acc_chip_id>=0xe0)&&(acc_chip_id<=0xe9))
	{
		if(ACC_RANGE_4G == range)
		{
			qmaX981_write_reg(0x0f, 0x02);
			acc_lsb_div = 2048;
		}
		else if(ACC_RANGE_8G == range)
		{
			qmaX981_write_reg(0x0f, 0x04);
			acc_lsb_div = 1024;
		}
		else if(ACC_RANGE_16G == range)
		{
			qmaX981_write_reg(0x0f, 0x08);
			acc_lsb_div = 512;
		}
		else if(ACC_RANGE_32G == range)
		{
			qmaX981_write_reg(0x0f, 0x0f);
			acc_lsb_div = 256;
		}
		else
		{
			qmaX981_write_reg(0x0f, 0x01);
			acc_lsb_div = 4096;
		}		
	}
}

void qmaX981_read_fifo(void)
{
	short i;
	unsigned char reg_data[6];
	unsigned char reg_0x0e = 0;
	short raw_data[3];
	
	qmaX981_read_reg(0x0e, &reg_0x0e, 1);
	QST_PRINTF("fifo level:%d\n", reg_0x0e);
	if(acc_chip_id==0xe8)
	{
		if(reg_0x0e == 64)
		{
			for(i=0; i<reg_0x0e; i++)
			{
				qmaX981_read_reg(0x3f,reg_data,6);
				raw_data[0] = (short)(((unsigned short)reg_data[1]<<8)|(reg_data[0]));	
				raw_data[1] = (short)(((unsigned short)reg_data[3]<<8)|(reg_data[2]));	
				raw_data[2] = (short)(((unsigned short)reg_data[5]<<8)|(reg_data[4]));
				raw_data[0] = raw_data[0]>>2;
				raw_data[1] = raw_data[1]>>2;
				raw_data[2] = raw_data[2]>>2;
				QST_PRINTF("FIFO level[%d]	DATA:	%d	%d	%d \n", 
					i,raw_data[0],raw_data[1],raw_data[2]);
			}
			qmaX981_write_reg(0x3E, 0x40);
		}
	}
}


void qmaX981_anymotion_enable(unsigned char flag)
{
	unsigned char reg_0x1a = 0;
	qmaX981_read_reg(0x1a, &reg_0x1a, 1);

	if(flag)
	{	
		reg_0x1a |= 0x01;
		qmaX981_write_reg(0x1a, reg_0x1a);
	}
	else
	{
		reg_0x1a &= 0xfe;
		qmaX981_write_reg(0x1a, reg_0x1a);
	}
}

#if defined(QMA7981_6100_FIFO)
void qmaX981_irq_fifo_hdlr(void)
{
//	unsigned char r_data[4];

	qmaX981_read_fifo();	
//	qmaX981_read_reg(0x0a,r_data,3);
//	QST_PRINTF("irq [%d %d %d]!!!", r_data[0],r_data[1],r_data[2]);
}
#endif

void qst_show_info(char flag)
{
	if(flag)
	{	
		GPIO_WriteHigh(GPIOD,GPIO_PIN_2);
#if defined(OLED_SUPPORT)
		OLED_ShowString(0,1,"Click!",12);
#endif
	}
	else
	{
		GPIO_WriteLow(GPIOD,GPIO_PIN_2);
#if defined(OLED_SUPPORT)
		OLED_Clear();
#endif
	}
}


unsigned char qmaX981_irq_hdlr(void)
{
	unsigned char r_data[4];
	//unsigned char reg_0x18 = 0;
	//unsigned char reg_0x1a = 0;
	unsigned char int_type = 0xff;

#if defined(QMA7981_6100_FIFO)
	if(acc_chip_id==0xe8)
	{
		qmaX981_irq_fifo_hdlr();
		return 0xff;
	}
#endif
	qmaX981_read_reg(0x09, r_data, 3);
//	QST_PRINTF("irq [%d %d %d]!!!", r_data[0],r_data[1],r_data[2]);
	if(r_data[0] & 0xF)
	{	
#if 0//defined(QMA7981_NO_MOTION)
		qmaX981_readreg(0x1a,&reg_0x1a,1);
		reg_0x1a |= 0x80;			// enable nomotion
		//reg_0x1a &= 0xfe; 		// disable anymotion
		qmaX981_writereg(0x1a, reg_0x1a);
#endif
		int_type = 1;
//		QST_PRINTF(" any motion!\n");
	}
	else if(r_data[0] & 0x80)
	{	
#if 0//defined(QMA7981_NO_MOTION)
		qmaX981_read_reg(0x1a,&reg_0x1a,1);
		reg_0x1a &= 0x7f;
		qmaX981_write_reg(0x1a, reg_0x1a);		// disable nomotion
#endif
		int_type = 2;
		QST_PRINTF(" no motion!\n");
	}
	else if(r_data[1] & 0x01)
	{	
		int_type = 3;
		
#if 0//defined(QMA7981_NO_MOTION)
		qmaX981_read_reg(0x1a,&reg_0x1a,1);
		reg_0x1a |= 0x80;			// enable nomotion
		//reg_0x1a &= 0xfe;			// disable anymotion
		qmaX981_write_reg(0x1a, reg_0x1a);
#endif
		QST_PRINTF(" significant motion!\n");
	}
	else if(r_data[1] & 0x40)
	{	
		int_type = 4;
		QST_PRINTF("  significant step int!\n");
	}
	else if(r_data[1] & 0x08)
	{
		int_type = 5;
		QST_PRINTF(" step int!\n");
	}
#if defined(QMA7981_HAND_UP_DOWN)
	else if(r_data[1] & 0x02)
	{
		int_type = 6;
		QST_PRINTF(" hand raise!\n");
		bsp_led_set(1);
	}
	else if(r_data[1] & 0x04)
	{
		int_type = 7;
		QST_PRINTF(" hand down!\n");
		bsp_led_set(0);
	}
#endif

	return int_type;
}


#if defined(QMA7981_HAND_UP_DOWN)
void qma7981_set_hand_up_down(int layout)
{
#if 1//defined(QMA7981_SWAP_XY)
	unsigned char reg_0x42 = 0;
#endif
	unsigned char reg_0x1e = 0;
	unsigned char reg_0x34 = 0;
	unsigned char yz_th_sel = 4;
	char y_th = -3; //-2;				// -16 ~ 15
	unsigned char x_th = 6;		// 0--7.5
	char z_th = 6;				// -8--7

#if 1//defined(QMA7981_SWAP_XY)	// swap xy
	if(layout%2)
	{
		qmaX981_read_reg(0x42, &reg_0x42, 1);
		reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
		qmaX981_write_reg(0x42, reg_0x42);
	}
#endif

	if((layout >=0) && (layout<=3))
	{
		z_th = 3;
		if((layout == 2)||(layout == 3))
			y_th = 3; 
		else if((layout == 0)||(layout == 1))	
			y_th = -3;
	}
	else if((layout >=4) && (layout<=7))
	{
		z_th = -3;
		
		if((layout == 6)||(layout == 7))
			y_th = 3; 
		else if((layout == 4)||(layout == 5))	
			y_th = -3;
	}

	// 0x34 YZ_TH_SEL[7:5]	Y_TH[4:0], default 0x9d  (YZ_TH_SEL   4   9.0 m/s2 | Y_TH  -3  -3 m/s2)
	//qmaX981_write_reg(0x34, 0x9d);	//|yz|>8 m/s2, y>-3 m/m2
	if((y_th&0x80))
	{
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= (y_th&0x0f)|0x10;
		qmaX981_write_reg(0x34, reg_0x34);
	}
	else
	{	
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= y_th;
		qmaX981_write_reg(0x34, reg_0x34);	//|yz|>8m/s2, y<3 m/m2
	}
	//Z_TH<7:4>: -8~7, LSB 1 (unit : m/s2)	X_TH<3:0>: 0~7.5, LSB 0.5 (unit : m/s2) 
	//qmaX981_write_reg(0x1e, 0x68);	//6 m/s2, 4 m/m2

#if 1
	qmaX981_write_reg(0x2a, (0x19|(0x03<<6)));			// 12m/s2 , 0.5m/s2
	qmaX981_write_reg(0x2b, (0x7c|(0x03>>2)));
#else
	qmaX981_write_reg(0x2a, (0x1e|(0x02<<6)));			// 15m/s2 , 0.4m/s2
	qmaX981_write_reg(0x2b, (0x7c|(0x02)));
#endif

	//qmaX981_read_reg(0x1e, &reg_0x1e, 1);
	if((z_th&0x80))
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= ((z_th<<4)|0x80);
		qmaX981_write_reg(0x1e, reg_0x1e);
	}
	else
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= (z_th<<4);
		qmaX981_write_reg(0x1e, reg_0x1e);
	}
}
#endif

#define STEP_W_TIME_L		300			// 300 ms
#define STEP_W_TIME_H		250			// 250*8 ms

uint8_t qmaX981_init(void)
{	
  	unsigned char DieId_H, DieId_L, WaferID;
	unsigned char reg_0x10 = 0;	
	unsigned char reg_0x11 = 0;	
#if defined(QMAX981_STEPCOUNTER)
	unsigned char reg_0x14 = 0;	
	unsigned char reg_0x15 = 0;	
#endif
	unsigned char reg_0x16 = 0;
	unsigned char reg_0x18 = 0;
	unsigned char reg_0x19 = 0;
	unsigned char reg_0x1a = 0;
#if defined(QMA7981_ANY_MOTION)||defined(QMA7981_NO_MOTION)
	unsigned char reg_0x2c = 0;
#endif

	qmaX981_layout = 0;
	qmaX981_set_layout(qmaX981_layout);

	qmaX981_read_reg(0x00, &acc_chip_id, 1);
	QST_PRINTF("qmaX981_init chipid=%d \n", acc_chip_id);
	if((acc_chip_id>=0xb0)&&(acc_chip_id<=0xb9))
	{
		qmaX981_write_reg(0x0f, 0x02);		// lsb 128
		qmaX981_set_range(ACC_RANGE_4G);
		qmaX981_write_reg(0x10, 0x05);
		qmaX981_write_reg(0x11, 0x80);
#if defined(QMAX981_STEPCOUNTER)
		qmaX981_write_reg(0x0f, 0x04);
		qmaX981_write_reg(0x11, 0x80);
		qmaX981_write_reg(0x10, 0x2a);
		qmaX981_write_reg(0x12, 0x8f);
		qmaX981_write_reg(0x13, 0x10);
		qmaX981_write_reg(0x14, 0x14);
		qmaX981_write_reg(0x15, 0x10);
		
		qmaX981_write_reg(0x16, 0x0c);
#endif
	}
	else if((acc_chip_id>=0xe0)&&(acc_chip_id<=0xe7))
	{
	  	qmaX981_read_reg(0x47, &DieId_L, 1);
	  	qmaX981_read_reg(0x48, &DieId_H, 1);
	  	qmaX981_read_reg(0x5a, &WaferID, 1);
		qst_printf("DieId_L:%d  DieId_H:%d  WaferID:%d \n", DieId_L, DieId_H, WaferID&0x1f);
	
		qmaX981_write_reg(0x36, 0xb6);
		delay_ms(50);
		qmaX981_write_reg(0x36, 0x00);
		qmaX981_set_range(ACC_RANGE_4G);	// 0.488 mg
//		qmaX981_write_reg(0x4a, 0x08);	//Force I2C I2C interface
		//0xe0	[MCLK/7695]
		//0xe1	[MCLK/3855]
		//0xe2	[MCLK/1935]
		//0xe3	[MCLK/975]
		reg_0x10 = 0xe1;
		qmaX981_write_reg(0x10, reg_0x10);
		reg_0x11 = 0x80;
		qmaX981_write_reg(0x11, reg_0x11);
		//[0x80:500K]  [0x81:333K]  [0x82:200K]  [0x83:100K]  
		//[0x84:50K]  [0x85:25K]  [0x86:12.5K]
		qmaX981_write_reg(0x5f, 0x80);
		qmaX981_write_reg(0x5f, 0x00);
// read reg
		qmaX981_read_reg(0x16, &reg_0x16, 1);
		qmaX981_read_reg(0x18, &reg_0x18, 1);
		qmaX981_read_reg(0x19, &reg_0x19, 1);
		qmaX981_read_reg(0x1a, &reg_0x1a, 1);
		QST_PRINTF("read reg[%d %d %d %d] \n", reg_0x16, reg_0x18, reg_0x19, reg_0x1a);
// read reg
		
#if defined(QMAX981_STEPCOUNTER)
		if(reg_0x11 == 0x80)		// 500K
		{
			reg_0x10 = 0xe1;
			qmaX981_write_reg(0x10, reg_0x10);

			reg_0x14 = (((STEP_W_TIME_L*100)/771)+1);		// odr 129.7hz, 7.71ms
			reg_0x15 = (((STEP_W_TIME_H*100)/771)+1);
			if(reg_0x10 == 0xe0)		// odr 65hz
			{
				reg_0x14 = (reg_0x14>>1);
				reg_0x15 = (reg_0x15>>1);
			}
			else if(reg_0x10 == 0xe5)	// odr 32.5hz
			{
				reg_0x14 = (reg_0x14>>2);
				reg_0x15 = (reg_0x15>>2);
			}
		}
		else if(reg_0x11 == 0x81)	// 333K
		{		
			reg_0x10 = 0xe1;
			qmaX981_write_reg(0x10, reg_0x10);

			reg_0x14 = (((STEP_W_TIME_L*100)/581)+1); 	// odr 172.0930233 hz, 5.81ms
			reg_0x15 = (((STEP_W_TIME_H*100)/581)+1);
			if(reg_0x10 == 0xe1)	// 86.38132296 hz
			{			
				reg_0x14 = (reg_0x14>>1);
				reg_0x15 = (reg_0x15>>1);
			}
			else if(reg_0x10 == 0xe0)		// 43.2748538
			{
				reg_0x14 = (reg_0x14>>2);
				reg_0x15 = (reg_0x15>>2);
			}
		}
		else if(reg_0x11 == 0x82)		// 200K
		{
			reg_0x10 = 0xe2;
			qmaX981_write_reg(0x10, reg_0x10);

			reg_0x14 = (((STEP_W_TIME_L*100)/967)+1); 	// 103.3591731 hz, 9.675 ms
			reg_0x15 = (((STEP_W_TIME_H*100)/967)+1);
			if(reg_0x10 == 0xe1)
			{			
				reg_0x14 = (reg_0x14>>1);		// 51.88067445 hz
				reg_0x15 = (reg_0x15>>1);
			}
			else if(reg_0x10 == 0xe3)
			{				
				reg_0x14 = (reg_0x14<<1);		// 205.1282051 hz				
				reg_0x15 = (reg_0x15<<1);
			}
		}		
		else if(reg_0x11 == 0x83)		// 100K
		{
			reg_0x10 = 0xe3;
			qmaX981_write_reg(0x10, reg_0x10);

			reg_0x14 = (((STEP_W_TIME_L*100)/975)+1); 	// 102.5641026 hz, 9.75 ms
			reg_0x15 = (((STEP_W_TIME_H*100)/975)+1);
			if(reg_0x10 == 0xe2)
			{
				reg_0x14 = (reg_0x14>>1);		// 51.67958656 hz
				reg_0x15 = (reg_0x15>>1);
			}
		}

		QST_PRINTF("0x14[%d] 0x15[%d] \n", reg_0x14, reg_0x15);
		qmaX981_write_reg(0x12, 0x94);
		qmaX981_write_reg(0x13, 0x80);		// clear step
		qmaX981_write_reg(0x13, 0x01);		// 0x7f(1/16) 0x00(1/8)
		qmaX981_write_reg(0x14, reg_0x14);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_write_reg(0x15, reg_0x15);		// STEP_TIME_UP<7:0>*8*(1/ODR) 

		//qmaX981_write_reg(0x1f, 0x09);		// 0 step
		//qmaX981_write_reg(0x1f, 0x29);		// 4 step
		//qmaX981_write_reg(0x1f, 0x49);		// 8 step
		//qmaX981_write_reg(0x1f, 0x69);		// 12 step
		//qmaX981_write_reg(0x1f, 0x89);		// 16 step
		qmaX981_write_reg(0x1f, 0xa9);		// 24 step
		//qmaX981_write_reg(0x1f, 0xc9);		// 32 step
		//qmaX981_write_reg(0x1f, 0xe9);		// 40 step

		// step int
		#if defined(QMA7981_STEP_INT)
		reg_0x16 |= 0x08;
		reg_0x19 |= 0x08;
		qmaX981_write_reg(0x16, reg_0x16);
		qmaX981_write_reg(0x19, reg_0x19);
		#endif
		#if defined(QMA7981_SIGNIFICANT_STEP)
		qmaX981_write_reg(0x1d, 0x26);		//every 30 step
		reg_0x16 |= 0x40;
		reg_0x19 |= 0x40;
		qmaX981_write_reg(0x16, reg_0x16);
		qmaX981_write_reg(0x19, reg_0x19);
		#endif
#endif

//RANGE<3:0> Acceleration range Resolution
//0001 2g 244ug/LSB
//0010 4g 488ug/LSB
//0100 8g 977ug/LSB
//1000 16g 1.95mg/LSB
//1111 32g 3.91mg/LSB
//Others 2g 244ug/LSB

//0x2c
//Duration = (NO_MOT_DUR<3:0> + 1) * 1s, if NO_MOT_DUR<5:4> =b00 
//Duration = (NO_MOT_DUR<3:0> + 4) * 5s, if NO_MOT_DUR<5:4> =b01 
//Duration = (NO_MOT_DUR<3:0> + 10) * 10s, if NO_MOT_DUR<5:4> =b1x 
//ANY_MOT_DUR<1:0>: any motion interrupt will be triggered when slope > ANY_MOT_TH for (ANY_MOT_DUR<1:0> + 1) samples 

//0x2e ANY MOTION MOT_CONF2
//TH= ANY_MOT_TH<7:0> * 16 * LSB 

#if defined(QMA7981_ANY_MOTION)
		reg_0x18 |= 0x04;	//0x07;
		reg_0x1a |= 0x01;
		reg_0x2c |= 0x00;	//0x01;		// 0x00
		
		qmaX981_write_reg(0x18, reg_0x18);
		qmaX981_write_reg(0x1a, reg_0x1a);
		qmaX981_write_reg(0x2c, reg_0x2c);
		//qmaX981_write_reg(0x2e, 0x18);		// 0.488*16*20 = 156mg
		//qmaX981_write_reg(0x2e, 0xc0);		// 0.488*16*196= 1.5g
		qmaX981_write_reg(0x2e, 0x80);		// 0.488*16*128 = 1g
		//qmaX981_write_reg(0x2e, 0x60);		// 0.488*16*96 = 750mg
		//qmaX981_write_reg(0x2e, 0x40);		// 0.488*16*64 = 500mg
		//qmaX981_write_reg(0x2e, 0x20);		// 0.488*16*32 = 250mg
#if defined(QMA7981_SIGNIFICANT_MOTION)
		//qmaX981_write_reg(0x2f, 0x0c|0x01);
		qmaX981_write_reg(0x2f, 0x01);
		reg_0x19 |= 0x01;
		qmaX981_write_reg(0x19, reg_0x19);
#endif
#endif

#if defined(QMA7981_NO_MOTION)
		reg_0x18 |= 0xe0;
		reg_0x1a |= 0x80;
		reg_0x2c |= 0x00;	//1s			//0x24;

		qmaX981_write_reg(0x18, reg_0x18);
		qmaX981_write_reg(0x1a, reg_0x1a);
		qmaX981_write_reg(0x2c, reg_0x2c);
		qmaX981_write_reg(0x2d, 0x14);
#endif

#if defined(QMA7981_HAND_UP_DOWN)
		qma7981_set_hand_up_down(qmaX981_layout);
		// hand up
		reg_0x16 |= 0x02;
		reg_0x19 |= 0x02;
		qmaX981_write_reg(0x16, reg_0x16);
		qmaX981_write_reg(0x19, reg_0x19);		
		// hand up
		// hand down
		reg_0x16 |= 0x04;
		reg_0x19 |= 0x04;
		qmaX981_write_reg(0x16, reg_0x16);
		qmaX981_write_reg(0x19, reg_0x19);
		// hand down	
#endif

#if defined(QMA7981_DATA_READY)
		reg_0x1a |= 0x10;
		qmaX981_write_reg(0x17, 0x10);
		qmaX981_write_reg(0x1a, reg_0x1a);
#endif

#if defined(QMA7981_INT_LATCH)
		qmaX981_write_reg(0x21, 0x1f);	// default 0x1c, step latch mode
#endif
// int default level set
		qmaX981_write_reg(0x20, 0x00);
// int default level set
	}
	else if(acc_chip_id == 0xe8)
	{	
		qmaX981_write_reg(0x36, 0xb6);
		delay_ms(50);
		qmaX981_write_reg(0x36, 0x00);
		qmaX981_set_range(ACC_RANGE_4G);	// 0.488 mg
		//0xe0	[65 hz		15.48 ms]
		//0xe1	[129 hz 	7.74 ms]
		//0xe2	[258 hz 	3.87 ms]
		reg_0x10 = 0xe0;
		qmaX981_write_reg(0x10, reg_0x10);

//		qmaX981_write_reg(0x4a, 0x08);	//Force I2C I2C interface
		qmaX981_write_reg(0x11, 0x80);
		qmaX981_write_reg(0x5f, 0x80);
		qmaX981_write_reg(0x5f, 0x00);
		qmaX981_write_reg(0x20, 0x05);
#if defined(QMA7981_6100_FIFO)
		qmaX981_write_reg(0x31, 0x20);
		qmaX981_write_reg(0x3E, 0x40);
		qmaX981_write_reg(0x17, 0x20);
		qmaX981_write_reg(0x1a, 0x20);
		qmaX981_write_reg(0x20, 0x05);
	#if defined(USE_SPI)
		//qmaX981_write_reg(0x21, 0x21);
	#else
		//qmaX981_write_reg(0x21, 0x01);
	#endif
#endif
	}
	else
	{
		acc_chip_id = 0;
	}

	if(acc_chip_id == 0xe7)
	{
	}

	QST_PRINTF("qmaX981_init done\n");

	return acc_chip_id;
}
#endif


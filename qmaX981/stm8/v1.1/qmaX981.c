
#include "stm8s.h"
#include "qst_i2c.h"
#if defined(USE_SPI)
#include "qst_spi.h"
#endif
#include "bsp_timer.h"

#if defined(QST_CONFIG_QMAX981)

//#define QMA7981_DOUBLE_TRIPLE_CLICK
#define QMA7981_ABNORMAL_SHAKE_CHECK

// for qma7981
#define QMA7981_ANY_MOTION
//#define QMA7981_NO_MOTION
//#define QMA7981_SIGNIFICANT_MOTION
//#define QMA7981_INT_LATCH
//#define QMA7981_HAND_UP_DOWN
//#define QMA7981_6100_FIFO
// 7981

#if defined(QMA7981_DOUBLE_TRIPLE_CLICK)
typedef struct
{
	unsigned char check_click;
	unsigned short click_num;
	unsigned short static_num;
	unsigned short t_msec_1;			// check_click timer
	unsigned short t_msec_2;			// check static timer
	unsigned short t_msec_out;			// timeout
}qst_click_check;

static unsigned int acc_data_curr[3];
static unsigned int acc_data[3];
static qst_click_check g_click;
#endif

#if defined(QMA7981_ABNORMAL_SHAKE_CHECK)
typedef struct
{
	unsigned char check_shake;
	unsigned short shake_num;
	unsigned short t_msec_1;
	unsigned short t_msec_out;			// timeout
}qst_shake_check;

static qst_shake_check g_shake;
#endif

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
static unsigned char acc_chip_id=0;
static unsigned short acc_lsb_div = 0;
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

void qmaX981_read_xyz(void)
{
	uint8_t reg_data[6];
	int16_t raw[3];

	qmaX981_read_reg(0x01, reg_data, 6);
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

	qst_printf("acc	%f	%f	%f\n",(float)raw[0]*9.807/acc_lsb_div,(float)raw[1]*9.807/acc_lsb_div,(float)raw[2]*9.807/acc_lsb_div);
}


#if defined(QMAX981_STEPCOUNTER)
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
	qst_printf("fifo level:%d\n", reg_0x0e);
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
				qst_printf("FIFO level[%d]	DATA:	%d	%d	%d \n", 
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
//	qst_printf("irq [%d %d %d]!!!", r_data[0],r_data[1],r_data[2]);
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

#if defined(QMA7981_DOUBLE_TRIPLE_CLICK)
void click_timer_cbk_out(int timerId)
{
	bsp_stop_timer(timerId);
	bsp_stop_timer(1);
	g_click.check_click = 1;
	g_click.static_num = 0;
	g_click.click_num = 0;
			
	qst_printf("qmaX981_timer_cbk_out \n");	
}

void click_timer_read_acc(int timerId)
{
	int data1, data2, ret;

	ret = qmaX981_read_acc(acc_data_curr);
	if(ret)
	{
		data1 = QMAX981_ABS(acc_data_curr[0])+QMAX981_ABS(acc_data_curr[1])+QMAX981_ABS(acc_data_curr[2]);
		data2 = QMAX981_ABS(acc_data[0])+QMAX981_ABS(acc_data[1])+QMAX981_ABS(acc_data[2]);
		//qst_printf("acc_diff = %d \n", QMAX981_ABS(data1-data2));
		if(QMAX981_ABS(data1-data2) < 500)
		{
			g_click.static_num++;
		}
		acc_data[0] = acc_data_curr[0];
		acc_data[1] = acc_data_curr[1];
		acc_data[2] = acc_data_curr[2];
	}
}

void click_timer_cbk_1(int timerId)
{
	bsp_stop_timer(timerId);
	g_click.check_click = 1;


	if(g_click.t_msec_2 > 0)
	{
		if(g_click.click_num > 1)
		{		
			qst_printf(" static_num=%d \n", g_click.static_num);
			if(g_click.static_num <= 9)
			{
				g_click.click_num = 0;
			}
		}
		g_click.static_num = 0;
		bsp_start_timer(1, g_click.t_msec_2, click_timer_read_acc);
	}
	bsp_start_timer(0, g_click.t_msec_out, click_timer_cbk_out);	
	if(g_click.click_num == 3)
	{
		qst_printf(" click detect!!! \n");
	}
}
#endif

#if defined(QMA7981_ABNORMAL_SHAKE_CHECK)
void shake_timer_cbk_out(int timerId)
{
	qst_printf("qmaX981_timer_cbk_out \n");	
	bsp_stop_timer(timerId);
	g_shake.check_shake = 1;
	g_shake.shake_num = 0;		
	qst_show_info(0);
}

void shake_timer_cbk_1(int timerId)
{
	//qst_printf("shake_timer_cbk_1 \n");
	bsp_stop_timer(timerId);
	g_shake.check_shake = 1;	
	bsp_start_timer(0, g_shake.t_msec_out, shake_timer_cbk_out);
	if(g_shake.shake_num >= 9)
	{
		qst_printf("abnormal shake, Warning!!! \n");
		qst_show_info(1);
	}
}
#endif

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
//	qst_printf("irq [%d %d %d]!!!", r_data[0],r_data[1],r_data[2]);
	if(r_data[0] & 0xF)
	{	
#if 0//defined(QMA7981_NO_MOTION)
		qmaX981_readreg(0x1a,&reg_0x1a,1);
		reg_0x1a |= 0x80;			// enable nomotion
		//reg_0x1a &= 0xfe; 		// disable anymotion
		qmaX981_writereg(0x1a, reg_0x1a);
#endif
#if defined(QMA7981_DOUBLE_TRIPLE_CLICK)
		if(g_click.check_click)
		{
			bsp_stop_timer(0);
			g_click.check_click = 0;
			g_click.static_num = 0; 		
			g_click.click_num++;
			bsp_start_timer(0, g_click.t_msec_1, click_timer_cbk_1);
			qst_printf(" any motion! %d\n", g_click.click_num);
		}		
#endif

#if defined(QMA7981_ABNORMAL_SHAKE_CHECK)
		if(g_shake.check_shake)
		{
			bsp_stop_timer(0);
			g_shake.check_shake = 0;		
			g_shake.shake_num++;
			bsp_start_timer(0, g_shake.t_msec_1, shake_timer_cbk_1);
			qst_printf(" any motion! %d\n", g_shake.shake_num);
		}
#endif
		int_type = 1;
//		qst_printf(" any motion!\n");
	}
	else if(r_data[0] & 0x80)
	{	
#if 0//defined(QMA7981_NO_MOTION)
		qmaX981_read_reg(0x1a,&reg_0x1a,1);
		reg_0x1a &= 0x7f;
		qmaX981_write_reg(0x1a, reg_0x1a);		// disable nomotion
#endif
		int_type = 2;
		qst_printf(" no motion!\n");
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
		qst_printf(" significant motion!\n");
	}
	else if(r_data[1] & 0x40)
	{	
		int_type = 4;
		qst_printf("  significant step int!\n");
	}
	else if(r_data[1] & 0x08)
	{
		int_type = 5;
		qst_printf(" step int!\n");
	}
#if defined(QMA7981_HAND_UP_DOWN)
	else if(r_data[1] & 0x02)
	{
		int_type = 6;
		qst_printf(" hand raise!\n");
	}
	else if(r_data[1] & 0x04)
	{
		int_type = 7;
		qst_printf(" hand down!\n");
	}
#endif

	return int_type;
}


uint8_t qmaX981_init(void)
{	
	unsigned char reg_0x10 = 0;	
	unsigned char reg_0x16 = 0;
	unsigned char reg_0x18 = 0;
	unsigned char reg_0x19 = 0;
	unsigned char reg_0x1a = 0;
#if defined(QMA7981_ANY_MOTION)||defined(QMA7981_NO_MOTION)
	unsigned char reg_0x2c = 0;
#endif
#if defined(QMA7981_HAND_UP_DOWN)
	unsigned char reg_0x42 = 0;
#endif

	qmaX981_read_reg(0x00, &acc_chip_id, 1);
	qst_printf("qmaX981_init chipid=%d \n", acc_chip_id);
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
		qmaX981_write_reg(0x36, 0xb6);
		delay_ms(50);
		qmaX981_write_reg(0x36, 0x00);
		qmaX981_set_range(ACC_RANGE_4G);	// 0.488 mg
		//0xe0	[65 hz		15.48 ms]
		//0xe1	[129 hz		7.74 ms]
		//0xe2	[258 hz		3.87 ms]
		reg_0x10 = 0xe1;
		qmaX981_write_reg(0x10, reg_0x10);

//		qmaX981_write_reg(0x4a, 0x08);	//Force I2C I2C interface
		qmaX981_write_reg(0x11, 0x80);
		qmaX981_write_reg(0x5f, 0x80);
		qmaX981_write_reg(0x5f, 0x00);
		delay_ms(20);
// read reg
		qmaX981_read_reg(0x16, &reg_0x16, 1);
		qmaX981_read_reg(0x18, &reg_0x18, 1);
		qmaX981_read_reg(0x19, &reg_0x19, 1);
		qmaX981_read_reg(0x1a, &reg_0x1a, 1);
		
		qst_printf("read reg[%d %d %d %d] \n", reg_0x16, reg_0x18, reg_0x19, reg_0x1a);
// read reg
		
#if defined(QMAX981_STEPCOUNTER)
		if(reg_0x10 == 0xe0)
		{
			// ODR: 65hz 15.48 ms
			qmaX981_write_reg(0x12, 0x94);
			qmaX981_write_reg(0x13, 0x80);		// clear step
			qmaX981_write_reg(0x13, 0x00);		// 
			qmaX981_write_reg(0x14, 0x12);		// STEP_TIME_LOW<7:0>*(1/ODR) 
			qmaX981_write_reg(0x15, 0x10);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
		}
		else if(reg_0x10 == 0xe1)
		{
			// ODR: 130hz 7.74 ms
			qmaX981_write_reg(0x12, 0x94);
			qmaX981_write_reg(0x13, 0x80);		// clear step
			qmaX981_write_reg(0x13, 0x00);		// 
			qmaX981_write_reg(0x14, 0x24);		// STEP_TIME_LOW<7:0>*(1/ODR) 
			qmaX981_write_reg(0x15, 0x20);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
		}
		else if(reg_0x10 == 0xe2)
		{
			// ODR: 258Hz 3.87 ms
			qmaX981_write_reg(0x12, 0x94);
			qmaX981_write_reg(0x13, 0x80);		// clear step
			qmaX981_write_reg(0x13, 0x00);		// 
			qmaX981_write_reg(0x14, 0x48);		// STEP_TIME_LOW<7:0>*(1/ODR) 
			qmaX981_write_reg(0x15, 0x40);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
		}

		//qmaX981_write_reg(0x1f, 0x00);

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
		reg_0x18 |= 0x07;
		reg_0x1a |= 0x01;
		reg_0x2c |= 0x00;	//0x01;		// 0x00
		
		qmaX981_write_reg(0x18, reg_0x18);
		qmaX981_write_reg(0x1a, reg_0x1a);
		qmaX981_write_reg(0x2c, reg_0x2c);
		//qmaX981_write_reg(0x2e, 0x18);		// 0.488*16*20 = 156mg
		//qmaX981_write_reg(0x2e, 0xc0);		// 0.488*16*128 = 1.5g
		//qmaX981_write_reg(0x2e, 0x80);		// 0.488*16*128 = 1g
		//qmaX981_write_reg(0x2e, 0x60);		// 0.488*16*128 = 750mg
		qmaX981_write_reg(0x2e, 0x40);		// 0.488*16*128 = 500mg
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
		reg_0x16 |= 0x02;
		reg_0x19 |= 0x02;
				
		qmaX981_write_reg(0x16, reg_0x16);
		qmaX981_write_reg(0x19, reg_0x19);
		// hand down
		reg_0x16 |= 0x04;
		reg_0x19 |= 0x04;
		qmaX981_write_reg(0x16, reg_0x16);
		qmaX981_write_reg(0x19, reg_0x19);
		// hand down	
		qmaX981_read_reg(0x42, &reg_0x42, 1);
	#if 1	// swap xy
		reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
		qmaX981_write_reg(0x42, reg_0x42);
	#endif
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
		delay_ms(20);
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
#if defined(QMA7981_DOUBLE_TRIPLE_CLICK)
		//memset(&g_click, 0, sizeof(g_click));
		g_click.check_click = 1;
		g_click.click_num = 0;
		g_click.static_num = 0;
		g_click.t_msec_1 = 200;
		g_click.t_msec_2 = 8;
		g_click.t_msec_out = 350;
#endif
#if defined(QMA7981_ABNORMAL_SHAKE_CHECK)
		g_shake.check_shake = 1;
		g_shake.shake_num = 0;
		g_shake.t_msec_1 = 200;
		g_shake.t_msec_out = 500;
#endif
	}

	qst_printf("qmaX981_init done\n");

	return acc_chip_id;
}
#endif


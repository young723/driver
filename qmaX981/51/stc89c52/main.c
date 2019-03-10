#include <REGX52.H>
//#include <AT89X52.H>
#include <stdio.h>
#include <string.h>
#include "qst_sw_i2c.h"
#include "qst_common.h"

/**********************51???????************************
*  ??:Keil U4 + STC89C52
*  ??:UART??????? A
*  ??:??
*  ??:11.0592MHZ
******************************************************************/

//#define jingzhen     11059200UL                         /*??11.0592M??*/         
//#define botelv   9600UL                     /*??????9600*/

extern void uart_init(void);
extern void qst_log(const char *format, ...);
extern s32 qmaX981_init(void);
extern s32 qmaX981_read_acc(s32 *accData);
extern s32 qmaX981_read_raw(s32 *rawData);
extern u8 qmaX981_readreg(u8 reg_add,u8 *buf,u8 num);

unsigned char g_qmaX981_int_flag = 0;
void delay(unsigned char i)
{
        unsigned char j,k;
        for(j=i;j>0;j--)
                for(k=90;k>0;k--);
}


void Int0Init()
{
	//设置INT0
	IT0=1;//跳变沿出发方式1:下降沿0:低电平
	EX0=1;//打开INT1的中断允许。	
	EA=1;//打开总中断	
}

void Int1Init()
{
	//设置INT1
	IT1=1;//跳变沿出发方式（下降沿）
	EX1=1;//打开INT1的中断允许。	
	EA=1;//打开总中断	
}


int main()
{
	int_32 acc_data[3];
	unsigned char databuf;
	
	uart_init();
	Int0Init();
	Int1Init();
	qst_log("QST start! i2c_check=%d\n");

	qmaX981_init();
	while(1)
	{
		qmaX981_read_raw(acc_data);
		//qst_log("acc_data[%d	%d	%d]\n", acc_data[0], acc_data[1], acc_data[2]);
		if(g_qmaX981_int_flag == 1)
		{
			qmaX981_readreg(0x0a, &databuf, 1);
			qst_log("qmaX981_int status=%d \n", (u16)databuf);
			g_qmaX981_int_flag = 0;
		}
		delay(10000);
	}

	return(0);
}

void Int0_handle()	interrupt 0		//外部中断0的中断函数
{
	EX0=0;
	//delay(1000);	 //延时消抖
	if(P3_2==0)
	{
		P2_5 = ~P2_5;
		g_qmaX981_int_flag = 1;
	}
	EX0=1;
}

void Int1_handle()	interrupt 2		//外部中断1的中断函数
{
	EX1=0;
	//delay(1000);	 //延时消抖
	if(P3_3==0)
	{
		P2_4 = ~P2_4;
		g_qmaX981_int_flag = 1;
	}
	EX1=1;
}

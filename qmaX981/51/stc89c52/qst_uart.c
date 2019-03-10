#include <REGX52.H>
//#include <AT89X52.H>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "qst_sw_i2c.h"
#include "qst_common.h"

#define FSOC 24000000L    //晶振
#define BAUD  9600        //波特率

volatile uint_8 uart_sending;

void uart_init(void)                                //?????
{
	EA=0; //关闭总中断
	TMOD&=0x0F;				//清零T1的控制位
	TMOD|=0x20;    //定时器1，模式2工作模式
	SCON=0x50;     //设置为工作方式1
	TH1=0xf3;//256-(FSOC/(BAUD*12*16));//0xf3;//4800 bps
	TL1=0xf3;//256-(FSOC/(BAUD*12*16));//0xf3;//256-jingzhen/(botelv*12*16);
	PCON|=0x80;    //发送速率加倍
	ES=1;         //打开接收中断
	TR1=1;        //打开计数器
	REN=1;        //???? 
	EA=1;         //打开总中断
}

#if 0
void uart_send_byte(unsigned char d)
{
	SBUF=d;
	uart_sending=1;
	while(uart_sending);
}
#endif

void uart_send_string(unsigned char * pd)
{
	while((*pd)!='\0')
	{
		//uart_send_byte(*pd);
		SBUF=* pd;
		uart_sending=1;
		while(uart_sending);
		pd++;
	}
}

void qst_log(const char *format, ...)
{
	va_list arg_list;
	int_8 xdata log_buf[60];								// qst log buffer

	memset(log_buf, 0, sizeof(log_buf));
	va_start(arg_list, format);
	//vsnprintf(qst_log_str, sizeof(qst_log_str), format, arg_list);
	vsprintf(log_buf,format, arg_list);	
	//sprintf(log_buf,format, arg_list);
	va_end(arg_list);
	uart_send_string(log_buf);
}


/*******************************************************************************
* 函数名         : Usart() interrupt 4
* 函数功能		  : 串口通信中断函数
* 输入           : 无
* 输出         	 : 无
*******************************************************************************/
void uart(void) interrupt 4
{
 if(RI)    //接收终端标志
 {
  RI=0;   //清除接收中断标志位
 }
 else			// 发送中断标志
 {
  TI=0;			//清除发送完成标志位
  uart_sending=0;  //清除发送完成标志位
 }
}

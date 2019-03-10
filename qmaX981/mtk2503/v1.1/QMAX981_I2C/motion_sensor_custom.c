
#define _MOTION_SENSOR_CUSTOM_C_

#if defined(MOTION_SENSOR_SUPPORT)
#include "kal_release.h"
#include "custom_config.h"
#include "gpio_sw.h"
#include "drv_comm.h"
#include "eint.h"
#include "motion_sensor.h"
#include "motion_sensor_custom.h"
#include "motion_sensor_I2C.h"
#include "motion_sensor_hw_define.h"
#include "kal_trace.h"
#include "stack_config.h"
#include "us_timer.h"
#include <math.h>
#include "MMIDataType.h"
#include "TimerEvents.h"
#include "math.h"
#include "fs_gprot.h"

/*========================================================================================================
										D E B U G
========================================================================================================*/
#define QMAX981_DEBUG
#if defined(QMAX981_DEBUG)&&defined(__MTK_TARGET__)
#define QMAX981_INFO			kal_prompt_trace
#define QMAX981_ERROR			kal_prompt_trace
#else
#define QMAX981_INFO(fmt, arg...)			do {} while (0)
#define QMAX981_ERROR(fmt, arg...)			do {} while (0)
#endif

/*========================================================================================================
										E X T E R N 
========================================================================================================*/
/*========================================================================================================
										D E F I N E
========================================================================================================*/
//-------------------------------------
// I2C
//-------------------------------------
#define QMAX981_SLAVE_ADDR			0x24   // FAE: 这个芯片I2C 7位地址只有0x12 0x13 两种
#define QMAX981_SLAVE_ADDR_2		0x26   // FAE: 这个芯片I2C 7位地址只有0x12 0x13 两种
kal_uint32 MS_DELAY_TIME = 100;  // Digital I2C interface supporting both fast mode (400kHz) and normal mode (100kHz)

//#define QMAX981_USE_SW_IIC

/*========================================================================================================
										T Y P E D E F
========================================================================================================*/
typedef enum
{
	QMAX981_TYPE_6981,
	QMAX981_TYPE_7981,
	QMAX981_TYPE_MAX
}qmaX981_chip_type;

struct hwmsen_convert {
	short sign[4];
	short map[4];
};


struct qmaX981_data
{
	kal_uint8			chip_id;
	qmaX981_chip_type	chip_type;	
	kal_uint8			layout;
	kal_int16			lsb_1g;					
	kal_uint32			step;
#if defined(QMAX981_USE_INT1)
	kal_uint8			int1_no;
	kal_uint8			int1_level;
#endif
#if defined(QMAX981_USE_INT2)
	kal_uint8			int2_no;
#endif
	struct hwmsen_convert *cvt;
};


/*========================================================================================================
										V A R I A B L E S
========================================================================================================*/
#ifdef MS_DBG
kal_char qma6981_debug_buff[200];
#endif
static struct qmaX981_data g_qmaX981;

struct hwmsen_convert qmaX981_map[] = {
	{ { 1, 1, 1}, {0, 1, 2} },
	{ {-1, 1, 1}, {1, 0, 2} },
	{ {-1, -1, 1}, {0, 1, 2} },
	{ { 1, -1, 1}, {1, 0, 2} },

	{ {-1, 1, -1}, {0, 1, 2} },
	{ { 1, 1, -1}, {1, 0, 2} },
	{ { 1, -1, -1}, {0, 1, 2} },
	{ {-1, -1, -1}, {1, 0, 2} }

};

//#define QMAX981_USE_CALI

#if defined(QMAX981_USE_CALI)
#define QMAX981_CALI_FILE		L"Z:\\qmax981cali.conf"
#define QMAX981_LSB_1G			64			// mg
#define QMAX981_CALI_NUM		20    
static int qmax981_cali[3]={0, 0, 0};
static kal_char qmax981_cali_flag = 0;
static void qmax981_read_file(kal_uint16 * filename, kal_char *data, int len);
static void qmax981_write_file(kal_uint16 * filename, kal_char *data, int len);
#endif


#if defined(QMAX981_CHECK_ABNORMAL_DATA)
extern int qmaX981_check_abnormal_data(int data_in, int *data_out);
#endif
#if defined(QMAX981_STEP_COUNTER_USE_INT)
extern void qmaX981_step_debounce_reset(void);
extern int qmaX981_step_debounce_int_work(int data, unsigned char irq_level);
extern int qmaX981_step_debounce_read_data(int result);
#endif

/*========================================================================================================
										F U N C T I O N----static
========================================================================================================*/
static void SW_i2c_udelay(kal_uint32 delay)
{
    kal_uint32 ust = 0; //ust_get_current_time
    kal_uint32 count = 0;
    kal_uint32 break_count = 0;

    ust = ust_get_current_time();
    do{
        if(ust_get_current_time() != ust)
            count++;
        else
            break_count++;
    }while((count < delay) && (break_count < 0xFFFFFF));
}

static void gsensor_delay_ms(kal_uint16 delay)
{
    kal_uint16 i=0;

    for(i=0; i<delay; i++)
    {
        SW_i2c_udelay(6268);//SW_i2c_udelay(1000);
    }
}

#if defined(QMAX981_USE_SW_IIC)
#ifdef __CUST_NEW__
extern const char gpio_ms_i2c_data_pin;
extern const char gpio_ms_i2c_clk_pin;

#define MS_SCL	gpio_ms_i2c_clk_pin
#define MS_SDA	gpio_ms_i2c_data_pin
#else
#define MS_SCL	16
#define MS_SDA	17
#endif
#define QMAX981_SLAVEADDR_W	0x24		// 0x26
#define QMAX981_SLAVEADDR_R	0x25		// ox27

#define MS_CLK_PIN_GPIO_MODE		GPIO_ModeSetup(MS_SCL,0)
#define	MS_DATA_PIN_GPIO_MODE		GPIO_ModeSetup(MS_SDA,0)
#define MS_I2C_CLK_OUTPUT			GPIO_InitIO(OUTPUT,MS_SCL)
#define MS_I2C_DATA_OUTPUT			GPIO_InitIO(OUTPUT,MS_SDA)
#define MS_I2C_DATA_INPUT		   	GPIO_InitIO(INPUT,MS_SDA)
#define MS_I2C_CLK_HIGH				GPIO_WriteIO(1,MS_SCL)
#define MS_I2C_CLK_LOW				GPIO_WriteIO(0,MS_SCL)
#define MS_I2C_DATA_HIGH			GPIO_WriteIO(1,MS_SDA)
#define MS_I2C_DATA_LOW				GPIO_WriteIO(0,MS_SDA)
#define MS_I2C_GET_BIT				GPIO_ReadIO(MS_SDA)

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
static kal_uint8 ms_ReadByteAck(void)
{
	kal_int8 i;
	kal_uint8 data;

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
static kal_uint8 ms_ReadByteNAck(void)
{
	kal_int8 i;
	kal_uint8 data;

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
static void ms_SendByte(kal_uint8 sData) 
{
	kal_int8 i;
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
}
/******************************************
	software I2C check ack bit
*******************************************/
static kal_bool ms_ChkAck(void)//
{
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
		
		return KAL_FALSE;
	}
	else					//Ack
	{
		SW_i2c_udelay(10);	//5
		MS_I2C_CLK_LOW;
		SW_i2c_udelay(10);	//5
		MS_I2C_DATA_OUTPUT;
		MS_I2C_DATA_LOW;

		return KAL_TRUE;
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

/******************************************
	QMA6981 ms delay function
		uint: ms
*******************************************/
/*
void qmaX981_DelayMS(kal_uint16 delay)
{
	kal_uint16 i=0;

	for(i=0; i<delay; i++)
	{
		SW_i2c_udelay(1000);
	}
}
*/


static kal_bool qmaX981_ReadBytes(kal_uint8* Data, kal_uint8 RegAddr)
{
	SW_i2c_start();						//start bit
	ms_SendByte(QMAX981_SLAVEADDR_W);		//slave address|write bit
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		SW_i2c_stop();
		return KAL_FALSE;
	}
		
	ms_SendByte(RegAddr);				//send RegAddr
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_Restart();						//restart bit

	ms_SendByte(QMAX981_SLAVEADDR_R);		//slave address|read bit
	if(KAL_FALSE == ms_ChkAck())
	{
		SW_i2c_stop();
		return KAL_FALSE;
	}

	*Data = ms_ReadByteNAck();

	SW_i2c_stop();						//stop bit

	//TO_DO: add debug code to display the data received

	return KAL_TRUE;
	
}


static kal_bool qmaX981_ConReadBytes(kal_uint8* Data, kal_uint8 RegAddr, kal_uint32 Length)
{
	kal_uint8* Data_ptr;
	kal_uint16 i;

	Data_ptr = Data;
	
	SW_i2c_start();						//start bit
	ms_SendByte(QMAX981_SLAVEADDR_W);		//slave address|write bit
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send write id		
		SW_i2c_stop();
		return KAL_FALSE;
	}
		
	ms_SendByte(RegAddr);				//send RegAddr
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send RegAddr		
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_Restart();						//restart bit

	ms_SendByte(QMAX981_SLAVEADDR_R);		//slave address|read bit
	if(KAL_FALSE == ms_ChkAck())
	{
		//TO_DO: display ack check fail when send read id		
		SW_i2c_stop();
		return KAL_FALSE;
	}

	for(i=Length; i>1; i--)
	{
		*Data_ptr = ms_ReadByteAck();	//read byte with ack
		Data_ptr++;
	}
	
	*Data_ptr = ms_ReadByteNAck();		//read byte with non-ack to stop reading

	SW_i2c_stop();						//stop bit

	//TO_DO: add debug code to display the data received

	return KAL_TRUE;
}


static kal_bool QMA6981_WriteBytes(kal_uint8 RegAddr, kal_uint8 Data)
{
	SW_i2c_start();						//start bit

	ms_SendByte(QMAX981_SLAVEADDR_W);		//slave address|write bit
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send write id
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_SendByte(RegAddr);				//send RegAddr
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send RegAddr
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_SendByte(Data);					//send parameter
	if(KAL_FALSE == ms_ChkAck())
	{
		//TO_DO: display check ack fail when send data
		SW_i2c_stop();
		return KAL_FALSE;
	}

	SW_i2c_stop();						//stop bit

	return KAL_TRUE;
}

#endif

//======================================
// [G-sensor]: i2c 读写函数
//======================================
kal_bool gsensor_i2c_write_byte(kal_uint8 ucBufferIndex, kal_uint8 pucData)
{
#if defined(QMAX981_USE_SW_IIC)
	return QMA6981_WriteBytes(ucBufferIndex, pucData);
#else
    return ms_i2c_send(QMAX981_SLAVE_ADDR, ucBufferIndex, &pucData, 1);
#endif
}

//#define gsensor_i2c_read_bytes(reg_no,buffer_name,length) ms_i2c_receive(QMAX981_SLAVE_ADDR, reg_no, buffer_name, length)

kal_bool gsensor_i2c_read_bytes(kal_uint8 reg_no, kal_uint8* buffer_name, kal_uint32 length)
{
#if defined(QMAX981_USE_SW_IIC)
	return qmaX981_ConReadBytes(buffer_name, reg_no, length);
#else
	return ms_i2c_receive(QMAX981_SLAVE_ADDR, reg_no, buffer_name, length);
#endif
}

/*========================================================================================================
										F U N C T I O N----public
========================================================================================================*/



#if defined(QMAX981_USE_INT1)
void qmaX981_sensor_eint1_hisr(void)
{
#if defined(QMAX981_STEP_COUNTER_USE_INT)
	kal_bool ret;
	unsigned char data[2];
	int result, i;

	for(i=0; i<3; i++)
	{
		ret = gsensor_i2c_read_bytes(QMAX981_STEP_CNT_L, data, 2);
		if(ret)
		{
			break;
		}
	}
	result = (data[1]<<8)|data[0];
	if(ret == KAL_FALSE)
	{
		QMAX981_ERROR(MOD_MATV, "qmaX981_sensor_eint1_hisr read step error!!");
		return;
	}

	if(g_qmaX981.int1_level == 0)
	{
		g_qmaX981.int1_level = 1;
		qmaX981_step_debounce_int_work(result, 1);
		EINT_Set_Polarity(g_qmaX981.int1_no,KAL_TRUE); 
	}
	else
	{
		g_qmaX981.int1_level = 0;
		qmaX981_step_debounce_int_work(result, 0);
		EINT_Set_Polarity(g_qmaX981.int1_no,KAL_FALSE); 
	}
#else
	kal_bool bResult=KAL_FALSE;	
	kal_uint8 reg_value;	

    bResult=gsensor_i2c_read_bytes(0x0a, &reg_value, 1);
	if(KAL_TRUE == bResult)
	{
		QMAX981_INFO(MOD_MATV, "INT1 status reg [%x] \n", reg_value);
		// to do...
	}
	else
	{
		QMAX981_INFO(MOD_MATV, "INT1 status reg read error! \n");
	}
#endif
}

static void qmaX981_setup_int1(void)
{
	g_qmaX981.int1_no = custom_eint_get_channel(motion_senosr_eint_chann);

	//EINT_SW_Debounce_Modify(g_qmaX981.int1_no, 10);
	//EINT_Mask(g_qmaX981.int1_no);
#if defined(QMAX981_STEP_COUNTER_USE_INT)
	qmaX981_step_debounce_reset();
#endif
	//EINT_SW_Debounce_Modify(g_qmaX981.int1_no, 10);
	EINT_Set_Sensitivity(g_qmaX981.int1_no, EDGE_SENSITIVE);
	EINT_Registration(g_qmaX981.int1_no, KAL_FALSE, KAL_FALSE, qmaX981_sensor_eint1_hisr, KAL_TRUE);
}
#endif


#if defined(QMAX981_USE_INT2)
void qmaX981_sensor_eint2_hisr(void)
{
	kal_bool bResult=KAL_FALSE;	
	kal_uint8 reg_value;	

    bResult=gsensor_i2c_read_bytes(0x0a, &reg_value, 1);
	if(KAL_TRUE == bResult)
	{
		QMAX981_INFO(MOD_MATV, "INT2 status reg [%x] \n", reg_value);
		// to do...
	}
	else
	{
		QMAX981_INFO(MOD_MATV, "INT2 status reg read error! \n");
	}

}

static void qmaX981_setup_int2(void)
{
	g_qmaX981.int2_no = 3;		// customer modify

	EINT_Set_Sensitivity(g_qmaX981.int2_no, EDGE_SENSITIVE);
	EINT_Registration(g_qmaX981.int2_no, KAL_FALSE, KAL_FALSE, qmaX981_sensor_eint2_hisr, KAL_TRUE);
}
#endif


void qmaX981_custom_get_data(kal_uint16 *x_adc, kal_uint16 *y_adc, kal_uint16 *z_adc)
{
	kal_bool bResult=KAL_FALSE;	
	kal_uint8 databuf[6] = {0};	
	kal_int16 data_raw[3]= {0};
	kal_int16 data_acc[3]= {0};
	kal_int16	i;
	
	bResult = gsensor_i2c_read_bytes(QMAX981_XOUTL, databuf, 6);

	if(KAL_FALSE == bResult)
	{
		QMAX981_ERROR(MOD_MATV, "[E]----[%s]: get data fail!\r\n", __func__);
		return;
	}
	else
	{
		//QMAX981_INFO(MOD_MATV, "[I]----[%s]: XL[REG_%x]=%x, XH[REG_%x]=%x.\r\n", __func__, QMAX981_XOUTL, databuf[0], QMAX981_XOUTH, databuf[1]);
		//QMAX981_INFO(MOD_MATV, "[I]----[%s]: YL[REG_%x]=%x, YH[REG_%x]=%x.\r\n", __func__, QMAX981_YOUTL, databuf[2], QMAX981_YOUTH, databuf[3]);
		//QMAX981_INFO(MOD_MATV, "[I]----[%s]: ZL[REG_%x]=%x, ZH[REG_%x]=%x.\r\n", __func__, QMAX981_ZOUTL, databuf[4], QMAX981_ZOUTH, databuf[5]);
	}

	data_raw[0] = (kal_int16)((databuf[1]<<2) |( databuf[0]>>6));
	data_raw[1] = (kal_int16)((databuf[3]<<2) |( databuf[2]>>6));
	data_raw[2] = (kal_int16)((databuf[5]<<2) |( databuf[4]>>6));
	//QMAX981_INFO(MOD_MATV, "[I]----[%s]: raw:(X=%d,Y=%d,Z=%d).\r\n", __func__, data_raw[0], data_raw[1], data_raw[2]);

	for(i=0; i<3; i++)	// 三轴数据 	 
	{								   
		if ( data_raw[i] == 0x0200 )		 // 防止溢出 10bit resolution, 512= 2^(10-1)
		{
			data_raw[i]= -512;		 
		}
		else if ( data_raw[i] & 0x0200 )  // 有符号位的，去符号位，其余各位取反加一 
		{							
			data_raw[i] -= 0x1; 		
			data_raw[i] = ~data_raw[i]; 	
			data_raw[i] &= 0x01ff;		
			data_raw[i] = -data_raw[i]; 
		}
	}
	data_raw[0] -= QMAX981_OFFSET_X;
	data_raw[1] -= QMAX981_OFFSET_Y;
	data_raw[2] -= QMAX981_OFFSET_Z;

	data_acc[g_qmaX981.cvt->map[0]] = g_qmaX981.cvt->sign[0] * data_raw[0];
	data_acc[g_qmaX981.cvt->map[1]] = g_qmaX981.cvt->sign[1] * data_raw[1];
	data_acc[g_qmaX981.cvt->map[2]] = g_qmaX981.cvt->sign[2] * data_raw[2];

	*x_adc = data_acc[0];
	*y_adc = data_acc[1];
	*z_adc = data_acc[2];
	
#if defined(QMAX981_USE_CALI)
	*x_adc += qmax981_cali[0];
	*y_adc += qmax981_cali[1];
	*z_adc += qmax981_cali[2];
#endif
	QMAX981_INFO(MOD_MATV, "acc raw data [%d %d %d]", data_acc[0], data_acc[1], data_acc[2]);
}


kal_bool qmaX981_get_acc_mg(kal_int32 *x_adc, kal_int32 *y_adc, kal_int32 *z_adc)
{
	kal_bool bResult=KAL_FALSE; 
	kal_uint8 databuf[6] = {0}; 
	kal_int16 data_raw[3]= {0};
	kal_int16 data_acc[3]= {0};
	kal_int16	i;
	
	bResult = gsensor_i2c_read_bytes(QMAX981_XOUTL, databuf, 6);

	if(KAL_FALSE == bResult)
	{
		QMAX981_ERROR(MOD_MATV, "[E]----[%s]: get data fail!\r\n", __func__);
		return KAL_FALSE;
	}
	else
	{
		//QMAX981_INFO(MOD_MATV, "[I]----[%s]: XL[REG_%x]=%x, XH[REG_%x]=%x.\r\n", __func__, QMAX981_XOUTL, databuf[0], QMAX981_XOUTH, databuf[1]);
		//QMAX981_INFO(MOD_MATV, "[I]----[%s]: YL[REG_%x]=%x, YH[REG_%x]=%x.\r\n", __func__, QMAX981_YOUTL, databuf[2], QMAX981_YOUTH, databuf[3]);
		//QMAX981_INFO(MOD_MATV, "[I]----[%s]: ZL[REG_%x]=%x, ZH[REG_%x]=%x.\r\n", __func__, QMAX981_ZOUTL, databuf[4], QMAX981_ZOUTH, databuf[5]);
	}

	data_raw[0] = (kal_int16)((databuf[1]<<2) |( databuf[0]>>6));
	data_raw[1] = (kal_int16)((databuf[3]<<2) |( databuf[2]>>6));
	data_raw[2] = (kal_int16)((databuf[5]<<2) |( databuf[4]>>6));
	//QMAX981_INFO(MOD_MATV, "[I]----[%s]: raw:(X=%d,Y=%d,Z=%d).\r\n", __func__, data_raw[0], data_raw[1], data_raw[2]);

	for(i=0; i<3; i++)	// 三轴数据 	 
	{								   
		if ( data_raw[i] == 0x0200 )		 // 防止溢出 10bit resolution, 512= 2^(10-1)
		{
			data_raw[i]= -512;		 
		}
		else if ( data_raw[i] & 0x0200 )  // 有符号位的，去符号位，其余各位取反加一 
		{							
			data_raw[i] -= 0x1; 		
			data_raw[i] = ~data_raw[i]; 	
			data_raw[i] &= 0x01ff;		
			data_raw[i] = -data_raw[i]; 
		}
	}
	data_raw[0] -= QMAX981_OFFSET_X;
	data_raw[1] -= QMAX981_OFFSET_Y;
	data_raw[2] -= QMAX981_OFFSET_Z;

	data_acc[g_qmaX981.cvt->map[0]] = g_qmaX981.cvt->sign[0] * data_raw[0];
	data_acc[g_qmaX981.cvt->map[1]] = g_qmaX981.cvt->sign[1] * data_raw[1];
	data_acc[g_qmaX981.cvt->map[2]] = g_qmaX981.cvt->sign[2] * data_raw[2];

#if defined(QMAX981_USE_CALI)
	data_acc[0] += qmax981_cali[0];
	data_acc[1] += qmax981_cali[1];
	data_acc[2] += qmax981_cali[2];
#endif

	*x_adc = (data_acc[0]*GRAVITY_1G)/g_qmaX981.lsb_1g;
	*y_adc = (data_acc[1]*GRAVITY_1G)/g_qmaX981.lsb_1g;
	*z_adc = (data_acc[2]*GRAVITY_1G)/g_qmaX981.lsb_1g;
	
	//QMAX981_INFO(MOD_MATV, "acc raw data [%d %d %d]", *x_adc, *y_adc, *z_adc);
	return KAL_TRUE;
}


#if defined(QMAX981_STEP_COUNTER)
void qmaX981_custom_reset_step(void)
{
	unsigned char databuf[2] = {0};
	unsigned char value_13;
	kal_bool bResult=KAL_FALSE;	

#if defined(QMAX981_STEP_COUNTER_USE_INT)
	qmaX981_step_debounce_reset();
#endif
	bResult = gsensor_i2c_read_bytes(0x13, databuf, 1);
	if(bResult == KAL_FALSE)
	{
		QMAX981_ERROR(MOD_MATV, "reset sc error = %d \n", bResult);
		return;
	}
	value_13 = databuf[0];
	
	bResult = gsensor_i2c_write_byte(0x13, 0x80);
	if(bResult == KAL_FALSE)
	{
		QMAX981_ERROR(MOD_MATV, "reset sc error = %d \n", bResult);
		return;
	}
	gsensor_delay_ms(20);
	bResult = gsensor_i2c_write_byte(0x13, value_13);
	if(bResult == KAL_FALSE)
	{
		QMAX981_ERROR(MOD_MATV, "reset sc error = %d \n", bResult);
		return;
	}
	QMAX981_INFO(MOD_MATV, "reset sc OK \n");
}


kal_bool qmaX981_custom_get_step(kal_uint32 *step)
{
	kal_bool bResult=KAL_FALSE;	
	kal_uint8 databuf[2] = {0};
	kal_int32	result;

	bResult = gsensor_i2c_read_bytes(QMAX981_STEP_CNT_L, databuf, 2);
	if(bResult == KAL_FALSE)
	{	
		QMAX981_ERROR(MOD_MATV, "[E]----[%s]: Motion Sensor get step fail!\r\n", __func__);
		return KAL_FALSE;
	}
	else
	{
		result = (databuf[1]<<8)|databuf[0];
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
		if(qmaX981_check_abnormal_data(result, &result) == 0)
		{
			QMAX981_ERROR(MOD_MATV, "qmaX981_check_abnormal_data error!!!\n");
			return KAL_FALSE;
		}
#endif
#if defined(QMAX981_STEP_COUNTER_USE_INT)
		result = qmaX981_step_debounce_read_data(result);
#endif
		*step = result;
		return KAL_TRUE;
	}
}
#endif


static kal_bool qmaX981_soft_reset(void)
{
    kal_bool  bResult=KAL_FALSE;
	
	bResult = gsensor_i2c_write_byte(0x36, 0xb6);
	gsensor_delay_ms(2); 
	bResult = gsensor_i2c_write_byte(0x36, 0x00); 

	return bResult;
}

static kal_bool qmaX981_get_chipid(void)
{
	kal_uint8 chip_id = 0;
    kal_bool  bResult=KAL_FALSE;

    bResult=gsensor_i2c_read_bytes(QMAX981_CHIP_ID, &chip_id, 1);
    
	QMAX981_INFO(MOD_MATV, "[%s]: read chip id(%x) !", __func__, chip_id);
	g_qmaX981.chip_id = chip_id;
    if((chip_id==0xa9)||(chip_id>=0xb0 && chip_id<=0xb6))
    {
    	g_qmaX981.chip_type = QMAX981_TYPE_6981;
    }
    else if((chip_id>=0xe0) && (chip_id<=0xe6))
    {
    	g_qmaX981.chip_type = QMAX981_TYPE_7981;
    }

	return bResult;
}

static kal_bool qmaX981_set_mode(kal_bool mode)
{
    kal_bool  bResult=KAL_FALSE;

#if defined(QMAX981_STEP_COUNTER)
	mode =  KAL_TRUE;
#endif
	if(mode == KAL_TRUE)	
    	bResult = gsensor_i2c_write_byte(QMAX981_MODE, 0x80); 
	else
    	bResult = gsensor_i2c_write_byte(QMAX981_MODE, 0x00); 

	return bResult;
}


static kal_bool qmaX981_set_range(unsigned char range)
{
    kal_bool  bResult=KAL_FALSE;

	//range = QMAX981_RANGE_8G;
	bResult = gsensor_i2c_write_byte(QMAX981_RANGE, range);

	if(range == QMAX981_RANGE_2G)
		g_qmaX981.lsb_1g = 256;
	else if(range == QMAX981_RANGE_4G)
		g_qmaX981.lsb_1g = 128;
	else if(range == QMAX981_RANGE_8G)
		g_qmaX981.lsb_1g = 64;
	else
		g_qmaX981.lsb_1g = 256;

	return bResult;
} 



static kal_bool qma6981_custom_init(void)
{
    kal_bool  bResult=KAL_FALSE;
    kal_uint8 reg_value = 0x00;

	bResult = qmaX981_set_range(QMAX981_RANGE_4G);
	bResult = gsensor_i2c_write_byte(QMAX981_ODR, 0x05);
	bResult = gsensor_i2c_write_byte(0x27, 0x00);
	bResult = gsensor_i2c_write_byte(0x28, 0x00);
	bResult = gsensor_i2c_write_byte(0x29, 0x00);
#if defined(QMAX981_STEP_COUNTER)
	bResult = qmaX981_set_range(QMAX981_RANGE_8G);
	// 220 ua, form phone, more used
	bResult = gsensor_i2c_write_byte(QMAX981_ODR, 0x2a);
	bResult = gsensor_i2c_write_byte(0x12, 0x8f);
	bResult = gsensor_i2c_write_byte(0x13, 0x10);
	bResult = gsensor_i2c_write_byte(0x14, 0x14);
	bResult = gsensor_i2c_write_byte(0x15, 0x10);
	// 220 ua
#if 0	// about 100 ua
	bResult = gsensor_i2c_write_byte(QMAX981_ODR, 0x06);
	bResult = gsensor_i2c_write_byte(0x11, 0x89);
	bResult = gsensor_i2c_write_byte(0x12, 0x8f);
	bResult = gsensor_i2c_write_byte(0x13, 0x10);
	bResult = gsensor_i2c_write_byte(0x14, 0x28);
	bResult = gsensor_i2c_write_byte(0x15, 0x20);
#endif // about 100 ua
	reg_value = 0x02;	// step axis select (xy:0x00  yz:0x01  xz:0x02)
	bResult = gsensor_i2c_write_byte(0x32, reg_value);
	bResult = gsensor_i2c_write_byte(0x27, QMAX981_OFFSET_X);
	bResult = gsensor_i2c_write_byte(0x28, QMAX981_OFFSET_Y);
	bResult = gsensor_i2c_write_byte(0x29, QMAX981_OFFSET_Z);
	bResult = gsensor_i2c_write_byte(0x16, 0x0c);
	#if defined(QMAX981_STEP_COUNTER_USE_INT)
	bResult = gsensor_i2c_write_byte(0x19, 0x08);
	#endif
#endif

#if defined(QMA6891_EXTRA_FUNC_1)
	bResult = gsensor_i2c_write_byte(0x0f, 0x04); // 0x01 :+-2g(1g=256)    0x04:+-8g(1g=64)
	bResult = gsensor_i2c_write_byte(0x10, 0x2b);
	bResult = gsensor_i2c_write_byte(0x12, 0x0f);
	bResult = gsensor_i2c_write_byte(0x27, 0x00);
	bResult = gsensor_i2c_write_byte(0x28, 0x00);
	bResult = gsensor_i2c_write_byte(0x29, 0x00);

	bResult = gsensor_i2c_write_byte(0x19, 0x00);
	bResult = gsensor_i2c_write_byte(0x16, 0x00);

	if(QMA6891_EXTRA_FUNC_1 & QMAX981_FOB_FLAG)
	{
		//Front/back z axis threshold, the actual g value is FB_Z_TH<7:0>*3.91mg+0.1g, independent of the selected g range
		bResult = gsensor_i2c_write_byte(0x30, 0xB4);
	}

	if(QMA6891_EXTRA_FUNC_1 & QMAX981_ORIENT_FLAG)
	{
		// Up/down x axis threshold, the actual g value is UD_X_TH<7:0>*3.91mg, independent of the selected g range,
		// the default value is 0.64g, corresponding to 40 degree
		bResult = gsensor_i2c_write_byte(0x2D, 0xBE);
		
		// Right/left y axis threshold, the actual g value is RL_Y_TH<7:0>*3.91mg, independent of the selected g range,
		// the default value is 0.64g, corresponding to 40 degree
		bResult = gsensor_i2c_write_byte(0x2F, 0xBE);
		
		//UD_Z_TH:
		//Up/down z axis threshold, the actual g value is UD_Z_TH<7:0>*3.91mg+0.1g, independent of the selected g range
		bResult = gsensor_i2c_write_byte(0x2C, 0x99);
		
		//L_Z_TH:
		//Right/left z axis threshold, the actual g value is RL_Z_TH<7:0>*3.91mg+0.1g, independent of the selected g range
		bResult = gsensor_i2c_write_byte(0x2E, 0x99);
	}

	bResult = gsensor_i2c_write_byte(0x16, QMA6891_EXTRA_FUNC_1);
	bResult = gsensor_i2c_write_byte(0x19, QMA6891_EXTRA_FUNC_1);
	
	bResult = gsensor_i2c_write_byte(0x11, 0x80); 
#elif defined(QMA6891_EXTRA_FUNC_2)
	bResult = gsensor_i2c_write_byte(0x0f, 0x04); // 0x01 :+-2g(1g=256)    0x04:+-8g(1g=64)
	bResult = gsensor_i2c_write_byte(0x10, 0x2b);
	bResult = gsensor_i2c_write_byte(0x12, 0x0f);
	bResult = gsensor_i2c_write_byte(0x27, 0x00);
	bResult = gsensor_i2c_write_byte(0x28, 0x00);
	bResult = gsensor_i2c_write_byte(0x29, 0x00);

	bResult = gsensor_i2c_write_byte(0x19, 0x00);
	bResult = gsensor_i2c_write_byte(0x16, 0x00);

	if((QMA6891_EXTRA_FUNC_2&QMAX981_HIGH_G_X_FLAG)
		||(QMA6891_EXTRA_FUNC_2&QMAX981_HIGH_G_Y_FLAG)
		||(QMA6891_EXTRA_FUNC_2&QMAX981_HIGH_G_Z_FLAG))
	{
		bResult = gsensor_i2c_write_byte(0x24, 0x82);
	// HIGH_DUR
		bResult = gsensor_i2c_write_byte(0x25, 0x0f);
		bResult = gsensor_i2c_write_byte(0x26, 0x40);
		bResult = gsensor_i2c_write_byte(0x17, QMA6891_EXTRA_FUNC_2);
		bResult = gsensor_i2c_write_byte(0x1a, 0x04);
	}
	 bResult = gsensor_i2c_write_byte(0x11, 0x80); 
#elif defined(QMA6891_EXTRA_FUNC_3)
	#if defined(QMAX981_STEP_COUNTER)
	bResult = gsensor_i2c_write_byte(QMAX981_ODR, 0x06);
	bResult = gsensor_i2c_write_byte(0x11, 0x89);
	bResult = gsensor_i2c_write_byte(0x12, 0x8f);
	bResult = gsensor_i2c_write_byte(0x13, 0x10);
	bResult = gsensor_i2c_write_byte(0x14, 0x28);
	bResult = gsensor_i2c_write_byte(0x15, 0x20);
	#endif
	// odr
	//bResult = gsensor_i2c_write_byte(0x10, 0x06); // 6:ODR是500HZ采样  5:250HZ	建议是6，5不太灵敏
	// TAP_QUIET<7>: tap quiet time, 1: 30ms, 0: 20ms 
	// TAP_SHOCK<6>: tap shock time, 1: 50ms, 0: 75ms
	// TAP_DUR<2:0>: the time window of the second tap event for double tap
	
	//TAP_DUR			Duration of TAP_DUR
	//000					50ms
	//001					100ms
	//010					150ms
	//011					200ms
	//100					250ms
	//101					375ms
	//110					500ms
	//111					700ms
	
	bResult = gsensor_i2c_write_byte(0x2A, 0x80); // 
	// TAP_TH<4:0>
	// 62.5*9=562.5 mg, TAP_TH is 62.5mg in 2g-range, 125mg in 4g-range, 250mg in 8g-range. 
	if(g_qmaX981.lsb_1g == 64)		// +-8g
		bResult = gsensor_i2c_write_byte(0x2B, 0x04); // threshold 1G default
	else if(g_qmaX981.lsb_1g == 128)	// +-4g
		bResult = gsensor_i2c_write_byte(0x2B, 0x08); // threshold 1G default
	else	// +-2g
		bResult = gsensor_i2c_write_byte(0x2B, 0x10); // threshold 1G default

	gsensor_i2c_read_bytes(0x16, &reg_value, 1);
	reg_value |= 0x20;	// register 0x16 bit5 S_TAP_EN, bit4 D_TAP_EN
	bResult = gsensor_i2c_write_byte(0x16, reg_value);
	gsensor_i2c_read_bytes(0x19, &reg_value, 1);
	reg_value |= 0x20;	// register 0x19 bit5 S_TAP, bit4 D_TAP, map to INT1
	bResult = gsensor_i2c_write_byte(0x19, reg_value); // 
#endif

	bResult = gsensor_i2c_write_byte(0x20, 0x00); // 低电平或者下降沿触发
	//bResult = gsensor_i2c_write_byte(0x20, 0x05);// 高电平或者上升沿触发
	return bResult;
}


static kal_bool qma7981_custom_init(void)
{
	return KAL_FALSE;
}

static void qmaX981_custom_init(void)
{
    kal_bool  bResult=KAL_FALSE;

#ifdef MS_DBG
	sprintf(qma6981_debug_buff,"qmaX981_custom_init start");
	kal_bootup_print(qma6981_debug_buff);
#endif
// i2c config	
	//ms_i2c_close();
	ms_i2c_configure(QMAX981_SLAVE_ADDR, MS_DELAY_TIME);
// chip id   
	memset(&g_qmaX981, 0, sizeof(g_qmaX981));

	qmaX981_set_mode(KAL_TRUE);
	qmaX981_soft_reset();
	qmaX981_set_mode(KAL_TRUE);
	qmaX981_get_chipid();

	if(g_qmaX981.chip_type == QMAX981_TYPE_6981)
		bResult = qma6981_custom_init();
	else if(g_qmaX981.chip_type == QMAX981_TYPE_7981)
		bResult = qma7981_custom_init();
	else
		bResult = KAL_FALSE;


	g_qmaX981.layout = 0;
	g_qmaX981.cvt = &qmaX981_map[g_qmaX981.layout];

#if defined(QMAX981_USE_INT1)
	qmaX981_setup_int1();
#endif
#if defined(QMAX981_USE_INT2)
	qmaX981_setup_int2();
#endif
    if(bResult)
    {
        QMAX981_INFO(MOD_MATV, "[I]----[%s]: success!", __func__);
    }
    else
    {
        QMAX981_ERROR(MOD_MATV, "[E]----[%s]: failure!", __func__);
    }

    //return bResult;
}

kal_bool qmaX981_query_gesture(kal_uint16 ms_gest_type)
{
    kal_bool  bResult=KAL_FALSE;

#if 0
	switch(ms_gest_type)
	{
		case MS_TAP:
			return KAL_TRUE;	
		case MS_STEP:
			return KAL_TRUE;
		case MS_DROP:
			return KAL_TRUE;
		case MS_FLIP:
			return KAL_TRUE;
			
		default:
			return KAL_FALSE;
	}
#else
	if(ms_gest_type == MS_STEP)
	{	
#if defined(QMAX981_STEP_COUNTER)
		bResult = qmaX981_custom_get_step(&g_qmaX981.step);
#endif
		return bResult;
	}
	else
	{
		return bResult;
	}
#endif
}

kal_bool qmaX981_get_sensor_params(kal_uint16 ms_params_type, MotionSensorQueryStruct *ms_params)
{	
	return KAL_TRUE;		
}

kal_bool qmaX981_set_sensor_params(kal_uint16 ms_params_type, kal_uint32 ms_params)
{
	return KAL_TRUE;	
}



void qmaX981_custom_pwr_up(void)
{
	qmaX981_set_mode(KAL_TRUE);  
}

void qmaX981_custom_pwr_down(void)
{
	qmaX981_set_mode(KAL_FALSE);
}


MotionSensor_custom_data_struct  ms_custom_data_def = 
{          
    /*X axis*/
    ACC_0G_X,   
    ACC_1G_X,
    ACC_MINUS1G_X,     
    /*Y axis*/
    ACC_0G_Y,   
    ACC_1G_Y,
    ACC_MINUS1G_Y,     
    /*Z axis*/
    ACC_0G_Z,   
    ACC_1G_Z,
    ACC_MINUS1G_Z,
    /*support interrupt or not*/
    KAL_FALSE,
    0,
    0/*channel*/    
};


MotionSensor_custom_data_struct * (ms_get_data)(void)
{
    return (&ms_custom_data_def);
} 


MotionSensor_customize_function_struct ms_custom_func=
{
	ms_get_data,
	qmaX981_custom_get_data,
	qmaX981_custom_init,
	qmaX981_custom_pwr_up,
	qmaX981_custom_pwr_down,
	NULL,
	NULL,
	NULL,
	NULL,
	qmaX981_query_gesture,
	qmaX981_get_sensor_params,
	qmaX981_set_sensor_params
};


MotionSensor_customize_function_struct *ms_GetFunc(void)
{
    return (&ms_custom_func);  
}


#if defined(QMAX981_USE_CALI)
static void qmax981_write_file(kal_uint16 * filename, kal_char *data, int len)
{
	FS_HANDLE fd = -1;

	if((fd = FS_Open(filename, FS_CREATE | FS_READ_WRITE)) < FS_NO_ERROR)	  /* mre is not folder */
	{		
		kal_prompt_trace(MOD_NAME, DRV_NAME "%s open file error!\r\n", __func__);
		FS_Close(fd);
		return;
	}
	else
	{
		FS_Write(fd, data, len, NULL);
		FS_Close(fd);
		return;
	}
	
}


static void qmax981_read_file(kal_uint16 * filename, kal_char *data, int len)
{
	FS_HANDLE fd = -1;
	kal_uint32 file_size = 0;
	
	if((fd = FS_Open(filename, FS_CREATE | FS_READ_WRITE)) < FS_NO_ERROR)	  /* mre is not folder */
	{
		kal_prompt_trace(MOD_NAME, DRV_NAME "%s open file error!\r\n", __func__);
		FS_Close(fd);
		return;
	}
	else
	{
		FS_Read(fd, data, len, NULL);
		FS_Close(fd);
		return;
	}
}

void qmax981_do_cali(void)
{
	kal_int16 data[3], data_avg[3];
	int icount, z_max, z_min;
	cenon_drv_ms_report_st  acc;

	data_avg[0] = 0;
	data_avg[1] = 0;
	data_avg[2] = 0;
	
	for(icount=0; icount<QMAX981_CALI_NUM; icount++)
	{
		//qmax981_read_raw(data);
		qmaX981_custom_get_data(&data[0], &data[1], &data[2]);
		data_avg[0] += data[0];
		data_avg[1] += data[1];
		data_avg[2] += data[2];
		// add by yangzhiqiang check vibrate
		if(icount == 0)
		{
			z_max = data[2];
			z_min = data[2];
		}
		else
		{
			z_max = (data[2]>z_max)?data[2]:z_max;
			z_min = (data[2]<z_min)?data[2]:z_min;
		}
		// add by yangzhiqiang check vibrate
		//mdelay(5);
		gsensor_delay_ms(5);
	}
	// add by yangzhiqiang check vibrate
	if((z_max-z_min)>(g_qmaX981.lsb_1g*3/10))
	{
		kal_prompt_trace(MOD_NAME, DRV_NAME "qmax981_cali_store check vibrate cali ingore!\n");
		return;
	}
	// add by yangzhiqiang check vibrate

	data_avg[0] = data_avg[0]/QMAX981_CALI_NUM;
	data_avg[1] = data_avg[1]/QMAX981_CALI_NUM;
	data_avg[2] = data_avg[2]/QMAX981_CALI_NUM;
	//printk("qmax981_cali_store data_avg[%d %d %d]\n", data_avg[0], data_avg[1], data_avg[2]);
	// add by yangzhiqiang check offset range
#if 0
	if(QMAX981_ABS(data_avg[2]-QMAX981_LSB_1G)>(QMAX981_LSB_1G*5/10))
	{
		printk("qmax981_cali_store check offset range cali ingore!\n");
		return count;
	}
#endif
	// add by yangzhiqiang check offset range
	data[0] = 0-data_avg[0];
	data[1] = 0-data_avg[1];
	data[2] = g_qmaX981.lsb_1g-data_avg[2];
	qmax981_cali[0] = data[0];
	qmax981_cali[1] = data[1];
	qmax981_cali[2] = data[2];
	kal_prompt_trace(MOD_NAME, DRV_NAME "qmax981_cali_store qmax981_cali[%d %d %d]\n", qmax981_cali[0], qmax981_cali[1], qmax981_cali[2]);
	qmax981_write_file(QMAX981_CALI_FILE, (kal_char *)qmax981_cali, sizeof(qmax981_cali));
}

void MDrv_Gsensor_Custom_Cali(void)
{
	qmax981_do_cali();
}

#endif

/*========================================================================================================
										THE END
========================================================================================================*/
#endif//#if defined(MOTION_SENSOR_SUPPORT)


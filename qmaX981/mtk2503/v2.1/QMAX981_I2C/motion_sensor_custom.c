
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
#if defined(QMA6891_EXTRA_FUNC_3) 
#include "kbd_table.h"
#include "keypad_sw.h"
#include "stack_ltlcom.h"
#endif
/*========================================================================================================
										D E B U G
========================================================================================================*/
#define QMAX981_DEBUG
#if defined(QMAX981_DEBUG)&&defined(__MTK_TARGET__)
#define QMAX981_INFO			kal_prompt_trace
#define QMAX981_ERROR			kal_prompt_trace
#define ms_dbg_print_ps 		dbg_print
#else
#ifndef WIN32
#define QMAX981_INFO(fmt, arg...)			do {} while (0)
#define QMAX981_ERROR(fmt, arg...)			do {} while (0)
#define ms_dbg_print_ps(...)
#endif
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
	QMAX981_TYPE_UNKNOW,
	QMAX981_TYPE_6981,
	QMAX981_TYPE_7981,
	QMAX981_TYPE_6100,
	QMAX981_TYPE_MAX
}qmaX981_chip_type;

enum
{
	QMAX981_AXIS_X     =     0,
	QMAX981_AXIS_Y     =     1,
	QMAX981_AXIS_Z     =     2,
	QMAX981_AXES_NUM   =     3
};

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
#if 0//defined(QMA6891_EXTRA_FUNC_3) 
	kal_uint8			tap_int1_no;
	kal_uint8			tap_int1_level;
	kal_uint8			step_int1_level;
#endif
};


/*========================================================================================================
										V A R I A B L E S
========================================================================================================*/
#ifdef MS_DBG
kal_char qma6981_debug_buff[200];
#endif
static struct qmaX981_data g_qmaX981;
static struct hwmsen_convert g_map;

//#define QMAX981_USE_CALI

#if defined(QMAX981_USE_CALI)
#define QMAX981_CALI_FILE		L"Z:\\qmax981cali.conf"
#define QMAX981_LSB_1G			64			// mg
#define QMAX981_CALI_NUM		20    
static int qmax981_cali[3]={0, 0, 0};
static kal_char qmax981_calied_flag = 0;
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
static void gsensor_delay_us(kal_uint32 delay)
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
        gsensor_delay_us(6268);//SW_i2c_udelay(1000);
    }
}


//======================================
// [G-sensor]: i2c 读写函数
//======================================
kal_bool gsensor_i2c_write_byte(kal_uint8 ucBufferIndex, kal_uint8 pucData)
{
#if defined(USE_QST_SW_IIC)
	return qst_sw_writereg(QMAX981_SLAVE_ADDR, ucBufferIndex, pucData);
#else
    return ms_i2c_send(QMAX981_SLAVE_ADDR, ucBufferIndex, &pucData, 1);
#endif
}

//#define gsensor_i2c_read_bytes(reg_no,buffer_name,length) ms_i2c_receive(QMAX981_SLAVE_ADDR, reg_no, buffer_name, length)

kal_bool gsensor_i2c_read_bytes(kal_uint8 reg_no, kal_uint8* buffer_name, kal_uint32 length)
{
#if defined(USE_QST_SW_IIC)
	return qst_sw_readreg(QMAX981_SLAVE_ADDR, reg_no, buffer_name, length);
#else
	return ms_i2c_receive(QMAX981_SLAVE_ADDR, reg_no, buffer_name, length);
#endif
}

/*========================================================================================================
										F U N C T I O N----public
========================================================================================================*/
#if defined(QMAX981_STEP_COUNTER_USE_INT)
void qmaX981_step_eint_hisr(void)
{
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
#ifndef WIN32
	QMAX981_INFO(MOD_MATV, "[%s]: hisr read data = %d!", __func__, result);
#endif
	if(g_qmaX981.int1_level == 0)
	{
		g_qmaX981.int1_level = 1;
		EINT_Set_Polarity(g_qmaX981.int1_no, KAL_TRUE); 
	}
	else
	{
		g_qmaX981.int1_level = 0;
		EINT_Set_Polarity(g_qmaX981.int1_no, KAL_FALSE); 
	}

	if(ret == KAL_FALSE)
	{
	#ifndef WIN32
		QMAX981_ERROR(MOD_MATV, "qmaX981_sensor_eint1_hisr read step error!!");
	#endif
		return;
	}

	qmaX981_step_debounce_int_work(result, g_qmaX981.int1_level);
}
#endif

#if defined(QMA6891_EXTRA_FUNC_3)
extern kal_bool             kbd_push_assert;
extern kbd_buffer_struct    kbd_buffer;
static void tap_kbd_push_onekey(kbd_event event,kal_uint8 key)  
{
	ASSERT(kbd_push_assert == KAL_FALSE);
	kbd_push_assert = KAL_TRUE;
	kbd_buffer.kbd_data_buffer[kbd_buffer.kbd_data_buffer_windex].Keyevent = event;
	kbd_buffer.kbd_data_buffer[kbd_buffer.kbd_data_buffer_windex].Keydata[0] = key;
	//kbd_push_time_stamp();
	kbd_buffer.kbd_data_buffer_windex++;
	kbd_buffer.kbd_data_buffer_windex &= (kbd_buffer_size-1);
	kbd_push_assert = KAL_FALSE;
}

void qmaX981_tap_eint_hisr(void)
{
	ilm_struct 	*ilm_ptr = NULL;

#ifndef WIN32
	QMAX981_ERROR(MOD_MATV, "qmaX981_tap_eint_hisr ");
#endif
	tap_kbd_push_onekey(kbd_onekey_press, DEVICE_KEY_END);
	DRV_BuildPrimitive(ilm_ptr,
				MOD_DRVKBD,
				MOD_UEM,
				MSG_ID_DRVUEM_KEYPAD_IND,
				NULL);
	msg_send_ext_queue(ilm_ptr); 

	tap_kbd_push_onekey(kbd_onekey_release, DEVICE_KEY_END);
	DRV_BuildPrimitive(ilm_ptr,
				MOD_DRVKBD,
				MOD_UEM,
				MSG_ID_DRVUEM_KEYPAD_IND,
				NULL);
	msg_send_ext_queue(ilm_ptr); 
}
#endif

#if defined(QMAX981_USE_INT1)
void qmaX981_eint1_enable(unsigned char flag)
{
	if(flag)
		EINT_Mask(g_qmaX981.int1_no);
	else
		EINT_UnMask(g_qmaX981.int1_no);
}

void qmaX981_sensor_eint1_hisr(void)
{
#if defined(QMA6891_EXTRA_FUNC_3)
	qmaX981_tap_eint_hisr();
#else 
#if defined(QMAX981_STEP_COUNTER_USE_INT)
	qmaX981_step_eint_hisr();
#endif
#endif
}

static void qmaX981_setup_int1(void)
{
	g_qmaX981.int1_no = custom_eint_get_channel(motion_senosr_eint_chann);
#ifndef WIN32
	QMAX981_INFO("[%s]: g_qmaX981.int1_no = %d!\r\n", __func__, g_qmaX981.int1_no);
#endif
#if defined(QMAX981_STEP_COUNTER_USE_INT)
	qmaX981_step_debounce_reset();
#endif
	//EINT_SW_Debounce_Modify(g_qmaX981.int1_no, 10);
	//EINT_Set_HW_Debounce(g_qmaX981.int1_no, 10);
	EINT_Set_Sensitivity(g_qmaX981.int1_no, EDGE_SENSITIVE);
	EINT_Registration(g_qmaX981.int1_no, KAL_TRUE, KAL_FALSE, qmaX981_sensor_eint1_hisr, KAL_TRUE);	
	EINT_Set_Polarity(g_qmaX981.int1_no,KAL_FALSE);	
	EINT_UnMask(g_qmaX981.int1_no);
}
#endif

#if 0//defined(QMA6891_EXTRA_FUNC_3)
extern kal_bool             kbd_push_assert;
extern kbd_buffer_struct    kbd_buffer;
static void tap_kbd_push_onekey(kbd_event event,kal_uint8 key)  
{
	ASSERT(kbd_push_assert == KAL_FALSE);
	kbd_push_assert = KAL_TRUE;
	kbd_buffer.kbd_data_buffer[kbd_buffer.kbd_data_buffer_windex].Keyevent = event;
	kbd_buffer.kbd_data_buffer[kbd_buffer.kbd_data_buffer_windex].Keydata[0] = key;
	//kbd_push_time_stamp();
	kbd_buffer.kbd_data_buffer_windex++;
	kbd_buffer.kbd_data_buffer_windex &= (kbd_buffer_size-1);
	kbd_push_assert = KAL_FALSE;
}

void tap_press_one_key(kal_uint32 key_code)
{
	ilm_struct 	*Kp_ilm = NULL;

	tap_kbd_push_onekey(kbd_onekey_press, key_code);
	DRV_BuildPrimitive(Kp_ilm,
						MOD_DRVKBD,
						MOD_UEM,
						MSG_ID_DRVUEM_KEYPAD_IND,
						NULL);
	msg_send_ext_queue(Kp_ilm);

	tap_kbd_push_onekey(kbd_onekey_release, key_code);
	DRV_BuildPrimitive(Kp_ilm,
						MOD_DRVKBD,
						MOD_UEM,
						MSG_ID_DRVUEM_KEYPAD_IND,
						NULL);
	msg_send_ext_queue(Kp_ilm);
}

void qmaX981_sensor_tap_eint1_hisr(void)
{
	ilm_struct 	*ilm_ptr = NULL;
	kal_bool ret;
	unsigned char reg_data[4];
	int result, i;

	for(i=0; i<3; i++)
	{
		ret = gsensor_i2c_read_bytes(0x0a, reg_data, 2);
		if(ret)
		{
			break;
		}
	}

	if(reg_data[0] & 0x20)
	{	
#ifndef WIN32
		ms_dbg_print_ps("[%s]: press!\r\n", __func__);
		DRV_BuildPrimitive(ilm_ptr,
					MOD_DRVKBD,
					MOD_UEM,
				#if defined(__FISE_FZD_SERVICE__)
					MSG_ID_APP_MAKE_FRIEND_IND,
				#else
					MSG_ID_DRVUEM_KEYPAD_IND,		
				#endif
					NULL);
		msg_send_ext_queue(ilm_ptr); 

#endif
	}

	if(reg_data[0] & 0x08)
	{
#ifndef WIN32
		if(g_qmaX981.step_int1_level == 0)
		{
			for(i=0; i<3; i++)
			{
				ret = gsensor_i2c_read_bytes(QMAX981_STEP_CNT_L, reg_data, 2);
				if(ret)
				{
					break;
				}
			}
			result = (reg_data[1]<<8)|reg_data[0];		
			kal_prompt_trace(MOD_MATV, "step_int_start step=%d \n", result);
			g_qmaX981.step_int1_level = 1;
			qmaX981_step_debounce_int_work(result, 1);
			//ret = gsensor_i2c_write_byte(0x16, 0x24);
			ret = gsensor_i2c_write_byte(0x19, 0x24);
		}
#endif
	}
	else if(reg_data[0] & 0x04)
	{
#ifndef WIN32
		if(g_qmaX981.step_int1_level == 1)
		{
			for(i=0; i<3; i++)
			{
				ret = gsensor_i2c_read_bytes(QMAX981_STEP_CNT_L, reg_data, 2);
				if(ret)
				{
					break;
				}
			}
			result = (reg_data[1]<<8)|reg_data[0];		
			kal_prompt_trace(MOD_MATV, "step_int_end step=%d \n", result);
			g_qmaX981.step_int1_level = 0;
			qmaX981_step_debounce_int_work(result, 0);
			//ret = gsensor_i2c_write_byte(0x16, 0x28);
			ret = gsensor_i2c_write_byte(0x19, 0x28);
		}
#endif
	}
}
#endif

void qmaX981_get_raw_data(kal_int32 raw_data[3])
{
	kal_bool bResult=KAL_FALSE;	
	kal_uint8 databuf[6] = {0};	
	kal_int16 read_data[3];
	kal_int16	i;

	bResult = gsensor_i2c_read_bytes(QMAX981_XOUTL, databuf, 6);
	if(KAL_FALSE == bResult)
	{
		#ifndef WIN32
		QMAX981_ERROR(MOD_MATV, "[E]----[%s]: get data fail!\r\n", __func__);
		#endif
		return;
	}

	if(g_qmaX981.chip_type == QMAX981_TYPE_6981)
	{
		read_data[0] = (kal_int16)((databuf[1]<<2) |( databuf[0]>>6));
		read_data[1] = (kal_int16)((databuf[3]<<2) |( databuf[2]>>6));
		read_data[2] = (kal_int16)((databuf[5]<<2) |( databuf[4]>>6));
		for(i=0; i<3; i++)	// 三轴数据 	 
		{								   
			if (read_data[i] == 0x0200)		 // 防止溢出 10bit resolution, 512= 2^(10-1)
			{
				read_data[i]= -512;		 
			}
			else if ( read_data[i] & 0x0200 )  // 有符号位的，去符号位，其余各位取反加一 
			{							
				read_data[i] -= 0x1; 		
				read_data[i] = ~read_data[i]; 	
				read_data[i] &= 0x01ff;		
				read_data[i] = -read_data[i]; 
			}
		}
		read_data[0] -= QMAX981_OFFSET_X;
		read_data[1] -= QMAX981_OFFSET_Y;
		read_data[2] -= QMAX981_OFFSET_Z;
	}
	else if((g_qmaX981.chip_type == QMAX981_TYPE_7981)||(g_qmaX981.chip_type == QMAX981_TYPE_6100))
	{
		read_data[0] = (kal_int16)(((kal_int16)databuf[1]<<8) |(databuf[0]));
		read_data[1] = (kal_int16)(((kal_int16)databuf[3]<<8) |(databuf[2]));
		read_data[2] = (kal_int16)(((kal_int16)databuf[5]<<8) |(databuf[4]));

		read_data[0] = read_data[0]>>2;
		read_data[1] = read_data[1]>>2;
		read_data[2] = read_data[2]>>2;
	}

	raw_data[g_map.map[0]] = (kal_int32)(g_map.sign[0] * read_data[0]);
	raw_data[g_map.map[1]] = (kal_int32)(g_map.sign[1] * read_data[1]);
	raw_data[g_map.map[2]] = (kal_int32)(g_map.sign[2] * read_data[2]);

#if defined(QMAX981_USE_CALI)
	if(qmax981_calied_flag == 1)
	{
		raw_data[0] += qmax981_cali[0];
		raw_data[1] += qmax981_cali[1];
		raw_data[2] += qmax981_cali[2];
	}
#endif

}

void qmaX981_custom_get_data(kal_uint16 *x_adc, kal_uint16 *y_adc, kal_uint16 *z_adc)
{
	kal_int32 data_acc[3]= {0};

	qmaX981_get_raw_data(data_acc);
	*x_adc = (kal_uint16 )data_acc[0];
	*y_adc = (kal_uint16 )data_acc[1];
	*z_adc = (kal_uint16 )data_acc[2];
	
#ifndef WIN32
	QMAX981_INFO(MOD_MATV, "qmaX981_custom_get_data [%d %d %d]", data_acc[0], data_acc[1], data_acc[2]);
#endif
}


kal_bool qmaX981_get_acc_mg(kal_int32 *x_adc, kal_int32 *y_adc, kal_int32 *z_adc)
{
	kal_bool bResult=KAL_FALSE; 
	kal_int32 raw_data[3];

	qmaX981_get_raw_data(raw_data);

	*x_adc = (raw_data[0]*GRAVITY_1G)/g_qmaX981.lsb_1g;
	*y_adc = (raw_data[1]*GRAVITY_1G)/g_qmaX981.lsb_1g;
	*z_adc = (raw_data[2]*GRAVITY_1G)/g_qmaX981.lsb_1g;

	QMAX981_INFO(MOD_MATV, "qmaX981_get_acc_mg [%d %d %d]", *x_adc, *y_adc, *z_adc);
}


#if defined(QMAX981_STEPCOUNTER)
kal_bool qmaX981_custom_get_step(kal_uint32 *step)
{
	kal_bool bResult=KAL_FALSE;	
	kal_uint8 databuf[3] = {0};
	kal_int32	result;

	bResult = gsensor_i2c_read_bytes(QMAX981_STEP_CNT_L, databuf, 2);
	if(bResult == KAL_FALSE)
	{
		QMAX981_ERROR(MOD_MATV, "[E]----[%s]: Motion Sensor get step fail!\r\n", __func__);
		return KAL_FALSE;
	}
	
	if(g_qmaX981.chip_type == QMAX981_TYPE_6981)
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
	}
	else if(g_qmaX981.chip_type == QMAX981_TYPE_7981)
	{
		bResult = gsensor_i2c_read_bytes(0x0e, &databuf[2], 1);
		if(bResult == KAL_FALSE)
		{
			QMAX981_ERROR(MOD_MATV, "[E]----[%s]: Motion Sensor get step fail!\r\n", __func__);
			return KAL_FALSE;
		}
		result = (((int)databuf[2]<<16)|((int)databuf[1]<<8)|databuf[0]);		
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
		if(qmaX981_check_abnormal_data(result, &result) == 0)
		{
			QMAX981_ERROR(MOD_MATV, "qmaX981_check_abnormal_data error!!!\n");
			return KAL_FALSE;
		}
#endif
	}
	
	*step = result;

	return KAL_TRUE;
}

void qmaX981_custom_reset_step(void){	
    unsigned char databuf[2] = {0}; 
    unsigned char value_13; 
    kal_bool bResult=KAL_FALSE;     
#if defined(QMAX981_STEP_COUNTER_USE_INT) 
    qmaX981_step_debounce_reset();
#endif    
    bResult = gsensor_i2c_read_bytes(0x13, databuf, 1); 
    if(bResult == KAL_FALSE){          
        return; 
    }   
    value_13 = databuf[0];      
    bResult = gsensor_i2c_write_byte(0x13, 0x80);   
    if(bResult == KAL_FALSE){           
        return; 
    }   
    gsensor_delay_ms(20);   
    bResult = gsensor_i2c_write_byte(0x13, value_13);   
    if(bResult == KAL_FALSE){           
        return; 
    }
}
#endif

static void qmaX981_set_layout(int layout)
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
	kal_uint8 count = 0;

	while(count++ <= 3)
	{
	    bResult=gsensor_i2c_read_bytes(QMAX981_CHIP_ID, &chip_id, 1);

		#ifndef WIN32
		QMAX981_INFO(MOD_MATV, "[%s]: read chip id(%x) !", __func__, chip_id);
		#endif
#ifndef WIN32
		ms_dbg_print_ps("[%s]: read chip id(%x),count =%d !\r\n", __func__, chip_id,count);
#endif
		g_qmaX981.chip_id = chip_id;
	    if((chip_id==0xa9)||(chip_id>=0xb0 && chip_id<=0xb6))
	    {
	    	g_qmaX981.chip_type = QMAX981_TYPE_6981;
			break;
	    }
	    else if((chip_id>=0xe0) && (chip_id<=0xe7))
	    {
	    	g_qmaX981.chip_type = QMAX981_TYPE_7981;
			break;
	    }		
	    else if(chip_id==0xe8)
	    {
	    	g_qmaX981.chip_type = QMAX981_TYPE_6100;
	    }
		gsensor_delay_ms(5);
	}
	return bResult;
}

static kal_bool qmaX981_set_mode(kal_bool mode)
{
    kal_bool  bResult=KAL_FALSE;

#if defined(QMAX981_STEPCOUNTER)
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

	if(g_qmaX981.chip_type == QMAX981_TYPE_6981)
	{
		if(range == QMAX981_RANGE_2G)
			g_qmaX981.lsb_1g = 256;
		else if(range == QMAX981_RANGE_4G)
			g_qmaX981.lsb_1g = 128;
		else if(range == QMAX981_RANGE_8G)
			g_qmaX981.lsb_1g = 64;
		else
			g_qmaX981.lsb_1g = 256;
	}
	else if((g_qmaX981.chip_type == QMAX981_TYPE_7981)||(g_qmaX981.chip_type == QMAX981_TYPE_6100))
	{
		if(range == QMAX981_RANGE_2G)
			g_qmaX981.lsb_1g = 4096;
		else if(range == QMAX981_RANGE_4G)
			g_qmaX981.lsb_1g = 2048;
		else if(range == QMAX981_RANGE_8G)
			g_qmaX981.lsb_1g = 1024;
		else if(range == QMAX981_RANGE_16G)
			g_qmaX981.lsb_1g = 512;
		else if(range == QMAX981_RANGE_32G)
			g_qmaX981.lsb_1g = 256;
		else
			g_qmaX981.lsb_1g = 4096;
	}

	return bResult;
} 



static kal_bool qma6981_custom_init(void)
{
    kal_bool  bResult=KAL_FALSE;
    kal_uint8 range=0;
	kal_uint8 reg_0x16=0;
	kal_uint8 reg_0x19=0;

	bResult = gsensor_i2c_write_byte(0x11, 0x80);
	bResult = qmaX981_set_range(QMAX981_RANGE_4G);
	bResult = gsensor_i2c_write_byte(QMAX981_ODR, 0x05);
	bResult = gsensor_i2c_write_byte(0x27, 0x00);
	bResult = gsensor_i2c_write_byte(0x28, 0x00);
	bResult = gsensor_i2c_write_byte(0x29, 0x00);

	bResult = gsensor_i2c_write_byte(0x16, reg_0x16);
	bResult = gsensor_i2c_write_byte(0x19, reg_0x19);
#if defined(QMAX981_STEPCOUNTER)
	#if 0
	bResult = gsensor_i2c_write_byte(QMAX981_ODR, 0x06);	
	bResult = gsensor_i2c_write_byte(0x11, 0x89);
	bResult = qmaX981_set_range(QMAX981_RANGE_8G);
	bResult = gsensor_i2c_write_byte(0x12, 0x8f);
	bResult = gsensor_i2c_write_byte(0x13, 0x10);
	bResult = gsensor_i2c_write_byte(0x14, 0x28);
	bResult = gsensor_i2c_write_byte(0x15, 0x20);
	#else	
	bResult = gsensor_i2c_write_byte(QMAX981_ODR, 0x2a);
	bResult = qmaX981_set_range(QMAX981_RANGE_8G);
	bResult = gsensor_i2c_write_byte(0x12, 0x8f);
	bResult = gsensor_i2c_write_byte(0x13, 0x0c);		// old 0x10
	bResult = gsensor_i2c_write_byte(0x14, 0x13);		// old 0x14
	bResult = gsensor_i2c_write_byte(0x15, 0x10);
	#endif

	if(g_qmaX981.layout%2)
		bResult = gsensor_i2c_write_byte(0x32, 0x02);
	else
		bResult = gsensor_i2c_write_byte(0x32, 0x01);
	
	bResult = gsensor_i2c_write_byte(0x27, QMAX981_OFFSET_X);
	bResult = gsensor_i2c_write_byte(0x28, QMAX981_OFFSET_Y);
	bResult = gsensor_i2c_write_byte(0x29, QMAX981_OFFSET_Z);

	reg_0x16 |= 0x0c;
	bResult = gsensor_i2c_write_byte(0x16, reg_0x16);
	#if defined(QMAX981_STEP_COUNTER_USE_INT)
	reg_0x19 |= 0x08;
	bResult = gsensor_i2c_write_byte(0x19, reg_0x19);
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

	reg_0x16 |= QMA6891_EXTRA_FUNC_1;
	reg_0x19 |= QMA6891_EXTRA_FUNC_1;
	bResult = gsensor_i2c_write_byte(0x16, QMA6891_EXTRA_FUNC_1);
	bResult = gsensor_i2c_write_byte(0x19, QMA6891_EXTRA_FUNC_1);
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
	#if !defined(QMAX981_STEPCOUNTER)
	bResult = gsensor_i2c_write_byte(0x10, 0x06); // 6:ODR是500HZ采样  5:250HZ	建议是6，5不太灵敏
	#endif
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
	bResult = gsensor_i2c_write_byte(0x2B, 0x04); // 

	// register 0x16 bit5 S_TAP_EN, bit4 D_TAP_EN
	reg_0x16 |= 0x20;
	reg_0x19 |= 0x20;
	bResult = gsensor_i2c_write_byte(0x16, 0x20); // 
	bResult = gsensor_i2c_write_byte(0x19, 0x20); // 
#endif
	bResult = gsensor_i2c_write_byte(0x20, 0x00); // 低电平或者下降沿触发
	//bResult = gsensor_i2c_write_byte(0x20, 0x05);// 高电平或者上升沿触发
	//bResult = gsensor_i2c_write_byte(0x21, 0x01); // latch mode
	
	return bResult;
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
		gsensor_i2c_read_bytes(0x42, &reg_0x42, 1);
		reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
		gsensor_i2c_write_byte(0x42, reg_0x42);
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
	//gsensor_i2c_write_byte(0x34, 0x9d);	//|yz|>8 m/s2, y>-3 m/m2
	if((y_th&0x80))
	{
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= (y_th&0x0f)|0x10;
		gsensor_i2c_write_byte(0x34, reg_0x34);
	}
	else
	{	
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= y_th;
		gsensor_i2c_write_byte(0x34, reg_0x34);	//|yz|>8m/s2, y<3 m/m2
	}
	//Z_TH<7:4>: -8~7, LSB 1 (unit : m/s2)	X_TH<3:0>: 0~7.5, LSB 0.5 (unit : m/s2) 
	//gsensor_i2c_write_byte(0x1e, 0x68);	//6 m/s2, 4 m/m2

	gsensor_i2c_write_byte(0x2a, (0x19|(0x03<<6)));			// 12m/s2 , 0.5m/s2
	gsensor_i2c_write_byte(0x2b, (0x7c|(0x03>>2)));
	//gsensor_i2c_write_byte(0x2a, (0x19|(0x02<<6)));			// 12m/s2 , 0.5m/s2
	//gsensor_i2c_write_byte(0x2b, (0x7c|(0x02)));

	//qmaX981_readreg(0x1e, &reg_0x1e, 1);
	if((z_th&0x80))
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= ((z_th<<4)|0x80);
		gsensor_i2c_write_byte(0x1e, reg_0x1e);
	}
	else
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= (z_th<<4);
		gsensor_i2c_write_byte(0x1e, reg_0x1e);
	}
}
#endif

#define STEP_W_TIME_L	300
#define STEP_W_TIME_H	250

static kal_bool qma7981_custom_init(void)
{
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

	gsensor_i2c_write_byte(0x36, 0xb6);
	gsensor_delay_ms(50);
	gsensor_i2c_write_byte(0x36, 0x00);
	qmaX981_set_range(QMAX981_RANGE_4G);	// 0.488 mg
	//0xe0	[65 hz		15.48 ms]
	//0xe1	[129 hz 	7.74 ms]
	//0xe2	[258 hz 	3.87 ms]
	reg_0x10 = 0xe1;
	gsensor_i2c_write_byte(0x10, reg_0x10);
	reg_0x11 = 0x80;
	gsensor_i2c_write_byte(0x11, reg_0x11);
//	gsensor_i2c_write_byte(0x4a, 0x08);	//Force I2C I2C interface
	gsensor_i2c_write_byte(0x5f, 0x80);
	gsensor_i2c_write_byte(0x5f, 0x00);	
	gsensor_delay_ms(20);
// read reg
	gsensor_i2c_read_bytes(0x16, &reg_0x16, 1);
	gsensor_i2c_read_bytes(0x18, &reg_0x18, 1);
	gsensor_i2c_read_bytes(0x19, &reg_0x19, 1);
	gsensor_i2c_read_bytes(0x1a, &reg_0x1a, 1);
	
	QMAX981_INFO(MOD_MATV,"read reg[%d %d %d %d] \n", reg_0x16, reg_0x18, reg_0x19, reg_0x1a);
// read reg
	
#if defined(QMAX981_STEPCOUNTER)
	if(reg_0x11 == 0x80)		// 500K
	{
		reg_0x14 = (((STEP_W_TIME_L*100)/771)+1);		// 0xe1 odr 129.7hz, 7.71ms
		reg_0x15 = (((STEP_W_TIME_H*100)/771)+1);
		if(reg_0x10 == 0xe0)		// odr 65hz
		{
			reg_0x14 = (reg_0x14>>1);
			reg_0x15 = (reg_0x15>>1);
		}
		else if(reg_0x10 == 0xe1)	// 129.7hz
		{
		}
		else if(reg_0x10 == 0xe5)	// odr 32.5hz
		{
			reg_0x14 = (reg_0x14>>2);
			reg_0x15 = (reg_0x15>>2);
		}
	}
	else if(reg_0x11 == 0x81)	// 333K
	{
		reg_0x14 = (((STEP_W_TIME_L*100)/581)+1);	// 0xe2 odr 172.0930233 hz, 5.81ms
		reg_0x15 = (((STEP_W_TIME_H*100)/581)+1);
		if(reg_0x10 == 0xe2)	// 172.0930233 hz
		{
		}
		else if(reg_0x10 == 0xe1)	// 86.38132296 hz
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
		reg_0x14 = (((STEP_W_TIME_L*100)/967)+1);	//0xe2 103.3591731 hz, 9.675 ms
		reg_0x15 = (((STEP_W_TIME_H*100)/967)+1);
		if(reg_0x10 == 0xe2)	// 103.3591731 hz
		{
		}
		else if(reg_0x10 == 0xe1)
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
		reg_0x14 = (((STEP_W_TIME_L*100)/975)+1);	// 0xe3 102.5641026 hz, 9.75 ms
		reg_0x15 = (((STEP_W_TIME_H*100)/975)+1);
		if(reg_0x10 == 0xe3)
		{
		}
		else if(reg_0x10 == 0xe2)
		{
			reg_0x14 = (reg_0x14>>1);		// 51.67958656 hz
			reg_0x15 = (reg_0x15>>1);
		}
	}

	QMAX981_INFO(MOD_MATV,"0x14[%d] 0x15[%d] \n", reg_0x14, reg_0x15);
	gsensor_i2c_write_byte(0x12, 0x94);
	gsensor_i2c_write_byte(0x13, 0x80);		// clear step
	gsensor_i2c_write_byte(0x13, 0x7f);		// 0x7f(1/16) 0x00(1/8)
	gsensor_i2c_write_byte(0x14, reg_0x14);		// STEP_TIME_LOW<7:0>*(1/ODR) 
	gsensor_i2c_write_byte(0x15, reg_0x15);		// STEP_TIME_UP<7:0>*8*(1/ODR) 

	//gsensor_i2c_write_byte(0x1f, 0x09); 	// 0 step
	//gsensor_i2c_write_byte(0x1f, 0x29); 	// 4 step
	//gsensor_i2c_write_byte(0x1f, 0x49); 	// 8 step
	//gsensor_i2c_write_byte(0x1f, 0x69); 	// 12 step
	//gsensor_i2c_write_byte(0x1f, 0x89); 	// 16 step
	gsensor_i2c_write_byte(0x1f, 0xa9);		// 24 step
	//gsensor_i2c_write_byte(0x1f, 0xc9); 	// 32 step
	//gsensor_i2c_write_byte(0x1f, 0xe9); 	// 40 step

	// step int
#if defined(QMA7981_STEP_INT)
	reg_0x16 |= 0x08;
	reg_0x19 |= 0x08;
	gsensor_i2c_write_byte(0x16, reg_0x16);
	gsensor_i2c_write_byte(0x19, reg_0x19);
#endif
#if defined(QMA7981_SIGNIFICANT_STEP)
	gsensor_i2c_write_byte(0x1d, 0x26);		//every 30 step
	reg_0x16 |= 0x40;
	reg_0x19 |= 0x40;
	gsensor_i2c_write_byte(0x16, reg_0x16);
	gsensor_i2c_write_byte(0x19, reg_0x19);
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
	reg_0x2c |= 0x00;	//0x01; 	// 0x00
	
	gsensor_i2c_write_byte(0x18, reg_0x18);
	gsensor_i2c_write_byte(0x1a, reg_0x1a);
	gsensor_i2c_write_byte(0x2c, reg_0x2c);
	//gsensor_i2c_write_byte(0x2e, 0x18);		// 0.488*16*20 = 156mg
	//gsensor_i2c_write_byte(0x2e, 0xc0);		// 0.488*16*192 = 1.5g
	//gsensor_i2c_write_byte(0x2e, 0x80);		// 0.488*16*128 = 1g
	//gsensor_i2c_write_byte(0x2e, 0x60);		// 0.488*16*96 = 750mg
	gsensor_i2c_write_byte(0x2e, 0x40);		// 0.488*16*64 = 500mg
#if defined(QMA7981_SIGNIFICANT_MOTION)
	//gsensor_i2c_write_byte(0x2f, 0x0c|0x01);
	gsensor_i2c_write_byte(0x2f, 0x01);
	reg_0x19 |= 0x01;
	gsensor_i2c_write_byte(0x19, reg_0x19);
#endif
#endif

#if defined(QMA7981_NO_MOTION)
	reg_0x18 |= 0xe0;
	reg_0x1a |= 0x80;
	reg_0x2c |= 0x00;	//1s			//0x24;

	gsensor_i2c_write_byte(0x18, reg_0x18);
	gsensor_i2c_write_byte(0x1a, reg_0x1a);
	gsensor_i2c_write_byte(0x2c, reg_0x2c);
	gsensor_i2c_write_byte(0x2d, 0x14);
#endif
 
#if defined(QMA7981_HAND_UP_DOWN)
	qma7981_set_hand_up_down(0);
	reg_0x16 |= 0x02;	// hand down
	reg_0x19 |= 0x02;
	gsensor_i2c_write_byte(0x16, reg_0x16);
	gsensor_i2c_write_byte(0x19, reg_0x19);
	reg_0x16 |= 0x04;	// hand down
	reg_0x19 |= 0x04;
	gsensor_i2c_write_byte(0x16, reg_0x16);
	gsensor_i2c_write_byte(0x19, reg_0x19);
#endif

#if defined(QMA7981_DATA_READY)
	reg_0x1a |= 0x10;
	gsensor_i2c_write_byte(0x17, 0x10);
	gsensor_i2c_write_byte(0x1a, reg_0x1a);
#endif

#if defined(QMA7981_INT_LATCH)
	gsensor_i2c_write_byte(0x21, 0x1f);	// default 0x1c, step latch mode
#endif
// int level set high, defalut low 
	gsensor_i2c_write_byte(0x20, 0x00);		// default 0x05
// int default level set

	return KAL_TRUE;
}

static kal_bool qma6100_custom_init(void)
{
	unsigned char reg_0x10 = 0; 

	gsensor_i2c_write_byte(0x36, 0xb6);
	gsensor_delay_ms(50);
	gsensor_i2c_write_byte(0x36, 0x00);
	qmaX981_set_range(QMAX981_RANGE_4G);	// 0.488 mg
	//0xe0	[65 hz		15.48 ms]
	//0xe1	[129 hz 	7.74 ms]
	//0xe2	[258 hz 	3.87 ms]
	reg_0x10 = 0xe0;
	gsensor_i2c_write_byte(0x10, reg_0x10);

//		gsensor_i2c_write_byte(0x4a, 0x08);	//Force I2C I2C interface
	gsensor_i2c_write_byte(0x11, 0x80);
	gsensor_i2c_write_byte(0x5f, 0x80);
	gsensor_i2c_write_byte(0x5f, 0x00);
	gsensor_i2c_write_byte(0x20, 0x05);
#if defined(QMA7981_6100_FIFO)
	gsensor_i2c_write_byte(0x31, 0x20);
	gsensor_i2c_write_byte(0x3E, 0x40);
	gsensor_i2c_write_byte(0x17, 0x20);
	gsensor_i2c_write_byte(0x1a, 0x20);
	gsensor_i2c_write_byte(0x20, 0x05);
#endif
#if defined(USE_SPI)
	//gsensor_i2c_write_byte(0x21, 0x21);
#else
	//gsensor_i2c_write_byte(0x21, 0x01);
#endif

	return KAL_TRUE;
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
#if defined(USE_QST_SW_IIC)
	i2c_CheckDevice(QMAX981_SLAVE_ADDR);
#endif
	memset(&g_qmaX981, 0, sizeof(g_qmaX981));
	g_qmaX981.layout = 0;
	qmaX981_set_layout(g_qmaX981.layout);
	g_qmaX981.chip_type = QMAX981_TYPE_UNKNOW;
	qmaX981_set_mode(1);
	qmaX981_soft_reset();
	qmaX981_set_mode(1);
	qmaX981_get_chipid();

	if(g_qmaX981.chip_type == QMAX981_TYPE_6981)
		bResult = qma6981_custom_init();
	else if(g_qmaX981.chip_type == QMAX981_TYPE_7981)
		bResult = qma7981_custom_init();
	else if(g_qmaX981.chip_type == QMAX981_TYPE_6100)
		bResult = qma6100_custom_init();
	else
		bResult = KAL_FALSE;

#if defined(QMAX981_USE_INT1)
	qmaX981_setup_int1();
#endif	

    if(bResult)
    {
    	#ifndef WIN32
        QMAX981_INFO(MOD_MATV, "[I]----[%s]: success!", __func__);
		#endif
    }
    else
    {
    	#ifndef WIN32
        QMAX981_ERROR(MOD_MATV, "[E]----[%s]: failure!", __func__);
		#endif
    }
#if defined(QMAX981_USE_CALI)
	qmax981_read_file(QMAX981_CALI_FILE, (kal_char *)qmax981_cali, sizeof(qmax981_cali));
#endif
}

kal_bool qmaX981_query_gesture(kal_uint16 ms_gest_type)
{
    kal_bool  bResult=KAL_FALSE;

	switch(ms_gest_type)
	{
		case MS_TAP:
			return KAL_FALSE;	
		case MS_STEP:
#if defined(QMAX981_STEPCOUNTER)
			bResult = qmaX981_custom_get_step(&g_qmaX981.step);
#endif
			return bResult;
		case MS_DROP:
			return KAL_FALSE;
		case MS_FLIP:
			return KAL_FALSE;
		default:
			return KAL_FALSE;
	}
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
	qmaX981_set_mode(1);  
}

void qmaX981_custom_pwr_down(void)
{
	qmaX981_set_mode(0);
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
	
	if((fd = FS_Open(filename, FS_READ_ONLY)) < FS_NO_ERROR)	  /* FS_CREATE */
	{
		FS_Close(fd);
		kal_prompt_trace(MOD_MATV, "%s open file error!\r\n", __func__);
		qmax981_calied_flag = 0;
		qmax981_cali[0] = 0;
		qmax981_cali[1] = 0;
		qmax981_cali[2] = 0;
		return;
	}
	else
	{
		FS_Read(fd, data, len, NULL);
		FS_Close(fd);
		kal_prompt_trace(MOD_MATV, "%s open file OK!\r\n", __func__);
		qmax981_calied_flag = 1;
		return;
	}
}

void qmax981_do_cali(void)
{
	kal_int32 data[3], data_avg[3];
	int icount, z_max, z_min;

	data_avg[0] = 0;
	data_avg[1] = 0;
	data_avg[2] = 0;

	qmax981_calied_flag = 0;	
	qmax981_cali[0] = 0;
	qmax981_cali[1] = 0;
	qmax981_cali[2] = 0;
	for(icount=0; icount<QMAX981_CALI_NUM; icount++)
	{
		qmaX981_get_raw_data(data);
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
		gsensor_delay_ms(5);
	}
	// add by yangzhiqiang check vibrate
	if((z_max-z_min)>(g_qmaX981.lsb_1g*3/10))
	{
		kal_prompt_trace(MOD_MATV, "qmax981_cali_store check vibrate cali ingore!\n");
		return;
	}
	// add by yangzhiqiang check vibrate

	data_avg[0] = data_avg[0]/QMAX981_CALI_NUM;
	data_avg[1] = data_avg[1]/QMAX981_CALI_NUM;
	data_avg[2] = data_avg[2]/QMAX981_CALI_NUM;

	data[0] = 0-data_avg[0];
	data[1] = 0-data_avg[1];
	data[2] = g_qmaX981.lsb_1g-data_avg[2];
	qmax981_cali[0] = data[0];
	qmax981_cali[1] = data[1];
	qmax981_cali[2] = data[2];
	kal_prompt_trace(MOD_MATV, "qmax981_cali_store qmax981_cali[%d %d %d]\n", qmax981_cali[0], qmax981_cali[1], qmax981_cali[2]);
	qmax981_write_file(QMAX981_CALI_FILE, (kal_char *)qmax981_cali, sizeof(qmax981_cali));	
	qmax981_calied_flag = 1;
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


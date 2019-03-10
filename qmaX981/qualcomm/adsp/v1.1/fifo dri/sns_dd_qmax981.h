/*******************************************************************************
 * Copyright (c) 2017, QST Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/



#ifndef _SNS_DD_QMAX981_REF_H
#define _SNS_DD_QMAX981_REF_H

#include <stdlib.h>
#include "sns_ddf_attrib.h"
#include "sns_ddf_comm.h"
#include "sns_ddf_common.h"
#include "sns_ddf_driver_if.h"
#include "sns_ddf_memhandler.h"
#include "sns_ddf_signal.h"
#include "sns_ddf_smgr_if.h"
#include "sns_ddf_util.h"
#include "qurt_elite_diag.h"
#include "sns_log_api_public.h"
#include "sns_log_types_public.h"
#include "fixed_point.h"



/* Enable this flag to enable uImage operation */
#define QMAX981_ENABLE_UIMAGE_SUPPORT

#ifndef QMAX981_ENABLE_UIMAGE_SUPPORT
/* Define extended DDF uImage functions if missing in HD22 package */
#define sns_ddf_malloc_ex(ptr, size, smgr_handle)               		sns_ddf_malloc(ptr, size)
#define sns_ddf_mfree_ex(ptr, smgr_handle)                              sns_ddf_mfree(ptr)
#define sns_ddf_memhandler_malloc_ex(memhandler, size, smgr_handle)     sns_ddf_memhandler_malloc(memhandler, size)
//#define sns_ddf_smgr_is_in_uimage()     false
#endif

/* Enable this flag when compiling on OpenSSC HD22 builds */
//#define QST_COMPILE_FOR_HD22

#ifdef QST_COMPILE_FOR_HD22
#define QMAX981_LOG_PACKET SNS_LOG_CONVERTED_SENSOR_DATA_PUBLIC
//sns_ddf_driver_if_s sns_dd_vendor_if_2;
//#define SNS_DD_QMAX981_IF sns_dd_vendor_if_2
#else
#define QMAX981_LOG_PACKET SNS_LOG_CONVERTED_SENSOR_DATA
//sns_ddf_driver_if_s sns_dd_qmax981_if;
//#define SNS_DD_QMAX981_IF sns_dd_qmax981_if
#endif


#define QMA6981_CHIP_ID 0xB0
#define QMA7981_CHIP_ID 0XF9

#define QMAX981_CHIP_ID_REG                      0x00
#define QMAX981_X_AXIS_LSB_REG                   0x01
#define QMAX981_X_AXIS_MSB_REG                   0x02
#define QMAX981_Y_AXIS_LSB_REG                   0x03
#define QMAX981_Y_AXIS_MSB_REG                   0x04
#define QMAX981_Z_AXIS_LSB_REG                   0x05
#define QMAX981_Z_AXIS_MSB_REG                   0x06
#define QMAX981_STEP_CNT_LSB_REG                 0x07
#define QMAX981_STEP_CNT_MSB_REG                 0X08
#define QMAX981_INT_STAT0_REG                    0x0A
#define QMAX981_INT_STAT1_REG                    0x0B
#define QMAX981_INT_STAT2_REG                    0x0C
#define QMAX981_INT_STAT3_REG                    0x0D
#define QMAX981_STATUS_FIFO_REG                  0x0E
#define QMAX981_RANGE_SEL_REG                    0x0F
#define QMAX981_BW_SEL_REG                       0x10
#define QMAX981_PWR_CTRL_REG                     0x11
#define QMAX981_STEP_CONF0_REG                   0x12
#define QMAX981_STEP_CONF1_REG                   0x13
#define QMAX981_STEP_TIME_LOW_REG                0x14
#define QMAX981_STEP_TIME_UP_REG                 0x15
#define QMAX981_INT_ENABLE0_REG                  0x16
#define QMAX981_INT_ENABLE1_REG                  0x17
#define QMAX981_INT_SRC_REG                      0x18
#define QMAX981_INT_MAP_0_REG                    0x19
#define QMAX981_INT_MAP_1_REG                    0x1a
#define QMAX981_INT_MAP_2_REG                    0x1b
#define QMAX981_INT_MAP_3_REG                    0x1c
#define QMAX981_INT_SET_REG                      0x20
#define QMAX981_INT_CTRL_REG                     0x21
#define QMAX981_LOW_DURN_REG                     0x22
#define QMAX981_LOW_THRES_REG                    0x23
#define QMAX981_LOW_HIGH_HYST_REG                0x24
#define QMAX981_HIGH_DURN_REG                    0x25
#define QMAX981_HIGH_THRES_REG                   0x26
#define QMAX981_OS_CUST_X                        0x27
#define QMAX981_OS_CUST_Y                        0x28
#define QMAX981_OS_CUST_Z                        0x29
#define QMAX981_TAP_PARAM_REG                    0x2A
#define QMAX981_TAP_THRES_REG                    0x2B
#define QMAX981_4D6D_CONF0_REG                   0x2c
#define QMAX981_4D6D_CONF1_REG                   0x2d
#define QMAX981_4D6D_CONF2_REG                   0x2e
#define QMAX981_4D6D_CONF3_REG                   0x2f
#define QMAX981_4D6D_CONF4_REG                   0x30
#define QMAX981_FIFO_WML_TRIG                    0x31
#define QMAX981_SELF_TEST_REG                    0x32
#define QMAX981_NVM_CTRL_REG                     0x33
#define QMAX981_RESET_REG                        0x36
#define QMAX981_FIFO_MODE_REG                    0x3E
#define QMAX981_FIFO_DATA_OUTPUT_REG             0x3F

/* Accelerometer QMA6981 ranges, converted to Q16. */
#define QMAX981_ACCEL_SENSOR_RANGE_2G_MAX         FX_FLTTOFIX_Q16(2*G)
#define QMAX981_ACCEL_SENSOR_RANGE_2G_MIN         FX_FLTTOFIX_Q16(-2*G)
#define QMAX981_ACCEL_SENSOR_RANGE_4G_MAX         FX_FLTTOFIX_Q16(4*G)
#define QMAX981_ACCEL_SENSOR_RANGE_4G_MIN         FX_FLTTOFIX_Q16(-4*G)
#define QMAX981_ACCEL_SENSOR_RANGE_8G_MAX         FX_FLTTOFIX_Q16(8*G)
#define QMAX981_ACCEL_SENSOR_RANGE_8G_MIN         FX_FLTTOFIX_Q16(-8*G)

#define QMAX981_MAX_BW                  7
#define QMAX981_ACCEL_SENSOR_BW_VALUE_7_81Hz      FX_FLTTOFIX_Q16(7.81)
#define QMAX981_ACCEL_SENSOR_BW_VALUE_15_63Hz     FX_FLTTOFIX_Q16(15.63)
#define QMAX981_ACCEL_SENSOR_BW_VALUE_31_25Hz     FX_FLTTOFIX_Q16(31.25)
#define QMAX981_ACCEL_SENSOR_BW_VALUE_62_50Hz     FX_FLTTOFIX_Q16(62.5)
#define QMAX981_ACCEL_SENSOR_BW_VALUE_125Hz       FX_FLTTOFIX_Q16(125)
#define QMAX981_ACCEL_SENSOR_BW_VALUE_250Hz       FX_FLTTOFIX_Q16(250)
#define QMAX981_ACCEL_SENSOR_BW_VALUE_500Hz       FX_FLTTOFIX_Q16(500)
#define QMAX981_ACCEL_SENSOR_BW_VALUE_1000Hz      FX_FLTTOFIX_Q16(1000)

#define QMA6981_ACTIVE_CURRENT          220
#define QMA6981_LOWPW_CURRENT         2

#define QST_ACC_NUM_AXIS    3
#define QMA6981_RANGE_NUM    3
#define QMA6981_BIT_LEN     10
#define QMA6981_MAX_FREQ    500

#define QMA6981_MAX_FIFO_LEVEL 32
#define QMA6981_TAP_SUPPORT

/* Accelerometer QMA6981 Full Scales in register setting. */
typedef enum
{
    QMA6981_RANGE_2G   = 0x01,  /*corresponding value in register setting*/
    QMA6981_RANGE_4G   = 0x02,
    QMA6981_RANGE_8G   = 0x04,
} qma6981_range;


/* Accelerometer QMA6981 sensitivity for each range. */
 typedef enum
{
    QMA6981_SENS_2G  = 2,
    QMA6981_SENS_4G  = 4,
    QMA6981_SENS_8G  = 8,
}qma6981_sens;


/* Accelerometer QMA6981 output data rate in register setting */
 typedef enum
{
    QMAX981_REGV_BW_3_9HZ   =     0x00,
    QMAX981_REGV_BW_7_8HZ   =     0x01,
    QMAX981_REGV_BW_15_6HZ  =     0x02,
    QMAX981_REGV_BW_31_2HZ  =     0x03,
    QMAX981_REGV_BW_62_5HZ  =     0x04,
    QMAX981_REGV_BW_125HZ   =     0x05,
    QMAX981_REGV_BW_250HZ   =     0x06,
    QMAX981_REGV_BW_500HZ   =     0x07
} qma6981_odr;

/* Accelerometer QMA6981 Driver State Information Structure*/

typedef struct {
	uint32_t rate;
	uint32_t reg;
}qma6981_odr_bw_map_t;

typedef struct {
    /*< Handle used with sns_ddf_notify_data.*/
    sns_ddf_handle_t smgr_hndl;
    /*< Handle used to access the I2C bus. */
    sns_ddf_handle_t port_handle;
    sns_ddf_device_access_s*  		dev_info;
    /* timer obj to be used  */
    sns_ddf_timer_s          sns_dd_tmr_obj;
    /*< Device-to-phone axis mapping. */
    sns_ddf_axes_map_s axes_map;
    /*< bias setting. */
    q16_t bias[QST_ACC_NUM_AXIS];
    /*< data cache for storing sensor data. */
    q16_t data_cache[QST_ACC_NUM_AXIS];
    q16_t data_cache_fifo[QST_ACC_NUM_AXIS];

    //sns_ddf_sensor_data_s   sensor_data;
    sns_ddf_sensor_data_s   f_frames_cache;
    /*< Current sensitivity. */
    qma6981_sens sstvt_adj;

    /*< Current range selection.*/
    qma6981_range range;

    qma6981_odr data_rate;

    uint32_t		gpio_num;
	uint8_t			fifo_level;
	uint8_t			fifo_int_config;
	sns_ddf_time_t	irq_ts_start;
	sns_ddf_time_t	irq_ts_end;
	bool			tap_irq_flag;	
	//uint8_t			fifo_report_flag;

    /*< Current power state: ACTIVE or LOWPOWER */
    sns_ddf_powerstate_e power_state;

    uint8_t device_select;
    sns_ddf_odr_t acc_cur_rate;

} sns_dd_qmax981_state_t;



/*Funtion Prototypes Declaration*/
sns_ddf_status_e sns_dd_qmax981_init(
    sns_ddf_handle_t* dd_handle_ptr,
    sns_ddf_handle_t smgr_handle,
    sns_ddf_nv_params_s* nv_params,
    sns_ddf_device_access_s device_info[],
    uint32_t num_devices,
    sns_ddf_memhandler_s* memhandler,
    sns_ddf_sensor_e* sensors[],
    uint32_t* num_sensors);

sns_ddf_status_e sns_dd_qmax981_reset(sns_ddf_handle_t dd_handle);

sns_ddf_status_e sns_dd_qmax981_set_attr(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_attribute_e attrib,
    void* value);

sns_ddf_status_e sns_dd_qmax981_get_attr(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_attribute_e attrib,
    sns_ddf_memhandler_s* memhandler,
    void** value,
    uint32_t* num_elems);

sns_ddf_status_e sns_dd_qmax981_self_test(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_test_e test,
    uint32_t* err);

sns_ddf_status_e sns_dd_qmax981_probe(
    sns_ddf_device_access_s*  device_info,
    sns_ddf_memhandler_s*     memhandler,
    uint32_t*                 num_sensors,
    sns_ddf_sensor_e**        sensors );

sns_ddf_status_e sns_dd_qmax981_get_data(
    sns_ddf_handle_t        dd_handle,
    sns_ddf_sensor_e        sensors[],
    uint32_t                num_sensors,
    sns_ddf_memhandler_s*   memhandler,
    sns_ddf_sensor_data_s*  data[]
);

sns_ddf_status_e sns_dd_qmax981_enable_sched_data(
        sns_ddf_handle_t    dd_handle,
        sns_ddf_sensor_e    sensor_type,
        bool                enable);


void sns_dd_qmax981_handle_timer(
            sns_ddf_handle_t dd_handle,
            void* arg);

void sns_dd_qmax981_handle_irq(
        sns_ddf_handle_t       dd_handle,
        uint32_t               gpio_num,
        sns_ddf_time_t         timestamp);

sns_ddf_status_e sns_dd_qmax981_trigger_fifo_data(
        sns_ddf_handle_t    dd_handle,
        sns_ddf_sensor_e    sensor,
        uint16_t            num_samples,
        bool                trigger_now);

sns_ddf_status_e qmax981_read_bytes(
        sns_ddf_handle_t 	port_handle,
        uint8_t	 			reg_addr,
        uint8_t*			data,
        uint8_t 			len);
sns_ddf_status_e qmax981_write_byte(
        sns_ddf_handle_t 	port_handle,
        uint8_t 			reg_addr,
        uint8_t*			data);


// Enable the following macro to see debug out
#define SNS_QMAX981_DEBUG

#ifdef SNS_QMAX981_DEBUG

#if defined QDSP6

#ifndef QMAX981_ENABLE_UIMAGE_SUPPORT
#include "msg.h"
#define DD_LOW_PRIO   MSG_LEGACY_LOW    /*< Low priority debug message. */
#define DD_MED_PRIO   MSG_LEGACY_MED    /*< Medium priority debug message. */
#define DD_HIGH_PRIO  MSG_LEGACY_HIGH   /*< High priority debug message. */
#define DD_ERROR_PRIO MSG_LEGACY_ERROR  /*< Error priority debug message. */
#define DD_FATAL_PRIO MSG_LEGACY_FATAL  /*< Fatal priority debug message. */
#define DD_MEDIUM_PRIO DBG_MED_PRIO
#define QMAX981_MSG_0(level,msg)          MSG(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg)
#define QMAX981_MSG_1(level,msg,p1)       MSG_1(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1)
#define QMAX981_MSG_2(level,msg,p1,p2)    MSG_2(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1,p2)
#define QMAX981_MSG_3(level,msg,p1,p2,p3) MSG_3(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1,p2,p3)
#define QMAX981_STRMSG_1(level,msg,p1)    MSG_SPRINTF_1(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1)

#else

#include "msg.h" // Lyon
#define DD_LOW_PRIO   MSG_LEGACY_LOW    /*< Low priority debug message. */
#define DD_MED_PRIO   MSG_LEGACY_MED    /*< Medium priority debug message. */
#define DD_HIGH_PRIO  MSG_LEGACY_HIGH   /*< High priority debug message. */
#define DD_ERROR_PRIO MSG_LEGACY_ERROR  /*< Error priority debug message. */
#define DD_FATAL_PRIO MSG_LEGACY_FATAL  /*< Fatal priority debug message. */
#define DD_MEDIUM_PRIO DBG_MED_PRIO
//#define QMAX981_MSG_0(level,msg)          UMSG(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg)
//#define QMAX981_MSG_1(level,msg,p1)       UMSG_1(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1)
//#define QMAX981_MSG_2(level,msg,p1,p2)    UMSG_2(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1,p2)
//#define QMAX981_MSG_3(level,msg,p1,p2,p3) UMSG_3(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1,p2,p3)
//#define QMAX981_STRMSG_1(level,msg,p1)    UMSG_SPRINTF_1(MSG_SSID_SNS,DD_##level##_PRIO, "QMAX981: " msg,p1)
#define QMAX981_MSG_0(level,msg)          
#define QMAX981_MSG_1(level,msg,p1)       
#define QMAX981_MSG_2(level,msg,p1,p2)    
#define QMAX981_MSG_3(level,msg,p1,p2,p3) 
#define QMAX981_STRMSG_1(level,msg,p1)    
#endif //End of QMAX981_ENABLE_UIMAGE_SUPPORT

#else
#define MED MEDIUM
#include "sns_debug_str.h"
#define QMAX981_MSG_0(level,msg)          SNS_PRINTF_STRING_ID_##level##_0(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING0)
#define QMAX981_MSG_1(level,msg,p1)       SNS_PRINTF_STRING_ID_##level##_1(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING1,p1)
#define QMAX981_MSG_2(level,msg,p1,p2)    SNS_PRINTF_STRING_ID_##level##_2(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING2,p1,p2)
#define QMAX981_MSG_3(level,msg,p1,p2,p3) SNS_PRINTF_STRING_ID_##level##_3(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING3,p1,p2,p3)
#endif // End of QDSP6

#else
#define QMAX981_MSG_0(level,msg)
#define QMAX981_MSG_1(level,msg,p1)
#define QMAX981_MSG_2(level,msg,p1,p2)
#define QMAX981_MSG_3(level,msg,p1,p2,p3)
#endif // End of SNS_QMAX981_DEBUG

#endif /* End include guard  _SNS_DD_QMAX981_REF_H */


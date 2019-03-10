 /*  Date: 2015/12/22 14:00:00
  *  Revision: 1.4.4
  */








/*******************************************************************************
* Copyright (c) 2013, <Bosch Sensortec GmbH>
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

/*-----------------------------------------------------------------------------
 * Copyright (c) 2013-2015 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
  -----------------------------------------------------------------------------*/

/*==============================================================================

  S E N S O R S   P R E S S U R E	D R I V E R

DESCRIPTION

 Impelements the driver for the QMP6988 driver

==============================================================================*/


#ifndef __QMP6988_H__
#define __QMP6988_H__

#define BMP_NAME "qmp6988"

#define QMP6988_CONFIG_RUN_ON_OSSC		0
#define QMP6988_CONFIG_ENABLE_UIMAGE	1

#if QMP6988_CONFIG_RUN_ON_OSSC
#include "fixed_point.h"

//#include "qurt_elite_diag.h"
#include "sns_ddf_signal.h"
#include "sns_ddf_attrib.h"
#include "sns_ddf_comm.h"
#include "sns_ddf_common.h"
#include "sns_ddf_driver_if.h"
#include "sns_ddf_memhandler.h"
#include "sns_ddf_signal.h"
#include "sns_ddf_smgr_if.h"
#include "sns_ddf_util.h"
#include "stdbool.h"
#include <string.h>
#include "sns_log_api_public.h"
#include "sns_log_types_public.h"
//#include "sns_dd_inertial_test.h"
#else
#include "fixed_point.h"

#include "sns_memmgr.h"
#include "sns_ddf_attrib.h"
#include "sns_ddf_common.h"
#include "sns_ddf_comm.h"
#include "sns_ddf_driver_if.h"
#include "sns_ddf_memhandler.h"
#include "sns_ddf_smgr_if.h"
#include "sns_ddf_util.h"
//#include "sns_dd_inertial_test.h"
#include "sns_ddf_signal.h"
#include "sns_log_types.h"
#include "sns_log_api.h"
#include "sns_debug_str.h"
#include "sns_debug_api.h"
#endif


#if !QMP6988_CONFIG_ENABLE_UIMAGE
#define sns_ddf_malloc_ex(ptr, size, handle)                            sns_ddf_malloc(ptr, size)
#define sns_ddf_memhandler_malloc_ex(mem_handler, size, handle)         sns_ddf_memhandler_malloc(mem_handler, size)
#endif

/*!
 * @brief  Reported Temprature resolution is 0.01°C (Celsius).
 */
 extern const q16_t qmp6988_temperature_resolution;

/*!
 * @brief Reported Pressure resolution is 1.0 Pa (Pascal) = 0.01 hPa.
 * Expected range 80K to 120K Pascals.
 */
 extern const q16_t qmp6988_pressure_resolution;

#define DD_ATTR_LOCAL static


//#define QMP6988_DEBUG
//#define PRIMITIVE_DEBUG_MSG

#define DBG_MEDIUM_PRIO DBG_MED_PRIO

#ifdef QMP6988_DEBUG
#ifndef PRIMITIVE_DEBUG_MSG
#if !QMP6988_CONFIG_ENABLE_UIMAGE
#define QMP6988_MSG_0(level,msg)          MSG(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMP6988 - " msg)
#define QMP6988_MSG_1(level,msg,p1)       MSG_1(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMP6988 - " msg,p1)
#define QMP6988_MSG_2(level,msg,p1,p2)    MSG_2(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2)
#define QMP6988_MSG_3(level,msg,p1,p2,p3) MSG_3(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#define QMP6988_MSG_3_P(level,msg,p1,p2,p3) MSG_3(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#define QMP6988_MSG_3_F(level,msg,p1,p2,p3)  MSG_3(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#define QMP6988_MSG_4(level,msg,p1,p2,p3,p4)  MSG_4(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3,p4)
#else   //QMP6988_CONFIG_ENABLE_UIMAGE
#define QMP6988_MSG_0(level,msg)          UMSG(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg)
#define QMP6988_MSG_1(level,msg,p1)       UMSG_1(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1)
#define QMP6988_MSG_2(level,msg,p1,p2)    UMSG_2(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2)
#define QMP6988_MSG_3(level,msg,p1,p2,p3) UMSG_3(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#define QMP6988_MSG_3_P(level,msg,p1,p2,p3) UMSG_3(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#define QMP6988_MSG_3_F(level,msg,p1,p2,p3)  UMSG_3(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#define QMP6988_MSG_4(level,msg,p1,p2,p3,p4)  UMSG_4(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3,p4)
#endif  //QMP6988_CONFIG_ENABLE_UIMAGE
#else
#define QMP6988_MSG_0(level,msg)
#define QMP6988_MSG_1(level,msg,p1)
#define QMP6988_MSG_2(level,msg,p1,p2)
#define QMP6988_MSG_3(level,msg,p1,p2,p3)
#define QMP6988_MSG_4(level,msg,p1,p2,p3,p4)
//#define QMP6988_MSG_3_P(level,msg,p1,p2,p3)
#define QMP6988_MSG_3_P(level,msg,p1,p2,p3) //SNS_PRINTF_STRING_ID_##level##_3(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING3,p1,p2,p3)

#define MED MEDIUM
#endif
#else
#define QMP6988_MSG_0(level,msg)
#define QMP6988_MSG_1(level,msg,p1)
#define QMP6988_MSG_2(level,msg,p1,p2)
#define QMP6988_MSG_3(level,msg,p1,p2,p3)
#define QMP6988_MSG_4(level,msg,p1,p2,p3,p4)
#define QMP6988_MSG_3_P(level,msg,p1,p2,p3) //MSG_3(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#define QMP6988_MSG_3_F(level,msg,p1,p2,p3)  //MSG_3(MSG_SSID_SNS,DBG_##level##_PRIO, "QMP6988 - " msg,p1,p2,p3)
#endif

#define BST_ARRAY_SIZE(array)   (sizeof(array)/sizeof(array[0]))

struct bst_odr_ts_map {
    uint8_t odr;
    int32_t interval;
};

struct bst_val_pair {
    uint32_t l;
    uint32_t r;
};



//#define NULL ((void *)0)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define QMP6988_SLAVE_ADDRESS				(0x70<<1)

#define QMP6988_CHIP_ID						0x5C
#define QMP6988_CHIP_ID_REG					0xD1
#define QMP6988_RESET_REG             		0xE0  /* Device reset register */
#define QMP6988_DEVICE_STAT_REG             0xF3  /* Device state register */
#define QMP6988_CTRLMEAS_REG                0xF4  /* Measurement Condition Control Register */
#define QMP6988_IOSETUP_REG                 0xF5
/* data */
#define QMP6988_PRESSURE_MSB_REG            0xF7  /* Pressure MSB Register */
#define QMP6988_TEMPERATURE_MSB_REG         0xFA  /* Temperature MSB Reg */

/* compensation calculation */
#define QMP6988_CALIBRATION_DATA_START      0xA0 /* QMP6988 compensation coefficients */
#define QMP6988_CALIBRATION_DATA_LENGTH		25

#define SHIFT_RIGHT_4_POSITION				 4
#define SHIFT_LEFT_2_POSITION                2
#define SHIFT_LEFT_4_POSITION                4
#define SHIFT_LEFT_5_POSITION                5
#define SHIFT_LEFT_8_POSITION                8
#define SHIFT_LEFT_12_POSITION               12
#define SHIFT_LEFT_16_POSITION               16

/* power mode */
#define QMP6988_SLEEP_MODE                    0x00
#define QMP6988_FORCED_MODE                   0x01
#define QMP6988_NORMAL_MODE                   0x03

#define QMP6988_CTRLMEAS_REG_MODE__POS        0
#define QMP6988_CTRLMEAS_REG_MODE__MSK        0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN        2

/* oversampling */
#define QMP6988_OVERSAMPLING_SKIPPED          0x00
#define QMP6988_OVERSAMPLING_1X               0x01
#define QMP6988_OVERSAMPLING_2X               0x02
#define QMP6988_OVERSAMPLING_4X               0x03
#define QMP6988_OVERSAMPLING_8X               0x04
#define QMP6988_OVERSAMPLING_16X              0x05
#define QMP6988_OVERSAMPLING_32X              0x06
#define QMP6988_OVERSAMPLING_64X              0x07


#define QMP6988_CTRLMEAS_REG_OSRST__POS             5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK             0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN             3


#define QMP6988_CTRLMEAS_REG_OSRSP__POS             2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK             0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN             3


/* filter */
#define QMP6988_FILTERCOEFF_OFF               0x00
#define QMP6988_FILTERCOEFF_2                 0x01
#define QMP6988_FILTERCOEFF_4                 0x02
#define QMP6988_FILTERCOEFF_8                 0x03
#define QMP6988_FILTERCOEFF_16                0x04
#define QMP6988_FILTERCOEFF_32                0x05


#define QMP6988_CONFIG_REG                    0xF1  /*IIR filter co-efficient setting Register*/

#define QMP6988_CONFIG_REG_FILTER__POS        0
#define QMP6988_CONFIG_REG_FILTER__MSK        0x07
#define QMP6988_CONFIG_REG_FILTER__LEN        3


#define SUBTRACTOR 8388608



#define QMP6988_WR_FUNC_PTR\
    sns_ddf_status_e (*bus_write)(unsigned char, unsigned char,\
            unsigned char *, unsigned char)

#define QMP6988_RD_FUNC_PTR\
    sns_ddf_status_e (*bus_read)(unsigned char, unsigned char,\
            unsigned char *, unsigned char)

#define QMP6988_MDELAY_DATA_TYPE uint32_t
#define QMP6988_U16_t uint16_t
#define QMP6988_S16_t int16_t
#define QMP6988_U32_t uint32_t
#define QMP6988_S32_t int32_t
#define QMP6988_U64_t uint64_t
#define QMP6988_S64_t int64_t

/** this structure holds all device specific calibration parameters */

struct qmp6988_calibration_data {
	QMP6988_S32_t COE_a0;
	QMP6988_S16_t COE_a1;
	QMP6988_S16_t COE_a2;
	QMP6988_S32_t COE_b00;
	QMP6988_S16_t COE_bt1;
	QMP6988_S16_t COE_bt2;
	QMP6988_S16_t COE_bp1;
	QMP6988_S16_t COE_b11;
	QMP6988_S16_t COE_bp2;
	QMP6988_S16_t COE_b12;
	QMP6988_S16_t COE_b21;
	QMP6988_S16_t COE_bp3;
};

/** QMP6988 image registers data structure */

typedef enum
{
    QMP6988_MODE_ULTRA_LOW_POWER = 0,
    QMP6988_MODE_LOW_POWER = 1,
    QMP6988_MODE_STANDARD = 2,
    QMP6988_MODE_HIGH_RESOLUTION = 3,
    QMP6988_MODE_ULTRA_HIGH_RESOLUTION = 4,
    QMP6988_NUM_MODES
} qmp6988_mode_e ;



/*!
 * @brief  Driver context.
 *
 *  @i2c_handle - i2c handle used to access the I2C bus.
 *  @smgr_handle - sensor manager handle used to notify_data.
 *  @timer - timer object used to delay until conversion completed.
 *  @mode - power and oversampling mode.
 *  @state - idle or measurement in progress.
 *  @chip_id - reported from the device.
 *  @version - reported from the device.
 *  @param_b5 - calculated parameter depends on temperature.
 *  @ut - uncompansated temperature.
 *  @up - uncompansated pressure.
 *  @temperature - in 0.1°C.
 *  @pressure - in Pascal.
 *  @calibration - calibration parameters.
 */
typedef struct  {
    sns_ddf_handle_t    i2c_handle;
    sns_ddf_handle_t    smgr_handle;
    qmp6988_mode_e 		mode;
    uint8_t				chip_id;
    q16_t				data_cache[2];
	uint32_t 	odr_reported;	
	double 		temperature;
	double		pressure;
} sns_dd_qmp6988_state_t;


#endif

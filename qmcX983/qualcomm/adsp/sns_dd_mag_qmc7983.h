/*===========================================================================

EDIT HISTORY FOR FILE

This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.

when         who     what, where, why
--------     ---     ------------------------------------------------------ 
==========================================================================*/

#ifndef _DD_QMC7983_H
#define _DD_QMC7983_H

#include <stdlib.h>
#include "fixed_point.h"
#include "sns_ddf_util.h"
#include "sns_ddf_attrib.h"
#include "sns_ddf_common.h"
#include "sns_ddf_comm.h"
#include "sns_ddf_driver_if.h"
#include "sns_ddf_smgr_if.h"
#include "sns_ddf_memhandler.h"
#include "sns_ddf_signal.h"
#include "sns_log_api_public.h"
#include "sns_log_types_public.h"

/*=======================================================================
INTERNAL DEFINITIONS
========================================================================*/

/* Enable this flag to enable uImage operation */
#define QMCX983_ENABLE_UIMG
#define QMCX983_SOFTIRON_COMP

#ifndef QMCX983_ENABLE_UIMG
/* Define extended DDF uImage functions if missing in HD22 package */
#define sns_ddf_malloc_ex(ptr, size, smgr_handle)               		sns_ddf_malloc(ptr, size)
#define sns_ddf_mfree_ex(ptr, smgr_handle)                              sns_ddf_mfree(ptr)
#define sns_ddf_memhandler_malloc_ex(memhandler, size, smgr_handle)     sns_ddf_memhandler_malloc(memhandler, size)
#define sns_ddf_smgr_is_in_uimage()     false
#endif

/* Enable this flag when compiling on OpenSSC HD22 builds */
//#define QST_COMPILE_FOR_HD22

#ifdef QST_COMPILE_FOR_HD22
#define QMCX983_LOG_PACKET SNS_LOG_CONVERTED_SENSOR_DATA_PUBLIC
sns_ddf_driver_if_s sns_dd_vendor_if_1;
#define SNS_DD_QMCX983_IF sns_dd_vendor_if_1
#else
#define QMCX983_LOG_PACKET SNS_LOG_CONVERTED_SENSOR_DATA
sns_ddf_driver_if_s sns_dd_qmc7983_if;
#define SNS_DD_QMCX983_IF sns_dd_qmc7983_if
#endif

#define QMCX983_SUPPORTED_SENSORS_NUM  1

#define QMC6983_A1_D1             	0
#define QMC6983_E1		  			1
#define QMC7983                   	2
#define QMC7983_SLOW_SETRESET		3

/* Enable the following macro to see debug out */
#define SNSD_MAG_QMCX983_DEBUG

#ifdef SNSD_MAG_QMCX983_DEBUG
#ifdef QDSP6
#ifdef QST_COMPILE_FOR_HD22
#include "qurt_elite_diag.h"
#define DBG_MEDIUM_PRIO DBG_MED_PRIO
#define QMC7983_DMSG_0(level,msg)          UMSG(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMCX983: " msg)
#define QMC7983_DMSG_1(level,msg,p1)       UMSG_1(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMCX983: " msg,p1)
#define QMC7983_DMSG_2(level,msg,p1,p2)    UMSG_2(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMCX983: " msg,p1,p2)
#define QMC7983_DMSG_3(level,msg,p1,p2,p3) UMSG_3(MSG_SSID_QDSP6,DBG_##level##_PRIO, "QMCX983: " msg,p1,p2,p3)
#else
#define DBG_MEDIUM_PRIO DBG_MED_PRIO
#define QMC7983_DMSG_0(level,msg)          UMSG(MSG_SSID_SNS,DBG_##level##_PRIO, "QMCX983: " msg)
#define QMC7983_DMSG_1(level,msg,p1)       UMSG_1(MSG_SSID_SNS,DBG_##level##_PRIO, "QMCX983: " msg,p1)
#define QMC7983_DMSG_2(level,msg,p1,p2)    UMSG_2(MSG_SSID_SNS,DBG_##level##_PRIO, "QMCX983: " msg,p1,p2)
#define QMC7983_DMSG_3(level,msg,p1,p2,p3) UMSG_3(MSG_SSID_SNS,DBG_##level##_PRIO, "QMCX983: " msg,p1,p2,p3)
#endif //End of QST_COMPILE_FOR_HD22
#else
#define MED MEDIUM
#include "sns_debug_str.h"
#define QMC7983_DMSG_0(level,msg)          SNS_PRINTF_STRING_ID_##level##_0(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING0)
#define QMC7983_DMSG_1(level,msg,p1)       SNS_PRINTF_STRING_ID_##level##_1(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING1,p1)
#define QMC7983_DMSG_2(level,msg,p1,p2)    SNS_PRINTF_STRING_ID_##level##_2(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING2,p1,p2)
#define QMC7983_DMSG_3(level,msg,p1,p2,p3) SNS_PRINTF_STRING_ID_##level##_3(SNS_DBG_MOD_DSPS_SMGR,DBG_SMGR_GENERIC_STRING3,p1,p2,p3)
#endif // End of QDSP6
#else
#define QMC7983_DMSG_0(level,msg)
#define QMC7983_DMSG_1(level,msg,p1)
#define QMC7983_DMSG_2(level,msg,p1,p2)
#define QMC7983_DMSG_3(level,msg,p1,p2,p3)
#endif // End of SNSD_MAG_QMCX983_DEBUG


/* Use define to avoid numbers in code */
#define QMCX983_WRITE_ONE_BYTE                   1
#define QMCX983_READ_ONE_BYTE                    1
#define QMCX983_WRITE_TWO_BYTES                  2
#define QMCX983_READ_TWO_BYTES                   2

#define QMC7983_X_LSB_REG		0x00
#define QMC7983_X_MSB_REG		0x01
#define QMC7983_Y_LSB_REG		0x02
#define QMC7983_Y_MSB_REG		0x03
#define QMC7983_Z_LSB_REG		0x04
#define QMC7983_Z_MSB_REG		0x05
#define QMC7983_STATUS_1_REG	0x06
#define QMC7983_TEMP_LSB_REG    0X07
#define QMC7983_TEMP_MSB_REG	0x08
#define QMC7983_CONTROL_1_REG   0x09
#define QMC7983_CONTROL_2_REG	0x0A
#define QMC7983_SET_RESET_REG	0x0B
#define QMC7983_CHIPID_REG		0x0D

#define QMCX983_X_LSB_REG		QMC7983_X_LSB_REG
#define QMCX983_X_MSB_REG		QMC7983_X_MSB_REG
#define QMCX983_Y_LSB_REG		QMC7983_Y_LSB_REG
#define QMCX983_Y_MSB_REG		QMC7983_Y_MSB_REG
#define QMCX983_Z_LSB_REG		QMC7983_Z_LSB_REG
#define QMCX983_Z_MSB_REG		QMC7983_Z_MSB_REG
#define QMCX983_STATUS_1_REG	QMC7983_STATUS_1_REG
#define QMCX983_TEMP_LSB_REG    QMC7983_TEMP_LSB_REG
#define QMCX983_TEMP_MSB_REG	QMC7983_TEMP_MSB_REG
#define QMCX983_CONTROL_1_REG   QMC7983_CONTROL_1_REG
#define QMCX983_CONTROL_2_REG	QMC7983_CONTROL_2_REG
#define QMCX983_SET_RESET_REG	QMC7983_SET_RESET_REG
#define QMCX983_CHIPID_REG		QMC7983_CHIPID_REG

/* Supported datatypes */
typedef enum
{
   QMCX983_AXIS_X = 0,
   QMCX983_AXIS_Y,
   QMCX983_AXIS_Z,
   QMCX983_AXIS_NUMS,
} sdd_mag_data_type_e;

typedef enum
{
	OPT_KX = 0,
	OPT_KY,
	OPT_NUMS
}sns_dd_qmcx983_opt_e;

/* State struct for driver */
typedef struct
{
  	sns_ddf_handle_t         smgr_handle;                             /* SMGR handle */               
  	sns_ddf_handle_t         port_handle;                             /* Port handle for bus access */
  	uint8_t             	 device_select; 						  /* store the current connected device.*/
	int8_t					 opt_k[OPT_NUMS];								  /*Kx = opt_k[0],Ky = opt_k[1]*/
  	sns_ddf_timer_s          sns_dd_tmr_obj;                          /* timer obj */
  	q16_t                    data_cache[QMCX983_AXIS_NUMS];       /* data cache */
  	uint32_t                 gpio1;                                   /* GPIO for interrupt */
  	sns_ddf_odr_t            odr;                                     /* Sensor ODR setting */
  	uint16_t                 water_mark;                              /* FIFO water mark */
  	uint16_t                 range;                                   /* range index */
  	sns_ddf_axes_map_s       axes_map;
} sns_dd_qmc7983_state_t;

sns_ddf_status_e sns_dd_qmcx983_init
    (
    sns_ddf_handle_t*        dd_handle_ptr,
    sns_ddf_handle_t         smgr_handle,
    sns_ddf_nv_params_s*     nv_params,
    sns_ddf_device_access_s  device_info[],
    uint32_t                 num_devices,
    sns_ddf_memhandler_s*    memhandler,
    sns_ddf_sensor_e*        sensors[],
    uint32_t*                num_sensors
    );

sns_ddf_status_e sns_dd_qmcx983_get_data
    (
    sns_ddf_handle_t        dd_handle,
    sns_ddf_sensor_e        sensors[],
    uint32_t                num_sensors,
    sns_ddf_memhandler_s*   memhandler,
    sns_ddf_sensor_data_s*  data[] /* ignored by this async driver */
);

sns_ddf_status_e sns_dd_qmcx983_set_attrib
    (
    sns_ddf_handle_t     dd_handle,
    sns_ddf_sensor_e     sensor,
    sns_ddf_attribute_e  attrib,
    void*                value
    );

sns_ddf_status_e sns_dd_qmcx983_get_attrib
    (
    sns_ddf_handle_t     dd_handle,
    sns_ddf_sensor_e     sensor,
    sns_ddf_attribute_e  attrib,
    sns_ddf_memhandler_s* memhandler,
    void**               value,
    uint32_t*            num_elems
    );

void sns_dd_qmcx983_handle_timer 
    (
    sns_ddf_handle_t dd_handle, 
    void* arg
    );

void sns_dd_qmcx983_handle_irq
    (
    sns_ddf_handle_t dd_handle, 
    uint32_t          gpio_num,
    sns_ddf_time_t    timestamp
    );

sns_ddf_status_e sns_dd_qmcx983_reset
    (
    sns_ddf_handle_t dd_handle
    );

sns_ddf_status_e sns_dd_qmcx983_enable_sched_data
    (
    sns_ddf_handle_t  handle,
    sns_ddf_sensor_e  sensor,
    bool              enable
    );

sns_ddf_status_e sns_dd_qmcx983_run_test
    (
    sns_ddf_handle_t  dd_handle,
    sns_ddf_sensor_e  sensor,
    sns_ddf_test_e    test,
    uint32_t*         err
    );

sns_ddf_status_e sns_dd_qmcx983_probe
    (
    sns_ddf_device_access_s* device_info,
    sns_ddf_memhandler_s*    memhandler,
    uint32_t*                num_sensors,
    sns_ddf_sensor_e**       sensors
    );

sns_ddf_status_e sns_dd_qmcx983_trigger_fifo_data
(
    sns_ddf_handle_t         dd_handle,
    sns_ddf_sensor_e         sensor,
    uint16_t                 num_samples,
    bool                     trigger_now
);
void sns_dd_qmcx983_get_OPT(const sns_ddf_handle_t  handle);

#endif /* End include guard  _DD_QMC7983_H */


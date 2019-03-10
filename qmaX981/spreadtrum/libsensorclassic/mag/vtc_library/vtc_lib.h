/****************************************************************
*
* @file vtc_lib.h
*
* Copyright(C), 2016, Voltafield Corporation
*
****************************************************************/

#ifndef __VTC_LIB_H__
#define __VTC_LIB_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#include "vtc_type_def.h"

//#ifdef SUPPORT_POWER_ONLINE_CALIBRATION

//================================================================
typedef struct
{
    float ax;  //x value
    float ay;  //y value
    float az;  //z value
    float mx;  //x value
    float my;  //y value
    float mz;  //z value
    float ox;
    float oy;
    float oz;
    unsigned int odr; //samping rate
    char accuracy;
}AF_ValueTypeDef;

#define RESOLUTION_M 	0.2 // 0.05 // 0.2

extern FLOAT sfi_array[9];
extern FLOAT hdi_array[3];
extern INT16 mag_cali_check_err;

extern void vtc_get_lib_version(CHAR *version, CHAR *date);
extern void vtc_get_algo_version(CHAR *version, CHAR *date);
extern void vtc_set_mag_axis(INT16 *axis);
extern void vtc_get_magneticfield(FLOAT *mag, INT16 *status);
extern void vtc_get_orientation(FLOAT *ori, INT16 *status);
extern void vtc_get_gyroscope(FLOAT *gyr);
extern void vtc_get_rotation_vector(FLOAT *rov);
extern void vtc_get_gravity(FLOAT *gra);
extern void vtc_get_linear_acceleration(FLOAT *lin);
extern void vtc_algo_initial(void);
extern void vtc_odr_change(INT16 odr);
extern void mag_layout_adjustment(INT8 chip_device, INT8 chip_layout, FLOAT *mag);
extern BOOL vtc_algo_run(FLOAT *acc_data, FLOAT *mag_data, FLOAT *gyr_data,
                         UINT32 timestamp);

//================================================================

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* __VTC_LIB_H__ */

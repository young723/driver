/****************************************************************
*
* @file vtc_cust.h
*
* Copyright(C), 2016, Voltafield Corporation
*
****************************************************************/

#ifndef __VTC_CUST_H__
#define __VTC_CUST_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */


//#define LCD_BRIGHT_CALIBRATION
//#define SUPPORT_MAG_NOISE_FILTER
//#define CUSTOM_CONFIG_WIND
//#define CUSTOM_CONFIG_TRANSSION
//#define CUSTOM_CONFIG_HUAQIN
//#define SUPPORT_POWER_ONLINE_CALIBRATION
//#define SUPPORT_GYROSCOPE_AUTO_CALI
//#define ORI_FILTER_ON_TILT
//#define MAG_SET_ONCE

/****************************************************************
* Project Information
****************************************************************/
#define LIB_VERSION                 "VTCSTD04"
#define LIB_RLS_DATE                "20170504"

#define CUSTOMER_NAME               "Voltafield"
#define PROJECT_NAME                "Standard"

#define MAG_CHIP_DEVIVE             MAG_DEV_AF6133

/****************************************************************
* File path for recording calibration parametrs
****************************************************************/
#define MAG_CILIBRATION_RECORD_ENABLE   1
#define MAG_CILIBRATION_PATH    "/data/vtc_cali"

/****************************************************************
* The definition of sensor direction for vtc algorithm
*   1: use x-axis data of sensor
*   2: use y-axis data of sensor
*   3: use z-axis data of sensor
*
* for example:
*  #define ALGO_ACC_AXIS_X  -2  ==> x of algorithm uses -y of sensor data
*  #define ALGO_ACC_AXIS_Y   1  ==> y of algorithm uses x of sensor data
*  #define ALGO_ACC_AXIS_Z  -3  ==> z of algorithm uses -z of sensor data
****************************************************************/
//config layout of g-sensor(acc) for algorithm
#define ALGO_ACC_AXIS_X   1
#define ALGO_ACC_AXIS_Y   2
#define ALGO_ACC_AXIS_Z   3
//config layout of m-sensor(mag) for algorithm
#define ALGO_MAG_AXIS_X   1
#define ALGO_MAG_AXIS_Y   2
#define ALGO_MAG_AXIS_Z   3
//config layout of gyroscope for algorithm
#define ALGO_GYR_AXIS_X   1
#define ALGO_GYR_AXIS_Y   2
#define ALGO_GYR_AXIS_Z   3

/****************************************************************
* Calibration Option
*	  MAG_CALIBRATION_MAG_ONLY : use mag data only
*	  MAG_CALIBRATION_MAG_ORI  : use mag data and pitch/roll
*	  MAG_CALIBRATION_MAG_GYR  : use mag data and gyro data
*	  MAG_CALIBRATION_MAG_ORI_FSS  : use mag data and pitch/roll to implement FSS
****************************************************************/
enum mag_cali_mode {
	MAG_CALIBRATION_MAG_ONLY=0,
	MAG_CALIBRATION_MAG_ORI,
	MAG_CALIBRATION_MAG_GYR,
  MAG_CALIBRATION_MAG_ORI_FSS,
  MAG_CALIBRATION_MAG_SPHERE,
  MAG_CALIBRATION_MAG_GYR_FSS
};

#define MAG_CALIBRATION_METHOD  MAG_CALIBRATION_MAG_ONLY

/****************************************************************
* 9-axis compass (fusion) enable
*  0: disble (use 6-axis compass)
*  1: enable (use 9-axis compass)
****************************************************************/
#define ENABLE_9AXIS_COMPASS    0

/****************************************************************
* Support vtc soft-gyro function
*  0: disble
*  1: enable (run without real-Gyro only)
****************************************************************/
//#if (!ENABLE_9AXIS_COMPASS) && (MAG_CALIBRATION_METHOD != MAG_CALIBRATION_MAG_GYR)
  #define ENABLE_SOFTGYRO_FUNCTION    0
//#else
  //#define ENABLE_SOFTGYRO_FUNCTION    0
//#endif

/****************************************************************
* Optimal function switch
*
*    ENABLE_MAG_AXIS_ALIGNMENT : rotate xy plane to align axis and device
*    ENABLE_MAG_KALMAN_FILTER  : use kalman filter to reduce noise
*    ENABLE_MAG_MOVING_AVERAGE  : use moving average to smooth data
*    ENABLE_MAG_DATA_COMPENSATION : use special method to correct data
*
*    1: enable   0: disable
****************************************************************/
#define ENABLE_MAG_AXIS_ALIGNMENT       0///1
#define ENABLE_MAG_KALMAN_FILTER        0////1
#define ENABLE_MAG_MOVING_AVERAGE       0///1
#define ENABLE_MAG_DATA_COMPENSATION    0///1

/****************************************************************
* weight setting for moving average
*
* data(n) = data(n-1) + (data(n) - data(n-1))/(MVAVG_WEIGHT * odr)
* odr is output data rate
* 0 < MVAVG_WEIGHT <= 1
****************************************************************/
#define MAG_MVAVG_WEIGHT_MOVING     0.1
#define MAG_MVAVG_WEIGHT_STATIC     1.0
#define ORI_MVAVG_WEIGHT_MOVING     0.1
#define ORI_MVAVG_WEIGHT_STATIC     1.0

#define MVAVG_WEIGHT_SWITCH_ANGLE   2
/****************************************************************
*  quaternion weight of acc & mag
*
*  use STARTUP when sensor first start-up or calibrating parameters update
*  use NORMAL after fusion_start_delay_counter count down to 0
****************************************************************/
#define QUAT_WEIGHT_ACC_STARTUP     0.3
#define QUAT_WEIGHT_MAG_STARTUP     0.3

#define QUAT_WEIGHT_ACC_NORMAL     0.02
#define QUAT_WEIGHT_MAG_NORMAL    0.000

#define QUAT_WEIGHT_ACC_DELTA      (QUAT_WEIGHT_ACC_STARTUP-QUAT_WEIGHT_ACC_NORMAL)
#define QUAT_WEIGHT_MAG_DELTA      (QUAT_WEIGHT_MAG_STARTUP-QUAT_WEIGHT_MAG_NORMAL)

/****************************************************************
* data buffer size for calibration
* size = odr * CALI_DATA_SIZE_MULTIPLE
* size must be smaller than 300
****************************************************************/
#define CALI_DATA_SIZE_MULTIPLE   2.0

/****************************************************************
* fusion start delay time (second)
****************************************************************/
#define FUSION_START_DELAY_TIME   3

/****************************************************************
* configurations for accuracy judgement
*
*  45uT > MAG_INTERFERNECE_RANGE_2 > MAG_INTERFERNECE_RANGE_1 > 0
*
* accuracy is reduced as 2
*       magnetic field > (MAG_INTERFERNECE_RANGE_2 + Geomagnetic field)
*       or
*       magnetic field < (MAG_INTERFERNECE_RANGE_2 - Geomagnetic field)
*
* accuracy is reduced as 1
*       magnetic field > (MAG_INTERFERNECE_RANGE_1 + Geomagnetic field)
*       or
*       magnetic field < (MAG_INTERFERNECE_RANGE_1 - Geomagnetic fields)
*
* accuracy is reduced as 0
*       magnetic field > MAG_INTERFERNECE_THRESHOLD
****************************************************************/
enum accuracy_level{
  MAG_ACCURACY_UNRELIABLE = 0,
  MAG_ACCURACY_LOW = 1,
  MAG_ACCURACY_MEDIAN = 2,
  MAG_ACCURACY_HIGH = 3
};

#define MAG_INTERFERNECE_THRESHOLD   500  //uT
#define MAG_INTERFERNECE_RANGE_1     30   //uT
#define MAG_INTERFERNECE_RANGE_2     20   //uT

/****************************************************************
* Define effective range of soft-iron
*   SFI_XX, SFI_YY, SFI_ZZ are positive
****************************************************************/
#define SFI_XX_THRESHOLD_MAX     1.10
#define SFI_XX_THRESHOLD_MIN     0.70//0.90
#define SFI_YY_THRESHOLD_MAX     1.10
#define SFI_YY_THRESHOLD_MIN     0.70//0.90
#define SFI_ZZ_THRESHOLD_MAX     1.10
#define SFI_ZZ_THRESHOLD_MIN     0.70
#define SFI_YX_THRESHOLD_MAX     0.25//0.085
#define SFI_YX_THRESHOLD_MIN    -0.25//-0.085
#define SFI_ZX_THRESHOLD_MAX     0.35
#define SFI_ZX_THRESHOLD_MIN    -0.35
#define SFI_ZY_THRESHOLD_MAX     0.35
#define SFI_ZY_THRESHOLD_MIN    -0.35

/****************************************************************
* Default SFI matrix
****************************************************************/
#define DEFAULT_SFI_XX          1//0.9688// //1
#define DEFAULT_SFI_XY           -0.07//0
#define DEFAULT_SFI_XZ           0
#define DEFAULT_SFI_YX          -0.07//0// -0.1791//0
#define DEFAULT_SFI_YY          1// 1.0415///1
#define DEFAULT_SFI_YZ           0
#define DEFAULT_SFI_ZX           0//0.1190//0
#define DEFAULT_SFI_ZY           0//0.0032//0
#define DEFAULT_SFI_ZZ           1//0.9931///1

/****************************************************************
* Define threshold for radius of calibrated data
*
* MAX > 45uT,   0uT < MIN < 45uT,  WIDE > 5uT
****************************************************************/
#define CALI_RADIUS_THRESHOLD_MAX   80//60
#define CALI_RADIUS_THRESHOLD_MIN   15//25
#define CALI_RADIUS_THRESHOLD_WIDE  15

/****************************************************************
* Configurations for FSS (fast calibration)
*
*    1 > FSS_GYR_EFFECTIVE_RATIO > 0
*    40  > FSS_MAG_EFFECTIVE_STDEV > 2
****************************************************************/
#define FSS_GYR_EFFECTIVE_RATIO     0.6
#define FSS_MAG_EFFECTIVE_STDEV      10

/****************************************************************
* filter settings for softgyro function
*
*  ACC_FILTER_AVG_NUM: number to do moving average  (integer, range: 1~32)
*  ACC_FILTER_MID_NUM: number to do middle filter   (integer, range: 1~32)
*  MAG_FILTER_AVG_NUM: number to do moving average  (integer, range: 1~32)
*  MAG_FILTER_MID_NUM: number to do middle filter   (float, range: 1~32)
*  GYR_FILTER_EXPO_THRESHOLD: threshold to do exponential filter  (range: 0~1 degree)
*
****************************************************************/
#define ACC_FILTER_AVG_NUM           10
#define ACC_FILTER_MID_NUM           10
#define MAG_FILTER_AVG_NUM           10
#define MAG_FILTER_MID_NUM           10
#define GYR_FILTER_EXPO_THRESHOLD  0.25    // dps

/****************************************************************
* acc threshold for switching moving average
****************************************************************/
#define ZDB_ACC_SWITCH_THRESHOLD   0.07    // m*m/s

/****************************************************************
* The ratio of data size to discard when calibration fail
*   0.1 <= ratio <= 1
*
* example:
*     ratio = 0.1 and data size = 200
*
*     new 10% data (20p) and old 90% data (180p) to do calibration
****************************************************************/
#define CTN_DATA_DISCARD_RATIO  0.2
#define DSC_DATA_DISCARD_RATIO  0.2
#define FSS_DATA_DISCARD_RATIO  0.25

/****************************************************************
* Threshold to dectect motion for FSS
****************************************************************/
#define FSS_THRESHOLD_MAG     8    // uT
#define FSS_THRESHOLD_ORI    30    // degree
#define FSS_THRESHOLD_GYR    30    // degree

/****************************************************************
* Azimuth when pitch is from 90 to threshold is the same as one when pitch < 90
****************************************************************/
#define ROTATION_ANGLE_THRESHOLD  140

/****************************************************************
* Default optimal compensation mode ½Ç¶Èµ÷Õû£¬¿ÉÄÜ»á¶ªÊ§½Ç¶È
****************************************************************/
#define OPTIMAL_COMPENSATION_MODE  4//

/****************************************************************
* Gyroscope calibration threshold
*
* use 1 senond data to calculate stdev
****************************************************************/
#define GYRO_CALI_THRESHOLD        0.002   //rps

/****************************************************************
* API for vtc_mag_proc
****************************************************************/
extern INT16 vtc_debug_log[30];
extern INT16 vtc_cali_log[15];
extern FLOAT vtc_cae_log[10];

extern FLOAT cali_radius;

extern FLOAT dsc_mag_distance;
extern FLOAT dsc_pitch_roll_threshold;

extern FLOAT sfi_array[9];
extern FLOAT hdi_array[3];

extern INT16 vtc_algo_odr;
extern INT16 dsc_data_size;
extern INT16 ctn_data_size;
extern INT16 fss_data_size;
extern INT16 fss_data_num;

extern INT16 ctn_data_discard_size;
extern INT16 dsc_data_discard_size;
extern INT16 fss_data_discard_size;

extern INT16 dsc_reset_threshold;

extern FLOAT sfi_xx_threshold_max;
extern FLOAT sfi_xx_threshold_min;
extern FLOAT sfi_yy_threshold_max;
extern FLOAT sfi_yy_threshold_min;
extern FLOAT sfi_zz_threshold_max;
extern FLOAT sfi_zz_threshold_min;
extern FLOAT sfi_yx_threshold_max;
extern FLOAT sfi_yx_threshold_min;
extern FLOAT sfi_zx_threshold_max;
extern FLOAT sfi_zx_threshold_min;
extern FLOAT sfi_zy_threshold_max;
extern FLOAT sfi_zy_threshold_min;

extern FLOAT fss_gyr_effective_ratio;
extern FLOAT fss_mag_effective_stdev;

extern FLOAT fss_mag_threshold;
extern FLOAT fss_ori_threshold;

extern FLOAT fss_radius;
extern FLOAT fss_radius_ratio;

extern FLOAT fss_mag_threshold;
extern FLOAT fss_ori_threshold;
extern FLOAT fss_gyr_threshold;

extern INT16 mag_quad_accuracy;
extern INT16 mag_cali_check_err;
extern INT16 mag_cali_check_ctn;

extern void vtc_algo_version(CHAR *version, CHAR *date);
extern void clear_data_buffer(INT16 mode);
extern void set_mag_cali_radius_range(FLOAT max, FLOAT min, FLOAT wide);
extern BOOL mag_calibration(FLOAT *mag, FLOAT *ori, FLOAT *gyr, FLOAT gdt,
                                       INT16 mode, FLOAT *sfi, FLOAT *hdi);

/****************************************************************
* API for vtc_ori_algo
****************************************************************/
extern BOOL ori_calculation(FLOAT *fusion_data,
                            FLOAT *acc, FLOAT *mag, FLOAT *gyr,
                            INT16 flag, FLOAT dt, FLOAT *ori);

/****************************************************************
* API for vtc_fusion
****************************************************************/
extern FLOAT qacc_weight;
extern FLOAT qmag_weight;

/****************************************************************
* API for vtc_chip
****************************************************************/
extern FLOAT mag_field_range;
extern void vtc_chip_initial(INT16 chip_device);
extern void sensor_placement_adjustment(FLOAT *data, INT16 *axis);

/****************************************************************
* API for vtc_optimal
****************************************************************/
extern INT16 m_kmf_start_delay;
extern FLOAT m_kmf_output[3];

extern FLOAT m_mvavg_weight_fast;
extern FLOAT m_mvavg_weight_slow;
extern FLOAT magnet_pre[3];

extern FLOAT o_mvavg_weight_fast;
extern FLOAT o_mvavg_weight_slow;
extern FLOAT o_mvavg_weight_switch_angle;

extern void config_optimal_mode(INT16 mode);
extern void optimal_mag_filter(FLOAT *magnet);
extern void optimal_mag_align(FLOAT *magnet, FLOAT angle);
extern void optimal_moving_average(FLOAT *magnet, FLOAT *orient);
extern void optimal_data_compensation(FLOAT *magnet, FLOAT *orient);
extern void mag_xyz_rotation(FLOAT *magnet, FLOAT pitch, FLOAT roll, FLOAT *rotate);

/****************************************************************
* API for vtc_accuracy
****************************************************************/
extern FLOAT m_interf_mag_threshold;
extern FLOAT m_interf_enviro_range1;
extern FLOAT m_interf_enviro_range2;
extern INT16 m_interf_enviro_threshold_1;
extern INT16 m_interf_enviro_threshold_2;

extern INT16 vtc_accuracy_level(FLOAT *mag_data, INT16 pre_acc);

/****************************************************************
* API for vtc_sw_gyro
****************************************************************/
extern FLOAT zdb_mag_ratio;
extern FLOAT zdb_acc_ratio;
extern FLOAT zdb_acc_threshold;

extern void vtc_softgyro_initialize(void);
extern BOOL set_softgyro_filter(INT16 index, FLOAT* arg);
extern BOOL softgyro_calculation(FLOAT *acc, FLOAT *mag, FLOAT gdt,
                                 FLOAT *gyr, FLOAT *vec, FLOAT *gra, FLOAT *lin);
//================================================================

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* __VTC_CUST_H__ */

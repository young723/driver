/****************************************************************
*
* @file vtc_math.h
*
* Copyright(C), 2016, Voltafield Corporation
*
****************************************************************/

#ifndef __VTC_MATH_H__
#define __VTC_MATH_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#define SUPPORT_CLIB_MATH

//================================================================
#define INVERSE_MATRIX_MAX_SIZE    9  //max size is N x N matrix

#ifndef PI
  #define PI  3.14159265359
#endif

#define radian2degree(x)  (x * 180 / PI)
#define degree2radian(x)  (x * PI / 180)

#define vtc_sign(x)       ((x < 0) ? -1 : 1)
#define vtc_fabs(x)       (vtc_sign(x) * x)

//================================================================
extern FLOAT DM[INVERSE_MATRIX_MAX_SIZE][INVERSE_MATRIX_MAX_SIZE];
extern FLOAT invDM[INVERSE_MATRIX_MAX_SIZE][INVERSE_MATRIX_MAX_SIZE];
//================================================================
extern INT16 vtc_debug_log[30];
extern INT16 vtc_cali_log[15];
extern FLOAT vtc_cae_log[10];
//================================================================
extern DOUBLE vtc_sqrt(DOUBLE x);
extern DOUBLE vtc_sin(DOUBLE x);
extern DOUBLE vtc_cos(DOUBLE x);
extern DOUBLE vtc_tan(DOUBLE x);
extern DOUBLE vtc_asin(DOUBLE x);
extern DOUBLE vtc_acos(DOUBLE x);
extern DOUBLE vtc_atan(DOUBLE x);
extern DOUBLE vtc_atan2(DOUBLE y, DOUBLE x);

extern DOUBLE vtc_cbrt(DOUBLE x);
extern BOOL sensor_data_normalize(FLOAT *output, FLOAT *input);
extern FLOAT sensor_data_stdev(FLOAT *input, INT16 num);
extern BOOL inverse_matrix(FLOAT *src, FLOAT *dis, INT8 n);
//================================================================

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* __VTC_MATH_H__ */

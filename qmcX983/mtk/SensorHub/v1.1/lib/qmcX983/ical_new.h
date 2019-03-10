/*
 * ICAL.h
 *
 *  Created on: 2016Äê6ÔÂ17ÈÕ
 *      Author: sky
 */

#ifndef __ICAL_NEW_H__
#define __ICAL_NEW_H__

#include "float.h"
#include "string.h"

// Internal Type Definitions
typedef unsigned char	UCHAR;
typedef char			INT8;
//typedef INT8			int8_t;
typedef unsigned char	UINT8;
//typedef UINT8			uint8_t;
typedef short			INT16;
//typedef INT16			int16_t;
typedef unsigned short	UINT16;
//typedef UINT16			uint16_t;
typedef long			INT32;
//typedef INT32			int32_t;
typedef unsigned long	UINT32;
//typedef UINT32			uint32_t;
typedef float			REAL;
//typedef long long		int64_t;
typedef int64_t			INT64;
//typedef unsigned long long		uint64_t;
typedef uint64_t		UINT64;

#define ICAL_ERROR		-1
#define ICAL_SUCCESS		1

#define ICAL_HDATA_SIZE 	32
#define ICAL_TRIALBUF_SIZE	20
#define ICAL_SELBUF_SIZE	4

#define ICAL_HR_TH		10//11
#define ICAL_HO_TH		0.15
#define ICAL_DIFF_RR 	5


#ifdef 	ICAL_PRECISION_DOUBLE
typedef	double			REAL;
#define ICAL_EPSILON	DBL_EPSILON
#define ICAL_FMAX		DBL_MAX
#define ICAL_FMIN		DBL_MIN

#else
typedef	float			REAL;
#define ICAL_EPSILON	FLT_EPSILON
#define ICAL_FMAX		FLT_MAX
#define ICAL_FMIN		FLT_MIN

#endif


#define SENSOR_FILTER_NUM 20

typedef struct data_xyz_t {
	REAL x;
	REAL y;
	REAL z;
}DATA;


typedef union _SENSORS_VEC_T {

	REAL v[3];
	DATA data;
	
} SENSORS_VEC_T;


typedef struct _ICAL_AOC_VAR {
	SENSORS_VEC_T		trialMag[ICAL_TRIALBUF_SIZE];
	SENSORS_VEC_T		biasbuf[ICAL_SELBUF_SIZE];
	REAL rr[ICAL_SELBUF_SIZE]; //radius
} ICAL_AOC_VAR;


int qst_ical_init(void);
int convert_magnetic(REAL *raw, REAL *result, int8_t *accuracy);
#endif /* __ICAL_NEW_H__ */

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <seos.h>
#include <util.h>
#include <sensors.h>
#include <plat/inc/rtc.h>

#include <magnetometer.h>
#include <contexthub_core.h>
#include <cust_mag.h>

#ifndef CFG_MAG_CALIBRATION_IN_AP
#include "ICAL_GDF30X.h"	//"ical_new.h"
#endif

/*-------------------------------------------------------------------*/
/* Magnetometer registers mapping */
/*-------------------------------------------------------------------*/

/* vendor chip id*/
#define QMC6308_CHIP_ID_REG		0x00

/*data output register*/
#define QMC6308_DATA_OUT_X_LSB_REG		0x01
#define QMC6308_DATA_OUT_X_MSB_REG		0x02
#define QMC6308_DATA_OUT_Y_LSB_REG		0x03
#define QMC6308_DATA_OUT_Y_MSB_REG		0x04
#define QMC6308_DATA_OUT_Z_LSB_REG		0x05
#define QMC6308_DATA_OUT_Z_MSB_REG		0x06

/*Status registers */
#define QMC6308_STATUS_REG    0x09

/* configuration registers */
#define QMC6308_CTL_REG_ONE	0x0A  /* Contrl register one */
#define QMC6308_CTL_REG_TWO	0x0B  /* Contrl register two */

 
/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/

/* Magnetic Sensor Operating Mode MODE[1:0]*/
#define QMC6308_SUSPEND_MODE	0x00
#define QMC6308_NORMAL_MODE		0x01
#define QMC6308_SINGLE_MODE		0x02
#define QMC6308_H_PFM_MODE		0x03

/*data output rate OSR2[2:0]*/
#define OUTPUT_DATA_RATE_800HZ 	0x00
#define OUTPUT_DATA_RATE_400HZ 	0x01
#define OUTPUT_DATA_RATE_200HZ 	0x02
#define OUTPUT_DATA_RATE_100HZ 	0x03

/*oversample Ratio  OSR[1]*/
#define OVERSAMPLE_RATE_256		0x01
#define OVERSAMPLE_RATE_128 	0x00

#define SET_RESET_ON 0x00
#define SET_ONLY_ON 0x01
#define SET_RESET_OFF 0x02

typedef struct sensor_mask {
	unsigned char mask0;
	unsigned char mask1;
	unsigned char mask2;
	unsigned char mask3;	
}SENSOR_MASK;

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
#include "ical_new.h"
#endif


/*qmc7983 register address*/
#define OUTPUT_X_L_REG	0x00
#define OUTPUT_X_M_REG	0x01
#define OUTPUT_Y_L_REG	0x02
#define OUTPUT_Y_M_REG	0x03
#define OUTPUT_Z_L_REG	0x04
#define OUTPUT_Z_M_REG	0x05

#define QMCX983_STATUS_REG	0x06
#define QMCX983_CTL_REG_ONE	0x09 /*bit[7:6]->osr bit[5:4]->range bit[3:2]->odr bit[1:0]->mode*/  
#define QMCX983_CTL_REG_TWO	0x0A 
#define QMCX983_SR_PERIOD_REG	0x0B

#define QMCX983_DEVICE_ID_REG	0x0D


#define QMC6983_A1_D1	0
#define QMC6983_E1	1
#define QMC6983_E1_Metal	2
#define QMC7983_Vertical	3
#define QMC7983_Slope	4


/*Defines an operation mode of the qmcX983.*/
/*
osr:00->512, 01->256, 10->128, 11->64
range:00->2G, 01->8G, 10->12G, 11->20G 
odr:00->10Hz, 01->50Hz, 10->100Hz, 11->200Hz
mode: 00->standby, 01->continuous, 10->selftest
*/
#define QMCX983_MODE_10HZ_MEASURE  0x11
#define QMCX983_MODE_50HZ_MEASURE  0x15
#define QMCX983_MODE_100HZ_MEASURE  0x19
#define QMCX983_MODE_200HZ_MEASURE 0x1D
#define QMCX983_MODE_POWERDOWN     0x1C

#define QST_2G_SENSITIVITY	10000
#define QST_8G_SENSITIVITY	2500
#define QST_12G_SENSITIVITY	1666
#define QST_16G_SENSITIVITY	1000

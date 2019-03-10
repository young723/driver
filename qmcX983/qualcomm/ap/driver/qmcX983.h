/****************************************************************************
*   This software is licensed under the terms of the GNU General Public License version 2,
*   as published by the Free Software Foundation, and may be copied, distributed, and
*   modified under those terms.
*   This program is distributed in the hope that it will be useful, but WITHOUT ANY 
*   WARRANTY; without even the implied warranty of MERCHANTABILITY or
*   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
*   for more details.
*
*   Copyright (C) 2012 by QST(Shanghai XiRui Keji) Corporation 
****************************************************************************/

#ifndef __QMCX983_H__
#define __QMCX983_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define QMCX983_IOCTL_BASE 'm'
/* The following define the IOCTL command values via the ioctl macros */
#define QMCX983_SET_MODE			  _IOW(QMCX983_IOCTL_BASE, 1, int)
#define QMCX983_SET_RANGE		      _IOW(QMCX983_IOCTL_BASE, 2, int)
#define QMCX983_READ_MAGN_XYZ	      _IOR(QMCX983_IOCTL_BASE, 3, char *)
#define QMCX983_SET_OUTPUT_DATA_RATE  _IOR(QMCX983_IOCTL_BASE, 4, char *)
#define QMCX983_SET_OVERSAMPLE_RATIO  _IOWR(QMCX983_IOCTL_BASE, 5, char *)

#define MSENSOR						   0x83
/*IOCTL for HAL*/
#define QMCX983_IOCTL_GET_DIRECTION		_IOR(MSENSOR,0x44,char)
#define QMCX983_IOCTL_GET_OTPK 			_IOR(MSENSOR,0x45,int[2])

#define QMCX983_DEV_NAME         "qmcX983"
#define QMC_SYSCLS_NAME			"compass"
#define QMC_SYSDEV_NAME			"qmcX983"
#define QMC_MISCDEV_NAME		"msensor"
/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/
#define QMCX983_SETRESET_FREQ_FAST  1
/* Magnetic Sensor Operating Mode */
#define QMCX983_STANDBY_MODE	0x00
#define QMCX983_CC_MODE			0x01
#define QMCX983_SELFTEST_MODE	0x02
#define QMCX983_RESERVE_MODE	0x03


/* Magnetometer output data rate  */
#define QMCX983_ODR_10		0x00	/* 0.75Hz output data rate */
#define QMCX983_ODR_50		0x01	/* 1.5Hz output data rate */
#define QMCX983_ODR_100		0x02	/* 3Hz output data rate */
#define QMCX983_ODR7_200	0x03	/* 7.5Hz output data rate */


/* Magnetometer full scale  */
#define QMCX983_RNG_2G		0x00
#define QMCX983_RNG_8G		0x01
#define QMCX983_RNG_12G		0x02
#define QMCX983_RNG_20G		0x03

#define RNG_2G		2
#define RNG_8G		8
#define RNG_12G		12
#define RNG_20G		20

/*data output rate HZ*/
#define DATA_OUTPUT_RATE_10HZ 	0x00
#define DATA_OUTPUT_RATE_50HZ 	0x01
#define DATA_OUTPUT_RATE_100HZ 	0x02
#define DATA_OUTPUT_RATE_200HZ 	0x03

/*oversample Ratio */
#define OVERSAMPLE_RATIO_512 	0x00
#define OVERSAMPLE_RATIO_256 	0x01
#define OVERSAMPLE_RATIO_128 	0x02
#define OVERSAMPLE_RATIO_64 	0x03

#ifdef __KERNEL__

struct QMCX983_platform_data {
	char layout;
	int gpio_DRDY;
	int gpio_RSTN;
};
#endif /* __KERNEL__ */


#endif  /* __QMCX983_H__ */

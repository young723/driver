/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef FIS210X_GYRO_H
#define FIS210X_GYRO_H

#include <linux/ioctl.h>

//#define FIS210X_GYRO_CREATE_MISC_DEV
#define FIS210X_GYRO_MTK_8_1
//#define FIS210X_GYRO_MTK_KK

#define FIS210X_GYRO_DEV_NAME				"fis210x_g"
#define FIS210X_GYRO_TAG                  	"[gyro-fis210x] "
#if 1
#define FIS210X_GYRO_FUN(f)               	printk(FIS210X_GYRO_TAG"%s\n", __FUNCTION__)
#define FIS210X_GYRO_ERR(fmt, args...)    	pr_err(FIS210X_GYRO_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define FIS210X_GYRO_LOG(fmt, args...)    	pr_err(FIS210X_GYRO_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#else
#define FIS210X_GYRO_FUN(f)               	do {} while (0)
#define FIS210X_GYRO_ERR(fmt, args...)    	do {} while (0)
#define FIS210X_GYRO_LOG(fmt, args...)    	do {} while (0)
#endif

#if defined(FIS210X_GYRO_MTK_KK)
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/batch.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <cust_eint.h>
#include <pmic_drv.h>

#elif defined(FIS210X_ACC_MTK_8_1)
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/suspend.h>

#include <sensors_io.h>
#endif

#ifndef I2C_MASK_FLAG
#define I2C_MASK_FLAG   (0x00ff)
#define I2C_DMA_FLAG    (0x2000)
#define I2C_ENEXT_FLAG  (0x0200)
#endif

#define FIS210X_I2C_SLAVE_ADDR		0x6a		//00x6a x6b

#define PI				(3.1415926)
#define DEFREE_SCALE	1024	//132		//1024
#define DEGREE_TO_RAD	58671	//7563	//58671			//(180*DEFREE_SCALE)/PI;

#define FISIMU_CTRL5_ACC_HPF_ENABLE		(0x01)
#define FISIMU_CTRL5_ACC_LPF_ENABLE		(0x02)
#define FISIMU_CTRL5_GYR_HPF_ENABLE		(0x04)
#define FISIMU_CTRL5_GYR_LPF_ENABLE		(0x08)
#define FISIMU_CTRL5_GYR_HPF01_ENABLE	(0x10)

#define FISIMU_CTRL7_DISABLE_ALL		(0x0)
#define FISIMU_CTRL7_ACC_ENABLE			(0x1)
#define FISIMU_CTRL7_GYR_ENABLE			(0x2)
#define FISIMU_CTRL7_MAG_ENABLE			(0x4)
#define FISIMU_CTRL7_AE_ENABLE			(0x8)
#define FISIMU_CTRL7_ENABLE_MASK		(0xF)

enum FisImu_mode
{
	FIS_MODE_NOMAL,
	FIS_MODE_LOW_POWER,
	FIS_MODE_POWER_DOWN
};

enum FisImu_LpfConfig
{
	Lpf_Disable, /*!< \brief Disable low pass filter. */
	Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum FisImu_HpfConfig
{
	Hpf_Disable, /*!< \brief Disable high pass filter. */
	Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum FisImu_GyrRange
{
	GyrRange_32dps = 0 << 3,   /*!< \brief +-32 degrees per second. */
	GyrRange_64dps = 1 << 3,   /*!< \brief +-64 degrees per second. */
	GyrRange_128dps = 2 << 3,  /*!< \brief +-128 degrees per second. */
	GyrRange_256dps = 3 << 3,  /*!< \brief +-256 degrees per second. */
	GyrRange_512dps = 4 << 3,  /*!< \brief +-512 degrees per second. */
	GyrRange_1024dps = 5 << 3, /*!< \brief +-1024 degrees per second. */
	GyrRange_2048dps = 6 << 3, /*!< \brief +-2048 degrees per second. */
	GyrRange_2560dps = 7 << 3  /*!< \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum FisImu_GyrOdr
{
	GyrOdr_1024Hz			= 0,	/*!< \brief High resolution 1024Hz output rate. */
	GyrOdr_256Hz			= 1,	/*!< \brief High resolution 256Hz output rate. */
	GyrOdr_128Hz			= 2,	/*!< \brief High resolution 128Hz output rate. */
	GyrOdr_32Hz				= 3,	/*!< \brief High resolution 32Hz output rate. */
	GyrOdr_OIS_8192Hz		= 6,	/*!< \brief OIS Mode 8192Hz output rate. */
	GyrOdr_OIS_LL_8192Hz	= 7		/*!< \brief OIS LL Mode 8192Hz output rate. */
};

enum FisImu_GyrUnit
{
	GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

enum FIS210xRegister
{
	/*! \brief FIS device identifier register. */
	FisRegister_WhoAmI=0, // 0
	/*! \brief FIS hardware revision register. */
	FisRegister_Revision, // 1
	/*! \brief General and power management modes. */
	FisRegister_Ctrl1, // 2
	/*! \brief Accelerometer control. */
	FisRegister_Ctrl2, // 3
	/*! \brief Gyroscope control. */
	FisRegister_Ctrl3, // 4
	/*! \brief Magnetometer control. */
	FisRegister_Ctrl4, // 5
	/*! \brief Data processing settings. */
	FisRegister_Ctrl5, // 6
	/*! \brief AttitudeEngine control. */
	FisRegister_Ctrl6, // 7
	/*! \brief Sensor enabled status. */
	FisRegister_Ctrl7, // 8
	/*! \brief Reserved - do not write. */
	FisRegister_Ctrl8, // 9
	/*! \brief Host command register. */
	FisRegister_Ctrl9,
	/*! \brief Calibration register 1 least significant byte. */
	FisRegister_Cal1_L,
	/*! \brief Calibration register 1 most significant byte. */
	FisRegister_Cal1_H,
	/*! \brief Calibration register 2 least significant byte. */
	FisRegister_Cal2_L,
	/*! \brief Calibration register 2 most significant byte. */
	FisRegister_Cal2_H,
	/*! \brief Calibration register 3 least significant byte. */
	FisRegister_Cal3_L,
	/*! \brief Calibration register 3 most significant byte. */
	FisRegister_Cal3_H,
	/*! \brief Calibration register 4 least significant byte. */
	FisRegister_Cal4_L,
	/*! \brief Calibration register 4 most significant byte. */
	FisRegister_Cal4_H,
	/*! \brief FIFO control register. */
	FisRegister_FifoCtrl,
	/*! \brief FIFO data register. */
	FisRegister_FifoData,
	/*! \brief FIFO status register. */
	FisRegister_FifoStatus,
	/*! \brief Output data overrun and availability. */
	FisRegister_Status0,
	/*! \brief Miscellaneous status register. */
	FisRegister_Status1,
	/*! \brief Sample counter. */
	FisRegister_CountOut,
	/*! \brief Accelerometer X axis least significant byte. */
	FisRegister_Ax_L,
	/*! \brief Accelerometer X axis most significant byte. */
	FisRegister_Ax_H,
	/*! \brief Accelerometer Y axis least significant byte. */
	FisRegister_Ay_L,
	/*! \brief Accelerometer Y axis most significant byte. */
	FisRegister_Ay_H,
	/*! \brief Accelerometer Z axis least significant byte. */
	FisRegister_Az_L,
	/*! \brief Accelerometer Z axis most significant byte. */
	FisRegister_Az_H,
	/*! \brief Gyroscope X axis least significant byte. */
	FisRegister_Gx_L,
	/*! \brief Gyroscope X axis most significant byte. */
	FisRegister_Gx_H,
	/*! \brief Gyroscope Y axis least significant byte. */
	FisRegister_Gy_L,
	/*! \brief Gyroscope Y axis most significant byte. */
	FisRegister_Gy_H,
	/*! \brief Gyroscope Z axis least significant byte. */
	FisRegister_Gz_L,
	/*! \brief Gyroscope Z axis most significant byte. */
	FisRegister_Gz_H,
	/*! \brief Magnetometer X axis least significant byte. */
	FisRegister_Mx_L,
	/*! \brief Magnetometer X axis most significant byte. */
	FisRegister_Mx_H,
	/*! \brief Magnetometer Y axis least significant byte. */
	FisRegister_My_L,
	/*! \brief Magnetometer Y axis most significant byte. */
	FisRegister_My_H,
	/*! \brief Magnetometer Z axis least significant byte. */
	FisRegister_Mz_L,
	/*! \brief Magnetometer Z axis most significant byte. */
	FisRegister_Mz_H,
	/*! \brief Quaternion increment W least significant byte. */
	FisRegister_Q1_L = 45,
	/*! \brief Quaternion increment W most significant byte. */
	FisRegister_Q1_H,
	/*! \brief Quaternion increment X least significant byte. */
	FisRegister_Q2_L,
	/*! \brief Quaternion increment X most significant byte. */
	FisRegister_Q2_H,
	/*! \brief Quaternion increment Y least significant byte. */
	FisRegister_Q3_L,
	/*! \brief Quaternion increment Y most significant byte. */
	FisRegister_Q3_H,
	/*! \brief Quaternion increment Z least significant byte. */
	FisRegister_Q4_L,
	/*! \brief Quaternion increment Z most significant byte. */
	FisRegister_Q4_H,
	/*! \brief Velocity increment X least significant byte. */
	FisRegister_Dvx_L,
	/*! \brief Velocity increment X most significant byte. */
	FisRegister_Dvx_H,
	/*! \brief Velocity increment Y least significant byte. */
	FisRegister_Dvy_L,
	/*! \brief Velocity increment Y most significant byte. */
	FisRegister_Dvy_H,
	/*! \brief Velocity increment Z least significant byte. */
	FisRegister_Dvz_L,
	/*! \brief Velocity increment Z most significant byte. */
	FisRegister_Dvz_H,
	/*! \brief Temperature output. */
	FisRegister_Temperature,
	/*! \brief AttitudeEngine clipping flags. */
	FisRegister_AeClipping,
	/*! \brief AttitudeEngine overflow flags. */
	FisRegister_AeOverflow,
};

enum 
{
	FIS210X_GYRO_AXIS_X		=	0,
	FIS210X_GYRO_AXIS_Y		=	1,
	FIS210X_GYRO_AXIS_Z		=	2,
	FIS210X_GYRO_AXIS_NUM
};

typedef struct
{
	struct gyro_hw 				hw;
	struct hwmsen_convert		cvt;
	u8							chip_id;
	enum FisImu_GyrRange		range;	
	enum FisImu_GyrOdr			odr;
	enum FisImu_GyrUnit			uint;
	u16							resolution;
	u16							scale;
	atomic_t					trace;	
	atomic_t					delay;	
	atomic_t					layout;	
	atomic_t					suspend;	
	s16 						cali_sw[FIS210X_GYRO_AXIS_NUM+1];
	s8                    		offset[FIS210X_GYRO_AXIS_NUM+1];
}fis210x_gyro_t;

extern struct i2c_client *fis210x_get_i2c_client(void);

#endif


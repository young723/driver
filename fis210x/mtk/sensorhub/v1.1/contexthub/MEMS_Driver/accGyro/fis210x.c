/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <algos/time_sync.h>
#include <atomic.h>
#include <cpu/inc/cpuMath.h>
#include <gpio.h>
#include <heap.h>
#include <hostIntf.h>
/* #include <isr.h> */
#include <nanohub_math.h>
#include <nanohubPacket.h>
/* #include <plat/inc/exti.h> */
/* #include <plat/inc/gpio.h> */
/* #include <plat/inc/syscfg.h> */
#include <plat/inc/rtc.h>
#include <sensors.h>
#include <seos.h>
#include <slab.h>
#include <spi.h>
#include <plat/inc/spichre.h>
#include <spichre-plat.h>
#include <timer.h>
/* #include <variant/inc/sensType.h> */
#include <variant/inc/variant.h>
#include <util.h>
#include <accGyro.h>
#include <cust_accGyro.h>
#include "hwsen.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <contexthub_core.h>
#include "eint.h"
#include <performance.h>
#include <API_sensor_calibration.h>

//#define FIS210X_DUMP_REGISTER

#define FIS210X_ACC_NAME		"fis210x_acc"
#define FIS210X_GYRO_NAME		"fis210x_gyro"

#define FISIMU_CTRL7_DISABLE_ALL 		(0x0)
#define FISIMU_CTRL7_ACC_ENABLE 		(0x1)
#define FISIMU_CTRL7_GYR_ENABLE 		(0x2)
#define FISIMU_CTRL7_MAG_ENABLE 		(0x4)
#define FISIMU_CTRL7_AE_ENABLE 			(0x8)
#define FISIMU_CTRL7_ENABLE_MASK		(0xF)

#define FISIMU_CTRL5_ACC_HPF_ENABLE	(0x01)
#define FISIMU_CTRL5_ACC_LPF_ENABLE	(0x02)
#define FISIMU_CTRL5_GYR_HPF_ENABLE	(0x04)
#define FISIMU_CTRL5_GYR_LPF_ENABLE	(0x08)
#define FISIMU_CTRL5_GYR_HPF01_ENABLE	(0x10)

enum FIS210xRegister
{
	/*! \brief FIS device identifier register. */
	FisRegister_WhoAmI, // 0
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

enum FIS210x_Ctrl9Command
{
	/*! \brief No operation. */
	Ctrl9_Nop = 0,
	/*! \brief Reset FIFO. */
	Ctrl9_ResetFifo = 0x2,
	/*! \brief Set magnetometer X calibration values. */
	Ctrl9_SetMagXCalibration = 0x6,
	/*! \brief Set magnetometer Y calibration values. */
	Ctrl9_SetMagYCalibration = 0x7,
	/*! \brief Set magnetometer Z calibration values. */
	Ctrl9_SetMagZCalibration = 0x8,
	/*! \brief Set accelerometer offset correction value. */
	Ctrl9_SetAccelOffset = 0x12,
	/*! \brief Set gyroscope offset correction value. */
	Ctrl9_SetGyroOffset = 0x13,
	/*! \brief Set accelerometer sensitivity. */
	Ctrl9_SetAccelSensitivity = 0x14,
	/*! \brief Set gyroscope sensitivity. */
	Ctrl9_SetGyroSensitivity = 0x15,
	/*! \brief Update magnemoter bias compensation. */
	Ctrl9_UpdateMagBias = 0xB,
	/*! \brief Trigger motion on demand sample. */
	Ctrl9_TriggerMotionOnDemand = 0x0c,
	/*! \brief Update gyroscope bias compensation. */
	Ctrl9_UpdateAttitudeEngineGyroBias = 0xE,
	/*! \brief Read frequency correction value. */
	Ctrl9_ReadTrimmedFrequencyValue = 0x18,
	/*! \brief Prepare for FIFO read sequence. */
	Ctrl9_ReadFifo = 0x0D,
	/*! \brief Set wake on motion parameters. */
	Ctrl9_ConfigureWakeOnMotion = 0x19,
};

enum FIS210x_AccRange
{
	AccRange_2g = 0 << 3, /*!< \brief +/- 2g range */
	AccRange_4g = 1 << 3, /*!< \brief +/- 4g range */
	AccRange_8g = 2 << 3, /*!< \brief +/- 8g range */
	AccRange_16g = 3 << 3 /*!< \brief +/- 16g range */
};
/*!
 * \brief Accelerometer output data rate.
 */
enum FIS210x_AccOdr
{
	AccOdr_1024Hz = 0,  /*!< \brief High resolution 1024Hz output rate. */
	AccOdr_256Hz = 1, /*!< \brief High resolution 256Hz output rate. */
	AccOdr_128Hz = 2, /*!< \brief High resolution 128Hz output rate. */
	AccOdr_32Hz = 3,  /*!< \brief High resolution 32Hz output rate. */
	AccOdr_LowPower_128Hz = 4, /*!< \brief Low power 128Hz output rate. */
	AccOdr_LowPower_64Hz = 5,  /*!< \brief Low power 64Hz output rate. */
	AccOdr_LowPower_25Hz = 6,  /*!< \brief Low power 25Hz output rate. */
	AccOdr_LowPower_3Hz = 7    /*!< \brief Low power 3Hz output rate. */
};

/*!
 * \brief Accelerometer output units.
 */
enum FIS210x_AccUnit
{
	AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
	AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};
/*!
 * \brief Low pass filter configuration.
 */
enum FIS210x_LpfConfig
{
	Lpf_Disable, /*!< \brief Disable low pass filter. */
	Lpf_Enable   /*!< \brief Enable low pass filter. */
};
/*!
 * \brief High pass filter configuration.
 */
enum FIS210x_HpfConfig
{
	Hpf_Disable, /*!< \brief Disable high pass filter. */
	Hpf_Enable   /*!< \brief Enable high pass filter. */
};
/*!
 * \brief Gyroscope dynamic range configuration.
 */
enum FIS210x_GyrRange
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
enum FIS210x_GyrOdr
{
	GyrOdr_1024Hz			= 0,	/*!< \brief High resolution 1024Hz output rate. */
	GyrOdr_256Hz			= 1,	/*!< \brief High resolution 256Hz output rate. */
	GyrOdr_128Hz			= 2,	/*!< \brief High resolution 128Hz output rate. */
	GyrOdr_32Hz				= 3,	/*!< \brief High resolution 32Hz output rate. */
    GyrOdr_0Hz				= 4,	/*!< \brief High resolution 32Hz output rate. */
	GyrOdr_OIS_8192Hz		= 6,	/*!< \brief OIS Mode 8192Hz output rate. */
	GyrOdr_OIS_LL_8192Hz	= 7		/*!< \brief OIS LL Mode 8192Hz output rate. */
};
	
enum FIS210x_GyrUnit
{
	GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

enum FIS210x_AeOdr
{
	AeOdr_1Hz = 0,  /*!< \brief 1Hz output rate. */
	AeOdr_2Hz = 1,  /*!< \brief 2Hz output rate. */
	AeOdr_4Hz = 2,  /*!< \brief 4Hz output rate. */
	AeOdr_8Hz = 3,  /*!< \brief 8Hz output rate. */
	AeOdr_16Hz = 4, /*!< \brief 16Hz output rate. */
	AeOdr_32Hz = 5, /*!< \brief 32Hz output rate. */
	AeOdr_64Hz = 6,  /*!< \brief 64Hz output rate. */
	AeOdr_motionOnDemand = 128
};

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif
#define ONE_G (9.807f)

#define max(x, y)							((x) > (y) ? (x) : (y))
#define SPI_PACKET_SIZE  					30
#define SPI_BUF_SIZE    					(1024 + 4)
#define SPI_WRITE_0(addr, data)				spiQueueWrite(addr, data, 2)
#define SPI_WRITE_1(addr, data, delay)		spiQueueWrite(addr, data, delay)
#define GET_SPI_WRITE_MACRO(_1, _2, _3, NAME, ...) 			NAME
#define SPI_WRITE(...)						GET_SPI_WRITE_MACRO(__VA_ARGS__, SPI_WRITE_1, SPI_WRITE_0)(__VA_ARGS__)

#define SPI_READ_0(addr, size, buf) 		spiQueueRead(addr, size, buf, 0)
#define SPI_READ_1(addr, size, buf, delay)	spiQueueRead(addr, size, buf, delay)
#define GET_SPI_READ_MACRO(_1, _2, _3, _4, NAME, ...) NAME
#define SPI_READ(...)						GET_SPI_READ_MACRO(__VA_ARGS__, SPI_READ_1, SPI_READ_0)(__VA_ARGS__)

#define EVT_SENSOR_ANY_MOTION				sensorGetMyEventType(SENS_TYPE_ANY_MOTION)

enum Fis210xState {
	STATE_SAMPLE = CHIP_SAMPLING,
	STATE_FIFO = CHIP_FIFO,
	STATE_CONVERT = CHIP_CONVERT,
	STATE_SAMPLE_DONE = CHIP_SAMPLING_DONE,
	STATE_ACC_ENABLE = CHIP_ACC_ENABLE,
	STATE_ACC_ENABLE_DONE = CHIP_ACC_ENABLE_DONE,
	STATE_ACC_DISABLE = CHIP_ACC_DISABLE,
	STATE_ACC_DISABLE_DONE = CHIP_ACC_DISABLE_DONE,
	STATE_ACC_RATECHG = CHIP_ACC_RATECHG,
	STATE_ACC_RATECHG_DONE = CHIP_ACC_RATECHG_DONE,
	
    STATE_ACC_CALI = CHIP_ACC_CALI,
    STATE_ACC_CALI_DONE = CHIP_ACC_CALI_DONE,
    STATE_ACC_CFG = CHIP_ACC_CFG,
    STATE_ACC_CFG_DONE = CHIP_ACC_CFG_DONE,

	STATE_GYRO_ENABLE = CHIP_GYRO_ENABLE,
	STATE_GYRO_ENABLE_DONE = CHIP_GYRO_ENABLE_DONE,
	STATE_GYRO_DISABLE = CHIP_GYRO_DISABLE,
	STATE_GYRO_DISABLE_DONE = CHIP_GYRO_DISABLE_DONE,
	STATE_GYRO_RATECHG = CHIP_GYRO_RATECHG,
	STATE_GYRO_RATECHG_DONE = CHIP_GYRO_RATECHG_DONE,
	
    STATE_GYRO_CALI = CHIP_GYRO_CALI,
    STATE_GYRO_CALI_DONE = CHIP_GYRO_CALI_DONE,
    STATE_GYRO_CFG = CHIP_GYRO_CFG,
    STATE_GYRO_CFG_DONE = CHIP_GYRO_CFG_DONE,

	STATE_INIT_DONE = CHIP_INIT_DONE,
	STATE_IDLE = CHIP_IDLE,
	STATE_SW_RESET = CHIP_RESET,
	STATE_INIT_REG,
	STATE_SENSOR_REGISTRATION,
	STATE_EINT_REGISTRATION,
};

enum SensorIndex {
    ACC = 0,
    GYRO,

    NUM_OF_SENSOR
};

typedef struct 
{
	uint32_t	rate;
	int			reg;
} Fis210xRateTbl;

struct Fis210xConfig {
	enum FIS210x_AccRange	accRange;
	enum FIS210x_AccOdr		accOdr;
	enum FIS210x_GyrRange	gyrRange;
	enum FIS210x_GyrOdr		gyrOdr;
	enum FIS210x_AeOdr		aeOdr;
    float 					a_sensitivity;
    float 					g_sensitivity;
};

struct Fis210xTask {
    struct Fis210xConfig config;

    SpiCbkF spiCallBack;
    struct transferDataInfo dataInfo;
    struct accGyroDataPacket accGyroPacket;
    struct accGyro_hw *hw;
    struct sensorDriverConvert cvt;
	
    uint8_t *regBuffer;
    uint8_t *accBuffer;
    uint8_t *gyroBuffer;
	uint8_t ctrl5_value;
	uint8_t ctrl7_value;
	bool acc_power;
	bool gyr_power;
	bool acc_cali;
	bool gyr_cali;	
	int a_accuracy;
	int g_accuracy;
    float aStaticCali[AXES_NUM];
    float gStaticCali[AXES_NUM];
    int32_t accSwCali[AXES_NUM];
    int32_t gyroSwCali[AXES_NUM];
    int32_t debug_trace;
	uint64_t hwSampleTime;
    uint64_t swSampleTime;

    spi_cs_t cs;
    struct SpiMode mode;
    struct SpiPacket packets[SPI_PACKET_SIZE];
    struct SpiDevice *spiDev;
    uint8_t txrxBuffer[SPI_BUF_SIZE];
    uint16_t mWbufCnt;
    uint8_t mRegCnt;
    uint8_t mRetryLeft;
    /* data for factory */
    struct TripleAxisDataPoint accFactoryData;
    struct TripleAxisDataPoint gyroFactoryData;

    int latch_time_id;
} ;

static struct Fis210xTask mFis210x;
const Fis210xRateTbl Fis210xAccRates[] = 
{
	{SENSOR_HZ(32.0f), AccOdr_32Hz},
	{SENSOR_HZ(128.0f), AccOdr_128Hz},
	{SENSOR_HZ(256.0f), AccOdr_256Hz},
	{SENSOR_HZ(1024.0f), AccOdr_1024Hz}
};

const Fis210xRateTbl Fis210xGyroRates[] = 
{
	{SENSOR_HZ(32.0f), (uint8_t)GyrOdr_32Hz},
	{SENSOR_HZ(128.0f), (uint8_t)GyrOdr_128Hz},
	{SENSOR_HZ(256.0f), (uint8_t)GyrOdr_256Hz},
	{SENSOR_HZ(1024.0f), (uint8_t)GyrOdr_1024Hz}
};


static void spiQueueWrite(uint8_t addr, uint8_t data, uint32_t delay) {
    mFis210x.packets[mFis210x.mRegCnt].size = 2;
    mFis210x.packets[mFis210x.mRegCnt].txBuf = &mFis210x.txrxBuffer[mFis210x.mWbufCnt];
    mFis210x.packets[mFis210x.mRegCnt].rxBuf = &mFis210x.txrxBuffer[mFis210x.mWbufCnt];
    mFis210x.packets[mFis210x.mRegCnt].delay = delay * 1000;
    mFis210x.txrxBuffer[mFis210x.mWbufCnt++] = addr;
    mFis210x.txrxBuffer[mFis210x.mWbufCnt++] = data;
    mFis210x.mWbufCnt = (mFis210x.mWbufCnt + 3) & 0xFFFC;
    mFis210x.mRegCnt++;
}

static void spiQueueRead(uint8_t addr, size_t size, uint8_t **buf, uint32_t delay) {
    *buf = &mFis210x.txrxBuffer[mFis210x.mWbufCnt];
    mFis210x.packets[mFis210x.mRegCnt].size = size + 1;  // first byte will not contain valid data
    mFis210x.packets[mFis210x.mRegCnt].txBuf = &mFis210x.txrxBuffer[mFis210x.mWbufCnt];
    mFis210x.packets[mFis210x.mRegCnt].rxBuf = *buf;
    mFis210x.packets[mFis210x.mRegCnt].delay = delay * 1000;
    mFis210x.txrxBuffer[mFis210x.mWbufCnt++] = 0x80 | addr;
    mFis210x.mWbufCnt = (mFis210x.mWbufCnt + size + 3) & 0xFFFC;
    mFis210x.mRegCnt++;
}

static int spiBatchTxRx(struct SpiMode *mode,
        SpiCbkF callback, void *cookie, const char *src) {
    int err = 0;
    if (mFis210x.mWbufCnt > SPI_BUF_SIZE) {
        osLog(LOG_ERROR, "NO enough SPI buffer space, dropping transaction.\n");
        return -1;
    }
    if (mFis210x.mRegCnt > SPI_PACKET_SIZE) {
        osLog(LOG_ERROR, "spiBatchTxRx too many packets!\n");
        return -1;
    }
    err = spiMasterRxTx(mFis210x.spiDev, mFis210x.cs, mFis210x.packets, mFis210x.mRegCnt, mode, callback, cookie);
    mFis210x.mRegCnt = 0;
    mFis210x.mWbufCnt = 0;
    return err;
}

static void FIS210x_enableSensors(uint8_t enableFlags)
{
	uint8_t data[2];
	if(enableFlags & FISIMU_CTRL7_AE_ENABLE)
	{
		enableFlags |= FISIMU_CTRL7_ACC_ENABLE | FISIMU_CTRL7_GYR_ENABLE;
	}

	data[0] = FisRegister_Ctrl7;
	data[1] = enableFlags & FISIMU_CTRL7_ENABLE_MASK;
	SPI_WRITE(data[0], data[1], 100);

	//vTaskDelay(100);
}

static void FIS210x_configureAccelerometer(enum FIS210x_AccRange range,enum FIS210x_AccOdr odr,enum FIS210x_AccUnit unit,
													enum FIS210x_LpfConfig lpfEnable,enum FIS210x_HpfConfig hpfEnable)
{
	// Set the CTRL2 register to configure dynamic range and ODR
	uint8_t data[2] = {
		FisRegister_Ctrl2,            // Register address
		(uint8_t)range | (uint8_t)odr // Configuration
	};
	//FisImu_writeRegisters(data, sizeof(data));
	SPI_WRITE(data[0], data[1], 100);
	// Store the scale factor for use when processing raw data
	switch(range)
	{
		case AccRange_2g:
			mFis210x.config.a_sensitivity = (1 << 14);	//1.0f / (1 << 14);
			break;
		case AccRange_4g:
			mFis210x.config.a_sensitivity = (1 << 13);	//1.0f / (1 << 13);
			break;
		case AccRange_8g:
			mFis210x.config.a_sensitivity = (1 << 12);	//1.0f / (1 << 12);
			break;
		case AccRange_16g:
			mFis210x.config.a_sensitivity = (1 << 11);	//;1.0f / (1 << 11);
			break;
		default:
			osLog(LOG_ERROR, "FIS210x_configureAccelerometer range error!\n");
			//assert(0); // Invalid dynamic range selection
			break;
	}
	// Conversion from g to m/s^2 if necessary
	//if(unit == AccUnit_ms2)
	//{
	//	mFis210x.config.a_sensitivity *= ONE_G;
	//}

	if(lpfEnable == Lpf_Enable)
	{
		mFis210x.ctrl5_value |= FISIMU_CTRL5_ACC_LPF_ENABLE;
	}
	else
	{
		mFis210x.ctrl5_value &= ~FISIMU_CTRL5_ACC_LPF_ENABLE;
	}
	
	if(hpfEnable == Hpf_Enable)
	{
		mFis210x.ctrl5_value |= FISIMU_CTRL5_ACC_HPF_ENABLE;
	}
	else
	{
		mFis210x.ctrl5_value &= ~FISIMU_CTRL5_ACC_HPF_ENABLE;
	}
	
	SPI_WRITE(FisRegister_Ctrl5, mFis210x.ctrl5_value, 100);
}
													

static void FIS210x_configureGyroscope(enum FIS210x_GyrRange range, enum FIS210x_GyrOdr odr, enum FIS210x_GyrUnit unit,
											enum FIS210x_LpfConfig lpfEnable, enum FIS210x_HpfConfig hpfEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	uint8_t data[] = {
		FisRegister_Ctrl3,            // Register address
		(uint8_t)range | (uint8_t)odr // Configuration
	};
	//FisImu_writeRegisters(data, sizeof(data));
	SPI_WRITE(data[0], data[1], 100);

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case GyrRange_32dps:
			mFis210x.config.g_sensitivity = (1 << 10);	//1.0f / (1 << 10);
			break;
		case GyrRange_64dps:
			mFis210x.config.g_sensitivity = (1 << 9);	//1.0f / (1 << 9);
			break;
		case GyrRange_128dps:
			mFis210x.config.g_sensitivity = (1 << 8);	//1.0f / (1 << 8);
			break;
		case GyrRange_256dps:
			mFis210x.config.g_sensitivity = (1 << 7);	//1.0f / (1 << 7);
			break;
		case GyrRange_512dps:
			mFis210x.config.g_sensitivity = (1 << 6);	//1.0f / (1 << 6);
			break;
		case GyrRange_1024dps:
			mFis210x.config.g_sensitivity = (1 << 5);	//1.0f / (1 << 5);
			break;
		case GyrRange_2048dps:
			mFis210x.config.g_sensitivity = (1 << 4);	//1.0f / (1 << 4);
			break;
		case GyrRange_2560dps:
			mFis210x.config.g_sensitivity = (1 << 3);	//1.0f / (1 << 3);
			break;
		default: 
			osLog(LOG_ERROR, "FIS210x_configureGyroscope range error!\n");
			//assert(0); // Invalid dynamic range selection
			break;
	}

	// Conversion from degrees/s to rad/s if necessary
	//if(unit == GyrUnit_rads)
	//{
	//	mFis210x.config.g_sensitivity *= M_PI / 180;
	//}

	if(lpfEnable == Lpf_Enable)
	{
		mFis210x.ctrl5_value |= FISIMU_CTRL5_GYR_LPF_ENABLE;
	}
	else
	{
		mFis210x.ctrl5_value &= ~FISIMU_CTRL5_GYR_LPF_ENABLE;
	}
	
	if(hpfEnable == Hpf_Enable)
	{
		mFis210x.ctrl5_value |= FISIMU_CTRL5_GYR_HPF_ENABLE;
	}
	else
	{
		mFis210x.ctrl5_value &= ~FISIMU_CTRL5_GYR_HPF_ENABLE;
	}
	
	SPI_WRITE(FisRegister_Ctrl5, mFis210x.ctrl5_value, 100);
}



static void accGetCalibration(int32_t *cali, int32_t size) {
    cali[AXIS_X] = mFis210x.accSwCali[AXIS_X];
    cali[AXIS_Y] = mFis210x.accSwCali[AXIS_Y];
    cali[AXIS_Z] = mFis210x.accSwCali[AXIS_Z];
    /* osLog(LOG_ERROR, "accGetCalibration cali x:%d, y:%d, z:%d\n", cali[AXIS_X], cali[AXIS_Y], cali[AXIS_Z]); */
}

static void accSetCalibration(int32_t *cali, int32_t size) {
    mFis210x.accSwCali[AXIS_X] = cali[AXIS_X];
    mFis210x.accSwCali[AXIS_Y] = cali[AXIS_Y];
    mFis210x.accSwCali[AXIS_Z] = cali[AXIS_Z];
    /* osLog(LOG_ERROR, "accSetCalibration cali x:%d, y:%d, z:%d\n", mFis210x.accSwCali[AXIS_X],
        mFis210x.accSwCali[AXIS_Y], mFis210x.accSwCali[AXIS_Z]); */
}

static void accGetData(void *sample) {
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mFis210x.accFactoryData.ix;
    tripleSample->iy = mFis210x.accFactoryData.iy;
    tripleSample->iz = mFis210x.accFactoryData.iz;
}

static void gyroGetCalibration(int32_t *cali, int32_t size)
{
    cali[AXIS_X] = mFis210x.gyroSwCali[AXIS_X];
    cali[AXIS_Y] = mFis210x.gyroSwCali[AXIS_Y];
    cali[AXIS_Z] = mFis210x.gyroSwCali[AXIS_Z];
    /* osLog(LOG_ERROR, "gyroGetCalibration cali x:%d, y:%d, z:%d\n", cali[AXIS_X], cali[AXIS_Y], cali[AXIS_Z]); */
}
static void gyroSetCalibration(int32_t *cali, int32_t size)
{
    mFis210x.gyroSwCali[AXIS_X] = cali[AXIS_X];
    mFis210x.gyroSwCali[AXIS_Y] = cali[AXIS_Y];
    mFis210x.gyroSwCali[AXIS_Z] = cali[AXIS_Z];
    /* osLog(LOG_ERROR, "gyroSetCalibration cali x:%d, y:%d, z:%d\n", mFis210x.gyroSwCali[AXIS_X],
        mFis210x.gyroSwCali[AXIS_Y], mFis210x.gyroSwCali[AXIS_Z]); */
}
static void gyroGetData(void *sample)
{
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mFis210x.gyroFactoryData.ix;
    tripleSample->iy = mFis210x.gyroFactoryData.iy;
    tripleSample->iz = mFis210x.gyroFactoryData.iz;
}


static void Fis210xSetDebugTrace(int32_t trace) {
    mFis210x.debug_trace = trace;
    osLog(LOG_ERROR, "%s ==> trace:%d\n", __func__, mFis210x.debug_trace);
}

static void accGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, FIS210X_ACC_NAME, sizeof(data->name));
}

static void gyroGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, FIS210X_GYRO_NAME, sizeof(data->name));
}

static int Fis210xSwReset(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {

    osLog(LOG_ERROR, "Fis210xSwReset\n");
	mFis210x.config.accRange = AccRange_8g;
	mFis210x.config.accOdr = AccOdr_128Hz;
	mFis210x.config.a_sensitivity = (1 << 12);
	mFis210x.config.gyrRange = GyrRange_1024dps;
	mFis210x.config.gyrOdr = GyrOdr_128Hz;
	mFis210x.config.g_sensitivity = (1 << 5);

	SPI_READ(FisRegister_Q1_L, 4, &mFis210x.regBuffer);
	
    return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
	//sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
	//return 0;
}

static int Fis210xInitReg(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    osLog(LOG_ERROR, "Fis210xInitReg\n");
    /* During init, reset all configurable registers to default values */

	FIS210x_configureAccelerometer(mFis210x.config.accRange,mFis210x.config.accOdr,AccUnit_ms2,Lpf_Enable,Hpf_Disable);
	FIS210x_configureGyroscope(mFis210x.config.gyrRange,mFis210x.config.gyrOdr,GyrUnit_rads,Lpf_Enable,Hpf_Disable);

    return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
}

static int Fis210xAccPowerOn(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
 void *inBuf, uint8_t inSize, uint8_t elemInSize,
 void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_ERROR, "Fis210xAccPowerOn\n");
    if(!mFis210x.acc_power) {
		mFis210x.acc_power = true;
		mFis210x.ctrl7_value |= FISIMU_CTRL7_ACC_ENABLE;
        FIS210x_enableSensors(mFis210x.ctrl7_value);
        return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
    }
	else {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
        return 0;
    }
}

static int Fis210xAccPowerOff(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_ERROR, "Fis210xAccPowerOff\n");

	if(mFis210x.acc_power) {
		mFis210x.acc_power = false;
		mFis210x.ctrl7_value &= ~FISIMU_CTRL7_ACC_ENABLE;
		if(!mFis210x.gyr_power) {
			// set imu to power down mode
		}
		
		FIS210x_enableSensors(mFis210x.ctrl7_value);
		return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
	}
	else {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
        return 0;
    }
}

static int Fis210xAccRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
						void *inBuf, uint8_t inSize, uint8_t elemInSize,
						void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    int index = 0;
	int array_len;
	uint32_t acc_rate;
	uint8_t data[2];

    struct accGyroCntlPacket cntlPacket;

	osLog(LOG_ERROR, "Fis210xAccRate\n");
    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if(ret < 0) {
        osLog(LOG_ERROR, "Fis210xAccRate, rx inSize and elemSize error\n");
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        return -1;
    }
	acc_rate = cntlPacket.rate;
	osLog(LOG_ERROR, "acc_rate = %d\n", acc_rate);
	array_len = sizeof(Fis210xAccRates)/sizeof(Fis210xAccRates[0]);
	for(index = 0; index<array_len; index++) {
		if(acc_rate <= Fis210xAccRates[index].rate)
			break;
	}
	if(index >= array_len)
	{
		osLog(LOG_ERROR, "Fis210xAccRate, rate out of range!\n");
		sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
		return -1;
	}
	else
	{
		mFis210x.config.accOdr = Fis210xAccRates[index].reg;
		data[0] = FisRegister_Ctrl2;
		data[1] = (uint8_t)mFis210x.config.accRange|(uint8_t)mFis210x.config.accOdr;
		SPI_WRITE(data[0], data[1], 100);
		
		return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
	}
}

static int Fis210xGyroPowerOn(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	 osLog(LOG_ERROR, "Fis210xGyroPowerOn\n");
	 if(!mFis210x.gyr_power) {
		 mFis210x.gyr_power = true;
		 mFis210x.ctrl7_value |= FISIMU_CTRL7_GYR_ENABLE;
		 FIS210x_enableSensors(mFis210x.ctrl7_value);
		 return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
	 }
	 else {
		 sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
		 return 0;
	 }
}

static int Fis210xGyroPowerOff(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
								void *inBuf, uint8_t inSize, uint8_t elemInSize,
								void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_ERROR, "Fis210xGyroPowerOff\n");

	if(mFis210x.gyr_power) {
		mFis210x.gyr_power = false;
		mFis210x.ctrl7_value &= ~FISIMU_CTRL7_GYR_ENABLE;
		if(!mFis210x.acc_power) {
			// set imu to power down mode
		}
		
		FIS210x_enableSensors(mFis210x.ctrl7_value);
		return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
	}
	else {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
        return 0;
    }
}

static int Fis210xGyroRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
						void *inBuf, uint8_t inSize, uint8_t elemInSize,
						void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    int index = 0;
	int array_len;
	uint32_t gyro_rate;
	uint8_t data[2];

    struct accGyroCntlPacket cntlPacket;

	osLog(LOG_ERROR, "Fis210xGyroRate\n");
    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if(ret < 0) {
        osLog(LOG_ERROR, "Fis210xGyroRate, rx inSize and elemSize error\n");
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        return -1;
    }
	gyro_rate = cntlPacket.rate;
	osLog(LOG_ERROR, "gyro_rate = %d\n", gyro_rate);
	array_len = sizeof(Fis210xGyroRates)/sizeof(Fis210xGyroRates[0]);
	for(index = 0; index<array_len; index++) {
		if(gyro_rate <= Fis210xGyroRates[index].rate)
			break;
	}
	if(index >= array_len)
	{
		osLog(LOG_ERROR, "Fis210xGyroRate, rate out of range!\n");
		sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
		return -1;
	}
	else
	{
		mFis210x.config.gyrOdr = Fis210xGyroRates[index].reg;
		data[0] = FisRegister_Ctrl3;
		data[1] = (uint8_t)mFis210x.config.gyrRange|(uint8_t)mFis210x.config.gyrOdr;
		SPI_WRITE(data[0], data[1], 100);
		
		return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
	}
}

static int Fis210xAccCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    float bias[AXES_NUM] = {0};

    mFis210x.acc_cali = true;
    osLog(LOG_ERROR, "Fis210xAccCali time:%lld\n", rtcGetTime());
    Acc_init_calibration(bias);

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}


static int Fis210xAccCfgCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
						   void *inBuf, uint8_t inSize, uint8_t elemInSize,
						   void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	int ret = 0;
	struct accGyroCaliCfgPacket caliCfgPacket;

	ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

	if (ret < 0) {
		sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
		osLog(LOG_ERROR, "Fis210xAccCfgCali, rx inSize and elemSize error\n");
		return -1;
	}
	osLog(LOG_ERROR, "Fis210xAccCfgCali: cfgData[0]:%d, cfgData[1]:%d, cfgData[2]:%d\n",
	caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1], caliCfgPacket.caliCfgData[2]);

	mFis210x.aStaticCali[0] = (float)caliCfgPacket.caliCfgData[0] / 1000;
	mFis210x.aStaticCali[1] = (float)caliCfgPacket.caliCfgData[1] / 1000;
	mFis210x.aStaticCali[2] = (float)caliCfgPacket.caliCfgData[2] / 1000;

	sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
	return 0;
}


static int Fis210xGyroCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    //float bias[AXES_NUM] = {0};
    float slope[AXES_NUM] = {0};
    float intercept[AXES_NUM] = {0};
    mFis210x.gyr_cali = true;

    osLog(LOG_ERROR, "Fis210xGyroCali\n");
    Gyro_init_calibration(slope, intercept);

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}


static int Fis210xGyroCfgCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct accGyroCaliCfgPacket caliCfgPacket;

    ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "gyroHwCaliCheck, rx inSize and elemSize error\n");
        return -1;
    }

    osLog(LOG_ERROR, "Fis210xGyroCfgCali: cfgData[0]:%d, cfgData[1]:%d, cfgData[2]:%d\n",
          caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1], caliCfgPacket.caliCfgData[2]);

    mFis210x.gStaticCali[0] = (float)caliCfgPacket.caliCfgData[0] / 1000;
    mFis210x.gStaticCali[1] = (float)caliCfgPacket.caliCfgData[1] / 1000;
    mFis210x.gStaticCali[2] = (float)caliCfgPacket.caliCfgData[2] / 1000;

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}



static int Fis210xSample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
						 void *inBuf, uint8_t inSize, uint8_t elemInSize,
						 void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	int ret = 0;

	osLog(LOG_ERROR, "Fis210xSample\n");
	ret = rxTransferDataInfo(&mFis210x.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
	if (ret < 0) {
		sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
		osLog(LOG_ERROR, "Fis210xSample, rx dataInfo error\n");
		return -1;
	}
//	if(mFis210x.acc_power && mFis210x.gyr_power)
//	{
//		SPI_READ((FisRegister_Ax_L|0xc0), 12, &mFis210x.regBuffer);
//	}

	//if(mFis210x.acc_power)
	//{
		//SPI_READ((FisRegister_Ax_L|0xc0), 6, &mFis210x.accBuffer);
		SPI_READ((FisRegister_Ax_L|0xc0), 12, &mFis210x.accBuffer);
	//}
	//if(mFis210x.gyr_power)
	//{
	//	SPI_READ((FisRegister_Gx_L|0xc0), 6, &mFis210x.gyroBuffer);
	//}

	return spiBatchTxRx(&mFis210x.mode, spiCallBack, next_state, __FUNCTION__);
}


static int Fis210xConvert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
					   void *inBuf, uint8_t inSize, uint8_t elemInSize,
					   void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	int16_t raw_data[AXES_NUM] = {0};
	int16_t remap_data[AXES_NUM] = {0};
    int32_t SwCali[AXES_NUM] = {0};
	uint8_t *reg_data = &mFis210x.accBuffer[1];
	uint8_t data_index = 0;
    uint8_t accEventSize = 0;
    uint8_t gyroEventSize = 0;
	//uint64_t SampleTime = 0;
	
    int32_t caliResult[AXES_NUM] = {0};
    float temp_data[AXES_NUM] = {0};
    float calibrated_data_output[AXES_NUM] = {0};
    int32_t delta_time = 0;
    int16_t status = 0;

	osLog(LOG_ERROR, "Fis210xConvert\n");
	struct accGyroData *data = mFis210x.accGyroPacket.outBuf;

	reg_data = &mFis210x.accBuffer[1];
	if(mFis210x.acc_power)
	{
		osLog(LOG_ERROR,
			  "ACCEL read buf [%x %x %x %x %x %x]\n",
			  reg_data[0],reg_data[1],reg_data[2],reg_data[3],reg_data[4],reg_data[5]);
		
		accGetCalibration(SwCali, 0);
		raw_data[AXIS_X] = (int16_t)(((uint16_t)reg_data[1]<<8)|((uint16_t)reg_data[0]));
		raw_data[AXIS_Y] = (int16_t)(((uint16_t)reg_data[3]<<8)|((uint16_t)reg_data[2]));
		raw_data[AXIS_Z] = (int16_t)(((uint16_t)reg_data[5]<<8)|((uint16_t)reg_data[4]));
		
		raw_data[AXIS_X] = raw_data[AXIS_X] + SwCali[AXIS_X];
		raw_data[AXIS_Y] = raw_data[AXIS_Y] + SwCali[AXIS_Y];
		raw_data[AXIS_Z] = raw_data[AXIS_Z] + SwCali[AXIS_Z];

		remap_data[mFis210x.cvt.map[AXIS_X]] = mFis210x.cvt.sign[AXIS_X] * raw_data[AXIS_X];
		remap_data[mFis210x.cvt.map[AXIS_Y]] = mFis210x.cvt.sign[AXIS_Y] * raw_data[AXIS_Y];
		remap_data[mFis210x.cvt.map[AXIS_Z]] = mFis210x.cvt.sign[AXIS_Z] * raw_data[AXIS_Z];

		temp_data[AXIS_X] = (float)remap_data[AXIS_X]*ONE_G/mFis210x.config.a_sensitivity;
		temp_data[AXIS_Y] = (float)remap_data[AXIS_Y]*ONE_G/mFis210x.config.a_sensitivity;
		temp_data[AXIS_Z] = (float)remap_data[AXIS_Z]*ONE_G/mFis210x.config.a_sensitivity;
		osLog(LOG_ERROR,"acc temp_data %f %f %f \n",(double)temp_data[AXIS_X],(double)temp_data[AXIS_Y],(double)temp_data[AXIS_Z]);

		if(1)//(mFis210x.acc_cali)
		{
            status = Acc_run_factory_calibration_timeout(delta_time,
                     temp_data, calibrated_data_output, (int *)&mFis210x.a_accuracy, rtcGetTime());
            if(status != 0) {
                mFis210x.acc_cali = false;
                if(status > 0) {
                    osLog(LOG_ERROR, "ACC cali detect shake %lld\n", rtcGetTime());
                    caliResult[AXIS_X] = (int32_t)(mFis210x.aStaticCali[AXIS_X] * 1000);
                    caliResult[AXIS_Y] = (int32_t)(mFis210x.aStaticCali[AXIS_Y] * 1000);
                    caliResult[AXIS_Z] = (int32_t)(mFis210x.aStaticCali[AXIS_Z] * 1000);
                    accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
                } else {
                    osLog(LOG_ERROR, "ACC cali time out %lld\n", rtcGetTime());
                }
            } else if(mFis210x.a_accuracy == 3) {
                mFis210x.acc_cali = false;
                mFis210x.aStaticCali[AXIS_X] = calibrated_data_output[AXIS_X] - temp_data[AXIS_X];
                mFis210x.aStaticCali[AXIS_Y] = calibrated_data_output[AXIS_Y] - temp_data[AXIS_Y];
                mFis210x.aStaticCali[AXIS_Z] = calibrated_data_output[AXIS_Z] - temp_data[AXIS_Z];
                caliResult[AXIS_X] = (int32_t)(mFis210x.aStaticCali[AXIS_X] * 1000);
                caliResult[AXIS_Y] = (int32_t)(mFis210x.aStaticCali[AXIS_Y] * 1000);
                caliResult[AXIS_Z] = (int32_t)(mFis210x.aStaticCali[AXIS_Z] * 1000);
                accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
                osLog(LOG_ERROR,
                      "ACCEL cali done %lld:caliResult[0]:%d, caliResult[1]:%d, caliResult[2]:%d, offset[0]:%f, offset[1]:%f, offset[2]:%f\n",
                      rtcGetTime(), caliResult[AXIS_X], caliResult[AXIS_Y], caliResult[AXIS_Z],
                      (double)mFis210x.aStaticCali[AXIS_X],
                      (double)mFis210x.aStaticCali[AXIS_Y],
                      (double)mFis210x.aStaticCali[AXIS_Z]);
            }
        }
		
		data[data_index].sensType = SENS_TYPE_ACCEL;
        data[data_index].x = temp_data[AXIS_X] + mFis210x.aStaticCali[AXIS_X];
        data[data_index].y = temp_data[AXIS_Y] + mFis210x.aStaticCali[AXIS_Y];
        data[data_index].z = temp_data[AXIS_Z] + mFis210x.aStaticCali[AXIS_Z];
		
		mFis210x.accFactoryData.ix = (int32_t)(data[data_index].x * ACCELEROMETER_INCREASE_NUM_AP);
        mFis210x.accFactoryData.iy = (int32_t)(data[data_index].y * ACCELEROMETER_INCREASE_NUM_AP);
        mFis210x.accFactoryData.iz = (int32_t)(data[data_index].z * ACCELEROMETER_INCREASE_NUM_AP);
		accEventSize = 1;
		data_index++;
	}
	reg_data = &mFis210x.accBuffer[7];
	if(mFis210x.gyr_power)
	{
		gyroGetCalibration(SwCali, 0);

		raw_data[AXIS_X] = (int16_t)(((uint16_t)reg_data[1]<<8)|((uint16_t)reg_data[0]));
		raw_data[AXIS_Y] = (int16_t)(((uint16_t)reg_data[3]<<8)|((uint16_t)reg_data[2]));
		raw_data[AXIS_Z] = (int16_t)(((uint16_t)reg_data[5]<<8)|((uint16_t)reg_data[4]));
		
		raw_data[AXIS_X] = raw_data[AXIS_X] + SwCali[AXIS_X];
		raw_data[AXIS_Y] = raw_data[AXIS_Y] + SwCali[AXIS_Y];
		raw_data[AXIS_Z] = raw_data[AXIS_Z] + SwCali[AXIS_Z];

		remap_data[mFis210x.cvt.map[AXIS_X]] = mFis210x.cvt.sign[AXIS_X] * raw_data[AXIS_X];
		remap_data[mFis210x.cvt.map[AXIS_Y]] = mFis210x.cvt.sign[AXIS_Y] * raw_data[AXIS_Y];
		remap_data[mFis210x.cvt.map[AXIS_Z]] = mFis210x.cvt.sign[AXIS_Z] * raw_data[AXIS_Z];

		temp_data[AXIS_X] = (float)remap_data[AXIS_X]*DEGREE_TO_RADIRAN_SCALAR/mFis210x.config.g_sensitivity;
		temp_data[AXIS_Y] = (float)remap_data[AXIS_Y]*DEGREE_TO_RADIRAN_SCALAR/mFis210x.config.g_sensitivity;
		temp_data[AXIS_Z] = (float)remap_data[AXIS_Z]*DEGREE_TO_RADIRAN_SCALAR/mFis210x.config.g_sensitivity;
		osLog(LOG_ERROR,"gyro temp_data %f %f %f \n",(double)temp_data[AXIS_X],(double)temp_data[AXIS_Y],(double)temp_data[AXIS_Z]);
		if(mFis210x.gyr_cali){
            status = Gyro_run_factory_calibration_timeout(delta_time,
                     temp_data, calibrated_data_output, (int *)&mFis210x.g_accuracy, 0, rtcGetTime());
            if(status != 0) {
                mFis210x.gyr_cali = false;
                if(status > 0) {
                    osLog(LOG_ERROR, "GYRO cali detect shake %lld\n", rtcGetTime());
                    caliResult[AXIS_X] = (int32_t)(mFis210x.gStaticCali[AXIS_X] * 1000);
                    caliResult[AXIS_Y] = (int32_t)(mFis210x.gStaticCali[AXIS_Y] * 1000);
                    caliResult[AXIS_Z] = (int32_t)(mFis210x.gStaticCali[AXIS_Z] * 1000);
                    accGyroSendCalibrationResult(SENS_TYPE_GYRO, (int32_t *)&caliResult[0], (uint8_t)status);
                } else {
                    osLog(LOG_ERROR, "GYRO cali time out %lld\n", rtcGetTime());
                }
            } else if(mFis210x.g_accuracy == 3) {
                mFis210x.gyr_cali = false;
                mFis210x.gStaticCali[AXIS_X] = calibrated_data_output[AXIS_X] - temp_data[AXIS_X];
                mFis210x.gStaticCali[AXIS_Y] = calibrated_data_output[AXIS_Y] - temp_data[AXIS_Y];
                mFis210x.gStaticCali[AXIS_Z] = calibrated_data_output[AXIS_Z] - temp_data[AXIS_Z];
                caliResult[AXIS_X] = (int32_t)(mFis210x.gStaticCali[AXIS_X] * 1000);
                caliResult[AXIS_Y] = (int32_t)(mFis210x.gStaticCali[AXIS_Y] * 1000);
                caliResult[AXIS_Z] = (int32_t)(mFis210x.gStaticCali[AXIS_Z] * 1000);
                accGyroSendCalibrationResult(SENS_TYPE_GYRO, (int32_t *)&caliResult[0], (uint8_t)status);
                osLog(LOG_ERROR,
                      "GYRO cali done %lld: caliResult[0]:%d, caliResult[1]:%d, caliResult[2]:%d, offset[0]:%f, offset[1]:%f, offset[2]:%f\n",
                      rtcGetTime(), caliResult[AXIS_X], caliResult[AXIS_Y], caliResult[AXIS_Z],
                      (double)mFis210x.gStaticCali[AXIS_X],
                      (double)mFis210x.gStaticCali[AXIS_Y],
                      (double)mFis210x.gStaticCali[AXIS_Z]);
            }
        }

		data[data_index].sensType = SENS_TYPE_GYRO;
		data[data_index].x = temp_data[AXIS_X] + mFis210x.gStaticCali[AXIS_X];
        data[data_index].y = temp_data[AXIS_Y] + mFis210x.gStaticCali[AXIS_Y];
        data[data_index].z = temp_data[AXIS_Z] + mFis210x.gStaticCali[AXIS_Z];

		mFis210x.gyroFactoryData.ix = (int32_t)(data[data_index].x * GYROSCOPE_INCREASE_NUM_AP / DEGREE_TO_RADIRAN_SCALAR);
        mFis210x.gyroFactoryData.iy = (int32_t)(data[data_index].y * GYROSCOPE_INCREASE_NUM_AP / DEGREE_TO_RADIRAN_SCALAR);
        mFis210x.gyroFactoryData.iz = (int32_t)(data[data_index].z * GYROSCOPE_INCREASE_NUM_AP / DEGREE_TO_RADIRAN_SCALAR);
		
		gyroEventSize = 1;
		data_index++;
	}

	//SampleTime = rtcGetTime();
	
    txTransferDataInfo(&mFis210x.dataInfo, accEventSize, gyroEventSize, mFis210x.hwSampleTime, data, 25.0f);
    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);

	return 0;
}


static void sensorCoreRegistration(void) {
    struct sensorCoreInfo mInfo;
    osLog(LOG_ERROR, "sensorCoreRegistration\n");

    /* Register sensor Core */
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    mInfo.sensType = SENS_TYPE_ACCEL;
    mInfo.gain = GRAVITY_EARTH_1000;
    mInfo.sensitivity = mFis210x.config.a_sensitivity;
    mInfo.cvt = mFis210x.cvt;
    mInfo.getCalibration = accGetCalibration;
    mInfo.setCalibration = accSetCalibration;
    mInfo.getData = accGetData;
    mInfo.setDebugTrace = Fis210xSetDebugTrace;
    mInfo.getSensorInfo = accGetSensorInfo;
    sensorCoreRegister(&mInfo);

    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    mInfo.sensType = SENS_TYPE_GYRO;
    mInfo.gain = GYROSCOPE_INCREASE_NUM_AP;
    mInfo.sensitivity = mFis210x.config.g_sensitivity;
    mInfo.cvt = mFis210x.cvt;
    mInfo.getCalibration = gyroGetCalibration;
    mInfo.setCalibration = gyroSetCalibration;
    mInfo.getData = gyroGetData;
    mInfo.getSensorInfo = gyroGetSensorInfo;
    sensorCoreRegister(&mInfo);
}

void Fis210xTimerCbkF(void)
{
    mFis210x.hwSampleTime = rtcGetTime();
}


static int Fis210xInitDone(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    osLog(LOG_ERROR, "Fis210xInitDone\n");
    //mt_eint_dis_hw_debounce(mFis210x.hw->eint_num);
    //mt_eint_registration(mFis210x.hw->eint_num, LEVEL_SENSITIVE, HIGH_LEVEL_TRIGGER, Fis210xIsr1, EINT_INT_UNMASK,EINT_INT_AUTO_UNMASK_OFF);
	sensorCoreRegistration();
	sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static struct sensorFsm Fis210xFsm[] = {
	sensorFsmCmd(STATE_SAMPLE, STATE_CONVERT, Fis210xSample),
	sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, Fis210xConvert),

	sensorFsmCmd(STATE_ACC_ENABLE, STATE_ACC_ENABLE_DONE, Fis210xAccPowerOn),
	sensorFsmCmd(STATE_ACC_DISABLE, STATE_ACC_DISABLE_DONE, Fis210xAccPowerOff),
	sensorFsmCmd(STATE_ACC_RATECHG, STATE_ACC_RATECHG_DONE, Fis210xAccRate),
    sensorFsmCmd(STATE_ACC_CALI, STATE_ACC_CALI_DONE, Fis210xAccCali),
    sensorFsmCmd(STATE_ACC_CFG, STATE_ACC_CFG_DONE, Fis210xAccCfgCali),

	sensorFsmCmd(STATE_GYRO_ENABLE, STATE_GYRO_ENABLE_DONE, Fis210xGyroPowerOn),
	sensorFsmCmd(STATE_GYRO_DISABLE, STATE_GYRO_DISABLE_DONE, Fis210xGyroPowerOff),
	sensorFsmCmd(STATE_GYRO_RATECHG, STATE_GYRO_RATECHG_DONE, Fis210xGyroRate),
	sensorFsmCmd(STATE_GYRO_CALI, STATE_GYRO_CALI_DONE, Fis210xGyroCali),
	sensorFsmCmd(STATE_GYRO_CFG, STATE_GYRO_CFG_DONE, Fis210xGyroCfgCali),

    sensorFsmCmd(STATE_SW_RESET, STATE_INIT_REG, Fis210xSwReset),
    sensorFsmCmd(STATE_INIT_REG, STATE_SENSOR_REGISTRATION, Fis210xInitReg),
    sensorFsmCmd(STATE_SENSOR_REGISTRATION, STATE_INIT_DONE, Fis210xInitDone),
};


int Fis210xInit(void) {
    int ret = 0;
    uint8_t txData[2] = {0}, rxData[2] = {0};

	//memset(&mFis210x, 0, sizeof(mFis210x));
	insertMagicNum(&mFis210x.accGyroPacket);
    mFis210x.hw = get_cust_accGyro("fis210x");
    if(NULL == mFis210x.hw) {
        osLog(LOG_ERROR, "get_cust_accGyro fail\n");
        ret = -1;
        goto err_out;
    }
    osLog(LOG_ERROR, "acc spi_num: %d direction:%d \n", mFis210x.hw->i2c_num, mFis210x.hw->direction);
    if(0 != (ret = sensorDriverGetConvert(mFis210x.hw->direction, &mFis210x.cvt))) {
        osLog(LOG_ERROR, "invalid direction: %d\n", mFis210x.hw->direction);
    }
    osLog(LOG_ERROR, "acc map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
        mFis210x.cvt.map[AXIS_X], mFis210x.cvt.map[AXIS_Y], mFis210x.cvt.map[AXIS_Z],
        mFis210x.cvt.sign[AXIS_X], mFis210x.cvt.sign[AXIS_Y], mFis210x.cvt.sign[AXIS_Z]);
	
    mFis210x.mode.speed = 8000000;    //8Mhz
    mFis210x.mode.bitsPerWord = 8;
    mFis210x.mode.cpol = SPI_CPOL_IDLE_HI;
    mFis210x.mode.cpha = SPI_CPHA_TRAILING_EDGE;
    mFis210x.mode.nssChange = true;
    mFis210x.mode.format = SPI_FORMAT_MSB_FIRST;
    mFis210x.mWbufCnt = 0;
    mFis210x.mRegCnt = 0;
    spiMasterRequest(mFis210x.hw->i2c_num, &mFis210x.spiDev);
    txData[0] = FisRegister_WhoAmI | 0x80;

    for (uint8_t i = 0; i < 3;) {
        ret = spiMasterRxTxSyncMode3(mFis210x.spiDev, rxData, txData, 2);
        if (ret >= 0 && (rxData[1] == 0xfc))
            break;
        ++i;
        if (i >= 3) {
            ret = -1;
            sendSensorErrToAp(ERR_SENSOR_ACC_GYR, ERR_CASE_ACC_GYR_INIT, FIS210X_GYRO_NAME);
            spiMasterRelease(mFis210x.spiDev);
            disable_latch_time(mFis210x.latch_time_id);
            free_latch_time(mFis210x.latch_time_id);
            goto err_out;
        }
    }
    osLog(LOG_ERROR, "Fis210xInit: auto detect success: %02x\n", rxData[1]);
    accSensorRegister();
    gyroSensorRegister();
    //anyMotionSensorRegister();
    registerAccGyroTimerCbk(Fis210xTimerCbkF);
    registerAccGyroInterruptMode(ACC_GYRO_FIFO_UNINTERRUPTIBLE);
    registerAccGyroDriverFsm(Fis210xFsm, ARRAY_SIZE(Fis210xFsm));
err_out:
    osLog(LOG_ERROR, "Fis210xInit: auto detect fail: %02x\n", rxData[1]);
    return ret;
}

#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(fis210x, SENS_TYPE_ACCEL, Fis210xInit);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(fis210x, OVERLAY_ID_ACCGYRO, Fis210xInit);
#endif


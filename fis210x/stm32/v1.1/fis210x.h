#ifndef FIS210X_H
#define FIS210X_H
#include "stm32f10x.h"

// for fis210x
#define AccRange_2g 	 0 << 3 /*!< \brief +/- 2g range */	
#define AccRange_4g 	 1 << 3 /*!< \brief +/- 4g range */	
#define AccRange_8g 	 2 << 3 /*!< \brief +/- 8g range */	
#define AccRange_16g 	 3 << 3 /*!< \brief +/- 16g range */

#define AccOdr_1024Hz				0  /*!< \brief High resolution 1024Hz output rate. */
#define AccOdr_256Hz				1 /*!< \brief High resolution 256Hz output rate. */
#define AccOdr_128Hz				2 /*!< \brief High resolution 128Hz output rate. */
#define AccOdr_32Hz					3  /*!< \brief High resolution 32Hz output rate. */
#define AccOdr_LowPower_128Hz		4 /*!< \brief Low power 128Hz output rate. */
#define AccOdr_LowPower_64Hz		5  /*!< \brief Low power 64Hz output rate. */
#define AccOdr_LowPower_25Hz		6  /*!< \brief Low power 25Hz output rate. */
#define AccOdr_LowPower_3Hz			7    /*!< \brief Low power 3Hz output rate. */

#define GyrRange_32dps				0 << 3   /*!< \brief +-32 degrees per second. */
#define GyrRange_64dps				1 << 3   /*!< \brief +-64 degrees per second. */
#define GyrRange_128dps				2 << 3  /*!< \brief +-128 degrees per second. */
#define GyrRange_256dps				3 << 3  /*!< \brief +-256 degrees per second. */
#define GyrRange_512dps				4 << 3  /*!< \brief +-512 degrees per second. */
#define GyrRange_1024dps			5 << 3 /*!< \brief +-1024 degrees per second. */
#define GyrRange_2048dps			6 << 3 /*!< \brief +-2048 degrees per second. */
#define GyrRange_2560dps			7 << 3  /*!< \brief +-2560 degrees per second. */

#define GyrOdr_1000Hz			 0	/*!< \brief High resolution 1024Hz output rate. */
#define GyrOdr_250Hz			 1	/*!< \brief High resolution 256Hz output rate. */
#define GyrOdr_125Hz			 2	/*!< \brief High resolution 128Hz output rate. */
#define GyrOdr_31_25Hz 			 3	/*!< \brief High resolution 32Hz output rate. */
#define GyrOdr_0Hz 			 	 4	/*!< \brief High resolution 32Hz output rate. */
#define GyrOdr_OIS_8100Hz		 6	/*!< \brief OIS Mode 8192Hz output rate. */
#define GyrOdr_OIS_LL_8100Hz	 7 	/*!< \brief OIS LL Mode 8192Hz output rate. */


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

// fis210x
#endif

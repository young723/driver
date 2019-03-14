#ifndef __QMCX983_H
#define __QMCX983_H
#include "stm32f10x.h"

//模块的A0引脚接GND，IIC的7位地址为0x2c，若接到VCC，需要改为0x2d
#define QMCX983_SLAVE_ADDRESS  (0x2c<<1)

	 /* Magnetometer registers mapping */
#define I2C_MASK_FLAG       		(0x00ff)
#define I2C_WR_FLAG         		(0x1000)
#define QMCX983_SETRESET_FREQ_FAST  1
#define RWBUF_SIZE					16
/* Magnetometer registers */
#define CTL_REG_ONE					0x09  /* Contrl register one */
#define CTL_REG_TWO					0x0a  /* Contrl register two */

/* Output register start address*/
#define OUT_X_REG		0x00

/*Status registers */
#define STA_REG_ONE		0x06
#define STA_REG_TWO		0x0c

/* Temperature registers */
#define TEMP_H_REG		0x08
#define TEMP_L_REG		0x07

/*different from qmc6983,the ratio register*/
#define RATIO_REG		0x0b
/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/
/* Magnetic Sensor Operating Mode */
#define QMCX983_STANDBY_MODE	0x00
#define QMCX983_CC_MODE			0x01
#define QMCX983_SELFTEST_MODE	0x02
#define QMCX983_RESERVE_MODE	0x03


/* Magnetometer output data rate  */
#define QMCX983_ODR_10			0x00	/* 0.75Hz output data rate */
#define QMCX983_ODR_50			0x01	/* 1.5Hz output data rate */
#define QMCX983_ODR_100			0x02	/* 3Hz output data rate */
#define QMCX983_ODR7_200		0x03	/* 7.5Hz output data rate */


/* Magnetometer full scale  */
#define QMCX983_RNG_2G			0x00
#define QMCX983_RNG_8G			0x01
#define QMCX983_RNG_12G			0x02
#define QMCX983_RNG_20G			0x03

#define RNG_2G					2
#define RNG_8G					8
#define RNG_12G					12
#define RNG_20G					20

/*data output register*/
#define OUT_X_M		0x01
#define OUT_X_L		0x00
#define OUT_Z_M		0x05
#define OUT_Z_L		0x04
#define OUT_Y_M		0x03
#define OUT_Y_L		0x02

#define SET_RATIO_REG   0x0b

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


#define SAMPLE_AVERAGE_8		(0x3 << 5)
#define OUTPUT_RATE_75			(0x6 << 2)
#define MEASURE_NORMAL			0
#define MEASURE_SELFTEST		0x1
#define GAIN_DEFAULT			(3 << 5)


// conversion of magnetic data (for bmm050) to uT units
// conversion of magnetic data to uT units
// 32768 = 1Guass = 100 uT
// 100 / 32768 = 25 / 8096
// 65536 = 360Degree
// 360 / 65536 = 45 / 8192
#define CONVERT_M			6
#define CONVERT_M_DIV		1//100			// 6/100 = CONVERT_M
#define CONVERT_O			1
#define CONVERT_O_DIV		100			// 1/64 = CONVERT_O
#define CONVERT_Q16			1
#define CONVERT_Q16_DIV		65536		// 1/64 = CONVERT_Gyro


#define MAX_FAILURE_COUNT	3
#define QMCX983_RETRY_COUNT	3
#define	QMCX983_BUFSIZE		0x20

#define QMCX983_AD0_CMP		1

#define QMCX983_AXIS_X            0
#define QMCX983_AXIS_Y            1
#define QMCX983_AXIS_Z            2
#define QMCX983_AXES_NUM          3

#define QMCX983_DEFAULT_DELAY		100


#define QMC6983_A1_D1				0
#define QMC6983_E1					1	
#define QMC7983						2
#define QMC7983_LOW_SETRESET		3
#define QMC6983_E1_Metal			4
#define QMC7983_Vertical			5
#define QMC7983_Slope				6

#define CALIBRATION_DATA_SIZE		28

#endif  /*__MPU6050*/

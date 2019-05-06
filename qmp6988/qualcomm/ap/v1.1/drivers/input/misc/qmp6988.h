/* BOSCH Pressure Sensor Driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef BOSCH_BARO_H
#define BOSCH_BARO_H

#include <linux/ioctl.h>
/*****************************************************
|  sensor  |   chip id  |     7-bit i2c address      |
-----------------------------------------------------|
|  qmp6988  |    0x5C    |0x70(SDO:Low)|0x56(SDO:High)|
*****************************************************/

#define QMP_DRIVER_VERSION "V1.0"

#define QMP_DEV_NAME        "qmp6988"

#define C_MAX_FIR_LENGTH (32)
#define MAX_SENSOR_NAME  (10)
#define QMP_DATA_NUM 1
#define QMP_PRESSURE         0
#define QMP_BUFSIZE			128

/*define bit fields*/
typedef struct bit_fields{
	s32 x:20;
	s32 y:12;
}BITFIELDS;


/* common definition */
#define QMP_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define QMP_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

	
#define SUBTRACTOR 8388608	
/* chip id */
#define QMP_CHIP_ID_REG	0xD1
#define QMP6988_CHIP_ID 0x5C

/*********************************[QMP6988]*************************************/
/* data type */
#define QMP6988_U16_t	u16
#define QMP6988_S16_t	s16
#define QMP6988_U32_t	u32
#define QMP6988_S32_t	s32
#define QMP6988_U64_t	u64
#define QMP6988_S64_t	s64

/* i2c address */
/* 7-bit addr: 0x70(SDO connected to GND); 0x56(SDO connected to VDDIO) */
#define QMP6988_I2C_ADDRESS						0x70

/* compensation calculation */
#define QMP6988_CALIBRATION_DATA_START 			0xA0 /* QMP6988 compensation coefficients */
#define QMP6988_CALIBRATION_DATA_LENGTH			25

#define SHIFT_RIGHT_4_POSITION					4
#define SHIFT_LEFT_2_POSITION					2
#define SHIFT_LEFT_4_POSITION					4
#define SHIFT_LEFT_5_POSITION					5
#define SHIFT_LEFT_8_POSITION					8
#define SHIFT_LEFT_12_POSITION					12
#define SHIFT_LEFT_16_POSITION					16


#define QMP6988_CTRLMEAS_REG					0xF4  /* Measurement Condition Control Register */
/* power mode */
#define QMP6988_SLEEP_MODE						0x00
#define QMP6988_FORCED_MODE						0x01
#define QMP6988_NORMAL_MODE						0x03

#define QMP6988_CTRLMEAS_REG_MODE__POS			0
#define QMP6988_CTRLMEAS_REG_MODE__MSK			0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN			2

/* oversampling */
#define QMP6988_OVERSAMPLING_SKIPPED			0x00
#define QMP6988_OVERSAMPLING_1X					0x01
#define QMP6988_OVERSAMPLING_2X					0x02
#define QMP6988_OVERSAMPLING_4X					0x03
#define QMP6988_OVERSAMPLING_8X					0x04
#define QMP6988_OVERSAMPLING_16X				0x05
#define QMP6988_OVERSAMPLING_32X				0x06
#define QMP6988_OVERSAMPLING_64X				0x07


#define QMP6988_CTRLMEAS_REG_OSRST__POS			5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK			0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN			3


#define QMP6988_CTRLMEAS_REG_OSRSP__POS			2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK			0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN			3


/* filter */
#define QMP6988_FILTERCOEFF_OFF					0x00
#define QMP6988_FILTERCOEFF_2					0x01
#define QMP6988_FILTERCOEFF_4					0x02
#define QMP6988_FILTERCOEFF_8					0x03
#define QMP6988_FILTERCOEFF_16					0x04
#define QMP6988_FILTERCOEFF_32					0x05

#define QMP6988_CONFIG_REG						0xF1  /*IIR filter co-efficient setting Register*/

#define QMP6988_CONFIG_REG_FILTER__POS			0
#define QMP6988_CONFIG_REG_FILTER__MSK			0x07
#define QMP6988_CONFIG_REG_FILTER__LEN			3

#define QMP6988_PRESSURE_MSB_REG              0xF7  /* Pressure MSB Register */
#define QMP6988_TEMPERATURE_MSB_REG           0xFA  /* Temperature MSB Reg */

/* sensor type */
enum SENSOR_TYPE_ENUM {
	QMP6988_TYPE = 0x0,
	INVALID_TYPE = 0xff
};

/* power mode */
enum QMP_POWERMODE_ENUM {
	QMP_SUSPEND_MODE = 0x0,
	QMP_FORCED_MODE,
	QMP_NORMAL_MODE,

	QMP_UNDEFINED_POWERMODE = 0xff
};

/* filter */
enum QMP_FILTER_ENUM {
	QMP_FILTER_OFF = 0x0,
	QMP_FILTER_2,
	QMP_FILTER_4,
	QMP_FILTER_8,
	QMP_FILTER_16,
	QMP_FILTER_32,
	QMP_UNDEFINED_FILTER = 0xff
};

/* oversampling */
enum QMP_OVERSAMPLING_ENUM {
	QMP_OVERSAMPLING_SKIPPED = 0x0,
	QMP_OVERSAMPLING_1X,
	QMP_OVERSAMPLING_2X,
	QMP_OVERSAMPLING_4X,
	QMP_OVERSAMPLING_8X,
	QMP_OVERSAMPLING_16X,
	QMP_OVERSAMPLING_32X,
	QMP_OVERSAMPLING_64X,
	QMP_UNDEFINED_OVERSAMPLING = 0xff
};

/* trace */
enum BAR_TRC {
	BAR_TRC_READ  = 0x01,
	BAR_TRC_RAWDATA = 0x02,
	BAR_TRC_IOCTL   = 0x04,
	BAR_TRC_FILTER  = 0x08,
	BAR_TRC_INFO  = 0x10,
};

/* s/w filter */
struct data_filter {
	u32 raw[C_MAX_FIR_LENGTH][QMP_DATA_NUM];
	int sum[QMP_DATA_NUM];
	int num;
	int idx;
};

/* qmp6988 calibration */
struct qmp6988_calibration_data {
	QMP6988_S32_t COE_a0;
	QMP6988_S16_t COE_a1;
	QMP6988_S16_t COE_a2;
	QMP6988_S32_t COE_b00;
	QMP6988_S16_t COE_bt1;
	QMP6988_S16_t COE_bt2;
	QMP6988_S16_t COE_bp1;
	QMP6988_S16_t COE_b11;
	QMP6988_S16_t COE_bp2;
	QMP6988_S16_t COE_b12;
	QMP6988_S16_t COE_b21;
	QMP6988_S16_t COE_bp3;
};

/*IOCTL CMD*/
#define BROMETER							0X87
#define BAROMETER_GET_RAW_DATA				_IOR(BROMETER, 0x05, int[2])
#define BAROMETER_GET_CALIBRATION_DATA		_IOR(BROMETER, 0x06, struct qmp6988_calibration_data)
#define BAROMETER_SET_CALC_DATA				_IOW(BROMETER, 0X07, int[2])
#define QMP_IOCTL_GET_STATUS				_IOR(BROMETER, 0x08, int)
#define QMP_IOCTL_GET_DELAY					_IOR(BROMETER, 0x09, int)

#endif

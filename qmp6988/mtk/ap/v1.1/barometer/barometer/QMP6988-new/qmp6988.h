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

/* apply low pass filter on output */
/*#define CONFIG_QMP_LOWPASS*/
#define CONFIG_ID_TEMPERATURE

#define QMP_DRIVER_VERSION "V1.0"

#define QMP_DEV_NAME        "qmp6988"

#define C_MAX_FIR_LENGTH (32)
#define MAX_SENSOR_NAME  (32)
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
#define QMP6988_U16_t u16
#define QMP6988_S16_t s16
#define QMP6988_U32_t u32
#define QMP6988_S32_t s32
#define QMP6988_U64_t u64
#define QMP6988_S64_t s64

/* i2c address */
/* 7-bit addr: 0x70(SDO connected to GND); 0x56(SDO connected to VDDIO) */
#define QMP6988_I2C_ADDRESS 0x70

/* compensation calculation */
#define QMP6988_CALIBRATION_DATA_START       0xA0 /* QMP6988 compensation coefficients */
#define QMP6988_CALIBRATION_DATA_LENGTH		25

#define SHIFT_RIGHT_4_POSITION				 4
#define SHIFT_LEFT_2_POSITION                2
#define SHIFT_LEFT_4_POSITION                4
#define SHIFT_LEFT_5_POSITION                5
#define SHIFT_LEFT_8_POSITION                8
#define SHIFT_LEFT_12_POSITION               12
#define SHIFT_LEFT_16_POSITION               16


#define QMP6988_CTRLMEAS_REG                  0xF4  /* Measurement Condition Control Register */
/* power mode */
#define QMP6988_SLEEP_MODE                    0x00
#define QMP6988_FORCED_MODE                   0x01
#define QMP6988_NORMAL_MODE                   0x03

#define QMP6988_CTRLMEAS_REG_MODE__POS              0
#define QMP6988_CTRLMEAS_REG_MODE__MSK              0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN              2

/* oversampling */
#define QMP6988_OVERSAMPLING_SKIPPED          0x00
#define QMP6988_OVERSAMPLING_1X               0x01
#define QMP6988_OVERSAMPLING_2X               0x02
#define QMP6988_OVERSAMPLING_4X               0x03
#define QMP6988_OVERSAMPLING_8X               0x04
#define QMP6988_OVERSAMPLING_16X              0x05
#define QMP6988_OVERSAMPLING_32X              0x06
#define QMP6988_OVERSAMPLING_64X              0x07


#define QMP6988_CTRLMEAS_REG_OSRST__POS             5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK             0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN             3


#define QMP6988_CTRLMEAS_REG_OSRSP__POS             2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK             0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN             3


/* filter */
#define QMP6988_FILTERCOEFF_OFF               0x00
#define QMP6988_FILTERCOEFF_2                 0x01
#define QMP6988_FILTERCOEFF_4                 0x02
#define QMP6988_FILTERCOEFF_8                 0x03
#define QMP6988_FILTERCOEFF_16                0x04
#define QMP6988_FILTERCOEFF_32                0x05


#define QMP6988_CONFIG_REG                    0xF1  /*IIR filter co-efficient setting Register*/

#define QMP6988_CONFIG_REG_FILTER__POS              0
#define QMP6988_CONFIG_REG_FILTER__MSK              0x07
#define QMP6988_CONFIG_REG_FILTER__LEN              3

/* data */
#define QMP6988_PRESSURE_MSB_REG              0xF7  /* Pressure MSB Register */
#define QMP6988_TEMPERATURE_MSB_REG           0xFA  /* Temperature MSB Reg */

#endif/* BOSCH_BARO_H */

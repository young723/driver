
#ifndef __QMA6981_H__
#define __QMA6981_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define QMA6981_AXIS_X          0
#define QMA6981_AXIS_Y          1
#define QMA6981_AXIS_Z          2
#define QMA6981_AXES_NUM        3

#define	QMA6981_ACC_DEV_NAME "qma6981"
#define	QMA6981_STEP_C_DEV_NAME "qma6981_step_c"


#define GRAVITY_EARTH_1000           9807	// about (9.80665f)*1000

#define QMA6981_ABS(X) ((X) < 0 ? (-1 * (X)) : (X))

/*Register Map*/
#define QMA6981_CHIP_ID		    	0x00
#define QMA6981_XOUTL				0x01	// 4-bit output value X
#define QMA6981_XOUTH				0x02	// 6-bit output value X
#define QMA6981_YOUTL				0x03	
#define QMA6981_YOUTH				0x04	
#define QMA6981_ZOUTL				0x05	
#define QMA6981_ZOUTH				0x06	
#define QMA6981_STEP_CNT_L			0x07
#define QMA6981_REG_RANGE			0x0f
#define QMA6981_REG_BW_ODR			0x10
#define QMA6981_REG_POWER_CTL		0x11
#define QMA6981_STEP_SAMPLE_CNT		0x12
#define QMA6981_STEP_PRECISION		0x13
#define QMA6981_STEP_TIME_LOW		0x14
#define QMA6981_STEP_TIME_UP		0x15
#define QMA6981_OS_CUST_X		    0x27
#define QMA6981_OS_CUST_Y			0x28
#define QMA6981_OS_CUST_Z			0x29
#define QMA6981_STEP_TIME_UP		0x15

/*I2C ADDRESS*/
#define QMA6981_I2C_SLAVE_ADDR		0x12

#define QMA6981_OFS 96

#define QMA6981_ERR_I2C                     -1
#define QMA6981_SUCCESS			     0

/*ODR SET @lower ODR*/
#define QMA6981_ODR_250HZ				0x0d
#define QMA6981_ODR_125HZ             	0x0c  
#define QMA6981_ODR_62HZ             	0x0b   
#define QMA6981_ODR_31HZ            	0x0a   
#define QMA6981_ODR_16HZ				0x09


/* Accelerometer Sensor Full Scale */
#define QMA6981_RANGE_2G   (1<<0)
#define QMA6981_RANGE_4G   (1<<1)
#define QMA6981_RANGE_8G   (1<<2)

#endif  /* __QMA6981_H__ */

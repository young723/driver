
#ifndef __QMAX981_H__
#define __QMAX981_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

//#define ANDROID80_ABOVE
#define QMAX981_CREATE_MISC_DEV
#define QMAX981_STEP_COUNTER
//#define QMAX981_FIFO_FUNC
//#define QMAX981_TAP_FUNC
//#define QMAX981_INT1_FUNC
//#define QMAX981_INT2_FUNC


#if defined(QMAX981_STEP_COUNTER)
#include "step_counter.h"
#define QMAX981_CHECK_ABNORMAL_DATA
//#define QMAX981_STEP_DEBOUNCE_IN_INT
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
#define QMAX981_INT1_FUNC
#endif
#endif

//#if defined(QMAX981_FIFO_FUNC)
//#define QMAX981_FIFO_USE_INT
//#define QMAX981_INT1_FUNC
//#endif

//#if defined(QMAX981_TAP_FUNC)
//#define QMAX981_INT1_FUNC
//#endif

#if defined(QMAX981_STEP_COUNTER)||defined(QMAX981_INT1_FUNC)||defined(QMAX981_INT2_FUNC)
#define QMAX981_FIX_REG
#endif

#define	QMAX981_ACC_DEV_NAME		"qmaX981"
#define	QMAX981_STEP_C_DEV_NAME		"qmaX981_step_c"

#define QMAX981_AXIS_X          0
#define QMAX981_AXIS_Y          1
#define QMAX981_AXIS_Z          2
#define QMAX981_AXES_NUM        3

/*I2C ADDRESS*/
#define QMAX981_I2C_SLAVE_ADDR		0x12	// AD0 GND 0x12, AD0 VDD 0x13
#define QMAX981_I2C_SLAVE_ADDR2		0x13	// AD0 GND 0x12, AD0 VDD 0x13
#define QMAX981_ERR_I2C				-1
#define QMAX981_SUCCESS				0

#define GRAVITY_EARTH_1000          9807	// about (9.80665f)*1000   mm/s2
#define QMAX981_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))
#if defined(QMAX981_STEP_COUNTER)
#define QMA6981_OFFSET 				0x60
#else
#define QMA6981_OFFSET 				0x00
#endif

#define QMAX981_DELAY				0xff
/*Register Map*/
#define QMAX981_CHIP_ID		    	0x00
#define QMAX981_XOUTL				0x01
#define QMAX981_XOUTH				0x02
#define QMAX981_YOUTL				0x03
#define QMAX981_YOUTH				0x04
#define QMAX981_ZOUTL				0x05
#define QMAX981_ZOUTH				0x06
#define QMAX981_STEP_CNT_L			0x07
#define QMAX981_INT_STAT0			0x0a
#define QMAX981_INT_STAT1			0x0b
#define QMAX981_INT_STAT2			0x0c
#define QMAX981_INT_STAT3			0x0d
#define QMAX981_FIFO_STATE			0x0e
#define QMAX981_REG_RANGE			0x0f
#define QMAX981_REG_BW_ODR			0x10
#define QMAX981_REG_POWER_CTL		0x11
#define QMAX981_STEP_SAMPLE_CNT		0x12
#define QMAX981_STEP_PRECISION		0x13
#define QMAX981_STEP_TIME_LOW		0x14
#define QMAX981_STEP_TIME_UP		0x15
#define QMAX981_INTPIN_CFG			0x20
#define QMAX981_INT_CFG				0x21
#define QMAX981_OS_CUST_X		    0x27
#define QMAX981_OS_CUST_Y			0x28
#define QMAX981_OS_CUST_Z			0x29
#define QMAX981_STEP_TIME_UP		0x15
/*ODR SET @lower ODR*/
#define QMA6981_ODR_1000HZ			0x07
#define QMA6981_ODR_500HZ			0x06
#define QMA6981_ODR_250HZ			0x05
#define QMA6981_ODR_125HZ			0x04  
#define QMA6981_ODR_62HZ			0x03   
#define QMA6981_ODR_31HZ			0x02   
#define QMA6981_ODR_16HZ			0x01
#define QMA6981_ODR_HIGH			0x20

/* Accelerometer Sensor Full Scale */
#define QMAX981_RANGE_2G			0x01
#define QMAX981_RANGE_4G			0x02
#define QMAX981_RANGE_8G			0x04
#define QMAX981_RANGE_16G			0x08
#define QMAX981_RANGE_32G			0x0f

/* 0x11 Set the sleep time, when device is in power cycling power saving.*/
#define QMA6981_SLEEP_DUR0			0x00
#define QMA6981_SLEEP_DUR1			0x06
#define QMA6981_SLEEP_DUR2			0x07
#define QMA6981_SLEEP_DUR4			0x08
#define QMA6981_SLEEP_DUR6			0x09
#define QMA6981_SLEEP_DUR10			0x0a
#define QMA6981_SLEEP_DUR25			0x0b
#define QMA6981_SLEEP_DUR50			0x0c
#define QMA6981_SLEEP_DUR100		0x0d
#define QMA6981_SLEEP_DUR500		0x0e
#define QMA6981_SLEEP_DUR1000		0x0f


// for ioctl
#define GSENSOR_IOCTL_RESET_SC         				_IO(GSENSOR, 0x10)
#define GSENSOR_IOCTL_SET_STICK_CHECK_REG         	_IOW(GSENSOR, 0x11, int)
#define GSENSOR_IOCTL_GET_STICK_STATUS         		_IOR(GSENSOR, 0x12, int)
#define GSENSOR_IOCTL_GET_FIFO_DATA         		_IOR(GSENSOR, 0x13, int*)
#define GSENSOR_IOCTL_GET_FIFO_RAW         			_IOR(GSENSOR, 0x14, int*)
#define GSENSOR_IOCTL_WRITE_REG         			_IOW(GSENSOR, 0x15, unsigned char*)
#define GSENSOR_IOCTL_READ_REG         				_IOR(GSENSOR, 0x16, unsigned char*)
#define GSENSOR_IOCTL_SET_WAKELOCK         			_IO(GSENSOR, 0x17)
//#define GSENSOR_IOCTL_SET_STEPCOUNTER				_IOW(GSENSOR, 0x18, long)

// for ioctl

#endif  /* __QMAX981_H__ */

#ifndef __QMA6981_H
#define __QMA6981_H

#include "qst_common.h"
#include "qst_sw_i2c.h"

#define QMAX981_ACC_I2C_ADDRESS		(0x12<<1)
#define QMAX981_ACC_I2C_ADDRESS2	(0x13<<1)

//#define QMAX981_USE_SW_I2C

#define GRAVITY_EARTH_1000          9807	// about (9.80665f)*1000   mm/s2

//#define QMAX981_FIFO_FUNC
//#define QMAX981_FIFO_USE_INT

//#define QMA7981_IRQ_TEST

#define QMAX981_TAP_FUNC
//#define QMAX981_INT_LATCH_MODE
//#define QMAX981_STEP_COUNTER
#if defined(QMAX981_STEP_COUNTER)
#define QMAX981_STEP_DEBOUNCE_IN_INT
#define QMAX981_CHECK_ABNORMAL_DATA
#endif

#if defined(QMAX981_FIFO_USE_INT)||defined(QMAX981_TAP_FUNC)||defined(QMAX981_STEP_DEBOUNCE_IN_INT)||defined(QMA7981_IRQ_TEST)
#define QMAX981_USE_IRQ1
#endif

#define QMAX981_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))
#if defined(QMAX981_STEP_COUNTER)
#define QMA6981_OFFSET 				0x60
#else
#define QMA6981_OFFSET 				0x00
#endif

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
#define QMA6981_ODR_250HZ			0x0d
#define QMA6981_ODR_125HZ			0x0c  
#define QMA6981_ODR_62HZ			0x0b   
#define QMA6981_ODR_31HZ			0x0a   
#define QMA6981_ODR_16HZ			0x09


/* Accelerometer Sensor Full Scale */
#define QMAX981_RANGE_2G			0x01
#define QMAX981_RANGE_4G			0x02
#define QMAX981_RANGE_8G			0x04
#define QMAX981_RANGE_16G			0x08
#define QMAX981_RANGE_32G			0x0f


extern s32 qmaX981_read_acc(s32 *accData);
extern s32 qmaX981_read_raw(s32 *rawData);
extern s32 qmaX981_init(void);
extern u8 qmaX981_chip_id(void);
extern u8 qmaX981_readreg(u8 reg_add,u8 *buf,u8 num);
extern u8 qmaX981_writereg(u8 reg_add,u8 reg_dat);
#if defined(QMAX981_STEP_COUNTER)
extern int qmaX981_read_stepcounter(void);
#endif

#endif  /*QMA6981*/

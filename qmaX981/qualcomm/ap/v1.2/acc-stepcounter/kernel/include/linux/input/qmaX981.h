#ifndef QMA6981_H
#define QMA6981_H

#include <linux/ioctl.h>

/* Device specific constant values */

#define QMA6981_CHIP_ID		    0x00
#define QMA6981_XOUTL			0x01	// 4-bit output value X
#define QMA6981_XOUTH			0x02	// 6-bit output value X
#define QMA6981_YOUTL			0x03
#define QMA6981_YOUTH			0x04
#define QMA6981_ZOUTL			0x05
#define QMA6981_ZOUTH			0x06
#define QMA6981_STEP_CNT 		0x07 //stepcounter
#define QMA6981_ODR				0x10
#define QMA6981_POWER			0x11

#define QMA6981_AXIS_X          0
#define QMA6981_AXIS_Y          1
#define QMA6981_AXIS_Z          2
#define QMA6981_AXES_NUM        3

#define QMA6981_ODR_250HZ		0x0d
#define QMA6981_ODR_125HZ		0x0c  
#define QMA6981_ODR_62HZ 		0x0b   
#define QMA6981_ODR_31HZ		0x0a   
#define QMA6981_ODR_16HZ		0x09

#define QMAX981_RANGE_2G			0x01
#define QMAX981_RANGE_4G			0x02
#define QMAX981_RANGE_8G			0x04
#define QMAX981_RANGE_16G			0x08
#define QMAX981_RANGE_32G			0x0f

#define QMA6981_ABS(X) 			((X) < 0 ? (-1 * (X)) : (X))
#define QMA6981_DIFF(x, y)		((x)>=(y)?((x)-(y)):((x)+65535-(y)))

#define QMA6981_CAL_SKIP_COUNT 		5
#define QMA6981_CAL_MAX				(10 + QMA6981_CAL_SKIP_COUNT)
#define QMA6981_CAL_NUM				99


/* IOCTLs for Acc Step Counter Control */
#define GSENSOR                         0x85
#define QMA_ACC_ENABLE					_IOW(GSENSOR, 0x01, char)
#define QMA_STP_ENABLE					_IOW(GSENSOR, 0x02, char)
#define QMA_STP_CLEAR					_IOR(GSENSOR, 0x03, char)

struct qmaX981_platform_data {
	char layout;
	int gpio_DRDY;
	int gpio_RSTN;
};

#endif

#ifndef __QMAX981_H__
#define __QMAX981_H__
		  
#include <linux/ioctl.h>	  

#define QMAX981_ACC_I2C_ADDR     0x12


typedef struct {
	unsigned short	x;		/**< X axis */
	unsigned short	y;		/**< Y axis */
	unsigned short	z;		/**< Z axis */
} GSENSOR_VECTOR3D;

typedef struct{
	int x;
	int y;
	int z;
} SENSOR_DATA;

struct QMAX981_acc_platform_data {
	int layout;
	int gpio_int1;
	};
	/* __KERNEL__ */

/*range*/
#define QMA6981_RANGE_2G   0x01		//+-2g	3.9mg/LSB
#define QMA6981_RANGE_4G   0x02		//+-4g	7.8mg/LSB
#define QMA6981_RANGE_8G  0x04		//+-8g	15.6ms/LSB

#define QMA6981_XOUTL			0x01	// 4-bit output value X
#define QMA6981_XOUTH			0x02	// 6-bit output value X
#define QMA6981_YOUTL			0x03	
#define QMA6981_YOUTH			0x04	
#define QMA6981_ZOUTL			0x05	
#define QMA6981_ZOUTH			0x06	
#define QMA6981_STEPCOUNT		0x07	// stepcount
#define QMA6981_SRST			0x04	// Sampling Rate Status
#define QMA6981_SPCNT			0x05	// Sleep Count
#define QMA6981_INTSU			0x17	// Interrupt Setup
#define QMA6981_MODE			0x11	// Mode
#define QMA6981_SR				0x10	// Auto-Wake/Sleep and Debounce Filter
#define QMA6981_PDET			0x09	// Tap Detection
#define QMA6981_PD				0x0a	// Tap Debounce Count
#define QMA6981_INT_MAP0		0x19	// INT MAP
#define QMA6981_RANGE           0x0f    //range set register
#define QMA6981_INT_STAT		0x09    //interrupt statues

#define QMA6981_FIFO_WTMK		0x31	// FIFO water mark level
#define QMA6981_FIFO_CONFIG		0x3e	// fifo configure
#define QMA6981_FIFO_DATA		0x3f	//fifo data out 

#define QMA6981_CHIP_ID			0x00
#define QMA6981_DIE_ID			0x07
#define QMA6981_DIE_ID_V2		0x47


/*Power Mode*/
#define QMAX981_MODE_STANDBY		0
#define QMAX981_MODE_ACTIVE			1


#define QMA6981_ODR_1000HZ				0x0f
#define QMA6981_ODR_500HZ				0x0e
#define QMA6981_ODR_250HZ				0x0d
#define QMA6981_ODR_125HZ             	0x0c  
#define QMA6981_ODR_62HZ             	0x0b   
#define QMA6981_ODR_31HZ            	0x0a   
#define QMA6981_ODR_16HZ				0x09

// initial value	
#define QMAX981_OFFSET			 		0x60
#define QMAX981_MAX_DELAY			200
#define QMAX981_DEFAULT_DELAY	20
#define ABSMIN_8G					(-8 * 1024)
#define ABSMAX_8G					(8 * 1024)
#define ABSMIN_1_5G					(-128)
#define ABSMAX_1_5G				    (128)
#define QMAX981_IRQ_NUMBER  139

#define QMAX981_STEP_COUNTER
#define QMA6981_STEP_COUNTER_USE_INT
   
#define QMAX981_ACC_IOCTL_BASE 				77 //0x1D
#define QMAX981_ACC_IOCTL_SET_DELAY		_IOW(QMAX981_ACC_IOCTL_BASE,0,int)
#define QMAX981_ACC_IOCTL_GET_DELAY		_IOR(QMAX981_ACC_IOCTL_BASE,1,int)
#define QMAX981_ACC_IOCTL_SET_ENABLE		_IOW(QMAX981_ACC_IOCTL_BASE,2,int)
#define QMAX981_ACC_IOCTL_GET_ENABLE		_IOR(QMAX981_ACC_IOCTL_BASE,3,int)
#define QMAX981_ACC_IOCTL_CALIBRATION	_IOW(QMAX981_ACC_IOCTL_BASE,4,int)

#ifdef QMAX981_STEP_COUNTER
#define QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE		_IOW(QMAX981_ACC_IOCTL_BASE,5,int)
#define QMAX981_ACC_IOCTL_GET_STEPCOUNT_ENABLE		_IOR(QMAX981_ACC_IOCTL_BASE,6,int)
#define QMAX981_ACC_IOCTL_CLEAR_STEPCOUNT		_IOR(QMAX981_ACC_IOCTL_BASE,7,int)
#endif

#endif


#ifndef QST_QMP6988_H
#define QST_QMP6988_H

#include <linux/ioctl.h>

#define QMP6988_TAG                  	"[qmp6988] "
#define QMP6988_FUN(f)               	pr_err(QMP6988_TAG"%s\n", __func__)
#define QMP6988_ERR(fmt, args...) 		pr_err(QMP6988_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define QMP6988_LOG(fmt, args...)    	pr_err(QMP6988_TAG fmt, ##args)

#define QMP6988_CREATE_MISC_DEVICE
#define QMP6988_DEV_NAME        "qmp6988"
#define QMP6988_BUFSIZE			96

#define QMP6988_CHIP_ID_REG						0xD1
#define QMP6988_RESET_REG             			0xE0  /* Device reset register */
#define QMP6988_CONFIG_REG						0xF1
#define QMP6988_DEVICE_STAT_REG             	0xF3  /* Device state register */
#define QMP6988_CTRLMEAS_REG					0xF4  /* Measurement Condition Control Register */
#define QMP6988_IO_SETUP_REG					0xF5  /* IO SETUP Register */

#define QMP6988_SLEEP_MODE                    	0x00
#define QMP6988_FORCED_MODE                   	0x01
#define QMP6988_NORMAL_MODE                   	0x03
#define QMP6988_OVERSAMPLING_SKIPPED          	0x00
#define QMP6988_OVERSAMPLING_1X               	0x01
#define QMP6988_OVERSAMPLING_2X               	0x02
#define QMP6988_OVERSAMPLING_4X               	0x03
#define QMP6988_OVERSAMPLING_8X               	0x04
#define QMP6988_OVERSAMPLING_16X              	0x05
#define QMP6988_OVERSAMPLING_32X              	0x06
#define QMP6988_OVERSAMPLING_64X              	0x07
#define QMP6988_FILTERCOEFF_OFF               	0x00
#define QMP6988_FILTERCOEFF_2                 	0x01
#define QMP6988_FILTERCOEFF_4                 	0x02
#define QMP6988_FILTERCOEFF_8                 	0x03
#define QMP6988_FILTERCOEFF_16                	0x04
#define QMP6988_FILTERCOEFF_32                	0x05
#define QMP6988_T_STANDBY_1MS					0x00
#define QMP6988_T_STANDBY_5MS					0x01
#define QMP6988_T_STANDBY_50MS					0x02
#define QMP6988_T_STANDBY_250MS					0x03
#define QMP6988_T_STANDBY_500MS					0x04
#define QMP6988_T_STANDBY_1S					0x05
#define QMP6988_T_STANDBY_2S					0x06
#define QMP6988_T_STANDBY_4S					0x07

#define SHIFT_RIGHT_4_POSITION				 	4
#define SHIFT_LEFT_2_POSITION                	2
#define SHIFT_LEFT_4_POSITION                	4
#define SHIFT_LEFT_5_POSITION                	5
#define SHIFT_LEFT_8_POSITION                	8
#define SHIFT_LEFT_12_POSITION               	12
#define SHIFT_LEFT_16_POSITION               	16

#define QMP6988_U16_t unsigned short
#define QMP6988_S16_t short
#define QMP6988_U32_t unsigned int
#define QMP6988_S32_t int
#define QMP6988_S64_t long

struct qmp6988_calibration_data 
{
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


typedef enum
{
	QMP6988_APP_WEATHER_REPORT,
	QMP6988_APP_DROP_DETECTION,
	QMP6988_APP_ElEVATOR_DETECTION,
	QMP6988_APP_STAIR_DETECTION,
	QMP6988_APP_INDOOR_NAVIGATION,

	QMP6988_APP_TOTAL
} qmp6988_app_e;

#ifndef BROMETER
#define BROMETER							0X87
#endif

#define BAROMETER_GET_CALI					_IOR(BROMETER, 0x06, struct qmp6988_calibration_data)
#ifndef BAROMETER_IOCTL_READ_CHIPINFO
#define BAROMETER_IOCTL_READ_CHIPINFO				_IOR(BROMETER, 0x04, int)
#endif

#ifdef CONFIG_COMPAT

#define COMPAT_BAROMETER_GET_CALI			_IOR(BROMETER, 0x06, struct qmp6988_calibration_data)
#ifndef COMPAT_BAROMETER_IOCTL_READ_CHIPINFO
#define COMPAT_BAROMETER_IOCTL_READ_CHIPINFO		_IOR(BROMETER, 0x04, compat_int_t)
#endif

#endif

#endif/* QST_QMP6988_H */

/* drivers/misc/qmaX981.c - QMA6981 Acceleration Sensor Driver
*
* Copyright (C) 2007-2014 HTC Corporation.
* Author: Hou-Kun Chen <houkun.chen@gmail.com>
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

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/input/qmaX981.h>

//#define QST_IRQ 
#define QMAX981_STEPCOUNTER
#if defined(QMAX981_STEPCOUNTER)
#define QMAX981_STEPCOUNTER_USE_INT
#define QMA6981_CHECK_ABNORMAL_DATA
#endif

#define QMA6981_DEBUG	1

#if QMA6981_DEBUG
#define MSE_TAG                  "[QMA6981] "
#define MSE_FUN(f)               printk(MSE_TAG"%s\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)    printk(KERN_ERR MSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)    printk(MSE_TAG fmt"\n", ##args)
#else
#define MSE_FUN(f)			do{}while(0)
#define MSE_ERR(fmt, args...)	do{}while(0)
#define MSE_LOG(fmt, args...)	do{}while(0)
#endif

#define QMA6981_VDD_MIN_UV       2000000
#define QMA6981_VDD_MAX_UV       3400000
#define QMA6981_VIO_MIN_UV       1500000
#define QMA6981_VIO_MAX_UV       3400000

#define QMA_MISCDEV_NAME 	"QST-qmaX981"
#define QMA_I2C_NAME 		"qst,qmaX981"
#define QMA_INPUT_DEVICE_NAME	"accelerometer"
#define QMA_DEV_NAME_ACCEL	"accelerometer"
#define QMA_DEV_NAME_STEPCOUNTER	"step_counter"

#define QMA6981_ACCEL_MIN_POLL_INTERVAL_MS	10
#define QMA6981_ACCEL_MAX_POLL_INTERVAL_MS	5000
#define QMA6981_ACCEL_DEFAULT_POLL_INTERVAL_MS	20

#define QMAX981_STEPCOUNTER_MIN_POLL_INTERVAL_MS	10
#define QMAX981_STEPCOUNTER_MAX_POLL_INTERVAL_MS	5000
#define QMAX981_STEPCOUNTER_DEFAULT_POLL_INTERVAL_MS	20

#define QMA6981_ACCEL_MIN_VALUE	-32768
#define QMA6981_ACCEL_MAX_VALUE	32767
#define QMAX981_STEPCOUNTER_MIN_VALUE	0
#define QMAX981_STEPCOUNTER_MAX_VALUE	65535

//#define QMA6981_USE_CALI
#if defined(QMA6981_USE_CALI)
#define QMA6981_CALI_FILE		"/persist/sensors/qmaX981cali.conf"
#if defined(QMAX981_STEPCOUNTER)
#define QMA6981_LSB_1G			64//range scale =8G
#else
//#define QMA6981_LSB_1G		256    // range scale =2G
#define QMA6981_LSB_1G			128    // range scale =4G
#endif
#define QMA6981_CALI_NUM		20

static int qmaX981_cali[3]={0, 0, 0};
static char qmaX981_cali_flag = 0;
static void qmaX981_read_file(char * filename, char *data, int len);
static void qmaX981_write_file(char * filename, char *data, int len);
#endif

typedef struct {
 int i:10;
 int rsv:22;
}data_convert_s;

struct hwmsen_convert {
	s8 sign[4];
	u8 map[4];
};

static struct hwmsen_convert map[] = {
 { { 1, 1, 1}, {0, 1, 2} },
 { {-1, 1, 1}, {1, 0, 2} },
 { {-1,-1, 1}, {0, 1, 2} },
 { { 1,-1, 1}, {1, 0, 2} },

 { {-1, 1,-1}, {0, 1, 2} },
 { { 1, 1,-1}, {1, 0, 2} },
 { { 1,-1,-1}, {0, 1, 2} },
 { {-1,-1,-1}, {1, 0, 2} },      

};

typedef enum
{
	CHIP_TYPE_QMA6981 = 0,
	CHIP_TYPE_QMA7981,
	CHIP_TYPE_QMA6100,
	CHIP_TYPE_UNDEFINE,

	CHIP_TYPE_MAX
}qmaX981_type;

struct qma_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct qmaX981_data
{
	struct i2c_client	*i2c;
	struct delayed_work	dwork;
	struct workqueue_struct	*work_queue;

	qmaX981_type chip_type;		
	int resolution;
	uint8_t				sense_data[6];

	struct mutex		op_mutex;
	struct mutex		sensor_mutex;
	struct mutex		val_mutex;

	int8_t			enable_flag;
	int64_t			delay;
	int 			range;

	char calibrate_buf[QMA6981_CAL_NUM];
	int cal_params[3];
	bool use_cal;
	atomic_t cal_status;

	struct input_dev *accel_dev;
	struct sensors_classdev accel_cdev;

	atomic_t	active;
	wait_queue_head_t	open_wq;
	struct qma_sensor_state state;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;
	bool acc_use_cal;
#if defined(QMAX981_STEPCOUNTER)
	struct delayed_work sc_dwork;
	struct mutex		op_sc_mutex;
	struct mutex		sensor_sc_mutex;
	struct mutex	val_sc_mutex;

	int8_t			sc_enable_flag;
	int64_t			sc_delay;
	#if defined(QMAX981_STEPCOUNTER_USE_INT)
	int		int_pin;
	uint32_t int_flags;
	#endif
	struct input_dev *stepcount_dev;
	struct sensors_classdev stepcount_cdev;
#endif
	int			layout;
	int				irq;
	int				gpio_rstn;
	int				power_enabled;
	struct hwmsen_convert 	cvt;
	struct regulator *vdd;
	struct regulator *vio;
};

#ifdef QMAX981_STEPCOUNTER_USE_INT
#if defined(QMA6981_INT_ALGO_2)

#define QMAX981_STEPCOUNTER_INT_NUM		(4)
#define QMAX981_STEPCOUNTER_DEBOUNCE	(8)

typedef struct
{
	unsigned int	report;
	unsigned int	start;
	unsigned int	last;
	unsigned int	curr;
	unsigned int	debounce;
	bool			step_valid;
}qmaX981_sc_algo;

static qmaX981_sc_algo	g_sc_algo;

#else

#define STEP_INT_START_VLUE	4
static int STEP_DUMMY_VLUE = 8;
#define STEP_END	false
#define STEP_START	true

struct qmaX981_stepcount{
	int stepcounter_pre_end;  
	int stepcounter_next_start;
	int stepcounter_next_end;
	int stepcounter_pre;
	int stepcounter_pre_fix;
	bool stepcounter_statu;
	bool stepcounter_start_int_update_diff;
	int back;
	int step_diff;
};
static struct qmaX981_stepcount step_count_index;

#endif

static bool int_status_flag = false;

#endif

#if defined(QMA6981_CHECK_ABNORMAL_DATA)
typedef struct
{
	int last_data;
	int curr_data;
	int more_data[3];
}qmaX981_data_check;

#define QMA6981_ABNORMAL_CHECK		30
static qmaX981_data_check g_qmaX981_data_c;

#endif
// add by yangzhiqiang
static unsigned char step_c_ref_report = 0;
// yangzhiqiang

/* Accelerometer information read by HAL */
static struct sensors_classdev qmaX981_acc_cdev = {
	.name = "accelerometer",
	.vendor = "QST Corporation",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "1228.8",
	.resolution = "0.6",
	.sensor_power = "0.35",
	.min_delay = QMA6981_ACCEL_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = QMA6981_ACCEL_MAX_POLL_INTERVAL_MS,
	.delay_msec = QMA6981_ACCEL_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#if defined(QMAX981_STEPCOUNTER)
/* stepcounter information read by HAL */
static struct sensors_classdev qmaX981_stepcount_cdev = {
	.name = "step_counter",
	.vendor = "QST Corporation",
	.version = 1,
	.type = SENSOR_TYPE_STEP_COUNTER,
	.max_range = "65536",
	.resolution = "1",
	.sensor_power = "0.35",	/* 0.5 mA */
	.min_delay = QMAX981_STEPCOUNTER_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = QMAX981_STEPCOUNTER_MAX_POLL_INTERVAL_MS,
	.delay_msec = QMAX981_STEPCOUNTER_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

static struct qmaX981_data *acc_qst = NULL;
static struct i2c_client *this_client = NULL;
static int qmaX981_power_set(struct qmaX981_data *data, bool on);
static int qma6981_initialize(struct i2c_client *client);
static int qma7981_initialize(struct i2c_client *client);
static int qma6100_initialize(struct i2c_client *client);
static int qmaX981_initialize(struct i2c_client *client);
static int qmaX981_read_raw_xyz( struct qmaX981_data *qma, int *data);

/***** I2C I/O function ***********************************************/
static int I2C_RxData(uint8_t *rxData,int length)
{
	int ret;
	struct i2c_client * i2c = this_client;
	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		MSE_ERR("transfer failed.");
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		MSE_ERR("transfer failed(size error).");
		return -ENXIO;
	}
#ifdef QST_DEBUG
	MSE_LOG("RxData: len=%02x, addr=%02x, data=%02x",length, addr, rxData[0]);
#endif
	return 0;
}

static int I2C_TxData(uint8_t *txData,int length)
{
	int ret;
	struct i2c_client * i2c = this_client;
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		MSE_ERR("transfer failed.");
		return ret;
	}
	else if (ret != ARRAY_SIZE(msg))
	{
		MSE_ERR("transfer failed(size error).");
		return -ENXIO;
	}

	MSE_LOG("TxData: len=%02x, addr=%02x data=%02x",length, txData[0], txData[1]);

	return 0;
}


static int qmaX981_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
	unsigned char databuf[2];

	databuf[0] = reg;
	databuf[1] = value;
	ret = I2C_TxData(databuf, 2);

	if(ret < 0)
	{
		return ret;
	}
	else
	{
		return 0;
	}
}

static int qmaX981_read_reg(unsigned char reg, unsigned char *buf, unsigned char len)
{
	int ret = 0;
	unsigned char databuf[8];
	unsigned char i;

	databuf[0] = reg;
	ret = I2C_RxData(databuf, len);
	if(ret < 0)
	{
		return ret;
	}	
	else
	{
		for(i=0; i<len; i++)
		{
			buf[i] = databuf[i];
		}
		return 0;
	}
}


static int qmax981_read_chip_id(void)
{
	unsigned char databuf[2];
	unsigned char chip_id;
	int res = 0;

	databuf[0] = QMA6981_CHIP_ID;
	res = I2C_RxData(databuf, 1);
	if(res)
	{
		MSE_LOG("read chip id error!!!");
		return -EFAULT;
	}
	chip_id = databuf[0];
	if((chip_id>=0xb0) && (chip_id<=0xb9))
	{
		acc_qst->chip_type = CHIP_TYPE_QMA6981;
	}
	else if((chip_id>=0xe0) && (chip_id<=0xe7))
	{
		acc_qst->chip_type = CHIP_TYPE_QMA7981;
	}
	else if(chip_id==0xe8)
	{
		acc_qst->chip_type = CHIP_TYPE_QMA6100;
	}
	else
	{
		acc_qst->chip_type = CHIP_TYPE_UNDEFINE;
		return -1;
	}
	MSE_LOG("chip id %x !!!", chip_id);

	return 0;
}

static int qmaX981_setreg_power(int en)
{
	int ret = 0;
	unsigned char data[2] = {0};

#if defined(QMAX981_STEPCOUNTER)
	return 0;
#endif
	data[0] = QMA6981_POWER;
	if(en)
		data[1] = 0x80;
	else
		data[1] = 0x00;
	
	ret = I2C_TxData(data, 2);
	if(ret)
	{
		MSE_ERR("qmaX981_setreg_power error!, %d\n", ret);
	}

	if(en)
	{
		if((acc_qst->chip_type == CHIP_TYPE_QMA7981)||(acc_qst->chip_type == CHIP_TYPE_QMA6100))
		{
			data[0] = 0x5f;
			data[1] = 0x80;
			ret = I2C_TxData(data, 2);
			data[0] = 0x5f;
			data[1] = 0x00;
			ret = I2C_TxData(data, 2);
			mdelay(20);
		}
	}
	return ret;
}

static int qmaX981_setreg_odr(unsigned int ms)
{
	u8	odr_reg[2];
	int ret;

#if defined(QMAX981_STEPCOUNTER)
	return 0;
#endif
	odr_reg[0] = QMA6981_ODR;
	if(ms <= 5){
		odr_reg[1] = QMA6981_ODR_250HZ;
	}
	else if(ms <= 10){
		odr_reg[1] = QMA6981_ODR_125HZ;
	}
	else if(ms <= 20){
		odr_reg[1] = QMA6981_ODR_62HZ;
	}
	else if(ms <= 50){
		odr_reg[1] = QMA6981_ODR_31HZ;
	}
	else if(ms <= 100){
		odr_reg[1] = QMA6981_ODR_16HZ;
	}
	else {
		odr_reg[1] = QMA6981_ODR_16HZ;
	}

	ret = I2C_TxData(odr_reg, 2);
	if(ret)
	{
		MSE_ERR("qmaX981_setreg_odr error!, %d\n", ret);
	}

	return ret;
}

#if defined(QMAX981_STEPCOUNTER)
static int qmaX981_setreg_reset_sc(void)
{
	int ret = 0;
	unsigned char data[2] = {0};
	unsigned char reg13_value = 0;

	data[0] = 0x13;
	I2C_RxData(data, 1);
	reg13_value = data[0];

	data[0] = 0x13;
	data[1] = 0x80; //clean data
	ret = I2C_TxData(data,2);
	if(ret < 0)
		MSE_ERR("qmaX981_setreg_reset_sc error!, %d\n", ret);

	mdelay(5);
	data[0] = 0x13;
	data[1] = reg13_value; //clean data
	ret = I2C_TxData(data,2);
	if(ret < 0)
		MSE_ERR("qmaX981_setreg_reset_sc error!, %d\n", ret);

	return ret;
}
#endif

static int qmaX981_open(struct inode *inode, struct file *file)
{
	file->private_data = acc_qst;
	return nonseekable_open(inode, file);
}

static int qmaX981_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long qmaX981_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	/* NOTE: In this function the size of "char" should be 1-byte. */
	//uint8_t mode;			/* for SET_MODE*/
	//int status;			/* for OPEN/CLOSE_STATUS */
	//int ret = 0;		/* Return value. */
	//void __user *argp = (void __user *)arg;
	//struct qmaX981_data *qma = file->private_data;

	switch (cmd)
	{
	case QMA_ACC_ENABLE:
		//TODO: Enable or Disable acc_workqueue
		MSE_LOG("QMA_ACC_ENABLE called.");
//		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf)))
//		{
//			MSE_ERR("copy_from_user failed.");
//			return -EFAULT;
//		}
		break;
	case QMA_STP_ENABLE:
		//TODO: Enable or Disable step counter - irq
		MSE_LOG("QMA_STP_ENABLE called.");
//		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf)))
//		{
//			MSE_ERR("copy_from_user failed.");
//			return -EFAULT;
//		}
		break;
	case QMA_STP_CLEAR:
#if defined(QMAX981_STEPCOUNTER)
		qmaX981_setreg_reset_sc();
#endif
		MSE_LOG("QMA_STP_CLEAR called.");
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations QMA6981_fops = {
	.owner = THIS_MODULE,
	.open = qmaX981_open,
	.release = qmaX981_release,
	.unlocked_ioctl = qmaX981_unlocked_ioctl,
};

static struct miscdevice qmaX981_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMA_MISCDEV_NAME,
	.fops = &QMA6981_fops,
};

static int qmaX981_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int ret = 0;

	struct qmaX981_data *qma = container_of(sensors_cdev,struct qmaX981_data, accel_cdev);
	MSE_LOG("%s enable=%d \n", __func__,enable);

	mutex_lock(&qma->sensor_mutex);
	qma->enable_flag = enable;

	if (enable)
	{

#if defined(QMA6981_USE_CALI)
		qmaX981_read_file(QMA6981_CALI_FILE, (char *)(qmaX981_cali), sizeof(qmaX981_cali));
#endif
#ifndef QMAX981_STEPCOUNTER
		ret = qmaX981_power_set(qma, true);
		if (ret) {
			MSE_ERR("Fail to power on the device!\n");
			goto mutex_exit;
		}
		
		ret = qmaX981_initialize();

		if (ret < 0)
			goto mutex_exit;
#endif
		schedule_delayed_work(&qma->dwork, (unsigned long)nsecs_to_jiffies64(qma->delay));
	}
	else
	{
		cancel_delayed_work_sync(&qma->dwork);
		#ifndef QMAX981_STEPCOUNTER
		ret = qmaX981_power_set(qma, false); //keep power on for sc	
		if (ret)
		{
			MSE_ERR("Fail to power off the device!\n");
			goto mutex_exit;
		}
		#endif
	}

#ifndef QMAX981_STEPCOUNTER
mutex_exit:
#endif
	mutex_unlock(&qma->sensor_mutex);
	//MSE_ERR("qmaX981_enable_set error!\n");
	return ret;
}

static int qmaX981_poll_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
	struct qmaX981_data *qma = container_of(sensors_cdev,
					struct qmaX981_data, accel_cdev);

	if (delay_msec < QMA6981_ACCEL_MIN_POLL_INTERVAL_MS)
		delay_msec = QMA6981_ACCEL_MIN_POLL_INTERVAL_MS;
	if (delay_msec > QMA6981_ACCEL_MAX_POLL_INTERVAL_MS)
		delay_msec = QMA6981_ACCEL_MAX_POLL_INTERVAL_MS;

	mutex_lock(&qma->val_mutex);
	qma->delay  = delay_msec * 1000*1000;
	mutex_unlock(&qma->val_mutex);

	qmaX981_setreg_odr(delay_msec);

	return 0;
}

static int qmaX981_axis_calibrate(int *cal_xyz)
{
	int xyz[3] = { 0 };
	int arry[3] = { 0 };
	int err;
	int i;

	for (i = 0; i < QMA6981_CAL_MAX; i++) {
		msleep(100);
		err = qmaX981_read_raw_xyz(acc_qst, xyz);
		if (err < 0) {
			printk("get_acceleration_data failed\n");
			return err;
		}
		if (i < QMA6981_CAL_SKIP_COUNT)
			continue;
		arry[0] += xyz[0];
		arry[1] += xyz[1];
		arry[2] += xyz[2];
	}
	cal_xyz[0] = arry[0] / (QMA6981_CAL_MAX - QMA6981_CAL_SKIP_COUNT);
	cal_xyz[1] = arry[1] / (QMA6981_CAL_MAX - QMA6981_CAL_SKIP_COUNT);
	cal_xyz[2] = arry[2] / (QMA6981_CAL_MAX - QMA6981_CAL_SKIP_COUNT);

	return 0;
}


static int qmaX981_calibrate(struct sensors_classdev *sensors_cdev,int axis, int apply_now)
{
	int err;
	int xyz[3] = { 0 };
	int raw_in_1g = 256;
#if 0
	if (acc_qst->enable_flag == false)
	{
		err = qmaX981_power_set(acc_qst, true);
		if (err) {
			MSE_ERR("Fail to power on the device!\n");
			return err;
		}
		err = qmaX981_initialize(acc_qst->i2c);
		if (err < 0)
			return err;
	}
#endif
	err = qmaX981_axis_calibrate(xyz);
	if(err)
	{
		MSE_LOG("qmaX981_calibrate qmaX981_axis_calibrate fail!\n");
		return err;
	}

	if(acc_qst->range == 0x01)
	{
		raw_in_1g = 256;
	}
	else if(acc_qst->range == 0x02)
	{
		raw_in_1g = 128;
	}
	else if(acc_qst->range == 0x04)
	{
		raw_in_1g = 64;
	}

	switch (axis) {
	case AXIS_X:
		xyz[1] = 0;
		xyz[2] = 0;
		break;
	case AXIS_Y:
		xyz[0] = 0;
		xyz[2] = 0;
		break;
	case AXIS_Z:
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = xyz[2] - raw_in_1g;
		break;
	case AXIS_XYZ:
		xyz[2] = xyz[2] - raw_in_1g;
		break;
	default:
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = 0;
		MSE_ERR( "can not calibrate accel\n");
		break;
	}
	memset(acc_qst->calibrate_buf, 0, sizeof(acc_qst->calibrate_buf));
	snprintf(acc_qst->calibrate_buf, sizeof(acc_qst->calibrate_buf),
			"%d,%d,%d", xyz[0], xyz[1], xyz[2]);
	if (apply_now) {
		acc_qst->cal_params[0] = xyz[0];
		acc_qst->cal_params[1] = xyz[1];
		acc_qst->cal_params[2] = xyz[2];
		acc_qst->use_cal = true;
	}

	return 0;
}


static int qmaX981_write_cal_params(struct sensors_classdev *sensors_cdev,struct cal_result_t *cal_result)
{
	acc_qst->cal_params[0] = cal_result->offset_x;
	acc_qst->cal_params[1] = cal_result->offset_y;
	acc_qst->cal_params[2] = cal_result->offset_z;

	snprintf(acc_qst->calibrate_buf, sizeof(acc_qst->calibrate_buf),
			"%d,%d,%d", acc_qst->cal_params[0], acc_qst->cal_params[1],
			acc_qst->cal_params[2]);
	acc_qst->use_cal = true;
	MSE_LOG( "qmaX981 read accel calibrate bias %d,%d,%d\n",
		acc_qst->cal_params[0], acc_qst->cal_params[1], acc_qst->cal_params[2]);

	return 0;
}


#if defined(QMAX981_STEPCOUNTER)
static int qmaX981_sc_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int ret = 0;

	struct qmaX981_data *qma = container_of(sensors_cdev,struct qmaX981_data, stepcount_cdev);

	mutex_lock(&qma->val_sc_mutex);
	qma->sc_enable_flag = enable;
	mutex_unlock(&qma->val_sc_mutex);

	mutex_lock(&qma->op_sc_mutex);
	if (enable)
	{
		step_c_ref_report = 1;       
		enable_irq(acc_qst->irq);
#if 0
		ret = qmaX981_power_set(qma, true);
		if (ret) {
			MSE_ERR("Fail to power on the device!\n");
			goto exit;
		}
		ret = qmaX981_initialize(qma->i2c);
		if (ret < 0)
			goto exit;
#endif
		schedule_delayed_work(&qma->sc_dwork, (unsigned long)nsecs_to_jiffies64(qma->sc_delay));
	}
	else
	{
		disable_irq_nosync(acc_qst->irq);
		cancel_delayed_work_sync(&qma->sc_dwork);
		//#ifndef QMAX981_STEPCOUNTER
		//ret = qmaX981_power_set(qma, false); //keep power on for sc
		//if (ret)
		//{
		//	MSE_ERR("Fail to power off the device!\n");
		//	goto exit;
		//}
		//#endif 
	}

//exit:
	mutex_unlock(&qma->op_sc_mutex);
	return ret;
}

static int qmaX981_sc_poll_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec){

	struct qmaX981_data *qma = container_of(sensors_cdev,struct qmaX981_data, stepcount_cdev);

	mutex_lock(&qma->val_sc_mutex);
	qma->sc_delay = delay_msec * 1000*1000;
	mutex_unlock(&qma->val_sc_mutex);

	return 0;
}
#endif

static ssize_t show_chipinfo_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	char strbuf[256];
	int output;
	unsigned char databuf;


	databuf = QMA6981_CHIP_ID;
	if(0 != I2C_RxData(&databuf, 1))
	{
		MSE_LOG("read chip id error!!!");
		return -EFAULT;
	}
	output = (int)databuf;

	sprintf(strbuf, "chipid:%d \n", output);

	return sprintf(buf, "%s\n", strbuf);
}


static ssize_t show_waferid_value(struct device *dev,
		struct device_attribute *attr, char *buf){
	int res;

	unsigned int chipid;
	unsigned char chipidh;
	unsigned char chipidl;

	unsigned char waferid;
	unsigned char waferid1;
	unsigned char waferid2;
	unsigned char waferid3;


	chipidh = 0x48;
	if((res = I2C_RxData(&chipidh, 1)))
	{
		MSE_LOG("read wafer chip h error!!!\n");
		return -EFAULT;
	}
	chipidl = 0x47;
	if((res = I2C_RxData(&chipidl, 1)))
	{
		MSE_LOG("read wafer chip l error!!!\n");
		return -EFAULT;
	}
	MSE_LOG("read wafer chip H:0x%x L:0x%x", chipidh, chipidl);
	chipid = (chipidh<<8)|chipidl;

	waferid1 = 0x59;
	if((res = I2C_RxData(&waferid1, 1)))
	{
		MSE_LOG("read wafer id 1 error!!!\n");
		return -EFAULT;
	}
	waferid2 = 0x41;
	if((res = I2C_RxData(&waferid2, 1)))
	{
		MSE_LOG("read wafer id 2 error!!!\n");
		return -EFAULT;
	}
	waferid3 = 0x40;
	if((res = I2C_RxData(&waferid3, 1)))
	{
		MSE_LOG("read wafer id 3 error!!!\n");
		return -EFAULT;
	}

	MSE_LOG("wafer ID: 0x%x 0x%x 0x%x\n", waferid1, waferid2, waferid3);

	waferid = (waferid1&0x10)|((waferid2>>4)&0x0c)|((waferid3>>6)&0x03);

	return sprintf(buf, " Chip ID:0x%x \n Wafer ID 0x%02x\n", chipid, waferid);
}

static ssize_t show_sensordata_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	int sensordata[3];
	char strbuf[64];

	qmaX981_read_raw_xyz(acc_qst, sensordata);
	MSE_LOG("sensordata %d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);
 
	memset(strbuf, 0, sizeof(strbuf));
	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}


static ssize_t show_dumpallreg_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
//	char strbuf[1024];
	char tempstrbuf[24];
	unsigned char databuf[2];
	int length=0;
	char  *strbuf;

	strbuf=(char*)kzalloc(sizeof(char)*1024, GFP_KERNEL);
	if(NULL != strbuf)
	{
		for(i =0;i<64;i++)
		{
			databuf[0] = i;
			res = I2C_RxData(databuf, 1);
			if(res < 0)
				MSE_LOG("qmaX981 dump registers 0x%02x failed !\n", i);

			length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "reg[0x%2x] = 0x%2x \n",i, databuf[0]);
			snprintf(strbuf+length*i , 1024-length*i, "%s",tempstrbuf);
		}
		res = scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
	}
	length = sprintf(buf, "%s\n", strbuf);
	kfree(strbuf);

	return length;
}

static ssize_t show_layout_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
//	char layout;
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	//mutex_lock(&qmaX981->val_mutex);
 //layout  = acc_qst->layout;
	//mutex_unlock(&qmaX981->val_mutex);

	return sprintf(buf,"current direction is %d\n",acc_qst->layout);
}

static ssize_t store_layout_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
//	struct i2c_client *client = to_i2c_client(dev);
//	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	int layout;

	if(1 == sscanf(buf, "%d", &layout)){
		//mutex_lock(&qmaX981->val_mutex);
		acc_qst->layout = layout;
		//mutex_unlock(&qmaX981->val_mutex);
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

unsigned char regbuf[2] = {0};
static ssize_t show_registers_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char data;
	int res;
	//sruct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	data = regbuf[0];

	res = I2C_RxData(&data, 1);
	if(res < 0)
	{
		MSE_LOG("%s failes\n",__func__);
	}

	MSE_LOG("%s REG[0x%2x] = 0x%02x\n",__func__,regbuf[0],data);

	return scnprintf(buf, PAGE_SIZE, "REG[0x%2x] = 0x%02x\n",regbuf[0],data); 
}

static ssize_t store_registers_value(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
//	struct i2c_client *client = to_i2c_client(dev);
//	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	int err = 0;
	unsigned char data[2] = {0};

	if(NULL == acc_qst)
	{
		MSE_ERR("qmaX981_data is NULL!\n");
		return 0;
	}

	sscanf(buf,"0x%2x",(unsigned int *)&regbuf[1]);

	data[1] = regbuf[1];
	data[0] = regbuf[0];
	err = I2C_TxData(data,2);

	if(err < 0)
	   MSE_ERR("%s failed\n",__func__);

	return count;
}

static ssize_t show_registers_addr(struct device *dev, struct device_attribute *attr, char *buf)
{

	MSE_LOG("%s REG_ADDR = 0x%02x\n",__func__,regbuf[0]);

	return scnprintf(buf, PAGE_SIZE, "REG_ADDR = 0x%02x\n", regbuf[0]);
}

static ssize_t store_registers_addr(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	if(NULL == acc_qst)
	{
		MSE_ERR("qmaX981_data is NULL!\n");
		return 0;
	}

	if(1 == sscanf(buf,"0x%2x",(unsigned int*)&regbuf[0])){
		MSE_LOG("%s REG_ADDR = 0x%2x\n",__func__,regbuf[0]);
	}else{
		regbuf[0] = 0x00;
	}

	return count;
}

static ssize_t qmaX981_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	//int64_t val;

	//mutex_lock(&qmaX981->val_mutex);
	//val = qmaX981->delay;
	//mutex_unlock(&qmaX981->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", acc_qst->delay);
}

static ssize_t qmaX981_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	unsigned long val = 0;

	//if (NULL == buf)
	//	return -EINVAL;

	//if (0 == count)
	//	return 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	//mutex_lock(&qmaX981->val_mutex);
	acc_qst->delay = val;
	//mutex_unlock(&qmaX981->val_mutex);

	return count;
}

static ssize_t qmaX981_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

//	int val;

	//mutex_lock(&qmaX981->val_mutex);
	//val = qmaX981->enable_flag;
	//mutex_unlock(&qmaX981->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", acc_qst->enable_flag);
}

static ssize_t qmaX981_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
//	struct i2c_client *client = to_i2c_client(dev);
//	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	unsigned long en = 0;
#ifndef QMAX981_STEPCOUNTER
	int ret;
#endif
	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtoul(buf, 10, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	MSE_LOG("%s: enable=%lu\n",__func__, en);

//	mutex_lock(&qmaX981->op_mutex);

	acc_qst->enable_flag = en;
	if (en)
	{
#ifndef QMAX981_STEPCOUNTER
		ret = qmaX981_power_set(acc_qst, true);
		if (ret) {
			MSE_ERR("Fail to power on the device!\n");
			goto mutex_exit;
		}
		ret = qmaX981_initialize(acc_qst->i2c);
		//if (ret < 0)
			//goto mutex_exit;
#endif
		schedule_delayed_work(&acc_qst->dwork, (unsigned long)nsecs_to_jiffies64(acc_qst->delay));
	}
	else
	{
		cancel_delayed_work_sync(&acc_qst->dwork);
#ifndef QMAX981_STEPCOUNTER
		ret = qmaX981_power_set(acc_qst, false); //keep power on for sc	
		if (ret)
		{
			MSE_ERR("Fail to power off the device!\n");
			//goto mutex_exit;
		}
#endif
	}
//#ifndef QMAX981_STEPCOUNTER
//mutex_exit:
//#endif
//	mutex_unlock(&qmaX981->op_mutex);

	return count;
}

#if defined(QMA6981_USE_CALI)
static void qmaX981_write_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDWR|O_CREAT, 0666);
	if (IS_ERR(fp))
	{
		printk("qmaX981_write_file open file error\n");
	}
	else
	{
		//printk("qmaX981_write_file data=0x%x len=%d\n", data, len);
		//snprintf();
		fp->f_op->write(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
	}

	set_fs(fs);
}

static void qmaX981_read_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	if(qmaX981_cali_flag == 1)
	{
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fp))
	{
		printk("qmaX981_read_file open file error\n");
	}
	else
	{
		//printk("qmaX981_read_file data=0x%x len=%d\n", data, len);
		fp->f_op->read(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
		qmaX981_cali_flag = 1;
	}

	set_fs(fs);
}

static ssize_t qmaX981_cali_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", qmaX981_cali[0], qmaX981_cali[1], qmaX981_cali[2]);
}

static ssize_t qmaX981_cali_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned long en = 0;
	int data[3], data_avg[3];
	int icount, z_max, z_min;

	if (strict_strtoul(buf, 10, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	if(en)
	{
		data_avg[0] = 0;
		data_avg[1] = 0;
		data_avg[2] = 0;
		for(icount=0; icount<QMA6981_CALI_NUM; icount++)
		{
			qmaX981_read_raw_xyz(acc_qst, data);
			data_avg[0] += data[0];
			data_avg[1] += data[1];
			data_avg[2] += data[2];
			// add by yangzhiqiang check vibrate
			if(icount == 0)
			{
				z_max = data[2];
				z_min = data[2];
			}
			else
			{
				z_max = (data[2]>z_max)?data[2]:z_max;
				z_min = (data[2]<z_min)?data[2]:z_min;
			}
			// add by yangzhiqiang check vibrate
			mdelay(5);
		}
		// add by yangzhiqiang check vibrate
		if((z_max-z_min)>(QMA6981_LSB_1G*3/10))
		{
			printk("qmaX981_cali_store check vibrate cali ingore!\n");
			return count;
		}
		// add by yangzhiqiang check vibrate

		data_avg[0] = data_avg[0]/QMA6981_CALI_NUM;
		data_avg[1] = data_avg[1]/QMA6981_CALI_NUM;
		data_avg[2] = data_avg[2]/QMA6981_CALI_NUM;
		printk("qmaX981_cali_store data_avg[%d %d %d]\n", data_avg[0], data_avg[1], data_avg[2]);
		// add by yangzhiqiang check offset range
#if 0
		if(QMA6981_ABS(data_avg[2]-QMA6981_LSB_1G)>(QMA6981_LSB_1G*5/10))
		{
			printk("qmaX981_cali_store check offset range cali ingore!\n");
			return count;
		}
#endif
		// add by yangzhiqiang check offset range
		data[0] = 0-data_avg[0];
		data[1] = 0-data_avg[1];
		data[2] = QMA6981_LSB_1G-data_avg[2];
		qmaX981_cali[0] += data[0];
		qmaX981_cali[1] += data[1];
		qmaX981_cali[2] += data[2];
		printk("qmaX981_cali_store offset[%d %d %d]\n", data[0], data[1], data[2]);
		printk("qmaX981_cali_store qmaX981_cali[%d %d %d]\n", qmaX981_cali[0], qmaX981_cali[1], qmaX981_cali[2]);
		qmaX981_write_file(QMA6981_CALI_FILE, (char *)qmaX981_cali, sizeof(qmaX981_cali));
		
	}
	else
	{
	}

	return count;
}
#endif


static ssize_t qmaX981_sc_clear(struct device *dev,struct device_attribute *attr, char *buf)
{
#if defined(QMAX981_STEPCOUNTER)
	qmaX981_setreg_reset_sc();
#endif
	return scnprintf(buf, PAGE_SIZE, "step clear done\n");
}

		
DEVICE_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
DEVICE_ATTR(waferid,    S_IRUGO, show_waferid_value, NULL);
DEVICE_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
DEVICE_ATTR(dumpallreg,  S_IRUGO , show_dumpallreg_value, NULL);
DEVICE_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
DEVICE_ATTR(registers_value, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH, show_registers_value, store_registers_value);
DEVICE_ATTR(registers_addr,  S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH, show_registers_addr, store_registers_addr);
DEVICE_ATTR(delay_acc, S_IRUGO | S_IWUSR, qmaX981_delay_show, qmaX981_delay_store);
DEVICE_ATTR(enable_acc, S_IRUGO | S_IWUSR, qmaX981_enable_show, qmaX981_enable_store);
#if defined(QMA6981_USE_CALI)
DEVICE_ATTR(cali, S_IRUGO | S_IWUSR, qmaX981_cali_show, qmaX981_cali_store);
#endif
DEVICE_ATTR(sc_clear, S_IRUGO, qmaX981_sc_clear, NULL);


static struct attribute *acc_attributes[] = {
	&dev_attr_chipinfo.attr,
	&dev_attr_waferid.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_layout.attr,
	&dev_attr_registers_value.attr,
	&dev_attr_registers_addr.attr,
	&dev_attr_delay_acc.attr,
	&dev_attr_enable_acc.attr,
#if defined(QMA6981_USE_CALI)
	&dev_attr_cali.attr,
#endif
	&dev_attr_sc_clear.attr,

	NULL
};

static struct attribute_group acc_attribute_group = {
	.name = "driver",
	.attrs = acc_attributes
};

/***** qma input device functions ***********************************/
static int qmaX981_input_init( struct i2c_client *client,
	struct qmaX981_data *sensor)
{
	int ret = 0;

	sensor->accel_dev = devm_input_allocate_device(&client->dev);
	if (!sensor->accel_dev) {
		MSE_ERR("Failed to allocate accelerometer input device\n");
		return -ENOMEM;
	}

	sensor->accel_dev->name = QMA_DEV_NAME_ACCEL;
	sensor->accel_dev->id.bustype = BUS_I2C;
	sensor->acc_use_cal = false;

	input_set_capability(sensor->accel_dev, EV_ABS, ABS_X);
	input_set_capability(sensor->accel_dev, EV_ABS, ABS_Y);
	input_set_capability(sensor->accel_dev, EV_ABS, ABS_Z);
	input_set_abs_params(sensor->accel_dev, ABS_X,QMA6981_ACCEL_MIN_VALUE, QMA6981_ACCEL_MAX_VALUE,0, 0);
	input_set_abs_params(sensor->accel_dev, ABS_Y,QMA6981_ACCEL_MIN_VALUE, QMA6981_ACCEL_MAX_VALUE,0, 0);
	input_set_abs_params(sensor->accel_dev, ABS_Z,QMA6981_ACCEL_MIN_VALUE, QMA6981_ACCEL_MAX_VALUE,0, 0);

	sensor->accel_dev->dev.parent = &client->dev;
	input_set_drvdata(sensor->accel_dev, sensor);
	ret = input_register_device(sensor->accel_dev);
	if (ret) {
		input_free_device(sensor->accel_dev);
		MSE_ERR("Failed to register input device\n");
		return -ENODEV;
	}

#if defined(QMAX981_STEPCOUNTER)
	sensor->stepcount_dev = devm_input_allocate_device(&client->dev);
	if (!sensor->stepcount_dev) {

		input_unregister_device(sensor->accel_dev);
		input_free_device(sensor->accel_dev);

		dev_err(&client->dev,
			"Failed to allocate stepcount input device\n");
		return -ENOMEM;
	}

	sensor->stepcount_dev->name = QMA_DEV_NAME_STEPCOUNTER;
	sensor->stepcount_dev->id.bustype = BUS_I2C;

	input_set_capability(sensor->stepcount_dev, EV_ABS, ABS_RX);
	input_set_capability(sensor->stepcount_dev, EV_REL, REL_RX);
	input_set_abs_params(sensor->stepcount_dev, ABS_RX,QMAX981_STEPCOUNTER_MIN_VALUE, QMAX981_STEPCOUNTER_MAX_VALUE,0, 0);
	input_set_abs_params(sensor->stepcount_dev, REL_RX,QMAX981_STEPCOUNTER_MIN_VALUE, QMAX981_STEPCOUNTER_MAX_VALUE,0, 0);
	
	sensor->stepcount_dev->dev.parent = &client->dev;
	input_set_drvdata(sensor->stepcount_dev, sensor);

	ret = input_register_device(sensor->stepcount_dev);
	if (ret) {
		MSE_ERR("Failed to register input device\n");

		input_unregister_device(sensor->accel_dev);
		input_free_device(sensor->accel_dev);

		input_free_device(sensor->stepcount_dev);
		return ret;
	}
#endif

	return ret;
}

static int qmaX981_input_destory( struct i2c_client *client,
	struct qmaX981_data *sensor)
{
	input_unregister_device(sensor->accel_dev);
	input_free_device(sensor->accel_dev);

#if defined(QMAX981_STEPCOUNTER)
	input_unregister_device(sensor->stepcount_dev);
	input_free_device(sensor->stepcount_dev);
#endif

	return 0;
}


static int qmaX981_suspend(struct device *dev)
{
	struct qmaX981_data *qma = dev_get_drvdata(dev);
	int ret = 0;

	cancel_delayed_work_sync(&qma->dwork);
#if defined(QMAX981_STEPCOUNTER)
	cancel_delayed_work_sync(&qma->sc_dwork);
	return ret;
#endif
	if(qma->enable_flag)
	{
		qmaX981_setreg_power(0);
		qmaX981_power_set(qma,false);
	}

	qma->state.power_on = qma->power_enabled;
#ifdef QST_IRQ
	ret = pinctrl_select_state(qma->pinctrl, qma->pin_sleep);
	if (ret)
		dev_err(dev, "Can't select pinctrl state\n");
#endif

	MSE_LOG("suspended");

	return ret;
}

static int qmaX981_resume(struct device *dev)
{
	struct qmaX981_data *qma = dev_get_drvdata(dev);
	int ret = 0;

#if defined(QMAX981_STEPCOUNTER)
	if(qma->enable_flag)
	{
		schedule_delayed_work(&qma->dwork,(unsigned long)nsecs_to_jiffies64(qma->delay));
	}
	if(qma->sc_enable_flag)
	{
		schedule_delayed_work(&qma->sc_dwork,(unsigned long)nsecs_to_jiffies64(qma->sc_delay));
		step_c_ref_report = 1;
	}
	return ret;
#endif

	if (qma->enable_flag)
	{
		ret = qmaX981_power_set(qma, true);
		if (ret) {
			MSE_ERR("Fail to power on the device!\n");
			return ret;
		}
		ret = qmaX981_initialize(qma->i2c);
		if (ret < 0)
			return ret;
		schedule_delayed_work(&qma->dwork,(unsigned long)nsecs_to_jiffies64(qma->delay));
	}
#ifdef QST_IRQ
	ret = pinctrl_select_state(qma->pinctrl, qma->pin_default);
	if (ret)
	{
		MSE_ERR("Can't select pinctrl state\n");
	}
#endif
	MSE_LOG("resumed");

	return ret;
}

#if defined(QMAX981_STEPCOUNTER)
static int qma6981_initialize(struct i2c_client *client)
{

	int ret = 0;
	int direction = acc_qst->layout;
	unsigned char data[2] = {0};
	MSE_FUN();
	//range  2g , 3.9mg/LSB

	acc_qst->range = QMAX981_RANGE_8G;
	data[0] = 0x0F;
	data[1] = QMAX981_RANGE_8G;
	
	if(data[1] == QMAX981_RANGE_4G)
		acc_qst->resolution = 128;
	else if(data[1] == QMAX981_RANGE_8G)
		acc_qst->resolution = 64;
	else					
		acc_qst->resolution = 256;

	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	//lower output data rate ,Bandwidth = 62.5Hz ODR = 2*BW = 125Hz
	data[0] = 0x10;
	data[1] = 0x2a;
	ret = I2C_TxData(data,2);
	if(ret < 0)
	  goto exit_i2c_err;

//active mode, sleep time 10ms
	data[0] = 0x11;
	data[1] = 0x80;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	//0x12,start step_count, sample counts 48
	data[0] = 0x12;
	data[1] = 0x8f;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	//0x13, dynamic precision 250mg
	data[0] = 0x13;
	data[1] = 0x10;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	//0x14,time window low 0.324s
	data[0] = 0x14;
	data[1] = 0x14;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	//0x15,time window up 2.16s
	data[0] = 0x15;
	data[1] = 0x10;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	//0x32,0x00 xy,
	data[0] = 0x32;
	if(direction%2)
		data[1] = 0x01;
	else
		data[1] = 0x02;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	data[0] = 0x27;
	data[1] = 0x64;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	data[0] = 0x28;
	data[1] = 0x64;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	data[0] = 0x29;
	data[1] = 0x64;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	data[0] = 0x16;
	data[1] = 0x0c;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

#if defined(QMAX981_STEPCOUNTER_USE_INT)
	// int1_step  int1_step_quit
	data[0] = 0x19;
	data[1] = 0x08;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;
#endif
	return 0;

exit_i2c_err:
	MSE_ERR("qma6981_initialize fail: %d\n",ret);
	return ret;
}
#else
static int qma6981_initialize(struct i2c_client *client)
{
	int ret = 0;
	unsigned char data[2] = {0};

	MSE_FUN();

	acc_qst->range = 0x02;
	data[0] = 0x0F;
	data[1] = QMAX981_RANGE_4G;

	if(data[1] == QMAX981_RANGE_4G)
		acc_qst->resolution = 128;
	else if(data[1] == QMAX981_RANGE_8G)
		acc_qst->resolution = 64;
	else					
		acc_qst->resolution = 256;
	
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	//lower output data rate ,Bandwidth = 62.5Hz ODR = 2*BW = 125Hz
	data[0] = 0x10;
	data[1] = 0x0c;
	ret = I2C_TxData(data,2);
	if(ret < 0)
	  goto exit_i2c_err;

//active mode, sleep time 0 ms
	data[0] = 0x11;
	data[1] = 0x80;
	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;

	return 0;

	exit_i2c_err:
		MSE_ERR("qma6981_initialize fail: %d\n",ret);
		return ret;
}
#endif

static int qma7981_initialize(struct i2c_client *client)
{
	//int ret = 0;
	unsigned char reg_0x0f = 0;
	unsigned char reg_0x10 = 0;
	unsigned char reg_0x16 = 0;
	unsigned char reg_0x18 = 0;
	unsigned char reg_0x19 = 0;
	unsigned char reg_0x1a = 0;
#if defined(QMA7981_ANY_MOTION)||defined(QMA7981_NO_MOTION)
	unsigned char reg_0x2c = 0;
#endif
#if defined(QMA7981_HAND_UP_DOWN)
	//unsigned char reg_0x42 = 0;
#endif

	qmaX981_write_reg(0x36, 0xb6);
	mdelay(5);
	qmaX981_write_reg(0x36, 0x00);
	qmaX981_write_reg(0x11, 0x80);

	reg_0x0f = QMAX981_RANGE_4G;
	qmaX981_write_reg(0x0f, reg_0x0f);	
	if(reg_0x0f == QMAX981_RANGE_4G)
		acc_qst->resolution = 2048;
	else if(reg_0x0f == QMAX981_RANGE_8G)
		acc_qst->resolution = 1024;
	else if(reg_0x0f == QMAX981_RANGE_16G)
		acc_qst->resolution = 512;
	else if(reg_0x0f == QMAX981_RANGE_32G)
		acc_qst->resolution = 256;
	else
		acc_qst->resolution = 4096;

	reg_0x10 = 0xe1;
	qmaX981_write_reg(0x10, reg_0x10);

	qmaX981_read_reg(0x16, &reg_0x16, 1);
	qmaX981_read_reg(0x18, &reg_0x18, 1);
	qmaX981_read_reg(0x19, &reg_0x19, 1);
	qmaX981_read_reg(0x1a, &reg_0x1a, 1);

#if defined(QMAX981_STEPCOUNTER)
	if(reg_0x10 == 0xe0)
	{
		// ODR: 65hz 15.48 ms
		qmaX981_write_reg(0x12, 0x94);
		qmaX981_write_reg(0x13, 0x80);		// clear step
		qmaX981_write_reg(0x13, 0x7f);		// 0x00
		qmaX981_write_reg(0x14, 0x12);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_write_reg(0x15, 0x10);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	else if(reg_0x10 == 0xe1)
	{
		// ODR: 130hz 7.74 ms
		qmaX981_write_reg(0x12, 0x94);
		qmaX981_write_reg(0x13, 0x80);		// clear step
		qmaX981_write_reg(0x13, 0x7f);		// 0x00
		qmaX981_write_reg(0x14, 0x24);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_write_reg(0x15, 0x20);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	else if(reg_0x10 == 0xe2)
	{
		// ODR: 258Hz 3.87 ms
		qmaX981_write_reg(0x12, 0x94);
		qmaX981_write_reg(0x13, 0x80);		// clear step
		qmaX981_write_reg(0x13, 0x7f);		// 0x00
		qmaX981_write_reg(0x14, 0x48);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_write_reg(0x15, 0x40);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	
#if defined(QMA7981_STEP_INT)
	reg_0x16 |= 0x08;
	reg_0x19 |= 0x08;
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
#endif
#if defined(QMA7981_SIGNIFICANT_STEP)
	qmaX981_write_reg(0x1d, 0x26);		//every 30 step
	reg_0x16 |= 0x40;
	reg_0x19 |= 0x40;
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
#endif
#endif

//RANGE<3:0> Acceleration range Resolution
//0001 2g 244ug/LSB
//0010 4g 488ug/LSB
//0100 8g 977ug/LSB
//1000 16g 1.95mg/LSB
//1111 32g 3.91mg/LSB
//Others 2g 244ug/LSB

//0x2c
//Duration = (NO_MOT_DUR<3:0>   1) * 1s, if NO_MOT_DUR<5:4> =b00 
//Duration = (NO_MOT_DUR<3:0>   4) * 5s, if NO_MOT_DUR<5:4> =b01 
//Duration = (NO_MOT_DUR<3:0>   10) * 10s, if NO_MOT_DUR<5:4> =b1x 
//ANY_MOT_DUR<1:0>: any motion interrupt will be triggered when slope > ANY_MOT_TH for (ANY_MOT_DUR<1:0>   1) samples 

//0x2e ANY MOTION MOT_CONF2
//TH= ANY_MOT_TH<7:0> * 16 * LSB 

#if defined(QMA7981_ANY_MOTION)
	reg_0x18 |= 0x07;
	reg_0x1a |= 0x01;
	reg_0x2c |= 0x00;
	
	qmaX981_write_reg(0x18, reg_0x18);
	qmaX981_write_reg(0x1a, reg_0x1a);
	qmaX981_write_reg(0x2c, reg_0x2c);
	qmaX981_write_reg(0x2e, 0x14);		// 0.488*16*32 = 250mg
			
#if defined(QMA7981_SIGNIFICANT_MOTION)
	qmaX981_write_reg(0x2f, 0x0c|0x01);
	reg_0x19 |= 0x01;
	qmaX981_write_reg(0x19, reg_0x19);
#endif
#endif
	
#if defined(QMA7981_NO_MOTION)
	reg_0x18 |= 0xe0;
	reg_0x1a |= 0x80;
	reg_0x2c |= 0x24;

	qmaX981_write_reg(0x18, reg_0x18);
	qmaX981_write_reg(0x1a, reg_0x1a);
	qmaX981_write_reg(0x2c, reg_0x2c);
	qmaX981_write_reg(0x2d, 0x14);
#endif

#if defined(QMA7981_HAND_UP_DOWN)
	reg_0x16 |= 0x02;
	reg_0x19 |= 0x02;
			
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
	// hand down
	reg_0x16 |= 0x04;
	reg_0x19 |= 0x04;
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
	// hand down	
#if 0	// swap xy
	//read_reg(0x42, &reg_0x42, 1);
	reg_0x42 = 0x42;
	qmaX981_RxData(&reg_0x42, 1);
	reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
	qmaX981_write_reg(0x42, reg_0x42);
#endif
#endif

#if defined(QMA7981_DATA_READY)
	reg_0x1a |= 0x10;
	qmaX981_write_reg(0x17, 0x10);
	qmaX981_write_reg(0x1a, reg_0x1a);
#endif
	
#if defined(QMA7981_INT_LATCH)
	//qmaX981_write_reg(0x21, 0x3f);	// default 0x1c, step latch mode
	qmaX981_write_reg(0x21, 0x1f);	// default 0x1c, step latch mode
#endif

	qmaX981_write_reg(0x11, 0x80);
	qmaX981_write_reg(0x5f, 0x80);
	qmaX981_write_reg(0x5f, 0x00);
	mdelay(20);

	return 0;
}

static int qma6100_initialize(struct i2c_client *client)
{
	unsigned char reg_0x0f;

	qmaX981_write_reg(0x36, 0xb6);
	mdelay(5);
	qmaX981_write_reg(0x36, 0x00);
	qmaX981_write_reg(0x11, 0x80);

	reg_0x0f = QMAX981_RANGE_4G;
	qmaX981_write_reg(0x0f, QMAX981_RANGE_4G);	
	if(reg_0x0f == QMAX981_RANGE_4G)
		acc_qst->resolution = 2048;
	else if(reg_0x0f == QMAX981_RANGE_8G)
		acc_qst->resolution = 1024;
	else if(reg_0x0f == QMAX981_RANGE_16G)
		acc_qst->resolution = 512;
	else if(reg_0x0f == QMAX981_RANGE_32G)
		acc_qst->resolution = 256;
	else
		acc_qst->resolution = 4096;

	qmaX981_write_reg(0x10, 0xe0);

	qmaX981_write_reg(0x11, 0x80);
	qmaX981_write_reg(0x5f, 0x80);
	qmaX981_write_reg(0x5f, 0x00);
	mdelay(20);

	return 0;
}

static int qmaX981_initialize(struct i2c_client *client)
{
	int err=0;

	if(acc_qst->chip_type == CHIP_TYPE_QMA6981)
	{
		err = qma6981_initialize(client);
	}
	else if(acc_qst->chip_type == CHIP_TYPE_QMA7981)
	{
		err = qma7981_initialize(client);
	}	
	else if(acc_qst->chip_type == CHIP_TYPE_QMA6100)
	{
		err = qma6100_initialize(client);
	}

	return err;
}

static int qmaX981_power_set(struct qmaX981_data *data, bool on)
{
	int ret = 0;
	int err = 0;

#if defined(QMAX981_STEPCOUNTER)
	on = true;
#endif
	if (!on && data->power_enabled)
	{
		ret = regulator_disable(data->vdd);
		if (ret)
		{
			MSE_ERR("Regulator vdd disable failed ret=%d", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			MSE_ERR(
				"Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	}
	else if (on && !data->power_enabled)
	{
		ret = regulator_enable(data->vdd);
		if (ret)
		{
			MSE_ERR("Regulator vdd enable failed ret=%d",ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			MSE_ERR("Regulator vio enable failed ret=%d",ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	}
	else
	{
		MSE_LOG("Power on=%d. enabled=%d",on, data->power_enabled);
	}

	return ret;
}

static int qmaX981_read_raw_xyz( struct qmaX981_data *qma, int *data)
{
	uint8_t dat_buf[6];
	int err;
//	data_convert_s out;
	short raw_data[3];
	int acc[3];
	int acc_layout[3];
	
	/* Read raw data */
	dat_buf[0] = QMA6981_XOUTL;
	err = I2C_RxData(dat_buf, 6);
	if (err < 0) {
		MSE_ERR("%s failed.\n",__func__);
		return err;
	}
//	MSE_LOG("raw data lsb: %d %d %d %d %d %d\n",
//	dat_buf[0], dat_buf[1], dat_buf[2], dat_buf[3],dat_buf[4], dat_buf[5]);

	if(qma->chip_type == CHIP_TYPE_QMA6981)
	{
#if 0
		acc[0] = out.i = (int)((dat_buf[1]<<2) |( dat_buf[0]>>6));
		acc[1] = out.i = (int)((dat_buf[3]<<2) |( dat_buf[2]>>6));
		acc[2] = out.i = (int)((dat_buf[5]<<2) |( dat_buf[4]>>6));
#else
		raw_data[0] = (short)(((short)dat_buf[1]<<8)|(dat_buf[0]));
		raw_data[1] = (short)(((short)dat_buf[3]<<8)|(dat_buf[2]));
		raw_data[2] = (short)(((short)dat_buf[5]<<8)|(dat_buf[4]));
		raw_data[0] = raw_data[0]>>6;
		raw_data[1] = raw_data[1]>>6;
		raw_data[2] = raw_data[2]>>6;
     
		acc[QMA6981_AXIS_X] = raw_data[0];
		acc[QMA6981_AXIS_Y] = raw_data[1];
		acc[QMA6981_AXIS_Z] = raw_data[2];
#endif

#if defined(QMAX981_STEPCOUNTER)	
		acc[QMA6981_AXIS_X] -= 100;
		acc[QMA6981_AXIS_Y] -= 100;
		acc[QMA6981_AXIS_Z] -= 100;
#endif
		acc_layout[QMA6981_AXIS_X] = acc[qma->cvt.map[QMA6981_AXIS_X]] * qma->cvt.sign[QMA6981_AXIS_X];
		acc_layout[QMA6981_AXIS_Y] = acc[qma->cvt.map[QMA6981_AXIS_Y]] * qma->cvt.sign[QMA6981_AXIS_Y];
		acc_layout[QMA6981_AXIS_Z] = acc[qma->cvt.map[QMA6981_AXIS_Z]] * qma->cvt.sign[QMA6981_AXIS_Z];
     
	}
	else if((qma->chip_type == CHIP_TYPE_QMA7981)||(qma->chip_type == CHIP_TYPE_QMA6100))
	{
		raw_data[0] = (short)(((short)dat_buf[1]<<8)|(dat_buf[0]));
		raw_data[1] = (short)(((short)dat_buf[3]<<8)|(dat_buf[2]));
		raw_data[2] = (short)(((short)dat_buf[5]<<8)|(dat_buf[4]));
		raw_data[0] = raw_data[0]>>2;
		raw_data[1] = raw_data[1]>>2;
		raw_data[2] = raw_data[2]>>2;
		
		acc_layout[QMA6981_AXIS_X] = raw_data[qma->cvt.map[QMA6981_AXIS_X]] * qma->cvt.sign[QMA6981_AXIS_X];
		acc_layout[QMA6981_AXIS_Y] = raw_data[qma->cvt.map[QMA6981_AXIS_Y]] * qma->cvt.sign[QMA6981_AXIS_Y];
		acc_layout[QMA6981_AXIS_Z] = raw_data[qma->cvt.map[QMA6981_AXIS_Z]] * qma->cvt.sign[QMA6981_AXIS_Z];
	}
	
	data[QMA6981_AXIS_X] = (int)((acc_layout[QMA6981_AXIS_X]*1000)/qma->resolution);
	data[QMA6981_AXIS_Y] = (int)((acc_layout[QMA6981_AXIS_Y]*1000)/qma->resolution);
	data[QMA6981_AXIS_Z] = (int)((acc_layout[QMA6981_AXIS_Z]*1000)/qma->resolution);

#if defined(QMA6981_USE_CALI)
	data[QMA6981_AXIS_X] += qmaX981_cali[QMA6981_AXIS_X];
	data[QMA6981_AXIS_Y] += qmaX981_cali[QMA6981_AXIS_Y];
	data[QMA6981_AXIS_Z] += qmaX981_cali[QMA6981_AXIS_Z];
#endif
	if(acc_qst->use_cal == true)
	{
		data[QMA6981_AXIS_X] -= acc_qst->cal_params[QMA6981_AXIS_X];
		data[QMA6981_AXIS_Y] -= acc_qst->cal_params[QMA6981_AXIS_Y];
		data[QMA6981_AXIS_Z] -= acc_qst->cal_params[QMA6981_AXIS_Z];
	}

	return 0;
}

static int qmaX981_report_data(struct qmaX981_data *qma)
{
	//uint8_t dat_buf[6];
	int ret;
	int acc[3];
	//int tmp;


	ktime_t ts;
	ts = ktime_get_boottime();

	ret = qmaX981_read_raw_xyz(qma, acc);
	if (ret)
		MSE_ERR("read xyz data err!");

	MSE_LOG("acc[%d %d %d]\n",acc[0], acc[1], acc[2]);

	input_report_abs(qma->accel_dev, ABS_X, acc[0]);
	input_report_abs(qma->accel_dev, ABS_Y, acc[1]);
	input_report_abs(qma->accel_dev, ABS_Z, acc[2]);
	input_event(qma->accel_dev, EV_SYN, SYN_TIME_SEC,ktime_to_timespec(ts).tv_sec);
	input_event(qma->accel_dev, EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(ts).tv_nsec);
	input_sync(qma->accel_dev);

	return 0;
}

static void qmaX981_dev_poll(struct work_struct *work)
{
	struct qmaX981_data *qma;
	int ret;

	qma = container_of((struct delayed_work *)work, struct qmaX981_data, dwork);

	ret = qmaX981_report_data(qma);
	if (ret < 0)
		MSE_LOG("Failed to report data");
	schedule_delayed_work(&qma->dwork,(unsigned long)nsecs_to_jiffies64(acc_qst->delay));
}

#if defined(QMAX981_STEPCOUNTER)
static void qmaX981_set_wakelock(int en)
{
	return;
}

int qmaX981_read_step(void)
{
	int res = 0;
	int i, count;
	uint8_t databuf[2];
	uint8_t databuf_h[2];

	qmaX981_set_wakelock(1);

	databuf[0] = QMA6981_STEP_CNT;
	databuf_h[0] = 0x0e;

	if(acc_qst->chip_type == CHIP_TYPE_QMA6981)
	{
		for(i=0; i < 3; i++)
		{
			res = I2C_RxData(databuf, 2);
			if(res == 0) 
			{
				break;
			}
		}
		count = ((int)databuf[1]<<8)|databuf[0];
	}
	else if(acc_qst->chip_type == CHIP_TYPE_QMA7981)
	{
		for(i=0; i < 3; i++)
		{
			res = I2C_RxData(databuf, 2);
			res += I2C_RxData(databuf_h, 1);
			if(res == 0) 
			{
				break;
			}
		}
		count = (((int)databuf_h[0]<<16)|((int)databuf[1]<<8)|databuf[0]);
	}
	
	if(res)
	{
		MSE_ERR("qmaX981_read_step error!");
		qmaX981_set_wakelock(0);
		return -1;
	}
	
	qmaX981_set_wakelock(0);

	return count;
}


int qmaX981_sc_get_data(void)
{
	int count = 0;
#if defined(QMA6981_CHECK_ABNORMAL_DATA)
	unsigned int diff;
#endif

	count = qmaX981_read_step();
	if(count < 0)
		return count;

#if defined(QMA6981_CHECK_ABNORMAL_DATA)
	g_qmaX981_data_c.curr_data = count;
	diff = QMA6981_DIFF(g_qmaX981_data_c.curr_data, g_qmaX981_data_c.last_data);
	if(diff > QMA6981_ABNORMAL_CHECK)
	{
		count = qmaX981_read_step();
		if(count < 0)
			return count;

		g_qmaX981_data_c.more_data[0] = count;

		count = qmaX981_read_step();
		if(count < 0)
			return count;

		g_qmaX981_data_c.more_data[1] = count;

		
		count = qmaX981_read_step();
		if(count < 0)
			return count;

		g_qmaX981_data_c.more_data[2] = count;

		if((g_qmaX981_data_c.more_data[0]==g_qmaX981_data_c.more_data[1])
			&&(g_qmaX981_data_c.more_data[1]==g_qmaX981_data_c.more_data[2]))
		{
			count = g_qmaX981_data_c.more_data[0];
			g_qmaX981_data_c.curr_data = g_qmaX981_data_c.more_data[0];
		}
		else
		{		
			return -1;
		}
	}
	g_qmaX981_data_c.last_data = count;
#endif

	return count;
}


#if defined(QMA6981_INT_ALGO_2)
static int qmaX981_sc_report_data(struct qmaX981_data *qma)
{
	int count;
	unsigned int diff;

	count = qmaX981_sc_get_data();
	if(count < 0)
	{
		return -1;
	}
#ifdef QMAX981_STEPCOUNTER_USE_INT
	g_sc_algo.curr = count;

	MSE_LOG("thread_read int_status_flag=%d\n", int_status_flag);
	MSE_LOG("thread_read step_valid=%d\n", g_sc_algo.step_valid);
	MSE_LOG("thread_read curr=%d start=%d diff=%d\n", g_sc_algo.curr,g_sc_algo.start, QMA6981_DIFF(g_sc_algo.curr, g_sc_algo.start));
	if(int_status_flag)
	{
		// stepcounter int start
		if(g_sc_algo.step_valid == false)
		{
			diff = QMA6981_DIFF(g_sc_algo.curr, g_sc_algo.start);
			if(diff >= g_sc_algo.debounce)
			{
				g_sc_algo.step_valid = true;
				g_sc_algo.report += diff+QMAX981_STEPCOUNTER_INT_NUM;
			}
			g_sc_algo.last = g_sc_algo.curr;
		}
		else
		{
			diff = QMA6981_DIFF(g_sc_algo.curr, g_sc_algo.last);
			g_sc_algo.report += diff;
			g_sc_algo.last = g_sc_algo.curr;
		}
	}
	else
	{
		diff = QMA6981_DIFF(g_sc_algo.curr, g_sc_algo.last);
		g_sc_algo.report += diff;
		g_sc_algo.last = g_sc_algo.curr;
		// stepcounter int stop
	}

	MSE_LOG("thread_read read report=%d\n", g_sc_algo.report);
	count = g_sc_algo.report;
#endif
	//mutex_unlock(&qma->sensor_sc_mutex);

	input_report_abs(qma->stepcount_dev, ABS_RX, count);
	input_sync(qma->stepcount_dev);	

	if(step_c_ref_report == 1)
	{
		input_report_rel(qma->stepcount_dev, REL_RX, step_value);	
		input_sync(qma->stepcount_dev);	
		step_c_ref_report = 0;
	}
	return 0;
}

#else

static int qmaX981_sc_report_data(struct qmaX981_data *qma)
{
	//int res;
	//char databuf[2];
	int resut;
	int	data;
#ifdef QMAX981_STEPCOUNTER_USE_INT
	int tempp = 0;
#endif

	resut = qmaX981_sc_get_data();
	if(resut < 0)
	{
		return -1;
	}
	
#ifdef QMAX981_STEPCOUNTER_USE_INT
	if(qma->chip_type == CHIP_TYPE_QMA6981)
	{
		if (resut < step_count_index.stepcounter_pre)
		{
			step_count_index.back++;
			data = resut- step_count_index.stepcounter_pre + 65536;
			 step_count_index.stepcounter_pre = resut;
		}
		else
		{
			//nothing
			step_count_index.stepcounter_pre = resut;
			data = resut;
		}

		if (step_count_index.stepcounter_statu == STEP_START)
		{
	/*
			if (data >= step_count_index.stepcounter_pre_end)
			{
				tempp = data - step_count_index.stepcounter_pre_end;
			}
			else
			{
				tempp = data - step_count_index.stepcounter_pre_end +65536;
			}
	*/
			
			if (data >= step_count_index.stepcounter_next_start)
			{
				tempp = data - step_count_index.stepcounter_next_start + 4;
			}
			else
			{
				tempp = data - step_count_index.stepcounter_next_start +65540;
			}

			MSE_LOG("ReadStepCounter_running data= %d,stepcounter_next_start = %d,tempp = %d,stepcounter_pre_end =%d \n",data,step_count_index.stepcounter_next_start,tempp,step_count_index.stepcounter_pre_end);
			
			if (tempp < (STEP_INT_START_VLUE+STEP_DUMMY_VLUE))
			{
				data = step_count_index.stepcounter_pre_fix;
				MSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
			}
			else
			{
				if (step_count_index.step_diff >data)
				{
					step_count_index.step_diff = 0;
				}
				else
				{
					data = data -  step_count_index.step_diff;
					step_count_index.stepcounter_pre_fix = data ;
					MSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
				}
			}
		
		}
		else 
		{
			// add by yangzhiqiang for step_diff
			if(step_count_index.step_diff > data)
			{
				step_count_index.step_diff = data;
			}
			// yangzhiqiang for step_diff		
#if 1//defined(QMA6981_STEP_TO_ZERO)
			step_count_index.stepcounter_pre_end = data;
#endif
			data  = data -  step_count_index.step_diff;
			step_count_index.stepcounter_pre_fix = data;
			MSE_LOG("ReadStepCounter_end stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
		}
		MSE_LOG("ReadStepCounter=%d, step_diff= %d\n",data,step_count_index.step_diff );
	}
	else
	{
		data = resut;
	}
#else
	data = resut;
#endif	

	input_report_abs(qma->stepcount_dev, ABS_RX, data);
	input_sync(qma->stepcount_dev); 
	if(step_c_ref_report == 1)
	{
		input_report_rel(qma->stepcount_dev, REL_RX, data);	
		input_sync(qma->stepcount_dev);	
		step_c_ref_report = 0;
	}

	return 0;
}
#endif

static void qmaX981_sc_dev_poll(struct work_struct *work){
	struct qmaX981_data *qma;
	int ret;

	qma = container_of((struct delayed_work *)work, struct qmaX981_data, sc_dwork);

	ret = qmaX981_sc_report_data(qma);
	if (ret < 0)
		MSE_LOG("Failed to report stepcount data");

	schedule_delayed_work(&qma->sc_dwork,(unsigned long)nsecs_to_jiffies64(acc_qst->sc_delay));
}
#endif

static int qmaX981_power_init(struct qmaX981_data *data)
{
	int ret;

	data->vdd = regulator_get(&data->i2c->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->i2c->dev,
		"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				QMA6981_VDD_MIN_UV,
				QMA6981_VDD_MAX_UV);
		if (ret) {
			dev_err(&data->i2c->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	data->vio = regulator_get(&data->i2c->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		dev_err(&data->i2c->dev,
			"Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		ret = regulator_set_voltage(data->vio,
				QMA6981_VIO_MIN_UV,
				QMA6981_VIO_MAX_UV);
		if (ret) {
			dev_err(&data->i2c->dev,
			"Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, QMA6981_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int qmaX981_power_deinit(struct qmaX981_data *data)
{
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd,
				0, QMA6981_VDD_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vio) > 0)
		regulator_set_voltage(data->vio,
				0, QMA6981_VIO_MAX_UV);

	regulator_put(data->vio);

	return 0;
}

#ifdef QMAX981_STEPCOUNTER_USE_INT
#if defined(QMA6981_INT_ALGO_2)
static irqreturn_t qmaX981_irq(int irq, void *handle)
{
	unsigned int diff;
	int read_data;

	read_data = qmaX981_sc_get_data();
	if(read_data < 0)
	{
		return IRQ_HANDLED;
	}

	int_status_flag = !int_status_flag;
	if(int_status_flag)
	{
		//MSE_LOG("qmaX981_irq, step start \n");
		g_sc_algo.step_valid = false;
		g_sc_algo.start = read_data;
		g_sc_algo.last = g_sc_algo.curr = g_sc_algo.start;
		//MSE_LOG("qmaX981_irq start=%d", g_sc_algo.start);

		irq_set_irq_type(acc_qst->irq,IRQF_TRIGGER_LOW);
	}
	else
	{
		//MSE_LOG("qmaX981_irq, step end \n");
		g_sc_algo.curr = read_data;
		//MSE_LOG("qmaX981_irq curr=%d\n", g_sc_algo.curr);
		//MSE_LOG("qmaX981_irq step_valid=%d\n", g_sc_algo.step_valid);
		//MSE_LOG("qmaX981_irq diff=%d\n", QMA6981_DIFF(g_sc_algo.curr, g_sc_algo.start));
		//MSE_LOG("qmaX981_irq report=%d\n", g_sc_algo.report);
		if(g_sc_algo.step_valid == false)
		{
			diff = QMA6981_DIFF(g_sc_algo.curr, g_sc_algo.start);
			if(diff >= g_sc_algo.debounce)
			{
				g_sc_algo.report += diff+QMAX981_STEPCOUNTER_INT_NUM;
			}
		}
		else
		{
			diff = QMA6981_DIFF(g_sc_algo.curr, g_sc_algo.last);
			g_sc_algo.report += diff;
			g_sc_algo.step_valid = false;
		}
		g_sc_algo.last = g_sc_algo.curr;

		irq_set_irq_type(acc_qst->irq,IRQF_TRIGGER_HIGH);
	}
	return IRQ_HANDLED;
}
#else
static irqreturn_t qmaX981_irq(int irq, void *handle)
{
	//int read_data;
	int temp = 0;
	int data = 0;
	int res, i;
	unsigned char databuf[2];

	//read_data = qmaX981_sc_get_data();
	databuf[0] = QMA6981_STEP_CNT;

	for(i=0; i < 3; i++)
	{
		res = I2C_RxData(databuf, 2);
		if (res == 0) 
		{
			break;
		}
	}
	data = databuf[0]|(databuf[1]<<8);
	if(data < 0)
	{
		return IRQ_HANDLED;
	}

	if (int_status_flag)
	{
		int_status_flag = false;//KAL_FALSE;
		step_count_index.stepcounter_next_end = data;
		step_count_index.stepcounter_statu = STEP_END;
		MSE_LOG("step_int stepcounter_next_end = %d stepcounter_next_start = %d\n", step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start);
		if (step_count_index.stepcounter_next_end < step_count_index.stepcounter_next_start)
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start;
		}
		MSE_LOG("step_int_end  temp =%d\n" ,temp);
		if (temp < STEP_DUMMY_VLUE)
		{
			step_count_index.step_diff += (temp+STEP_INT_START_VLUE);
			// add by yangzhiqiang for step_diff
			if(step_count_index.step_diff > data)
			{
				step_count_index.step_diff = data;
			}
			// yangzhiqiang for step_diff
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;

		}
		else
		{
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;
		}
		irq_set_irq_type(acc_qst->irq,IRQF_TRIGGER_HIGH);
		MSE_LOG("step_int_end\n" );
	}
	else
	{
		int_status_flag = true;	//KAL_TRUE;
		step_count_index.stepcounter_next_start= data;
		step_count_index.stepcounter_statu = STEP_START;
		MSE_LOG("step_int stepcounter_next_start = %d stepcounter_pre_end = %d\n", step_count_index.stepcounter_next_start,step_count_index.stepcounter_pre_end);
		if (step_count_index.stepcounter_next_start < step_count_index.stepcounter_pre_end)
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end;
		}
		MSE_LOG("step_int_start temp =%d\n" ,temp);
		if (temp >STEP_INT_START_VLUE)
		{
			step_count_index.step_diff += (temp - STEP_INT_START_VLUE);
		}

		irq_set_irq_type(acc_qst->irq,IRQF_TRIGGER_LOW);
		MSE_LOG("step_int_start\n" );
	}
	enable_irq(acc_qst->irq);

	return IRQ_HANDLED;
}

#endif

static int qmaX981_pinctrl_init(struct qmaX981_data *data)
{
	struct i2c_client *client = data->i2c;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default = pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep = pinctrl_lookup_state(data->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(data->pin_sleep);
	}

	return 0;
}

static int qmaX981_setup_irq(struct i2c_client *client)
{
	int irq, err = -EIO;
	//struct qmaX981_data *data = i2c_get_clientdata(client);

	if(gpio_is_valid(acc_qst->int_pin))
	{
		err = gpio_request(acc_qst->int_pin,"qst-int");
		if(err < 0){
			MSE_ERR("gpio_request, err=%d", err);
			return err;
		}
		err = gpio_direction_input(acc_qst->int_pin);
		if(err < 0){
			MSE_ERR("gpio_direction_input, err=%d", err);
			return err;
		}

		irq = gpio_to_irq(acc_qst->int_pin);
		MSE_LOG("int pin #=%d, irq=%d\n", acc_qst->int_pin, irq);
		if (irq <= 0){
			MSE_ERR("irq number is not specified, irq # = %d, int pin=%d\n",irq, acc_qst->int_pin);
			return irq;
		}
		acc_qst->irq = irq;

		if (acc_qst->irq){
				err = request_threaded_irq(
						acc_qst->irq,
						NULL,
						qmaX981_irq,
						IRQF_TRIGGER_HIGH|IRQF_ONESHOT|IRQF_NO_SUSPEND,	//IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
						dev_name(&client->dev),acc_qst);
				if (err < 0){
					MSE_ERR("%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);
					goto err_request_any_context_irq;
				}
		}

		disable_irq_nosync(acc_qst->irq);
	}

#if defined(QMA6981_INT_ALGO_2)
	memset(&g_sc_algo, 0, sizeof(g_sc_algo));
	g_sc_algo.debounce = QMAX981_STEPCOUNTER_DEBOUNCE;
#else
	memset(&step_count_index, 0, sizeof(step_count_index));
#endif

	return 0;

err_request_any_context_irq:
	gpio_free(acc_qst->int_pin);
	return err;
}
#endif


#ifdef CONFIG_OF
static int qmaX981_parse_dt(struct device *dev,
			struct qmaX981_data *data)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

#if defined(QMAX981_STEPCOUNTER_USE_INT)
	data->int_pin = of_get_named_gpio_flags(np, "qst,irq-gpio",0, &data->int_flags);
	if (data->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return data->int_pin;
	}
#endif
	rc = of_property_read_u32(np, "qst,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read accel_direction\n");
		return rc;
	} else {
		data->layout = (u8) temp_val;
	}
	return 0;
}
#else
static int qmaX981_parse_dt(struct device *dev,
			struct qmaX981_data *data)
{
	data->layout = 5;
	return -ENODEV;
}
#endif /* !CONFIG_OF */

int qmaX981_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	MSE_LOG("%s start.\n",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C|I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA))
	{
		MSE_ERR("%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	acc_qst = kzalloc(sizeof(struct qmaX981_data), GFP_KERNEL);
	if (!acc_qst)
	{
		MSE_ERR("%s: memory allocation failed.", __func__);
		err = -ENOMEM;

		goto exit1;
	}

	/*initialize variables in qmaX981_data */

	mutex_init(&acc_qst->sensor_mutex);
	mutex_init(&acc_qst->val_mutex);
	mutex_init(&acc_qst->op_mutex);
#if defined(QMAX981_STEPCOUNTER)
	mutex_init(&acc_qst->sensor_sc_mutex);
	mutex_init(&acc_qst->val_sc_mutex);
	mutex_init(&acc_qst->op_sc_mutex);
	acc_qst->sc_enable_flag = 0;
	acc_qst->sc_delay = 200*1000*1000;
#endif
	acc_qst->enable_flag = 0;
	acc_qst->delay = QMA6981_ACCEL_DEFAULT_POLL_INTERVAL_MS * 1000*1000;
	acc_qst->sc_delay = QMA6981_ACCEL_DEFAULT_POLL_INTERVAL_MS * 1000*1000;

	err = qmaX981_parse_dt(&client->dev, acc_qst);
	dev_err(&client->dev,"%s: qmaX981_parse_dt ret=%d\n", __func__, err);
	if (err)
	{
		goto exit2;
	}
	/* i2c initialization */
	acc_qst->i2c = client;
	this_client = acc_qst->i2c;
	i2c_set_clientdata(client, acc_qst);

	/*set struct hwmsen_convert map[]*/
	if (acc_qst->layout >= sizeof(map)/sizeof(map[0]))
     return -EINVAL;

	acc_qst->cvt = map[acc_qst->layout];

	err = qmaX981_power_init(acc_qst);
	if (err){
		dev_err(&client->dev, "Failed to get sensor regulators\n");
		err = -EINVAL;
		goto exit4;
	}
	err = qmaX981_power_set(acc_qst,true);
	if (err){
		dev_err(&client->dev, "Failed to enable sensor power\n");
		err = -EINVAL;
		goto exit4;
	}
	msleep(1);	
	
	err = qmax981_read_chip_id();
	if(err < 0)
		goto exit4;
 
#if 0 //defined(QMAX981_STEPCOUNTER)
     /*clear stepcounter vaule*/
     err = qmaX981_setreg_reset_sc();
     if (err < 0)
         goto exit4;
#endif

	err=qmaX981_initialize(client);
	if(err < 0)
		goto exit4;

	/*register input system*/
	err = qmaX981_input_init(client,acc_qst);
	if (err){
		MSE_ERR("%s: input_dev register failed", __func__);
		goto exit4;
	}
	input_set_drvdata(acc_qst->accel_dev, acc_qst);

	/* IRQ setup */
#ifdef QMAX981_STEPCOUNTER_USE_INT
	if (!qmaX981_pinctrl_init(acc_qst)) {
		err = pinctrl_select_state(acc_qst->pinctrl, acc_qst->pin_default);
		if (err) {
			dev_err(&client->dev, "Can't select pinctrl state\n");
			goto exit5;
		}
	}

	err=qmaX981_setup_irq(this_client);
	if (err < 0){
		MSE_ERR("request irq failed.");
		goto exit5;
	}
#endif

	INIT_DELAYED_WORK(&acc_qst->dwork, qmaX981_dev_poll);
#if defined(QMAX981_STEPCOUNTER)
	INIT_DELAYED_WORK(&acc_qst->sc_dwork, qmaX981_sc_dev_poll);
#endif

	/*register misc device*/
	err = misc_register(&qmaX981_dev);
	if (err) {
		MSE_ERR("qmaX981_dev register failed");
		goto exit6;
	}

	/*creat sysfs under misc devide*/
	err = sysfs_create_group(&qmaX981_dev.this_device->kobj,&acc_attribute_group);
	//err = sysfs_create_group(&acc_qst->accel_dev->dev.kobj,&acc_attribute_group);
	if (0 > err)
	{
		MSE_ERR("%s: sysfs_create_group fail\n",__func__);
		goto exit7;
	}
	kobject_uevent(&qmaX981_dev.this_device->kobj,KOBJ_ADD);

	acc_qst->accel_cdev = qmaX981_acc_cdev;
	acc_qst->accel_cdev.delay_msec = QMA6981_ACCEL_DEFAULT_POLL_INTERVAL_MS;
	acc_qst->accel_cdev.sensors_enable = qmaX981_enable_set;
	acc_qst->accel_cdev.sensors_poll_delay = qmaX981_poll_delay_set;

	acc_qst->accel_cdev.sensors_calibrate = qmaX981_calibrate;
	acc_qst->accel_cdev.sensors_write_cal_params = qmaX981_write_cal_params;
	memset(&acc_qst->accel_cdev.cal_result, 0, sizeof(acc_qst->accel_cdev.cal_result));
	acc_qst->accel_cdev.params = acc_qst->calibrate_buf;

	err = sensors_classdev_register(&client->dev, &acc_qst->accel_cdev);
	if (err) {
		MSE_ERR("class device create failed: %d\n",err);
		goto exit8;
	}
	
#if defined(QMAX981_STEPCOUNTER)
	acc_qst->stepcount_cdev = qmaX981_stepcount_cdev;
	acc_qst->stepcount_cdev.delay_msec = QMAX981_STEPCOUNTER_DEFAULT_POLL_INTERVAL_MS;
	acc_qst->stepcount_cdev.sensors_enable = qmaX981_sc_enable_set;
	acc_qst->stepcount_cdev.sensors_poll_delay = qmaX981_sc_poll_delay_set;

	err = sensors_classdev_register(&acc_qst->stepcount_dev->dev,&acc_qst->stepcount_cdev);
	if (err) {
		MSE_ERR(
			"create stepcount class device file failed!\n");
		err = -EINVAL;
		goto exit9;
	}
#endif
	qmaX981_power_set(acc_qst,false);
	MSE_LOG("%s successfully.\n",__func__);
	return 0;
	
#if defined(QMAX981_STEPCOUNTER)
exit9:
	sensors_classdev_unregister(&acc_qst->accel_cdev);
#endif
exit8:
	sysfs_remove_group(&qmaX981_dev.this_device->kobj,&acc_attribute_group);
exit7:
	misc_deregister(&qmaX981_dev);
exit6:
#ifdef QMAX981_STEPCOUNTER_USE_INT
	if (acc_qst->irq)
		free_irq(acc_qst->irq, acc_qst);
exit5:
#endif
	input_unregister_device(acc_qst->accel_dev);
exit4:
	qmaX981_power_set(acc_qst, false);
	qmaX981_power_deinit(acc_qst);
	i2c_set_clientdata(client, NULL);
exit2:
	if(acc_qst)
	{
		kfree(acc_qst);
		acc_qst = NULL;
	}
exit1:
exit0:
	return err;
}

static int qmaX981_remove(struct i2c_client *client)
{
	struct qmaX981_data *qma = i2c_get_clientdata(client);

#if defined(QMAX981_STEPCOUNTER)
	sensors_classdev_unregister(&qma->stepcount_cdev);
#endif
	sensors_classdev_unregister(&qma->accel_cdev);

	qmaX981_input_destory(client, acc_qst);
	if (qmaX981_power_set(qma, 0))
		MSE_ERR("power set failed.");

	//qmaX981_power_set(acc_qst,false);
	qmaX981_power_deinit(qma);

	sysfs_remove_group(&qmaX981_dev.this_device->kobj,&acc_attribute_group);

	if (misc_deregister(&qmaX981_dev) < 0)
		MSE_ERR("misc deregister failed.");

	if (qma->irq)
		free_irq(qma->irq, qma);

	kfree(qma);
	MSE_LOG("successfully removed.");
	return 0;
}

static const struct i2c_device_id qmaX981_id[] = {
	{QMA_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qmaX981_id);

static const struct dev_pm_ops qmaX981_pm_ops = {
	.suspend	= qmaX981_suspend,
	.resume		= qmaX981_resume,
};

static struct of_device_id qmaX981_match_table[] = {
	{ .compatible = "qst,qmaX981", },
	{ },
};

static struct i2c_driver qmaX981_driver = {
	.probe		= qmaX981_probe,
	.remove		= qmaX981_remove,
	.id_table	= qmaX981_id,
	.driver = {
		.name	= QMA_I2C_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = qmaX981_match_table,
		.pm		= &qmaX981_pm_ops,
	},
};

static int __init qmaX981_init(void)
{
	MSE_LOG("QST accelerometer driver: initialize.");
	return i2c_add_driver(&qmaX981_driver);
}

static void __exit qmaX981_exit(void)
{
	i2c_del_driver(&qmaX981_driver);
}


module_init(qmaX981_init);
module_exit(qmaX981_exit);

MODULE_AUTHOR("zhiqiang_yang@qstcorp.com");
MODULE_DESCRIPTION("QST accelerometer driver");
MODULE_LICENSE("GPL");


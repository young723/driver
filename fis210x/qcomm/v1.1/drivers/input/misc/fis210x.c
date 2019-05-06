
#include<linux/module.h>
#include<linux/err.h>
#include<linux/errno.h>
#include<linux/delay.h>
#include<linux/fs.h>
#include<linux/i2c.h>

#include<linux/input.h>
#include<linux/input-polldev.h>
#include<linux/miscdevice.h>
#include<linux/uaccess.h>
#include<linux/slab.h>

#include<linux/workqueue.h>
#include<linux/irq.h>
#include<linux/gpio.h>
#include<linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif
#include<linux/of_device.h>
#include<linux/of_address.h>
#include<linux/of_gpio.h>

#include<linux/wakelock.h>
#include<linux/mutex.h>
#include <linux/sensors.h>
#include "fis210x.h"


struct fis210x_data
{
	atomic_t 					acc_enable;
	atomic_t 					acc_delay;	
	atomic_t 					gyro_enable;
	atomic_t 					gyro_delay;

	char						calibrate_buf[FIS210X_CAL_NUM];
	int							cal_params[3];
	bool						use_cal;
	atomic_t					cal_status;

	struct input_dev 			*acc_input;
	struct input_dev 			*gyro_input;
	struct sensors_classdev 	accel_cdev;
	struct sensors_classdev 	gyro_cdev;
	struct delayed_work 		acc_work;
	struct delayed_work 		gyro_work;
	
	atomic_t 					position;
	struct mutex 				op_mutex;
	struct i2c_client 			*client;
	unsigned char				chip_id;

	short 						acc_lsb;
	short						acc_scale;
	enum FisImu_AccRange		acc_range;
	enum FisImu_AccOdr			acc_odr;
	enum FisImu_AccUnit			acc_uint;
	
	short 						gyro_lsb;
	short						gyro_scale;
	enum FisImu_GyrRange		gyro_range;	
	enum FisImu_GyrOdr			gyro_odr;
	enum FisImu_GyrUnit			gyro_uint;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_drv;
#endif 
};

#if defined(ACC_USE_CALI)
#define ACC_CALI_FILE		"/productinfo/acc_cali.conf"
#define ACC_LSB_1G			1000			// mg
#define ACC_CALI_NUM		20    
static int acc_cali[3]={0, 0, 0};
static char acc_cali_flag = 0;
static void acc_read_file(char * filename, char *data, int len);
static void acc_write_file(char * filename, char *data, int len);
#endif

static struct fis210x_convert g_map;
static struct fis210x_data *g_fis210x=NULL;

static struct sensors_classdev fis210x_acc_cdev = {
	.name = "accelerometer",
	.vendor = "QST Corporation",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,	
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "1228.8",
	.resolution = "0.6",
	.sensor_power = "0.35",
	.min_delay = 10 * 1000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev fis210x_gyro_cdev = {
	.name = "gyroscope",
	.vendor = "QST Corporation",
	.version = 1,	
	.handle = SENSORS_GYROSCOPE_HANDLE,	
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "65536",	
	.resolution = "1",	
	.sensor_power = "0.35",	/* 0.5 mA */
	.min_delay = 10 * 1000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

void fis210x_initialize(struct i2c_client *client);

static inline int fis210x_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{	
	signed int dummy = 0;
	mutex_lock(&g_fis210x->op_mutex);
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&g_fis210x->op_mutex);
	if (dummy < 0)
		return dummy;
	*data = dummy & 0x000000ff;
	return 0;
}

static inline int fis210x_smbus_write_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
	signed int dummy = 0;
    mutex_lock(&g_fis210x->op_mutex);
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	mutex_unlock(&g_fis210x->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static inline int fis210x_smbus_read_block(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	signed int dummy = 0;

	mutex_lock(&g_fis210x->op_mutex);
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&g_fis210x->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static inline int fis210x_smbus_write_block(struct i2c_client *client,unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	signed int dummy = 0;

	mutex_lock(&g_fis210x->op_mutex);
	dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&g_fis210x->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static int fis210x_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
	
	mutex_lock(&g_fis210x->op_mutex);
	ret = i2c_smbus_write_i2c_block_data(g_fis210x->client, reg, 1, &value);
	mutex_unlock(&g_fis210x->op_mutex);
	if(ret < 0)
	{
		return ret;
	}
	else
	{
		return 0;
	}
}

static int fis210x_read_reg(unsigned char reg, unsigned char *buf, unsigned char len)
{
	int ret = 0;
	
	mutex_lock(&g_fis210x->op_mutex);
	if(len == 1)
		*buf = i2c_smbus_read_byte_data(g_fis210x->client, reg);
	else
		ret = i2c_smbus_read_i2c_block_data(g_fis210x->client, reg, len, buf);
	
	mutex_unlock(&g_fis210x->op_mutex);
	if(ret < 0)
	{
		return ret;
	}
	else
	{
		return 0;
	}
}


static int fis210x_read_chip_id(struct i2c_client *client)
{	
	int res = 0;
	unsigned char chip_id;

	FIS210X_FUN();
	res = fis210x_smbus_read_byte(client, FisRegister_WhoAmI, &chip_id);
	if(res)
	{
		FIS210X_ERR("fis210x_read_chip_id error!");
	}
	else
	{
		FIS210X_ERR("fis210x_read_chip_id OK chip_id=0x%x ", chip_id);
		g_fis210x->chip_id = chip_id;
	}

	return 0;
}

static int fis210x_set_acc_range_odr(enum FisImu_AccRange range,enum FisImu_AccOdr odr)
{
	u8 acc_setting = 0;
	int err=0;

	switch(range)
	{
		case AccRange_2g:
			g_fis210x->acc_lsb= (1 << 14);
			break;
		case AccRange_4g:
			g_fis210x->acc_lsb = (1 << 13);
			break;
		case AccRange_8g:
			g_fis210x->acc_lsb = (1 << 12);
			break;
		case AccRange_16g:
			g_fis210x->acc_lsb = (1 << 11);
			break;
		default:			
			g_fis210x->acc_lsb = (1 << 14);
			break;
	}

	acc_setting = (u8)range | (u8)odr;
	err = fis210x_write_reg(FisRegister_Ctrl2, acc_setting);

	return err;
}

static int fis210x_set_gyro_range_odr(enum FisImu_GyrRange range,enum FisImu_GyrOdr odr)
{
	u8 gyro_setting = 0;
	int err=0;

	switch(range)
	{
		case GyrRange_32dps:
			g_fis210x->gyro_lsb = (1 << 10);
			break;
		case GyrRange_64dps:
			g_fis210x->gyro_lsb = (1 << 9);
			break;
		case GyrRange_128dps:
			g_fis210x->gyro_lsb = (1 << 8);
			break;
		case GyrRange_256dps:
			g_fis210x->gyro_lsb = (1 << 7);
			break;
		case GyrRange_512dps:
			g_fis210x->gyro_lsb = (1 << 6);
			break;
		case GyrRange_1024dps:
			g_fis210x->gyro_lsb = (1 << 5);
			break;
		case GyrRange_2048dps:
			g_fis210x->gyro_lsb = (1 << 4);
			break;
		case GyrRange_2560dps:
			g_fis210x->gyro_lsb = (1 << 3);
			break;
		default:
			g_fis210x->gyro_lsb = (1 << 5);			
	}

	gyro_setting = (u8)range | (u8)odr;
	err = fis210x_write_reg(FisRegister_Ctrl3, gyro_setting);

	return err;
}


void fis210x_enable_sensors(u8 enableFlags)
{
	if(enableFlags & FISIMU_CTRL7_AE_ENABLE)
	{
		enableFlags |= FISIMU_CTRL7_ACC_ENABLE | FISIMU_CTRL7_GYR_ENABLE;
	}

	enableFlags = enableFlags & FISIMU_CTRL7_ENABLE_MASK;
	fis210x_write_reg(FisRegister_Ctrl7, enableFlags);
}

static int fis210x_set_mode(enum FisImu_mode mode)
{
	u8 reg_value = 0;

	if(mode == FIS_MODE_LOW_POWER)
	{
		fis210x_set_acc_range_odr(AccRange_4g, AccOdr_128Hz);
		reg_value = 0;
		fis210x_write_reg(FisRegister_Ctrl1, reg_value);
		fis210x_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else if(mode == FIS_MODE_POWER_DOWN)
	{
		fis210x_set_acc_range_odr(AccRange_4g, AccOdr_128Hz);
		reg_value = 1;
		fis210x_write_reg(FisRegister_Ctrl1, reg_value);
		fis210x_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else
	{
		fis210x_set_acc_range_odr(g_fis210x->acc_range, g_fis210x->acc_odr);
		fis210x_set_gyro_range_odr(g_fis210x->gyro_range, g_fis210x->gyro_odr);
		reg_value = 0;
		fis210x_write_reg(FisRegister_Ctrl1, reg_value);
		fis210x_enable_sensors(FISIMU_CTRL7_ACC_ENABLE|FISIMU_CTRL7_GYR_ENABLE);
	}

	return 0;
}


int fis210x_setEnableBits(u8 address, u8 bitmask, bool enable)
{
	u8 data[2];
	int err = 0;

	// Read the current configuration into data buffer
	fis210x_smbus_read_byte(g_fis210x->client, address, &data[1]);

	// Update the required enable bits according to bitmask
	if(enable)
	{
		data[1] |= bitmask;
	}
	else
	{
		data[1] &= ~bitmask;
	}

	// Prepare register address
	data[0] = address;
	// Write the updated sensor enable register value
	err = fis210x_write_reg(data[0], data[1]);

	return err;
}


int fis210x_set_filter(enum FisImu_LpfConfig lpfEnable,enum FisImu_HpfConfig hpfEnable)
{
	int err = 0;
	// Configure accelerometer Low Pass Filter enable bit
	err = fis210x_setEnableBits(FisRegister_Ctrl5, FISIMU_CTRL5_ACC_LPF_ENABLE,lpfEnable == Lpf_Enable);
	// Configure accelerometer High Pass Filter enable bit
	err += fis210x_setEnableBits(FisRegister_Ctrl5, FISIMU_CTRL5_ACC_HPF_ENABLE,hpfEnable == Hpf_Enable);

	return err;
}


void fis210x_set_layout(int layout)
{
	if(layout == 0)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 1)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 2)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 3)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}	
	else if(layout == 4)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 5)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 6)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 7)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}
	else		
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
}


static int fis210x_get_delay(void)
{
	return atomic_read(&g_fis210x->acc_delay);
}

static void fis210x_set_delay(enum fis210x_type type, int delay)
{
	int delay_last;
	int is_enable;

	FIS210X_FUN();
	if(type == FIS210X_TYPE_ACC)
	{
		delay_last = atomic_read(&g_fis210x->acc_delay);
		is_enable = atomic_read(&g_fis210x->acc_enable);
		atomic_set(&g_fis210x->acc_delay, delay);
		if((delay_last != delay)&&(is_enable))
		{
			cancel_delayed_work_sync(&g_fis210x->acc_work);
			schedule_delayed_work(&g_fis210x->acc_work, msecs_to_jiffies(delay)+1);
		}
	}
	else if(type == FIS210X_TYPE_GYRO)
	{
		delay_last = atomic_read(&g_fis210x->gyro_delay);
		is_enable = atomic_read(&g_fis210x->gyro_enable);
		atomic_set(&g_fis210x->gyro_delay, delay);
		if((delay_last != delay)&&(is_enable))
		{
			cancel_delayed_work_sync(&g_fis210x->gyro_work);
			schedule_delayed_work(&g_fis210x->gyro_work, msecs_to_jiffies(delay)+1);
		}
	}
}

static int fis210x_acc_read_raw(int raw_xyz[3])
{
	unsigned char buf_reg[6];
	int ret = 0;

#if 1
	fis210x_read_reg(FisRegister_Ax_L, &buf_reg[0], 1);		// 0x19, 25
	fis210x_read_reg(FisRegister_Ax_H, &buf_reg[1], 1);
	fis210x_read_reg(FisRegister_Ay_L, &buf_reg[2], 1);
	fis210x_read_reg(FisRegister_Ay_H, &buf_reg[3], 1);
	fis210x_read_reg(FisRegister_Az_L, &buf_reg[4], 1);
	fis210x_read_reg(FisRegister_Az_H, &buf_reg[5], 1);
#else
	ret = fis210x_read_reg(FisRegister_Ax_L, buf_reg, 6); 	// 0x19, 25
#endif

	raw_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));
	FIS210X_LOG("fis210x_acc_read_raw	%d	%d	%d\n", raw_xyz[0],raw_xyz[1],raw_xyz[2]);

	return ret;
}


static int fis210x_read_accel_xyz(struct fis210x_acc *acc)
{
	int res = 0;

	int raw_xyz[3];
	int acc_xyz[3];

	res = fis210x_acc_read_raw(raw_xyz);

	//remap coordinate
	acc_xyz[g_map.map[0]] = g_map.sign[0]*raw_xyz[0];
	acc_xyz[g_map.map[1]] = g_map.sign[1]*raw_xyz[1];
	acc_xyz[g_map.map[2]] = g_map.sign[2]*raw_xyz[2];
	
	acc->x = (acc_xyz[0]*g_fis210x->acc_scale)/(g_fis210x->acc_lsb);
	acc->y = (acc_xyz[1]*g_fis210x->acc_scale)/(g_fis210x->acc_lsb);
	acc->z = (acc_xyz[2]*g_fis210x->acc_scale)/(g_fis210x->acc_lsb);

	return res;
}


static int fis210x_gyro_read_raw(int raw_xyz[3])
{
	unsigned char buf_reg[6];
	int ret = 0;
#if 1
	fis210x_read_reg(FisRegister_Gx_L, &buf_reg[0], 1);		// 0x19, 25
	fis210x_read_reg(FisRegister_Gx_H, &buf_reg[1], 1);
	fis210x_read_reg(FisRegister_Gy_L, &buf_reg[2], 1);
	fis210x_read_reg(FisRegister_Gy_H, &buf_reg[3], 1);
	fis210x_read_reg(FisRegister_Gz_L, &buf_reg[4], 1);
	fis210x_read_reg(FisRegister_Gz_H, &buf_reg[5], 1);
#else
	ret = fis210x_read_reg(FisRegister_Gx_L, buf_reg, 6);
#endif
	raw_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	return ret;
}

static int fis210x_read_gyro_xyz(struct fis210x_gyro *gyro)
{
	int res = 0;
	int raw_xyz[3];
	int gyro_xyz[3];

	res = fis210x_gyro_read_raw(raw_xyz);

	gyro_xyz[g_map.map[0]] = g_map.sign[0]*raw_xyz[0];
	gyro_xyz[g_map.map[1]] = g_map.sign[1]*raw_xyz[1];
	gyro_xyz[g_map.map[2]] = g_map.sign[2]*raw_xyz[2];
	
	gyro->x = (gyro_xyz[0]*g_fis210x->gyro_scale)/(g_fis210x->gyro_lsb);
	gyro->y = (gyro_xyz[1]*g_fis210x->gyro_scale)/(g_fis210x->gyro_lsb);
	gyro->z = (gyro_xyz[2]*g_fis210x->gyro_scale)/(g_fis210x->gyro_lsb);

	return res;
}


static void acc_work_func(struct work_struct *work)
{
	struct fis210x_acc acc = { 0 };
	int comres = -1;

	FIS210X_FUN();
	comres = fis210x_read_accel_xyz(&acc);
	if(comres)
	{
		comres = fis210x_read_accel_xyz(&acc);
		if(comres)
		{		
			schedule_delayed_work(&g_fis210x->acc_work, msecs_to_jiffies(atomic_read(&g_fis210x->acc_delay)));
			return;
		}
	}
	
	input_report_abs(g_fis210x->acc_input, ABS_X, acc.x);
	input_report_abs(g_fis210x->acc_input, ABS_Y, acc.y);
	input_report_abs(g_fis210x->acc_input, ABS_Z, acc.z);
	input_report_abs(g_fis210x->acc_input, ABS_THROTTLE, 3);
	input_sync(g_fis210x->acc_input);
	FIS210X_LOG("%s: [%d %d %d ]\n", __func__,acc.x,acc.y,acc.z);
	schedule_delayed_work(&g_fis210x->acc_work, msecs_to_jiffies(atomic_read(&g_fis210x->acc_delay)));
}


static void gyro_work_func(struct work_struct *work)
{
	struct fis210x_gyro gyro = { 0 };
	int comres = -1;

	FIS210X_FUN();
	comres = fis210x_read_gyro_xyz(&gyro);
	if(comres)
	{
		comres = fis210x_read_gyro_xyz(&gyro);
		if(comres)
		{		
			schedule_delayed_work(&g_fis210x->gyro_work, msecs_to_jiffies(atomic_read(&g_fis210x->gyro_delay)));
			return;
		}
	}
	
	input_report_abs(g_fis210x->gyro_input, ABS_X, gyro.x);
	input_report_abs(g_fis210x->gyro_input, ABS_Y, gyro.y);
	input_report_abs(g_fis210x->gyro_input, ABS_Z, gyro.z);
	input_report_abs(g_fis210x->gyro_input, ABS_THROTTLE, 3);
	input_sync(g_fis210x->gyro_input);
	FIS210X_LOG("%s: [%d %d %d ]\n", __func__,gyro.x,gyro.y,gyro.z);
	schedule_delayed_work(&g_fis210x->gyro_work, msecs_to_jiffies(atomic_read(&g_fis210x->gyro_delay)));
}


static int fis210x_input_init(struct fis210x_data *fis210x)
{
	struct input_dev *dev = NULL;
	int err = 0;

	FIS210X_LOG("acc input init\n");
	dev = input_allocate_device();
	if(!dev) {
		FIS210X_ERR("acc input can't allocate device!\n");
		return -ENOMEM;
	}

	dev->name = FIS210X_ACC_INPUT_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_capability(dev, EV_ABS, ABS_THROTTLE);
	input_set_abs_params(dev, ABS_X, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_THROTTLE, 0, 3, 0, 0);
	input_set_drvdata(dev, (void*)fis210x);
	err = input_register_device(dev);
	if(err < 0) {
		FIS210X_ERR("acc input can't register device!\n");
		input_free_device(dev);
		return err;
	}
	fis210x->acc_input = dev;


	FIS210X_LOG("gyro input init\n");
	dev = input_allocate_device();
	if(!dev) {
		FIS210X_ERR("gyro input can't allocate device!\n");
		return -ENOMEM;
	}
	dev->name = FIS210X_GYRO_INPUT_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_capability(dev, EV_ABS, ABS_THROTTLE);
	input_set_abs_params(dev, ABS_X, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_THROTTLE, 0, 3, 0, 0);
	input_set_drvdata(dev, (void*)fis210x);
	err = input_register_device(dev);
	if(err < 0) {
		FIS210X_ERR("gyro input can't register device!\n");
		input_free_device(dev);
		return err;
	}
	fis210x->gyro_input = dev;

	return 0;
}


static void fis210x_input_deinit(struct fis210x_data *fis210x)
{
	FIS210X_LOG("%s called\n", __func__);
	if(fis210x->acc_input)
	{
		input_unregister_device(fis210x->acc_input);
		input_free_device(fis210x->acc_input);
		fis210x->acc_input = NULL;
	}
	
	if(fis210x->gyro_input)
	{
		input_unregister_device(fis210x->gyro_input);
		input_free_device(fis210x->gyro_input);
		fis210x->gyro_input = NULL;
	}
}


static int fis210x_get_enable(enum fis210x_type type)
{
	FIS210X_FUN();

	if(type == FIS210X_TYPE_ACC)
		return atomic_read(&g_fis210x->acc_enable);
	else if(type == FIS210X_TYPE_GYRO)
		return atomic_read(&g_fis210x->gyro_enable);
	else
		return 0;
}

static int fis210x_set_enable(enum fis210x_type type, int enable)
{
	int acc_enable, gyro_enable;

	FIS210X_LOG("%s: type:%d--enable :%d\n",__func__,type, enable);
	if(type == FIS210X_TYPE_ACC)
	{
		atomic_set(&g_fis210x->acc_enable, enable);
		if(enable) 
		{
			schedule_delayed_work(&g_fis210x->acc_work,msecs_to_jiffies(atomic_read(&g_fis210x->acc_delay))+1);
		}	
		else
		{
			cancel_delayed_work_sync(&g_fis210x->acc_work);		
		}
	}
	else if(type == FIS210X_TYPE_GYRO)
	{
		atomic_set(&g_fis210x->gyro_enable, enable);
		if(enable) 
		{
			schedule_delayed_work(&g_fis210x->gyro_work,msecs_to_jiffies(atomic_read(&g_fis210x->gyro_delay))+1);
		}
		else
		{
			cancel_delayed_work_sync(&g_fis210x->gyro_work);		
		}
	}
	acc_enable = atomic_read(&g_fis210x->acc_enable);
	gyro_enable = atomic_read(&g_fis210x->gyro_enable);

	if(acc_enable||gyro_enable)
		fis210x_set_mode(FIS_MODE_NOMAL);
	else
		fis210x_set_mode(FIS_MODE_LOW_POWER);

	return 0;
}

static ssize_t show_init_acc_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;

	err = fis210x_read_chip_id(g_fis210x->client);
	if(err < 0)
	{
		FIS210X_ERR("%s: g_fis210x read id fail!\n", __func__);
	}
	fis210x_initialize(g_fis210x->client);

	return sprintf(buf, "init done!\n");
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "FIS210X\n");
}


static ssize_t show_sensordata_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fis210x_acc acc;
	
	fis210x_read_accel_xyz(&acc);

	return sprintf(buf, "%d %d %d\n",acc.x,acc.y,acc.z);
}
		
static ssize_t show_dumpallreg_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
	char strbuf[600];
	char tempstrbuf[24];
	unsigned char databuf[2]={0};
	int length=0;

	FIS210X_FUN();
	/* Check status register for data availability */
	for(i =0;i<=50;i++)
	{
		databuf[0] = i;
		res = fis210x_smbus_read_byte(g_fis210x->client, databuf[0], &databuf[1]);
		if(res < 0)
			FIS210X_LOG("qma6981 dump registers 0x%02x failed !\n", i);

		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "0x%2x=0x%2x\n",i, databuf[1]);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "%s",tempstrbuf);
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	FIS210X_FUN();

	return sprintf(buf, "%d\n", atomic_read(&g_fis210x->position));
}

static ssize_t store_layout_value(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int position = 0;

	FIS210X_FUN();

	if(1 == sscanf(buf, "%d", &position))
	{
		if((position >= 0) && (position <= 7))
		{
			atomic_set(&g_fis210x->position, position);
			fis210x_set_layout(position);
		}
	}
	else
	{
		FIS210X_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}


static ssize_t fis210x_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	FIS210X_FUN();

	return sprintf(buf, "acc:%d gyro:%d\n", atomic_read(&g_fis210x->acc_enable),atomic_read(&g_fis210x->gyro_enable));
}

static ssize_t fis210x_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int type, enable=0;

	FIS210X_FUN();
	
	if(2 == sscanf(buf, "%d %d", &type, &enable))
	{
		if((type==FIS210X_TYPE_ACC)||(type==FIS210X_TYPE_GYRO))
		{
			if(enable)
			{
				fis210x_set_enable(type, 1);
			}
			else
			{
				fis210x_set_enable(type, 0);
			}
		}
	}
	else
	{	
		FIS210X_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t fis210x_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	FIS210X_FUN();
	return sprintf(buf, "acc:%d gyro:%d\n", atomic_read(&g_fis210x->acc_delay),atomic_read(&g_fis210x->gyro_delay));
}

static ssize_t fis210x_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int type, delay;

	FIS210X_FUN();
	if(2 == sscanf(buf, "%d %d", &type, &delay))
	{
		if(delay > FIS210X_MAX_DELAY)
			delay = FIS210X_MAX_DELAY;
		else if(delay <= 1)
			delay = 1;

		if((type==FIS210X_TYPE_ACC)||(type==FIS210X_TYPE_GYRO))
		{
			fis210x_set_delay(type, delay);
		}
	}
	else
	{
		FIS210X_ERR("invalid format = '%s'\n", buf);
	}
	
	return count;
}

static unsigned char fis210x_debug_reg_addr=0x00;
//static unsigned char fis210x_debug_reg_value=0x00;
static unsigned char fis210x_debug_read_len=0x01;

static ssize_t fis210x_getreg(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res = 0;	
	unsigned char data=0xff;	

	FIS210X_FUN();
	res = fis210x_smbus_read_byte(g_fis210x->client, fis210x_debug_reg_addr, &data);

	return sprintf(buf, "0x%x\n",  data);
}

static ssize_t fis210x_setreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int addr, value;
	unsigned char data;	
	int res = 0;	
	
	FIS210X_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);	
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))	
	{		
		FIS210X_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);		
		fis210x_debug_reg_addr = (unsigned char)addr;		
		fis210x_debug_read_len = 1;		
		data = (unsigned char)value;		
		res = fis210x_smbus_write_byte(g_fis210x->client, fis210x_debug_reg_addr, &data);		
		if(res)		
		{			
			FIS210X_ERR("write reg 0x%02x fail\n", addr);		
		}	
	}	
	else
	{		
		FIS210X_ERR("store_reg get para error\n");	
	}
	
	return count;
}


#if defined(ACC_USE_CALI)
static void acc_write_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDWR|O_CREAT, 0666);
	if (IS_ERR(fp))
	{
		printk("acc_write_file open file error\n");
	}
	else
	{
		//printk("acc_write_file data=0x%x len=%d\n", data, len);
		//snprintf();
		fp->f_op->write(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
	}

	set_fs(fs);
}

static void acc_read_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	if(acc_cali_flag == 1)
	{
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fp))
	{
		acc_cali_flag = 1;
		printk("fis210x_read_file open file error\n");
	}
	else
	{
		//printk("fis210x_read_file data=0x%x len=%d\n", data, len);
		fp->f_op->read(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
		acc_cali_flag = 1;
	}

	set_fs(fs);
}

static ssize_t acc_cali_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", acc_cali[0], acc_cali[1], acc_cali[2]);
}

static ssize_t acc_cali_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int en = 0;
	int data[3], data_avg[3];
	int icount, z_max, z_min;	
	struct fis210x_acc acc;
	//struct fis210x_data *g_fis210x = i2c_get_clientdata(this_client);
#if 0
	if (strict_strtoul(buf, 10, &en))
		return -EINVAL;
#endif
	if(1 == sscanf(buf, "%d", &en))
	{
		
	}
	else
	{
		FIS210X_ERR("invalid format = '%s'\n", buf);
		return count;
	}
	en = en ? 1 : 0;

	if(en)
	{	
		data_avg[0] = 0;
		data_avg[1] = 0;
		data_avg[2] = 0;
		for(icount=0; icount<QMA6981_CALI_NUM; icount++)
		{
			//fis210x_read_raw_xyz(acc_qst, data);			
			fis210x_read_accel_xyz(&acc);
			data_avg[0] += acc.x;	//data[0];
			data_avg[1] += acc.y;	//data[1];
			data_avg[2] += acc.z;	//data[2];
			// add by yangzhiqiang check vibrate
			if(icount == 0)
			{
				z_max = acc.z;
				z_min = acc.z;
			}
			else
			{
				z_max = (acc.z>z_max)?acc.z:z_max;
				z_min = (acc.z<z_min)?acc.z:z_min;
			}
			// add by yangzhiqiang check vibrate
			mdelay(5);
		}
		// add by yangzhiqiang check vibrate
		if((z_max-z_min)>(QMA6981_LSB_1G*3/10))
		{
			printk("acc_cali_store check vibrate cali ingore!\n");
			return count;
		}
		// add by yangzhiqiang check vibrate

		data_avg[0] = data_avg[0]/QMA6981_CALI_NUM;
		data_avg[1] = data_avg[1]/QMA6981_CALI_NUM;
		data_avg[2] = data_avg[2]/QMA6981_CALI_NUM;
		printk("acc_cali_store data_avg[%d %d %d]\n", data_avg[0], data_avg[1], data_avg[2]);
		// add by yangzhiqiang check offset range
#if 0
		if(QMA6981_ABS(data_avg[2]-QMA6981_LSB_1G)>(QMA6981_LSB_1G*5/10))
		{
			printk("acc_cali_store check offset range cali ingore!\n");
			return count;
		}
#endif
		// add by yangzhiqiang check offset range
		data[0] = 0-data_avg[0];
		data[1] = 0-data_avg[1];
		data[2] = QMA6981_LSB_1G-data_avg[2];
		acc_cali[0] += data[0];
		acc_cali[1] += data[1];
		acc_cali[2] += data[2];
		printk("acc_cali_store offset[%d %d %d]\n", data[0], data[1], data[2]);
		printk("acc_cali_store acc_cali[%d %d %d]\n", acc_cali[0], acc_cali[1], acc_cali[2]);
		acc_write_file(QMA6981_CALI_FILE, (char *)acc_cali, sizeof(acc_cali));
		
	}
	else
	{
	}
	
	return count;
}
#endif


static DEVICE_ATTR(init_acc,		FIS210X_ATTR_R, show_init_acc_value, NULL);
static DEVICE_ATTR(chipinfo,		FIS210X_ATTR_R, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata,	FIS210X_ATTR_R, show_sensordata_value,    NULL);
static DEVICE_ATTR(dumpallreg,	FIS210X_ATTR_R, show_dumpallreg_value, NULL);
static DEVICE_ATTR(layout,		FIS210X_ATTR_WR, show_layout_value, store_layout_value);
static DEVICE_ATTR(enable_acc,	FIS210X_ATTR_WR, fis210x_enable_show , fis210x_enable_store);
static DEVICE_ATTR(delay_acc,		FIS210X_ATTR_WR, fis210x_delay_show , fis210x_delay_store);
static DEVICE_ATTR(setreg,		FIS210X_ATTR_WR, fis210x_getreg , fis210x_setreg);
#if defined(ACC_USE_CALI)
static DEVICE_ATTR(cali,			FIS210X_ATTR_WR, acc_cali_show , acc_cali_store);
#endif

static struct attribute *fis210x_attributes[] = {
	&dev_attr_init_acc.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_layout.attr,
	&dev_attr_enable_acc.attr,
	&dev_attr_delay_acc.attr,
	&dev_attr_setreg.attr,
#if defined(ACC_USE_CALI)
	&dev_attr_cali.attr,
#endif
	NULL
};

static struct attribute_group fis210x_attribute_group = {
	.name = "fis210x",
	.attrs = fis210x_attributes
};

static long fis210x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	//void __user *argp = (void __user *)arg;
	//int temp = 0;

	FIS210X_LOG("%s: cmd %x\n",__func__, cmd);
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));

	if(err)
	{
		FIS210X_ERR("%s: access isn't ok!\n", __func__);
		return -EFAULT;
	}

	if (NULL == g_fis210x->client)
	{
		FIS210X_ERR("%s: i2c client isn't exist!\n", __func__);
		return -EFAULT;
	}
	
	switch(cmd)
	{
	default:
		FIS210X_ERR("%s: can't recognize the cmd!\n", __func__);
		return 0;
	}
	
    return 0;
}


static int fis210x_open(struct inode *inode, struct file *file)
{
	int err = 0;

	FIS210X_FUN();

	err = nonseekable_open(inode, file);
	if (err < 0)
	{
		FIS210X_ERR("%s: open fail!\n", __func__);
		return err;
	}

	file->private_data = i2c_get_clientdata(g_fis210x->client);

	return 0;
}

static int fis210x_release(struct inode *inode, struct file *file)
{
	FIS210X_FUN();
	file->private_data = NULL;
	return 0;
}

static const struct file_operations fis210x_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = fis210x_open,
	.release = fis210x_release,
	.unlocked_ioctl = fis210x_unlocked_ioctl,
};

static struct miscdevice fis210x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FIS210X_DEV_NAME,
	.fops = &fis210x_acc_misc_fops,
};

static int fis210x_acc_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int ret = 0;

	ret = fis210x_set_enable(FIS210X_TYPE_ACC, enable);

	return ret;
}

static int fis210x_acc_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
	fis210x_set_delay(FIS210X_TYPE_ACC, delay_msec);
	return 0;
}

static int fis210x_gyro_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int ret = 0;

	ret = fis210x_set_enable(FIS210X_TYPE_GYRO, enable);

	return ret;
}

static int fis210x_gyro_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
	fis210x_set_delay(FIS210X_TYPE_GYRO, delay_msec);
	return 0;
}


static int fis210x_acc_axis_calibrate(int *cal_xyz)
{
	int xyz[3] = { 0 };
	int arry[3] = { 0 };
	int err;
	int i;

	for(i = 0; i < FIS210X_CAL_MAX; i++) 
	{
		msleep(100);
		err = fis210x_acc_read_raw(xyz);
		if (err < 0) {
			printk("get_acceleration_data failed\n");
			return err;
		}
		if (i < FIS210X_CAL_SKIP_COUNT)
			continue;
		arry[0] += xyz[0];
		arry[1] += xyz[1];
		arry[2] += xyz[2];
	}
	cal_xyz[0] = arry[0] / (FIS210X_CAL_MAX - FIS210X_CAL_SKIP_COUNT);
	cal_xyz[1] = arry[1] / (FIS210X_CAL_MAX - FIS210X_CAL_SKIP_COUNT);
	cal_xyz[2] = arry[2] / (FIS210X_CAL_MAX - FIS210X_CAL_SKIP_COUNT);

	return 0;
}


static int fis210x_acc_calibrate(struct sensors_classdev *sensors_cdev,int axis, int apply_now)
{
	int err;
	int xyz[3] = { 0 };
#if 0
	if (acc_qst->enable_flag == false)
	{
		err = qmaX981_power_set(acc_qst, true);
		if (err) {
			MSE_ERR("Fail to power on the device!\n");
			return err;
		}
		err = qma6981_initialize(acc_qst->i2c);
		if (err < 0)
			return err;
	}
#endif
	err = fis210x_acc_axis_calibrate(xyz);
	if(err)
	{
		FIS210X_ERR("fis210x_acc_calibrate fail!\n");
		return err;
	}

	switch (axis) {
	case axis_x:
		xyz[1] = 0;
		xyz[2] = 0;
		break;
	case axis_y:
		xyz[0] = 0;
		xyz[2] = 0;
		break;
	case axis_z:
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = xyz[2] - g_fis210x->acc_lsb;
		break;
	case axis_total:
		xyz[2] = xyz[2] - g_fis210x->acc_lsb;
		break;
	default:
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = 0;
		FIS210X_ERR( "can not calibrate accel\n");
		break;
	}
	memset(g_fis210x->calibrate_buf, 0, sizeof(g_fis210x->calibrate_buf));
	snprintf(g_fis210x->calibrate_buf, sizeof(g_fis210x->calibrate_buf),"%d,%d,%d", xyz[0], xyz[1], xyz[2]);
	if(apply_now) {
		g_fis210x->cal_params[0] = xyz[0];
		g_fis210x->cal_params[1] = xyz[1];
		g_fis210x->cal_params[2] = xyz[2];
		g_fis210x->use_cal = true;
	}
	
	return 0;
}


static int fis210x_acc_write_cal_params(struct sensors_classdev *sensors_cdev,struct cal_result_t *cal_result)
{
	g_fis210x->cal_params[0] = cal_result->offset_x;
	g_fis210x->cal_params[1] = cal_result->offset_y;
	g_fis210x->cal_params[2] = cal_result->offset_z;

	snprintf(g_fis210x->calibrate_buf, sizeof(g_fis210x->calibrate_buf),
			"%d,%d,%d", g_fis210x->cal_params[0], g_fis210x->cal_params[1],g_fis210x->cal_params[2]);
	g_fis210x->use_cal = true;
	FIS210X_LOG( "read accel calibrate bias %d,%d,%d\n",g_fis210x->cal_params[0], g_fis210x->cal_params[1], g_fis210x->cal_params[2]);

	return 0;
}



#ifdef CONFIG_OF
static int fis210x_acc_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;
	unsigned int position = 0;
	
	ret = of_property_read_u32(np, "layout", &position);
	if(ret){
		dev_err(dev, "fail to get g_range\n");
		return 0;
	}
	atomic_set(&g_fis210x->position, (int)position);
	return 0;
}
#endif

void fis210x_initialize(struct i2c_client *client)
{
	g_fis210x->acc_uint = AccUnit_ms2;
	g_fis210x->acc_range = AccRange_4g;
	g_fis210x->acc_odr = AccOdr_256Hz;
	if(g_fis210x->acc_uint == AccUnit_ms2)
		g_fis210x->acc_scale = 9807;
	else
		g_fis210x->acc_scale = 1000;

	g_fis210x->gyro_uint = GyrUnit_dps;
	g_fis210x->gyro_range = GyrRange_512dps;	// GyrRange_1024dps
	g_fis210x->gyro_odr = GyrOdr_256Hz;			// GyrOdr_1024Hz;	
	g_fis210x->gyro_scale = 1000;

	fis210x_set_mode(FIS_MODE_NOMAL);
	fis210x_set_filter(Lpf_Disable, Hpf_Disable);

	FIS210X_LOG("fis210x acc init OK.\n");
}

static int fis210x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	static struct fis210x_data *fis210x;
	int err = 0;
	int layout = 0;

	FIS210X_LOG("%s: start\n",__func__);
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C|I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA))
	{
		FIS210X_ERR("%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit;
	}
	fis210x = kzalloc(sizeof(struct fis210x_data), GFP_KERNEL);
	if(!fis210x) {
		FIS210X_ERR("%s: can't allocate memory for fis210x_data!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	g_fis210x = fis210x;
	atomic_set(&fis210x->position, 0);
#ifdef CONFIG_OF
	if(client->dev.of_node)
	{
		fis210x_acc_parse_dt(&client->dev);
	}
#endif
	layout = atomic_read(&fis210x->position);
	fis210x_set_layout(layout);

	mutex_init(&fis210x->op_mutex);
	atomic_set(&fis210x->acc_delay, 200);
	atomic_set(&fis210x->gyro_delay, 200);
	fis210x->client = client;
	i2c_set_clientdata(client, fis210x);
    //wake_lock_init(&sc_wakelock,WAKE_LOCK_SUSPEND,"sc wakelock");
    client->addr = 0x6a;	//0x6b
	err = fis210x_read_chip_id(client);
	if(err < 0)
	{
		goto exit1;
		FIS210X_ERR("%s: g_fis210x read id fail!\n", __func__);
	}
	fis210x_initialize(client);

	INIT_DELAYED_WORK(&fis210x->acc_work, acc_work_func);
	INIT_DELAYED_WORK(&fis210x->gyro_work, gyro_work_func);

	err = fis210x_input_init(fis210x);
	if(err < 0) {
		FIS210X_ERR("input init fail!\n");
		goto exit1;
	}

	err = misc_register(&fis210x_device);
	if(err) {
		FIS210X_ERR("%s: create register fail!\n", __func__);
		goto exit2;
	}

	//err = sysfs_create_group(&fis210x->acc_input->dev.kobj, &fis210x_attribute_group);
	err = sysfs_create_group(&fis210x_device.this_device->kobj, &fis210x_attribute_group);
	if(err < 0) {
		FIS210X_ERR("%s: create group fail!\n", __func__);
		goto exit3;
	}

	fis210x->accel_cdev = fis210x_acc_cdev;
	fis210x->accel_cdev.delay_msec = 200;
	fis210x->accel_cdev.sensors_enable = fis210x_acc_enable_set;
	fis210x->accel_cdev.sensors_poll_delay = fis210x_acc_delay_set;
	fis210x->accel_cdev.sensors_calibrate = fis210x_acc_calibrate;
	fis210x->accel_cdev.sensors_write_cal_params = fis210x_acc_write_cal_params;
	memset(&fis210x->accel_cdev.cal_result, 0, sizeof(fis210x->accel_cdev.cal_result));
	fis210x->accel_cdev.params = fis210x->calibrate_buf;
	err = sensors_classdev_register(&fis210x->acc_input->dev, &fis210x->accel_cdev);
	if(err)
	{
		FIS210X_ERR("class device create failed: %d\n",err);
		goto exit4;
	}
	
	fis210x->gyro_cdev = fis210x_gyro_cdev;
	fis210x->gyro_cdev.delay_msec = 200;
	fis210x->gyro_cdev.sensors_enable = fis210x_gyro_enable_set;
	fis210x->gyro_cdev.sensors_poll_delay = fis210x_gyro_delay_set;
	err = sensors_classdev_register(&fis210x->gyro_input->dev, &fis210x->gyro_cdev);
	if(err)
	{
		FIS210X_ERR("create stepcount class device file failed!\n");
		err = -EINVAL;
		goto exit5;
	}
	
	return 0;

exit5:
	sensors_classdev_unregister(&fis210x->accel_cdev);
exit4:
	sysfs_remove_group(&g_fis210x->acc_input->dev.kobj, &fis210x_attribute_group);
exit3:
	misc_deregister(&fis210x_device);
exit2:
	fis210x_input_deinit(fis210x);	
exit1:
	kfree(fis210x);
	g_fis210x = NULL;
exit:
	return err;	
}

static int fis210x_remove(struct i2c_client *client)
{
	FIS210X_FUN();
	if(g_fis210x)
	{
		sensors_classdev_unregister(&g_fis210x->gyro_cdev);
		sensors_classdev_unregister(&g_fis210x->accel_cdev);
		sysfs_remove_group(&g_fis210x->acc_input->dev.kobj, &fis210x_attribute_group);
		fis210x_input_deinit(g_fis210x);
		kfree(g_fis210x);
		g_fis210x = NULL;
	}
	return 0;
}

static int fis210x_i2c_remove(struct i2c_client *client)
{
	return fis210x_remove(client);
}

static int fis210x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	FIS210X_FUN();

	cancel_delayed_work_sync(&g_fis210x->acc_work);
	cancel_delayed_work_sync(&g_fis210x->gyro_work);
	fis210x_set_mode(FIS_MODE_POWER_DOWN);

	return 0;
}

static int fis210x_resume(struct i2c_client *client)
{
	int delay = fis210x_get_delay();

	FIS210X_FUN();
	if(fis210x_get_enable(FIS210X_TYPE_ACC))
	{
		fis210x_set_mode(FIS_MODE_NOMAL);
		schedule_delayed_work(&g_fis210x->acc_work,msecs_to_jiffies(delay));
	}
	if(fis210x_get_enable(FIS210X_TYPE_GYRO))
	{
		fis210x_set_mode(FIS_MODE_NOMAL);
		schedule_delayed_work(&g_fis210x->gyro_work,msecs_to_jiffies(delay));
	}

	return 0;
}


static const struct i2c_device_id fis210x_id[] = {
	{FIS210X_DEV_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, fis210x_id);

#ifdef CONFIG_OF
static const struct of_device_id fis210x_acc_of_match[] = {
       { .compatible = "qst,fis210x", },
       { }
};
#endif

MODULE_DEVICE_TABLE(of, fis210x_acc_of_match);
static struct i2c_driver fis210x_driver = {
	.driver = {
		.name = FIS210X_DEV_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = fis210x_acc_of_match,
#endif
	},
	.probe    = fis210x_i2c_probe,
	.remove   = fis210x_i2c_remove,
	.suspend  = fis210x_suspend,
	.resume   = fis210x_resume,
	.id_table = fis210x_id,
};

static int __init fis210x_i2c_init(void)
{
	FIS210X_LOG("%s accelerometer driver: init\n", FIS210X_DEV_NAME);

	return i2c_add_driver(&fis210x_driver);
}

static void __exit fis210x_i2c_exit(void)
{
	FIS210X_LOG("%s accelerometer driver exit\n", FIS210X_DEV_NAME);

	i2c_del_driver(&fis210x_driver);
}


module_init(fis210x_i2c_init);
//late_initcall(fis210x_i2c_init);
module_exit(fis210x_i2c_exit);

MODULE_DESCRIPTION("fis210x accelerometer driver");
MODULE_AUTHOR("QST-inc");
MODULE_LICENSE("GPL");
MODULE_VERSION(FIS210X_DEV_VERSION);


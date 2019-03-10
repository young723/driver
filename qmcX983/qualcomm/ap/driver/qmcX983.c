/****************************************************************************
*   This software is licensed under the terms of the GNU General Public License version 2,
*   as published by the Free Software Foundation,
*	and may be copied, distributed, and
*   modified under those terms.
*	This program is distributed in the hope that it will be useful,
*	but WITHOUT ANY
*   WARRANTY; without even the implied warranty of MERCHANTABILITY or
*	FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
*   for more details.
*
*	Copyright (C) 2012-2014 by QST(Shanghai XiRui Keji) Corporation
****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/qmcX983.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>

#define MSE_TAG						"[Msensor]"
#define MSE_FUN(f)					printk(MSE_TAG"%s\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)		printk(KERN_ERR MSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)		printk(MSE_TAG fmt, ##args)

/* Magnetometer registers */
#define CTL_REG_ONE	0x09  /* Contrl register one */
#define CTL_REG_TWO	0x0a  /* Contrl register two */

/* Output register start address*/
#define OUT_X_REG		0x00

/*Status registers */
#define STA_REG_ONE    0x06
#define STA_REG_TWO    0x0c

/* Temperature registers */
#define TEMP_H_REG 		0x08
#define TEMP_L_REG 		0x07

/*different from QMCX983,the ratio register*/
#define RATIO_REG		0x0b

/* POWER SUPPLY VOLTAGE RANGE */
#define QMCX983_VDD_MIN_UV	2160000
#define QMCX983_VDD_MAX_UV	3600000
#define QMCX983_VIO_MIN_UV	1650000
#define QMCX983_VIO_MAX_UV	3600000

#define QCOM_PLATFORM
#define	QMCX983_BUFSIZE		0x20

#define QMC6983_A1_D1             0
#define QMC6983_E1		  		  1	
#define QMC7983                   2
#define QMC7983_LOW_SETRESET      3

/*
 * QMCX983 magnetometer data
 * brief Structure containing magnetic field values for x,y and z-axis in
 * signed short
*/

struct QMCX983_t {
	short	x, /**< x-axis magnetic field data. Range -8000 to 8000. */
			y, /**< y-axis magnetic field data. Range -8000 to 8000. */
			z; /**< z-axis magnetic filed data. Range -8000 to 8000. */
};

/* Save last device state for power down */
struct QMC_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct QMCX983_data{
	struct i2c_client 				*client;
	struct input_dev 				*input;
	struct device					*class_dev;
	struct class					*compass;
	struct sensors_classdev			cdev;
	struct workqueue_struct 		*work_queue;
	struct delayed_work 			dwork;
	
	struct mutex					val_mutex;
	struct mutex					op_mutex;
	
	wait_queue_head_t 				state_wq;
	
	int64_t 						delay;
	int								power_enabled;
	uint32_t 						enable_flag;
	char							layout;
	int								gpio_rstn;
	int								auto_report;
	short 							xy_sensitivity;
	short 							z_sensitivity;
	struct regulator 				*vdd;
	struct regulator 				*vio;
	struct QMC_sensor_state			state;
};

static struct QMCX983_data *mag;
static int chip_id = QMC6983_E1;
static int OTP_Kx;
static int OTP_Ky;

static struct sensors_classdev sensors_cdev = {
	.name = "qmcX983-mag",
	.vendor = "QST",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1228.8",
	.resolution = "0.04",
	.sensor_power = "0.35",
	.min_delay = 10000,
	.max_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


static char qmcX983_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len);

static char qmcX983_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len);

int qmcX983_read_mag_xyz(struct QMCX983_t *data);
int qmcX983_set_range(short range);
static int qmc_compass_power_set(struct QMCX983_data *data, bool on);
static int qmc_compass_power_init(struct QMCX983_data *data, bool on);
static int qmc_compass_parse_dt(struct device *dev,struct QMCX983_data *mag);

/* X,Y and Z-axis magnetometer data readout
 * param *mag pointer to \ref QMCX983_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
 */
int qmcX983_read_mag_xyz(struct QMCX983_t *data)
{
	int res;
	unsigned char mag_data[6] = {0};
	int hw_d[3] = {0};
	int t1 = 0;
	unsigned char rdy = 0;
	
	while(!(rdy & 0x07) && t1<3){
		res=qmcX983_i2c_read(0x06,mag_data,1);
		rdy=mag_data[0];
		MSE_LOG("qmcX983 Status Register is (0x%02X)\n",rdy);
		t1 ++;
	}
	
	res = qmcX983_i2c_read(OUT_X_REG, mag_data, 6);

	MSE_LOG("raw data: %d %d %d %d %d %d\n",
	mag_data[0], mag_data[1], mag_data[2],
	mag_data[3], mag_data[4], mag_data[5]);
	
	hw_d[0] = (short) (((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short) (((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short) (((mag_data[5]) << 8) | mag_data[4]);


	hw_d[0] = hw_d[0] * 1000 / mag->xy_sensitivity;
	hw_d[1] = hw_d[1] * 1000 / mag->xy_sensitivity;
	hw_d[2] = hw_d[2] * 1000 / mag->z_sensitivity;

	MSE_LOG("Hx=%d Hy=%d Hz=%d\n",hw_d[0],hw_d[1],hw_d[2]);
	data->x = hw_d[0];
	data->y = hw_d[1];
	data->z = hw_d[2];

	return res;
}

/* Set the Gain range */
int qmcX983_set_range(short range)
{
	int err = 0;
	int ran = 0;	
	unsigned char data[1] = {0};
	
	MSE_FUN(f);
	
	switch (range) {
	case QMCX983_RNG_2G:
		ran = RNG_2G;
		break;
	case QMCX983_RNG_8G:
		ran = RNG_8G;
		break;
	case QMCX983_RNG_12G:
		ran = RNG_12G;
		break;
	case QMCX983_RNG_20G:
		ran = RNG_20G;
		break;
	default:
		return -EINVAL;
	}
	mag->xy_sensitivity = 20000/ran;
	mag->z_sensitivity = 20000/ran;

	qmcX983_i2c_read(CTL_REG_ONE, &data[0], 1);
	data[0] &= 0xcf;
	data[0] |= (range << 4);
	
	err = qmcX983_i2c_write(CTL_REG_ONE, &data[0], 1);
	
	return err;
}

/* Set the sensor mode */
int qmcX983_set_mode(char mode)
{
	int err = 0;
	unsigned char data;

	qmcX983_i2c_read(CTL_REG_ONE, &data, 1);
	data &= 0xfc;
	data |= mode;
	err = qmcX983_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}

int qmcX983_set_ratio(char ratio)
{
	int err = 0;
	unsigned char data;

	data = ratio;
	err = qmcX983_i2c_write(RATIO_REG, &data, 1);
	return err;
}

int qmcX983_set_output_data_rate(char rate)
{
	int err = 0;
	unsigned char data;

	data = rate;
	data &= 0xf3;
	data |= (rate << 2);
	err = qmcX983_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}

int qmcX983_set_oversample_ratio(char ratio)
{
	int err = 0;
	unsigned char data;

	data = ratio;
	data &= 0x3f;
	data |= (ratio << 6);
	err = qmcX983_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}


/*  i2c write routine for QMCX983 magnetometer */
static char qmcX983_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len)
{
	int ret;
	int i;

	if (mag->client == NULL)  /*  No global client pointer? */
		return -1;
	for (i = 0; i < len; i++) {
		ret = i2c_smbus_write_byte_data(mag->client,
						  reg_addr++, data[i]);
		if (ret) {
			MSE_ERR("i2c write error\n");
			return ret;
		}
	}
	return 0;
}

/*  i2c read routine for QMCX983 magnetometer */
static char qmcX983_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	char ret = 0;
	int i = 0;

	if (mag->client == NULL)  /*  No global client pointer? */
		return -1;

	while (i < len) 
   {
		ret = i2c_smbus_read_byte_data(mag->client,
						 reg_addr++);		
		if (ret >= 0) {
			data[i] = ret;
			i++;
		} else {
			MSE_ERR("i2c read error\n");
			return ret;
		}
		ret = len;
	}
	return ret;
}

static void qmcX983_start_measure(struct QMCX983_data *QMCX983)
{
	int err = 0;
	unsigned char data = 0;
	data = 0x1d;
	MSE_LOG("qmcX983_start_measure data = 0x%0x\n",data);
	err = qmcX983_i2c_write(CTL_REG_ONE, &data, 1);
}

static void qmcX983_stop_measure(struct QMCX983_data *QMCX983)
{
	unsigned char data = 0;
	data = 0x1c;
	MSE_LOG("qmcX983_stop_measure data = 0x%0x\n",data);
	qmcX983_i2c_write(CTL_REG_ONE, &data, 1);
}

static int qmcX983_enable(struct QMCX983_data *QMCX983)
{
	int err = 0;
	unsigned char data = 0;

	MSE_FUN(f);
	
	data = 0x40;
	err = qmcX983_i2c_write(0x20, &data, 1);
	
	data = 0x01;
	err = qmcX983_i2c_write(0x21, &data, 1);	
	
	if(chip_id == QMC6983_E1 || chip_id == QMC7983 || chip_id == QMC7983_LOW_SETRESET)
	{

		data = 0x80;
		err = qmcX983_i2c_write(0x29,&data,1); 	
		
		data = 0x0c;
		err = qmcX983_i2c_write(0x0a,&data,1); 		
	}
	
	qmcX983_set_range(QMCX983_RNG_8G);  
	qmcX983_set_ratio(QMCX983_SETRESET_FREQ_FAST);
	qmcX983_start_measure(QMCX983);	
	msleep(10);
	
	return err;
}

static int qmcX983_disable(struct QMCX983_data *QMCX983)
{
	qmcX983_stop_measure(QMCX983);
	MSE_FUN(f);
	return 0;
}


static void qmcX983_poll_work(struct work_struct *work)
{
	int ret;
	struct QMCX983_data *QMCX983 = container_of((struct delayed_work *)work,
						struct QMCX983_data, dwork);
						
	unsigned char data[6] = {0};
	
	if (!QMCX983->enable_flag)
		return ;
	
	ret = qmcX983_read_mag_xyz((struct QMCX983_t *)data);
	if (ret < 0) {
		MSE_ERR("qmcX983_read_mag_xyz error\n");
	}

	input_report_abs(QMCX983->input, ABS_X, ((struct QMCX983_t *)data)->x);
	input_report_abs(QMCX983->input, ABS_Y, ((struct QMCX983_t *)data)->y);
	input_report_abs(QMCX983->input, ABS_Z, ((struct QMCX983_t *)data)->z);
	input_sync(QMCX983->input);

	//submit work to work_queue within jiffies
	queue_delayed_work(QMCX983->work_queue,&QMCX983->dwork,
						(unsigned long)nsecs_to_jiffies64(QMCX983->delay));
}

static int qmcX983_input_init(struct QMCX983_data *QMCX983)
{
	struct input_dev *dev;
	int ret;
	MSE_FUN(f);
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "compass";
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	/* Magnetic field (limited to 16bit) */
	input_set_abs_params(dev, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_Z, -32768, 32767, 0, 0);			

	ret = input_register_device(dev);

	input_set_drvdata(dev, QMCX983);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	QMCX983->input= dev;
	MSE_LOG("%s: ok!\n",__func__);
	return 0;
}


static int qmcX983_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct QMCX983_data *mag = container_of(sensors_cdev,
			struct QMCX983_data, cdev);
			
	MSE_LOG("%s : delay = %d\n",__func__, delay_msec);

	mutex_lock(&mag->val_mutex);
	mag->delay = delay_msec * 1000 * 1000;
	mutex_unlock(&mag->val_mutex);
	return 0;
}

static int qmcX983_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	
	struct QMCX983_data *mag = container_of(sensors_cdev,
			struct QMCX983_data, cdev);	
			
	mutex_lock(&mag->val_mutex);
	mag->enable_flag = enable;
	mutex_unlock(&mag->val_mutex);
	
	mutex_lock(&mag->op_mutex);
	
	if (enable){
		ret = qmc_compass_power_set(mag, true);
		if (ret) {
			MSE_ERR("Fail to power on the device!\n");
			goto exit;
		}
		qmcX983_enable(mag);		
		if(mag->auto_report){
			queue_delayed_work(mag->work_queue,&mag->dwork,
								(unsigned long)nsecs_to_jiffies64(mag->delay));}
	} else {
		if(mag->auto_report){
			cancel_delayed_work_sync(&mag->dwork);
		}
		ret = qmc_compass_power_set(mag, false);
		if (ret) {
			MSE_ERR("Fail to power off the device!\n");
			goto exit;
		}
		qmcX983_disable(mag);		
	}
 exit:
	mutex_unlock(&mag->op_mutex);
	return ret;
}
	
static ssize_t qmc_delay_mag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	int64_t val;

	mutex_lock(&QMCX983->val_mutex);
	val = QMCX983->delay;
	mutex_unlock(&QMCX983->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", val);	
}

static ssize_t qmc_delay_mag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	
	unsigned long val = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&QMCX983->val_mutex);
	QMCX983->delay = val;
	mutex_unlock(&QMCX983->val_mutex);

	return count;
}

static ssize_t qmc_enable_mag_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&QMCX983->val_mutex);
	val = QMCX983->enable_flag;
	mutex_unlock(&QMCX983->val_mutex);
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t qmc_enable_mag_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	
	unsigned long en = 0;

	if (NULL == buf)
		return -EINVAL;
	
	if (0 == count)
		return 0;
	
	if (strict_strtoul(buf, 10, &en))
		return -EINVAL;
	
	en = en ? 1 : 0;
	
	MSE_LOG("%s: enable=%lu\n",__func__, en);

	mutex_lock(&QMCX983->op_mutex);
	if (en) {
		qmcX983_enable(QMCX983);
		queue_delayed_work(QMCX983->work_queue,&QMCX983->dwork,
							(unsigned long)nsecs_to_jiffies64(QMCX983->delay));
	} else {
		qmcX983_disable(QMCX983);
		cancel_delayed_work_sync(&QMCX983->dwork);
	}
	mutex_unlock(&QMCX983->op_mutex);
	
	mutex_lock(&QMCX983->val_mutex);	
	QMCX983->enable_flag = en;
	mutex_unlock(&QMCX983->val_mutex);

	return count;
}


#ifdef QCOM_PLATFORM
static unsigned char regbuf[2] = {0};
static ssize_t show_registers_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	//char strbuf[QMCX983_BUFSIZE];
	unsigned char data;
	//unsigned char rdy = 0;
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	
	mutex_lock(&QMCX983->op_mutex);
	qmcX983_i2c_read(regbuf[0], &data, 1);
	mutex_unlock(&QMCX983->op_mutex);
				
	MSE_LOG("%s REG[0x%2x] = 0x%02x\n",__func__,regbuf[0],data);  

	return scnprintf(buf, PAGE_SIZE, "REG[0x%2x] = 0x%02x\n",regbuf[0],data);		 
}

static ssize_t store_registers_value(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	int err = 0;
	unsigned char data[2] = {0};
	
	if(NULL == QMCX983)
	{
		MSE_ERR("QMCX983_i2c_data is NULL!\n");
		return 0;
	}
	
	sscanf(buf,"0x%2x",(unsigned int*)&regbuf[1]);
	
	data[1] = regbuf[1];
	data[0] = regbuf[0];
	err = qmcX983_i2c_write(data[0],&data[1],1);
	
	if(err != 0)
	   MSE_ERR("%s Reg[0x%2x] ---> 0x%2x\n",__func__,regbuf[0],regbuf[1]);

	return err;
}

static ssize_t show_registers_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
       
	MSE_LOG("%s REG_ADDR = 0x%02x\n",__func__,regbuf[0]);  
	   
	return scnprintf(buf, PAGE_SIZE, "REG_ADDR = 0x%02x\n", regbuf[0]);
}

static ssize_t store_registers_addr(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	if(NULL == QMCX983)
	{
		MSE_ERR("QMCX983_i2c_data is NULL!\n");
		return 0;
	}
	
	if(1 == sscanf(buf,"0x%2x",(unsigned int*)&regbuf[0])){
		MSE_LOG("%s REG_ADDR = 0x%2x\n",__func__,regbuf[0]);
	}else{
		regbuf[0] = 0x00;
	}

	return count;
}

static ssize_t show_dumpallreg_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[300];
	char tempstrbuf[24];
	unsigned char data;
	int length=0;

	int i;
  
	/* Check status register for data availability */
	for(i =0;i<12;i++)
	{
		qmcX983_i2c_read(i, &data, 1);
		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "reg[0x%2x]=0x%2x \n",i, data);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "%s\n",tempstrbuf);
	}
	
	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);	   
}
#endif

static ssize_t show_layout_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",mag->layout);	
}

static ssize_t show_OTP_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d,%d\n",OTP_Kx,OTP_Ky);	
}

#ifdef QCOM_PLATFORM
DEVICE_ATTR(mag_dumpallreg,  S_IRUGO , show_dumpallreg_value, NULL);
DEVICE_ATTR(mag_registers_value, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH, show_registers_value, store_registers_value);
DEVICE_ATTR(mag_registers_addr,  S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH, show_registers_addr, store_registers_addr);
#endif
DEVICE_ATTR(delay_mag, S_IRUGO | S_IWUSR, qmc_delay_mag_show, qmc_delay_mag_store);
DEVICE_ATTR(enable_mag, S_IRUGO | S_IWUSR, qmc_enable_mag_show, qmc_enable_mag_store);
DEVICE_ATTR(mag_layout, S_IRUGO | S_IWUSR, show_layout_value, NULL);
DEVICE_ATTR(mag_otp, S_IRUGO | S_IWUSR, show_OTP_value, NULL);

static struct attribute *mag_attributes[] = {
#ifdef QCOM_PLATFORM
	&dev_attr_mag_dumpallreg.attr,
	&dev_attr_mag_registers_value.attr,
	&dev_attr_mag_registers_addr.attr,
#endif 
	&dev_attr_delay_mag.attr,
	&dev_attr_enable_mag.attr,
	&dev_attr_mag_layout.attr,
	&dev_attr_mag_otp.attr,
	NULL
};

static struct attribute_group mag_attribute_group = {
	.attrs = mag_attributes
};

static int qmcX983_open(struct inode *inode, struct file *file)
{
	file->private_data = mag;
	return nonseekable_open(inode, file);
}

static int qmcX983_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* ioctl command for QMCX983 device file */
static long qmcX983_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int buff[2]={0};
	void __user *argp = (void __user *)arg;
	
	struct QMCX983_data *mag = file->private_data;
	
	MSE_LOG("qmcX983_unlocked_ioctl - cmd=%x, arg = %x\n",cmd, *((unsigned char *)arg));
	
	if(mag->client == NULL)
	{
		return -EFAULT; 
	}
	
	switch (cmd)
	{
		case QMCX983_SET_RANGE:
			if (copy_from_user(data, (unsigned char *)argp, sizeof(data[0])) != 0) {
				MSE_ERR("qmcX983_set_range error\n");
				return -EFAULT;
			}
			err = qmcX983_set_range(*data);
			return err;

		case QMCX983_SET_MODE:
			if (copy_from_user(data, (unsigned char *)argp, sizeof(data[0])) != 0) {
				MSE_ERR("qmcX983_set_mode error\n");
				return -EFAULT;
			}
			err = qmcX983_set_mode(data[0]);
			return err;
	 
		case QMCX983_READ_MAGN_XYZ:
			err = qmcX983_read_mag_xyz((struct QMCX983_t *)data);
			MSE_LOG("QMCX983_READ_MAGN_XYZ[%d, %d, %d, %d, %d, %d]\n",
					data[0], data[1], data[2],
					data[3], data[4], data[5]);
			if (copy_to_user(argp, (struct QMCX983_t *)data, sizeof(data)) != 0) {
				return -EFAULT;
			}
			return err;

		case QMCX983_SET_OUTPUT_DATA_RATE:
			if (copy_from_user(data, (unsigned char *)argp, sizeof(data[0])) != 0) {
				MSE_ERR("qmcX983_set_output_data_rate error\n");
				return -EFAULT;
			}
			err = qmcX983_set_output_data_rate(data[0]);
			return err;

		case QMCX983_SET_OVERSAMPLE_RATIO:
			if (copy_from_user(data, (unsigned char *)argp, data[0]) != 0) {
				MSE_ERR("qmcX983_set_oversample_ratio error\n");
				return -EFAULT;
			}
			err = qmcX983_set_oversample_ratio(data[0]);
			return err;
			
		case QMCX983_IOCTL_GET_DIRECTION:
			if(copy_to_user(argp,&(mag->layout),sizeof(char))!=0){
				return -EFAULT;
			}
			break;
		case QMCX983_IOCTL_GET_OTPK:
			buff[0] = OTP_Kx;
			buff[1] = OTP_Ky;
			if(copy_to_user(argp,buff,sizeof(buff))!=0){
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	return 0;
}

static const struct file_operations qmcX983_fops = {
	.owner = THIS_MODULE,
	.open = qmcX983_open,
	.release = qmcX983_release,
	.unlocked_ioctl = qmcX983_unlocked_ioctl,
};

static struct miscdevice qmc_compass_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMC_MISCDEV_NAME,
	.fops = &qmcX983_fops,
};


static int qmcX983_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct QMCX983_platform_data *pdata;
	int err = 0;
	unsigned char databuf[2] = {0};
	unsigned char data = 0;

	MSE_FUN();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		MSE_ERR("%s: check_functionality failed.\n", __func__);
		err = -ENODEV;
		goto exit0;
	}
	
	mag = kzalloc(sizeof(struct QMCX983_data), GFP_KERNEL);
	if (!mag) {
		MSE_ERR("%s: memory allocation failed\n", __func__);
		err = -ENOMEM;
		goto exit1;
	}
	
	mag->delay = -1;
	mutex_init(&mag->val_mutex);
	mutex_init(&mag->op_mutex);
	
	mag->enable_flag = 0;
	
	init_waitqueue_head(&mag->state_wq);
	
	if (client->dev.of_node) {
		err = qmc_compass_parse_dt(&client->dev, mag);
		if (err) {
			MSE_ERR("Unable to parse platfrom data err = %d\n", err);
			goto exit2;
		}
	} else {
		if (client->dev.platform_data) {
			/* Copy platform data to local. */
			pdata = client->dev.platform_data;
			mag->layout = pdata->layout;
			mag->auto_report = 1;
		} else {
		/* Platform data is not available.
		   Layout and information should be set by each application. */
			mag->layout = 0;
			mag->auto_report = 1;
			MSE_LOG("%s: No platform data.\n",__func__);
		}
	}
	
	mag->client = client;
	i2c_set_clientdata(client, mag);

	/* read chip id */
	qmcX983_i2c_read(0x0d,databuf,1);

	if (databuf[0] == 0xff )
	{
		chip_id = QMC6983_A1_D1;
	}
	else if(databuf[0] == 0x31) //QMC7983 asic 
	{
		qmcX983_i2c_read(0x3E,databuf,1);
		if((databuf[0] & 0x20) == 1)
			chip_id = QMC6983_E1;
		else if((databuf[0] & 0x20) == 0)
			chip_id = QMC7983;
	}
	else if(databuf[0] == 0x32) //QMC7983 asic low setreset
	{
		chip_id = QMC7983_LOW_SETRESET;	
	}
	else 
	{
		MSE_ERR("No QST CHIP ONBOARD\n");
		goto exit2;
	}
	MSE_LOG("%s chip_id = %d\n",__func__,chip_id);	
	
	if(chip_id == QMC6983_A1_D1){
		OTP_Kx = 0;
		OTP_Ky = 0;
	}else{
		//read kx
		data = 0x0a;
		qmcX983_i2c_write(0X2e,&data,1);
	
		qmcX983_i2c_read(0X2f,&databuf[0],1);
		
		if(((databuf[0]&0x3f) >> 5) == 1)
			OTP_Kx = (databuf[0]&0x1f)-32;
		else
			OTP_Kx = databuf[0]&0x1f;	
		
		MSE_LOG("%s: OTP_Kx %d\n",__func__,OTP_Kx);
		
		//read ky
		data = 0x0d;
		qmcX983_i2c_write(0x2e,&data,1);
		
		qmcX983_i2c_read(0x2f,&databuf[0],1);
		
		data = 0x0f;
		qmcX983_i2c_write(0x2e,&data,1);
		
		qmcX983_i2c_read(0x2f,&databuf[1],1);
		
		if((databuf[0] >> 7) == 1)
			OTP_Ky = (((databuf[0]&0x70) >> 4)*4 + (databuf[1] >> 6))-32;
		else
			OTP_Ky = (((databuf[0]&0x70) >> 4)*4 + (databuf[1] >> 6));	
		
		MSE_LOG("%s: OTP_Ky %d\n",__func__,OTP_Ky);
	}
	
	/* check connection */
	err = qmc_compass_power_init(mag, true);
	if (err < 0)
		goto exit2;

	err = qmc_compass_power_set(mag, true);
	if (err < 0)
		goto exit3;

	/* Create input device for msensor */
	err = qmcX983_input_init(mag);
	if (err < 0) {
		MSE_ERR("%s: error init input dev interface\n",__func__);
		goto exit4;
	}
	
	mag->work_queue = alloc_workqueue("qmcX983_poll_work",WQ_NON_REENTRANT,0);
	INIT_DELAYED_WORK(&mag->dwork, qmcX983_poll_work);
	
	err = misc_register(&qmc_compass_dev);
	
	if(err){
		MSE_ERR("%s: misc register failed\n",__func__);
		goto exit5;
	}
	
	err = sysfs_create_group(&qmc_compass_dev.this_device->kobj,&mag_attribute_group);
	
	if(err < 0){
		MSE_ERR("%s: sysfs_create_group fail\n",__func__);
		goto exit6;
	}
	
	kobject_uevent(&qmc_compass_dev.this_device->kobj,KOBJ_ADD);
	
	mag->cdev = sensors_cdev;
	mag->cdev.sensors_enable = qmcX983_enable_set;
	mag->cdev.sensors_poll_delay = qmcX983_poll_delay_set;

	mag->delay = sensors_cdev.delay_msec * 1000 *1000;
	
	err = sensors_classdev_register(&mag->input->dev, &mag->cdev);
	if (err) {
		MSE_ERR("%s: class device create failed: %d\n",__func__,err);
		goto exit7;
	}

	qmc_compass_power_set(mag,false);
	
	MSE_LOG("%s: successfully probed.\n",__func__);
	return 0;
	
exit7:
	sysfs_remove_group(&qmc_compass_dev.this_device->kobj,&mag_attribute_group);
exit6:
	misc_deregister(&qmc_compass_dev);
exit5:
	input_unregister_device(mag->input);
exit4:
	qmc_compass_power_set(mag,0);
exit3:
	qmc_compass_power_init(mag, 0);
exit2:
	kfree(mag);
exit1:
exit0:
	return err;
}

static int qmcX983_remove(struct i2c_client *client)
{
	struct QMCX983_data *mag = i2c_get_clientdata(client);
	
	if(mag->auto_report){
		cancel_delayed_work_sync(&mag->dwork);
	}

	if (qmc_compass_power_set(mag, false))
		MSE_ERR("power off failed.\n");
	
	if (qmc_compass_power_init(mag, false))
		MSE_ERR("power deinit failed.\n");

	sensors_classdev_unregister(&mag->cdev);
	sysfs_remove_group(&qmc_compass_dev.this_device->kobj,&mag_attribute_group);
	misc_deregister(&qmc_compass_dev);
	input_unregister_device(mag->input);

	kfree(mag);
	mag->client = NULL;
	MSE_LOG("%s: successfully removed.\n",__func__);
	return 0;
}

static int qmc_compass_power_set(struct QMCX983_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			MSE_ERR("Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			MSE_ERR("Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			MSE_ERR("Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			MSE_ERR("Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		MSE_LOG("Power on=%d. enabled=%d\n",on, data->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		MSE_LOG("Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int qmc_compass_power_init(struct QMCX983_data *data, bool on)
{
	int rc;
	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				QMCX983_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				QMCX983_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			MSE_ERR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				QMCX983_VDD_MIN_UV, QMCX983_VDD_MAX_UV);
			if (rc) {
				MSE_ERR("Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			MSE_ERR("Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				QMCX983_VIO_MIN_UV, QMCX983_VIO_MAX_UV);
			if (rc) {
				MSE_ERR("Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, QMCX983_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int qmc_compass_parse_dt(struct device *dev,
				struct QMCX983_data *mag)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "qst,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		MSE_ERR("Unable to read qst,layout\n");
		return rc;
	} else {
		mag->layout = temp_val;
	}

	mag->auto_report = of_property_read_bool(np, "qst,auto-report");

	return 0;
}

static int qmcX983_suspend(struct device *dev)
{
	int res = 0;
	
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	
	if(QMCX983->enable_flag && QMCX983->auto_report){
		cancel_delayed_work_sync(&QMCX983->dwork);	
	}
	
	qmcX983_disable(QMCX983);
	
	QMCX983->state.power_on = QMCX983->power_enabled;
	if (QMCX983->state.power_on) {
		res = qmc_compass_power_set(QMCX983,false);
		if(res){
			MSE_ERR("failed to suspend QMCX983\n");
		}
	}
	MSE_LOG("qmcX983_suspend\n");
	return res;
}

static int qmcX983_resume(struct device *dev)
{
	struct QMCX983_data *QMCX983 = dev_get_drvdata(dev);
	int res = 0 ;
	
	if (QMCX983->state.power_on) {
		res = qmc_compass_power_set(QMCX983,true);
		if (res) {
			MSE_ERR("Sensor power resume fail!\n");
			goto exit;
		}
		qmcX983_enable(QMCX983);
		
		queue_delayed_work(QMCX983->work_queue,&QMCX983->dwork,
							(unsigned long)nsecs_to_jiffies64(QMCX983->delay));	
	}
	
	MSE_LOG("qmcX983_resume\n");
	
exit:	
	return res;
}


static const struct i2c_device_id QMCX983_i2c_id[] = {
	{QMCX983_DEV_NAME,0},
	{}
};

static const struct dev_pm_ops QMC_compass_pm_ops = {
	.suspend	= qmcX983_suspend,
	.resume		= qmcX983_resume,
};

static struct of_device_id QMCX983_match_table[] = {
	{ .compatible = "qst,qmcX983", },
	{ },
};

static struct i2c_driver qmcX983_driver = {
	.probe = qmcX983_probe,
	.remove = qmcX983_remove,
	.id_table = QMCX983_i2c_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = QMCX983_DEV_NAME,
		.of_match_table = QMCX983_match_table,
		.pm	= &QMC_compass_pm_ops,
	},
};

static int __init qmcX983_init(void)
{
	MSE_FUN(f);	 
	return i2c_add_driver(&qmcX983_driver);
}

static void __exit qmcX983_exit(void)
{
	MSE_FUN(f);
	i2c_del_driver(&qmcX983_driver);
}

module_init(qmcX983_init);
module_exit(qmcX983_exit);

MODULE_DESCRIPTION("QMC983 magnetometer driver");
MODULE_AUTHOR("QST");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.1");



/* qmc6308.c - qmc6308 compass driver
 *
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

#include <cust_mag.h>
#include "qmc6308.h"
#include "mag.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*----------------------------------------------------------------------------*/
#define DEBUG 1
#define qmc6308_DEV_NAME         "qmc6308"
#define DRIVER_VERSION          "driver version V1.0"
/*----------------------------------------------------------------------------*/

#define	QMC6308_STR_BUFSIZE		32
#define QMC6308_RW_BUFSIZE      16

#define qmc6308_AXIS_X            0
#define qmc6308_AXIS_Y            1
#define qmc6308_AXIS_Z            2
#define qmc6308_AXES_NUM          3

#define qmc6308_DEFAULT_DELAY 100

#define MSE_TAG					"[QMC-Msensor] "
#define MSE_FUN(f)				pr_info(MSE_TAG"%s\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)	pr_err(MSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)	pr_info(MSE_TAG fmt, ##args)


static struct i2c_client *this_client = NULL;
static short qmcd_delay = qmc6308_DEFAULT_DELAY;

//static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;

SENSOR_MASK mask = {
	.mask0 = 0x80,
	.mask1 = 0xA0,
	.mask2 = 0xB0,
	.mask3 = 0xC0,
};

static atomic_t open_flag = ATOMIC_INIT(0);
static unsigned char v_open_flag = 0x00;

/*----------------------------------------------------------------------------*/

static const struct i2c_device_id qmc6308_i2c_id[] = {{qmc6308_DEV_NAME,0},{}};

static struct mag_hw mag_cust;
static struct mag_hw *qst_hw = &mag_cust;

/* For  driver get cust info */
struct mag_hw *get_cust_mag(void)
{
	return &mag_cust;
}

/*----------------------------------------------------------------------------*/
static int qmc6308_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int qmc6308_i2c_remove(struct i2c_client *client);
static int qmc6308_suspend(struct device *dev);
static int qmc6308_resume(struct device *dev);
static int qmc6308_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int qmc6308_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs);
static int qmc6308_flush(void);
static int qmc6308_m_get_data(int *x, int *y, int *z, int *status);


/*----------------------------------------------------------------------------*/
typedef enum {
    QMC_FUN_DEBUG  = 0x01,
	QMC_DATA_DEBUG = 0x02,
	QMC_HWM_DEBUG  = 0x04,
	QMC_CTR_DEBUG  = 0x08,
	QMC_I2C_DEBUG  = 0x10,
} QMC_TRC;

/*----------------------------------------------------------------------------*/
struct qmc6308_i2c_data {
    struct i2c_client *client;
    struct mag_hw hw;
	struct hwmsen_convert	cvt;
    atomic_t layout;
    atomic_t trace;	
	short sensitivity;
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

#define DATA_AVG_DELAY 6
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
    { .compatible = "mediatek,msensor", },
    {},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops qmc6308_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(qmc6308_suspend, qmc6308_resume)
};
#endif

static struct i2c_driver qmc6308_i2c_driver = {
    .driver = {
        .name  = qmc6308_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm    = &qmc6308_pm_ops,
#endif
#ifdef CONFIG_OF
	.of_match_table = mag_of_match,
#endif
    },
	.probe      = qmc6308_i2c_probe,
	.remove     = qmc6308_i2c_remove,
	.detect = qmc6308_i2c_detect,
	.id_table = qmc6308_i2c_id,
};


static int qmc6308_local_init(void);
static int qmc6308_local_remove(void);
static int qmc6308_init_flag = -1; // 0<==>OK  -1 <==> fail


static struct mag_init_info qmc6308_init_info = {
        .name = qmc6308_DEV_NAME,
        .init = qmc6308_local_init,
        .uninit = qmc6308_local_remove,
};



static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&read_i2c_xyz);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&read_i2c_xyz);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		MSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&read_i2c_xyz);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{	
	/*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&read_i2c_xyz);
	if (!client) {
		mutex_unlock(&read_i2c_xyz);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&read_i2c_xyz);
	return err;
}


static int I2C_RxData(char *rxData, int length)
{
	

	struct i2c_client *client = this_client;
	int res = 0;
	char addr = rxData[0];

	if ((rxData == NULL) || (length < 1))
		return -EINVAL;
	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
		return -1;
	return 0;

}

static int I2C_TxData(char *txData, int length)
{	

	struct i2c_client *client = this_client;
	int res = 0;
	char addr = txData[0];
	u8 *buff = &txData[1];

	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;

}


/* X,Y and Z-axis magnetometer data readout
 * param *mag pointer to \ref qmc6308_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
 */

static int qmc6308_read_mag_xyz(int *data)
{
	int res;
	unsigned char mag_data[6];
	int hw_d[3] = {0};

	int t1 = 0;
	unsigned char rdy = 0;
	struct i2c_client *client = this_client;
	struct qmc6308_i2c_data *clientdata = i2c_get_clientdata(client);

    MSE_FUN();

	/* Check status register for data availability */
	while(!(rdy & 0x01) && (t1 < 3)){
		rdy = QMC6308_STATUS_REG;
		res = I2C_RxData(&rdy,1);
		t1 ++;
		MSE_LOG("qmc6308 Status register is (%02X)\n", rdy);
	}

	mag_data[0] = QMC6308_DATA_OUT_X_LSB_REG;

	res = I2C_RxData(mag_data, 6);
	if(res != 0)
    {
		return -EFAULT;
	}
	
	MSE_LOG("qmc6308 mag_data[%02x, %02x, %02x, %02x, %02x, %02x]\n",
		mag_data[0], mag_data[1], mag_data[2],
		mag_data[3], mag_data[4], mag_data[5]);

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);

	//Unit:mG  1G = 100uT = 1000mG
	hw_d[0] = hw_d[0] * 1000 / clientdata->sensitivity;
	hw_d[1] = hw_d[1] * 1000 / clientdata->sensitivity;
	hw_d[2] = hw_d[2] * 1000 / clientdata->sensitivity;

	MSE_LOG("Hx=%d, Hy=%d, Hz=%d\n",hw_d[0],hw_d[1],hw_d[2]);

	data[qmc6308_AXIS_X] = clientdata->cvt.sign[qmc6308_AXIS_X]*hw_d[clientdata->cvt.map[qmc6308_AXIS_X]];
	data[qmc6308_AXIS_Y] = clientdata->cvt.sign[qmc6308_AXIS_Y]*hw_d[clientdata->cvt.map[qmc6308_AXIS_Y]];
	data[qmc6308_AXIS_Z] = clientdata->cvt.sign[qmc6308_AXIS_Z]*hw_d[clientdata->cvt.map[qmc6308_AXIS_Z]];

	MSE_LOG("qmc6308 data [%d, %d, %d]\n", data[0], data[1], data[2]);
	
	return res;
}


/* Set the sensor mode */
int qmc6308_set_mode(unsigned char mode)
{
	int err = 0;

	unsigned char data[2] = {0};
	
	data[0] = QMC6308_CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[1] = (data[0]& ~0x03) | mode;
	MSE_LOG("qmc6308_set_mode, 0x0A = [%02x]->[%02x]", data[0],data[1]);
	data[0] = QMC6308_CTL_REG_ONE;
	
	err = I2C_TxData(data, 2);

	return err;
}

int qmc6308_set_output_data_rate(unsigned char rate){
	
	int err = 0;

	unsigned char data[2] = {0};
	
	data[0] = QMC6308_CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[1] = (data[0]& ~0xE0) | (rate << 5);
	MSE_LOG("qmc6308_set_output_data_rate, 0x0A = [%02x]->[%02x]", data[0],data[1]);
	data[0] = QMC6308_CTL_REG_ONE;
	
	err = I2C_TxData(data, 2);

	return err;	
}

static int qmc6308_enable(struct i2c_client *client)
{

	unsigned char data[2];
	int ret;

	data[1] = 0x40;
	data[0] = 0x0d;
	ret = I2C_TxData(data, 2);

	data[1] = 0x08;
	data[0] = QMC6308_CTL_REG_TWO; //0x0b
	ret = I2C_TxData(data, 2);

/*
	data[1] = 0x63;
	data[0] = QMC6308_CTL_REG_ONE; //0x0a
	err = I2C_TxData(data, 2);
*/
	ret = qmc6308_set_output_data_rate(OUTPUT_DATA_RATE_200HZ);
	ret = qmc6308_set_mode(QMC6308_H_PFM_MODE);
	usleep_range(20000,30000); //fixit for amr ready
	
	return ret;
}

static int qmc6308_disable(struct i2c_client *client)
{
	int ret;
	
	ret = qmc6308_set_mode(QMC6308_SUSPEND_MODE);
	
	return ret;
}

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;

#ifdef	CONFIG_HAS_EARLYSUSPEND
static int qmc6308_SetPowerMode(struct i2c_client *client, bool enable)
{
	if(enable == true)
	{
		if(qmc6308_enable(client))
		{
			MSE_LOG("qmc6308: set power mode failed!\n");
			return -EINVAL;
		}
		else
		{
			MSE_LOG("qmc6308: set power mode enable ok!\n");
		}
	}
	else
	{
		if(qmc6308_disable(client))
		{
			MSE_LOG("qmc6308: set power mode failed!\n");
			return -EINVAL;
		}
		else
		{
			MSE_LOG("qmc6308: set power mode disable ok!\n");
		}
	}

	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static void qmc6308_power(struct mag_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;
	power_on = on;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	unsigned char databuf[1] = {0};
	int len = 0;
	int i,res;
	
	u8 reg[3] = {0x00, 0x38, 0x39};
	
	for(i = 0; i < 3; i++)
	{
		databuf[0] = reg[i];
		res = I2C_RxData(databuf, 1);
		
		if(res < 0)
		{
			MSE_LOG("qmc6308 read register 0x%02x failed !\n", i);
			len += snprintf(buf + len, PAGE_SIZE - len, "I2C error!\n");
		}
		else
		{			
			len += snprintf(buf + len, PAGE_SIZE - len , "reg[0x%2x]=0x%2x\n", i, databuf[0]);
		}		
	}
	
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int sensordata[3];

	qmc6308_read_mag_xyz(sensordata);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct qmc6308_i2c_data *data = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw.direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct qmc6308_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw.direction, &data->cvt))
		{
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw.direction);
		}
		else
		{
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw.direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct qmc6308_i2c_data *obj = i2c_get_clientdata(client);
	ssize_t len = 0;

	if(NULL == obj)
	{
		MSE_ERR("qmc6308_i2c_data is null!!\n");
		return -EINVAL;
	}
	
	//if(obj->hw)
	//{
		len += scnprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw.i2c_num, obj->hw.direction, obj->hw.power_id, obj->hw.power_vol);
	//}


	len += scnprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	len += scnprintf(buf+len, PAGE_SIZE-len, "open_flag = 0x%x, v_open_flag=0x%x\n",
			atomic_read(&open_flag), v_open_flag);
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct qmc6308_i2c_data *obj = i2c_get_clientdata(this_client);
	
	if(NULL == obj)
	{
		MSE_ERR("qmc6308_i2c_data is null!!\n");
		return -EINVAL;
	}

	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct qmc6308_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		MSE_ERR("qmc6308_i2c_data is null!!\n");
		return -EINVAL;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		MSE_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}

	return count;
}


static ssize_t store_registers_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int addr, value;
	u8 data[2] = {0};
	int res = 0;
	
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))
	{
		MSE_LOG("get para OK, addr = 0x%x value = 0x%x\n", addr, value);
		data[0] = (u8)addr;
		data[1] = (u8)value;

		res = I2C_TxData(data,2);
		if(res < 0)
		{
			MSE_ERR("store_registers_value 0x%02x fail\n", addr);
		}
	}
	else
	{
		MSE_ERR("store_registers_value get para error\n");
	}

	return count;
}

static ssize_t show_regiter_map(struct device_driver *ddri, char *buf)
{
	int res ;
	int i = 0;

	unsigned char databuf[2];
	int write_offset = 0;
	
	MSE_FUN();

	/* Check status register for data availability */	
	for(i = 0; i < 12; i++)
	{
		databuf[0] = i;
		res = I2C_RxData(databuf, 1);
		if(res < 0)
		{
			MSE_LOG("qmc6308 dump registers 0x%02x failed !\n", i);
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "I2C error!\n");
		}
		else
		{			
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "0x%2x=0x%2x\n", i, databuf[0]);
		}
	}

	return write_offset;
}

int FctShipmntTestProcess_Body(void)
{
	return 1;
}

static ssize_t store_shipment_test(struct device_driver * ddri,const char * buf, size_t count)
{
	return count;            
}

static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;
	res = FctShipmntTestProcess_Body();
	if(1 == res)
	{
	   MSE_LOG("shipment_test pass\n");
	   strcpy(result,"y");
	}
	else if(-1 == res)
	{
	   MSE_LOG("shipment_test fail\n");
	   strcpy(result,"n");
	}
	else
	{
	  MSE_LOG("shipment_test NaN\n");
	  strcpy(result,"NaN");
	}
	
	return sprintf(buf, "%s\n", result);        
}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
static DRIVER_ATTR(regmap,  S_IRUGO , show_regiter_map, NULL);
static DRIVER_ATTR(setreg,   S_IRUGO | S_IWUSR, NULL, store_registers_value);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *qmc6308_attr_list[] = {
	&driver_attr_shipmenttest,
	&driver_attr_regmap,
  &driver_attr_setreg,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
};
/*----------------------------------------------------------------------------*/
static int qmc6308_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(qmc6308_attr_list);
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, qmc6308_attr_list[idx]);
		if(err < 0)
		{
			MSE_ERR("driver_create_file (%s) = %d\n", qmc6308_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int qmc6308_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)ARRAY_SIZE(qmc6308_attr_list);

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, qmc6308_attr_list[idx]);
	}

	return 0;
}


/*----------------------------------------------------------------------------*/
static int qmc6308_open(struct inode *inode, struct file *file)
{
	struct qmc6308_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if(atomic_read(&obj->trace) & QMC_CTR_DEBUG)
	{
		MSE_LOG("Open device node:qmc6308\n");
	}
	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int qmc6308_release(struct inode *inode, struct file *file)
{
	struct qmc6308_i2c_data *obj = i2c_get_clientdata(this_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & QMC_CTR_DEBUG)
	{
		MSE_LOG("Release device node:qmc6308\n");
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static long qmc6308_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	//char buff[QMC6308_STR_BUFSIZE];				/* for chip information */
	char rwbuf[16]; 		/* for READ/WRITE */
	int ret =-1;				

	struct i2c_client *client = this_client;
	struct qmc6308_i2c_data *clientdata = i2c_get_clientdata(client);

	if ((clientdata != NULL) && (atomic_read(&clientdata->trace) & QMC_FUN_DEBUG))
		MSE_LOG("qmc6308_unlocked_ioctl !cmd= 0x%x\n", cmd);
	
	switch (cmd){
	case QMC_IOCTL_WRITE:
		if(argp == NULL)
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		if(copy_from_user(rwbuf, argp, sizeof(rwbuf)))
		{
			MSE_LOG("copy_from_user failed.");
			return -EFAULT;
		}

		if((rwbuf[0] < 2) || (rwbuf[0] > (QMC6308_RW_BUFSIZE-1)))
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		ret = I2C_TxData(&rwbuf[1], rwbuf[0]);
		if(ret < 0)
		{
			return ret;
		}
		break;
			
	case QMC_IOCTL_READ:
		if(argp == NULL)
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		
		if(copy_from_user(rwbuf, argp, sizeof(rwbuf)))
		{
			MSE_LOG("copy_from_user failed.");
			return -EFAULT;
		}

		if((rwbuf[0] < 1) || (rwbuf[0] > (QMC6308_RW_BUFSIZE-1)))
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		ret = I2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
		{
			return ret;
		}
		if(copy_to_user(argp, rwbuf, rwbuf[0]+1))
		{
			MSE_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	
	default:
		MSE_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
		return -ENOIOCTLCMD;
		break;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static struct file_operations qmc6308_fops = {
//	.owner = THIS_MODULE,
	.open = qmc6308_open,
	.release = qmc6308_release,
	.unlocked_ioctl = qmc6308_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice qmc6308_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "qst_msensor",
    .fops = &qmc6308_fops,
};
/*----------------------------------------------------------------------------*/

static int qmc6308_m_open_report_data(int en)
{
	return 0;
}
static int qmc6308_m_set_delay(u64 delay)
{
	int value = (int) delay / 1000 / 1000;
	struct qmc6308_i2c_data *data = NULL;

	if (unlikely(this_client == NULL)) {
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) {
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}

	if(value <= 10)
		qmcd_delay = 10;

	qmcd_delay = value;

	return 0;
}
static int qmc6308_m_enable(int en)
{
	struct qmc6308_i2c_data *data = NULL;

	if (unlikely(this_client == NULL)) 
	{
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) 
	{
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}

	if(en == 1)
	{
		v_open_flag |= 0x01;
		//we start measurement here.
		qmc6308_enable(this_client);
	}
	else
	{
		qmc6308_disable(this_client);
		v_open_flag &= 0x3e;
	}

	atomic_set(&open_flag, v_open_flag);
	
	MSE_ERR("qmc6308 v_open_flag = 0x%x,open_flag= 0x%x\n",v_open_flag, atomic_read(&open_flag));
	return 0;
}


static int qmc6308_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int qmc6308_flush(void)
{
	return mag_flush_report();
}

static int qmc6308_m_get_data(int *x, int *y, int *z, int *status)
{
	struct qmc6308_i2c_data *data = NULL;
	int mag[3];

	if (unlikely(this_client == NULL)) {
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) {
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}
	qmc6308_read_mag_xyz(mag);
	
	*x = mag[0];
	*y = mag[1];
	*z = mag[2];
	*status = 3;

#if DEBUG
	if(atomic_read(&data->trace) & QMC_HWM_DEBUG)
	{				
		MSE_LOG("Hwm get m-sensor data: %d, %d, %d,status %d!\n", *x, *y, *z, *status);
	}		
#endif

	return 0;
}
static int qmc6308_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	struct qmc6308_i2c_data *data = NULL;
	int en = (enabledisable == true ? 1 : 0);
	int err;
    
	if (unlikely(this_client == NULL)) 
	{
		MSE_LOG("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) 
	{
		MSE_LOG("data is null!\n");
		return -EINVAL;
	}

	if(en == 1)
	{
		v_open_flag |= 0x01;
		//we start measurement here.
		qmc6308_enable(this_client);
	}
	else
	{
		qmc6308_disable(this_client);
		v_open_flag &= 0x3e;
	}

	atomic_set(&open_flag, v_open_flag);
	
	MSE_LOG("qmc6308 v_open_flag = 0x%x,open_flag= 0x%x\n",v_open_flag, atomic_read(&open_flag));

	err = qmc6308_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MSE_LOG("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}

static int qmc6308_factory_get_data(int32_t data[3], int *status)
{
	int32_t factory_data[3] = {0};
	int ret = 0;
	
	/* get raw data */
	ret = qmc6308_m_get_data(&factory_data[0], &factory_data[1], &factory_data[2], status);
	
	data[0] = factory_data[0] / 10; //mG to uT
	data[1] = factory_data[1] / 10; //mG to uT
	data[2] = factory_data[2] / 10; //mG to uT
	return  ret;
}
static int qmc6308_factory_get_raw_data(int32_t data[3])
{
	MSE_LOG("do not support qmc6308_factory_get_raw_data!\n");
	return 0;
}
static int qmc6308_factory_enable_calibration(void)
{
	return 0;
}
static int qmc6308_factory_clear_cali(void)
{
	return 0;
}
static int qmc6308_factory_set_cali(int32_t data[3])
{
	return 0;
}
static int qmc6308_factory_get_cali(int32_t data[3])
{
	return 0;
}
static int qmc6308_factory_do_self_test(void)
{
	return 0;
}

static struct mag_factory_fops qmc6308_factory_fops = {
	.enable_sensor = qmc6308_factory_enable_sensor,
	.get_data = qmc6308_factory_get_data,
	.get_raw_data = qmc6308_factory_get_raw_data,
	.enable_calibration = qmc6308_factory_enable_calibration,
	.clear_cali = qmc6308_factory_clear_cali,
	.set_cali = qmc6308_factory_set_cali,
	.get_cali = qmc6308_factory_get_cali,
	.do_self_test = qmc6308_factory_do_self_test,
};

static struct mag_factory_public qmc6308_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &qmc6308_factory_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int qmc6308_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmc6308_i2c_data *obj = i2c_get_clientdata(client);
    
	qmc6308_power(&obj->hw, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int qmc6308_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmc6308_i2c_data *obj = i2c_get_clientdata(client);
	
	qmc6308_power(&obj->hw, 1);

	return 0;
}

static int qmc6308_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, qmc6308_DEV_NAME, sizeof(info->type));
	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void qmc6308_early_suspend(struct early_suspend *h)
{
	struct qmc6308_i2c_data *obj = container_of(h, struct qmc6308_i2c_data, early_drv);

	if(NULL == obj)
	{
		MSE_ERR("null pointer!!\n");
		return;
	}
	
	qmc6308_power(&obj->hw, 0);
	
	if(qmc6308_SetPowerMode(obj->client, false))
	{
		MSE_LOG("qmc6308: write power control fail!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
static void qmc6308_late_resume(struct early_suspend *h)
{
	struct qmc6308_i2c_data *obj = container_of(h, struct qmc6308_i2c_data, early_drv);

	if(NULL == obj)
	{
		MSE_ERR("null pointer!!\n");
		return;
	}

	qmc6308_power(&obj->hw, 1);
	
	/// we can not start measurement , because we have no single measurement mode 

}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

static int qmc6308_device_check(void){
	unsigned char databuf[2] = {0};
	int ret = 0; 	
	
	databuf[0] = 0x00;
	ret = I2C_RxData(databuf, 1);
	if(ret < 0){
		MSE_ERR("%s: I2C_RxData failed\n",__func__);
		return ret;
	}

	if((databuf[0] & mask.mask1) != mask.mask1 ||
		(databuf[0] & mask.mask2) != mask.mask2 ||
		(databuf[0] & mask.mask3) != mask.mask3 ||
		(databuf[0] & mask.mask0) != mask.mask0)
	{
		ret = -1;	
	}
	return ret;
}
/*----------------------------------------------------------------------------*/
static int qmc6308_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct qmc6308_i2c_data *data = NULL;
    const char lib_name[64] = "calmodule_sensor";

	int err = 0;

	struct mag_control_path mag_ctl = {0};
	struct mag_data_path mag_data = {0};

	MSE_FUN();

	data = kmalloc(sizeof(struct qmc6308_i2c_data), GFP_KERNEL);
	if(data == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}

	err = get_mag_dts_func(client->dev.of_node, &data->hw);
	if (err < 0) {
		MSE_ERR("%s. get dts info fail\n",__FUNCTION__);
		err = -EFAULT;
		goto exit_kfree;
	}

	MSE_LOG("%s: direction: %d\n",__FUNCTION__,data->hw.direction);
	client->addr = 0x2c;

	err = hwmsen_get_convert(data->hw.direction, &data->cvt);	
	if (err) {
		MSE_ERR("qmc6308 invalid direction: %d\n", data->hw.direction);
		goto exit_kfree;
	}
	
	atomic_set(&data->layout, data->hw.direction);
	
	atomic_set(&data->trace, 0);


	//mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;

	err = qmc6308_device_check();
	if(err < 0)
	{
		MSE_LOG("%s check device faild!\n",__FUNCTION__);
		goto exit_kfree;
	}

	err = qmc6308_create_attr(&(qmc6308_init_info.platform_diver_addr->driver));

	if (err < 0)
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = misc_register(&qmc6308_device);

	if(err < 0)
	{
		MSE_ERR("qmc6308_device register failed\n");
		goto exit_misc_device_register_failed;	
	}
	
	err = mag_factory_device_register(&qmc6308_factory_device);
	if (err) {
		MSE_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_factory_device_register_failed;
	}
	
	mag_ctl.enable = qmc6308_m_enable;
	mag_ctl.set_delay = qmc6308_m_set_delay;
	mag_ctl.open_report_data = qmc6308_m_open_report_data;
	mag_ctl.is_report_input_direct = false;
	mag_ctl.is_support_batch = data->hw.is_batch_supported;
	mag_ctl.batch = qmc6308_batch;
	mag_ctl.flush = qmc6308_flush;
	mag_ctl.libinfo.deviceid = 0x80;
	mag_ctl.libinfo.layout = data->hw.direction;
  memcpy(mag_ctl.libinfo.libname,lib_name,sizeof(lib_name));
	err = mag_register_control_path(&mag_ctl);
	if (err) {
		MAG_PR_ERR("register mag control path err\n");
		goto exit_hwm_attach_failed;
	}
	mag_data.div = CONVERT_M_DIV;
	mag_data.get_data = qmc6308_m_get_data;
	err = mag_register_data_path(&mag_data);
	if (err){
		MAG_PR_ERR("register data control path err\n");
		goto exit_hwm_attach_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = qmc6308_early_suspend,
	data->early_drv.resume   = qmc6308_late_resume,
	register_early_suspend(&data->early_drv);
#endif
	
    qmc6308_init_flag = 0;

	MSE_LOG("%s: OK\n", __func__);
	return 0;

exit_hwm_attach_failed:
	mag_factory_device_deregister(&qmc6308_factory_device);
exit_misc_factory_device_register_failed:
	misc_deregister(&qmc6308_device);
exit_misc_device_register_failed:
	qmc6308_delete_attr(&(qmc6308_init_info.platform_diver_addr->driver));
exit_sysfs_create_group_failed:
exit_kfree:
	kfree(data);
exit:
	MSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/



static int qmc6308_i2c_remove(struct i2c_client *client)
{
	int err;

	err = qmc6308_delete_attr(&(qmc6308_init_info.platform_diver_addr->driver));

	if (err < 0)
	{
		MSE_ERR("qmc6308_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&qmc6308_device);
	mag_factory_device_deregister(&qmc6308_factory_device);
	return 0;
}

static int qmc6308_local_init(void)
{


	qmc6308_power(qst_hw, 1);

	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&qmc6308_i2c_driver))
	{
		MSE_ERR("add driver error\n");
		return -EINVAL;
	}

    if(-1 == qmc6308_init_flag)
    {
        MSE_ERR("%s failed!\n",__func__);
        return -EINVAL;
    }

	return 0;
}


static int qmc6308_local_remove(void)
{

	qmc6308_power(qst_hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&qmc6308_i2c_driver);
	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init qmc6308_init(void)
{
  
	mag_driver_add(&qmc6308_init_info);
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit qmc6308_exit(void)
{

}
/*----------------------------------------------------------------------------*/
module_init(qmc6308_init);
module_exit(qmc6308_exit);

MODULE_AUTHOR("QST Corp");
MODULE_DESCRIPTION("qmc6308 compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

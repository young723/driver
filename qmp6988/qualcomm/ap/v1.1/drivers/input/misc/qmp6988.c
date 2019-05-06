/* QST Pressure Sensor Driver
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
 * History: V1.0 --- [2016.08.08]Driver creation
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
//#include <linux/math64.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>

//#include <linux/qmp6988.h>
#include "qmp6988.h"

#define C_I2C_FIFO_SIZE  8
#define QMP_DUGUG
#if defined(QMP_DUGUG)
#define QMP_TAG                  "[QMP6988] "
#define QMP_FUN(f)               printk(QMP_TAG"%s\n", __func__)
#define QMP_ERR(fmt, args...)	 printk(QMP_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define QMP_LOG(fmt, args...)    printk(QMP_TAG fmt, ##args)
#else
#define QMP_FUN(f)
#define QMP_ERR(fmt, args...)
#define QMP_LOG(fmt, args...)
#endif

static int qmp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int qmp_i2c_remove(struct i2c_client *client);
static int qmp_i2c_detect(struct i2c_client *client,struct i2c_board_info *info);
static int qmp_suspend(struct i2c_client *client, pm_message_t msg);
static int qmp_resume(struct i2c_client *client);

/* qmp i2c client data */
struct qmp_i2c_data {
	struct	sensors_classdev cdev;
	struct i2c_client *client;	
	u32	power_enabled;
	enum SENSOR_TYPE_ENUM sensor_type;
	enum QMP_POWERMODE_ENUM power_mode;
	enum QMP_FILTER_ENUM hw_filter;
	enum QMP_OVERSAMPLING_ENUM oversampling_p;
	enum QMP_OVERSAMPLING_ENUM oversampling_t;
	struct qmp6988_calibration_data qmp6988_cali;
	struct input_dev *input;
	struct delayed_work work;
	s32 delay;
	s32 enable;
	s32 pressure;
	s32 temperature;

	struct mutex lock;
	atomic_t trace;
	atomic_t suspend;
	atomic_t filter;

#if defined(CONFIG_QMP_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
};

struct qmp_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct qmp_regulator qmp_vreg[] = {
	{NULL, "vdd", 2850000, 2850000},
	{NULL, "vddio", 1800000, 1800000},
};


static struct sensors_classdev pressure_sensors_cdev = {
	.name = "qmp6988",
	.vendor = "QST",
	.version = 1,
	.handle = SENSORS_PRESSURE_HANDLE,
	.type = SENSOR_TYPE_PRESSURE,
	.max_range = "1100.0",
	.resolution = "0.01",
	.sensor_power = "0.67",
	.min_delay = 20000,	/* us */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,	/* millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


//static struct i2c_board_info __initdata i2c_qmp6988={ I2C_BOARD_INFO(QMP_DEV_NAME, QMP6988_I2C_ADDRESS)};

static struct qmp_i2c_data *g_qmp = NULL;
static const struct i2c_device_id qmp_i2c_id[] = {
	{QMP_DEV_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id qmp_of_match[] = {
	{.compatible = "qst,qmp6988"},
	{},
};
#endif

static struct i2c_driver qmp_i2c_driver = {
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	QMP_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = qmp_of_match,
#endif
	},
	.probe = qmp_i2c_probe,
	.remove	= qmp_i2c_remove,
	.detect	= qmp_i2c_detect,
	.suspend = qmp_suspend,
	.resume	= qmp_resume,
	.id_table =	qmp_i2c_id,
};

//static DECLARE_WAIT_QUEUE_HEAD(open_wq);
static int qmp_init_flag =  -1;

/* I2C operation functions */
static int qmp_i2c_read_block(struct i2c_client *client,u8 addr, u8 *buf, u8 len)
{
	return i2c_smbus_read_i2c_block_data(client, addr, len, buf);
}

static int qmp_i2c_write_block(struct i2c_client *client, u8 addr, u8 *buf, u8 len)
{
	return i2c_smbus_write_i2c_block_data(client, addr, len, buf);
	//return i2c_smbus_write_byte_data(client, addr, buf[0]);
}

static int qmp_config_regulator(struct i2c_client *client, bool on)
{
	int rc = 0, i;
	int num_vreg = ARRAY_SIZE(qmp_vreg);

	if(on)
	{
		for(i = 0; i < num_vreg; i++)
		{
			qmp_vreg[i].vreg = regulator_get(&client->dev,qmp_vreg[i].name);
			if(IS_ERR(qmp_vreg[i].vreg))
			{
				rc = PTR_ERR(qmp_vreg[i].vreg);
				QMP_ERR("%s:regulator get failed rc=%d\n",__func__, rc);
				qmp_vreg[i].vreg = NULL;
				goto error_vdd;
			}
			if(regulator_count_voltages(qmp_vreg[i].vreg) > 0) 
			{
				rc = regulator_set_voltage(qmp_vreg[i].vreg,qmp_vreg[i].min_uV, qmp_vreg[i].max_uV);
				if(rc) 
				{
					QMP_ERR("%s:set_voltage failed rc=%d\n",__func__, rc);
					regulator_put(qmp_vreg[i].vreg);
					qmp_vreg[i].vreg = NULL;
					goto error_vdd;
				}
			}
			rc = regulator_enable(qmp_vreg[i].vreg);
			if(rc)
			{
				QMP_ERR("%s: regulator_enable failed rc =%d\n",__func__, rc);
				if(regulator_count_voltages(qmp_vreg[i].vreg)> 0)
				{
					regulator_set_voltage(qmp_vreg[i].vreg,0, qmp_vreg[i].max_uV);
				}
				regulator_put(qmp_vreg[i].vreg);
				qmp_vreg[i].vreg = NULL;
				goto error_vdd;
			}
		}
		return rc;
	} 
	else
	{
		i = num_vreg;
	}
error_vdd:
	while(--i >= 0)
	{
		if(!IS_ERR_OR_NULL(qmp_vreg[i].vreg))
		{
			if(regulator_count_voltages(qmp_vreg[i].vreg) > 0) 
			{
				regulator_set_voltage(qmp_vreg[i].vreg, 0, qmp_vreg[i].max_uV);
			}
			regulator_disable(qmp_vreg[i].vreg);
			regulator_put(qmp_vreg[i].vreg);
			qmp_vreg[i].vreg = NULL;
		}
	}
	return rc;
}

static void qmp_init_hw(struct i2c_client * client)
{
	int ret = 0;
	if(client)
	{
		ret = qmp_config_regulator(client, true);
		usleep_range(15000, 20000);
	}
}

static void qmp_deinit_hw(struct i2c_client * client)
{
	int ret = 0;
	if(client)
	{
		ret = qmp_config_regulator(client, false);
	}
}

static int qmp_set_power(int on)
{
	int rc = 0;
	int num_vreg = ARRAY_SIZE(qmp_vreg);
	int i;

	if(!on && g_qmp->power_enabled)
	{
		for(i = 0; i < num_vreg; i++)
		{
			rc = regulator_disable(qmp_vreg[i].vreg);
			if(rc) 
			{
				QMP_ERR("Regulator vdd disable failed rc=%d\n",rc);
				return rc;
			}
		}
		g_qmp->power_enabled = 0;
	}
	else if(on && !g_qmp->power_enabled)
	{
		for(i = 0; i < num_vreg; i++)
		{
			rc = regulator_enable(qmp_vreg[i].vreg);
			if(rc)
			{
				QMP_ERR("Regulator vdd enable failed rc=%d\n",rc);
				return rc;
			}
		}
		/* The minimum start up time of bmp18x is 10ms */
		usleep_range(15000, 20000);
		g_qmp->power_enabled = 1;
	}
	else
	{
		QMP_LOG("Power on=%d. enabled=%d\n",on, g_qmp->power_enabled);
	}

	return rc;
}

#ifdef CONFIG_OF
static int qmp_parse_dt(struct device *dev)
{
#if 0
	int ret = 0;
	u32 val;
#if 0
	ret = of_property_read_u32(dev->of_node, "qst,chip-id", &val);
	if(ret)
	{
		QMP_ERR("no chip_id from dt\n");
		return ret;
	}
	g_qmp->chip_id = (u8)val;
#endif
	ret = of_property_read_u32(dev->of_node, "qst,oversample_p", &val);
	if(ret)
	{
		QMP_ERR("no default_oversampling_p from dt\n");
		return ret;
	}
	g_qmp->oversampling_p = (u8)val;

	ret = of_property_read_u32(dev->of_node, "qst,oversample_t", &val);
	if(ret)
	{
		QMP_ERR("no default_oversampling_p from dt\n");
		return ret;
	}

	//g_qmp->default_sw_oversampling = of_property_read_bool(dev->of_node,"qst,sw-oversample");
#endif
	return 0;
}
#else
static int qmp_parse_dt(struct device *dev)
{
	return 0;
}
#endif

/* get chip type */
static int qmp_get_chip_type(void)
{
	s32 err = 0;
	u8 chip_id = 0;
	//struct qmp_i2c_data *obj = i2c_get_clientdata(client);

	err = qmp_i2c_read_block(g_qmp->client, QMP_CHIP_ID_REG, &chip_id, 0x01);
	QMP_LOG("qmp6988 chip_id=%x", chip_id);
	if (err != 0)
		return err;

	switch (chip_id) {
	case QMP6988_CHIP_ID:
		g_qmp->sensor_type = QMP6988_TYPE;
		break;
	default:
		g_qmp->sensor_type = INVALID_TYPE;
		break;
	}

	QMP_LOG("chip id = 0x%x,\n",chip_id);

	if (g_qmp->sensor_type == INVALID_TYPE) {
		QMP_ERR("unknown pressure sensor\n");
		return -1;
	}
	return 0;
}


static int qmp_get_calibration_data(void)
{
	s32 status = 0;
	u8 a_data_u8r[QMP6988_CALIBRATION_DATA_LENGTH] = {0};
	
	if (g_qmp->sensor_type == QMP6988_TYPE) {
		status = qmp_i2c_read_block(g_qmp->client,QMP6988_CALIBRATION_DATA_START,a_data_u8r,QMP6988_CALIBRATION_DATA_LENGTH);

		if (status < 0)
			return status;

		g_qmp->qmp6988_cali.COE_a0 = (QMP6988_S32_t)((((QMP6988_U32_t)a_data_u8r[18] << SHIFT_LEFT_12_POSITION) \
			| ((QMP6988_U32_t)a_data_u8r[19] << SHIFT_LEFT_4_POSITION) \
			| ((QMP6988_U32_t)a_data_u8r[24] & 0x0f))<<12);
		g_qmp->qmp6988_cali.COE_a0 = g_qmp->qmp6988_cali.COE_a0>>12;
		
		g_qmp->qmp6988_cali.COE_a1 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[20]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[21]);
		g_qmp->qmp6988_cali.COE_a2 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[22]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[23]);
		
		g_qmp->qmp6988_cali.COE_b00 = (QMP6988_S32_t)((((QMP6988_U32_t)a_data_u8r[0] << SHIFT_LEFT_12_POSITION) \
			| ((QMP6988_U32_t)a_data_u8r[1] << SHIFT_LEFT_4_POSITION) \
			| (((QMP6988_U32_t)a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))<<12);
		g_qmp->qmp6988_cali.COE_b00 = g_qmp->qmp6988_cali.COE_b00>>12;
		
		g_qmp->qmp6988_cali.COE_bt1 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[2]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[3]);
		g_qmp->qmp6988_cali.COE_bt2 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[4]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[5]);
		g_qmp->qmp6988_cali.COE_bp1 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[6]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[7]);
		g_qmp->qmp6988_cali.COE_b11 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[8]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[9]);
		g_qmp->qmp6988_cali.COE_bp2 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[10]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[11]);
		g_qmp->qmp6988_cali.COE_b12 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[12]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[13]);		
		g_qmp->qmp6988_cali.COE_b21 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[14]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[15]);
		g_qmp->qmp6988_cali.COE_bp3 = (QMP6988_S16_t)(((QMP6988_U16_t)(a_data_u8r[16]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[17]);			

		QMP_LOG("<-----------calibration data-------------->\n");
		QMP_LOG("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
				g_qmp->qmp6988_cali.COE_a0,g_qmp->qmp6988_cali.COE_a1,g_qmp->qmp6988_cali.COE_a2,g_qmp->qmp6988_cali.COE_b00);
		QMP_LOG("COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\n",
				g_qmp->qmp6988_cali.COE_bt1,g_qmp->qmp6988_cali.COE_bt2,g_qmp->qmp6988_cali.COE_bp1,g_qmp->qmp6988_cali.COE_b11);
		QMP_LOG("COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\n",
				g_qmp->qmp6988_cali.COE_bp2,g_qmp->qmp6988_cali.COE_b12,g_qmp->qmp6988_cali.COE_b21,g_qmp->qmp6988_cali.COE_bp3);
		QMP_LOG("<-----------calibration data-------------->\n");
	}
	return 0;
}

static int qmp_set_powermode(enum QMP_POWERMODE_ENUM power_mode)
{
	u8 err = 0, data = 0, actual_power_mode = 0;

	QMP_LOG("qmp_set_powermode  power_mode=%d \n", power_mode); 

	//if(power_mode == g_qmp->power_mode)
	//	return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE) {
		if(power_mode == QMP_SUSPEND_MODE)
		{
			actual_power_mode = QMP6988_SLEEP_MODE;
		}		
		else if(power_mode == QMP_FORCED_MODE)
		{
			actual_power_mode = QMP6988_FORCED_MODE;
		}
		else if(power_mode == QMP_NORMAL_MODE)
		{
			actual_power_mode = QMP6988_NORMAL_MODE;
		}
		else 
		{
			err = -EINVAL;
			QMP_ERR("invalid power mode = %d\n", power_mode);
			return err;
		}
		err = qmp_i2c_read_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,QMP6988_CTRLMEAS_REG_MODE, actual_power_mode);
		err += qmp_i2c_write_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
	}
 
	if(err < 0)
		QMP_ERR("set power mode failed, err = %d\n", err);
	else
		g_qmp->power_mode = power_mode;

 	return err;
}

static int qmp_set_filter(enum QMP_FILTER_ENUM filter)
{
	u8 err = 0, data = 0, actual_filter = 0;

	QMP_LOG("hw filter = %d, old hw filter = %d\n", filter, g_qmp->hw_filter);
	//if(filter == g_qmp->hw_filter)
	//	return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE) {
		if(filter == QMP_FILTER_OFF)
			actual_filter = QMP6988_FILTERCOEFF_OFF;
		else if(filter == QMP_FILTER_2)
			actual_filter = QMP6988_FILTERCOEFF_2;
		else if(filter == QMP_FILTER_4)
			actual_filter = QMP6988_FILTERCOEFF_4;
		else if(filter == QMP_FILTER_8)
			actual_filter = QMP6988_FILTERCOEFF_8;
		else if(filter == QMP_FILTER_16)
			actual_filter = QMP6988_FILTERCOEFF_16;
		else if(filter == QMP_FILTER_32)
			actual_filter = QMP6988_FILTERCOEFF_32;
		else {
			err = -EINVAL;
			QMP_ERR("invalid hw filter = %d\n", filter);
			return err;
		}
		err = qmp_i2c_read_block(g_qmp->client,QMP6988_CONFIG_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,QMP6988_CONFIG_REG_FILTER, actual_filter);
		err += qmp_i2c_write_block(g_qmp->client,QMP6988_CONFIG_REG, &data, 1);
	}

	if(err < 0)
		QMP_ERR("set hw filter failed, err = %d\n",err);
	else
		g_qmp->hw_filter = filter;

	return err;
}

static int qmp_set_oversampling_p(enum QMP_OVERSAMPLING_ENUM oversampling_p)
{
	u8 err = 0, data = 0, actual_oversampling_p = 0;

		QMP_LOG("oversampling_p = %d, old oversampling_p = %d\n",oversampling_p, g_qmp->oversampling_p);

	//if(oversampling_p == g_qmp->oversampling_p)
	//	return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE) {
		if (oversampling_p == QMP_OVERSAMPLING_SKIPPED)
			actual_oversampling_p = QMP6988_OVERSAMPLING_SKIPPED;
		else if (oversampling_p == QMP_OVERSAMPLING_1X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_1X;
		else if (oversampling_p == QMP_OVERSAMPLING_2X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_2X;
		else if (oversampling_p == QMP_OVERSAMPLING_4X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_4X;
		else if (oversampling_p == QMP_OVERSAMPLING_8X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_8X;
		else if (oversampling_p == QMP_OVERSAMPLING_16X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_16X;
		else if (oversampling_p == QMP_OVERSAMPLING_32X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_32X;
		else if (oversampling_p == QMP_OVERSAMPLING_64X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_64X;
		else {
			err = -EINVAL;
			QMP_ERR("invalid oversampling_p = %d\n",oversampling_p);
			return err;
		}
		err = qmp_i2c_read_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,QMP6988_CTRLMEAS_REG_OSRSP, actual_oversampling_p);
		err += qmp_i2c_write_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
	}

	if(err < 0)
		QMP_ERR("set pressure oversampling failed, err = %d \n", err);
	else
		g_qmp->oversampling_p = oversampling_p;

	return err;
}

static int qmp_set_oversampling_t(enum QMP_OVERSAMPLING_ENUM oversampling_t)
{
	u8 err = 0, data = 0, actual_oversampling_t = 0;

	QMP_LOG("oversampling_t = %d, old oversampling_t = %d\n", oversampling_t, g_qmp->oversampling_t);

	//if(oversampling_t == g_qmp->oversampling_t)
	//	return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE) {
		if (oversampling_t == QMP_OVERSAMPLING_SKIPPED)
			actual_oversampling_t = QMP6988_OVERSAMPLING_SKIPPED;
		else if (oversampling_t == QMP_OVERSAMPLING_1X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_1X;
		else if (oversampling_t == QMP_OVERSAMPLING_2X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_2X;
		else if (oversampling_t == QMP_OVERSAMPLING_4X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_4X;
		else if (oversampling_t == QMP_OVERSAMPLING_8X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_8X;
		else if (oversampling_t == QMP_OVERSAMPLING_16X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_16X;
		else if (oversampling_t == QMP_OVERSAMPLING_16X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_32X;
		else if (oversampling_t == QMP_OVERSAMPLING_16X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_64X;
		else {
			err = -EINVAL;
			QMP_ERR("invalid oversampling_t = %d\n",oversampling_t);
			return err;
		}
		err = qmp_i2c_read_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,QMP6988_CTRLMEAS_REG_OSRST, actual_oversampling_t);
		err += qmp_i2c_write_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
	}

	if (err < 0)
		QMP_ERR("set temperature oversampling failed, err = %d \n", err);
	else
		g_qmp->oversampling_t = oversampling_t;

	return err;
}

static int qmp_read_raw_data(s32 *pressure, s32 *temp)
{
	s32 err = 0;
	unsigned char a_data_u8r[6] = {0};
	QMP6988_U32_t Praw, Traw;

	if(g_qmp->sensor_type == QMP6988_TYPE) 
	{
		err = qmp_i2c_read_block(g_qmp->client,QMP6988_PRESSURE_MSB_REG, a_data_u8r, 6);

		if(err < 0)
		{
			QMP_ERR("read raw pressure failed, err = %d\n", err);
			return err;
		}
		Praw = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_u8r[0])) << SHIFT_LEFT_16_POSITION) 
		|(((QMP6988_U16_t)(a_data_u8r[1])) << SHIFT_LEFT_8_POSITION)
		|(a_data_u8r[2]));
		*pressure = (QMP6988_S32_t)(Praw - SUBTRACTOR);

		Traw = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_u8r[3])) << SHIFT_LEFT_16_POSITION)
			|(((QMP6988_U16_t)(a_data_u8r[4])) << SHIFT_LEFT_8_POSITION)
			| (a_data_u8r[5]));
		*temp = (QMP6988_S32_t)(Traw - SUBTRACTOR);
		
		QMP_LOG("Praw[%d]	pressure[%d]\n", Praw,*pressure);
		QMP_LOG("Traw[%d]	tempearture[%d]\n", Traw,*temp);
		if(g_qmp->power_mode==QMP_FORCED_MODE)
		{
			qmp_set_powermode(QMP_FORCED_MODE);
		}
	}
#ifdef CONFIG_QMP_LOWPASS
/*
*Example: firlen = 16, filter buffer = [0] ... [15],
*when 17th data come, replace [0] with this new data.
*Then, average this filter buffer and report average value to upper layer.
*/
	if (atomic_read(&g_qmp->filter)) {
		if (atomic_read(&g_qmp->fir_en) &&
			!atomic_read(&g_qmp->suspend)) {
			int idx, firlen = atomic_read(&g_qmp->firlen);

			if (g_qmp->fir.num < firlen) {
				g_qmp->fir.raw[g_qmp->fir.num][QMP_PRESSURE] =
					*pressure;
				g_qmp->fir.sum[QMP_PRESSURE] += *pressure;
				if (atomic_read(&g_qmp->trace) &
					BAR_TRC_FILTER) {
					QMP_LOG("add [%2d] [%5d] => [%5d]\n",
					g_qmp->fir.num,
					g_qmp->fir.raw
					[g_qmp->fir.num][QMP_PRESSURE],
					g_qmp->fir.sum[QMP_PRESSURE]);
				}
				g_qmp->fir.num++;
				g_qmp->fir.idx++;
			} else {
				idx = g_qmp->fir.idx % firlen;
				g_qmp->fir.sum[QMP_PRESSURE] -=
					g_qmp->fir.raw[idx][QMP_PRESSURE];
				g_qmp->fir.raw[idx][QMP_PRESSURE] = *pressure;
				g_qmp->fir.sum[QMP_PRESSURE] += *pressure;
				g_qmp->fir.idx++;
				*pressure = g_qmp->fir.sum[QMP_PRESSURE]/firlen;
				if (atomic_read(&g_qmp->trace) &
					BAR_TRC_FILTER) {
					QMP_LOG("add [%2d][%5d]=>[%5d]:[%5d]\n",
					idx,
					g_qmp->fir.raw[idx][QMP_PRESSURE],
					g_qmp->fir.sum[QMP_PRESSURE],
					*pressure);
				}
			}
		}
	}
#endif
	return err;
}


static void qmp_measure_control(s32 enable)
{		
	if(enable)
	{
		qmp_set_powermode(QMP_NORMAL_MODE);
		if(g_qmp->delay > 1)
			schedule_delayed_work(&g_qmp->work, msecs_to_jiffies(g_qmp->delay));
	}
	else
	{
		qmp_set_powermode(QMP_SUSPEND_MODE);
		cancel_delayed_work_sync(&g_qmp->work);
	}
}


/* qmp setting initialization */
static int qmp_init_client(struct i2c_client *client)
{
	int err = 0;

	QMP_FUN();

	err = qmp_get_chip_type();
	if (err < 0) {
		QMP_ERR("get chip type failed, err = %d\n", err);
		return err;
	}

	err = qmp_get_calibration_data();
	if (err < 0) {
		QMP_ERR("get calibration data failed, err = %d\n", err);
		return err;
	}
	err = qmp_set_powermode(QMP_SUSPEND_MODE);
	if (err < 0) {
		QMP_ERR("set power mode failed, err = %d\n", err);
		return err;
	}

	err = qmp_set_filter(QMP_FILTER_OFF);
	if (err < 0) {
		QMP_ERR("set hw filter failed, err = %d\n", err);
		return err;
	}

	err = qmp_set_oversampling_p(QMP_OVERSAMPLING_16X);
	if (err < 0) {
		QMP_ERR("set pressure oversampling failed, err = %d\n", err);
		return err;
	}

	err = qmp_set_oversampling_t(QMP_OVERSAMPLING_2X);
	if (err < 0) {
		QMP_ERR("set temperature oversampling failed, err = %d\n", err);
		return err;
	}

	return 0;
}

static int qmp_open(struct inode *inode, struct file *file)
{
	file->private_data = g_qmp;

	if (file->private_data == NULL) {
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int qmp_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long qmp_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	struct qmp_i2c_data *obj = g_qmp;	//(struct qmp_i2c_data *)file->private_data;
	//struct i2c_client *client = obj->client;
	//s8 strbuf[QMP_BUFSIZE];
	s32 dat_buf[2];
	void __user *data;
	s32 delay,status;
	s32 err = 0;

	QMP_LOG("qmp6988_unlocked_ioctl cmd=%d \n", cmd);
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,(void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		QMP_ERR("access error: %08X, (%2d, %2d)\n",cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case BAROMETER_GET_RAW_DATA:
		data = (void __user *)arg;
		if(NULL == data) {
			err = -EINVAL;
			break;
		}
		qmp_read_raw_data(&dat_buf[0],&dat_buf[1]);
		if(copy_to_user(data, dat_buf, sizeof(dat_buf))) {
			err = -EFAULT;
			break;
		}	
		break;
	
	case BAROMETER_GET_CALIBRATION_DATA:
		data = (void __user *)arg;
		if(NULL == data) {
			err = -EINVAL;
			break;
		}
		QMP_LOG("get cali : %d %d %d \n", g_qmp->qmp6988_cali.COE_a0,g_qmp->qmp6988_cali.COE_a1,g_qmp->qmp6988_cali.COE_a2);
		if(copy_to_user(data, &g_qmp->qmp6988_cali, sizeof(struct qmp6988_calibration_data))) {
			err = -EFAULT;
			break;
		}	
		break;

	case BAROMETER_SET_CALC_DATA:
		data = (void __user *)arg;
		if(NULL == data) {
			err = -EINVAL;
			break;
		}
		if(copy_from_user(dat_buf, data, sizeof(dat_buf))){
			err = -EFAULT;
			break;
		}
		obj->pressure = dat_buf[0];
		obj->temperature = dat_buf[1];
		break;	

	case QMP_IOCTL_GET_STATUS:
		data = (void __user *)arg;
		status = obj->enable;
		if(copy_to_user(data, &status, sizeof(status)))
		{
			QMP_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
		
	case QMP_IOCTL_GET_DELAY:
		data = (void __user *)arg;
	    delay = obj->delay;
	    if(copy_to_user(data, &delay, sizeof(delay))) {
	         QMP_LOG("copy_to_user failed.");
	         return -EFAULT;
	    }
	    break;
	default:
		QMP_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

#if 0//IS_ENABLED(CONFIG_COMPAT)
static long compat_qmp6988_unlocked_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{

	QMP_FUN();

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		QMP_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_BAROMETER_IOCTL_INIT:
	case COMPAT_BAROMETER_IOCTL_READ_CHIPINFO:
	case COMPAT_BAROMETER_GET_PRESS_DATA:
	case COMPAT_BAROMETER_GET_TEMP_DATA: {
		QMP_LOG("compat_ion_ioctl : BAROMETER_IOCTL_XXX command is 0x%x\n", cmd);
		return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
	}
	default:
		QMP_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif

/*----------------------------------------------------------------------------*/
static const struct file_operations qmp_fops = {
	.owner = THIS_MODULE,
	.open = qmp_open,
	.release = qmp_release,
	.unlocked_ioctl = qmp_unlocked_ioctl,
#if 0//IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_qmp6988_unlocked_ioctl,
#endif
};

static struct miscdevice qmp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "qmp6988_misc",
	.fops = &qmp_fops,
};

static int qmp_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	QMP_FUN();
	if(NULL == obj) {
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}

	if(msg.event == PM_EVENT_SUSPEND) {
		atomic_set(&obj->suspend, 1);
		qmp_measure_control(0);	
		qmp_set_power(0);
	}
	return err;
}

static int qmp_resume(struct i2c_client *client)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	QMP_FUN();
	if(NULL == obj) {
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}

	qmp_set_power(1);
	err = qmp_init_client(obj->client);
	if(err) {
		QMP_ERR("initialize client fail\n");
		return err;
	}
	qmp_measure_control(g_qmp->enable);
#ifdef CONFIG_QMP_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif
	atomic_set(&obj->suspend, 0);
	return 0;
}

static int qmp_i2c_detect(struct i2c_client *client,struct i2c_board_info *info)
{
	strncpy(info->type, QMP_DEV_NAME, sizeof(info->type));
	return 0;
}

static int qmp_input_init(struct qmp_i2c_data *data)
{
	struct input_dev *dev;
	s32 err=0;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = QMP_DEV_NAME;
	dev->id.bustype = BUS_I2C;

	//input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_MISC);	
	input_set_abs_params(dev, ABS_X, -2147483648, 2147483648, 0, 0);
	input_set_abs_params(dev, ABS_Y, -2147483648, 2147483648, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if(err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input = dev;

	return 0;
}


static void qmp_input_delete(struct qmp_i2c_data *data)
{
	struct input_dev *dev = data->input;

	if(dev)
	{
		input_unregister_device(dev);
		input_free_device(dev);
	}
}

static void qmp_work_func(struct work_struct *work)
{
	//struct qmp_i2c_data *client_data = container_of((struct delayed_work *)work,struct qmp_i2c_data, work);
	s32 err = 0;

	err = qmp_read_raw_data(&g_qmp->pressure, &g_qmp->temperature);
	if(err == 0)
	{
		input_event(g_qmp->input, EV_ABS, ABS_X, g_qmp->pressure);
		input_event(g_qmp->input, EV_ABS, ABS_Y, g_qmp->temperature);
		input_sync(g_qmp->input);
	}
	if(g_qmp->delay > 1)
	{
		schedule_delayed_work(&g_qmp->work, msecs_to_jiffies(g_qmp->delay));
	}
}

static ssize_t show_delay(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_qmp->delay);
}

static ssize_t store_delay(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long delay;
	s32 status = kstrtoul(buf, 10, &delay);

	QMP_LOG("store_delay=%ld status=%d\n",delay, status);
	if((status == 0)&&(delay>1000000)) 
	{
		g_qmp->delay = (int)(delay/1000/1000);
	}

	return status;
}

static ssize_t show_enable(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_qmp->enable);
}

static ssize_t store_enable(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	s32 enable;
	s32 status = kstrtoint(buf, 10, &enable);

	QMP_LOG("store_enable=%d status=%d\n",enable, status);
	if(status == 0)
	{
		enable = enable ? 1 : 0;
		if(g_qmp->enable != enable)
		{
			g_qmp->enable = enable;
			qmp_measure_control(g_qmp->enable);
		}
		return count;
	}

	return status;
}

static ssize_t show_batch(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_qmp->enable);
}

static ssize_t store_batch(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	return store_enable(dev, attr, buf, count);
}

static ssize_t show_iptnum(struct device *dev,struct device_attribute *attr, char *buf)
{
	const char *devname = NULL;

	devname = dev_name(&(g_qmp->input->dev));
	return snprintf(buf, PAGE_SIZE, "%s\n", devname+5);
}

static ssize_t show_selftest(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "OK\n");
}

static ssize_t store_selftest(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int action;
	int status = kstrtoint(buf, 10, &action);

	QMP_LOG("store_selftest=%d status=%d\n",action, status);

	return count;
}

static u8 qmp6988_read_reg_addr=0x00;
static u8 qmp6988_read_reg_len=0x01;

static ssize_t show_read_reg(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char a_data_u8r[256];
	int icount, write_offset;

	qmp_i2c_read_block(g_qmp->client,qmp6988_read_reg_addr,a_data_u8r,qmp6988_read_reg_len);

	write_offset = 0;
	for(icount=0; icount<qmp6988_read_reg_len; icount++)
	{
		write_offset += snprintf(buf+write_offset, 4096-write_offset, "0x%02x=%02x\n", 
			(unsigned char)(qmp6988_read_reg_addr+icount), a_data_u8r[icount]);
	}

	return write_offset;
}

static ssize_t store_read_reg(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int addr, value;
	
	if(2 == sscanf(buf, "0x%x %d", &addr, &value))
	{	
		QMP_LOG("addr=%02x, value=%02x\n", addr, value);
		qmp6988_read_reg_addr = (unsigned char)addr;
		qmp6988_read_reg_len = (unsigned char)value;
	}

	return count;
}

static ssize_t store_set_reg(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int addr, value;
	unsigned char data[2];
	int res = 0;

	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))
	{
		QMP_LOG("addr=%02x, value=%02x\n", addr, value);
		data[0] = (unsigned char)addr;
		data[1] = (unsigned char)value;
		res = qmp_i2c_write_block(g_qmp->client,data[0], &data[1], 1);
	}

	return count;
}

static ssize_t show_pre_temp(struct device *dev,struct device_attribute *attr, char *buf)
{
	int pressure_raw, temp_raw;
	
	qmp_read_raw_data(&pressure_raw, &temp_raw);

	return sprintf(buf, "raw data:%d %d\n", pressure_raw,temp_raw);
}

static ssize_t show_cali(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
		g_qmp->qmp6988_cali.COE_a0,g_qmp->qmp6988_cali.COE_a1,g_qmp->qmp6988_cali.COE_a2,
		g_qmp->qmp6988_cali.COE_b00,g_qmp->qmp6988_cali.COE_bt1,g_qmp->qmp6988_cali.COE_bt2,
		g_qmp->qmp6988_cali.COE_bp1,g_qmp->qmp6988_cali.COE_b11,
		g_qmp->qmp6988_cali.COE_bp2,g_qmp->qmp6988_cali.COE_b12,
		g_qmp->qmp6988_cali.COE_b21,g_qmp->qmp6988_cali.COE_bp3);
}


static DEVICE_ATTR(delay, S_IWUSR|S_IRUGO, show_delay, store_delay);
static DEVICE_ATTR(enable, S_IWUSR|S_IRUGO, show_enable, store_enable);
static DEVICE_ATTR(batch, S_IWUSR|S_IRUGO, show_batch, store_batch);
static DEVICE_ATTR(iptnum, S_IRUGO, show_iptnum, NULL);
static DEVICE_ATTR(selftest, S_IWUSR|S_IRUGO, show_selftest, store_selftest);
static DEVICE_ATTR(read_reg, S_IRUGO|S_IWUGO, show_read_reg, store_read_reg);
static DEVICE_ATTR(set_reg, S_IRUGO|S_IWUGO, NULL, store_set_reg);
static DEVICE_ATTR(pre_temp, S_IRUGO, show_pre_temp, NULL);
static DEVICE_ATTR(cali, S_IRUGO, show_cali, NULL);


/*!
 * @brief device attribute files
*/
static struct attribute *qmp_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_batch.attr,
	&dev_attr_iptnum.attr,
	&dev_attr_selftest.attr,
	&dev_attr_read_reg.attr,
	&dev_attr_set_reg.attr,
	&dev_attr_pre_temp.attr,
	&dev_attr_cali.attr,
	NULL
};

/*!
 * @brief attribute files group
*/
static const struct attribute_group qmp_attr_group = {
	/**< bmp attributes */
//	.name = "qmp6988",
	.attrs = qmp_attributes,
};


static int qmp_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enabled)
{
	if(g_qmp->enable == enabled)
	{
		QMP_LOG("already %s\n", enabled ? "enabled" : "disabled");
	}
	g_qmp->enable = enabled;
	QMP_LOG("g_qmp->enable %d\n", g_qmp->enable);

	qmp_measure_control(g_qmp->enable);
	return 0;
}

static int qmp_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec)
{
	g_qmp->delay = delay_msec;
	QMP_LOG("g_qmp->delay %d\n", g_qmp->delay);
	qmp_measure_control(g_qmp->enable);

	return 0;
}

static int qmp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct qmp_i2c_data *obj;
	int err = 0;

	QMP_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	g_qmp = obj;
	obj->client = client;
	i2c_set_clientdata(client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->power_mode = QMP_UNDEFINED_POWERMODE;
	obj->hw_filter = QMP_UNDEFINED_FILTER;
	obj->oversampling_p = QMP_UNDEFINED_OVERSAMPLING;
	obj->oversampling_t = QMP_UNDEFINED_OVERSAMPLING;
	obj->delay = 200;
	mutex_init(&obj->lock);
	//init_waitqueue_head(&open_wq);
#ifdef CONFIG_QMP_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif
// get form dts
	qmp_parse_dt(&client->dev);
// get from dts
	qmp_init_hw(obj->client);
	g_qmp->power_enabled = 0;
	qmp_set_power(1);

	err = qmp_init_client(client);
	if (err)
		goto exit_init_client_failed;
	
	err = qmp_input_init(obj);
	if (err != 0)
		goto exit_init_client_failed;

#if 1
	err = misc_register(&qmp_device);
	if (err) {
		QMP_ERR("misc device register failed, err = %d\n", err);
		goto exit_input_device_register_failed;
	}
#endif
	INIT_DELAYED_WORK(&obj->work, qmp_work_func);

	//err = sysfs_create_group(&obj->input->dev.kobj, &qmp_attr_group);
	err = sysfs_create_group(&qmp_device.this_device->kobj, &qmp_attr_group);
	if (err)
	{
		goto exit_misc_device_register_failed;
	}
	
	obj->cdev = pressure_sensors_cdev;
	obj->cdev.sensors_enable = qmp_enable_set;
	obj->cdev.sensors_poll_delay = qmp_poll_delay_set;
	err = sensors_classdev_register(&obj->input->dev, &obj->cdev);
	if(err)
	{
		goto exit_sysfs_create_failed;
	}

	qmp_init_flag = 0;
	QMP_LOG("%s: OK\n", __func__);
	return 0;

exit_sysfs_create_failed:
	sysfs_remove_group(&obj->input->dev.kobj, &qmp_attr_group);
exit_misc_device_register_failed:
	err = misc_deregister(&qmp_device);
exit_input_device_register_failed:
	qmp_input_delete(obj);
exit_init_client_failed:
	kfree(obj);
exit:
	QMP_ERR("err = %d\n", err);
	qmp_init_flag =  -1;
	return err;
}

static int qmp_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	
	err = misc_deregister(&qmp_device);
	if(g_qmp)
	{
		sysfs_remove_group(&g_qmp->input->dev.kobj, &qmp_attr_group);
		qmp_input_delete(g_qmp);
		kfree(g_qmp);
		g_qmp = NULL;
	}
	qmp_set_powermode(QMP_SUSPEND_MODE);
	qmp_set_power(0);
	qmp_deinit_hw(client);

	return 0;
}

static int __init qmp_init(void)
{
	i2c_add_driver(&qmp_i2c_driver);

	return 0;
}

static void __exit qmp_exit(void)
{
	i2c_del_driver(&qmp_i2c_driver);
}
module_init(qmp_init);
module_exit(qmp_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("QMP6988 Driver");
MODULE_AUTHOR("QST YZQ");
MODULE_VERSION(QMP_DRIVER_VERSION);

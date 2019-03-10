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
#include <linux/math64.h>

#include <cust_baro.h>
#include "qmp6988.h"
#include "barometer.h"
/* #include <linux/hwmsen_helper.h> */

/* #define POWER_NONE_MACRO MT65XX_POWER_NONE */

/* sensor type */
enum SENSOR_TYPE_ENUM {
	QMP6988_TYPE = 0x0,
	INVALID_TYPE = 0xff
};

/* power mode */
enum QMP_POWERMODE_ENUM {
	QMP_SUSPEND_MODE = 0x0,
//	QMP_FORCED_MODE,
	QMP_NORMAL_MODE,

	QMP_UNDEFINED_POWERMODE = 0xff
};

/* filter */
enum QMP_FILTER_ENUM {
	QMP_FILTER_OFF = 0x0,
	QMP_FILTER_2,
	QMP_FILTER_4,
	QMP_FILTER_8,
	QMP_FILTER_16,
	QMP_FILTER_32,
	QMP_UNDEFINED_FILTER = 0xff
};

/* oversampling */
enum QMP_OVERSAMPLING_ENUM {
	QMP_OVERSAMPLING_SKIPPED = 0x0,
	QMP_OVERSAMPLING_1X,
	QMP_OVERSAMPLING_2X,
	QMP_OVERSAMPLING_4X,
	QMP_OVERSAMPLING_8X,
	QMP_OVERSAMPLING_16X,
	QMP_OVERSAMPLING_32X,
	QMP_OVERSAMPLING_64X,
	QMP_UNDEFINED_OVERSAMPLING = 0xff
};

/* trace */
enum BAR_TRC {
	BAR_TRC_READ  = 0x01,
	BAR_TRC_RAWDATA = 0x02,
	BAR_TRC_IOCTL   = 0x04,
	BAR_TRC_FILTER  = 0x08,
	BAR_TRC_INFO  = 0x10,
};

/* s/w filter */
struct data_filter {
	u32 raw[C_MAX_FIR_LENGTH][QMP_DATA_NUM];
	int sum[QMP_DATA_NUM];
	int num;
	int idx;
};

/* qmp6988 calibration */
struct qmp6988_calibration_data {
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

/* qmp i2c client data */
struct qmp_i2c_data {
	struct i2c_client *client;
	struct baro_hw *hw;

	/* sensor info */
	u8 sensor_name[MAX_SENSOR_NAME];
	enum SENSOR_TYPE_ENUM sensor_type;
	enum QMP_POWERMODE_ENUM power_mode;
	u8 hw_filter;
	u8 oversampling_p;
	u8 oversampling_t;
	struct qmp6988_calibration_data qmp6988_cali;

	/* calculated temperature correction coefficient */
	s32 t_fine;

	/*misc*/
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

/*IOCTL CMD*/
#define BROMETER							0X87
#define BAROMETER_GET_PRESS_RAW_DATA		_IOR(BROMETER, 0x05, int)
#define BAROMETER_GET_TEMP_RAW_DATA			_IOR(BROMETER, 0x06, int)
#define BAROMETER_GET_CALIBRATION_DATA		_IOR(BROMETER, 0x07, struct qmp6988_calibration_data)
#define BAROMETER_SET_PT_DATA				_IOW(BROMETER, 0X08, int[2])
#define QMP_IOCTL_GET_OPEN_STATUS			_IOR(BROMETER, 0x09, int)
#define QMP_IOCTL_GET_CLOSE_STATUS			_IOR(BROMETER, 0x0A, int)
#define QMP_IOCTL_GET_DELAY					_IOR(BROMETER, 0x10, int)

#define QMP_TAG                  "[QMP6988] "
#define QMP_FUN(f)               pr_debug(QMP_TAG"%s\n", __func__)
#define QMP_ERR(fmt, args...)	 pr_err(QMP_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define QMP_LOG(fmt, args...)    pr_debug(QMP_TAG fmt, ##args)

static struct i2c_driver qmp_i2c_driver;
static struct qmp_i2c_data *obj_i2c_data = NULL;
static const struct i2c_device_id qmp_i2c_id[] = {
	{QMP_DEV_NAME, 0},
	{}
};

/*array for pressure and temperature*/
static int sensor_data[2] = {0};

static short qmcd_delay = 200;
static struct mutex sensor_data_mutex;
static atomic_t open_flag = ATOMIC_INIT(0);
static unsigned char v_open_flag = 0x00;
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

/* Maintain  cust info here */
struct baro_hw baro_cust;
static struct baro_hw *hw = &baro_cust;
/* For baro driver get cust info */
struct baro_hw *get_cust_baro(void)
{
	return &baro_cust;
}

#ifdef CONFIG_MTK_LEGACY
static struct i2c_board_info qmp_i2c_info __initdata = {
	I2C_BOARD_INFO(QMP_DEV_NAME, QMP6988_I2C_ADDRESS)
};
#endif
static int QMP_GetOpenStatus(void);
static int QMP_GetCloseStatus(void);

static int qmp_local_init(void);
static int  qmp_remove(void);
static int qmp_init_flag =  -1;
static struct baro_init_info qmp_init_info = {
		.name = "qmp6988",
		.init = qmp_local_init,
		.uninit = qmp_remove,
};

/* I2C operation functions */
static int qmp_i2c_read_block(struct i2c_client *client,
			u8 addr, u8 *data, u8 len)
{
	u8 reg_addr = addr;
	u8 *rxbuf = data;
	u8 left = len;
	u8 retry;
	u8 offset = 0;

	struct i2c_msg msg[2] = {
		{
		    .addr = client->addr,
		    .flags = 0,
		    .buf = &reg_addr,
		    .len = 1,
		},
		{
		    .addr = client->addr,
		    .flags = I2C_M_RD,
		},
	};

	if (rxbuf == NULL)
		return -1;

	while (left > 0) {
		retry = 0;
		reg_addr = addr + offset;
		msg[1].buf = &rxbuf[offset];

		if (left > C_I2C_FIFO_SIZE) {
			msg[1].len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE;
			offset += C_I2C_FIFO_SIZE;
		} else {
			msg[1].len = left;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg[0], 2) != 2) {
			retry++;

			if (retry == 20) {
				QMP_ERR("i2c read register=%#x length=%d failed\n", addr + offset, len);
				return -EIO;
			}
		}
	}

	return 0;
}

static int qmp_i2c_write_block(struct i2c_client *client, u8 addr,
			u8 *data, u8 len)
{
	u8 buffer[C_I2C_FIFO_SIZE];
	u8 *txbuf = data;
	u8 left = len;
	u8 offset = 0;
	u8 retry = 0;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = buffer,
	};

	if (txbuf == NULL)
		return -1;

	while (left > 0) {
		retry = 0;
		/* register address */
		buffer[0] = addr + offset;

		if (left >= C_I2C_FIFO_SIZE) {
			memcpy(&buffer[1], &txbuf[offset], C_I2C_FIFO_SIZE - 1);
			msg.len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE - 1;
			offset += C_I2C_FIFO_SIZE - 1;
		} else {
			memcpy(&buffer[1], &txbuf[offset], left);
			msg.len = left + 1;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg, 1) != 1) {
			retry++;

			if (retry == 20) {
				QMP_ERR("i2c write register=%#x length=%d failed\n", buffer[0], len);
				return -EIO;
			}

			QMP_LOG("i2c write addr %#x, retry %d\n",
				buffer[0], retry);
		}
	}

	return 0;
}

static void qmp_power(struct baro_hw *hw, unsigned int on)
{
	static unsigned int power_on;
#if 0
	if (hw->power_id != POWER_NONE_MACRO) {/* have externel LDO */
		QMP_LOG("power %s\n", on ? "on" : "off");
		if (power_on == on) {/* power status not change */
			QMP_LOG("ignore power control: %d\n", on);
		} else if (on) {/* power on */
			if (!hwPowerOn(hw->power_id, hw->power_vol,
				QMP_DEV_NAME))
				QMP_ERR("power on failed\n");
		} else {/* power off */
			if (!hwPowerDown(hw->power_id, QMP_DEV_NAME))
				QMP_ERR("power off failed\n");
		}
	}
#endif
	power_on = on;
}

/* get chip type */
static int qmp_get_chip_type(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);

	/* QMP_FUN(f); */

	err = qmp_i2c_read_block(client, QMP_CHIP_ID_REG, &chip_id, 0x01);
	if (err != 0)
		return err;

	switch (chip_id) {
	case QMP6988_CHIP_ID:
		obj->sensor_type = QMP6988_TYPE;
		strncpy(obj->sensor_name, "qmp6988", sizeof(obj->sensor_name));
		break;
	default:
		obj->sensor_type = INVALID_TYPE;
		strncpy(obj->sensor_name, "unknown sensor", sizeof(obj->sensor_name));
		break;
	}

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_LOG("[%s]chip id = %#x, sensor name = %s\n", __func__,
			chip_id, obj->sensor_name);

	if (obj->sensor_type == INVALID_TYPE) {
		QMP_ERR("unknown pressure sensor\n");
		return -1;
	}
	return 0;
}


static int qmp_get_calibration_data(struct i2c_client *client)
{
	struct qmp_i2c_data *obj =
		(struct qmp_i2c_data *)i2c_get_clientdata(client);
	int status = 0;

	BITFIELDS temp_COE;
	
	if (obj->sensor_type == QMP6988_TYPE) {
		u8 a_data_u8r[QMP6988_CALIBRATION_DATA_LENGTH] = {0};

		status = qmp_i2c_read_block(client,
			QMP6988_CALIBRATION_DATA_START,
			a_data_u8r,
			QMP6988_CALIBRATION_DATA_LENGTH);
		if (status < 0)
			return status;

		temp_COE.x = (QMP6988_U32_t)((a_data_u8r[18] << \
			SHIFT_LEFT_12_POSITION) | (a_data_u8r[19] << \
			SHIFT_LEFT_4_POSITION) | (a_data_u8r[24] & 0x0f));
		obj->qmp6988_cali.COE_a0 = 	temp_COE.x;
		
		obj->qmp6988_cali.COE_a1 = (QMP6988_S16_t)(((a_data_u8r[20]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[21]);
		obj->qmp6988_cali.COE_a2 = (QMP6988_S16_t)(((a_data_u8r[22]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[23]);
		
		temp_COE.x = (QMP6988_U32_t)((a_data_u8r[0] << \
			SHIFT_LEFT_12_POSITION) | (a_data_u8r[1] << \
			SHIFT_LEFT_4_POSITION) | ((a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION));		
		obj->qmp6988_cali.COE_b00 = temp_COE.x;	
		
		obj->qmp6988_cali.COE_bt1 = (QMP6988_S16_t)(((a_data_u8r[2]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[3]);
		obj->qmp6988_cali.COE_bt2 = (QMP6988_S16_t)(((a_data_u8r[4]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[5]);
		obj->qmp6988_cali.COE_bp1 = (QMP6988_S16_t)(((a_data_u8r[6]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[7]);
		obj->qmp6988_cali.COE_b11 = (QMP6988_S16_t)(((a_data_u8r[8]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[9]);
		obj->qmp6988_cali.COE_bp2 = (QMP6988_S16_t)(((a_data_u8r[10]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[11]);
		obj->qmp6988_cali.COE_b12 = (QMP6988_S16_t)(((a_data_u8r[12]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[13]);		
		obj->qmp6988_cali.COE_b21 = (QMP6988_S16_t)(((a_data_u8r[14]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[15]);
		obj->qmp6988_cali.COE_bp3 = (QMP6988_S16_t)(((a_data_u8r[16]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[17]);			

		QMP_LOG("<-----------calibration data-------------->\n");
		QMP_LOG("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
				obj->qmp6988_cali.COE_a0,obj->qmp6988_cali.COE_a1,obj->qmp6988_cali.COE_a2,obj->qmp6988_cali.COE_b00);
		QMP_LOG("COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\n",
				obj->qmp6988_cali.COE_bt1,obj->qmp6988_cali.COE_bt2,obj->qmp6988_cali.COE_bp1,obj->qmp6988_cali.COE_b11);
		QMP_LOG("COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\n",
				obj->qmp6988_cali.COE_bp2,obj->qmp6988_cali.COE_b12,obj->qmp6988_cali.COE_b21,obj->qmp6988_cali.COE_bp3);
		QMP_LOG("<-----------calibration data-------------->\n");
	}
	return 0;
}

static int qmp_set_powermode(struct i2c_client *client,
		enum QMP_POWERMODE_ENUM power_mode)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_power_mode = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_LOG("[%s] power_mode = %d, old power_mode = %d\n", __func__,
			power_mode, obj->power_mode);

	if (power_mode == obj->power_mode)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == QMP6988_TYPE) {/* QMP6988 */
		if (power_mode == QMP_SUSPEND_MODE) {
			actual_power_mode = QMP6988_SLEEP_MODE;
		} else if (power_mode == QMP_NORMAL_MODE) {
			actual_power_mode = QMP6988_NORMAL_MODE;
		}else {
			err = -EINVAL;
			QMP_ERR("invalid power mode = %d\n", power_mode);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = qmp_i2c_read_block(client,
			QMP6988_CTRLMEAS_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,
			QMP6988_CTRLMEAS_REG_MODE, actual_power_mode);
		err += qmp_i2c_write_block(client,
			QMP6988_CTRLMEAS_REG, &data, 1);
	}

	if (err < 0)
		QMP_ERR("set power mode failed, err = %d, sensor name = %s\n",
			err, obj->sensor_name);
	else
		obj->power_mode = power_mode;

	mutex_unlock(&obj->lock);
	return err;
}

static int qmp_set_filter(struct i2c_client *client,
		enum QMP_FILTER_ENUM filter)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_filter = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_LOG("[%s] hw filter = %d, old hw filter = %d\n", __func__,
			filter, obj->hw_filter);

	if (filter == obj->hw_filter)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == QMP6988_TYPE) {/* QMP6988 */
		if (filter == QMP_FILTER_OFF)
			actual_filter = QMP6988_FILTERCOEFF_OFF;
		else if (filter == QMP_FILTER_2)
			actual_filter = QMP6988_FILTERCOEFF_2;
		else if (filter == QMP_FILTER_4)
			actual_filter = QMP6988_FILTERCOEFF_4;
		else if (filter == QMP_FILTER_8)
			actual_filter = QMP6988_FILTERCOEFF_8;
		else if (filter == QMP_FILTER_16)
			actual_filter = QMP6988_FILTERCOEFF_16;
		else if (filter == QMP_FILTER_32)
			actual_filter = QMP6988_FILTERCOEFF_32;
		else {
			err = -EINVAL;
			QMP_ERR("invalid hw filter = %d\n", filter);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = qmp_i2c_read_block(client,
			QMP6988_CONFIG_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,
			QMP6988_CONFIG_REG_FILTER, actual_filter);
		err += qmp_i2c_write_block(client,
			QMP6988_CONFIG_REG, &data, 1);
	}

	if (err < 0)
		QMP_ERR("set hw filter failed, err = %d, sensor name = %s\n",
			err, obj->sensor_name);
	else
		obj->hw_filter = filter;

	mutex_unlock(&obj->lock);
	return err;
}

static int qmp_set_oversampling_p(struct i2c_client *client,
		enum QMP_OVERSAMPLING_ENUM oversampling_p)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_oversampling_p = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_LOG("[%s] oversampling_p = %d, old oversampling_p = %d\n", __func__,
			oversampling_p, obj->oversampling_p);

	if (oversampling_p == obj->oversampling_p)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == QMP6988_TYPE) {/* QMP6988 */
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
			QMP_ERR("invalid oversampling_p = %d\n",
				oversampling_p);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = qmp_i2c_read_block(client,
			QMP6988_CTRLMEAS_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,
			QMP6988_CTRLMEAS_REG_OSRSP, actual_oversampling_p);
		err += qmp_i2c_write_block(client,
			QMP6988_CTRLMEAS_REG, &data, 1);
	}

	if (err < 0)
		QMP_ERR("set pressure oversampling failed, err = %d,sensor name = %s\n", err, obj->sensor_name);
	else
		obj->oversampling_p = oversampling_p;

	mutex_unlock(&obj->lock);
	return err;
}

static int qmp_set_oversampling_t(struct i2c_client *client,
		enum QMP_OVERSAMPLING_ENUM oversampling_t)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_oversampling_t = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_LOG("[%s] oversampling_t = %d, old oversampling_t = %d\n", __func__,
			oversampling_t, obj->oversampling_t);

	if (oversampling_t == obj->oversampling_t)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == QMP6988_TYPE) {/* QMP6988 */
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
			QMP_ERR("invalid oversampling_t = %d\n",
				oversampling_t);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = qmp_i2c_read_block(client,
			QMP6988_CTRLMEAS_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,
			QMP6988_CTRLMEAS_REG_OSRST, actual_oversampling_t);
		err += qmp_i2c_write_block(client,
			QMP6988_CTRLMEAS_REG, &data, 1);
	}

	if (err < 0)
		QMP_ERR("set temperature oversampling failed, err = %d, sensor name = %s\n", err, obj->sensor_name);
	else
		obj->oversampling_t = oversampling_t;

	mutex_unlock(&obj->lock);
	return err;
}

static int qmp_read_raw_temperature(struct i2c_client *client,
			s32 *temperature)
{
	struct qmp_i2c_data *obj;
	s32 err = 0;
	QMP6988_U32_t Traw;
	
	if (NULL == client) {
		err = -EINVAL;
		return err;
	}

	obj = i2c_get_clientdata(client);

	mutex_lock(&obj->lock);

	if (obj->sensor_type == QMP6988_TYPE) {/* QMP6988 */
		unsigned char a_data_u8r[3] = {0};

		err = qmp_i2c_read_block(client,
			QMP6988_TEMPERATURE_MSB_REG, a_data_u8r, 3);
		if (err < 0) {
			QMP_ERR("read raw temperature failed, err = %d\n", err);
			mutex_unlock(&obj->lock);
			return err;
		}
		Traw = (QMP6988_U32_t)(
		(((QMP6988_U32_t)(a_data_u8r[0])) << SHIFT_LEFT_16_POSITION) |
		(((QMP6988_U16_t)(a_data_u8r[1])) << SHIFT_LEFT_8_POSITION) | 
		(a_data_u8r[2]));
		
		*temperature = (QMP6988_S32_t)(Traw - SUBTRACTOR);
		
		QMP_LOG("%s: Traw[%d]	Temperature[%d]\n",__func__,Traw,*temperature);
	}
	mutex_unlock(&obj->lock);

	return err;
}

static int qmp_read_raw_pressure(struct i2c_client *client, s32 *pressure)
{
	struct qmp_i2c_data *priv;
	s32 err = 0;
	QMP6988_U32_t Praw;
	if (NULL == client) {
		err = -EINVAL;
		return err;
	}

	priv = i2c_get_clientdata(client);

	mutex_lock(&priv->lock);

	if (priv->sensor_type == QMP6988_TYPE) {/* QMP6988 */
		unsigned char a_data_u8r[3] = {0};

		err = qmp_i2c_read_block(client,
			QMP6988_PRESSURE_MSB_REG, a_data_u8r, 3);
		if (err < 0) {
			QMP_ERR("read raw pressure failed, err = %d\n", err);
			mutex_unlock(&priv->lock);
			return err;
		}
		
		Praw = (QMP6988_U32_t)(
		(((QMP6988_U32_t)(a_data_u8r[0])) << SHIFT_LEFT_16_POSITION) |
		(((QMP6988_U16_t)(a_data_u8r[1])) << SHIFT_LEFT_8_POSITION) |
		(a_data_u8r[2]));
		
		*pressure = (QMP6988_S32_t)(Praw - SUBTRACTOR);
		
		QMP_LOG("%s: Praw[%d]	pressure[%d]\n",__func__,Praw,*pressure);
	}

#ifdef CONFIG_QMP_LOWPASS
/*
*Example: firlen = 16, filter buffer = [0] ... [15],
*when 17th data come, replace [0] with this new data.
*Then, average this filter buffer and report average value to upper layer.
*/
	if (atomic_read(&priv->filter)) {
		if (atomic_read(&priv->fir_en) &&
			!atomic_read(&priv->suspend)) {
			int idx, firlen = atomic_read(&priv->firlen);

			if (priv->fir.num < firlen) {
				priv->fir.raw[priv->fir.num][QMP_PRESSURE] =
					*pressure;
				priv->fir.sum[QMP_PRESSURE] += *pressure;
				if (atomic_read(&priv->trace) &
					BAR_TRC_FILTER) {
					QMP_LOG("add [%2d] [%5d] => [%5d]\n",
					priv->fir.num,
					priv->fir.raw
					[priv->fir.num][QMP_PRESSURE],
					priv->fir.sum[QMP_PRESSURE]);
				}
				priv->fir.num++;
				priv->fir.idx++;
			} else {
				idx = priv->fir.idx % firlen;
				priv->fir.sum[QMP_PRESSURE] -=
					priv->fir.raw[idx][QMP_PRESSURE];
				priv->fir.raw[idx][QMP_PRESSURE] = *pressure;
				priv->fir.sum[QMP_PRESSURE] += *pressure;
				priv->fir.idx++;
				*pressure = priv->fir.sum[QMP_PRESSURE]/firlen;
				if (atomic_read(&priv->trace) &
					BAR_TRC_FILTER) {
					QMP_LOG("add [%2d][%5d]=>[%5d]:[%5d]\n",
					idx,
					priv->fir.raw[idx][QMP_PRESSURE],
					priv->fir.sum[QMP_PRESSURE],
					*pressure);
				}
			}
		}
	}
#endif

	mutex_unlock(&priv->lock);
	return err;
}

/*
*get compensated temperature
*unit:1 degrees centigrade
*/
static int qmp_get_temperature(struct i2c_client *client,
		char *buf, int bufsize)
{
	struct qmp_i2c_data *obj;

	s32 temperature = 0;

	if (NULL == buf)
		return -EINVAL;

	if (NULL == client) {
		QMP_ERR("i2c_client is NULL\n");
		*buf = 0;
		return -EINVAL;
	}

	obj = i2c_get_clientdata(client);

	if (NULL == obj) {
		QMP_ERR("qmp_i2c_data is NULL\n");
		return -EINVAL;
	}
	mutex_lock(&sensor_data_mutex);
	if (obj->sensor_type == QMP6988_TYPE) {/* QMP6988 */
		temperature = sensor_data[1]; //temperature data
	}
	mutex_unlock(&sensor_data_mutex);
	
	sprintf(buf, "%08x", temperature);
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL) {
		QMP_LOG("temperature: %d\n", temperature);
		QMP_LOG("temperature/100: %d\n", temperature/100);
		QMP_LOG("compensated temperature value: %s\n", buf);
	}

	return 0;
}

/*
*get compensated pressure
*unit: hectopascal(hPa)
*/
static int qmp_get_pressure(struct i2c_client *client, char *buf, int bufsize)
{
	struct qmp_i2c_data *obj = NULL;
	s32 pressure = 0;

	if (NULL == buf)
		return -EINVAL;

	if (NULL == client) {
		*buf = 0;
		return -EINVAL;
	}

	obj = i2c_get_clientdata(client);
	if (NULL == obj) {
		return -EINVAL;
	}

	mutex_lock(&sensor_data_mutex);
	if (obj->sensor_type == QMP6988_TYPE) {/* QMP6988 */
		pressure = sensor_data[0]; //pressure data
	}
	mutex_unlock(&sensor_data_mutex);

	sprintf(buf, "%08x", pressure);
	
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL) {
		QMP_LOG("pressure: %d\n", pressure);
		QMP_LOG("pressure/100: %d\n", pressure/100);
		QMP_LOG("compensated pressure value: %s\n", buf);
	}

	return 0;
}

/*
0xF5 = 0x00; Standy time = 1ms
0xF1 = 0x01; IIR = 2
0xF4 = 0x57; temp_ave = 2,press_ave = 16,power_mode = normal mode
*/
/* qmp setting initialization */
static int qmp_init_client(struct i2c_client *client)
{
	int err = 0;

	QMP_FUN();

	err = qmp_get_chip_type(client);
	if (err < 0) {
		QMP_ERR("get chip type failed, err = %d\n", err);
		return err;
	}

	err = qmp_get_calibration_data(client);
	if (err < 0) {
		QMP_ERR("get calibration data failed, err = %d\n", err);
		return err;
	}

	err = qmp_set_powermode(client, QMP_SUSPEND_MODE);
	if (err < 0) {
		QMP_ERR("set power mode failed, err = %d\n", err);
		return err;
	}

	err = qmp_set_filter(client, QMP_FILTER_2);
	if (err < 0) {
		QMP_ERR("set hw filter failed, err = %d\n", err);
		return err;
	}

	err = qmp_set_oversampling_p(client, QMP_OVERSAMPLING_16X);
	if (err < 0) {
		QMP_ERR("set pressure oversampling failed, err = %d\n", err);
		return err;
	}

	err = qmp_set_oversampling_t(client, QMP_OVERSAMPLING_2X);
	if (err < 0) {
		QMP_ERR("set temperature oversampling failed, err = %d\n", err);
		return err;
	}

	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct qmp_i2c_data *obj = obj_i2c_data;

	if (NULL == obj) {
		QMP_ERR("qmp i2c data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", obj->sensor_name);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct qmp_i2c_data *obj = obj_i2c_data;
	char strbuf[QMP_BUFSIZE] = "";

	if (NULL == obj) {
		QMP_ERR("qmp i2c data pointer is null\n");
		return 0;
	}

	qmp_get_pressure(obj->client, strbuf, QMP_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct qmp_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		QMP_ERR("qmp i2c data pointer is null\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf,
		size_t count)
{
	struct qmp_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		QMP_ERR("i2c_data obj is null\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		QMP_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct qmp_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		QMP_ERR("qmp i2c data pointer is null\n");
		return 0;
	}

	if (obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw->i2c_num,
			obj->hw->direction,
			obj->hw->power_id,
			obj->hw->power_vol);
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	len += snprintf(buf+len, PAGE_SIZE-len, "i2c addr:%#x,ver:%s\n",
			obj->client->addr, QMP_DRIVER_VERSION);

	return len;
}

static ssize_t show_power_mode_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct qmp_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		QMP_ERR("qmp i2c data pointer is null\n");
		return 0;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "%s mode\n",
		obj->power_mode == QMP_NORMAL_MODE ? "normal" : "suspend");

	return len;
}

static ssize_t store_power_mode_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct qmp_i2c_data *obj = obj_i2c_data;
	unsigned long power_mode;
	int err;

	if (obj == NULL) {
		QMP_ERR("qmp i2c data pointer is null\n");
		return 0;
	}

	err = kstrtoul(buf, 10, &power_mode);

	if (err == 0) {
		err = qmp_set_powermode(obj->client,
			(enum QMP_POWERMODE_ENUM)(!!(power_mode)));
		if (err)
			return err;
		return count;
	}
	return err;
}

static DRIVER_ATTR(chipinfo,	S_IRUGO,	show_chipinfo_value,	NULL);
static DRIVER_ATTR(sensordata,	S_IRUGO,	show_sensordata_value,	NULL);
static DRIVER_ATTR(trace,	S_IWUSR | S_IRUGO,
		show_trace_value,	store_trace_value);
static DRIVER_ATTR(status,	S_IRUGO,	show_status_value,	NULL);
static DRIVER_ATTR(powermode,	S_IWUSR | S_IRUGO,
		show_power_mode_value,	store_power_mode_value);

static struct driver_attribute *qmp_attr_list[] = {
	&driver_attr_chipinfo,	/* chip information*/
	&driver_attr_sensordata,/* dump sensor data*/
	&driver_attr_trace,	/* trace log*/
	&driver_attr_status,	/* cust setting */
	&driver_attr_powermode,	/* power mode */
//	&driver_attr_selftest,	/* self test */
};

static int qmp_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(qmp_attr_list)/sizeof(qmp_attr_list[0]));

	if (NULL == driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, qmp_attr_list[idx]);
		if (err) {
			QMP_ERR("driver_create_file (%s) = %d\n",
			qmp_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int qmp_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(qmp_attr_list)/sizeof(qmp_attr_list[0]));

	if (NULL == driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, qmp_attr_list[idx]);

	return err;
}

#ifdef CONFIG_ID_TEMPERATURE
int temperature_operate(void *self, uint32_t command, void *buff_in,
		int size_in, void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct qmp_i2c_data *priv = (struct qmp_i2c_data *)self;
	struct hwm_sensor_data *temperature_data;
	char buff[QMP_BUFSIZE];

	switch (command) {
	case SENSOR_DELAY:
	/* under construction */
	break;

	case SENSOR_ENABLE:
	if ((buff_in == NULL) || (size_in < sizeof(int))) {
		QMP_ERR("enable sensor parameter error\n");
		err = -EINVAL;
	} else {
		/* value:[0--->suspend, 1--->normal] */
		value = *(int *)buff_in;
		QMP_LOG("sensor enable/disable command: %s\n",
			value ? "enable" : "disable");

		if(value)
		{
			v_open_flag |= 0x02;
		}
		else
		{
			v_open_flag &= 0x01;
		}
		err = qmp_set_powermode(priv->client,
			(enum QMP_POWERMODE_ENUM)(!!value));
		if (err)
			QMP_ERR("set power mode failed, err = %d\n", err);
		}
		atomic_set(&open_flag,v_open_flag);
		wake_up(&open_wq);
	break;

	case SENSOR_GET_DATA:
	if ((buff_out == NULL) ||
		(size_out < sizeof(struct hwm_sensor_data))) {
		QMP_ERR("get sensor data parameter error\n");
		err = -EINVAL;
	} else {
		temperature_data = (struct hwm_sensor_data *)buff_out;
		err = qmp_get_temperature(priv->client, buff, QMP_BUFSIZE);
		if (err) {
			QMP_ERR("get compensated temperature value failed,err = %d\n", err);
			return -1;
		}
		if (sscanf(buff, "%x", &temperature_data->values[0]) != 1)
			QMP_ERR("sscanf parsing fail\n");
		temperature_data->values[1] = temperature_data->values[2] = 0;
		temperature_data->status = SENSOR_STATUS_ACCURACY_HIGH;
		temperature_data->value_divide = 100;
	}
	break;

	default:
		QMP_ERR("temperature operate function no this parameter %d\n",
			command);
		err = -1;
	break;
	}

	return err;
}
#endif/* CONFIG_ID_TEMPERATURE */

static int QMP_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}
static int QMP_GetCloseStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}

/*daemon for saving data*/
static int QMP_SaveData(int *buf){
	struct qmp_i2c_data *obj = obj_i2c_data;
	
	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);
	
	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_LOG("[%s] Pressure = %d, Temperature = %d\n", __func__,sensor_data[0], sensor_data[1]);
	
	return 0;
}


static int qmp6988_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data;

	if (file->private_data == NULL) {
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int qmp6988_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long qmp6988_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct qmp_i2c_data *obj = (struct qmp_i2c_data *)file->private_data;
	struct i2c_client *client = obj->client;
	char strbuf[QMP_BUFSIZE];
	u32 dat = 0;
	s32 dat_buf = 0;
	void __user *data;
	int value[2] = {0};
	int delay;
	int err = 0;
	int status;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		QMP_ERR("access error: %08X, (%2d, %2d)\n",
			cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case BAROMETER_IOCTL_INIT:
		qmp_init_client(client);
		err = qmp_set_powermode(client, QMP_NORMAL_MODE);
		if (err) {
			err = -EFAULT;
			break;
		}
	break;

	case BAROMETER_IOCTL_READ_CHIPINFO:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		strncpy(strbuf, obj->sensor_name, sizeof(strbuf));
		if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
			err = -EFAULT;
			break;
		}
	break;

	case BAROMETER_GET_PRESS_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		qmp_get_pressure(client, strbuf, QMP_BUFSIZE);
		if (sscanf(strbuf, "%x", &dat) != 1)
			QMP_ERR("sscanf parsing fail\n");
		if (copy_to_user(data, &dat, sizeof(dat))) {
			err = -EFAULT;
			break;
		}
	break;

	case BAROMETER_GET_TEMP_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		qmp_get_temperature(client, strbuf, QMP_BUFSIZE);
		if (sscanf(strbuf, "%x", &dat) != 1)
			QMP_ERR("sscanf parsing fail\n");
		if (copy_to_user(data, &dat, sizeof(dat))) {
			err = -EFAULT;
			break;
		}
	break;

	case BAROMETER_GET_PRESS_RAW_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		qmp_read_raw_pressure(client,&dat_buf);
		if (copy_to_user(data, &dat_buf, sizeof(dat_buf))) {
			err = -EFAULT;
			break;
		}	
	break;
	
	case BAROMETER_GET_TEMP_RAW_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		qmp_read_raw_temperature(client,&dat_buf);
		if (copy_to_user(data, &dat_buf, sizeof(dat_buf))) {
			err = -EFAULT;
			break;
		}	
	break;
	
	case BAROMETER_GET_CALIBRATION_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		if (copy_to_user(data, &obj->qmp6988_cali, sizeof(struct qmp6988_calibration_data))) {
			err = -EFAULT;
			break;
		}	
	break;

	case BAROMETER_SET_PT_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(value, data, sizeof(value))){
			err = -EFAULT;
			break;
		}
		QMP_SaveData(value);	
	break;	

	case QMP_IOCTL_GET_OPEN_STATUS:
		data = (void __user *) arg;
		status = QMP_GetOpenStatus();
		if(copy_to_user(data, &status, sizeof(status)))
		{
			QMP_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case QMP_IOCTL_GET_CLOSE_STATUS:
		data = (void __user *) arg;
		status = QMP_GetCloseStatus();
		if(copy_to_user(data, &status, sizeof(status)))
		{
			QMP_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
		
	case QMP_IOCTL_GET_DELAY:
		data = (void __user *) arg;
	    delay = qmcd_delay;
	    if (copy_to_user(data, &delay, sizeof(delay))) {
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
#if IS_ENABLED(CONFIG_COMPAT)
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
		return filp->f_op->unlocked_ioctl(filp, cmd,
			(unsigned long)compat_ptr(arg));
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
	.open = qmp6988_open,
	.release = qmp6988_release,
	.unlocked_ioctl = qmp6988_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_qmp6988_unlocked_ioctl,
#endif
};

static struct miscdevice qmp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "barometer",
	.fops = &qmp_fops,
};

static int qmp_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	if (NULL == obj) {
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {

		atomic_set(&obj->suspend, 1);
		err = qmp_set_powermode(obj->client, QMP_SUSPEND_MODE);
		if (err) {
			QMP_ERR("qmp set suspend mode failed, err = %d\n", err);
			return err;
		}
		qmp_power(obj->hw, 0);
	}
	return err;
}

static int qmp_resume(struct i2c_client *client)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	if (NULL == obj) {
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		QMP_FUN();

	qmp_power(obj->hw, 1);

	err = qmp_init_client(obj->client);
	if (err) {
		QMP_ERR("initialize client fail\n");
		return err;
	}

	err = qmp_set_powermode(obj->client, QMP_NORMAL_MODE);
	if (err) {
		QMP_ERR("qmp set normal mode failed, err = %d\n", err);
		return err;
	}

#ifdef CONFIG_QMP_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	atomic_set(&obj->suspend, 0);
	return 0;
}

static int qmp_i2c_detect(struct i2c_client *client,
		struct i2c_board_info *info)
{
	strncpy(info->type, QMP_DEV_NAME, sizeof(info->type));
	return 0;
}

static int qmp_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

static int qmp_enable_nodata(int en)
{
	//struct qmp_i2c_data *obj = i2c_get_clientdata(obj_i2c_data->client);
	int res = 0;
	int retry = 0;
	bool power = false;

	if (1 == en){
		power = true;
		v_open_flag |= 0x01;
	}
		

	if (0 == en){
		power = false;
		v_open_flag &= 0x02;
	}
	
	for (retry = 0; retry < 3; retry++) {
		res = qmp_set_powermode(obj_i2c_data->client, (enum QMP_POWERMODE_ENUM)(!!power));
		if (res == 0) {
			QMP_LOG("qmp_set_powermode done\n");
			break;
		}
		QMP_ERR("qmp_set_powermode fail\n");
	}

	
	
	if (res != 0) {
		QMP_ERR("qmp_set_powermode fail!\n");
		return -1;
	}
	
	atomic_set(&open_flag,v_open_flag);
	wake_up(&open_wq);
	
	return 0;
}

static int qmp_set_delay(u64 ns)
{
	int mdelay = ns;
	
	do_div(mdelay, 1000000);
	
	if(mdelay < 10)
		qmcd_delay = 10;
	
	qmcd_delay = mdelay;
	
	return 0;
}

static int qmp_get_data(int *value, int *status)
{
	char buff[QMP_BUFSIZE];
	int err = 0;

	err = qmp_get_pressure(obj_i2c_data->client, buff, QMP_BUFSIZE);
	if (err) {
		QMP_ERR("get compensated pressure value failed, err = %d\n", err);
		return -1;
	}
	if (sscanf(buff, "%x", value) != 1)
		QMP_ERR("sscanf parsing fail\n");
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int qmp_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct qmp_i2c_data *obj;
	struct baro_control_path ctl = {0};
	struct baro_data_path data = {0};
#ifdef CONFIG_ID_TEMPERATURE
	struct hwmsen_object sobj_t;
#endif
	int err = 0;

	QMP_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	obj->hw = get_cust_baro();
	obj_i2c_data = obj;
	obj->client = client;
	i2c_set_clientdata(client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->power_mode = QMP_UNDEFINED_POWERMODE;
	obj->hw_filter = QMP_UNDEFINED_FILTER;
	obj->oversampling_p = QMP_UNDEFINED_OVERSAMPLING;
	obj->oversampling_t = QMP_UNDEFINED_OVERSAMPLING;
	mutex_init(&obj->lock);
	mutex_init(&sensor_data_mutex);
	
	init_waitqueue_head(&open_wq);
#ifdef CONFIG_QMP_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif

	err = qmp_init_client(client);
	if (err)
		goto exit_init_client_failed;

	err = misc_register(&qmp_device);
	if (err) {
		QMP_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}

	err = qmp_create_attr(&(qmp_init_info.platform_diver_addr->driver));
	if (err) {
		QMP_ERR("create attribute failed, err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.open_report_data = qmp_open_report_data;
	ctl.enable_nodata = qmp_enable_nodata;
	ctl.set_delay  = qmp_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;

	err = baro_register_control_path(&ctl);
	if (err) {
		QMP_ERR("register baro control path err\n");
		goto exit_hwmsen_attach_pressure_failed;
	}

	data.get_data = qmp_get_data;
	data.vender_div = 100;
	err = baro_register_data_path(&data);
	if (err) {
		QMP_ERR("baro_register_data_path failed, err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}
	err = batch_register_support_info(ID_PRESSURE, obj->hw->is_batch_supported, data.vender_div, 0);
	if (err) {
		QMP_ERR("register baro batch support err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}

#ifdef CONFIG_ID_TEMPERATURE
	sobj_t.self = obj;
	sobj_t.polling = 1;
	sobj_t.sensor_operate = temperature_operate;
	err = hwmsen_attach(ID_TEMPRERATURE, &sobj_t);
	if (err) {
		QMP_ERR("hwmsen attach failed, err = %d\n", err);
		goto exit_hwmsen_attach_temperature_failed;
	}
#endif/* CONFIG_ID_TEMPERATURE */

	qmp_init_flag = 0;
	QMP_LOG("%s: OK\n", __func__);
	return 0;

#ifdef CONFIG_ID_TEMPERATURE
exit_hwmsen_attach_temperature_failed:
	hwmsen_detach(ID_PRESSURE);
#endif/* CONFIG_ID_TEMPERATURE */
exit_hwmsen_attach_pressure_failed:
	qmp_delete_attr(&(qmp_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
	misc_deregister(&qmp_device);
exit_misc_device_register_failed:
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

	err = hwmsen_detach(ID_PRESSURE);
	if (err)
		QMP_ERR("hwmsen_detach ID_PRESSURE failed, err = %d\n", err);

#ifdef CONFIG_ID_TEMPERATURE
	err = hwmsen_detach(ID_TEMPRERATURE);
	if (err)
		QMP_ERR("hwmsen_detach ID_TEMPRERATURE failed, err = %d\n",
			err);
#endif

	err = qmp_delete_attr(&(qmp_init_info.platform_diver_addr->driver));
	if (err)
		QMP_ERR("qmp_delete_attr failed, err = %d\n", err);

	err = misc_deregister(&qmp_device);
	if (err)
		QMP_ERR("misc_deregister failed, err = %d\n", err);

	obj_i2c_data = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int qmp_remove(void)
{
	struct baro_hw *hw = get_cust_baro();

	QMP_FUN();
	qmp_power(hw, 0);
	i2c_del_driver(&qmp_i2c_driver);
	return 0;
}

static int  qmp_local_init(void)
{
	struct baro_hw *hw = get_cust_baro();

	qmp_power(hw, 1);
	if (i2c_add_driver(&qmp_i2c_driver)) {
		QMP_ERR("add driver error\n");
		return -1;
	}
	if (-1 == qmp_init_flag)
		return -1;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id baro_of_match[] = {
	{.compatible = "mediatek,PRESSURE"},
	{},
};
#endif
static struct i2c_driver qmp_i2c_driver = {
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	QMP_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = baro_of_match,
#endif
	},
	.probe = qmp_i2c_probe,
	.remove	= qmp_i2c_remove,
	.detect	= qmp_i2c_detect,
	.suspend = qmp_suspend,
	.resume	= qmp_resume,
	.id_table =	qmp_i2c_id,
};

static int __init qmp_init(void)
{
	const char *name = "mediatek,qmp6988";

	hw = get_baro_dts_func(name, hw);
	if (!hw)
		QMP_ERR("get cust_baro dts info fail\n");

	QMP_FUN();
#ifdef CONFIG_MTK_LEGACY
	i2c_register_board_info(hw->i2c_num, &qmp_i2c_info, 1);
#endif
	baro_driver_add(&qmp_init_info);

	return 0;
}

static void __exit qmp_exit(void)
{
	QMP_FUN();
}
module_init(qmp_init);
module_exit(qmp_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("QMP6988 I2C Driver");
MODULE_AUTHOR("QST industries,Inc");
MODULE_VERSION(QMP_DRIVER_VERSION);

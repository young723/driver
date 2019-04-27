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
	struct i2c_client *client;
	u8 sensor_name[MAX_SENSOR_NAME];
	enum SENSOR_TYPE_ENUM sensor_type;
	enum QMP_POWERMODE_ENUM power_mode;
	u8 hw_filter;
	u8 oversampling_p;
	u8 oversampling_t;
	u8 qmp6988_cali_get_flag;
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


static struct qmp_i2c_data *g_qmp = NULL;
static const struct i2c_device_id qmp_i2c_id[] = {
	{QMP_DEV_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id baro_of_match[] = {
	{.compatible = "QST,PRESSURE"},
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

//static DECLARE_WAIT_QUEUE_HEAD(open_wq);
static int qmp_init_flag =  -1;

/* I2C operation functions */
static int qmp_i2c_read_block(struct i2c_client *client,u8 addr, u8 *data, u8 len)
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

static int qmp_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
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


/* get chip type */
static int qmp_get_chip_type(void)
{
	s32 err = 0;
	u8 chip_id = 0;
	//struct qmp_i2c_data *obj = i2c_get_clientdata(client);

	err = qmp_i2c_read_block(g_qmp->client, QMP_CHIP_ID_REG, &chip_id, 0x01);
	QMP_LOG("qmp6988 chip_id=%x", chip_id);
	if(err != 0)
		return err;

	switch(chip_id)
	{
	case QMP6988_CHIP_ID:
		g_qmp->sensor_type = QMP6988_TYPE;
		strncpy(g_qmp->sensor_name, "qmp6988", sizeof(g_qmp->sensor_name));
		break;
	default:
		g_qmp->sensor_type = INVALID_TYPE;
		strncpy(g_qmp->sensor_name, "unknown sensor", sizeof(g_qmp->sensor_name));
		break;
	}

	QMP_LOG("[%s]chip id = %#x, sensor name = %s\n", __func__,chip_id, g_qmp->sensor_name);

	if(g_qmp->sensor_type == INVALID_TYPE) {
		QMP_ERR("unknown pressure sensor\n");
		return -1;
	}
	return 0;
}


static int qmp_get_calibration_data(void)
{
	s32 status = 0;
	u8 a_data_u8r[QMP6988_CALIBRATION_DATA_LENGTH] = {0};
	
	if(g_qmp->sensor_type == QMP6988_TYPE) 
	{
		if(g_qmp->qmp6988_cali_get_flag)
			return 0;

		status = qmp_i2c_read_block(g_qmp->client,QMP6988_CALIBRATION_DATA_START,a_data_u8r,QMP6988_CALIBRATION_DATA_LENGTH);

		if (status < 0)
			return status;

		g_qmp->qmp6988_cali.COE_a0 = (QMP6988_S32_t)((((QMP6988_S32_t)a_data_u8r[18] << SHIFT_LEFT_12_POSITION) \
			| ((QMP6988_S32_t)a_data_u8r[19] << SHIFT_LEFT_4_POSITION) \
			| ((QMP6988_S32_t)a_data_u8r[24] & 0x0f))<<12);
		g_qmp->qmp6988_cali.COE_a0 = g_qmp->qmp6988_cali.COE_a0>>12;
		
		g_qmp->qmp6988_cali.COE_a1 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[20]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[21]);
		g_qmp->qmp6988_cali.COE_a2 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[22]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[23]);
		
		g_qmp->qmp6988_cali.COE_b00 = (QMP6988_S32_t)((((QMP6988_S32_t)a_data_u8r[0] << SHIFT_LEFT_12_POSITION) \
			| ((QMP6988_S32_t)a_data_u8r[1] << SHIFT_LEFT_4_POSITION) \
			| (((QMP6988_S32_t)a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))<<12);
		g_qmp->qmp6988_cali.COE_b00 = g_qmp->qmp6988_cali.COE_b00>>12;
		
		g_qmp->qmp6988_cali.COE_bt1 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[2]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[3]);
		g_qmp->qmp6988_cali.COE_bt2 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[4]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[5]);
		g_qmp->qmp6988_cali.COE_bp1 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[6]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[7]);
		g_qmp->qmp6988_cali.COE_b11 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[8]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[9]);
		g_qmp->qmp6988_cali.COE_bp2 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[10]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[11]);
		g_qmp->qmp6988_cali.COE_b12 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[12]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[13]);		
		g_qmp->qmp6988_cali.COE_b21 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[14]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[15]);
		g_qmp->qmp6988_cali.COE_bp3 = (QMP6988_S16_t)(((QMP6988_S16_t)(a_data_u8r[16]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[17]);			

		QMP_LOG("<-----------calibration data-------------->\n");
		QMP_LOG("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
				g_qmp->qmp6988_cali.COE_a0,g_qmp->qmp6988_cali.COE_a1,g_qmp->qmp6988_cali.COE_a2,g_qmp->qmp6988_cali.COE_b00);
		QMP_LOG("COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\n",
				g_qmp->qmp6988_cali.COE_bt1,g_qmp->qmp6988_cali.COE_bt2,g_qmp->qmp6988_cali.COE_bp1,g_qmp->qmp6988_cali.COE_b11);
		QMP_LOG("COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\n",
				g_qmp->qmp6988_cali.COE_bp2,g_qmp->qmp6988_cali.COE_b12,g_qmp->qmp6988_cali.COE_b21,g_qmp->qmp6988_cali.COE_bp3);
		QMP_LOG("<-----------calibration data-------------->\n");

		g_qmp->qmp6988_cali_get_flag = 1;
	}
	return 0;
}

static int qmp_set_powermode(enum QMP_POWERMODE_ENUM power_mode)
{
	u8 err = 0, data = 0, actual_power_mode = 0;

	QMP_LOG("qmp_set_powermode  power_mode=%d \n", power_mode); 

	if(power_mode == g_qmp->power_mode)
		return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE)
	{
		if(power_mode == QMP_SUSPEND_MODE)
		{
			actual_power_mode = QMP6988_SLEEP_MODE;
		}
		else if(power_mode == QMP_NORMAL_MODE)
		{
			actual_power_mode = QMP6988_NORMAL_MODE;
		}
		else if(power_mode == QMP_FORCED_MODE)
		{
			actual_power_mode = QMP6988_FORCED_MODE;
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
	if(filter == g_qmp->hw_filter)
		return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE)
	{
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
		else
		{
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

	if(oversampling_p == g_qmp->oversampling_p)
		return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE) 
	{
		if(oversampling_p == QMP_OVERSAMPLING_SKIPPED)
			actual_oversampling_p = QMP6988_OVERSAMPLING_SKIPPED;
		else if(oversampling_p == QMP_OVERSAMPLING_1X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_1X;
		else if(oversampling_p == QMP_OVERSAMPLING_2X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_2X;
		else if(oversampling_p == QMP_OVERSAMPLING_4X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_4X;
		else if(oversampling_p == QMP_OVERSAMPLING_8X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_8X;
		else if(oversampling_p == QMP_OVERSAMPLING_16X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_16X;
		else if(oversampling_p == QMP_OVERSAMPLING_32X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_32X;
		else if(oversampling_p == QMP_OVERSAMPLING_64X)
			actual_oversampling_p = QMP6988_OVERSAMPLING_64X;
		else
		{
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

	if(oversampling_t == g_qmp->oversampling_t)
		return 0;

	if(g_qmp->sensor_type == QMP6988_TYPE)
	{
		if(oversampling_t == QMP_OVERSAMPLING_SKIPPED)
			actual_oversampling_t = QMP6988_OVERSAMPLING_SKIPPED;
		else if(oversampling_t == QMP_OVERSAMPLING_1X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_1X;
		else if(oversampling_t == QMP_OVERSAMPLING_2X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_2X;
		else if(oversampling_t == QMP_OVERSAMPLING_4X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_4X;
		else if(oversampling_t == QMP_OVERSAMPLING_8X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_8X;
		else if(oversampling_t == QMP_OVERSAMPLING_16X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_16X;
		else if(oversampling_t == QMP_OVERSAMPLING_16X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_32X;
		else if(oversampling_t == QMP_OVERSAMPLING_16X)
			actual_oversampling_t = QMP6988_OVERSAMPLING_64X;
		else 
		{
			err = -EINVAL;
			QMP_ERR("invalid oversampling_t = %d\n",oversampling_t);
			return err;
		}
		err = qmp_i2c_read_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
		data = QMP_SET_BITSLICE(data,QMP6988_CTRLMEAS_REG_OSRST, actual_oversampling_t);
		err += qmp_i2c_write_block(g_qmp->client,QMP6988_CTRLMEAS_REG, &data, 1);
	}

	if(err < 0)
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


/* qmp setting initialization */
static int qmp_init_client(struct i2c_client *client)
{
	int err = 0;

	QMP_FUN();

	err = qmp_get_chip_type();
	if(err < 0)
	{
		QMP_ERR("get chip type failed, err = %d\n", err);
		return err;
	}
	err = qmp_get_calibration_data();
	if(err < 0)
	{
		QMP_ERR("get calibration data failed, err = %d\n", err);
		return err;
	}
	err = qmp_set_powermode(QMP_SUSPEND_MODE);
	if(err < 0)
	{
		QMP_ERR("set power mode failed, err = %d\n", err);
		return err;
	}
	err = qmp_set_filter(QMP_FILTER_8);
	if(err < 0)
	{
		QMP_ERR("set hw filter failed, err = %d\n", err);
		return err;
	}
	err = qmp_set_oversampling_p(QMP_OVERSAMPLING_16X);
	if(err < 0)
	{
		QMP_ERR("set pressure oversampling failed, err = %d\n", err);
		return err;
	}
	err = qmp_set_oversampling_t(QMP_OVERSAMPLING_2X);
	if(err < 0)
	{
		QMP_ERR("set temperature oversampling failed, err = %d\n", err);
		return err;
	}

	return 0;
}

static int qmp6988_open(struct inode *inode, struct file *file)
{
	file->private_data = (void*)g_qmp;

	if(file->private_data == NULL)
	{
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

static long qmp6988_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
//	struct qmp_i2c_data *obj = g_qmp;	//(struct qmp_i2c_data *)file->private_data;
	//struct i2c_client *client = obj->client;
	//s8 strbuf[QMP_BUFSIZE];
	s32 dat_buf[2];
	void __user *data;
	s32 delay,enable;
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
	case PRESS_IOCTL_GET_RAW_DATA:
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
	
	case PRESS_IOCTL_GET_CALI:
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

	case PRESS_IOCTL_SET_ENABLE:
		data = (void __user *)arg;
		if(NULL == data) {
			err = -EINVAL;
			break;
		}
		if(copy_from_user(&enable, data, sizeof(enable))){
			err = -EFAULT;
			break;
		}
		enable = enable ? 1 : 0;
		if(g_qmp->enable != enable)
		{
			if(enable)
			{
				qmp_set_powermode(QMP_NORMAL_MODE);
				if(g_qmp->delay > 1)
					schedule_delayed_work(&g_qmp->work,msecs_to_jiffies(g_qmp->delay));
			}
			else
			{
				qmp_set_powermode(QMP_SUSPEND_MODE);
				cancel_delayed_work_sync(&g_qmp->work);
			}
			g_qmp->enable = enable;
		}
		break;
	
	case PRESS_IOCTL_SET_DELAY:
		data = (void __user *)arg;
		if(NULL == data) {
			err = -EINVAL;
			break;
		}
		if(copy_from_user(&delay, data, sizeof(delay))){
			err = -EFAULT;
			break;
		}
		g_qmp->delay = delay;

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
static const struct file_operations qmp_fops = 
{
	.owner = THIS_MODULE,
	.open = qmp6988_open,
	.release = qmp6988_release,
	.unlocked_ioctl = qmp6988_unlocked_ioctl,
#if 0//IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_qmp6988_unlocked_ioctl,
#endif
};

static struct miscdevice qmp_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMP_DEV_NAME,
	.fops = &qmp_fops,
};

static int qmp_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	QMP_FUN();
	if(NULL == obj) 
	{
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}

	if(msg.event == PM_EVENT_SUSPEND) 
	{
		atomic_set(&obj->suspend, 1);
		err = qmp_set_powermode(QMP_SUSPEND_MODE);
		if(err)
		{
			QMP_ERR("qmp set suspend mode failed, err = %d\n", err);
			return err;
		}
	}
	return err;
}

static int qmp_resume(struct i2c_client *client)
{
	struct qmp_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	QMP_FUN();
	if(NULL == obj) 
	{
		QMP_ERR("null pointer\n");
		return -EINVAL;
	}

	err = qmp_init_client(obj->client);
	if(err)
	{
		QMP_ERR("initialize client fail\n");
		return err;
	}
	err = qmp_set_powermode(QMP_NORMAL_MODE);
	if(err)
	{
		QMP_ERR("qmp set normal mode failed, err = %d\n", err);
		return err;
	}

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
	if(!dev)
		return -ENOMEM;
	dev->name = QMP_DEV_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);	
	//input_set_abs_params(dev, ABS_X, -2147483648, 2147483648, 0, 0);
	//input_set_abs_params(dev, ABS_Y, -2147483648, 2147483648, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if(err < 0)
	{
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
	s32 err = 0;

	err = qmp_read_raw_data(&g_qmp->pressure, &g_qmp->temperature);
	if(err == 0)
	{
		input_event(g_qmp->input, EV_ABS, ABS_X, g_qmp->pressure);
		input_event(g_qmp->input, EV_ABS, ABS_Y, g_qmp->temperature);
		input_sync(g_qmp->input);
	}
	if(g_qmp->delay > 1)
		schedule_delayed_work(&g_qmp->work, msecs_to_jiffies(g_qmp->delay));
}

static ssize_t show_cali(struct device *dev,struct device_attribute *attr, char *buf)
{
	qmp_init_client(g_qmp->client);
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
		g_qmp->qmp6988_cali.COE_a0,g_qmp->qmp6988_cali.COE_a1,g_qmp->qmp6988_cali.COE_a2,
		g_qmp->qmp6988_cali.COE_b00,g_qmp->qmp6988_cali.COE_bt1,g_qmp->qmp6988_cali.COE_bt2,
		g_qmp->qmp6988_cali.COE_bp1,g_qmp->qmp6988_cali.COE_b11,
		g_qmp->qmp6988_cali.COE_bp2,g_qmp->qmp6988_cali.COE_b12,
		g_qmp->qmp6988_cali.COE_b21,g_qmp->qmp6988_cali.COE_bp3);
}

static ssize_t show_delay(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_qmp->delay);
}

static ssize_t store_delay(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int delay;
	s32 status = kstrtoint(buf, 10, &delay);

	QMP_LOG("store_delay delay=%d\n",delay);
	if((status == 0)&&(delay>0)) 
	{
		g_qmp->delay = delay;
	}

	return count;
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
			if(enable)
			{
				qmp_set_powermode(QMP_NORMAL_MODE);
				if(g_qmp->delay > 1)
					schedule_delayed_work(&g_qmp->work,msecs_to_jiffies(g_qmp->delay));
			}
			else
			{
				qmp_set_powermode(QMP_SUSPEND_MODE);
				cancel_delayed_work_sync(&g_qmp->work);
			}
			g_qmp->enable = enable;
		}
	}

	return count;
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

static ssize_t show_dump_reg(struct device *dev,struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
	int write_offset = 0;
	unsigned char databuf[2];

	write_offset = 0;
	for(i=0xa0;i<=0xff;i++)
	{
		res = qmp_i2c_read_block(g_qmp->client, i, &databuf[0], 1);
		if(res)
		{
			QMP_ERR("qmaX981 dump registers 0x%02x failed !\n", i);
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "error!\n");
		}
		else
		{			
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "0x%2x=0x%2x\n", i, databuf[0]);
		}

	}
	return write_offset;
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

	return sprintf(buf, "%d %d\n", pressure_raw,temp_raw);
}

#define QMP6988_ATTR_WR						0666
#define QMP6988_ATTR_R						0444
#define QMP6988_ATTR_W						0222

static DEVICE_ATTR(cali, 		QMP6988_ATTR_R, 	show_cali, 		NULL);
static DEVICE_ATTR(delay, 	QMP6988_ATTR_WR, 	show_delay, 	store_delay);
static DEVICE_ATTR(enable, 	QMP6988_ATTR_WR, 	show_enable, 	store_enable);
static DEVICE_ATTR(batch, 	QMP6988_ATTR_WR, 	show_batch, 	store_batch);
static DEVICE_ATTR(iptnum, 	QMP6988_ATTR_R, 	show_iptnum, 	NULL);
static DEVICE_ATTR(selftest, 	QMP6988_ATTR_R, 	show_selftest, 	NULL);
static DEVICE_ATTR(dump_reg, 	QMP6988_ATTR_R,		show_dump_reg, 	NULL);
static DEVICE_ATTR(read_reg, 	QMP6988_ATTR_WR,	show_read_reg, 	store_read_reg);
static DEVICE_ATTR(set_reg, 	QMP6988_ATTR_W, 	NULL, 			store_set_reg);
static DEVICE_ATTR(pre_temp, 	QMP6988_ATTR_R, 	show_pre_temp, 	NULL);

/*!
 * @brief device attribute files
*/
static struct attribute *qmp_attributes[] = {
	&dev_attr_cali.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_batch.attr,
	&dev_attr_iptnum.attr,
	&dev_attr_selftest.attr,
	&dev_attr_dump_reg.attr,
	&dev_attr_read_reg.attr,
	&dev_attr_set_reg.attr,
	&dev_attr_pre_temp.attr,
	NULL
};

/*!
 * @brief attribute files group
*/
static const struct attribute_group qmp_attr_group = {
	/**< bmp attributes */
	//.name = "qmp6988",
	.attrs = qmp_attributes,
};


static int qmp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct qmp_i2c_data *obj;
	int err = 0;

	QMP_FUN();
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if(!obj)
	{
		err = -ENOMEM;
		goto exit;
	}
	g_qmp = obj;
	client->addr = QMP6988_I2C_ADDRESS;
	//client->timing = 100;
	obj->client = client;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->sensor_type = QMP6988_TYPE;
	obj->power_mode = QMP_UNDEFINED_POWERMODE;
	obj->hw_filter = QMP_UNDEFINED_FILTER;
	obj->oversampling_p = QMP_UNDEFINED_OVERSAMPLING;
	obj->oversampling_t = QMP_UNDEFINED_OVERSAMPLING;
	obj->delay = 200;
	mutex_init(&obj->lock);	
	INIT_DELAYED_WORK(&obj->work, qmp_work_func);
	//init_waitqueue_head(&open_wq);
#ifdef CONFIG_QMP_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif

#if 1
	err = qmp_init_client(client);
	if(err)
		goto exit_init_client_failed;
#endif	
	err = qmp_input_init(obj);
	if(err)
		goto exit_init_client_failed;

	err = misc_register(&qmp_device);
	if(err)
	{
		QMP_ERR("misc device register failed, err = %d\n", err);
		goto exit_input_device_register_failed;
	}
	//err = sysfs_create_group(&obj->input->dev.kobj, &qmp_attr_group);
	err = sysfs_create_group(&qmp_device.this_device->kobj, &qmp_attr_group);
	if(err)
	{
		goto exit_misc_device_register_failed;
	}
	qmp_init_flag = 0;
	QMP_LOG("%s: OK\n", __func__);
	return 0;

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
	
	if(g_qmp)
	{
		//sysfs_remove_group(&g_qmp->input->dev.kobj, &qmp_attr_group);
		sysfs_create_group(&qmp_device.this_device->kobj, &qmp_attr_group);
		err = misc_deregister(&qmp_device);
		qmp_input_delete(g_qmp);
		kfree(g_qmp);
		g_qmp = NULL;
	}

	return 0;
}

#if 0
static struct i2c_board_info __initdata i2c_qmp6988={ I2C_BOARD_INFO(QMP_DEV_NAME, QMP6988_I2C_ADDRESS)};
#endif
static int __init qmp_init(void)
{
#if 0
	i2c_register_board_info(0, &i2c_qmp6988, 1);
#endif
	i2c_add_driver(&qmp_i2c_driver);

	return 0;
}

static void __exit qmp_exit(void)
{
	i2c_del_driver(&qmp_i2c_driver);
}
module_init(qmp_init);
module_exit(qmp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QMP6988 I2C Driver");
MODULE_AUTHOR("QST industries,Inc");
MODULE_VERSION(QMP_DRIVER_VERSION);

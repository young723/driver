
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
#include "barometer.h"
#include "qmp6988.h"
/* #include <linux/hwmsen_helper.h> */

typedef struct qmp6988_data
{
	int power_mode;
	int p_oversampling;
	int t_oversampling;
	int iir_filter;

	struct i2c_client *client;
	struct baro_hw hw;
	struct mutex lock;
	atomic_t trace;
	atomic_t suspend;
}qmp6988_data_t;

static int qmp6988_local_init(void);
static int qmp6988_remove(void);
static int qmp6988_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int qmp6988_i2c_remove(struct i2c_client *client);

static int qmp6988_get_data(int *value, int *status);

static int qmp6988_init_flag = -1;
static struct baro_init_info qmp6988_init_info = {
	.name = QMP6988_DEV_NAME,
	.init = qmp6988_local_init,
	.uninit = qmp6988_remove,
};

static QMP6988_S32_t _b00, _a0;
static QMP6988_S32_t _bt1, _bp1;
static QMP6988_S32_t _bt2;
static QMP6988_S32_t _b11, _bp2;
static QMP6988_S32_t _b12, _b21, _bp3;
static QMP6988_S32_t _a1, _a2;

static struct qmp6988_calibration_data qmp6988_cali;
static qmp6988_data_t *g_qmp6988=NULL;


/* I2C operation functions */
static int qmp6988_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
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

	if(rxbuf == NULL)
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
				QMP6988_ERR("i2c read register=%#x length=%d failed\n", addr + offset,
					len);
				return -EIO;
			}
		}
	}

	return 0;
}

static int qmp6988_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
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
				QMP6988_ERR("i2c write register=%#x length=%d failed\n", buffer[0],
					len);
				return -EIO;
			}

			QMP6988_LOG("i2c write addr %#x, retry %d\n", buffer[0], retry);
		}
	}

	return 0;
}


static int qmp6988_get_chip_type(void)
{
	u8 chip_id;
	
	QMP6988_FUN();
	qmp6988_i2c_read_block(g_qmp6988->client, QMP6988_CHIP_ID_REG, &chip_id, 1);
	QMP6988_LOG("chip id = 0x%x", chip_id);
	if(chip_id == 0x5c)
		return 0;
	else
		return -1;
}

static int qmp6988_get_calibration_data(void)
{
	int index = 0;
	u8 buf_wr[25];

	qmp6988_i2c_read_block(g_qmp6988->client, 0xA0, buf_wr, 25);
#if 1
	for(index=0;index<25;index++)
	{
		QMP6988_LOG("qmp6988 cali reg[%d]=0x%x \n", index, buf_wr[index]); 
	}
#endif
	qmp6988_cali.COE_a0 = (QMP6988_S32_t)((((QMP6988_U32_t)buf_wr[18] << SHIFT_LEFT_12_POSITION) \
		| ((QMP6988_U32_t)buf_wr[19] << SHIFT_LEFT_4_POSITION) \
		| ((QMP6988_U32_t)buf_wr[24] & 0x0f))<<12);
	qmp6988_cali.COE_a0 = qmp6988_cali.COE_a0>>12;

	qmp6988_cali.COE_a1 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[20] << SHIFT_LEFT_8_POSITION) | buf_wr[21]);
	qmp6988_cali.COE_a2 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[22] << SHIFT_LEFT_8_POSITION) | buf_wr[23]);

	qmp6988_cali.COE_b00 = (QMP6988_S32_t)((((QMP6988_U32_t)buf_wr[0] << SHIFT_LEFT_12_POSITION) \
		| ((QMP6988_U32_t)buf_wr[1] << SHIFT_LEFT_4_POSITION) \
		| (((QMP6988_U32_t)buf_wr[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))<<12);
	qmp6988_cali.COE_b00 = qmp6988_cali.COE_b00>>12;


	qmp6988_cali.COE_bt1 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[2] << SHIFT_LEFT_8_POSITION) | buf_wr[3]);
	qmp6988_cali.COE_bt2 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[4] << SHIFT_LEFT_8_POSITION) | buf_wr[5]);
	qmp6988_cali.COE_bp1 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[6] << SHIFT_LEFT_8_POSITION) | buf_wr[7]);
	qmp6988_cali.COE_b11 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[8] << SHIFT_LEFT_8_POSITION) | buf_wr[9]);
	qmp6988_cali.COE_bp2 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[10] << SHIFT_LEFT_8_POSITION) | buf_wr[11]);
	qmp6988_cali.COE_b12 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[12] << SHIFT_LEFT_8_POSITION) | buf_wr[13]);		
	qmp6988_cali.COE_b21 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[14] << SHIFT_LEFT_8_POSITION) | buf_wr[15]);
	qmp6988_cali.COE_bp3 = (QMP6988_S16_t)(((QMP6988_U16_t)buf_wr[16] << SHIFT_LEFT_8_POSITION) | buf_wr[17]);			

	QMP6988_LOG("<-----------calibration data-------------->\n");
	QMP6988_LOG("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
		qmp6988_cali.COE_a0,qmp6988_cali.COE_a1,qmp6988_cali.COE_a2,qmp6988_cali.COE_b00);
	QMP6988_LOG("COE_bt1[%d] COE_bt2[%d] COE_bp1[%d] COE_b11[%d]\n",
		qmp6988_cali.COE_bt1,qmp6988_cali.COE_bt2,qmp6988_cali.COE_bp1,qmp6988_cali.COE_b11);
	QMP6988_LOG("COE_bp2[%d] COE_b12[%d] COE_b21[%d] COE_bp3[%d]\n",
		qmp6988_cali.COE_bp2,qmp6988_cali.COE_b12,qmp6988_cali.COE_b21,qmp6988_cali.COE_bp3);
	QMP6988_LOG("<-----------calibration data-------------->\n");

	_b00 = qmp6988_cali.COE_b00; // 20Q4
	_bt1 = 2982L * (int64_t)qmp6988_cali.COE_bt1 + 107370906L; // 28Q15
	_bt2 = 329854L * (int64_t)qmp6988_cali.COE_bt2 + 108083093L; // 34Q38
	_bp1 = 19923L * (int64_t)qmp6988_cali.COE_bp1 + 1133836764L; // 31Q20
	_b11 = 2406L * (int64_t)qmp6988_cali.COE_b11+ 118215883L; // 28Q34
	_bp2 = 3079L * (int64_t)qmp6988_cali.COE_bp2 - 181579595L; // 29Q43
	_b12 = 6846L * (int64_t)qmp6988_cali.COE_b12 + 85590281L; // 29Q53
	_b21 = 13836L * (int64_t)qmp6988_cali.COE_b21 + 79333336L; // 29Q60
	_bp3 = 2915L * (int64_t)qmp6988_cali.COE_bp3 + 157155561L; // 28Q65
	_a0 = qmp6988_cali.COE_a0; // 20Q4
	_a1 = 3608L * (int32_t)qmp6988_cali.COE_a1 - 1731677965L; // 31Q23
	_a2 = 16889L * (int32_t) qmp6988_cali.COE_a2 - 87619360L; // 30Q47

	QMP6988_LOG("<----------- int calibration data -------------->\n");
	QMP6988_LOG("a0[%ld]	a1[%ld]	a2[%ld]	b00[%ld]\n",_a0,_a1,_a2,_b00);
	QMP6988_LOG("bt1[%ld]	bt2[%ld]	bp1[%ld]	b11[%ld]\n",_bt1,_bt2,_bp1,_b11);
	QMP6988_LOG("bp2[%ld]	b12[%ld]	b21[%ld]	bp3[%ld]\n",_bp2,_b12,_b21,_bp3);
	QMP6988_LOG("<----------- int calibration data -------------->\n");

	return 0;
}

static int qmp6988_set_powermode(int power_mode)
{
	unsigned char reg_data;

	//g_qmp6988->power_mode = power_mode;
	qmp6988_i2c_read_block(g_qmp6988->client, QMP6988_CTRLMEAS_REG, &reg_data, 1);
	reg_data = reg_data&0xfc;
	reg_data |= power_mode;
	qmp6988_i2c_write_block(g_qmp6988->client, QMP6988_CTRLMEAS_REG, &reg_data, 1);
	//g_qmp6988->power_mode = power_mode;
	//QMP6988_LOG("qmp_set_powermode 0xf4=0x%x \n", reg_data);
	mdelay(5);

	return 0;
}


static int qmp6988_set_filter(int filter)
{
	unsigned char reg_data;

	if((filter>=QMP6988_FILTERCOEFF_OFF) &&(filter<=QMP6988_FILTERCOEFF_32))
	{
		reg_data = (filter&0x07);
		qmp6988_i2c_write_block(g_qmp6988->client, QMP6988_CONFIG_REG, &reg_data, 1);
		g_qmp6988->iir_filter = filter;
	}

	return 0;
}

static int qmp6988_set_oversampling_p(int oversampling_p)
{
	unsigned char reg_data;

	qmp6988_i2c_read_block(g_qmp6988->client, QMP6988_CTRLMEAS_REG, &reg_data, 1);
	if((oversampling_p>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_p<=QMP6988_OVERSAMPLING_64X))
	{
		reg_data &= 0xe3;
		reg_data |= (oversampling_p<<2);
		qmp6988_i2c_write_block(g_qmp6988->client, QMP6988_CTRLMEAS_REG, &reg_data, 1);
		g_qmp6988->p_oversampling = oversampling_p;
	}

	return 0;
}

static int qmp6988_set_oversampling_t(int oversampling_t)
{
	unsigned char reg_data;

	qmp6988_i2c_read_block(g_qmp6988->client, QMP6988_CTRLMEAS_REG, &reg_data, 1);
	if((oversampling_t>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_t<=QMP6988_OVERSAMPLING_64X))
	{
		reg_data &= 0x1f;
		reg_data |= (oversampling_t<<5);
		qmp6988_i2c_write_block(g_qmp6988->client, QMP6988_CTRLMEAS_REG, &reg_data, 1);
		g_qmp6988->t_oversampling = oversampling_t;
	}	

	return 0;
}

static int qmp6988_read_raw_data(s32 *temp, s32 *press)
{
	int err = 0;
	int P_read, T_read;
	int T_int, P_int;
	unsigned char reg_data[6];

	err = qmp6988_i2c_read_block(g_qmp6988->client, 0xF7, reg_data, 6);
	if(err)
	{
		QMP6988_ERR("qmp6988 read raw error! \n");
		return -1;
	}

	P_read = (QMP6988_S32_t)(
		(((QMP6988_S32_t)reg_data[0]) << SHIFT_LEFT_16_POSITION) |
		(((QMP6988_S32_t)reg_data[1]) << SHIFT_LEFT_8_POSITION) |
		((QMP6988_S32_t)reg_data[2]));
	P_read = (QMP6988_S32_t)(P_read - 8388608);

	T_read = (QMP6988_S32_t)(
		(((QMP6988_S32_t)(reg_data[3])) << SHIFT_LEFT_16_POSITION) |
		(((QMP6988_S32_t)(reg_data[4])) << SHIFT_LEFT_8_POSITION) | 
		((QMP6988_S32_t)reg_data[5]));
	T_read = (QMP6988_S32_t)(T_read - 8388608);

	T_int = qmp6988_convTx_02e(T_read);
	P_int = qmp6988_get_pressure_02e(P_read, T_int);

	*temp = T_int;	//T_read;
	*press = P_int;	//P_read;

	if(g_qmp6988->power_mode == QMP6988_FORCED_MODE)
	{	
		qmp6988_set_powermode(QMP6988_FORCED_MODE);
	}

	return err;
}


void qmp6988_set_app(qmp6988_app_e app)
{
	QMP6988_LOG("qmp6988_set_app :%d\n", app);

	if(app == QMP6988_APP_WEATHER_REPORT)	//Weather monitoring
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_2X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_OFF);
	}
	else if(app == QMP6988_APP_DROP_DETECTION)		//Drop detection
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_2X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_OFF);
	}
	else if(app == QMP6988_APP_ElEVATOR_DETECTION)		//Elevator detection 
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_8X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_4);
	}
	else if(app == QMP6988_APP_STAIR_DETECTION) 	//Stair detection
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_16X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_2X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_8);
	}
	else if(app == QMP6988_APP_INDOOR_NAVIGATION)		//Indoor navigation
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_32X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_4X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_32);
	}
	else
	{
	}
}


static QMP6988_S16_t qmp6988_convTx_02e(QMP6988_S32_t dt) 
{
	QMP6988_S16_t ret;
	QMP6988_S64_t wk1, wk2;

	// wk1: 60Q4 // bit size
	wk1 = ((QMP6988_S64_t)_a1 * (QMP6988_S64_t)dt); // 31Q23+24-1=54 (54Q23)
	wk2 = ((QMP6988_S64_t)_a2 * (QMP6988_S64_t)dt) >> 14; // 30Q47+24-1=53 (39Q33)
	wk2 = (wk2 * (QMP6988_S64_t)dt) >> 10; // 39Q33+24-1=62 (52Q23)
	wk2 = ((wk1 + wk2) / 32767) >> 19; // 54,52->55Q23 (20Q04)
	ret = (QMP6988_S16_t)((_a0 + wk2) >> 4); // 21Q4 -> 17Q0
	return ret;
}

static QMP6988_S32_t qmp6988_get_pressure_02e( QMP6988_S32_t dp, QMP6988_S16_t tx)
{
	QMP6988_S32_t ret;
	QMP6988_S64_t wk1, wk2, wk3;

	// wk1 = 48Q16 // bit size
	wk1 = ((QMP6988_S64_t)_bt1 * (QMP6988_S64_t)tx); // 28Q15+16-1=43 (43Q15)
	wk2 = ((QMP6988_S64_t)_bp1 * (QMP6988_S64_t)dp) >> 5; // 31Q20+24-1=54 (49Q15)
	wk1 += wk2; // 43,49->50Q15
	wk2 = ((QMP6988_S64_t)_bt2 * (QMP6988_S64_t)tx) >> 1; // 34Q38+16-1=49 (48Q37)
	wk2 = (wk2 * (QMP6988_S64_t)tx) >> 8; // 48Q37+16-1=63 (55Q29)
	wk3 = wk2; // 55Q29
	wk2 = ((QMP6988_S64_t)_b11 * (QMP6988_S64_t)tx) >> 4; // 28Q34+16-1=43 (39Q30)
	wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1; // 39Q30+24-1=62 (61Q29)
	wk3 += wk2; // 55,61->62Q29
	wk2 = ((QMP6988_S64_t)_bp2 * (QMP6988_S64_t)dp) >> 13; // 29Q43+24-1=52 (39Q30)
	wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1; // 39Q30+24-1=62 (61Q29)
	wk3 += wk2; // 62,61->63Q29
	wk1 += wk3 >> 14; // Q29 >> 14 -> Q15
	wk2 = ((QMP6988_S64_t)_b12 * (QMP6988_S64_t)tx); // 29Q53+16-1=45 (45Q53)
	wk2 = (wk2 * (QMP6988_S64_t)tx) >> 22; // 45Q53+16-1=61 (39Q31)
	wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1; // 39Q31+24-1=62 (61Q30)
	wk3 = wk2; // 61Q30
	wk2 = ((QMP6988_S64_t)_b21 * (QMP6988_S64_t)tx) >> 6; // 29Q60+16-1=45 (39Q54)
	wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23; // 39Q54+24-1=62 (39Q31)
	wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1; // 39Q31+24-1=62 (61Q20)
	wk3 += wk2; // 61,61->62Q30
	wk2 = ((QMP6988_S64_t)_bp3 * (QMP6988_S64_t)dp) >> 12; // 28Q65+24-1=51 (39Q53)
	wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23; // 39Q53+24-1=62 (39Q30)
	wk2 = (wk2 * (QMP6988_S64_t)dp); // 39Q30+24-1=62 (62Q30)
	wk3 += wk2; // 62,62->63Q30
	wk1 += wk3 >> 15; // Q30 >> 15 = Q15
	wk1 /= 32767L;
	wk1 >>= 11; // Q15 >> 7 = Q4
	wk1 += _b00; // Q4 + 20Q4
	//wk1 >>= 4; // 28Q4 -> 24Q0
	ret = (QMP6988_S32_t)wk1;
	return ret;
}


static int qmp6988_init_client(void)
{
	if(qmp6988_get_chip_type())
	{
		QMP6988_ERR("qmp6988_get_chip_type fail!");
		return -1;
	}
	qmp6988_get_calibration_data();
	qmp6988_set_powermode(g_qmp6988->power_mode);
	qmp6988_set_app(QMP6988_APP_WEATHER_REPORT);
	
	return 0;
}

static int qmp6988_do_selftest(void)
{
	int err = 0;

	return err;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("i2c data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", "qmp6988");
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int temp, press;

	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("i2c data pointer is null\n");
		return 0;
	}

	qmp6988_read_raw_data(&temp, &press);
	return snprintf(buf, PAGE_SIZE, "%d %d\n", press, temp);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("i2c data pointer is null\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&g_qmp6988 ->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("i2c_data obj is null\n");
		return 0;
	}

	if(sscanf(buf, "0x%x", &trace) == 1)
		atomic_set(&g_qmp6988->trace, trace);
	else
		QMP6988_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("qmp6988 i2c data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", qmp6988_do_selftest());
}

static ssize_t show_dumpinfo_value(struct device_driver *ddri, char *buf)
{
	int res;
	int i, write_offset;
	unsigned char reg_value;

	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("qmp6988 i2c data pointer is null\n");
		return 0;
	}
	
	write_offset = 0;
	for(i =QMP6988_CONFIG_REG;i<=QMP6988_IO_SETUP_REG;i++)
	{
		res = qmp6988_i2c_read_block(g_qmp6988->client, (u8)i, &reg_value, 1);
		if(res)
		{
			QMP6988_ERR("fis210x dump registers 0x%02x failed !\n", i);
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "error!\n");
		}
		else
		{			
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "0x%2x=0x%2x\n", i, reg_value);
		}

	}

	write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
		qmp6988_cali.COE_a0,qmp6988_cali.COE_a1,qmp6988_cali.COE_a2,qmp6988_cali.COE_b00);
	write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "COE_bt1[%d] COE_bt2[%d] COE_bp1[%d] COE_b11[%d]\n",
		qmp6988_cali.COE_bt1,qmp6988_cali.COE_bt2,qmp6988_cali.COE_bp1,qmp6988_cali.COE_b11);
	write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "COE_bp2[%d] COE_b12[%d] COE_b21[%d] COE_bp3[%d]\n",
		qmp6988_cali.COE_bp2,qmp6988_cali.COE_b12,qmp6988_cali.COE_b21,qmp6988_cali.COE_bp3);

	return write_offset;
}

static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
	int write_offset;

	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("qmp6988 i2c data pointer is null\n");
		return 0;
	}
	
	write_offset = 0;
	write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
		qmp6988_cali.COE_a0,qmp6988_cali.COE_a1,qmp6988_cali.COE_a2,
		qmp6988_cali.COE_b00,qmp6988_cali.COE_bt1,qmp6988_cali.COE_bt2,
		qmp6988_cali.COE_bp1,qmp6988_cali.COE_b11,
		qmp6988_cali.COE_bp2,qmp6988_cali.COE_b12,
		qmp6988_cali.COE_b21,qmp6988_cali.COE_bp3);

	return write_offset;
}


static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(selftest, S_IRUGO, show_selftest_value, NULL);
static DRIVER_ATTR(dumpinfo, S_IRUGO, show_dumpinfo_value, NULL);
static DRIVER_ATTR(calidata, S_IRUGO, show_calidata_value, NULL);

static struct driver_attribute *qmp6988_attr_list[] = {
	&driver_attr_chipinfo,		/* chip information */
	&driver_attr_sensordata,	/* dump sensor data */
	&driver_attr_trace,			/* trace log */
	&driver_attr_selftest,		/* self test */
	&driver_attr_dumpinfo,		/* dump info */
	&driver_attr_calidata,		/* cali data */
};

static int qmp6988_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(qmp6988_attr_list));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, qmp6988_attr_list[idx]);
		if(err)
		{
			QMP6988_ERR("driver_create_file (%s) = %d\n",
			qmp6988_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int qmp6988_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(qmp6988_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, qmp6988_attr_list[idx]);
	}

	return err;
}


#if defined(QMP6988_CREATE_MISC_DEVICE)
static int qmp6988_open(struct inode *inode, struct file *file)
{
	file->private_data = g_qmp6988;

	if (file->private_data == NULL) {
		QMP6988_ERR("null pointer\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int qmp6988_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long qmp6988_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	//qmp6988_data_t *obj = (qmp6988_data_t *)file->private_data;
	//struct i2c_client *client = obj->client;
	char strbuf[QMP6988_BUFSIZE];
	void __user *data;
	int err = 0;
	int temp, press;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	QMP6988_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
	if(err) {
		return -EFAULT;
	}

	switch(cmd) {
	case BAROMETER_IOCTL_INIT:
		err = qmp6988_init_client();
		if (err) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		memset(strbuf, 0, sizeof(strbuf));
		strlcpy(strbuf, "qmp6988", sizeof(strbuf));
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_GET_PRESS_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		qmp6988_get_data(&press, &temp);
		if(copy_to_user(data, &press, sizeof(press))) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_GET_TEMP_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		qmp6988_get_data(&press, &temp);
		if(copy_to_user(data, &temp, sizeof(temp))) {
			err = -EFAULT;
			break;
		}
		break;
	case BAROMETER_GET_CALI:
		data = (void __user *)arg;
		QMP6988_LOG("IOCTL BAROMETER_GET_CALI\n");
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if(copy_to_user(data, &qmp6988_cali, sizeof(struct qmp6988_calibration_data))) {
			err = -EFAULT;
			break;
		}

	default:
		QMP6988_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_qmp6988_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	QMP6988_FUN();

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		QMP6988_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_BAROMETER_IOCTL_INIT:
	case COMPAT_BAROMETER_IOCTL_READ_CHIPINFO:
	case COMPAT_BAROMETER_GET_PRESS_DATA:
	case COMPAT_BAROMETER_GET_TEMP_DATA:{
			QMP6988_LOG("compat_ion_ioctl : BAROMETER_IOCTL_XXX command is 0x%x\n", cmd);
			return filp->f_op->unlocked_ioctl(filp, cmd,
							  (unsigned long)compat_ptr(arg));
		}
	default:
		QMP6988_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations qmp6988_fops = {
	.owner = THIS_MODULE,
	.open = qmp6988_open,
	.release = qmp6988_release,
	.unlocked_ioctl = qmp6988_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_qmp6988_unlocked_ioctl,
#endif
};

static struct miscdevice qmp6988_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "barometer",
	.fops = &qmp6988_fops,
};
#endif

#if 0
static int qmp6988_sensor_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	return 0;
}

static int qmp6988_sensor_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long qmp6988_sensor_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	//qmp6988_data_t *obj = (qmp6988_data_t *)file->private_data;
	//struct i2c_client *client = obj->client;
	char strbuf[QMP6988_BUFSIZE];
	void __user *data;
	int err = 0;
//	int temp, press;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	QMP6988_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
	if(err) {
		return -EFAULT;
	}

	switch(cmd) {
	case BAROMETER_IOCTL_INIT:
		err = qmp6988_init_client();
		if (err) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		memset(strbuf, 0, sizeof(strbuf));
		strlcpy(strbuf, "qmp6988", sizeof(strbuf));
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_GET_CALI:
		data = (void __user *)arg;
		QMP6988_LOG("IOCTL BAROMETER_GET_CALI\n");
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if(copy_to_user(data, &qmp6988_cali, sizeof(struct qmp6988_calibration_data))) {
			err = -EFAULT;
			break;
		}

	default:
		QMP6988_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}



static const struct file_operations qmp6988_sensor_fops = {
	.owner = THIS_MODULE,
	.open = qmp6988_sensor_open,
	.release = qmp6988_sensor_release,
	.unlocked_ioctl = qmp6988_sensor_unlocked_ioctl,
};

static struct sensor_attr_t qmp6988_sensor_dev = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "qmp6988_sensor",
	.fops = &qmp6988_sensor_fops,
};
#endif


#ifdef CONFIG_PM_SLEEP
static int qmp6988_suspend(struct device *dev)
{
	int err = 0;
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmp6988_i2c_data *obj = i2c_get_clientdata(client);
	QMP6988_FUN();
	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("null pointer\n");
		return -EINVAL;
	}
	atomic_set(&g_qmp6988->suspend, 1);

	return err;
}

static int qmp6988_resume(struct device *dev)
{
	int err=0;
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmp6988_i2c_data *obj = i2c_get_clientdata(client);

	QMP6988_FUN();
	if(g_qmp6988 == NULL)
	{
		QMP6988_ERR("null pointer\n");
		return -EINVAL;
	}
	atomic_set(&g_qmp6988->suspend, 0);

	return err;
}
#endif

#if 0
static int qmp6988_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, QMP6988_DEV_NAME, sizeof(info->type));
	return 0;
}
#endif

static int qmp6988_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

static int qmp6988_enable_nodata(int en)
{
	int res = 0;
	bool power = false;

	if(en == 1)
		power = true;

	if(en == 0)
		power = false;

	if(power)
		res = qmp6988_set_powermode(g_qmp6988->power_mode);
	else
		res = qmp6988_set_powermode(QMP6988_SLEEP_MODE);

	if(res != 0)
	{
		QMP6988_ERR("qmp6988_set_powermode fail!\n");
		return -1;
	}
	QMP6988_LOG("qmp6988_set_powermode OK!\n");

	return 0;
}


static int qmp6988_set_delay(u64 ns)
{
	return 0;
}

static int qmp6988_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return qmp6988_set_delay(samplingPeriodNs);
}

static int qmp6988_flush(void)
{
	return baro_flush_report();
}

static int qmp6988_get_data(int *value, int *status)
{
	int temp_raw, press_raw;
	int err = 0;

	err = qmp6988_read_raw_data(&temp_raw, &press_raw);
	if(err)
	{
		QMP6988_ERR("get compensated pressure value failed, err = %d\n", err);
		return -1;
	}

	*value = press_raw;
	*status = SENSOR_STATUS_ACCURACY_HIGH;

	return 0;
}

#if defined(QMP6988_GET_DATA_2)
static int qmp6988_get_data2(int *value, int *value2, int *status)
{
	int temp_raw, press_raw;
	int err = 0;

	err = qmp6988_read_raw_data(&temp_raw, &press_raw);
	if(err)
	{
		QMP6988_ERR("get compensated pressure value failed, err = %d\n", err);
		return -1;
	}

	*value = press_raw;
	*value2 = temp_raw;
	*status = SENSOR_STATUS_ACCURACY_HIGH;

	return 0;
}
#endif

#if !defined(QMP6988_CREATE_MISC_DEVICE)
static int qmp6988_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err = 0;

	err = qmp6988_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		QMP6988_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = qmp6988_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		QMP6988_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}

	return 0;
}

static int qmp6988_factory_get_data(int32_t *data)
{
	int err = 0, status = 0;

	err = qmp6988_get_data(data, &status);
	if(err < 0)
	{
		QMP6988_ERR("%s get data fail\n", __func__);
		return -1;
	}

	return 0;
}

static int qmp6988_factory_get_raw_data(int32_t *data)
{
	return 0;
}

static int qmp6988_factory_enable_calibration(void)
{
	return 0;
}

static int qmp6988_factory_clear_cali(void)
{
	return 0;
}

static int qmp6988_factory_set_cali(int32_t offset)
{
	return 0;
}

static int qmp6988_factory_get_cali(int32_t *offset)
{
	return 0;
}

static int qmp6988_factory_do_self_test(void)
{
	return 0;
}

static struct baro_factory_fops qmp6988_factory_fops = {
	.enable_sensor = qmp6988_factory_enable_sensor,
	.get_data = qmp6988_factory_get_data,
	.get_raw_data = qmp6988_factory_get_raw_data,
	.enable_calibration = qmp6988_factory_enable_calibration,
	.clear_cali = qmp6988_factory_clear_cali,
	.set_cali = qmp6988_factory_set_cali,
	.get_cali = qmp6988_factory_get_cali,
	.do_self_test = qmp6988_factory_do_self_test,
};

static struct baro_factory_public qmp6988_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &qmp6988_factory_fops,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id qmp6988_of_match[] = 
{
	{.compatible = "mediatek,barometer"},
	{.compatible = "mediatek,bar"},
	{},
};
#endif

static const struct i2c_device_id qmp6988_i2c_id[] = 
{
	{QMP6988_DEV_NAME, 0},
	{}
};

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops qmp6988_pm_ops = 
{
	SET_SYSTEM_SLEEP_PM_OPS(qmp6988_suspend, qmp6988_resume)
};
#endif

static struct i2c_driver qmp6988_i2c_driver = 
{
	.driver = 
	{
		.owner = THIS_MODULE,
		.name = QMP6988_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &qmp6988_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = qmp6988_of_match,
#endif
	},
	.probe = qmp6988_i2c_probe,
	.remove = qmp6988_i2c_remove,
	//.detect = qmp6988_i2c_detect,
	.id_table = qmp6988_i2c_id,
};

static int qmp6988_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	qmp6988_data_t *obj = NULL;
	struct baro_control_path ctl = { 0 };
	struct baro_data_path data = { 0 };
	int err = 0;

	QMP6988_FUN();
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if(!obj)
	{
		err = -ENOMEM;
		goto exit;
	}
	err = get_baro_dts_func(client->dev.of_node, &obj->hw);
	if(err < 0)
	{
		QMP6988_ERR("get cust_baro dts info fail\n");
		goto exit_init_client_failed;
	}

	g_qmp6988 = obj;
	g_qmp6988->client = client;
	i2c_set_clientdata(client, g_qmp6988);

	atomic_set(&g_qmp6988->trace, 0);
	atomic_set(&g_qmp6988->suspend, 0);
	g_qmp6988->power_mode = QMP6988_FORCED_MODE;
	g_qmp6988->iir_filter = QMP6988_FILTERCOEFF_OFF;
	g_qmp6988->p_oversampling = QMP6988_OVERSAMPLING_16X;
	g_qmp6988->t_oversampling = QMP6988_OVERSAMPLING_2X;

	mutex_init(&obj->lock);
	client->addr = 0x70;
	err = qmp6988_init_client();
	if(err)
	{
		client->addr = 0x56;
		err = qmp6988_init_client();	
	}
	if(err)
	{
		goto exit_init_client_failed;
	}

#if defined(QMP6988_CREATE_MISC_DEVICE)
	err = misc_register(&qmp6988_device);
#else
	err = baro_factory_device_register(&qmp6988_factory_device);
#endif
	if(err)
	{
		QMP6988_ERR("baro_factory device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}
#if 0
// add by yangzhiqiang for ioctl
	err = sensor_attr_register(&qmp6988_sensor_dev);
	if(err)
	{
		QMP6988_ERR("unable to register qmp6988_sensor_dev misc device!!\n");
		goto exit_create_attr_failed;
	}
// add by yangzhiqiang
#endif	
	err = qmp6988_create_attr(&(qmp6988_init_info.platform_diver_addr->driver));
	if(err)
	{
		QMP6988_ERR("create attribute failed, err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.open_report_data = qmp6988_open_report_data;
	ctl.enable_nodata = qmp6988_enable_nodata;
	ctl.set_delay = qmp6988_set_delay;
	ctl.batch = qmp6988_batch;
	ctl.flush = qmp6988_flush;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw.is_batch_supported;
	err = baro_register_control_path(&ctl);
	if(err)
	{
		QMP6988_ERR("register baro control path err\n");
		goto exit_hwmsen_attach_pressure_failed;
	}

	data.get_data = qmp6988_get_data;
#if defined(QMP6988_GET_DATA_2)
	data.get_data_2 = qmp6988_get_data2;
#endif
	data.vender_div = 16;
	err = baro_register_data_path(&data);
	if(err)
	{
		QMP6988_ERR("baro_register_data_path failed, err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}
#if 0
	err = batch_register_support_info(ID_PRESSURE, obj->hw.is_batch_supported, data.vender_div,0);
	if(err)
	{
		QMP6988_ERR("register baro batch support err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}
#endif
	qmp6988_init_flag = 0;
	QMP6988_LOG("%s: OK\n", __func__);
	return 0;

exit_hwmsen_attach_pressure_failed:
	qmp6988_delete_attr(&(qmp6988_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
	#if defined(QMP6988_CREATE_MISC_DEVICE)
	misc_deregister(&qmp6988_device);
	#endif
exit_misc_device_register_failed:
exit_init_client_failed:
	kfree(obj);
exit:
	obj = NULL;
	g_qmp6988 = NULL;
	QMP6988_ERR("err = %d\n", err);
	qmp6988_init_flag = -1;

	return err;
}

static int qmp6988_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = qmp6988_delete_attr(&(qmp6988_init_info.platform_diver_addr->driver));
	if(err)
		QMP6988_ERR("qmp6988_delete_attr failed, err = %d\n", err);

#if defined(QMP6988_CREATE_MISC_DEVICE)
	misc_deregister(&qmp6988_device);
#else
	baro_factory_device_deregister(&qmp6988_factory_device);
#endif
	i2c_unregister_device(client);
	if(g_qmp6988)
	{
		kfree(g_qmp6988);
		g_qmp6988 = NULL;
	}

	return 0;
}

static int qmp6988_remove(void)
{
	/*struct baro_hw *hw = get_cust_baro(); */

	QMP6988_FUN();
	i2c_del_driver(&qmp6988_i2c_driver);
	return 0;
}

static int qmp6988_local_init(void)
{
	if(i2c_add_driver(&qmp6988_i2c_driver))
	{
		QMP6988_ERR("add driver error\n");
		return -1;
	}
	if(-1 == qmp6988_init_flag)
		return -1;
  
	QMP6988_LOG("qmp6988_local_init done---\n");
	return 0;
}

static int __init qmp6988_init(void)
{
	QMP6988_FUN();

	baro_driver_add(&qmp6988_init_info);
	return 0;
}

static void __exit qmp6988_exit(void)
{
	QMP6988_FUN();
}

module_init(qmp6988_init);
module_exit(qmp6988_exit);


/*MODULE_LICENSE("GPLv2");*/
MODULE_DESCRIPTION("QMP6988 I2C Driver");
MODULE_AUTHOR("zhiqiang_yang@qstcorp.com");
MODULE_VERSION("V1.2");

/*****************************************************************************
 *
 * Copyright (c) QST, Inc.  All rights reserved.
 *
 * This source is subject to the QST Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of QST Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("QST Software")
 * have been modified by QST Inc. All revisions are subject to any receiver's
 * applicable license agreements with QST Inc.
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
 *
 *****************************************************************************/
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

#include "qmaX981.h"

#define QMA6981_ABS(X) 			((X) < 0 ? (-1 * (X)) : (X))
#define QMA6981_DIFF(x, y)		((x)>=(y)?((x)-(y)):((x)+65535-(y)))
//#define QMA6981_USE_CALI


static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t acc_flag = ATOMIC_INIT(0);
#if defined(QMAX981_STEP_COUNTER)
static atomic_t sc_flag = ATOMIC_INIT(0);
#endif
static unsigned char v_open_flag = 0x00;


enum qmaX981_axis {
	QMAX981_AXIS_X = 0,
	QMAX981_AXIS_Y,
	QMAX981_AXIS_Z,
	QMAX981_AXIS_NUM
};

typedef enum
{
	CHIP_TYPE_QMA6981 = 0,
	CHIP_TYPE_QMA7981,
	CHIP_TYPE_QMA6100,
	CHIP_TYPE_UNDEFINE,

	CHIP_TYPE_MAX
}qmaX981_type;

struct qmaX981_acc {
	signed short x;
	signed short y;
	signed short z;	
#ifdef QMAX981_STEP_COUNTER
	int stepcount;
#endif
};

struct qmaX981_convert {
	signed char sign[3];
	unsigned char map[3];
};

#ifdef QMA6981_STEP_COUNTER_USE_INT

#define STEP_INT_START_VLUE	4
#define STEP_END	false
#define STEP_START	true

static int STEP_DUMMY_VLUE = 16;	// QST0103 old 12

//static int cnt_step_start = 0;
//static int cnt_step_end = 0;

static atomic_t cnt_step_start = ATOMIC_INIT(0);
static atomic_t cnt_step_end = ATOMIC_INIT(0);
static atomic_t status_level = ATOMIC_INIT(0);

static bool wake_lock_status = false;
static struct wake_lock sc_wakelock;
struct qma6981_stepcount{
	//int stepcounter_pre_start;
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
bool int_statu_flag = false;
static struct qma6981_stepcount step_count_index;
#endif


struct qmaX981_data {
	atomic_t enable;                /* attribute value */
	atomic_t step_count_enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	atomic_t position;
	qmaX981_type chip_type;

	struct qmaX981_acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex op_mutex;
	
	struct i2c_client *client;
	struct input_dev *input;
	
	struct delayed_work work;
	
	unsigned int resolution;

#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend; 
#endif

#if defined(QMAX981_STEP_COUNTER)
	struct input_dev *input_sc;
	struct delayed_work sc_work;
	int sc_dely;
#endif

#ifdef QMA6981_STEP_COUNTER_USE_INT
	struct 	work_struct  eint_work;
	struct  device_node *irq_node;
	int 	irq;
#endif

};


#if defined(QMA6981_CHECK_ABNORMAL_DATA)
typedef struct
{
	int last_data;
	int curr_data;
	int more_data[3];
}qmaX981_data_check;

#define QMAX981_ABNORMAL_DIFF		30
static qmaX981_data_check g_qmaX981_data_c;
#endif

#if defined(QMA6981_USE_CALI)
//#define QMA6981_CALI_FILE		"/persist/sensors/qma6981cali.conf"
#define QMA6981_CALI_FILE		"/productinfo/qma6981cali.conf"
#define QMA6981_LSB_1G			1000			// mg
#define QMA6981_CALI_NUM		20    
static int qma6981_cali[3]={0, 0, 0};
static char qma6981_cali_flag = 0;
static void qma6981_read_file(char * filename, char *data, int len);
static void qma6981_write_file(char * filename, char *data, int len);
#endif

#if defined(QMA7981_HAND_UP_DOWN)
static int hud_layout = 0;
#endif
static struct i2c_client *this_client = NULL;

#define QMAX981_DEV_NAME			"qmax981"
#define QMAX981_DEV_VERSION			"1.0.1"
#define QMAX981_INPUT_NAME			"accelerometer"
#if defined(QMAX981_STEP_COUNTER)
#define QMAX981_SC_INPUT_NAME		"step_counter"
#endif

//#define DEBUG
#ifdef DEBUG
#define LOG_TAG				        "[QMA-Gsensor] "
#define GSE_FUN( )			        printk(KERN_INFO LOG_TAG"%s\n", __FUNCTION__)
#define GSE_LOG(fmt, args...)		printk(KERN_INFO LOG_TAG fmt, ##args)
#define GSE_ERR(fmt, args...)		printk(KERN_ERR  LOG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define LOG_TAG				       
#define GSE_FUN( )			        
#define GSE_LOG(fmt, args...)		
#define GSE_ERR(fmt, args...)
#endif
// Transformation matrix for chip mounting position


static struct qmaX981_convert g_map;

static struct qmaX981_data *g_qmaX981=NULL;

#if defined(QMAX981_STEP_COUNTER)
static int qmaX981_read_step(int *data);
static void step_c_report_rel(void);
static unsigned char step_c_ref_report = 0;
#endif

static inline int qmaX981_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	
	signed int dummy = 0;
	mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	*data = dummy & 0x000000ff;
	return 0;
}

static inline int qmaX981_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	signed int dummy = 0;
    mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static inline int qmaX981_smbus_read_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);	
	signed int dummy = 0;

	mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static inline int qmaX981_smbus_write_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	signed int dummy = 0;

	mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static int qmaX981_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
	
	mutex_lock(&g_qmaX981->op_mutex);
	ret = i2c_smbus_write_i2c_block_data(g_qmaX981->client, reg, 1, &value);
	mutex_unlock(&g_qmaX981->op_mutex);
	if(ret < 0)
	{
		return ret;
	}
	else
	{
		return 0;
	}
}


static int qmaX981_read_chip_id(struct i2c_client *client)
{	
	int res = 0;
	unsigned char chip_id;

	GSE_FUN();

	res = qmaX981_smbus_read_byte(client, QMA6981_CHIP_ID, &chip_id);
	if(res)
	{
		GSE_ERR("%s: i2c error!\n", __func__);
		return -1;
	}
	GSE_LOG("qmaX981_read_chip_id = 0x%x \n", chip_id);

	if((chip_id>=0xb0) && (chip_id<=0xb9))
	{
		g_qmaX981->chip_type = CHIP_TYPE_QMA6981;
	}
	else if((chip_id>=0xe0) && (chip_id<=0xe7))
	{
		g_qmaX981->chip_type = CHIP_TYPE_QMA7981;
	}
	else if(chip_id==0xe8)
	{
		g_qmaX981->chip_type = CHIP_TYPE_QMA6100;
	}
	else
	{
		g_qmaX981->chip_type = CHIP_TYPE_UNDEFINE;
		return -1;
	}

	return 0;
}

static int qmaX981_set_mode(struct i2c_client *client, unsigned char mode)
{
	int rc = 0;
	unsigned char data = 0;

#ifdef QMAX981_STEP_COUNTER
	return 0;
#endif
	if(mode == QMAX981_MODE_STANDBY)
	{
		data = 0x00;
		rc = qmaX981_smbus_write_byte(client,QMA6981_MODE,&data);

	}
	else if (mode == QMAX981_MODE_ACTIVE)
	{
		data = 0x80;
		rc = qmaX981_smbus_write_byte(client,QMA6981_MODE,&data);
		if((g_qmaX981->chip_type == CHIP_TYPE_QMA7981)||(g_qmaX981->chip_type == CHIP_TYPE_QMA6100))
		{
			data = 0x80;
			rc = qmaX981_smbus_write_byte(client,0x5f,&data);
			data = 0x00;
			rc = qmaX981_smbus_write_byte(client,0x5f,&data);
		}
		msleep(20);
	}

	return 0;
}

/*
{
	{ { 1, 1, 1}, {0, 1, 2} },
	{ {-1, 1, 1}, {1, 0, 2} },
	{ {-1,-1, 1}, {0, 1, 2} },
	{ { 1,-1, 1}, {1, 0, 2} },

	{ {-1, 1,-1}, {0, 1, 2} },
	{ { 1, 1,-1}, {1, 0, 2} },
	{ { 1,-1,-1}, {0, 1, 2} },
	{ {-1,-1,-1}, {1, 0, 2} },		

};
*/

void qmaX981_set_layout(int layout)
{
	if(layout == 0)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 1)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 2)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 3)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}	
	else if(layout == 4)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 5)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 6)
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else if(layout == 7)
	{
		g_map.sign[QMAX981_AXIS_X] = -1;
		g_map.sign[QMAX981_AXIS_Y] = -1;
		g_map.sign[QMAX981_AXIS_Z] = -1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
	else		
	{
		g_map.sign[QMAX981_AXIS_X] = 1;
		g_map.sign[QMAX981_AXIS_Y] = 1;
		g_map.sign[QMAX981_AXIS_Z] = 1;
		g_map.map[QMAX981_AXIS_X] = QMAX981_AXIS_X;
		g_map.map[QMAX981_AXIS_Y] = QMAX981_AXIS_Y;
		g_map.map[QMAX981_AXIS_Z] = QMAX981_AXIS_Z;
	}
}


#ifdef QMAX981_STEP_COUNTER
static int qmaX981_clear_stepcount(struct i2c_client *client)
{
	int rc = 0;
	unsigned char data = 0;
	unsigned char data_back = 0;

	rc = qmaX981_smbus_read_byte(client, 0x13, &data_back);
	data = 0x80;
	rc = qmaX981_smbus_write_byte(client,0x13,&data);
#ifdef QMA6981_STEP_COUNTER_USE_INT
	memset(&step_count_index, 0, sizeof(step_count_index));
#endif
	rc = qmaX981_smbus_write_byte(client,0x13,&data_back);

	return 0;
}
#endif

static int qmaX981_get_delay(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return atomic_read(&qmaX981->delay);
}

static void qmaX981_set_delay(struct device *dev, int delay)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);	
	int delay_last =  atomic_read(&qmaX981->delay);

	GSE_FUN();
	
	atomic_set(&qmaX981->delay, delay);

	mutex_lock(&qmaX981->enable_mutex);

	if (delay_last != delay) {
		cancel_delayed_work_sync(&qmaX981->work);
		schedule_delayed_work(&qmaX981->work, msecs_to_jiffies(delay) + 1);
	}

	mutex_unlock(&qmaX981->enable_mutex);
}

static int qma6981_initialize(struct qmaX981_data *qmaX981)
{
	int rc = 0;
	unsigned char data = 0;
	int layout=0;

	// reset all regs
	data = 0xb6;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x36,&data);
	mdelay(5);
	// reset all regs

	//0x01 range  2g , 3.9mg/LSB  @2g
	//0x04 range  8g , 15.6mg/LSB @8g
#if defined(QMAX981_STEP_COUNTER)

	data = QMA6981_RANGE_8G;
	qmaX981->resolution = 64;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x0F,&data);

	data = 0x2a;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x10,&data);

	data = 0x8f;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x12,&data);

	data = 0x0c;		//0x0d; QST0103 2019/03/06.
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x13,&data);

	data = 0x13;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x14,&data);

	data = 0x10;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x15,&data);

	data = 0x0c;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x16,&data);

#ifdef QMA6981_STEP_COUNTER_USE_INT	
	data = 0x08;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x19,&data);
#endif

	data = QMAX981_OFFSET;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x27,&data);

	data = QMAX981_OFFSET;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x28,&data);

	data = QMAX981_OFFSET;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x29,&data);
	
	//0x32,0x00 xy;0x01 yz; 0x02 xz;
	
	layout = atomic_read(&qmaX981->position);
	if(layout%2)
		data = 0x02;		// QST0103 2019/03/06
	else
		data = 0x01;		// QST0103 2019/03/06
		
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x32,&data);
	

	data = 0x80;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x11,&data);

#else
	data = QMA6981_RANGE_4G;
	qmaX981->resolution = 128;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x0F,&data);

	data = QMA6981_ODR_125HZ;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x10,&data);

	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x27,&data);

	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x28,&data);

	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x29,&data);

	data = 0x0c;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x16,&data);
	
	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x19,&data);

	data = 0x80;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x11,&data);
#endif

	return rc;
}

#if defined(QMA7981_HAND_UP_DOWN)
void qma7981_set_hand_up_down(int layout)
{
#if 1//defined(QMA7981_SWAP_XY)
	unsigned char reg_0x42 = 0;
#endif
	unsigned char reg_0x1e = 0;
	unsigned char reg_0x34 = 0;
	unsigned char yz_th_sel = 4;
	char y_th = -3; //-2;				// -16 ~ 15
	unsigned char x_th = 6; 	// 0--7.5
	char z_th = 6;				// -8--7

#if 1//defined(QMA7981_SWAP_XY)	// swap xy
	if(layout%2)
	{
		//qmaX981_readreg(0x42, &reg_0x42, 1);
		qmaX981_smbus_read_byte(g_qmaX981->client, 0x42, &reg_0x42);
		reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
		qmaX981_write_reg(0x42, reg_0x42);
	}
#endif

	if((layout >=0) && (layout<=3))
	{
		z_th = 3;
		if((layout == 2)||(layout == 3))
			y_th = 3; 
		else if((layout == 0)||(layout == 1))	
			y_th = -3;
	}
	else if((layout >=4) && (layout<=7))
	{
		z_th = -3;
		
		if((layout == 6)||(layout == 7))
			y_th = 3; 
		else if((layout == 4)||(layout == 5))	
			y_th = -3;
	}

	// 0x34 YZ_TH_SEL[7:5]	Y_TH[4:0], default 0x9d  (YZ_TH_SEL   4   9.0 m/s2 | Y_TH  -3  -3 m/s2)
	//qmaX981_write_reg(0x34, 0x9d); //|yz|>8 m/s2, y>-3 m/m2
	if((y_th&0x80))
	{
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= (y_th&0x0f)|0x10;
		qmaX981_write_reg(0x34, reg_0x34);
	}
	else
	{	
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= y_th;
		qmaX981_write_reg(0x34, reg_0x34);	//|yz|>8m/s2, y<3 m/m2
	}
	//Z_TH<7:4>: -8~7, LSB 1 (unit : m/s2)	X_TH<3:0>: 0~7.5, LSB 0.5 (unit : m/s2) 
	//qmaX981_write_reg(0x1e, 0x68); //6 m/s2, 4 m/m2

	qmaX981_write_reg(0x2a, (0x19|(0x03<<6)));			// 12m/s2 , 0.5m/s2
	qmaX981_write_reg(0x2b, (0x7c|(0x03>>2)));
	//qmaX981_write_reg(0x2a, (0x19|(0x02<<6))); 		// 12m/s2 , 0.5m/s2
	//qmaX981_write_reg(0x2b, (0x7c|(0x02)));

	//qmaX981_readreg(0x1e, &reg_0x1e, 1);
	if((z_th&0x80))
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= ((z_th<<4)|0x80);
		qmaX981_write_reg(0x1e, reg_0x1e);
	}
	else
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= (z_th<<4);
		qmaX981_write_reg(0x1e, reg_0x1e);
	}
}
#endif

#define STEP_W_TIME_L	300
#define STEP_W_TIME_H	250
static int qma7981_initialize(void)
{
	//int ret = 0;
	unsigned char reg_0x0f = 0;
	unsigned char reg_0x10 = 0;
	unsigned char reg_0x11 = 0;
#if defined(QMAX981_STEP_COUNTER)
	unsigned char reg_0x14 = 0;
	unsigned char reg_0x15 = 0;
#endif
	unsigned char reg_0x16 = 0;
	unsigned char reg_0x18 = 0;
	unsigned char reg_0x19 = 0;
	unsigned char reg_0x1a = 0;
#if defined(QMA7981_ANY_MOTION)||defined(QMA7981_NO_MOTION)
	unsigned char reg_0x2c = 0;
#endif

	qmaX981_write_reg(0x36, 0xb6);
	mdelay(5);
	qmaX981_write_reg(0x36, 0x00);
	//qmaX981_write_reg(0x11, 0x80);

	reg_0x0f = QMAX981_RANGE_4G;
	qmaX981_write_reg(0x0f, reg_0x0f);	
	if(reg_0x0f == QMAX981_RANGE_4G)
		g_qmaX981->resolution = 2048;
	else if(reg_0x0f == QMAX981_RANGE_8G)
		g_qmaX981->resolution = 1024;
	else if(reg_0x0f == QMAX981_RANGE_16G)
		g_qmaX981->resolution = 512;
	else if(reg_0x0f == QMAX981_RANGE_32G)
		g_qmaX981->resolution = 256;
	else
		g_qmaX981->resolution = 4096;

	reg_0x10 = 0xe1;
	qmaX981_write_reg(0x10, reg_0x10);
	reg_0x11 = 0x80;
	qmaX981_write_reg(0x11, reg_0x11);

	qmaX981_smbus_read_byte(g_qmaX981->client, 0x16, &reg_0x16);
	qmaX981_smbus_read_byte(g_qmaX981->client, 0x18, &reg_0x18);
	qmaX981_smbus_read_byte(g_qmaX981->client, 0x19, &reg_0x19);
	qmaX981_smbus_read_byte(g_qmaX981->client, 0x1a, &reg_0x1a);

#if defined(QMAX981_STEP_COUNTER)
	reg_0x11 = 0x80;
	reg_0x10 = 0xe1;

	if(reg_0x11 == 0x80)		// 500K
	{
		reg_0x14 = (((STEP_W_TIME_L*100)/771)+1);		// 0xe1 odr 129.7hz, 7.71ms
		reg_0x15 = (((STEP_W_TIME_H*100)/771)+1);
		if(reg_0x10 == 0xe0)		// odr 65hz
		{
			reg_0x14 = (reg_0x14>>1);
			reg_0x15 = (reg_0x15>>1);
		}
		else if(reg_0x10 == 0xe1)
		{
		}
		else if(reg_0x10 == 0xe5)	// odr 32.5hz
		{
			reg_0x14 = (reg_0x14>>2);
			reg_0x15 = (reg_0x15>>2);
		}
	}
	else if(reg_0x11 == 0x81)	// 333K
	{		
		reg_0x14 = (((STEP_W_TIME_L*100)/581)+1);	// 0xe2 odr 172.0930233 hz, 5.81ms
		reg_0x15 = (((STEP_W_TIME_H*100)/581)+1);
		if(reg_0x10 == 0xe2)
		{
		}
		else if(reg_0x10 == 0xe1)	// 86.38132296 hz
		{			
			reg_0x14 = (reg_0x14>>1);
			reg_0x15 = (reg_0x15>>1);
		}
		else if(reg_0x10 == 0xe0)		// 43.2748538
		{
			reg_0x14 = (reg_0x14>>2);
			reg_0x15 = (reg_0x15>>2);
		}
	}
	else if(reg_0x11 == 0x82)		// 200K
	{
		reg_0x14 = (((STEP_W_TIME_L*100)/967)+1);	// 0xe2 103.3591731 hz, 9.675 ms
		reg_0x15 = (((STEP_W_TIME_H*100)/967)+1);
		if(reg_0x10 == 0xe1)
		{			
			reg_0x14 = (reg_0x14>>1);		// 51.88067445 hz
			reg_0x15 = (reg_0x15>>1);
		}
		else if(reg_0x10 == 0xe2)
		{
		}
		else if(reg_0x10 == 0xe3)
		{				
			reg_0x14 = (reg_0x14<<1);		// 205.1282051 hz				
			reg_0x15 = (reg_0x15<<1);
		}
	}		
	else if(reg_0x11 == 0x83)		// 100K
	{
		reg_0x14 = (((STEP_W_TIME_L*100)/975)+1);	// 0xe3 102.5641026 hz, 9.75 ms
		reg_0x15 = (((STEP_W_TIME_H*100)/975)+1);
		if(reg_0x10 == 0xe3)
		{
		}
		else if(reg_0x10 == 0xe2)
		{
			reg_0x14 = (reg_0x14>>1);		// 51.67958656 hz
			reg_0x15 = (reg_0x15>>1);
		}
	}
	GSE_LOG("0x14[%d] 0x15[%d] \n", reg_0x14, reg_0x15);
	qmaX981_write_reg(0x10, reg_0x10);
	qmaX981_write_reg(0x11, reg_0x11);
	qmaX981_write_reg(0x12, 0x94);
	qmaX981_write_reg(0x12, 0x94);
	qmaX981_write_reg(0x13, 0x80);		// clear step
	qmaX981_write_reg(0x13, 0x7f);		// 0x7f(1/16) 0x00(1/8)
	qmaX981_write_reg(0x14, reg_0x14);		// STEP_TIME_LOW<7:0>*(1/ODR) 
	qmaX981_write_reg(0x15, reg_0x15);		// STEP_TIME_UP<7:0>*8*(1/ODR) 

	//qmaX981_write_reg(0x1f, 0x09);		// 0 step
	//qmaX981_write_reg(0x1f, 0x29);		// 4 step
	//qmaX981_write_reg(0x1f, 0x49);		// 8 step
	//qmaX981_write_reg(0x1f, 0x69);		// 12 step
	//qmaX981_write_reg(0x1f, 0x89);		// 16 step
	qmaX981_write_reg(0x1f, 0xa9);		// 24 step
	//qmaX981_write_reg(0x1f, 0xc9);		// 32 step
	//qmaX981_write_reg(0x1f, 0xe9);		// 40 step

	// step int
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
//Duration = (NO_MOT_DUR<3:0> + 1) * 1s, if NO_MOT_DUR<5:4> =b00 
//Duration = (NO_MOT_DUR<3:0> + 4) * 5s, if NO_MOT_DUR<5:4> =b01 
//Duration = (NO_MOT_DUR<3:0> + 10) * 10s, if NO_MOT_DUR<5:4> =b1x 
//ANY_MOT_DUR<1:0>: any motion interrupt will be triggered when slope > ANY_MOT_TH for (ANY_MOT_DUR<1:0> + 1) samples 

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
	qma7981_set_hand_up_down(hud_layout);
	// hand up
	reg_0x16 |= 0x02;
	reg_0x19 |= 0x02;
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
	// hand down
	// hand down
	reg_0x16 |= 0x04;
	reg_0x19 |= 0x04;
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
	// hand down	
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

	//qmaX981_write_reg(0x11, 0x80);
	qmaX981_write_reg(0x5f, 0x80);
	qmaX981_write_reg(0x5f, 0x00);
	mdelay(20);

	return 0;
}

static int qma6100_initialize(void)
{
	unsigned char reg_0x0f;

	qmaX981_write_reg(0x36, 0xb6);
	mdelay(5);
	qmaX981_write_reg(0x36, 0x00);
	qmaX981_write_reg(0x11, 0x80);

	reg_0x0f = QMAX981_RANGE_4G;
	qmaX981_write_reg(0x0f, QMAX981_RANGE_4G);	
	if(reg_0x0f == QMAX981_RANGE_4G)
		g_qmaX981->resolution = 2048;
	else if(reg_0x0f == QMAX981_RANGE_8G)
		g_qmaX981->resolution = 1024;
	else if(reg_0x0f == QMAX981_RANGE_16G)
		g_qmaX981->resolution = 512;
	else if(reg_0x0f == QMAX981_RANGE_32G)
		g_qmaX981->resolution = 256;
	else
		g_qmaX981->resolution = 4096;

	qmaX981_write_reg(0x10, 0xe0);

	qmaX981_write_reg(0x11, 0x80);
	qmaX981_write_reg(0x5f, 0x80);
	qmaX981_write_reg(0x5f, 0x00);
	mdelay(20);

	return 0;
}

static int qmaX981_read_accel_xyz(struct qmaX981_acc *acc)
{
	int res = -1;
	unsigned char data[6] = { 0 };
	s16 raw[3] = { 0 };
	int i;

	res = qmaX981_smbus_read_block(g_qmaX981->client, QMA6981_XOUTL, data, 6);
	if(res)
	{
		return res;
	}

	if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		raw[0] = (s16)((data[1]<<2) |( data[0]>>6));
		raw[1] = (s16)((data[3]<<2) |( data[2]>>6));
		raw[2] = (s16)((data[5]<<2) |( data[4]>>6));

		for(i=0;i<3;i++)
		{
			if ( raw[i] == 0x0200 )
				raw[i]= -512;
			else if ( raw[i] & 0x0200 ){
				raw[i] -= 0x1;	
				raw[i] = ~raw[i];
				raw[i] &= 0x01ff;
				raw[i] = -raw[i];
			}
		}

#ifdef QMAX981_STEP_COUNTER
		raw[0] -= QMAX981_OFFSET;
		raw[1] -= QMAX981_OFFSET;
		raw[2] -= QMAX981_OFFSET;
#endif
	}
	else if((g_qmaX981->chip_type == CHIP_TYPE_QMA7981)||(g_qmaX981->chip_type == CHIP_TYPE_QMA6100))
	{	
		raw[0] = (short)(((short)data[1]<<8)|(data[0]));
		raw[1] = (short)(((short)data[3]<<8)|(data[2]));
		raw[2] = (short)(((short)data[5]<<8)|(data[4]));
		raw[0] = raw[0]>>2;
		raw[1] = raw[1]>>2;
		raw[2] = raw[2]>>2;
	}
	
	raw[0] = ((raw[0]*1000)/g_qmaX981->resolution);
	raw[1] = ((raw[1]*1000)/g_qmaX981->resolution);
	raw[2] = ((raw[2]*1000)/g_qmaX981->resolution);
	GSE_LOG("%s_A: %d %d %d\n",__func__,raw[0],raw[1],raw[2]);


	acc->x = g_map.sign[QMAX981_AXIS_X] * raw[g_map.map[QMAX981_AXIS_X]];
	acc->y = g_map.sign[QMAX981_AXIS_Y] * raw[g_map.map[QMAX981_AXIS_Y]];
	acc->z = g_map.sign[QMAX981_AXIS_Z] * raw[g_map.map[QMAX981_AXIS_Z]];
	
#if defined(QMA6981_USE_CALI)
	acc->x += qma6981_cali[QMAX981_AXIS_X];
	acc->y += qma6981_cali[QMAX981_AXIS_Y];
	acc->z += qma6981_cali[QMAX981_AXIS_Z];
#endif
	
	GSE_LOG("%s_B: %d %d %d\n",__func__,acc->x,acc->y,acc->z);

	return res;
}


#ifdef QMA6981_STEP_COUNTER_USE_INT
static void qma6981_set_wakelock(int en)
{
	#if 1
	if((en == 1)&&(wake_lock_status == false))
	{
		wake_lock(&sc_wakelock);
		wake_lock_status = true;
		GSE_LOG("yzqlock enable wakelock\n");
	}
	else if((en == 0)&&(wake_lock_status == true))
	{
		wake_unlock(&sc_wakelock);		
		wake_lock_status = false;
		GSE_LOG("yzqlock disable wakelock\n");
	}
	#endif
}

/*
void QMA6981_eint_func(void)
{
	if(qmaX981 == NULL)
	{
		return;
	}	

	schedule_work(&qmaX981->eint_work);
}*/

static irqreturn_t QMA6981_eint_handler(int irq, void *desc)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	
	disable_irq_nosync(irq);		// add by yangzhiqiang 2017/03/02

	schedule_work(&qmaX981->eint_work);

	return IRQ_HANDLED;
} 


int QMA6981_setup_eint(struct i2c_client *client)
{
	int err = 0;
	struct qmaX981_data *obj = i2c_get_clientdata(client);        

	memset(&step_count_index, 0, sizeof(step_count_index));
	gpio_request(QMAX981_IRQ_NUMBER, "gsensor_irq_pin");
	gpio_direction_input(QMAX981_IRQ_NUMBER);	
	obj->irq = gpio_to_irq(QMAX981_IRQ_NUMBER);
	// QST0103 2018/03/06
	err = request_irq(obj->irq, QMA6981_eint_handler, IRQF_TRIGGER_RISING |IRQF_NO_SUSPEND, "gsensor_irq_pin", NULL);
	//err = request_irq(obj->irq, QMA6981_eint_handler, IRQF_TRIGGER_HIGH|IRQF_NO_SUSPEND, "gsensor_irq_pin", NULL);
	//enable_irq_wake(obj->irq);
	disable_irq(obj->irq);
	enable_irq(obj->irq);
    return 0;
}

static void QMA6981_eint_work(struct work_struct *work)
{
	//struct qmaX981_data *obj = container_of((struct delayed_work *)work, struct qmaX981_data, work);//error
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	//int gpio39_status = -1;	
	int res = 0,temp = 0;
	int retry;
	unsigned char databuf[6]; 
	u16 data = 0;
//	gpio39_status = gpio_get_value(QMAX981_IRQ_NUMBER);

	//enable_irq(qmaX981->irq);
	qma6981_set_wakelock(1);
	if(qmaX981 && qmaX981->client)
	{
		retry = 0;
		while(retry < 5)
		{
			res	= qmaX981_smbus_read_block(qmaX981->client, QMA6981_STEPCOUNT, databuf, 2);
			if(res == 0)
			{
				break;
			}
			retry++;
		}
		if(res)
		{
			GSE_LOG("QMA6981_eint_work read i2c error!");
			enable_irq(qmaX981->irq);
			qma6981_set_wakelock(0);
			return;
		}
	}
	else
	{
		enable_irq(qmaX981->irq);
		qma6981_set_wakelock(0);
		return;
	}
	qma6981_set_wakelock(0);
	data = (u16)(databuf[1]<<8)|databuf[0];

	if(int_statu_flag)
	{	
		int_statu_flag = false;
		step_count_index.stepcounter_next_end = data;
		step_count_index.stepcounter_statu = STEP_END;
		//cnt_step_end ++;
		atomic_add(1,&cnt_step_end);
		atomic_set(&status_level,0);
		GSE_LOG("qma6981** stepcounter_next_end = %d stepcounter_next_start = %d\n",step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start);
		if (step_count_index.stepcounter_next_end < step_count_index.stepcounter_next_start)
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start;
		}
		GSE_LOG("qma6981**step_int_end  temp =%d,step_count_index.step_diff =%d\n" ,temp,step_count_index.step_diff);
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
		
		GSE_LOG("qma6981**step_int_end  stepcounter_pre_end = %d , stepcounter_next_end = %d stepcounter_next_start = %d,step_count_index.step_diff =%d\n",	step_count_index.stepcounter_pre_end ,step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start,step_count_index.step_diff );
		//res = request_irq(qmaX981->irq, QMA6981_eint_handler, IRQF_TRIGGER_RISING, "gsensor-eint", NULL);
		//irq_set_irq_type(qmaX981->irq,IRQF_TRIGGER_HIGH);
		irq_set_irq_type(qmaX981->irq,IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND);				// |IRQF_NO_SUSPEND
		GSE_LOG("qma6981**step_int_end\n" );
	}
	else
	{
		int_statu_flag = true;
		step_count_index.stepcounter_next_start= data;
		step_count_index.stepcounter_statu = STEP_START;
		//cnt_step_start ++;
		atomic_add(1,&cnt_step_start);
		atomic_set(&status_level,1);
		GSE_LOG("qma6981** stepcounter_next_start = %d stepcounter_pre_end = %d\n", step_count_index.stepcounter_next_start,step_count_index.stepcounter_pre_end);
		if (step_count_index.stepcounter_next_start < step_count_index.stepcounter_pre_end)
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end;
		}
		GSE_LOG("qma6981**step_int_start  temp =%d,step_count_index.step_diff =%d\n" ,temp,step_count_index.step_diff);
		if (temp >STEP_INT_START_VLUE)
		{
			step_count_index.step_diff += (temp - STEP_INT_START_VLUE);
		}
		GSE_LOG("qma6981**start  stepcounter_pre_end = %d , stepcounter_next_end = %d stepcounter_next_start = %d,step_count_index.step_diff =%d\n",	step_count_index.stepcounter_pre_end ,step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start,step_count_index.step_diff );
		//irq_set_irq_type(qmaX981->irq,IRQF_TRIGGER_LOW);
		irq_set_irq_type(qmaX981->irq,IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND);	//|IRQF_NO_SUSPEND
		GSE_LOG("step_int_start\n" );
	}
	
	//disable_irq(qmaX981->irq);
	enable_irq(qmaX981->irq);
	return;
}

#endif

#if defined(QMA6981_CHECK_ABNORMAL_DATA)
int qmaX981_check_abnormal_data(int data_in, int *data_out)
{
	int ret = 0;
	int step_num=0;
	int diff;
	//unsigned char data[2];

	g_qmaX981_data_c.curr_data = data_in;
	diff = QMA6981_DIFF(g_qmaX981_data_c.curr_data, g_qmaX981_data_c.last_data);
	if(diff > QMAX981_ABNORMAL_DIFF)
	{
		// read data 1
		ret = qmaX981_read_step(&step_num);
		if(ret)
			g_qmaX981_data_c.more_data[0] = -1;
		else
			g_qmaX981_data_c.more_data[0] = step_num;	//((data[1]<<8) |( data[0]));
		
		// read data 2
		ret = qmaX981_read_step(&step_num);
		if(ret)
			g_qmaX981_data_c.more_data[1] = -1;
		else
			g_qmaX981_data_c.more_data[1] = step_num;	//((data[1]<<8) |( data[0]));


		// read data 3
		ret = qmaX981_read_step(&step_num);
		if(ret)
			g_qmaX981_data_c.more_data[2] = -1;
		else
			g_qmaX981_data_c.more_data[2] = step_num;	//((data[1]<<8) |( data[0]));


		if((g_qmaX981_data_c.more_data[0]<0)||(g_qmaX981_data_c.more_data[1]<0)||(g_qmaX981_data_c.more_data[2]<0))
		{
			*data_out = g_qmaX981_data_c.last_data;//cxg
			return -1;
		}
		
		//if((QMA6981_ABS(g_qmaX981_data_c.more_data[0]-g_qmaX981_data_c.curr_data) > 1)
		//	||(QMA6981_ABS(g_qmaX981_data_c.more_data[1]-g_qmaX981_data_c.curr_data) > 1)
		//	||(QMA6981_ABS(g_qmaX981_data_c.more_data[2]-g_qmaX981_data_c.curr_data) > 1)
		//	)
		if((g_qmaX981_data_c.more_data[0]==g_qmaX981_data_c.more_data[1])
			&&(g_qmaX981_data_c.more_data[1]==g_qmaX981_data_c.more_data[2]))
		{		
			*data_out = g_qmaX981_data_c.more_data[0];
			g_qmaX981_data_c.last_data = g_qmaX981_data_c.more_data[0];
		}
		else
		{
			*data_out = g_qmaX981_data_c.last_data;
			return -1;
		}
	}
	else
	{
		g_qmaX981_data_c.last_data = g_qmaX981_data_c.curr_data;
		*data_out = g_qmaX981_data_c.last_data;//cxg
	}

	return 0;
}
#endif

static void acc_work_func(struct work_struct *work)
{
	static struct qmaX981_acc acc = { 0 };
	struct qmaX981_data *qmaX981 = container_of((struct delayed_work *)work, struct qmaX981_data, work);
	int comres = -1;

	GSE_FUN();
	comres = qmaX981_read_accel_xyz(&acc);
	if(comres)
	{
		comres = qmaX981_read_accel_xyz(&acc);
		if(comres)
		{		
			schedule_delayed_work(&qmaX981->work, msecs_to_jiffies(atomic_read(&qmaX981->delay)));
			return;
		}
	}
	
	qmaX981->value.x = acc.x;
	qmaX981->value.y = acc.y;
	qmaX981->value.z = acc.z;
	
	input_report_abs(qmaX981->input, ABS_X, acc.x);
	input_report_abs(qmaX981->input, ABS_Y, acc.y);
	input_report_abs(qmaX981->input, ABS_Z, acc.z);
	input_report_abs(qmaX981->input, ABS_THROTTLE, 3);
	
	input_sync(qmaX981->input);
		
	GSE_LOG("%s: [%d %d %d ]\n",__func__,acc.x,acc.y,acc.z);
	schedule_delayed_work(&qmaX981->work, msecs_to_jiffies(atomic_read(&qmaX981->delay)));
}


#if defined(QMAX981_STEP_COUNTER)
static int qmaX981_read_step(int *data)
{
	int res = 0;
	unsigned char buff[3];
	int step_num = 0;

	if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		res = qmaX981_smbus_read_block(g_qmaX981->client, QMA6981_STEPCOUNT, buff, 2);
		if(res)
			return res;

		step_num = (int)(((unsigned short)buff[1]<<8)|buff[0]);
	}
	else if(g_qmaX981->chip_type == CHIP_TYPE_QMA7981)
	{
		res = qmaX981_smbus_read_block(g_qmaX981->client, QMA6981_STEPCOUNT, buff, 2);
		if(res)
			return res;
		
		res = qmaX981_smbus_read_block(g_qmaX981->client, 0x0e, &buff[2], 1);
		if(res)
			return res;
		
		step_num = (((int)buff[2]<<16)|((int)buff[1]<<8)|buff[0]);
	}
	g_qmaX981->value.stepcount = step_num;
	*data = step_num;

	return 0;
}

static void step_c_report_rel(void){
	
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);	

	if(step_c_ref_report == 1)
	{
		step_c_ref_report = 0;
		input_report_rel(qmaX981->input_sc, REL_X, qmaX981->value.stepcount);
		input_sync(qmaX981->input_sc);
	}
}

static void step_c_work_func(struct work_struct *work){
	int ret;
	struct qmaX981_data *qmaX981;	
	int resut = 0;
	int resut__ext = 0;
#ifdef QMA6981_STEP_COUNTER_USE_INT 
	int tempp = 0;
#endif

	//GSE_FUN();
	qmaX981 = container_of((struct delayed_work *)work, struct qmaX981_data, sc_work);

	ret = qmaX981_read_step(&resut);
	if(ret)
	{
		ret = qmaX981_read_step(&resut);
		if(ret)
		{		
			schedule_delayed_work(&qmaX981->sc_work, msecs_to_jiffies(qmaX981->sc_dely));
			return;
		}
	}
#if defined(QMA6981_CHECK_ABNORMAL_DATA)
	qmaX981_check_abnormal_data(resut, &resut);
#endif
//	GSE_LOG("%s: %d\n",__func__,resut);

#ifdef QMA6981_STEP_COUNTER_USE_INT
	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		if (resut < step_count_index.stepcounter_pre)
		{
			step_count_index.back++;
			resut__ext = resut- step_count_index.stepcounter_pre + 65536;
			step_count_index.stepcounter_pre = resut;
		}
		else
		{
			//nothing
			step_count_index.stepcounter_pre = resut;
			resut__ext = resut;
		}

		if (step_count_index.stepcounter_statu == STEP_START)
		{	
			if (resut__ext >= step_count_index.stepcounter_next_start)
			{
				tempp = resut__ext - step_count_index.stepcounter_next_start + 4;
			}
			else
			{
				tempp = resut__ext - step_count_index.stepcounter_next_start +65540;
			}

			GSE_LOG("ReadStepCounter_running resut__ext= %d,stepcounter_next_start = %d,tempp = %d,stepcounter_pre_end =%d \n",resut__ext,step_count_index.stepcounter_next_start,tempp,step_count_index.stepcounter_pre_end);
			GSE_LOG("ReadStepCounter_running 00 step_count_index.step_diff =%d\n" ,step_count_index.step_diff);
			if (tempp < (STEP_INT_START_VLUE+STEP_DUMMY_VLUE))
			{
				resut__ext = step_count_index.stepcounter_pre_fix;
				GSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
			}
			else
			{
				if (step_count_index.step_diff >resut__ext)
				{
					step_count_index.step_diff = 0;
				}
				else
				{
					resut__ext = resut__ext -  step_count_index.step_diff;
					step_count_index.stepcounter_pre_fix = resut__ext ;
					GSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
				}
			}
		
		}
		else 
		{
			GSE_LOG("ReadStepCounter_running 11 step_count_index.step_diff =%d\n" ,step_count_index.step_diff);
			// add by yangzhiqiang for step_diff
			if(step_count_index.step_diff > resut__ext)
			{
				step_count_index.step_diff = resut__ext;
			}
			// yangzhiqiang for step_diff		

			step_count_index.stepcounter_pre_end = resut__ext;

			resut__ext  = resut__ext -  step_count_index.step_diff;
			step_count_index.stepcounter_pre_fix = resut__ext;
			GSE_LOG("ReadStepCounter_end stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
		}
		GSE_LOG("ReadStepCounter=%d, step_diff= %d\n",resut__ext,step_count_index.step_diff );
	}
	else
	{
		resut__ext = resut;
	}
#else
	resut__ext = resut;
#endif

	qmaX981->value.stepcount = resut__ext;

	step_c_report_rel();
	input_report_abs(qmaX981->input_sc, ABS_X, qmaX981->value.stepcount);
	input_sync(qmaX981->input_sc);
	
	GSE_LOG("%s: step_value: %d \n",__func__,resut__ext);
	schedule_delayed_work(&qmaX981->sc_work, msecs_to_jiffies(qmaX981->sc_dely));
}
#endif

static int qmaX981_input_init(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = NULL;
	int err = 0;

	GSE_LOG("%s called\n", __func__);
	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}

	dev->name = QMAX981_INPUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_capability(dev, EV_ABS, ABS_THROTTLE);
	
	input_set_abs_params(dev, ABS_X, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_8G, ABSMAX_8G, 0, 0);
	
	input_set_abs_params(dev, ABS_THROTTLE, 0, 3, 0, 0);
	
	input_set_drvdata(dev, qmaX981);

	err = input_register_device(dev);
	
	if (err < 0) {
		pr_err("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}
	
	qmaX981->input = dev;

	return 0;
}


static void qmaX981_input_deinit(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = qmaX981->input;

	GSE_LOG("%s called\n", __func__);
	input_unregister_device(dev);
	
	input_free_device(dev);
}

#ifdef QMAX981_STEP_COUNTER
static int qmaX981_sc_input_init(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = NULL;
	int err = 0;

	GSE_LOG("%s called\n", __func__);
	
	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}
	dev->name = QMAX981_SC_INPUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_REL, REL_X);
	input_set_abs_params(dev, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(dev, REL_X, 0, 65535, 0, 0);
	
	input_set_drvdata(dev, qmaX981);

	err = input_register_device(dev);
	
	if (err < 0) {
		pr_err("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}
	
	qmaX981->input_sc = dev;

	return 0;
}

static void qmaX981_sc_input_deinit(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = qmaX981->input_sc;

	GSE_LOG("%s called\n", __func__);

	input_unregister_device(dev);
	
	input_free_device(dev);
}

#endif

static int qmaX981_get_enable(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return atomic_read(&open_flag);
}

static int qmaX981_set_enable(struct device *dev, int enable)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_LOG("%s: enable :%d\n",__func__,enable);

	if (enable) 
	{
#if defined(QMA6981_USE_CALI)
		qma6981_read_file(QMA6981_CALI_FILE, (char *)(qma6981_cali), sizeof(qma6981_cali));
#endif
#if defined(QMAX981_STEP_COUNTER)
		step_c_ref_report = 1;
#endif
		//step_c_report_rel();//input rel data after enabled
		atomic_set(&acc_flag, 1);	
		if(atomic_read(&open_flag) == 0){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_ACTIVE);
		}
		v_open_flag |= 0x01;
		schedule_delayed_work(&qmaX981->work,msecs_to_jiffies(atomic_read(&qmaX981->delay)));
	}else{
		atomic_set(&acc_flag, 0);
		v_open_flag &= 0x3e;
		if(atomic_read(&open_flag)){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY);
		}	
		cancel_delayed_work_sync(&qmaX981->work);		
	}
	
	if(v_open_flag == 0){
		qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY); //disable for step counter
	}

	atomic_set(&open_flag, v_open_flag);
	GSE_LOG("%s: open_flag :%d \n",__func__,atomic_read(&open_flag));
	return 0;
}

#ifdef QMAX981_STEP_COUNTER
static int qmaX981_get_stepcount_enable(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return atomic_read(&open_flag);
}


static int qmaX981_set_stepcount_enable(struct device *dev, int enable)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	
	GSE_LOG("%s: enable :%d\n",__func__,enable);

	if (enable) 
	{
	// add by yangzhiqiang
		step_c_ref_report = 1;
	// add by yangzhiqiang
		atomic_set(&sc_flag, 1);	
		if(atomic_read(&open_flag) == 0){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_ACTIVE);
		}
		v_open_flag |= 0x02;
		schedule_delayed_work(&qmaX981->sc_work,msecs_to_jiffies(qmaX981->sc_dely));
	}else{
		atomic_set(&sc_flag, 0);
		
		if(atomic_read(&open_flag)){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY);
		}
		v_open_flag &= 0x3d;		
		cancel_delayed_work_sync(&qmaX981->sc_work);		
	}
	
	if(v_open_flag == 0){
		qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY); //disable for step counter
	}
	
	atomic_set(&open_flag, v_open_flag);
	GSE_LOG("%s: open_flag :%d sc_flag %d\n",__func__,atomic_read(&open_flag),atomic_read(&sc_flag));
	return 0;
}
#endif

#ifdef QMAX981_STEP_COUNTER
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_power_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf=0;
	int ret =0;

	ret = qmaX981_smbus_read_byte(g_qmaX981->client, 0x11, &databuf);
	if(ret)
		return sprintf(buf, "show_sc_power_value error\n");
	else
		return sprintf(buf, "Stepcounter power status 0x%2x\n", databuf);
}

static ssize_t store_sc_power_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(g_qmaX981->client,0x11,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_count_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf=0;
	int ret =0;

	ret = qmaX981_smbus_read_byte(g_qmaX981->client, 0x12, &databuf);

	if(ret)
		return sprintf(buf, "show_sc_count_value error\n");
	else
		return sprintf(buf, "Stepcounter sample count 0x%2x\n", databuf);
}

static ssize_t store_sc_count_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(g_qmaX981->client,0x12,&data);
		if(ret)
			GSE_ERR("store_sc_count_value error!\n");
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_recision_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf=0;
	int ret =0;

	ret = qmaX981_smbus_read_byte(g_qmaX981->client, 0x13, &databuf);
	if(ret)
		return sprintf(buf, "show_sc_recision_value error\n");
	else
		return sprintf(buf, "Stepcounter precision 0x%2x\n", databuf);
}

static ssize_t store_sc_recision_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(g_qmaX981->client,0x13,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_timelow_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf=0;
	int ret =0;

	ret = qmaX981_smbus_read_byte(g_qmaX981->client, 0x14, &databuf);

	if(ret)
		return sprintf(buf, "show_sc_timelow_value error\n");
	else
		return sprintf(buf, "Stepcounter time low 0x%2x\n", databuf);
}

static ssize_t store_sc_timelow_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(g_qmaX981->client,0x14,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_timeup_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf=0;
	int ret =0;

	ret = qmaX981_smbus_read_byte(g_qmaX981->client, 0x15, &databuf);

	if(ret)
		return sprintf(buf, "show_sc_timeup_value error\n");
	else
		return sprintf(buf, "Stepcounter time up 0x%2x\n", databuf);
}

static ssize_t store_sc_timeup_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(g_qmaX981->client,0x15,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_axis_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf = 0;
	int ret=0;

	ret = qmaX981_smbus_read_byte(g_qmaX981->client, 0x32, &databuf);

	if(ret)
		return sprintf(buf, "show_sc_axis_value error\n");
	else
		return sprintf(buf, "Stepcounter saxis is 0x%2x\n", databuf);
}


static ssize_t store_sc_axis_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data=0;
	
	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(g_qmaX981->client,0x32,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t qmaX981_sc_value_show(struct device *dev,
		struct device_attribute *attr, char *buf){
	int step_value=0;

	qmaX981_read_step(&step_value);
	
	return sprintf(buf, "step (%d)\n",step_value);			
}

static ssize_t qmaX981_clear_stepcount_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0;
	int error = 0;

	GSE_FUN();

	if(1 == sscanf(buf, "%x", &value))
	{
		error = qmaX981_clear_stepcount(g_qmaX981->client);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

static ssize_t show_sc_interrupt_num(struct device *dev,
		struct device_attribute *attr, char *buf){
	
	char strbuf[256];

	sprintf(strbuf, "%d,%d,%d", atomic_read(&cnt_step_start),atomic_read(&cnt_step_end),atomic_read(&status_level));

	return sprintf(buf, "%s\n", strbuf);	
}

static ssize_t qmaX981_enable_sc_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int value;

	if(1 == sscanf(buf, "%d", &value))
	{
		if(value)
			qmaX981_set_stepcount_enable(&(g_qmaX981->client->dev), 1);
		else
			qmaX981_set_stepcount_enable(&(g_qmaX981->client->dev), 0);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
#endif

static ssize_t show_init_acc_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;

	err = qmaX981_read_chip_id(this_client);
	if(err < 0) {
		GSE_ERR("%s: qmaX981 read id fail!\n", __func__);
	}
	if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
		qma6981_initialize(g_qmaX981);
	else if(g_qmaX981->chip_type == CHIP_TYPE_QMA7981)
		qma7981_initialize();
	else if(g_qmaX981->chip_type == CHIP_TYPE_QMA6100)
		qma6100_initialize();

	return sprintf(buf, "init done!\n");
}

static ssize_t show_chipinfo_value(struct device *dev,
		struct device_attribute *attr, char *buf){

	if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
		return sprintf(buf, "qma6981\n");
	else if(g_qmaX981->chip_type == CHIP_TYPE_QMA7981)
		return sprintf(buf, "qma7981\n");
	else if(g_qmaX981->chip_type == CHIP_TYPE_QMA6100)
		return sprintf(buf, "qma6100\n");
	else
		return sprintf(buf, "unknow!\n");
}

static ssize_t show_waferid_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	int res;
	
	unsigned int chipid=0;
	unsigned char chipidh=0;
	unsigned char chipidl=0;
	
	unsigned char waferid=0;
	unsigned char waferid1=0;
	unsigned char waferid2=0;
	unsigned char waferid3=0;

	if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x48, &chipidh);
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x47, &chipidl);
		GSE_LOG("read wafer chip H:0x%x L:0x%x", chipidh, chipidl);
		chipid = (chipidh<<8)|chipidl;
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x59, &waferid1);
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x41, &waferid2);
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x40, &waferid3);
		GSE_LOG("wafer ID: 0x%x 0x%x 0x%x\n", waferid1, waferid2, waferid3);
		waferid = (waferid1&0x10)|((waferid2>>4)&0x0c)|((waferid3>>6)&0x03);
	}
	else if((g_qmaX981->chip_type == CHIP_TYPE_QMA7981)||(g_qmaX981->chip_type == CHIP_TYPE_QMA6100))
	{
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x48, &chipidh);
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x47, &chipidl);
		GSE_LOG("read wafer chip H:0x%x L:0x%x", chipidh, chipidl);
		chipid = (chipidh<<8)|chipidl;
		res = qmaX981_smbus_read_byte(g_qmaX981->client, 0x5a, &waferid1);
		waferid = waferid1&0x1f;
	}

	return sprintf(buf, " Chip id:0x%x \n Wafer ID 0x%02x\n", chipid, waferid);			
}

static ssize_t show_sensordata_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct qmaX981_acc acc;
	
	qmaX981_read_accel_xyz(&acc);

	return sprintf(buf, "(%d %d %d)\n",acc.x,acc.y,acc.z);
}
		
static ssize_t show_dumpallreg_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
	char strbuf[600];
	char tempstrbuf[24];
	unsigned char databuf[2]={0};
	int length=0;

	GSE_FUN();
	/* Check status register for data availability */
	for(i=0;i<=50;i++)
	{
		databuf[0] = i;
		res = qmaX981_smbus_read_byte(g_qmaX981->client, databuf[0], &databuf[1]);
		if(res < 0)
			GSE_LOG("qma6981 dump registers 0x%02x failed !\n", i);

		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "0x%2x=0x%2x\n",i, databuf[1]);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "%s",tempstrbuf);
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}

static ssize_t show_dumpallreg2_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
	char strbuf[600];
	char tempstrbuf[24];
	unsigned char databuf[2]={0};
	int length, len;

	length = 0;
	GSE_FUN();
	/* Check status register for data availability */
	for(i=51;i<=100;i++)
	{
		databuf[0] = i;
		res = qmaX981_smbus_read_byte(g_qmaX981->client, databuf[0], &databuf[1]);
		if(res < 0)
			GSE_LOG("qma6981 dump registers 0x%02x failed !\n", i);

		len = scnprintf(tempstrbuf, sizeof(tempstrbuf), "0x%2x=0x%2x\n",i, databuf[1]);
		snprintf(strbuf+length, sizeof(strbuf)-length, "%s",tempstrbuf);
		length += len;
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}


/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	GSE_FUN();

	return sprintf(buf, "%d\n",atomic_read(&g_qmaX981->position));
}

static ssize_t store_layout_value(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int position = 0;

	GSE_FUN();

	if(1 == sscanf(buf, "%d", &position))
	{
		if ((position >= 0) && (position <= 7))
		{
			atomic_set(&g_qmaX981->position, position);
			qmaX981_set_layout(position);
		}
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t qmaX981_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	GSE_FUN();

	return sprintf(buf, "%d\n", atomic_read(&g_qmaX981->enable));
}

static ssize_t qmaX981_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int enable=0;

	GSE_FUN();
	
	if(1 == sscanf(buf, "%d", &enable))
	{
		if (enable)
			qmaX981_set_enable(&(this_client->dev), 1);
		else
			qmaX981_set_enable(&(this_client->dev), 0);
	}
	else
	{	
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t qmaX981_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	GSE_FUN();

	return sprintf(buf, "%d\n", atomic_read(&g_qmaX981->delay));
}

static ssize_t qmaX981_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int delay=0;

	GSE_FUN();

	
	if(1 == sscanf(buf, "%d", &delay))
	{
		if(delay > QMAX981_MAX_DELAY)
			delay = QMAX981_MAX_DELAY;
		else if(delay <= 1)
			delay = 0;

		qmaX981_set_delay(&(this_client->dev),delay);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}
	
	return count;
}

static unsigned char qma6981_debug_reg_addr=0x00;
//static unsigned char qma6981_debug_reg_value=0x00;
static unsigned char qma6981_debug_read_len=0x01;

static ssize_t qmaX981_getreg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int res = 0;	
	unsigned char data=0xff;	

	GSE_FUN();
	res = qmaX981_smbus_read_byte(this_client, qma6981_debug_reg_addr, &data);

	return sprintf(buf, "0x%x\n",  data);
}

static ssize_t qmaX981_setreg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int addr, value;
	unsigned char data;	
	int res = 0;	
	
	GSE_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);	
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))	
	{		
		GSE_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);		
		qma6981_debug_reg_addr = (unsigned char)addr;		
		qma6981_debug_read_len = 1;		
		data = (unsigned char)value;		
		res = qmaX981_smbus_write_byte(this_client,qma6981_debug_reg_addr,&data);		
		if(res)		
		{			
			GSE_ERR("write reg 0x%02x fail\n", addr);		
		}	
	}	
	else
	{		
		GSE_ERR("store_reg get para error\n");	
	}
	
	return count;
}


#if defined(QMA6981_USE_CALI)
static void qma6981_write_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDWR|O_CREAT, 0666);
	if (IS_ERR(fp))
	{
		printk("qma6981_write_file open file error\n");
	}
	else
	{
		//printk("qma6981_write_file data=0x%x len=%d\n", data, len);
		//snprintf();
		fp->f_op->write(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
	}

	set_fs(fs);
}

static void qma6981_read_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	if(qma6981_cali_flag == 1)
	{
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fp))
	{
		qma6981_cali_flag = 1;
		printk("qma6981_read_file open file error\n");
	}
	else
	{
		//printk("qma6981_read_file data=0x%x len=%d\n", data, len);
		fp->f_op->read(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
		qma6981_cali_flag = 1;
	}

	set_fs(fs);
}

static ssize_t qma6981_cali_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", qma6981_cali[0], qma6981_cali[1], qma6981_cali[2]);
}

static ssize_t qma6981_cali_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int en = 0;
	int data[3], data_avg[3];
	int icount, z_max, z_min;	
	struct qmaX981_acc acc;
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
#if 0
	if (strict_strtoul(buf, 10, &en))
		return -EINVAL;
#endif
	if(1 == sscanf(buf, "%d", &en))
	{
		
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
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
			//qma6981_read_raw_xyz(acc_qst, data);			
			qmaX981_read_accel_xyz(&acc);
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
			printk("qma6981_cali_store check vibrate cali ingore!\n");
			return count;
		}
		// add by yangzhiqiang check vibrate

		data_avg[0] = data_avg[0]/QMA6981_CALI_NUM;
		data_avg[1] = data_avg[1]/QMA6981_CALI_NUM;
		data_avg[2] = data_avg[2]/QMA6981_CALI_NUM;
		printk("qma6981_cali_store data_avg[%d %d %d]\n", data_avg[0], data_avg[1], data_avg[2]);
		// add by yangzhiqiang check offset range
#if 0
		if(QMA6981_ABS(data_avg[2]-QMA6981_LSB_1G)>(QMA6981_LSB_1G*5/10))
		{
			printk("qma6981_cali_store check offset range cali ingore!\n");
			return count;
		}
#endif
		// add by yangzhiqiang check offset range
		data[0] = 0-data_avg[0];
		data[1] = 0-data_avg[1];
		data[2] = QMA6981_LSB_1G-data_avg[2];
		qma6981_cali[0] += data[0];
		qma6981_cali[1] += data[1];
		qma6981_cali[2] += data[2];
		printk("qma6981_cali_store offset[%d %d %d]\n", data[0], data[1], data[2]);
		printk("qma6981_cali_store qma6981_cali[%d %d %d]\n", qma6981_cali[0], qma6981_cali[1], qma6981_cali[2]);
		qma6981_write_file(QMA6981_CALI_FILE, (char *)qma6981_cali, sizeof(qma6981_cali));
		
	}
	else
	{
	}
	
	return count;
}
#endif

#if defined(QMA7981_HAND_UP_DOWN)
static ssize_t show_hud_layout(struct device *dev, struct device_attribute *attr, char *buf)
{
	GSE_FUN();

	return sprintf(buf, "%d\n",hud_layout);
}

static ssize_t store_hud_layout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int position = 0;

	GSE_FUN();

	if(1 == sscanf(buf, "%d", &position))
	{
		if ((position >= 0) && (position <= 7))
		{
			hud_layout = position;
			qma7981_initialize();
		}
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
#ifdef QMAX981_STEP_COUNTER
static DEVICE_ATTR(spower,		QMAX981_ATTR_WR, show_sc_power_value, store_sc_power_value);
static DEVICE_ATTR(sspcount,		QMAX981_ATTR_WR, show_sc_count_value, store_sc_count_value);
static DEVICE_ATTR(sprecision,	QMAX981_ATTR_WR, show_sc_recision_value, store_sc_recision_value);
static DEVICE_ATTR(stimelow,		QMAX981_ATTR_WR, show_sc_timelow_value, store_sc_timelow_value);
static DEVICE_ATTR(stimeup,		QMAX981_ATTR_WR, show_sc_timeup_value, store_sc_timeup_value);
static DEVICE_ATTR(saxis,			QMAX981_ATTR_WR, show_sc_axis_value, store_sc_axis_value);
static DEVICE_ATTR(sc_value,		QMAX981_ATTR_R, qmaX981_sc_value_show , NULL);
static DEVICE_ATTR(sc_cls,		QMAX981_ATTR_W, NULL , qmaX981_clear_stepcount_store);
static DEVICE_ATTR(sc_interrupt_num,	QMAX981_ATTR_R, show_sc_interrupt_num , NULL);
static DEVICE_ATTR(enable_sc,		QMAX981_ATTR_W, NULL , qmaX981_enable_sc_store);
#endif 

static DEVICE_ATTR(init_acc,		QMAX981_ATTR_R, show_init_acc_value, NULL);
static DEVICE_ATTR(chipinfo,		QMAX981_ATTR_R, show_chipinfo_value, NULL);
static DEVICE_ATTR(waferid,		QMAX981_ATTR_R, show_waferid_value, NULL);
static DEVICE_ATTR(sensordata,	QMAX981_ATTR_R, show_sensordata_value,    NULL);
static DEVICE_ATTR(dumpallreg,	QMAX981_ATTR_R, show_dumpallreg_value, NULL);
static DEVICE_ATTR(dumpallreg2,	QMAX981_ATTR_R, show_dumpallreg2_value, NULL);
static DEVICE_ATTR(layout,		QMAX981_ATTR_WR, show_layout_value, store_layout_value);
static DEVICE_ATTR(enable_acc,	QMAX981_ATTR_WR, qmaX981_enable_show , qmaX981_enable_store);
static DEVICE_ATTR(delay_acc,		QMAX981_ATTR_WR, qmaX981_delay_show , qmaX981_delay_store);
static DEVICE_ATTR(setreg,		QMAX981_ATTR_WR, qmaX981_getreg , qmaX981_setreg);
#if defined(QMA6981_USE_CALI)
static DEVICE_ATTR(cali,			QMAX981_ATTR_WR, qma6981_cali_show , qma6981_cali_store);
#endif
#if defined(QMA7981_HAND_UP_DOWN)
static DEVICE_ATTR(hudlayout,			QMAX981_ATTR_WR, show_hud_layout , store_hud_layout);
#endif


static struct attribute *qmaX981_attributes[] = {
#ifdef QMAX981_STEP_COUNTER
	&dev_attr_spower.attr,
	&dev_attr_sspcount.attr,
	&dev_attr_sprecision.attr,
	&dev_attr_stimelow.attr,
	&dev_attr_stimeup.attr,
	&dev_attr_saxis.attr,
	&dev_attr_sc_value.attr,
	&dev_attr_sc_cls.attr,
	&dev_attr_sc_interrupt_num.attr,
	&dev_attr_enable_sc.attr,
#endif
	&dev_attr_init_acc.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_waferid.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_dumpallreg2.attr,
	&dev_attr_layout.attr,
	&dev_attr_enable_acc.attr,
	&dev_attr_delay_acc.attr,
	&dev_attr_setreg.attr,
#if defined(QMA6981_USE_CALI)
	&dev_attr_cali.attr,
#endif
#if defined(QMA7981_HAND_UP_DOWN)
	&dev_attr_hudlayout.attr,
#endif
	NULL
};

static struct attribute_group qmaX981_attribute_group = {
	//.name = "qmax981",
	.attrs = qmaX981_attributes
};

static long qmaX981_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;
	struct qmaX981_data *qmaX981 = file->private_data;
#ifdef QMAX981_STEP_COUNTER
	//unsigned char data;
#endif
	int temp = 0;

	GSE_LOG("%s: cmd %x\n",__func__, cmd);

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));

	if(err)
	{
		GSE_ERR("%s: access isn't ok!\n", __func__);
		return -EFAULT;
	}

	if (NULL == this_client)
	{
		GSE_ERR("%s: i2c client isn't exist!\n", __func__);
		return -EFAULT;
	}
	
	switch(cmd)
	{
	case QMAX981_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&temp, argp, sizeof(temp)))
		{
			GSE_ERR("%s: set delay copy error!\n", __func__);
			return -EFAULT;
		}
		if (temp < 0 || temp > 1000)
		{
			GSE_ERR("%s: set delay over limit!\n", __func__);
			return -EINVAL;
		}
		qmaX981_set_delay(&(qmaX981->client->dev),temp);
		break;

	case QMAX981_ACC_IOCTL_GET_DELAY:
		temp = qmaX981_get_delay(&(qmaX981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp)))
		{
			GSE_ERR("%s: get delay copy error!\n", __func__);
			return -EFAULT;
		}
		break;

	case QMAX981_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&temp, argp, sizeof(temp)))
		{
			GSE_ERR("%s: set enable copy error!\n", __func__);
			return -EFAULT;
		}

		if (1 == temp)
			qmaX981_set_enable(&(qmaX981->client->dev), 1);
		else if (0 == temp)
			qmaX981_set_enable(&(qmaX981->client->dev), 0);
		else
		{
			GSE_ERR("%s: set enable over limit!\n", __func__);
			return -EINVAL;
		}
			
		break;

	case QMAX981_ACC_IOCTL_GET_ENABLE:
		temp = qmaX981_get_enable(&(qmaX981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp)))
		{
			GSE_ERR("%s: get enable copy error!\n", __func__);
			return -EINVAL;
		}
		break;

#ifdef QMAX981_STEP_COUNTER
	case QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE:
		GSE_LOG("QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE START!\n");
		if (copy_from_user(&temp, argp, sizeof(temp)))
		{
			GSE_ERR("%s: set enable copy error!\n", __func__);
			return -EFAULT;
		}
		GSE_LOG("QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE OK! %d\n",temp);
		if (1 == temp)
			qmaX981_set_stepcount_enable(&(qmaX981->client->dev), 1);
		else if (0 == temp)
			qmaX981_set_stepcount_enable(&(qmaX981->client->dev), 0);
		else
		{
			GSE_ERR("%s: set enable over limit!\n", __func__);
			return -EINVAL;
		}
			
		break;

	case QMAX981_ACC_IOCTL_GET_STEPCOUNT_ENABLE:
		temp = qmaX981_get_stepcount_enable(&(qmaX981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp)))
		{
			GSE_ERR("%s: get enable copy error!\n", __func__);
			return -EINVAL;
		}
		break;
		
	case QMAX981_ACC_IOCTL_CLEAR_STEPCOUNT:
		temp = qmaX981_clear_stepcount((qmaX981->client));
		//if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
		//{
		//	data = 0x10;
		//	temp = qmaX981_smbus_write_byte(qmaX981->client,0x13,&data);
		//}
	break;
#endif
			
	case QMAX981_ACC_IOCTL_CALIBRATION:
		GSE_ERR("%s: don't handle the command!\n", __func__);
		return -EINVAL;
		

	default:
		GSE_ERR("%s: can't recognize the cmd!\n", __func__);
		return 0;
	}
	
    return 0;
}


static int qmaX981_open(struct inode *inode, struct file *file)
{
	int err = 0;

	GSE_FUN();

	err = nonseekable_open(inode, file);
	if (err < 0)
	{
		GSE_ERR("%s: open fail!\n", __func__);
		return err;
	}

	file->private_data = i2c_get_clientdata(this_client);

	return 0;
}

static int qmaX981_release(struct inode *inode, struct file *file)
{
	GSE_FUN();
	file->private_data = NULL;
	return 0;
}

static const struct file_operations qmaX981_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = qmaX981_open,
	.release = qmaX981_release,
	.unlocked_ioctl = qmaX981_unlocked_ioctl,
};

static struct miscdevice qmaX981_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMAX981_DEV_NAME,
	.fops = &qmaX981_acc_misc_fops,
};

#ifdef CONFIG_OF
static struct QMAX981_acc_platform_data *qmaX981_acc_parse_dt(struct device *dev)
{
	struct QMAX981_acc_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct lis3dh_acc_platform_data");
		return NULL;
	}

/*	pdata->gpio_int1 = of_get_gpio(np, 0);
	GSE_LOG(" %s:get gpio_int1  %d\n",__func__,pdata->gpio_int1);
	if(pdata->gpio_int1 < 0){
		dev_err(dev, "fail to get irq_gpio_number\n");
		GSE_LOG(" get irq1_gpio_number fail\n");
		goto fail;
	}*/
	
	ret = of_property_read_u32(np, "layout", &pdata->layout);
	if(ret){
		dev_err(dev, "fail to get g_range\n");
		goto fail;
	}
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static int qmaX981_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{

	static struct qmaX981_data *qmaX981;
	struct QMAX981_acc_platform_data *pdata;
	int err = 0;
	int layout = 0;

	GSE_LOG("%s: start\n",__func__);

	/* check i2c function */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GSE_ERR("%s: i2c function not support!\n", __func__);
		err = -ENODEV;
		goto exit;
	}
	
	/* setup private data */
	qmaX981 = kzalloc(sizeof(struct qmaX981_data), GFP_KERNEL);
	if (!qmaX981) {
		GSE_ERR("%s: can't allocate memory for qmaX981_data!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}

	atomic_set(&qmaX981->position, 0);
#ifdef CONFIG_OF
	if(client->dev.of_node)
	{
		pdata = qmaX981_acc_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
			atomic_set(&qmaX981->position, pdata->layout);		
		}
		else{
			atomic_set(&qmaX981->position, 0);
		}
	}
#endif

	layout = atomic_read(&qmaX981->position);
	qmaX981_set_layout(layout);

	mutex_init(&qmaX981->enable_mutex);
	mutex_init(&qmaX981->value_mutex);	
	mutex_init(&qmaX981->op_mutex);
	
	atomic_set(&qmaX981->delay, QMAX981_DEFAULT_DELAY);
	
	this_client = client;
	qmaX981->client = client;
	i2c_set_clientdata(client, qmaX981);
	g_qmaX981 = qmaX981;

#ifdef QMA6981_STEP_COUNTER_USE_INT
    wake_lock_init(&sc_wakelock,WAKE_LOCK_SUSPEND,"sc wakelock");
#endif

	g_qmaX981->chip_type = CHIP_TYPE_UNDEFINE;
#if 1
	err = qmaX981_read_chip_id(this_client);
	if(err < 0) {
		goto exit1;
		GSE_ERR("%s: qmaX981 read id fail!\n", __func__);
	}
	if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
		qma6981_initialize(qmaX981);
	else if(g_qmaX981->chip_type == CHIP_TYPE_QMA7981)
		qma7981_initialize();
	else if(g_qmaX981->chip_type == CHIP_TYPE_QMA6100)
		qma6100_initialize();
#endif

	INIT_DELAYED_WORK(&qmaX981->work, acc_work_func);
#ifdef QMAX981_STEP_COUNTER
	INIT_DELAYED_WORK(&qmaX981->sc_work, step_c_work_func);
	qmaX981->sc_dely = 200;
	#ifdef QMA6981_CHECK_ABNORMAL_DATA
    memset(&g_qmaX981_data_c,0,sizeof(qmaX981_data_check));  //cxg
    #endif
#endif 

#ifdef QMA6981_STEP_COUNTER_USE_INT
	if(g_qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		INIT_WORK(&qmaX981->eint_work, QMA6981_eint_work); 
		QMA6981_setup_eint(this_client);
	}
#endif

	err = qmaX981_input_init(qmaX981);
	if (err < 0) {
		goto exit1;
		GSE_ERR("input init fail!\n");
	}

#ifdef QMAX981_STEP_COUNTER
	err = qmaX981_sc_input_init(qmaX981);
	if (err < 0) {
		GSE_ERR("qmaX981_sc_input_init fail!\n");
		goto exit2;
	}
#endif

	err = sysfs_create_group(&qmaX981->input->dev.kobj, &qmaX981_attribute_group);
	if (err < 0) {
		GSE_ERR("%s: create group fail!\n", __func__);
		#ifdef QMAX981_STEP_COUNTER
			goto exit3;
		#else
			goto exit2;
		#endif
	}

	err = misc_register(&qmaX981_device);
	if (err) {
		GSE_ERR("%s: create register fail!\n", __func__);
		goto exit4;
	}
	return 0;
exit4:
	sysfs_remove_group(&qmaX981->input->dev.kobj, &qmaX981_attribute_group);	
#ifdef QMAX981_STEP_COUNTER
exit3:
	qmaX981_sc_input_deinit(qmaX981);
#endif 
exit2:
	qmaX981_input_deinit(qmaX981);	
exit1:
	kfree(qmaX981);
exit:
	return err;	
}

static int qmaX981_remove(struct i2c_client *client)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	GSE_FUN();
	sysfs_remove_group(&qmaX981->input->dev.kobj, &qmaX981_attribute_group);
#ifdef QMAX981_STEP_COUNTER
	qmaX981_sc_input_deinit(qmaX981);
#endif 	
	qmaX981_input_deinit(qmaX981);
	kfree(qmaX981);
	return 0;
}

static int qmaX981_i2c_remove(struct i2c_client *client)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	int rc;
	unsigned char data = 0x00;
	rc = qmaX981_smbus_write_byte(this_client,QMA6981_MODE,&data);
	
	GSE_FUN();	
	return qmaX981_remove(qmaX981->client);
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static int qmaX981_suspend(struct i2c_client *client, pm_message_t mesg)
{

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	GSE_FUN();
	
//	mutex_lock(&qmaX981->enable_mutex);
	//if (atomic_read(&open_flag)){
	if(atomic_read(&acc_flag))
		cancel_delayed_work_sync(&qmaX981->work);
#if defined(QMAX981_STEP_COUNTER)
	if(atomic_read(&sc_flag))
		cancel_delayed_work_sync(&qmaX981->sc_work);
#endif
		qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY);
	//	atomic_set(&open_flag,0);
	//}
//	mutex_unlock(&qmaX981->enable_mutex);

	return 0;
}

static int qmaX981_resume(struct i2c_client *client)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	int delay = qmaX981_get_delay(&(client->dev));

	GSE_FUN();

	qmaX981_set_delay(&client->dev, delay);

//	mutex_lock(&qmaX981->enable_mutex);

#ifndef QMAX981_STEP_COUNTER
	qmaX981_set_mode(qmaX981->client, QMAX981_MODE_ACTIVE);
#endif
#if defined(QMAX981_STEP_COUNTER)
	if(atomic_read(&sc_flag))
		schedule_delayed_work(&qmaX981->sc_work,msecs_to_jiffies(qmaX981->sc_dely));
#endif
	if(atomic_read(&acc_flag))
		schedule_delayed_work(&qmaX981->work,msecs_to_jiffies(delay));

//	mutex_unlock(&qmaX981->enable_mutex);
	return 0;
}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void qmaX981_early_suspend (struct early_suspend* es)
{
	GSE_FUN();
	qmaX981_suspend(this_client,(pm_message_t){.event=0});
}

static void qmaX981_early_resume (struct early_suspend* es)
{
	GSE_FUN();

	qmaX981_resume(this_client);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id qmaX981_id[] = {
	{QMAX981_DEV_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, qmaX981_id);

static const struct of_device_id qmaX981_acc_of_match[] = {
       { .compatible = "QST,qmax981", },
       { }
};

MODULE_DEVICE_TABLE(of, qmaX981_acc_of_match);
static struct i2c_driver qmaX981_driver = {
	.driver = {
		.name = QMAX981_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = qmaX981_acc_of_match,
	},
	.probe    = qmaX981_i2c_probe,
	.remove   = qmaX981_i2c_remove,
	//.suspend  = qmaX981_suspend,
	//.resume   = qmaX981_resume,
	.id_table = qmaX981_id,
};

static int __init qmaX981_i2c_init(void)
{
	GSE_LOG("%s accelerometer driver: init\n", QMAX981_DEV_NAME);

	return i2c_add_driver(&qmaX981_driver);
}

static void __exit qmaX981_i2c_exit(void)
{
	GSE_LOG("%s accelerometer driver exit\n", QMAX981_DEV_NAME);

	i2c_del_driver(&qmaX981_driver);
}


module_init(qmaX981_i2c_init);
//late_initcall(qmaX981_i2c_init);
module_exit(qmaX981_i2c_exit);

MODULE_DESCRIPTION("qmaX981 accelerometer driver");
MODULE_AUTHOR("YZQ QST");
MODULE_LICENSE("GPL");
MODULE_VERSION(QMAX981_DEV_VERSION);

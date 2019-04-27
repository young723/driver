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

#include "fis210x.h"

//#define ACC_USE_CALI

enum fis210x_axis {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
	AXIS_NUM
};

struct fis210x_report
{
	int x;
	int y;
	int z;
};

struct fis210x_convert {
	signed char sign[3];
	unsigned char map[3];
};



struct fis210x_data {
	atomic_t enable;
	atomic_t delay;	
	atomic_t gyro_enable;
	atomic_t gyro_delay;
	atomic_t position;
	
	struct mutex op_mutex;
	struct i2c_client *client;
	struct input_dev *acc_input;
	struct input_dev *gyro_input;
	struct delayed_work acc_work;
	struct delayed_work gyro_work;
	unsigned char chip_id;
	int acc_lsb_1g;	
	int gyro_lsb_1g;	
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
#endif 
};


#if defined(ACC_USE_CALI)
#define ACC_CALI_FILE		"/productinfo/fis210xcali.conf"
#define ACC_LSB_1G			1000			// mg
#define ACC_CALI_NUM		20    
static int acc_cali[3]={0, 0, 0};
static char acc_cali_flag = 0;
static void acc_read_file(char * filename, char *data, int len);
static void acc_write_file(char * filename, char *data, int len);
#endif


static struct fis210x_convert g_map;
static struct fis210x_data *g_fis210x=NULL;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fis210x_early_suspend (struct early_suspend* es);
static void fis210x_late_resume(struct early_suspend* es);
#endif


#if 0
static int fis210x_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
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

static int fis210x_smbus_write_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
	signed int dummy = 0;
    mutex_lock(&g_fis210x->op_mutex);
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	mutex_unlock(&g_fis210x->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}
#endif

#if defined(BUILD_ON_SPREADTRUM)
static int fis210x_smbus_read_block(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	signed int dummy = 0;

	mutex_lock(&g_fis210x->op_mutex);
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&g_fis210x->op_mutex);
	if(dummy < 0)
		return dummy;
	return 0;
}

static int fis210x_smbus_write_block(struct i2c_client *client,unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	signed int dummy = 0;

	mutex_lock(&g_fis210x->op_mutex);
	dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&g_fis210x->op_mutex);
	if(dummy < 0)
		return dummy;
	return 0;
}
#endif

static int fis210x_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
#if defined(BUILD_ON_MTK_KK)
	unsigned char txbuf[8];
	int loop_i;
#endif

#if defined(BUILD_ON_MTK_KK)
	txbuf[0] = reg;
	txbuf[1] = value;
	
	for(loop_i = 0; loop_i < 5; loop_i++)
	{
		mutex_lock(&g_fis210x->op_mutex);
		ret = i2c_master_send(g_fis210x->client, txbuf, 2);
		mutex_unlock(&g_fis210x->op_mutex);
		if(ret < 0)
		{
			FIS210X_ERR("try:%d,i2c_master_send error:%d\n",loop_i,  ret);
		} 
		else
		{
			break;
		}
		mdelay(5);
	}
#elif defined(BUILD_ON_SPREADTRUM)
	ret = fis210x_smbus_write_block(g_fis210x->client, reg, &value, 1);
#endif
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
#if defined(BUILD_ON_MTK_KK)
	int loop_i;
#endif

#if defined(BUILD_ON_MTK_KK)
	g_fis210x->client->addr = (g_fis210x->client->addr&I2C_MASK_FLAG)|I2C_WR_FLAG|I2C_RS_FLAG;
	buf[0] = reg;
	for(loop_i = 0; loop_i < 5; loop_i++) 
	{		 
		mutex_lock(&g_fis210x->op_mutex);
		ret = i2c_master_send(g_fis210x->client, (const char*)buf, ((len<<0X08) | 0X01));		
		mutex_unlock(&g_fis210x->op_mutex);
		if(ret >= 0)
			break;

		FIS210X_ERR("qmaX981 i2c_read retry %d times\n", loop_i);
		mdelay(5); 
	}		 
	g_fis210x->client->addr = g_fis210x->client->addr & I2C_MASK_FLAG;

#elif defined(BUILD_ON_SPREADTRUM)
	ret = fis210x_smbus_read_block(g_fis210x->client, reg, buf, len);
#endif

	if(ret < 0)
	{
		return ret;
	}
	else
	{
		return 0;
	}
}


static int fis210x_read_chip_id(void)
{	
	int res = 0;
	unsigned char chip_id;

	FIS210X_FUN();

	res = fis210x_read_reg(FisRegister_WhoAmI, &chip_id, 1);
	g_fis210x->chip_id = chip_id;
	if(g_fis210x->chip_id == 0xfc)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

void fis210x_enable_sensors(unsigned char enableFlags)
{
	if(enableFlags & FISIMU_CTRL7_AE_ENABLE)
	{
		enableFlags |= FISIMU_CTRL7_ACC_ENABLE | FISIMU_CTRL7_GYR_ENABLE;
	}

	enableFlags = enableFlags & FISIMU_CTRL7_ENABLE_MASK;
	fis210x_write_reg(FisRegister_Ctrl7, enableFlags);
}

int fis210x_acc_set_range_odr(enum FisImu_AccRange range,enum FisImu_AccOdr odr)
{
	unsigned char acc_setting = 0;
	int err=0;

	switch(range)
	{
		case AccRange_2g:
			g_fis210x->acc_lsb_1g = (1 << 14);
			break;
		case AccRange_4g:
			g_fis210x->acc_lsb_1g = (1 << 13);
			break;
		case AccRange_8g:
			g_fis210x->acc_lsb_1g = (1 << 12);
			break;
		case AccRange_16g:
			g_fis210x->acc_lsb_1g = (1 << 11);
			break;
		default:			
			g_fis210x->acc_lsb_1g = (1 << 14);
			break;
	}

	acc_setting = (unsigned char)range | (unsigned char)odr;
	err = fis210x_write_reg(FisRegister_Ctrl2, acc_setting);

	return err;
}

int fis210x_gyro_set_range_odr(enum FisImu_GyrRange range,enum FisImu_GyrOdr odr)
{
	unsigned char gyro_setting = 0;
	int err=0;

	switch(range)
	{
		case GyrRange_32dps:
			g_fis210x->gyro_lsb_1g = (1 << 10);
			break;
		case GyrRange_64dps:
			g_fis210x->gyro_lsb_1g = (1 << 9);
			break;
		case GyrRange_128dps:
			g_fis210x->gyro_lsb_1g = (1 << 8);
			break;
		case GyrRange_256dps:
			g_fis210x->gyro_lsb_1g = (1 << 7);
			break;
		case GyrRange_512dps:
			g_fis210x->gyro_lsb_1g = (1 << 6);
			break;
		case GyrRange_1024dps:
			g_fis210x->gyro_lsb_1g = (1 << 5);
			break;
		case GyrRange_2048dps:
			g_fis210x->gyro_lsb_1g = (1 << 4);
			break;
		case GyrRange_2560dps:
			g_fis210x->gyro_lsb_1g = (1 << 3);
			break;
		default:
			g_fis210x->gyro_lsb_1g = (1 << 5);			
	}

	gyro_setting = (unsigned char)range | (unsigned char)odr;
	err = fis210x_write_reg(FisRegister_Ctrl3, gyro_setting);

	return err;
}


static int fis210x_set_mode(unsigned char mode)
{
	unsigned char reg_value = 0;

	if(mode == FIS_MODE_LOW_POWER)
	{
		reg_value = 0;
		fis210x_write_reg(FisRegister_Ctrl1, reg_value);
		fis210x_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else if(mode == FIS_MODE_POWER_DOWN)
	{
		reg_value = 1;
		fis210x_write_reg(FisRegister_Ctrl1, reg_value);
		fis210x_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else
	{
		fis210x_acc_set_range_odr(AccRange_8g, AccOdr_256Hz);
		fis210x_gyro_set_range_odr(GyrRange_1024dps, GyrOdr_1024Hz);
		
		reg_value = 0;
		fis210x_write_reg(FisRegister_Ctrl1, reg_value);
		fis210x_enable_sensors(FISIMU_CTRL7_ACC_ENABLE|FISIMU_CTRL7_GYR_ENABLE);
	}

	return 0;
}


void fis210x_set_layout(int layout)
{
	if(layout == 0)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 1)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 2)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 3)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}	
	else if(layout == 4)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 5)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 6)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 7)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else		
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
}


static int fis210x_read_accel_xyz(struct fis210x_report *acc)
{
	int res = 0;

	unsigned char buf_reg[6];
	short read_xyz[3];
	int acc_xyz[3];

#if 1
	res += fis210x_read_reg(FisRegister_Ax_L, &buf_reg[0], 1); 	// 0x19, 25
	res += fis210x_read_reg(FisRegister_Ax_H, &buf_reg[1], 1);
	res += fis210x_read_reg(FisRegister_Ay_L, &buf_reg[2], 1);
	res += fis210x_read_reg(FisRegister_Ay_H, &buf_reg[3], 1);
	res += fis210x_read_reg(FisRegister_Az_L, &buf_reg[4], 1);
	res += fis210x_read_reg(FisRegister_Az_H, &buf_reg[5], 1);
#else
	res = fis210x_read_reg(FisRegister_Ax_L, buf_reg, 6); 	// 0x19, 25
#endif

	read_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	read_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	read_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	acc_xyz[g_map.map[0]] = g_map.sign[0]*read_xyz[0];
	acc_xyz[g_map.map[1]] = g_map.sign[1]*read_xyz[1];
	acc_xyz[g_map.map[2]] = g_map.sign[2]*read_xyz[2];

	acc->x = acc_xyz[0]*1000/g_fis210x->acc_lsb_1g;
	acc->y = acc_xyz[1]*1000/g_fis210x->acc_lsb_1g;
	acc->z = acc_xyz[2]*1000/g_fis210x->acc_lsb_1g;

	return res;
}


static int fis210x_read_gyro_xyz(struct fis210x_report *gyro)
{
	int res = 0;
	unsigned char buf_reg[6];
	short read_xyz[3];
	int gyro_xyz[3];

#if 1
	res += fis210x_read_reg(FisRegister_Gx_L, &buf_reg[0], 1);		// 0x19, 25
	res += fis210x_read_reg(FisRegister_Gx_H, &buf_reg[1], 1);
	res += fis210x_read_reg(FisRegister_Gy_L, &buf_reg[2], 1);
	res += fis210x_read_reg(FisRegister_Gy_H, &buf_reg[3], 1);
	res += fis210x_read_reg(FisRegister_Gz_L, &buf_reg[4], 1);
	res += fis210x_read_reg(FisRegister_Gz_H, &buf_reg[5], 1);
#else
	res = fis210x_read_reg(FisRegister_Gx_L, buf_reg, 6); 	// 0x19, 25
#endif
	read_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	read_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	read_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	gyro_xyz[g_map.map[0]] = g_map.sign[0]*read_xyz[0];
	gyro_xyz[g_map.map[1]] = g_map.sign[1]*read_xyz[1];
	gyro_xyz[g_map.map[2]] = g_map.sign[2]*read_xyz[2];

	gyro->x = (gyro_xyz[0]*1000)/(g_fis210x->gyro_lsb_1g);
	gyro->y = (gyro_xyz[1]*1000)/(g_fis210x->gyro_lsb_1g);
	gyro->z = (gyro_xyz[2]*1000)/(g_fis210x->gyro_lsb_1g);

	return res;
}


static void acc_work_func(struct work_struct *work)
{
	static struct fis210x_report acc = { 0 };
	int comres = -1;

	comres = fis210x_read_accel_xyz(&acc);
	if(comres)
	{
		comres = fis210x_read_accel_xyz(&acc);
		if(comres)
		{		
			schedule_delayed_work(&g_fis210x->acc_work, msecs_to_jiffies(atomic_read(&g_fis210x->delay)));
			return;
		}
	}
	
	input_report_abs(g_fis210x->acc_input, ABS_X, acc.x);
	input_report_abs(g_fis210x->acc_input, ABS_Y, acc.y);
	input_report_abs(g_fis210x->acc_input, ABS_Z, acc.z);
	input_report_abs(g_fis210x->acc_input, ABS_THROTTLE, 3);
	
	input_sync(g_fis210x->acc_input);
		
	FIS210X_LOG("%s: [%d %d %d ]\n",__func__,acc.x,acc.y,acc.z);
	schedule_delayed_work(&g_fis210x->acc_work, msecs_to_jiffies(atomic_read(&g_fis210x->delay)));
}


static void gyro_work_func(struct work_struct *work)
{
	static struct fis210x_report gyro = { 0 };
	int comres = -1;

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
		
	FIS210X_LOG("%s: [%d %d %d ]\n",__func__,gyro.x,gyro.y,gyro.z);
	schedule_delayed_work(&g_fis210x->gyro_work, msecs_to_jiffies(atomic_read(&g_fis210x->gyro_delay)));
}



static int fis210x_input_init(struct fis210x_data *fis210x)
{
	struct input_dev *dev = NULL;
	struct input_dev *gyro_dev = NULL;
	int err = 0;

	FIS210X_LOG("%s called\n", __func__);
	// acc input
	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s: can't allocate device!\n", __func__);
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
		pr_err("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}
	fis210x->acc_input = dev;
	// acc input

	// gyro input
	gyro_dev = input_allocate_device();
	if (!gyro_dev) {
		pr_err("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}
	gyro_dev->name = FIS210X_GYRO_INPUT_NAME;
	gyro_dev->id.bustype = BUS_I2C;
	input_set_capability(gyro_dev, EV_ABS, ABS_X);
	input_set_capability(gyro_dev, EV_ABS, ABS_Y);
	input_set_capability(gyro_dev, EV_ABS, ABS_Z);
	input_set_capability(gyro_dev, EV_ABS, ABS_THROTTLE);
	input_set_abs_params(gyro_dev, ABS_X, ABSMIN_GYRO, ABSMAX_GYRO, 0, 0);
	input_set_abs_params(gyro_dev, ABS_Y, ABSMIN_GYRO, ABSMAX_GYRO, 0, 0);
	input_set_abs_params(gyro_dev, ABS_Z, ABSMIN_GYRO, ABSMAX_GYRO, 0, 0);
	input_set_abs_params(gyro_dev, ABS_THROTTLE, 0, 3, 0, 0);
	input_set_drvdata(gyro_dev, (void*)fis210x);
	err = input_register_device(gyro_dev);
	
	if(err < 0) {
		pr_err("%s: can't register device!\n", __func__);
		input_free_device(gyro_dev);
		return err;
	}
	fis210x->gyro_input = gyro_dev;
	// gyro input

	return 0;
}


static void fis210x_input_deinit(struct fis210x_data *fis210x)
{
	FIS210X_LOG("%s called\n", __func__);

	input_unregister_device(fis210x->acc_input);
	input_free_device(fis210x->acc_input);
	fis210x->acc_input = NULL;
	
	input_unregister_device(fis210x->gyro_input);
	input_free_device(fis210x->gyro_input);
	fis210x->gyro_input = NULL;
}


static int fis210x_get_enable(enum FisImu_chip chip_type)
{
	FIS210X_FUN();

	if(chip_type == FIS_IMU_ACC)
		return atomic_read(&g_fis210x->enable);
	else if(chip_type == FIS_IMU_GYRO)
		return atomic_read(&g_fis210x->gyro_enable);
	else
		return 0;
}

static int fis210x_set_enable(enum FisImu_chip chip_type, int enable)
{
	int acc_enable, gyro_enable;

	FIS210X_LOG("%s: chip_type:%d  enable :%d\n",__func__,chip_type, enable);
	if(chip_type == FIS_IMU_ACC)
		atomic_set(&g_fis210x->enable, enable);
	else if(chip_type == FIS_IMU_GYRO)
		atomic_set(&g_fis210x->gyro_enable, enable);

	acc_enable = (atomic_read(&g_fis210x->enable));
	gyro_enable = (atomic_read(&g_fis210x->gyro_enable)); 

	if(acc_enable || gyro_enable) 
	{
		fis210x_set_mode(FIS_MODE_NOMAL);
		if(acc_enable)
			schedule_delayed_work(&g_fis210x->acc_work, msecs_to_jiffies(atomic_read(&g_fis210x->delay)));
		else
			cancel_delayed_work_sync(&g_fis210x->acc_work); 	

		if(gyro_enable)
			schedule_delayed_work(&g_fis210x->gyro_work, msecs_to_jiffies(atomic_read(&g_fis210x->gyro_delay)));
		else
			cancel_delayed_work_sync(&g_fis210x->gyro_work);
	}
	else
	{
		fis210x_set_mode(FIS_MODE_LOW_POWER);
		cancel_delayed_work_sync(&g_fis210x->acc_work);
		cancel_delayed_work_sync(&g_fis210x->gyro_work);
	}
	
	return 0;
}


static int fis210x_get_delay(enum FisImu_chip chip_type)
{
	if(chip_type == FIS_IMU_ACC)
		return atomic_read(&g_fis210x->delay);
	else if(chip_type == FIS_IMU_GYRO)		
		return atomic_read(&g_fis210x->gyro_delay);
	else
		return 0;
}

static void fis210x_set_delay(enum FisImu_chip chip_type, int delay)
{
	FIS210X_LOG("fis210x_set_delay chip_type:%d, delay:%d", chip_type, delay);

	if(chip_type == FIS_IMU_ACC)
		atomic_set(&g_fis210x->delay, delay);
	else if(chip_type == FIS_IMU_GYRO)		
		atomic_set(&g_fis210x->gyro_delay, delay);
}

static ssize_t show_init_dev_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;

	err = fis210x_read_chip_id();
	if(err < 0)
	{
		FIS210X_ERR("%s: g_fis210x read id fail!\n", __func__);
	}

	return sprintf(buf, "init done!\n");
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "FIS210X\n");
}

static ssize_t show_sensordata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fis210x_report acc;
	struct fis210x_report gyro;
	
	fis210x_read_accel_xyz(&acc);
	fis210x_read_accel_xyz(&gyro);

	return sprintf(buf, "%d %d %d\n%d %d %d",acc.x,acc.y,acc.z, gyro.x,gyro.y,gyro.z);
}
		
static ssize_t show_dumpallreg_value(struct device *dev, struct device_attribute *attr, char *buf)
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
		res = fis210x_read_reg(databuf[0], &databuf[1], 1);
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

static ssize_t store_layout_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int position = 0;

	FIS210X_FUN();

	if(1 == sscanf(buf, "%d", &position))
	{
		if ((position >= 0) && (position <= 7))
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


static ssize_t fis210x_enable_acc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	FIS210X_FUN();

	return sprintf(buf, "%d\n", atomic_read(&g_fis210x->enable));
}

static ssize_t fis210x_enable_acc_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int enable=0;

	FIS210X_FUN();
	
	if(1 == sscanf(buf, "%d", &enable))
	{
		if(enable)
			fis210x_set_enable(FIS_IMU_ACC, 1);
		else
			fis210x_set_enable(FIS_IMU_ACC, 0);
	}
	else
	{	
		FIS210X_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t fis210x_delay_acc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	FIS210X_FUN();
	return sprintf(buf, "%d\n", atomic_read(&g_fis210x->delay));
}

static ssize_t fis210x_delay_acc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int delay=0;

	FIS210X_FUN();
	if(1 == sscanf(buf, "%d", &delay))
	{
		if(delay > FIS210X_MAX_DELAY)
			delay = FIS210X_MAX_DELAY;
		else if(delay <= 1)
			delay = 1;

		fis210x_set_delay(FIS_IMU_ACC, delay);
	}
	else
	{
		FIS210X_ERR("invalid format = '%s'\n", buf);
	}
	
	return count;
}


static ssize_t fis210x_enable_gyro_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	FIS210X_FUN();

	return sprintf(buf, "%d\n", atomic_read(&g_fis210x->gyro_enable));
}

static ssize_t fis210x_enable_gyro_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int enable=0;

	FIS210X_FUN();
	
	if(1 == sscanf(buf, "%d", &enable))
	{
		if(enable)
			fis210x_set_enable(FIS_IMU_GYRO, 1);
		else
			fis210x_set_enable(FIS_IMU_GYRO, 0);
	}
	else
	{	
		FIS210X_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t fis210x_delay_gyro_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	FIS210X_FUN();
	return sprintf(buf, "%d\n", atomic_read(&g_fis210x->gyro_delay));
}

static ssize_t fis210x_delay_gyro_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int delay=0;

	FIS210X_FUN();
	if(1 == sscanf(buf, "%d", &delay))
	{
		if(delay > FIS210X_MAX_DELAY)
			delay = FIS210X_MAX_DELAY;
		else if(delay <= 1)
			delay = 1;

		fis210x_set_delay(FIS_IMU_GYRO, delay);
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
	res = fis210x_read_reg(fis210x_debug_reg_addr, &data, 1);

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
		res = fis210x_write_reg(fis210x_debug_reg_addr, data);		
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
	struct fis210x_data acc;
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

static DEVICE_ATTR(init_dev,		FIS210X_ATTR_R, show_init_dev_value, NULL);
static DEVICE_ATTR(chipinfo,		FIS210X_ATTR_R, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata,	FIS210X_ATTR_R, show_sensordata_value,    NULL);
static DEVICE_ATTR(dumpallreg,	FIS210X_ATTR_R, show_dumpallreg_value, NULL);
static DEVICE_ATTR(layout,		FIS210X_ATTR_WR, show_layout_value, store_layout_value);
static DEVICE_ATTR(enable_acc,	FIS210X_ATTR_WR, fis210x_enable_acc_show , fis210x_enable_acc_store);
static DEVICE_ATTR(delay_acc,		FIS210X_ATTR_WR, fis210x_delay_acc_show , fis210x_delay_acc_store);
static DEVICE_ATTR(enable_gyro,	FIS210X_ATTR_WR, fis210x_enable_gyro_show , fis210x_enable_gyro_store);
static DEVICE_ATTR(delay_gyro,	FIS210X_ATTR_WR, fis210x_delay_gyro_show , fis210x_delay_gyro_store);
static DEVICE_ATTR(setreg,		FIS210X_ATTR_WR, fis210x_getreg , fis210x_setreg);
#if defined(ACC_USE_CALI)
static DEVICE_ATTR(cali,			FIS210X_ATTR_WR, acc_cali_show , acc_cali_store);
#endif

static struct attribute *fis210x_attributes[] = {
	&dev_attr_init_dev.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_layout.attr,
	&dev_attr_enable_acc.attr,
	&dev_attr_delay_acc.attr,
	&dev_attr_enable_gyro.attr,
	&dev_attr_delay_gyro.attr,
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
	void __user *argp = (void __user *)arg;
	int temp = 0;

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
	case ACC_IOCTL_SET_ENABLE:		
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			FIS210X_ERR("%s: ACC_IOCTL_SET_ENABLE copy error!\n", __func__);
			return -EFAULT;
		}
		if(temp)
			fis210x_set_enable(FIS_IMU_ACC, 1);
		else
			fis210x_set_enable(FIS_IMU_ACC, 0);
			
		break;
	
	case ACC_IOCTL_SET_DELAY:
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			FIS210X_ERR("%s: ACC_IOCTL_SET_DELAY copy error!\n", __func__);
			return -EFAULT;
		}
		fis210x_set_delay(FIS_IMU_ACC, temp);
			
		break;

	case GYRO_IOCTL_SET_ENABLE:		
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			FIS210X_ERR("%s: GYRO_IOCTL_SET_ENABLE copy error!\n", __func__);
			return -EFAULT;
		}
		if(temp)
			fis210x_set_enable(FIS_IMU_GYRO, 1);
		else
			fis210x_set_enable(FIS_IMU_GYRO, 0);
			
		break;
	
	case GYRO_IOCTL_SET_DELAY:
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			FIS210X_ERR("%s: GYRO_IOCTL_SET_DELAY copy error!\n", __func__);
			return -EFAULT;
		}
		fis210x_set_delay(FIS_IMU_GYRO, temp);
			
		break;
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

static int fis210x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	static struct fis210x_data *fis210x;
	int err = 0;
	int layout = 0;

	FIS210X_LOG("%s: start\n",__func__);
	fis210x = kzalloc(sizeof(struct fis210x_data), GFP_KERNEL);
	if(!fis210x) {
		FIS210X_ERR("%s: can't allocate memory for fis210x_data!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
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
	atomic_set(&fis210x->delay, 200);
	atomic_set(&fis210x->gyro_delay, 200);
	atomic_set(&fis210x->enable, 0);
	atomic_set(&fis210x->gyro_enable, 0);
	client->addr = FIS210X_I2C_SLAVE_ADDR;
	fis210x->client = client;
	i2c_set_clientdata(client, fis210x);
	g_fis210x = fis210x;

	err = fis210x_read_chip_id();
	if(err < 0)
	{
		goto exit1;
		FIS210X_ERR("%s: g_fis210x read id fail!\n", __func__);
	}

	err = fis210x_input_init(fis210x);
	if(err < 0) 
	{
		goto exit1;
		FIS210X_ERR("input init fail!\n");
	}
	
	err = misc_register(&fis210x_device);
	if (err) {
		FIS210X_ERR("%s: create register fail!\n", __func__);
		goto exit2;
	}
#if 1
	err = sysfs_create_group(&fis210x->acc_input->dev.kobj, &fis210x_attribute_group);
	if(err < 0) {
		FIS210X_ERR("%s: create group fail!\n", __func__);
		goto exit3;
	}	
	err = sysfs_create_group(&fis210x->gyro_input->dev.kobj, &fis210x_attribute_group);
	if(err < 0) {
		sysfs_remove_group(&fis210x->acc_input->dev.kobj, &fis210x_attribute_group);
		FIS210X_ERR("%s: create group fail!\n", __func__);
		goto exit3;
	}
#else
	err = sysfs_create_group(&fis210x_device->this_device.kobj, &fis210x_attribute_group);
	if(err < 0) {
		FIS210X_ERR("%s: create group fail!\n", __func__);
		goto exit2;
	}
#endif
	
	INIT_DELAYED_WORK(&fis210x->acc_work, acc_work_func);
	INIT_DELAYED_WORK(&fis210x->gyro_work, gyro_work_func);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	fis210x->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,	
	fis210x->early_drv.suspend	= fis210x_early_suspend,
	fis210x->early_drv.resume	= fis210x_late_resume,
	register_early_suspend(&fis210x->early_drv);
#endif
	return 0;

exit3:
	misc_deregister(&fis210x_device);
exit2:
	fis210x_input_deinit(fis210x);	
exit1:
	kfree(fis210x);
exit:
	return err;	
}

static int fis210x_remove(struct i2c_client *client)
{
	FIS210X_FUN();
	if(g_fis210x)
	{
#if 1
		sysfs_remove_group(&g_fis210x->acc_input->dev.kobj, &fis210x_attribute_group);
		sysfs_remove_group(&g_fis210x->gyro_input->dev.kobj, &fis210x_attribute_group);
#else
		sysfs_remove_group(&fis210x_device->this_device.kobj, &fis210x_attribute_group);
#endif
		fis210x_input_deinit(g_fis210x);
		misc_deregister(&fis210x_device);
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
	int acc_delay, gyro_delay;
	int acc_enable, gyro_enable;

	acc_delay = fis210x_get_delay(FIS_IMU_ACC);
	gyro_delay = fis210x_get_delay(FIS_IMU_GYRO);
	acc_enable = fis210x_get_enable(FIS_IMU_ACC);
	gyro_enable =fis210x_get_enable(FIS_IMU_GYRO);
	
	if(acc_enable || gyro_enable)
	{
		fis210x_set_mode(FIS_MODE_NOMAL);

		if(acc_enable)
			schedule_delayed_work(&g_fis210x->acc_work,msecs_to_jiffies(acc_delay));
		else
			cancel_delayed_work_sync(&g_fis210x->acc_work);

		if(gyro_enable)
			schedule_delayed_work(&g_fis210x->gyro_work,msecs_to_jiffies(gyro_delay));
		else
			cancel_delayed_work_sync(&g_fis210x->gyro_work);
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fis210x_early_suspend(struct early_suspend* es)
{
	FIS210X_FUN();
	fis210x_suspend(g_fis210x->client,(pm_message_t){.event=0});
}

static void fis210x_late_resume(struct early_suspend* es)
{
	FIS210X_FUN();
	fis210x_resume(g_fis210x->client);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

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

#if defined(BUILD_ON_MTK_KK)
static struct i2c_board_info __initdata i2c_fis210x={ I2C_BOARD_INFO(FIS210X_DEV_NAME, (FIS210X_I2C_SLAVE_ADDR))};
#endif

static int __init fis210x_i2c_init(void)
{
	FIS210X_LOG("fis210x_i2c_init\n");
#if defined(BUILD_ON_MTK_KK)
	i2c_register_board_info(0, &i2c_fis210x, 1);
#endif
	return i2c_add_driver(&fis210x_driver);
}

static void __exit fis210x_i2c_exit(void)
{
	FIS210X_LOG("fis210x_i2c_exit\n");
	i2c_del_driver(&fis210x_driver);
}


module_init(fis210x_i2c_init);
//late_initcall(fis210x_i2c_init);
module_exit(fis210x_i2c_exit);

MODULE_DESCRIPTION("fis210x driver");
MODULE_AUTHOR("yangzhiqiang@qstcorp.com");
MODULE_LICENSE("GPL");
MODULE_VERSION(FIS210X_DEV_VERSION);


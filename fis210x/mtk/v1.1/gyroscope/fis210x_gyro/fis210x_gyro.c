/*****************************************************************************
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 *****************************************************************************/

#include <cust_gyro.h>
#include <gyroscope.h>
//#include <hwmsensor.h>
//#include <hwmsen_dev.h> 
//s#include <hwmsen_helper.h>

#include "fis210x_gyro.h"


#define FIS210X_BUFSIZE		256
#define I2C_FIFO_SIZE		8


static const struct i2c_device_id fis210x_gyro_i2c_id[] = {{FIS210X_GYRO_DEV_NAME, 0}, {} };
#if defined(FIS210X_GYRO_MTK_KK)
static struct i2c_board_info __initdata i2c_fis210x_gyro={ I2C_BOARD_INFO(FIS210X_GYRO_DEV_NAME, 0x6b)};
#endif
static int fis210x_gyro_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int fis210x_gyro_i2c_remove(struct i2c_client *client);
static int fis210x_gyro_local_init(struct platform_device *pdev);
static int fis210x_gyro_remove(void);

extern int fis210x_acc_set_mode(enum FisImu_mode mode, u8 power_flag);



struct i2c_client *fis210x_gyro_i2c_client=NULL;
static struct gyro_init_info fis210x_gyro_init_info = 
{
	.name = FIS210X_GYRO_DEV_NAME,
	.init = fis210x_gyro_local_init,
	.uninit = fis210x_gyro_remove,
};

static fis210x_gyro_t *fis210x_gyro=NULL;
static int fis210x_gyro_init_flag = -1;


#if 0
/* I2C operation functions */
static int fis210x_gyro_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.flags = 0,
			.len = 1,
			.buf = &beg
		},
		{
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		}
	};
	if (!client)
		return -EINVAL;
	msgs[0].addr = client->addr;
	msgs[1].addr = client->addr;
#if 0
	else if (len > C_I2C_FIFO_SIZE) {
		FIS210X_GYRO_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
#endif
	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != 2) {
		FIS210X_GYRO_ERR("i2c_transfer error: %x %x (%d %p %d) %d\n",
				msgs[0].addr, client->addr, addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}
	return err;
}

static int fis210x_gyro_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	/*
	 *because address also occupies one byte,
	 *the maximum length for write is 7 bytes
	 */
	int err, idx = 0, num = 0;
	char buf[32];

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		FIS210X_GYRO_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		FIS210X_GYRO_ERR("send command error!!\n");
		return -EFAULT;
	}
	err = 0;

	return err;
}
#endif

int fis210x_gyro_i2c_read(u8 reg_addr, u8 *data, u8 len)
{
#if 1
	int ret;

	if(len > 1)
	{
		reg_addr |= 0x80;
	}
	ret = i2c_smbus_read_i2c_block_data(fis210x_gyro_i2c_client, reg_addr, len, data);
	if(ret < 0)
	{
		FIS210X_GYRO_ERR("gyro i2c read error!\n");
		return -EFAULT;
	}
	else
	{
		return 0;
	}
#else
	int err = 0, loop_i;

	data[0] = reg_addr;
	for(loop_i = 0; loop_i < 5; loop_i++) 
	{		 
		fis210x_gyro_i2c_client->addr = (fis210x_gyro_i2c_client->addr&I2C_MASK_FLAG)|I2C_WR_FLAG|I2C_RS_FLAG;		 
		err = i2c_master_send(fis210x_gyro_i2c_client, (const char*)data, ((len<<0X08) | 0X01)); 
		if(err >= 0)
			break;

		FIS210X_GYRO_ERR("qmaX981 i2c_read retry %d times\n", loop_i);
		mdelay(5); 
	}		 
	fis210x_gyro_i2c_client->addr = fis210x_gyro_i2c_client->addr & I2C_MASK_FLAG;

	if((err < 0)||(loop_i>=5))
		return -EFAULT;
	else
		return 0;
#endif
}

EXPORT_SYMBOL(fis210x_gyro_i2c_read);

int fis210x_gyro_i2c_write(u8 reg_addr, u8 *data, u8 len)
{
	int err, loop_i;
	unsigned char txbuf[I2C_FIFO_SIZE];

	err =0;
	if((!fis210x_gyro_i2c_client)||(!data)||(len >= I2C_FIFO_SIZE-1)) 
	{
		FIS210X_GYRO_ERR(" client or length %d exceeds %d\n", len, I2C_FIFO_SIZE);
		return -EINVAL;
	}
	txbuf[0] = reg_addr;
	for(loop_i = 0; loop_i < len; loop_i++)
	{
		txbuf[1+loop_i] = data[loop_i];
	}

	fis210x_gyro_i2c_client->addr = fis210x_gyro_i2c_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < 5; loop_i++)
	{
		err = i2c_master_send(fis210x_gyro_i2c_client, txbuf, len+1);
		if(err < 0)
		{
			FIS210X_GYRO_ERR("try:%d,i2c_master_send error:%d\n",loop_i,  err);
		} 
		else
		{
			break;
		}
		mdelay(5);
	}

	if((err < 0)||(loop_i>=5))
		return -EFAULT;
	else
		return 0;
}

EXPORT_SYMBOL(fis210x_gyro_i2c_write);


static void fis210x_gyro_power(struct gyro_hw *hw, unsigned int on)
{

}

/*!
 * @brief Reset calibration for acc
 *
 * @param[in] client the pointer of i2c_client
 *
 * @return zero success, non-zero failed
 */
static int fis210x_gyro_ResetCalibration(struct i2c_client *client)
{
	fis210x_gyro_t *obj = fis210x_gyro;//i2c_get_clientdata(client);
	int err = 0;

	FIS210X_GYRO_LOG("fis210x_gyro_ResetCalibration\n");
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));

	return err;    
}

static int fis210x_gyro_ReadCalibration(struct i2c_client *client, int dat[FIS210X_GYRO_AXIS_NUM])
{
	fis210x_gyro_t *obj = fis210x_gyro;//i2c_get_clientdata(client);
	int mul;
	
	mul = 0;//only SW Calibration, disable HW Calibration
	
	dat[obj->cvt.map[FIS210X_GYRO_AXIS_X]] = obj->cvt.sign[FIS210X_GYRO_AXIS_X]*(obj->offset[FIS210X_GYRO_AXIS_X]*mul + obj->cali_sw[FIS210X_GYRO_AXIS_X]);
	dat[obj->cvt.map[FIS210X_GYRO_AXIS_Y]] = obj->cvt.sign[FIS210X_GYRO_AXIS_Y]*(obj->offset[FIS210X_GYRO_AXIS_Y]*mul + obj->cali_sw[FIS210X_GYRO_AXIS_Y]);
	dat[obj->cvt.map[FIS210X_GYRO_AXIS_Z]] = obj->cvt.sign[FIS210X_GYRO_AXIS_Z]*(obj->offset[FIS210X_GYRO_AXIS_Z]*mul + obj->cali_sw[FIS210X_GYRO_AXIS_Z]); 				   

	return 0;
}

static int fis210x_gyro_ReadCalibrationEx(struct i2c_client *client, int act[FIS210X_GYRO_AXIS_NUM], int raw[FIS210X_GYRO_AXIS_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	fis210x_gyro_t *obj = fis210x_gyro;//i2c_get_clientdata(client);
	int mul;

	mul = 0;//only SW Calibration, disable HW Calibration
	FIS210X_GYRO_LOG("fis210x_gyro_ReadCalibrationEx\n");
	raw[FIS210X_GYRO_AXIS_X] = obj->offset[FIS210X_GYRO_AXIS_X]*mul + obj->cali_sw[FIS210X_GYRO_AXIS_X];
	raw[FIS210X_GYRO_AXIS_Y] = obj->offset[FIS210X_GYRO_AXIS_Y]*mul + obj->cali_sw[FIS210X_GYRO_AXIS_Y];
	raw[FIS210X_GYRO_AXIS_Z] = obj->offset[FIS210X_GYRO_AXIS_Z]*mul + obj->cali_sw[FIS210X_GYRO_AXIS_Z];

	act[obj->cvt.map[FIS210X_GYRO_AXIS_X]] = obj->cvt.sign[FIS210X_GYRO_AXIS_X]*raw[FIS210X_GYRO_AXIS_X];
	act[obj->cvt.map[FIS210X_GYRO_AXIS_Y]] = obj->cvt.sign[FIS210X_GYRO_AXIS_Y]*raw[FIS210X_GYRO_AXIS_Y];
	act[obj->cvt.map[FIS210X_GYRO_AXIS_Z]] = obj->cvt.sign[FIS210X_GYRO_AXIS_Z]*raw[FIS210X_GYRO_AXIS_Z];						  
						   
	return 0;
}

static int fis210x_gyro_WriteCalibration(struct i2c_client *client, int dat[FIS210X_GYRO_AXIS_NUM])
{
	fis210x_gyro_t *obj = fis210x_gyro;//i2c_get_clientdata(client);
	int err = 0;
	int cali[FIS210X_GYRO_AXIS_NUM], raw[FIS210X_GYRO_AXIS_NUM];

	FIS210X_GYRO_FUN();
	if(0 != fis210x_gyro_ReadCalibrationEx(client, cali, raw))	/*offset will be updated in obj->offset*/
	{ 
		FIS210X_GYRO_ERR("read offset fail, %d\n", err);
		return err;
	}

	/*calculate the real offset expected by caller*/
	cali[FIS210X_GYRO_AXIS_X] += dat[FIS210X_GYRO_AXIS_X];
	cali[FIS210X_GYRO_AXIS_Y] += dat[FIS210X_GYRO_AXIS_Y];
	cali[FIS210X_GYRO_AXIS_Z] += dat[FIS210X_GYRO_AXIS_Z];

	obj->cali_sw[FIS210X_GYRO_AXIS_X] = obj->cvt.sign[FIS210X_GYRO_AXIS_X]*(cali[obj->cvt.map[FIS210X_GYRO_AXIS_X]]);
	obj->cali_sw[FIS210X_GYRO_AXIS_Y] = obj->cvt.sign[FIS210X_GYRO_AXIS_Y]*(cali[obj->cvt.map[FIS210X_GYRO_AXIS_Y]]);
	obj->cali_sw[FIS210X_GYRO_AXIS_Z] = obj->cvt.sign[FIS210X_GYRO_AXIS_Z]*(cali[obj->cvt.map[FIS210X_GYRO_AXIS_Z]]);	

	return err;
}


int fis210x_gyro_setEnableBits(u8 address, u8 bitmask, bool enable)
{
	u8 data[2];
	int err = 0;

	// Read the current configuration into data buffer
	fis210x_gyro_i2c_read(address, &data[1], 1);

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
	err = fis210x_gyro_i2c_write(data[0], &data[1], 1);

	return err;
}

static int fis210x_gyro_check_id(void)
{
	int err = 0;

	err = fis210x_gyro_i2c_read(FisRegister_WhoAmI, &(fis210x_gyro->chip_id), 1);
	if(err)
	{
		return err;
	}
	FIS210X_GYRO_LOG("chip id=0x%x", fis210x_gyro->chip_id);
	if(fis210x_gyro->chip_id == 0xfc)
		return 0;
	else
		return -1;
}

int fis210x_gyro_set_range_odr(enum FisImu_GyrRange range,enum FisImu_GyrOdr odr)
{
	u8 gyro_setting = 0;
	int err=0;

	switch(range)
	{
		case GyrRange_32dps:
			fis210x_gyro->resolution = (1 << 10);
			break;
		case GyrRange_64dps:
			fis210x_gyro->resolution = (1 << 9);
			break;
		case GyrRange_128dps:
			fis210x_gyro->resolution = (1 << 8);
			break;
		case GyrRange_256dps:
			fis210x_gyro->resolution = (1 << 7);
			break;
		case GyrRange_512dps:
			fis210x_gyro->resolution = (1 << 6);
			break;
		case GyrRange_1024dps:
			fis210x_gyro->resolution = (1 << 5);
			break;
		case GyrRange_2048dps:
			fis210x_gyro->resolution = (1 << 4);
			break;
		case GyrRange_2560dps:
			fis210x_gyro->resolution = (1 << 3);
			break;
		default:
			fis210x_gyro->resolution = (1 << 5);			
	}

	gyro_setting = (u8)range | (u8)odr;
	err = fis210x_gyro_i2c_write(FisRegister_Ctrl3, &gyro_setting, 1);

	return err;
}


int fis210x_gyro_set_filter(enum FisImu_LpfConfig lpfEnable,enum FisImu_HpfConfig hpfEnable)
{
	int err = 0;

	// Configure accelerometer Low Pass Filter enable bit
	err = fis210x_gyro_setEnableBits(FisRegister_Ctrl5, FISIMU_CTRL5_GYR_LPF_ENABLE,lpfEnable == Lpf_Enable);
	// Configure accelerometer High Pass Filter enable bit
	err += fis210x_gyro_setEnableBits(FisRegister_Ctrl5, FISIMU_CTRL5_GYR_HPF_ENABLE,hpfEnable == Hpf_Enable);

	return err;
}

void fis210x_gyro_enable_sensors(u8 enableFlags)
{
	if(enableFlags & FISIMU_CTRL7_AE_ENABLE)
	{
		enableFlags |= FISIMU_CTRL7_ACC_ENABLE | FISIMU_CTRL7_GYR_ENABLE;
	}

	enableFlags = enableFlags & FISIMU_CTRL7_ENABLE_MASK;
	fis210x_gyro_i2c_write(FisRegister_Ctrl7, &enableFlags, 1);
}


int fis210x_gyro_set_mode(enum FisImu_mode mode)
{
#if 0
	u8 reg_value = 0;

	if(mode == FIS_MODE_LOW_POWER)
	{
		//fis210x_gyro_set_range_odr(AccRange_4g, AccOdr_128Hz);
	
		reg_value = 0;
		fis210x_gyro_i2c_write(FisRegister_Ctrl1, &reg_value, 1);
		fis210x_gyro_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else if(mode == FIS_MODE_POWER_DOWN)
	{
		//fis210x_gyro_set_range_odr(AccRange_4g, AccOdr_128Hz);
	
		reg_value = 1;
		fis210x_gyro_i2c_write(FisRegister_Ctrl1, &reg_value, 1);
		fis210x_gyro_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else
	{
		fis210x_gyro_set_range_odr(fis210x_gyro->range, fis210x_gyro->odr);
		
		reg_value = 0;
		fis210x_gyro_i2c_write(FisRegister_Ctrl1, &reg_value, 1);
		fis210x_gyro_enable_sensors(FISIMU_CTRL7_ACC_ENABLE|FISIMU_CTRL7_GYR_ENABLE);
	}
#else
	fis210x_acc_set_mode(mode, 1);
#endif

	return 0;
}

static int fis210x_gyro_init_client(void)
{
	int err=0;

	fis210x_gyro_set_mode(FIS_MODE_NOMAL);
	fis210x_gyro_set_range_odr(fis210x_gyro->range, fis210x_gyro->odr);
	//err = fis210x_gyro_set_filter(Lpf_Disable, Hpf_Disable);

	FIS210X_GYRO_LOG("fis210x acc init OK.\n");
	return err;
}

static int fis210x_gyro_read_raw(int raw_xyz[3])
{
	unsigned char buf_reg[6];
	short read_xyz[3];

#if 0
	fis210x_gyro_i2c_read(FisRegister_Gx_L, &buf_reg[0], 1);		// 0x1f, 31
	fis210x_gyro_i2c_read(FisRegister_Gx_H, &buf_reg[1], 1);
	fis210x_gyro_i2c_read(FisRegister_Gy_L, &buf_reg[2], 1);
	fis210x_gyro_i2c_read(FisRegister_Gy_H, &buf_reg[3], 1);
	fis210x_gyro_i2c_read(FisRegister_Gz_L, &buf_reg[4], 1);
	fis210x_gyro_i2c_read(FisRegister_Gz_H, &buf_reg[5], 1);
#else
	fis210x_gyro_i2c_read(FisRegister_Gx_L, buf_reg, 6); 	// 0x1f, 31
#endif

	read_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	read_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	read_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	raw_xyz[0] = read_xyz[0];
	raw_xyz[1] = read_xyz[1];
	raw_xyz[2] = read_xyz[2];
	//printk("fis210x_gyro_read_raw	%d	%d	%d\n", raw_xyz[0],raw_xyz[1],raw_xyz[2]);

	return 0;
}

static int fis210x_gyro_read_data(int out_xyz[3])
{
	int raw_xyz[3];
	int gyro_xyz[3];

	fis210x_gyro_read_raw(raw_xyz);

	raw_xyz[0] += fis210x_gyro->cali_sw[0];
	raw_xyz[1] += fis210x_gyro->cali_sw[1];
	raw_xyz[2] += fis210x_gyro->cali_sw[2];
	
	//remap coordinate
	gyro_xyz[fis210x_gyro->cvt.map[0]] = fis210x_gyro->cvt.sign[0]*raw_xyz[0];
	gyro_xyz[fis210x_gyro->cvt.map[1]] = fis210x_gyro->cvt.sign[1]*raw_xyz[1];
	gyro_xyz[fis210x_gyro->cvt.map[2]] = fis210x_gyro->cvt.sign[2]*raw_xyz[2];
	
	out_xyz[0] = (gyro_xyz[0]*DEFREE_SCALE)/(fis210x_gyro->resolution);
	out_xyz[1] = (gyro_xyz[1]*DEFREE_SCALE)/(fis210x_gyro->resolution);
	out_xyz[2] = (gyro_xyz[2]*DEFREE_SCALE)/(fis210x_gyro->resolution);

	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", "fis210x");
}

static ssize_t show_init_value(struct device_driver *ddri, char *buf)
{
	fis210x_gyro_init_client();
	return snprintf(buf, PAGE_SIZE, "%s\n", "init done!");
}


static ssize_t show_gyro_range_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", fis210x_gyro->range);
}

static ssize_t store_gyro_range_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	fis210x_gyro->range = data;
	err = fis210x_gyro_set_range_odr(fis210x_gyro->range, fis210x_gyro->odr);
	if (err < 0) {
		FIS210X_GYRO_ERR("set acc range = %d failed.\n", (int)data);
		return err;
	}
	return count;
}

static ssize_t show_gyro_odr_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", fis210x_gyro->odr);
}

static ssize_t store_gyro_odr_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	fis210x_gyro->odr = data;
	err = fis210x_gyro_set_range_odr(fis210x_gyro->range, fis210x_gyro->odr);
	if (err < 0) {
		FIS210X_GYRO_ERR("set acc bandwidth failed.\n");
		return err;
	}

	return count;
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int out_xyz[3];

	fis210x_gyro_read_data(out_xyz);
	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", out_xyz[0],out_xyz[1],out_xyz[2]);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	int err = 0;
	int len = 0;
	int mul;
	int tmp[FIS210X_GYRO_AXIS_NUM] = { 0 };
	fis210x_gyro_t *obj = fis210x_gyro;
	struct i2c_client *client = fis210x_gyro_i2c_client;

	if (err)
		return -EINVAL;
	err = fis210x_gyro_ReadCalibration(client, tmp);
	if (err)
		return -EINVAL;

	mul = 0;//obj->resolution / fis210x_gyro_offset_resolution.sensitivity;
	len +=
		snprintf(buf + len, PAGE_SIZE - len,
			 "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
			 obj->offset[FIS210X_GYRO_AXIS_X], obj->offset[FIS210X_GYRO_AXIS_Y],
			 obj->offset[FIS210X_GYRO_AXIS_Z], obj->offset[FIS210X_GYRO_AXIS_X],
			 obj->offset[FIS210X_GYRO_AXIS_Y], obj->offset[FIS210X_GYRO_AXIS_Z]);
	len +=
		snprintf(buf + len, PAGE_SIZE - len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			 obj->cali_sw[FIS210X_GYRO_AXIS_X], obj->cali_sw[FIS210X_GYRO_AXIS_Y],
			 obj->cali_sw[FIS210X_GYRO_AXIS_Z]);

	len +=
		snprintf(buf + len, PAGE_SIZE - len,
			 "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			 obj->offset[FIS210X_GYRO_AXIS_X] * mul + obj->cali_sw[FIS210X_GYRO_AXIS_X],
			 obj->offset[FIS210X_GYRO_AXIS_Y] * mul + obj->cali_sw[FIS210X_GYRO_AXIS_Y],
			 obj->offset[FIS210X_GYRO_AXIS_Z] * mul + obj->cali_sw[FIS210X_GYRO_AXIS_Z],
			 tmp[FIS210X_GYRO_AXIS_X], tmp[FIS210X_GYRO_AXIS_Y],
			 tmp[FIS210X_GYRO_AXIS_Z]);

	return len;

}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int err, x, y, z;
	int dat[FIS210X_GYRO_AXIS_NUM] = { 0 };
	struct i2c_client *client = fis210x_gyro_i2c_client;

	if (!strncmp(buf, "rst", 3)) {
		err = fis210x_gyro_ResetCalibration(client);
		if (err)
			FIS210X_GYRO_ERR("reset offset err = %d\n", err);
	} else if (sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z) == 3) {
		dat[FIS210X_GYRO_AXIS_X] = x;
		dat[FIS210X_GYRO_AXIS_Y] = y;
		dat[FIS210X_GYRO_AXIS_Z] = z;
		err = fis210x_gyro_WriteCalibration(client, dat);
		if (err)
			FIS210X_GYRO_ERR("write calibration err = %d\n", err);
	} else {
		FIS210X_GYRO_ERR("set calibration value by invalid format.\n");
	}
	return count;
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not support\n");
}

static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	int err;

	err = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&fis210x_gyro->trace));
	return err;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (sscanf(buf, "0x%x", &trace) == 1)
		atomic_set(&fis210x_gyro->trace, trace);
	else
		FIS210X_GYRO_ERR("invalid content: '%s'\n", buf);

	return count;
}


static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	fis210x_gyro_t *data = fis210x_gyro;

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		       data->hw.direction, atomic_read(&data->layout), data->cvt.sign[0],
		       data->cvt.sign[1], data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1],
		       data->cvt.map[2]);
}

static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	fis210x_gyro_t *data = fis210x_gyro;
	int layout = 0;
	int ret = 0;

	if (kstrtos32(buf, 10, &layout) == 0) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {
			FIS210X_GYRO_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw.direction, &data->cvt)) {
			FIS210X_GYRO_ERR("invalid layout: %d, restore to %d\n", layout, data->hw.direction);
		} else {
			FIS210X_GYRO_ERR("invalid layout: (%d, %d)\n", layout, data->hw.direction);
			ret = hwmsen_get_convert(0, &data->cvt);
			if (!ret)
				FIS210X_GYRO_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
	} else {
		FIS210X_GYRO_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}


static ssize_t fis210x_delay_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&fis210x_gyro->delay));
}

static ssize_t fis210x_delay_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long data;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	if (data < 1)
		data = 1;

	atomic_set(&fis210x_gyro->delay, (unsigned int)data);
	return count;
}

static ssize_t show_dumpallreg_value(struct device_driver *ddri, char *buf)
{
	int res;
	int i =0;
	int write_offset = 0;
	unsigned char databuf[2];

	write_offset = 0;
	for(i =0;i<=FisRegister_AeOverflow;i++)
	{
		res = fis210x_gyro_i2c_read(i, &databuf[0], 1);
		if(res)
		{
			FIS210X_GYRO_LOG("qmaX981 dump registers 0x%02x failed !\n", i);
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "error!\n");
		}
		else
		{			
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "0x%2x=0x%2x\n", i, databuf[0]);
		}

	}
	return write_offset;
}

static ssize_t store_setreg(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int addr, value;
	u8 data[C_I2C_FIFO_SIZE];
	int res = 0;
	
	FIS210X_GYRO_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))
	{
		FIS210X_GYRO_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);
		data[0] = (u8)addr;
		data[1] = (u8)value;
		res = fis210x_gyro_i2c_write(data[0], &data[1], 1);

		if(res)
		{
			FIS210X_GYRO_LOG("write reg 0x%02x fail\n", addr);
		}
	}
	else
	{
		FIS210X_GYRO_LOG("store_reg get para error\n");
	}

	return count;
}


static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(init, S_IWUSR | S_IRUGO, show_init_value, NULL);
static DRIVER_ATTR(gyro_range, S_IWUSR | S_IRUGO, show_gyro_range_value, store_gyro_range_value);
static DRIVER_ATTR(gyro_odr, S_IWUSR | S_IRUGO, show_gyro_odr_value, store_gyro_odr_value);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(delay, S_IRUGO | S_IWUSR, fis210x_delay_show, fis210x_delay_store);
static DRIVER_ATTR(dumpallreg,	S_IRUGO, show_dumpallreg_value, NULL);
static DRIVER_ATTR(setreg,		S_IWUSR|S_IWGRP, NULL, store_setreg);


static struct driver_attribute *fis210x_gyro_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_init,	   /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_gyro_range,		/*g sensor range for compass tilt compensation*/
	&driver_attr_gyro_odr,		/*g sensor bandwidth for compass tilt compensation*/
	&driver_attr_layout,
	&driver_attr_delay,	
	&driver_attr_dumpallreg, 
	&driver_attr_setreg
};


static int fis210x_gyro_create_attr(struct device_driver *driver)
{
	int err = 0;
	int idx = 0;
	int num = ARRAY_SIZE(fis210x_gyro_attr_list);

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, fis210x_gyro_attr_list[idx]);
		if (err) {
			FIS210X_GYRO_ERR("create driver file (%s) failed.\n",
				fis210x_gyro_attr_list[idx]->attr.name);
			break;
		}
	}
	return err;
}

static int fis210x_gyro_delete_attr(struct device_driver *driver)
{
	int idx = 0;
	int err = 0;
	int num = ARRAY_SIZE(fis210x_gyro_attr_list);

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, fis210x_gyro_attr_list[idx]);
	return err;
}

#if 0//def CONFIG_PM
static int fis210x_gyro_pm_suspend(void)
{
	FIS210X_GYRO_FUN();

	return 0;
}

static int fis210x_gyro_pm_resume(void)
{
	FIS210X_GYRO_FUN();

	return 0;

}

static int fis210x_gyro_pm_event_handler(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		fis210x_gyro_pm_suspend();
		return NOTIFY_DONE;
	case PM_POST_SUSPEND:
		fis210x_gyro_pm_resume();
		return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

static struct notifier_block fis210x_gyro_pm_notifier_func = {
	.notifier_call = fis210x_gyro_pm_event_handler,
	.priority = 0,
};
#endif				/* CONFIG_PM */


#if defined(CONFIG_PM_SLEEP)&&defined(FIS210X_GYRO_MTK_8_1)
static int fis210x_gyro_suspend(struct device *dev)
{
	fis210x_gyro_set_mode(FIS_MODE_POWER_DOWN);
	atomic_set(&fis210x_gyro->suspend, 1);
	fis210x_gyro_power(&fis210x_gyro->hw, 0);

	return 0;
}

static int fis210x_gyro_resume(struct device *dev)
{
	fis210x_gyro_power(&fis210x_gyro->hw, 1);
	fis210x_gyro_set_mode(FIS_MODE_NOMAL);
	atomic_set(&fis210x_gyro->suspend, 0);

	return 0;
}
#else
static int fis210x_gyro_suspend(struct i2c_client *client, pm_message_t msg)
{
	int err = 0;
	//struct fis210x_gyro_t *obj = i2c_get_clientdata(client);
	FIS210X_GYRO_LOG("fis210x_gyro_suspend");
	
	if(NULL == fis210x_gyro)
	{
		FIS210X_GYRO_LOG("null pointer!!\n");
		return 0;
	}
	if(msg.event == PM_EVENT_SUSPEND)
	{
		err = fis210x_gyro_set_mode(FIS_MODE_POWER_DOWN);
		atomic_set(&fis210x_gyro->suspend, 1);
		
		fis210x_gyro_power(&fis210x_gyro->hw, 0);
	}

	return 0;
}

static int fis210x_gyro_resume(struct i2c_client *client)
{
	//struct fis210x_gyro_t *obj = i2c_get_clientdata(client);
	int err = 0;

	FIS210X_GYRO_LOG("fis210x_gyro_resume");
	if(NULL == fis210x_gyro)
	{
		FIS210X_GYRO_LOG("null pointer!!\n");
		return 0;
	}
	fis210x_gyro_power(&fis210x_gyro->hw, 1);
	err = fis210x_gyro_set_mode(FIS_MODE_NOMAL);
	atomic_set(&fis210x_gyro->suspend, 0);

	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)&&defined(FIS210X_GYRO_MTK_8_1)
static const struct dev_pm_ops fis210x_gyro_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fis210x_gyro_suspend, fis210x_gyro_resume)
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id gyroscope_of_match[] = {
	{.compatible = "mediatek,gyroscope",},
	{.compatible = "mediatek,gyro",},
	{},
};
#endif

static struct i2c_driver fis210x_gyro_i2c_driver = {
	.driver = 
	{
		.name = FIS210X_GYRO_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gyroscope_of_match,
#endif
	},
	.probe = fis210x_gyro_i2c_probe,
	.remove = fis210x_gyro_i2c_remove,
	.id_table = fis210x_gyro_i2c_id,
#ifndef FIS210X_GYRO_MTK_8_1
	.suspend = fis210x_gyro_suspend,
	.resume = fis210x_gyro_resume,
#endif
};

static int fis210x_gyro_open_report_data(int open)
{
	return 0;
}


#ifdef CUSTOM_KERNEL_SENSORHUB
int fis210x_gyro_scp_SetPowerMode(int enable, int sensorType)
{
	static unsigned int gyroscope_scp_en_map;
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;

	mutex_lock(&qmaX981_scp_mutex);

	if (sensorType >= 32) {		
		mutex_unlock(&qmaX981_scp_mutex);
		QMAX981_ERR("Out of index!\n");
		return -1;
	}

	if (1 == enable)
		gyroscope_scp_en_map |= (1 << sensorType);
	else
		gyroscope_scp_en_map &= ~(1 << sensorType);


	if (0 == gyroscope_scp_en_map)
		enable = 0;
	else
		enable = 1;

	if (qmaX981->scp_mode != enable) {
		qmaX981->scp_mode = enable;

		req.activate_req.sensorType = ID_ACCELEROMETER;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
		if (err)
			QMAX981_ERR("SCP_sensorHub_req_send fail!\n");
	}

	mutex_unlock(&qmaX981_scp_mutex);

	return err;
}
EXPORT_SYMBOL(qmaX981_scp_SetPowerMode);

static int fis210x_gyro_scp_enable_nodata(int en)
{
	fis210x_gyro_scp_SetPowerMode(en, ID_ACCELEROMETER);
}

#else
static int fis210x_gyro_enable_nodata(int en)
{
	int err = 0;
	bool power = false;

	if(en == 1)
	{
		power = true;
		// add by yangzhiqiang for test
		//fis210x_gyro_check_id();		
		fis210x_gyro_init_client();		
		mdelay(200);
	}
	else
	{
		power = false;
		err = fis210x_gyro_set_mode(FIS_MODE_POWER_DOWN);
	}

	if(err < 0) {
		FIS210X_GYRO_ERR("fis210x_gyro_SetPowerMode failed.\n");
		return -1;
	}
	FIS210X_GYRO_LOG("fis210x_gyro_enable_nodata ok!\n");
	return err;
}
#endif	/* #ifdef CUSTOM_KERNEL_SENSORHUB */

static int fis210x_gyro_set_delay(u64 ns)
{
    int msec;
	int err = 0;
	int sample_odr = 0;

    msec = (int)ns/1000/1000;
	if (msec <= 5)
		sample_odr = GyrOdr_1024Hz;
	else if (msec <= 10)
		sample_odr = GyrOdr_256Hz;
	else
		sample_odr = GyrOdr_128Hz;

	fis210x_gyro->odr = sample_odr;	
	atomic_set(&fis210x_gyro->delay, msec);
	err = fis210x_gyro_set_range_odr(fis210x_gyro->range, fis210x_gyro->odr);


	return err;
}


#ifdef FIS210X_GYRO_MTK_8_1
static int fis210x_gyro_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;
	int sample_odr = 0;
	int err = 0;

	value = (int)samplingPeriodNs/1000/1000;
	if (value <= 5)
		sample_odr = GyrOdr_1024Hz;
	else if (value <= 10)
		sample_odr = GyrOdr_256Hz;
	else
		sample_odr = GyrOdr_128Hz;

	fis210x_gyro->odr = sample_odr;		
	atomic_set(&fis210x_gyro->delay, value);
	err = fis210x_gyro_set_range_odr(fis210x_gyro->range, fis210x_gyro->odr);
	if (err < 0) {
		FIS210X_GYRO_ERR("set delay parameter error!\n");
		return -1;
	}
	FIS210X_GYRO_LOG("fis210x acc set delay = (%d) ok.\n", value);
	return 0;
}

static int fis210x_gyro_flush(void)
{
	return gyro_flush_report();
}
#endif

static int fis210x_gyro_get_data(int* x ,int* y,int* z, int* status)
{
	int data[3]={0};
	int ret=0;

	ret = fis210x_gyro_read_data(data);
 
	*x = data[0];
	*y = data[1];
	*z = data[2]; 
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return ret;
}


static int fis210x_gyro_get_raw_data(int *x, int *y, int *z)
{
	int data[3]={0};
	int ret=0;

	ret = fis210x_gyro_read_raw(data);	
	*x = data[0];
	*y = data[1];
	*z = data[2]; 

	return ret;
}

#if defined(FIS210X_GYRO_MTK_8_1)
static int fis210x_gyro_get_temperature(int *temperature)
{
	*temperature = 25;
	return 0;
}
#endif


#if defined(FIS210X_GYRO_CREATE_MISC_DEV)
static int fis210x_gyro_open(struct inode *inode, struct file *file)
{
	file->private_data = fis210x_gyro_i2c_client;
	if (file->private_data == NULL) {
		FIS210X_GYRO_ERR("file->private_data is null pointer.\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int fis210x_gyro_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long fis210x_gyro_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char strbuf[FIS210X_BUFSIZE] = { 0 };
	void __user *data;
#ifdef FIS210X_GYRO_MTK_KK
	SENSOR_DATA sensor_data;
#else
	struct SENSOR_DATA sensor_data;
#endif
	int err = 0;
	int smtRes = 0;
	int cali[3] = { 0 };	
	int value[3] = {0};
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	fis210x_gyro_t *obj = fis210x_gyro;

	FIS210X_GYRO_LOG("fis210x_gyro_unlocked_ioctl %d\n", cmd);
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		FIS210X_GYRO_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}
	switch (cmd) {
	case GYROSCOPE_IOCTL_INIT:
		fis210x_gyro_init_client();
		break;
/*
	case GYROSCOPE_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		sprintf(strbuf, "fis210x_gyro");
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
			err = -EFAULT;
		break;
*/
	case GYROSCOPE_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		fis210x_gyro_read_data(value);
		FIS210X_GYRO_LOG("gyro io read: %d %d %d \n", value[0],value[1],value[2]);
		sprintf(strbuf, "%x %x %x", value[0], value[1], value[2]);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
			err = -EFAULT;
		break;
#ifdef GYROSCOPE_IOCTL_SMT_DATA
	case GYROSCOPE_IOCTL_SMT_DATA:
		data = (void __user *)arg;
		smtRes = 1;
		if(copy_to_user(data, &smtRes,  sizeof(smtRes)))
			return -EFAULT;
		return 0;
#endif
	case GYROSCOPE_IOCTL_READ_SENSORDATA_RAW:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		fis210x_gyro_read_raw(value);
		sprintf(strbuf, "%x %x %x", value[0], value[1], value[2]);
		if (copy_to_user(data, &strbuf, strlen(strbuf) + 1))
			err = -EFAULT;
		break;
	case GYROSCOPE_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		if (atomic_read(&obj->suspend)) {
			FIS210X_GYRO_ERR("can't perform calibration in suspend state.\n");
			err = -EINVAL;
		} else {
			cali[FIS210X_GYRO_AXIS_X] = sensor_data.x * obj->resolution / DEFREE_SCALE;
			cali[FIS210X_GYRO_AXIS_Y] = sensor_data.y * obj->resolution / DEFREE_SCALE;
			cali[FIS210X_GYRO_AXIS_Z] = sensor_data.z * obj->resolution / DEFREE_SCALE;
			err = fis210x_gyro_WriteCalibration(client, cali);
		}
		break;
	case GYROSCOPE_IOCTL_CLR_CALI:
		err = fis210x_gyro_ResetCalibration(client);
		break;
	case GYROSCOPE_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = fis210x_gyro_ReadCalibration(client, cali);
		if (err) {
			FIS210X_GYRO_ERR("read calibration failed.\n");
			break;
		}
		sensor_data.x = cali[FIS210X_GYRO_AXIS_X] * DEFREE_SCALE / obj->resolution;
		sensor_data.y = cali[FIS210X_GYRO_AXIS_Y] * DEFREE_SCALE / obj->resolution;
		sensor_data.z = cali[FIS210X_GYRO_AXIS_Z] * DEFREE_SCALE / obj->resolution;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			err = -EFAULT;
		break;
#ifdef GYROSCOPE_IOCTL_ENABLE_CALI
	case GYROSCOPE_IOCTL_ENABLE_CALI:
		err = 0;
		break;
#endif
	default:
		FIS210X_GYRO_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}
	return err;
}

#ifdef CONFIG_COMPAT
static long fis210x_gyro_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if(!file->f_op || !file->f_op->unlocked_ioctl) 
	{
		FIS210X_GYRO_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch(cmd) 
	{
	case COMPAT_GYROSCOPE_IOCTL_INIT:
	case COMPAT_GYROSCOPE_IOCTL_SMT_DATA:
	case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA_RAW:
	case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA:
		/* NVRAM will use below ioctl */
	case COMPAT_GYROSCOPE_IOCTL_SET_CALI:
	case COMPAT_GYROSCOPE_IOCTL_CLR_CALI:
	case COMPAT_GYROSCOPE_IOCTL_GET_CALI:
#ifdef COMPAT_GYROSCOPE_IOCTL_ENABLE_CALI
	case COMPAT_GYROSCOPE_IOCTL_ENABLE_CALI:
#endif
		FIS210X_GYRO_LOG("compat_ion_ioctl : GYROSCOPE_IOCTL_XXX command is 0x%x\n", cmd);
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));
	default:
		GYRO_PR_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}

#endif


static const struct file_operations fis210x_gyro_fops = {
	.owner = THIS_MODULE,
	.open = fis210x_gyro_open,
	.release = fis210x_gyro_release,
	.unlocked_ioctl = fis210x_gyro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fis210x_gyro_compat_ioctl,
#endif
};

static struct miscdevice fis210x_gyro_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gyroscope",
	.fops = &fis210x_gyro_fops,
};
#else
static int fis210x_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = fis210x_gyro_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		FIS210X_GYRO_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = fis210x_gyro_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		FIS210X_GYRO_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int fis210x_factory_get_data(int32_t data[3], int *status)
{
	int err = 0;
	int out_xyz[3];

	err = fis210x_gyro_read_data(out_xyz);
	data[0] = out_xyz[0];
	data[1] = out_xyz[1];
	data[2] = out_xyz[2];
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;	

	return 0;
}

static int fis210x_factory_get_raw_data(int32_t data[3])
{
	int raw_xyz[3];

	fis210x_gyro_read_raw(raw_xyz);
	data[0] = raw_xyz[0];
	data[1] = raw_xyz[1];
	data[2] = raw_xyz[2];
	FIS210X_GYRO_ERR("don't support fis210x_factory_get_raw_data!\n");
	return 0;
}
static int fis210x_factory_enable_calibration(void)
{
	return 0;
}
static int fis210x_factory_clear_cali(void)
{
	int err = 0;

	err = fis210x_gyro_ResetCalibration(fis210x_gyro_i2c_client);
	if (err) {
		FIS210X_GYRO_ERR("fis210x_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int fis210x_factory_set_cali(int32_t data[3])
{
	fis210x_gyro_t *obj = fis210x_gyro;
	int err = 0;
	int cali[FIS210X_GYRO_AXIS_NUM] = { 0 };

	cali[FIS210X_GYRO_AXIS_X] = data[0]*obj->resolution/DEFREE_SCALE;
	cali[FIS210X_GYRO_AXIS_Y] = data[1]*obj->resolution/DEFREE_SCALE;
	cali[FIS210X_GYRO_AXIS_Z] = data[2]*obj->resolution/DEFREE_SCALE;
	err = fis210x_gyro_WriteCalibration(fis210x_gyro_i2c_client, cali);
	if (err) {
		FIS210X_GYRO_ERR("fis210x_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int fis210x_factory_get_cali(int32_t data[3])
{
	fis210x_gyro_t *obj = fis210x_gyro;
	int err = 0;
	int cali[3] = { 0 };

	err = fis210x_gyro_ReadCalibration(fis210x_gyro_i2c_client, cali);
	if (err) {
		FIS210X_GYRO_ERR("fis210x_ReadCalibration failed!\n");
		return -1;
	}
	data[0] = cali[FIS210X_GYRO_AXIS_X]*DEFREE_SCALE/obj->resolution;
	data[1] = cali[FIS210X_GYRO_AXIS_Y]*DEFREE_SCALE/obj->resolution;
	data[2] = cali[FIS210X_GYRO_AXIS_Z]*DEFREE_SCALE/obj->resolution;

	return 0;
}
static int fis210x_factory_do_self_test(void)
{
	return 0;
}

static struct gyro_factory_fops fis210x_factory_fops = 
{
	.enable_sensor = fis210x_factory_enable_sensor,
	.get_data = fis210x_factory_get_data,
	.get_raw_data = fis210x_factory_get_raw_data,
	.enable_calibration = fis210x_factory_enable_calibration,
	.clear_cali = fis210x_factory_clear_cali,
	.set_cali = fis210x_factory_set_cali,
	.get_cali = fis210x_factory_get_cali,
	.do_self_test = fis210x_factory_do_self_test,
};

static struct gyro_factory_public fis210x_gyro_factory_device = 
{
	.gain = 1,
	.sensitivity = 1,
	.fops = &fis210x_factory_fops,
};
#endif

static int fis210x_gyro_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;	
	fis210x_gyro_t *obj = NULL;
	struct gyro_control_path gyro_ctl = {0};
	struct gyro_data_path gyro_data = {0};

	FIS210X_GYRO_FUN();
	obj = kzalloc(sizeof(fis210x_gyro_t), GFP_KERNEL);
	if (obj == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}
	fis210x_gyro = obj;
	fis210x_gyro_i2c_client = client;
	
#if defined(FIS210X_GYRO_MTK_8_1)
	err = get_gyro_dts_func(client->dev.of_node, &obj->hw);
#elif defined(FIS210X_GYRO_MTK_KK)
	memcpy(&obj->hw, get_cust_gyro_hw(), sizeof(struct gyro_hw));
#else
	get_gyro_dts_func("mediatek,gyroscope", &obj->hw);
#endif
	if(err) 
	{
		FIS210X_GYRO_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_kfree;
	}

	fis210x_gyro_power(&obj->hw, 1);
	fis210x_gyro->uint = GyrUnit_rads;
	fis210x_gyro->range = GyrRange_1024dps;	//GyrRange_1024dps;
	fis210x_gyro->odr = GyrOdr_1024Hz;
	fis210x_gyro->scale = 1000;

	fis210x_gyro_i2c_client = client;
	fis210x_gyro_i2c_client->addr = 0x6a;
	FIS210X_GYRO_LOG("acc not init i2c addr =0x%x \n", fis210x_gyro_i2c_client->addr);

	if(fis210x_gyro_check_id())
	{
		FIS210X_GYRO_ERR("fis210x_gyro_check_id error\n");
		goto exit_kfree;
	}

	err = fis210x_gyro_init_client();	
	fis210x_gyro_set_mode(FIS_MODE_POWER_DOWN);
	if(err)
	{
		FIS210X_GYRO_ERR("fis210x_gyro_init_client error\n");
		goto exit_kfree;
	}
	
	i2c_set_clientdata(client, obj);

	if(0 != (err = hwmsen_get_convert(obj->hw.direction, &obj->cvt)))
	{
		FIS210X_GYRO_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_kfree;
	}

	atomic_set(&obj->layout,obj->hw.direction);	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

#if defined(FIS210X_GYRO_CREATE_MISC_DEV)
	err = misc_register(&fis210x_gyro_misc_device);
	if(err)
	{
		FIS210X_GYRO_ERR("misc register failed\n");
		goto exit_kfree;
	}
#else
	err = gyro_factory_device_register(&fis210x_gyro_factory_device);
	if(err) {
		FIS210X_GYRO_ERR("fis210x_factory_device register failed.\n");
		goto exit_kfree;
	}
#endif
	err = fis210x_gyro_create_attr(&(fis210x_gyro_init_info.platform_diver_addr->driver));
	if (err) {
		FIS210X_GYRO_ERR("create attribute failed.\n");
		goto exit_kfree;
	}

	gyro_ctl.open_report_data = fis210x_gyro_open_report_data;
#ifdef CUSTOM_KERNEL_SENSORHUB
	gyro_ctl.enable_nodata = fis210x_gyro_scp_enable_nodata;
#else
	gyro_ctl.enable_nodata = fis210x_gyro_enable_nodata;
#endif
	gyro_ctl.set_delay = fis210x_gyro_set_delay;
#if defined(FIS210X_GYRO_MTK_8_1)
	gyro_ctl.batch = fis210x_gyro_batch;
	gyro_ctl.flush = fis210x_gyro_flush;
#endif
	gyro_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	gyro_ctl.is_support_batch = obj->hw.is_batch_supported;
#else
	gyro_ctl.is_support_batch = false;
#endif
	err = gyro_register_control_path(&gyro_ctl);
	if(err)
	{
		FIS210X_GYRO_ERR("register gyro control path err\n");
		goto exit_attr;
	}

	gyro_data.get_data = fis210x_gyro_get_data;
	gyro_data.get_raw_data = fis210x_gyro_get_raw_data;
#if defined(FIS210X_GYRO_MTK_8_1)
	gyro_data.get_temperature = fis210x_gyro_get_temperature;
#endif
	gyro_data.vender_div = DEGREE_TO_RAD;		//1000;
	err = gyro_register_data_path(&gyro_data);
	if(err)
	{
		FIS210X_GYRO_ERR("register gyro data path err\n");
		goto exit_attr;
	}
	
#if 0//def CONFIG_PM
	err = register_pm_notifier(&fis210x_gyro_pm_notifier_func);
	if (err) {
		FIS210X_GYRO_ERR("Failed to register PM notifier.\n");
		goto exit_kfree;
	}
#endif	/* CONFIG_PM */

	fis210x_gyro_init_flag = 0;
	return 0;

	exit_attr:
#if defined(FIS210X_GYRO_CREATE_MISC_DEV)
	misc_deregister(&fis210x_gyro_misc_device);
#else
	gyro_factory_device_deregister(&fis210x_gyro_factory_device);
#endif
	fis210x_gyro_delete_attr(&(fis210x_gyro_init_info.platform_diver_addr->driver));
	exit_kfree:
	kfree(obj);
	fis210x_gyro = NULL;
	exit:
	fis210x_gyro_init_flag = -1;

	return err;
}

static int fis210x_gyro_i2c_remove(struct i2c_client *client)
{
	fis210x_gyro_delete_attr(&(fis210x_gyro_init_info.platform_diver_addr->driver));
#if defined(FIS210X_GYRO_CREATE_MISC_DEV)
	misc_deregister(&fis210x_gyro_misc_device);
#else
	gyro_factory_device_deregister(&fis210x_gyro_factory_device);
#endif
	i2c_unregister_device(client);
	
	fis210x_gyro_i2c_client = NULL;
	if(fis210x_gyro)
	{
		kfree(fis210x_gyro);
		fis210x_gyro=NULL;
	}
	
	return 0;
}

static int fis210x_gyro_local_init(struct platform_device *pdev)
{
	FIS210X_GYRO_FUN();
	if(i2c_add_driver(&fis210x_gyro_i2c_driver))
	{
		FIS210X_GYRO_ERR("add driver error\n");
		printk("[gyro-fis210x] add driver error!\n");
		return -1;
	}
	if(-1 == fis210x_gyro_init_flag)
		return -1;
	FIS210X_GYRO_LOG("fis210x gyro local init.\n");
	return 0;
}

static int fis210x_gyro_remove(void)
{
	i2c_del_driver(&fis210x_gyro_i2c_driver);
	return 0;
}

static int __init fis210x_gyro_init(void)
{
	FIS210X_GYRO_FUN();
#if defined(FIS210X_GYRO_MTK_KK)
	i2c_register_board_info(0, &i2c_fis210x_gyro, 1);
#endif
	gyro_driver_add(&fis210x_gyro_init_info);
	return 0;
}

static void __exit fis210x_gyro_exit(void)
{
	FIS210X_GYRO_FUN();
}


module_init(fis210x_gyro_init);
module_exit(fis210x_gyro_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("fis210x_gyro I2C driver");
MODULE_AUTHOR("zhiqiang_yang@qstcorp.com");



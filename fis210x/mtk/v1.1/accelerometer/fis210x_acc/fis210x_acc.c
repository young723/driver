
#include <accel.h>
#include <cust_acc.h>

//#include <sensors_io.h>
//#include <hwmsensor.h>
//#include <hwmsen_dev.h> 
//#include <hwmsen_helper.h>
#include "fis210x_acc.h"

#define FIS210X_BUFSIZE		256
#define I2C_FIFO_SIZE	8

#if defined(FIS210X_ACC_MTK_KK)
extern struct acc_hw *fis210x_get_cust_acc_hw(void);
static struct i2c_board_info __initdata i2c_fis210x={I2C_BOARD_INFO(FIS210X_ACC_DEV_NAME, (FIS210X_I2C_SLAVE_ADDR))};
#endif
static const struct i2c_device_id fis210x_acc_i2c_id[] = {{FIS210X_ACC_DEV_NAME, 0}, {} };

static int fis210x_acc_local_init(void);
static int fis210x_acc_remove(void);
static int fis210x_acc_init_client(void);

#if defined(USE_SPI)
static int fis210x_acc_spi_probe(struct spi_device *spi);
static int fis210x_acc_spi_remove(struct spi_device *spi);
#else
static int fis210x_acc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int fis210x_acc_i2c_remove(struct i2c_client *client);
#endif


struct i2c_client *fis210x_acc_i2c_client=NULL;
static struct acc_init_info fis210x_acc_init_info = 
{
	.name = FIS210X_ACC_DEV_NAME,
	.init = fis210x_acc_local_init,
	.uninit = fis210x_acc_remove,
};

static fis210x_acc_t *fis210x_acc=NULL;
static int fis210x_acc_init_flag = -1;

#if defined(USE_SPI)
int fis210x_spi_read_bytes(struct spi_device *spi, u16 addr, u8 *rx_buf, u32 data_len)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;
	u32 package, reminder, retry;

	package = (data_len + 2) / 1024;
	reminder = (data_len + 2) % 1024;

	if ((package > 0) && (reminder != 0)) {
		xfer = kzalloc(sizeof(*xfer) * 4, GFP_KERNEL);
		retry = 1;
	} else {
		xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
		retry = 0;
	}
	if (xfer == NULL) {
		FIS210X_ACC_ERR("%s, no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	//mutex_lock(&accelgyro_obj_data->spi_lock);
	tmp_buf = fis210x_acc->spi_buffer;

	spi_message_init(&msg);
	memset(tmp_buf, 0, 1 + data_len);
	if(data_len > 1)
		*tmp_buf = (u8) (addr & 0xFF) | 0xC0;
	else
		*tmp_buf = (u8) (addr & 0xFF) | 0x80;
		
	xfer[0].tx_buf = tmp_buf;
	xfer[0].rx_buf = tmp_buf;
	xfer[0].len = 1 + data_len;
	xfer[0].delay_usecs = 5;

	xfer[0].speed_hz = 4 * 1000 * 1000;	/*4M */

	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(spi, &msg);
	memcpy(rx_buf, tmp_buf + 1, data_len);

	if(xfer)
	{
		kfree(xfer);
		xfer = NULL;
	}
	//mutex_unlock(&accelgyro_obj_data->spi_lock);

	return 0;
}

int fis210x_spi_write_bytes(struct spi_device *spi, u16 addr, u8 *tx_buf, u32 data_len)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;
	u32 package, reminder, retry;

	package = (data_len + 1) / 1024;
	reminder = (data_len + 1) % 1024;

	if ((package > 0) && (reminder != 0)) {
		xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
		retry = 1;
	} else {
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
		retry = 0;
	}
	if (xfer == NULL) {
		FIS210X_ACC_ERR("%s, no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}
	tmp_buf = fis210x_acc->spi_buffer;

	//mutex_lock(&fis210x_acc->spi_lock);
	spi_message_init(&msg);
	if(data_len > 1)
		*tmp_buf = (u8) (addr & 0x7F);
	else
		*tmp_buf = (u8) (addr & 0x3F);

	if (retry) {
		memcpy(tmp_buf + 1, tx_buf, (package * 1024 - 1));
		xfer[0].len = package * 1024;
	} else {
		memcpy(tmp_buf + 1, tx_buf, data_len);
		xfer[0].len = data_len + 1;
	}
	xfer[0].tx_buf = tmp_buf;
	xfer[0].delay_usecs = 5;

	xfer[0].speed_hz = 4 * 1000 * 1000;

	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(spi, &msg);

	if (retry) {
		addr = addr + package * 1024 - 1;
		spi_message_init(&msg);
		*tmp_buf = (u8) (addr & 0xFF);
		memcpy(tmp_buf + 1, (tx_buf + package * 1024 - 1), reminder);
		xfer[1].tx_buf = tmp_buf;
		xfer[1].len = reminder + 1;
		xfer[1].delay_usecs = 5;

		xfer[0].speed_hz = 4 * 1000 * 1000;

		spi_message_add_tail(&xfer[1], &msg);
		spi_sync(spi, &msg);
	}

	if(xfer)
	{
		kfree(xfer);
		xfer = NULL;
	}
	//mutex_unlock(&fis210x_acc->spi_lock);

	return 0;
}
#endif

#if 0
/* I2C operation functions */
static int fis210x_acc_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
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

	//else if (len > I2C_FIFO_SIZE) {
	//	FIS210X_ACC_ERR(" length %d exceeds %d\n", len, I2C_FIFO_SIZE);
	//	return -EINVAL;
	//}
	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != 2) {
		FIS210X_ACC_ERR("i2c_transfer error: %x %x (%d %p %d) %d\n",
				msgs[0].addr, client->addr, addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}
	return err;
}


static int fis210x_acc_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	/*
	 *because address also occupies one byte,
	 *the maximum length for write is 7 bytes
	 */
	int err, idx = 0, num = 0;
	char buf[32];

	if(!client)
		return -EINVAL;
	//else if (len > I2C_FIFO_SIZE) {
	//	FIS210X_ACC_ERR(" length %d exceeds %d\n", len, I2C_FIFO_SIZE);
	//	return -EINVAL;
	//}
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if(err < 0) {
		FIS210X_ACC_ERR("send command error!!\n");
		return -EFAULT;
	}
	err = 0;

	return err;
}
#endif

int fis210x_acc_i2c_read(u8 reg_addr, u8 *data, u8 len)
{
	int ret;

#if defined(USE_SPI)
	ret = fis210x_spi_read_bytes(fis210x_acc->spi_dev, reg_addr, data, len);
#else
	if(len > 1)
	{
		reg_addr |= 0x80;
	}
	ret = i2c_smbus_read_i2c_block_data(fis210x_acc_i2c_client, reg_addr, len, data);
#endif

	if(ret < 0)
		return -EFAULT;
	else
		return 0;

}
EXPORT_SYMBOL(fis210x_acc_i2c_read);

int fis210x_acc_i2c_write(u8 reg_addr, u8 *data, u8 len)
{
#if defined(USE_SPI)
	int ret = 0;

	ret = fis210x_spi_write_bytes(fis210x_acc->spi_dev, reg_addr, data, len);

	return ret;
#else
	int err, loop_i;
	unsigned char txbuf[I2C_FIFO_SIZE];

	err =0;
	if((!fis210x_acc_i2c_client)||(!data)||(len >= I2C_FIFO_SIZE-1)) 
	{
		FIS210X_ACC_ERR(" client or length %d exceeds %d\n", len, I2C_FIFO_SIZE);
		return -EINVAL;
	}
	txbuf[0] = reg_addr;
	for(loop_i = 0; loop_i < len; loop_i++)
	{
		txbuf[1+loop_i] = data[loop_i];
	}

	fis210x_acc_i2c_client->addr = fis210x_acc_i2c_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < 5; loop_i++)
	{
		err = i2c_master_send(fis210x_acc_i2c_client, txbuf, len+1);
		if(err < 0)
		{
			FIS210X_ACC_ERR("try:%d,i2c_master_send error:%d\n",loop_i,  err);
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
#endif
}
EXPORT_SYMBOL(fis210x_acc_i2c_write);


struct i2c_client *fis210x_get_i2c_client(void)
{
	return fis210x_acc_i2c_client;
}

EXPORT_SYMBOL(fis210x_get_i2c_client);


static void fis210x_acc_power(struct acc_hw *hw, unsigned int on)
{

}

/*!
 * @brief Reset calibration for acc
 *
 * @param[in] client the pointer of i2c_client
 *
 * @return zero success, non-zero failed
 */
static int fis210x_acc_ResetCalibration(struct i2c_client *client)
{
	fis210x_acc_t *obj = fis210x_acc;//i2c_get_clientdata(client);
	int err = 0;

	FIS210X_ACC_LOG("fis210x_acc_ResetCalibration\n");
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));

	return err;    
}

static int fis210x_acc_ReadCalibration(struct i2c_client *client, int dat[FIS210X_ACC_AXIS_NUM])
{
	fis210x_acc_t *obj = fis210x_acc;//i2c_get_clientdata(client);
	int mul;
	
	FIS210X_ACC_FUN();
	mul = 0;//only SW Calibration, disable HW Calibration
	
	dat[obj->cvt.map[FIS210X_ACC_AXIS_X]] = obj->cvt.sign[FIS210X_ACC_AXIS_X]*(obj->offset[FIS210X_ACC_AXIS_X]*mul + obj->cali_sw[FIS210X_ACC_AXIS_X]);
	dat[obj->cvt.map[FIS210X_ACC_AXIS_Y]] = obj->cvt.sign[FIS210X_ACC_AXIS_Y]*(obj->offset[FIS210X_ACC_AXIS_Y]*mul + obj->cali_sw[FIS210X_ACC_AXIS_Y]);
	dat[obj->cvt.map[FIS210X_ACC_AXIS_Z]] = obj->cvt.sign[FIS210X_ACC_AXIS_Z]*(obj->offset[FIS210X_ACC_AXIS_Z]*mul + obj->cali_sw[FIS210X_ACC_AXIS_Z]); 				   

	return 0;
}

static int fis210x_acc_ReadCalibrationEx(struct i2c_client *client, int act[FIS210X_ACC_AXIS_NUM], int raw[FIS210X_ACC_AXIS_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	fis210x_acc_t *obj = fis210x_acc;//i2c_get_clientdata(client);
	int mul;

	mul = 0;//only SW Calibration, disable HW Calibration
	FIS210X_ACC_LOG("fis210x_acc_ReadCalibrationEx\n");
	raw[FIS210X_ACC_AXIS_X] = obj->offset[FIS210X_ACC_AXIS_X]*mul + obj->cali_sw[FIS210X_ACC_AXIS_X];
	raw[FIS210X_ACC_AXIS_Y] = obj->offset[FIS210X_ACC_AXIS_Y]*mul + obj->cali_sw[FIS210X_ACC_AXIS_Y];
	raw[FIS210X_ACC_AXIS_Z] = obj->offset[FIS210X_ACC_AXIS_Z]*mul + obj->cali_sw[FIS210X_ACC_AXIS_Z];

	act[obj->cvt.map[FIS210X_ACC_AXIS_X]] = obj->cvt.sign[FIS210X_ACC_AXIS_X]*raw[FIS210X_ACC_AXIS_X];
	act[obj->cvt.map[FIS210X_ACC_AXIS_Y]] = obj->cvt.sign[FIS210X_ACC_AXIS_Y]*raw[FIS210X_ACC_AXIS_Y];
	act[obj->cvt.map[FIS210X_ACC_AXIS_Z]] = obj->cvt.sign[FIS210X_ACC_AXIS_Z]*raw[FIS210X_ACC_AXIS_Z];						  
						   
	return 0;
}

static int fis210x_acc_WriteCalibration(struct i2c_client *client, int dat[FIS210X_ACC_AXIS_NUM])
{
	fis210x_acc_t *obj = fis210x_acc;//i2c_get_clientdata(client);
	int err = 0;
	int cali[FIS210X_ACC_AXIS_NUM], raw[FIS210X_ACC_AXIS_NUM];

	FIS210X_ACC_FUN();
	if(0 != fis210x_acc_ReadCalibrationEx(client, cali, raw))	/*offset will be updated in obj->offset*/
	{ 
		FIS210X_ACC_ERR("read offset fail, %d\n", err);
		return err;
	}

	/*calculate the real offset expected by caller*/
	cali[FIS210X_ACC_AXIS_X] += dat[FIS210X_ACC_AXIS_X];
	cali[FIS210X_ACC_AXIS_Y] += dat[FIS210X_ACC_AXIS_Y];
	cali[FIS210X_ACC_AXIS_Z] += dat[FIS210X_ACC_AXIS_Z];

	obj->cali_sw[FIS210X_ACC_AXIS_X] = obj->cvt.sign[FIS210X_ACC_AXIS_X]*(cali[obj->cvt.map[FIS210X_ACC_AXIS_X]]);
	obj->cali_sw[FIS210X_ACC_AXIS_Y] = obj->cvt.sign[FIS210X_ACC_AXIS_Y]*(cali[obj->cvt.map[FIS210X_ACC_AXIS_Y]]);
	obj->cali_sw[FIS210X_ACC_AXIS_Z] = obj->cvt.sign[FIS210X_ACC_AXIS_Z]*(cali[obj->cvt.map[FIS210X_ACC_AXIS_Z]]);	

	return err;
}


int fis210x_acc_setEnableBits(u8 address, u8 bitmask, bool enable)
{
	u8 data[2];
	int err = 0;

	// Read the current configuration into data buffer
	fis210x_acc_i2c_read(address, &data[1], 1);

	// Update the required enable bits according to bitmask
	if (enable)
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
	err = fis210x_acc_i2c_write(data[0], &data[1], 1);

	return err;
}

static int fis210x_acc_check_id(void)
{
	int err = 0;
	int icount = 0;
	u8 chip_id[2];

	icount = 0;
	while(fis210x_acc->chip_id != 0xfc)
	{
		err = fis210x_acc_i2c_read(FisRegister_WhoAmI, chip_id, 1);
		fis210x_acc->chip_id = chip_id[0];
		//if(err)
		//{
		//	return err;
		//}
		mdelay(5);
		if(icount++>2)
		{
			break;
		}
	}
	FIS210X_ACC_LOG("chip id=0x%x", fis210x_acc->chip_id);
	if(fis210x_acc->chip_id == 0xfc)
		return 0;
	else
		return -1;
}

int fis210x_acc_set_range_odr(enum FisImu_AccRange range,enum FisImu_AccOdr odr)
{
	u8 acc_setting = 0;
	int err=0;

	switch(range)
	{
		case AccRange_2g:
			fis210x_acc->resolution = (1 << 14);
			break;
		case AccRange_4g:
			fis210x_acc->resolution = (1 << 13);
			break;
		case AccRange_8g:
			fis210x_acc->resolution = (1 << 12);
			break;
		case AccRange_16g:
			fis210x_acc->resolution = (1 << 11);
			break;
		default:			
			fis210x_acc->resolution = (1 << 14);
			break;
	}

	acc_setting = (u8)range | (u8)odr;
	err = fis210x_acc_i2c_write(FisRegister_Ctrl2, &acc_setting, 1);

	return err;
}


int fis210x_acc_set_filter(enum FisImu_LpfConfig lpfEnable,enum FisImu_HpfConfig hpfEnable)
{
	int err = 0;
	// Configure accelerometer Low Pass Filter enable bit
	err = fis210x_acc_setEnableBits(FisRegister_Ctrl5, FISIMU_CTRL5_ACC_LPF_ENABLE,lpfEnable == Lpf_Enable);
	// Configure accelerometer High Pass Filter enable bit
	err += fis210x_acc_setEnableBits(FisRegister_Ctrl5, FISIMU_CTRL5_ACC_HPF_ENABLE,hpfEnable == Hpf_Enable);

	return err;
}

void fis210x_acc_enable_sensors(u8 enableFlags)
{
	if(enableFlags & FISIMU_CTRL7_AE_ENABLE)
	{
		enableFlags |= FISIMU_CTRL7_ACC_ENABLE | FISIMU_CTRL7_GYR_ENABLE;
	}

	enableFlags = enableFlags & FISIMU_CTRL7_ENABLE_MASK;
	fis210x_acc_i2c_write(FisRegister_Ctrl7, &enableFlags, 1);
}


int fis210x_acc_set_mode(enum FisImu_mode mode, u8 power_flag)
{
	u8 reg_value = 0;

	if(fis210x_acc == NULL)
	{
		FIS210X_ACC_LOG("fis210x_acc_set_mode NULL return! ");
		return 0;
	}
	if((fis210x_acc->mode_acc == FIS_MODE_NOMAL)||(fis210x_acc->mode_gyro == FIS_MODE_NOMAL))
	{
		FIS210X_ACC_LOG("fis210x_acc_set_mode return! ");

		if(power_flag == 0)
			fis210x_acc->mode_acc = mode;
		else
			fis210x_acc->mode_gyro = mode;
		return 0;
	}

	if(power_flag == 0)
		fis210x_acc->mode_acc = mode;
	else
		fis210x_acc->mode_gyro = mode;

	if(mode == FIS_MODE_LOW_POWER)
	{
		fis210x_acc_set_range_odr(AccRange_4g, AccOdr_128Hz);
	
		reg_value = 0;
		fis210x_acc_i2c_write(FisRegister_Ctrl1, &reg_value, 1);

		fis210x_acc_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else if(mode == FIS_MODE_POWER_DOWN)
	{
		fis210x_acc_set_range_odr(AccRange_4g, AccOdr_128Hz);
	
		reg_value = 1;
		fis210x_acc_i2c_write(FisRegister_Ctrl1, &reg_value, 1);

		fis210x_acc_enable_sensors(FISIMU_CTRL7_DISABLE_ALL);
	}
	else
	{
		fis210x_acc_set_range_odr(fis210x_acc->range, fis210x_acc->odr);
		
		reg_value = 0;
		fis210x_acc_i2c_write(FisRegister_Ctrl1, &reg_value, 1);
		fis210x_acc_enable_sensors(FISIMU_CTRL7_ACC_ENABLE|FISIMU_CTRL7_GYR_ENABLE);
	}

	return 0;
}

static int fis210x_acc_init_client(void)
{
	int err = 0;

	fis210x_acc_set_mode(FIS_MODE_NOMAL, 0);
	//err = fis210x_acc_set_filter(Lpf_Disable, Hpf_Disable);

	FIS210X_ACC_LOG("fis210x acc init OK.\n");
	
	return err;
}

static int fis210x_acc_read_raw(int raw_xyz[3])
{
	unsigned char buf_reg[6];
	short read_xyz[3];

#if 0
	fis210x_acc_i2c_read(FisRegister_Ax_L, &buf_reg[0], 1);		// 0x19, 25
	fis210x_acc_i2c_read(FisRegister_Ax_H, &buf_reg[1], 1);
	fis210x_acc_i2c_read(FisRegister_Ay_L, &buf_reg[2], 1);
	fis210x_acc_i2c_read(FisRegister_Ay_H, &buf_reg[3], 1);
	fis210x_acc_i2c_read(FisRegister_Az_L, &buf_reg[4], 1);
	fis210x_acc_i2c_read(FisRegister_Az_H, &buf_reg[5], 1);
#else
	fis210x_acc_i2c_read(FisRegister_Ax_L, buf_reg, 6); 	// 0x19, 25
#endif

	read_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	read_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	read_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	raw_xyz[0] = read_xyz[0];
	raw_xyz[1] = read_xyz[1];
	raw_xyz[2] = read_xyz[2];
//	FIS210X_ACC_LOG("fis210x_acc_read_raw	%d	%d	%d\n", raw_xyz[0],raw_xyz[1],raw_xyz[2]);

	return 0;
}

static int fis210x_acc_read_data(int out_xyz[3])
{
	int raw_xyz[3];
	int acc_xyz[3];

	fis210x_acc_read_raw(raw_xyz);

	raw_xyz[0] += fis210x_acc->cali_sw[0];
	raw_xyz[1] += fis210x_acc->cali_sw[1];
	raw_xyz[2] += fis210x_acc->cali_sw[2];
	
	//remap coordinate
	acc_xyz[fis210x_acc->cvt.map[0]] = fis210x_acc->cvt.sign[0]*raw_xyz[0];
	acc_xyz[fis210x_acc->cvt.map[1]] = fis210x_acc->cvt.sign[1]*raw_xyz[1];
	acc_xyz[fis210x_acc->cvt.map[2]] = fis210x_acc->cvt.sign[2]*raw_xyz[2];
	
	out_xyz[0] = (acc_xyz[0]*GRAVITY_EARTH_1000)/(fis210x_acc->resolution);
	out_xyz[1] = (acc_xyz[1]*GRAVITY_EARTH_1000)/(fis210x_acc->resolution);
	out_xyz[2] = (acc_xyz[2]*GRAVITY_EARTH_1000)/(fis210x_acc->resolution);

	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", "fis210x");
}

static ssize_t show_init_value(struct device_driver *ddri, char *buf)
{
	fis210x_acc_init_client();
	return snprintf(buf, PAGE_SIZE, "%s\n", "init done!");
}

static ssize_t show_acc_range_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", fis210x_acc->range);
}

static ssize_t store_acc_range_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	fis210x_acc->range = data;
	err = fis210x_acc_set_range_odr(fis210x_acc->range, fis210x_acc->odr);
	if (err < 0) {
		FIS210X_ACC_ERR("set acc range = %d failed.\n", (int)data);
		return err;
	}
	return count;
}

static ssize_t show_acc_odr_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", fis210x_acc->odr);
}

static ssize_t store_acc_odr_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	fis210x_acc->odr = data;
	err = fis210x_acc_set_range_odr(fis210x_acc->range, fis210x_acc->odr);
	if (err < 0) {
		FIS210X_ACC_ERR("set acc bandwidth failed.\n");
		return err;
	}

	return count;
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int out_xyz[3];

	fis210x_acc_read_data(out_xyz);
	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", out_xyz[0],out_xyz[1],out_xyz[2]);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	int err = 0;
	int len = 0;
	int mul;
	int tmp[FIS210X_ACC_AXIS_NUM] = { 0 };
	fis210x_acc_t *obj = fis210x_acc;
	struct i2c_client *client = fis210x_acc_i2c_client;

	if (err)
		return -EINVAL;
	err = fis210x_acc_ReadCalibration(client, tmp);
	if (err)
		return -EINVAL;

	mul = 0;//obj->resolution / fis210x_acc_offset_resolution.sensitivity;
	len +=
		snprintf(buf + len, PAGE_SIZE - len,
			 "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
			 obj->offset[FIS210X_ACC_AXIS_X], obj->offset[FIS210X_ACC_AXIS_Y],
			 obj->offset[FIS210X_ACC_AXIS_Z], obj->offset[FIS210X_ACC_AXIS_X],
			 obj->offset[FIS210X_ACC_AXIS_Y], obj->offset[FIS210X_ACC_AXIS_Z]);
	len +=
		snprintf(buf + len, PAGE_SIZE - len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			 obj->cali_sw[FIS210X_ACC_AXIS_X], obj->cali_sw[FIS210X_ACC_AXIS_Y],
			 obj->cali_sw[FIS210X_ACC_AXIS_Z]);

	len +=
		snprintf(buf + len, PAGE_SIZE - len,
			 "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			 obj->offset[FIS210X_ACC_AXIS_X] * mul + obj->cali_sw[FIS210X_ACC_AXIS_X],
			 obj->offset[FIS210X_ACC_AXIS_Y] * mul + obj->cali_sw[FIS210X_ACC_AXIS_Y],
			 obj->offset[FIS210X_ACC_AXIS_Z] * mul + obj->cali_sw[FIS210X_ACC_AXIS_Z],
			 tmp[FIS210X_ACC_AXIS_X], tmp[FIS210X_ACC_AXIS_Y],
			 tmp[FIS210X_ACC_AXIS_Z]);

	return len;

}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int err, x, y, z;
	int dat[FIS210X_ACC_AXIS_NUM] = { 0 };
	struct i2c_client *client = fis210x_acc_i2c_client;

	if (!strncmp(buf, "rst", 3)) {
		err = fis210x_acc_ResetCalibration(client);
		if (err)
			FIS210X_ACC_ERR("reset offset err = %d\n", err);
	} else if (sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z) == 3) {
		dat[FIS210X_ACC_AXIS_X] = x;
		dat[FIS210X_ACC_AXIS_Y] = y;
		dat[FIS210X_ACC_AXIS_Z] = z;
		err = fis210x_acc_WriteCalibration(client, dat);
		if (err)
			FIS210X_ACC_ERR("write calibration err = %d\n", err);
	} else {
		FIS210X_ACC_ERR("set calibration value by invalid format.\n");
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

	err = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&fis210x_acc->trace));
	return err;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (sscanf(buf, "0x%x", &trace) == 1)
		atomic_set(&fis210x_acc->trace, trace);
	else
		FIS210X_ACC_ERR("invalid content: '%s'\n", buf);

	return count;
}


static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	fis210x_acc_t *data = fis210x_acc;

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		       data->hw.direction, atomic_read(&data->layout), data->cvt.sign[0],
		       data->cvt.sign[1], data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1],
		       data->cvt.map[2]);
}

static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	fis210x_acc_t *data = fis210x_acc;
	int layout = 0;
	int ret = 0;

	if (kstrtos32(buf, 10, &layout) == 0) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {
			FIS210X_ACC_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw.direction, &data->cvt)) {
			FIS210X_ACC_ERR("invalid layout: %d, restore to %d\n", layout, data->hw.direction);
		} else {
			FIS210X_ACC_ERR("invalid layout: (%d, %d)\n", layout, data->hw.direction);
			ret = hwmsen_get_convert(0, &data->cvt);
			if (!ret)
				FIS210X_ACC_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
	} else {
		FIS210X_ACC_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}


static ssize_t fis210x_delay_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&fis210x_acc->delay));
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

	atomic_set(&fis210x_acc->delay, (unsigned int)data);
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
		res = fis210x_acc_i2c_read(i, &databuf[0], 1);
		if(res)
		{
			FIS210X_ACC_LOG("fis210x dump registers 0x%02x failed !\n", i);
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
	u8 data[I2C_FIFO_SIZE];
	int res = 0;
	
	FIS210X_ACC_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))
	{
		FIS210X_ACC_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);
		data[0] = (u8)addr;
		data[1] = (u8)value;
		res = fis210x_acc_i2c_write(data[0], &data[1], 1);

		if(res)
		{
			FIS210X_ACC_LOG("write reg 0x%02x fail\n", addr);
		}
	}
	else
	{
		FIS210X_ACC_LOG("store_reg get para error\n");
	}

	return count;
}


static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(init, S_IWUSR | S_IRUGO, show_init_value, NULL);
static DRIVER_ATTR(acc_range, S_IWUSR | S_IRUGO, show_acc_range_value, store_acc_range_value);
static DRIVER_ATTR(acc_odr, S_IWUSR | S_IRUGO, show_acc_odr_value, store_acc_odr_value);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(delay, S_IRUGO | S_IWUSR, fis210x_delay_show, fis210x_delay_store);
static DRIVER_ATTR(dumpallreg,	S_IRUGO, show_dumpallreg_value, NULL);
static DRIVER_ATTR(setreg,		S_IWUSR|S_IWGRP, NULL, store_setreg);


static struct driver_attribute *fis210x_acc_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_init,	   /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_acc_range,		/*g sensor range for compass tilt compensation*/
	&driver_attr_acc_odr,		/*g sensor bandwidth for compass tilt compensation*/
	&driver_attr_layout,
	&driver_attr_delay,	
	&driver_attr_dumpallreg, 
	&driver_attr_setreg
};


static int fis210x_acc_create_attr(struct device_driver *driver)
{
	int err = 0;
	int idx = 0;
	int num = ARRAY_SIZE(fis210x_acc_attr_list);

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, fis210x_acc_attr_list[idx]);
		if (err) {
			FIS210X_ACC_ERR("create driver file (%s) failed.\n",
				fis210x_acc_attr_list[idx]->attr.name);
			break;
		}
	}
	return err;
}

static int fis210x_acc_delete_attr(struct device_driver *driver)
{
	int idx = 0;
	int err = 0;
	int num = ARRAY_SIZE(fis210x_acc_attr_list);

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, fis210x_acc_attr_list[idx]);
	return err;
}

#if 0//def CONFIG_PM
static int fis210x_acc_pm_suspend(void)
{
	FIS210X_ACC_FUN();

	return 0;
}

static int fis210x_acc_pm_resume(void)
{
	FIS210X_ACC_FUN();

	return 0;

}

static int fis210x_acc_pm_event_handler(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		fis210x_acc_pm_suspend();
		return NOTIFY_DONE;
	case PM_POST_SUSPEND:
		fis210x_acc_pm_resume();
		return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

static struct notifier_block fis210x_acc_pm_notifier_func = {
	.notifier_call = fis210x_acc_pm_event_handler,
	.priority = 0,
};
#endif				/* CONFIG_PM */


#if defined(CONFIG_PM_SLEEP)&&defined(FIS210X_ACC_MTK_8_1)
static int fis210x_acc_suspend(struct device *dev)
{
	fis210x_acc_set_mode(FIS_MODE_POWER_DOWN, 0);
	atomic_set(&fis210x_acc->suspend, 1);
	fis210x_acc_power(&fis210x_acc->hw, 0);

	return 0;
}

static int fis210x_acc_resume(struct device *dev)
{
	fis210x_acc_power(&fis210x_acc->hw, 1);
	fis210x_acc_set_mode(FIS_MODE_NOMAL, 0);
	atomic_set(&fis210x_acc->suspend, 0);

	return 0;
}
#else
static int fis210x_acc_suspend(struct i2c_client *client, pm_message_t msg)
{
	//struct fis210x_acc_t *obj = i2c_get_clientdata(client);
	FIS210X_ACC_LOG("fis210x_acc_suspend");
	
	if(NULL == fis210x_acc)
	{
		FIS210X_ACC_LOG("null pointer!!\n");
		return 0;
	}
	if(msg.event == PM_EVENT_SUSPEND)
	{
		fis210x_acc_set_mode(FIS_MODE_POWER_DOWN, 0);
		atomic_set(&fis210x_acc->suspend, 1);
		
		fis210x_acc_power(&fis210x_acc->hw, 0);
	}

	return 0;
}

static int fis210x_acc_resume(struct i2c_client *client)
{
	//struct fis210x_acc_t *obj = i2c_get_clientdata(client);
	int err = 0;

	FIS210X_ACC_LOG("fis210x_acc_resume");
	if(NULL == fis210x_acc)
	{
		FIS210X_ACC_LOG("null pointer!!\n");
		return 0;
	}
	fis210x_acc_power(&fis210x_acc->hw, 1);
	fis210x_acc_set_mode(FIS_MODE_NOMAL, 0);
	atomic_set(&fis210x_acc->suspend, 0);

	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)&&defined(FIS210X_ACC_MTK_8_1)
static const struct dev_pm_ops fis210x_acc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fis210x_acc_suspend, fis210x_acc_resume)
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{.compatible = "mediatek,gsensor",},
	{},
};
#endif

#if defined(USE_SPI)
static struct spi_device_id fis210x_acc_spi_id = {FIS210X_ACC_DEV_NAME, 0};

static struct spi_driver fis210x_acc_spi_driver = {
	.driver = {
		   .name = FIS210X_ACC_DEV_NAME,
		   .bus = &spi_bus_type,
#ifdef CONFIG_OF
		   .of_match_table = gsensor_of_match,
#endif
		   },
	.probe = fis210x_acc_spi_probe,
	.remove = fis210x_acc_spi_remove,
	.id_table = &fis210x_acc_spi_id,
};

#if 0
static struct mt_chip_conf fis210x_spi_conf = {
    .setuptime=10,
    .holdtime=10,
    .high_time=10, // 10--6m 15--4m 20--3m 30--2m [ 60--1m 120--0.5m  300--0.2m]
    .low_time=10,
    .cs_idletime=10,
    .ulthgh_thrsh=0,
    .cpol=1,
    .cpha=1,
    .rx_mlsb=SPI_MSB,
    .tx_mlsb=SPI_MSB,
    .tx_endian=0,
    .rx_endian=0,
    .com_mod=FIFO_TRANSFER,
    .pause=0,
    .finish_intr=5,
    .deassert=0,
    .ulthigh=0,
    .tckdly=0,
};
#endif

static struct spi_board_info fis210x_spi_info[] __initdata = {
	[0] = {
		.modalias = FIS210X_ACC_DEV_NAME,
		.max_speed_hz = (6*1000000),
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_3,
		//.controller_data = &fis210x_spi_conf,
	},
};

#else

static struct i2c_driver fis210x_acc_i2c_driver = {
	.driver = 
	{
		.name = FIS210X_ACC_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
#endif
	},
	.probe = fis210x_acc_i2c_probe,
	.remove = fis210x_acc_i2c_remove,
	.id_table = fis210x_acc_i2c_id,	
#ifndef FIS210X_ACC_MTK_8_1
	.suspend = fis210x_acc_suspend,
	.resume = fis210x_acc_resume,
#endif
};

#endif

static int fis210x_acc_open_report_data(int open)
{
	return 0;
}


#ifdef CUSTOM_KERNEL_SENSORHUB
int fis210x_acc_scp_SetPowerMode(int enable, int sensorType)
{
	static unsigned int gsensor_scp_en_map;
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
		gsensor_scp_en_map |= (1 << sensorType);
	else
		gsensor_scp_en_map &= ~(1 << sensorType);


	if (0 == gsensor_scp_en_map)
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

static int fis210x_acc_scp_enable_nodata(int en)
{
	fis210x_acc_scp_SetPowerMode(en, ID_ACCELEROMETER);
}

#else
static int fis210x_acc_enable_nodata(int en)
{
	int err = 0;
	bool power = false;

	if(en == 1)
	{
		power = true;
		fis210x_acc_init_client();
		mdelay(200);
	}
	else
	{
		power = false;
		err = fis210x_acc_set_mode(FIS_MODE_POWER_DOWN, 0);
	}

	if(err < 0) {
		FIS210X_ACC_ERR("fis210x_acc_SetPowerMode failed.\n");
		return -1;
	}
	FIS210X_ACC_LOG("fis210x_acc_enable_nodata ok!\n");
	return err;
}
#endif	/* #ifdef CUSTOM_KERNEL_SENSORHUB */

static int fis210x_acc_set_delay(u64 ns)
{
    int msec;
	int err = 0;
	int sample_odr = 0;

    msec = (int)ns/1000/1000;
	if (msec <= 5)
		sample_odr = AccOdr_1024Hz;
	else if (msec <= 10)
		sample_odr = AccOdr_256Hz;
	else
		sample_odr = AccOdr_128Hz;

	fis210x_acc->odr = sample_odr;	
	atomic_set(&fis210x_acc->delay, msec);
	err = fis210x_acc_set_range_odr(fis210x_acc->range, fis210x_acc->odr);


	return err;
}


#ifdef FIS210X_ACC_MTK_8_1
static int fis210x_acc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;
	int sample_odr = 0;
	int err = 0;

	value = (int)samplingPeriodNs/1000/1000;
	if (value <= 5)
		sample_odr = AccOdr_1024Hz;
	else if (value <= 10)
		sample_odr = AccOdr_256Hz;
	else
		sample_odr = AccOdr_128Hz;

	fis210x_acc->odr = sample_odr;		
	atomic_set(&fis210x_acc->delay, value);
	err = fis210x_acc_set_range_odr(fis210x_acc->range, fis210x_acc->odr);
	if (err < 0) {
		FIS210X_ACC_ERR("set delay parameter error!\n");
		return -1;
	}
	FIS210X_ACC_LOG("fis210x acc set delay = (%d) ok.\n", value);
	return 0;
}

static int fis210x_acc_flush(void)
{
	return acc_flush_report();
}
#endif

static int fis210x_acc_get_data(int* x ,int* y,int* z, int* status)
{
	int data[3]={0};
	int ret=0;

	ret = fis210x_acc_read_data(data);
 
	*x = data[0];
	*y = data[1];
	*z = data[2]; 
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return ret;
}


static int fis210x_acc_get_raw_data(int *x, int *y, int *z)
{
	int data[3]={0};
	int ret=0;

	ret = fis210x_acc_read_raw(data);	
	*x = data[0];
	*y = data[1];
	*z = data[2]; 

	return ret;
}



#if defined(FIS210X_ACC_CREATE_MISC_DEV)
static int fis210x_acc_open(struct inode *inode, struct file *file)
{
	file->private_data = fis210x_acc_i2c_client;
	if (file->private_data == NULL) {
		FIS210X_ACC_ERR("file->private_data is null pointer.\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int fis210x_acc_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long fis210x_acc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char strbuf[FIS210X_BUFSIZE] = { 0 };
	void __user *data;
#if defined(FIS210X_ACC_MTK_KK)
	SENSOR_DATA sensor_data;
#else
	struct SENSOR_DATA sensor_data;
#endif
	int err = 0;
	int cali[3] = { 0 };	
	int value[3] = {0};
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	fis210x_acc_t *obj = fis210x_acc;

	FIS210X_ACC_ERR("fis210x_acc_unlocked_ioctl %d\n", cmd);
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		FIS210X_ACC_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}
	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		fis210x_acc_init_client();
		break;
	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		sprintf(strbuf, "fis210x_acc");
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
			err = -EFAULT;
		break;
	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		fis210x_acc_read_data(value);
		sprintf(strbuf, "%x %x %x", value[0], value[1], value[2]);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
			err = -EFAULT;
		break;
/*
	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			err = -EFAULT;
		break;
*/
	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		fis210x_acc_read_raw(value);
		sprintf(strbuf, "%x %x %x", value[0], value[1], value[2]);
		if (copy_to_user(data, &strbuf, strlen(strbuf) + 1))
			err = -EFAULT;
		break;
	case GSENSOR_IOCTL_SET_CALI:
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
			FIS210X_ACC_ERR("can't perform calibration in suspend state.\n");
			err = -EINVAL;
		} else {
			cali[FIS210X_ACC_AXIS_X] = sensor_data.x
			    * obj->resolution / GRAVITY_EARTH_1000;
			cali[FIS210X_ACC_AXIS_Y] = sensor_data.y
			    * obj->resolution / GRAVITY_EARTH_1000;
			cali[FIS210X_ACC_AXIS_Z] = sensor_data.z
			    * obj->resolution / GRAVITY_EARTH_1000;
			err = fis210x_acc_WriteCalibration(client, cali);
		}
		break;
	case GSENSOR_IOCTL_CLR_CALI:
		err = fis210x_acc_ResetCalibration(client);
		break;
	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = fis210x_acc_ReadCalibration(client, cali);
		if (err) {
			FIS210X_ACC_ERR("read calibration failed.\n");
			break;
		}
		sensor_data.x = cali[FIS210X_ACC_AXIS_X]
		    * GRAVITY_EARTH_1000 / obj->resolution;
		sensor_data.y = cali[FIS210X_ACC_AXIS_Y]
		    * GRAVITY_EARTH_1000 / obj->resolution;
		sensor_data.z = cali[FIS210X_ACC_AXIS_Z]
		    * GRAVITY_EARTH_1000 / obj->resolution;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			err = -EFAULT;
		break;
#ifdef GSENSOR_IOCTL_ENABLE_CALI
	case GSENSOR_IOCTL_ENABLE_CALI:
		FIS210X_ACC_LOG("GSENSOR_IOCTL_ENABLE_CALI\n");
		err = 0;
		break;
#endif
	default:
		FIS210X_ACC_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

#ifdef CONFIG_COMPAT
static long fis210x_acc_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	/* FIS210X_ACC_ERR("fis210x_acc_compat_ioctl cmd:0x%x\n", cmd); */

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA,
					       (unsigned long)arg32);
		if (err) {
			FIS210X_ACC_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) {
			FIS210X_ACC_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		if (err) {
			FIS210X_ACC_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		if (err) {
			FIS210X_ACC_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
#ifdef COMPAT_GSENSOR_IOCTL_ENABLE_CALI
	case COMPAT_GSENSOR_IOCTL_ENABLE_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
	
		err =
			file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_ENABLE_CALI, (unsigned long)arg32);
		if (err) {
			FIS210X_ACC_ERR("GSENSOR_IOCTL_ENABLE_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
#endif
	default:
		FIS210X_ACC_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}
#endif


static const struct file_operations fis210x_acc_fops = {
	.owner = THIS_MODULE,
	.open = fis210x_acc_open,
	.release = fis210x_acc_release,
	.unlocked_ioctl = fis210x_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fis210x_acc_compat_ioctl,
#endif
};

static struct miscdevice fis210x_acc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &fis210x_acc_fops,
};
#else
static int fis210x_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = fis210x_acc_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		FIS210X_ACC_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = fis210x_acc_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		FIS210X_ACC_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int fis210x_factory_get_data(int32_t data[3], int *status)
{
	int err = 0;
	int out_xyz[3];

	err = fis210x_acc_read_data(out_xyz);
	data[0] = out_xyz[0];
	data[1] = out_xyz[1];
	data[2] = out_xyz[2];
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;	

	return 0;
}

static int fis210x_factory_get_raw_data(int32_t data[3])
{
	int raw_xyz[3];

	fis210x_acc_read_raw(raw_xyz);
	data[0] = raw_xyz[0];
	data[1] = raw_xyz[1];
	data[2] = raw_xyz[2];
	FIS210X_ACC_ERR("don't support fis210x_factory_get_raw_data!\n");
	return 0;
}
static int fis210x_factory_enable_calibration(void)
{
	return 0;
}
static int fis210x_factory_clear_cali(void)
{
	int err = 0;

	err = fis210x_acc_ResetCalibration(fis210x_acc_i2c_client);
	if (err) {
		FIS210X_ACC_ERR("fis210x_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int fis210x_factory_set_cali(int32_t data[3])
{
	fis210x_acc_t *obj = fis210x_acc;
	int err = 0;
	int cali[FIS210X_ACC_AXIS_NUM] = { 0 };

	cali[FIS210X_ACC_AXIS_X] = data[0]*obj->resolution/GRAVITY_EARTH_1000;
	cali[FIS210X_ACC_AXIS_Y] = data[1]*obj->resolution/GRAVITY_EARTH_1000;
	cali[FIS210X_ACC_AXIS_Z] = data[2]*obj->resolution/GRAVITY_EARTH_1000;
	err = fis210x_acc_WriteCalibration(fis210x_acc_i2c_client, cali);
	if (err) {
		FIS210X_ACC_ERR("fis210x_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int fis210x_factory_get_cali(int32_t data[3])
{
	fis210x_acc_t *obj = fis210x_acc;
	int err = 0;
	int cali[3] = { 0 };

	err = fis210x_acc_ReadCalibration(fis210x_acc_i2c_client, cali);
	if (err) {
		FIS210X_ACC_ERR("fis210x_ReadCalibration failed!\n");
		return -1;
	}
	data[0] = cali[FIS210X_ACC_AXIS_X]*GRAVITY_EARTH_1000/obj->resolution;
	data[1] = cali[FIS210X_ACC_AXIS_Y]*GRAVITY_EARTH_1000/obj->resolution;
	data[2] = cali[FIS210X_ACC_AXIS_Z]*GRAVITY_EARTH_1000/obj->resolution;

	return 0;
}
static int fis210x_factory_do_self_test(void)
{
	return 0;
}

static struct accel_factory_fops fis210x_factory_fops = 
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

static struct accel_factory_public fis210x_factory_device = 
{
	.gain = 1,
	.sensitivity = 1,
	.fops = &fis210x_factory_fops,
};
#endif

#if defined(USE_SPI)
static int fis210x_acc_spi_probe(struct spi_device *spi)
{
	int err = 0;	
	fis210x_acc_t *obj = NULL;
	struct acc_control_path acc_ctl = {0};
	struct acc_data_path acc_data = {0};

	FIS210X_ACC_FUN();
	obj = kzalloc(sizeof(fis210x_acc_t), GFP_KERNEL);
	if (obj == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}
	fis210x_acc = obj;	
	fis210x_acc_i2c_client = NULL;
	obj->spi_buffer = kzalloc(15*1024, GFP_KERNEL);
	if(obj->spi_buffer == NULL)
	{
		FIS210X_ACC_ERR("kzalloc spi_buffer fail.\n");
		goto exit;
	}

	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 4*1000*1000;
	fis210x_acc->spi_dev = spi;

	err = get_accel_dts_func(spi->dev.of_node, &obj->hw);

	if(err) 
	{
		FIS210X_ACC_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_kfree;
	}
	fis210x_acc_power(&obj->hw, 1);
	err = fis210x_acc_check_id();
	if(err)
	{
		FIS210X_ACC_ERR("fis210x_acc_check_id error\n");
		goto exit_kfree;
	}
	FIS210X_ACC_LOG("direction =0x%x \n", obj->hw.direction);
	if(0 != (err = hwmsen_get_convert(obj->hw.direction, &obj->cvt)))
	{
		FIS210X_ACC_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_kfree;
	}

	atomic_set(&obj->layout,obj->hw.direction); 
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	spi_set_drvdata(spi, obj);
	fis210x_acc->uint = AccUnit_ms2;
	fis210x_acc->range = AccRange_4g;
	fis210x_acc->odr = AccOdr_256Hz;
	fis210x_acc->mode_acc = FIS_MODE_POWER_DOWN;
	fis210x_acc->mode_gyro = FIS_MODE_POWER_DOWN;

	if(fis210x_acc->uint == AccUnit_ms2)
		fis210x_acc->scale = 9807;
	else
		fis210x_acc->scale = 1000;
	err = fis210x_acc_init_client();	
	// after init set power down
	fis210x_acc_set_mode(FIS_MODE_POWER_DOWN, 0);
	// after init set power down
	if(err)
	{
		FIS210X_ACC_ERR("fis210x_acc_init_client error\n");
		goto exit_kfree;
	}
#if defined(FIS210X_ACC_CREATE_MISC_DEV)
	err = misc_register(&fis210x_acc_misc_device);
	if(err)
	{
		FIS210X_ACC_ERR("misc register failed\n");
		goto exit_kfree;
	}
#else
	err = accel_factory_device_register(&fis210x_factory_device);
	if(err) {
		FIS210X_ACC_ERR("fis210x_factory_device register failed.\n");
		goto exit_kfree;
	}
#endif
	err = fis210x_acc_create_attr(&(fis210x_acc_init_info.platform_diver_addr->driver));
	if (err) {
		FIS210X_ACC_ERR("create attribute failed.\n");
		goto exit_kfree;
	}

	acc_ctl.open_report_data= fis210x_acc_open_report_data;
#ifdef CUSTOM_KERNEL_SENSORHUB
	acc_ctl.enable_nodata = fis210x_acc_scp_enable_nodata;
#else
	acc_ctl.enable_nodata = fis210x_acc_enable_nodata;
#endif
	acc_ctl.set_delay = fis210x_acc_set_delay;
#if defined(FIS210X_ACC_MTK_8_1)
	acc_ctl.batch = fis210x_acc_batch;
	acc_ctl.flush = fis210x_acc_flush;
#endif
	acc_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	acc_ctl.is_support_batch = obj->hw.is_batch_supported;
#else
	acc_ctl.is_support_batch = false;
#endif
	err = acc_register_control_path(&acc_ctl);
	if(err)
	{
		FIS210X_ACC_ERR("register acc control path err\n");
		goto exit_attr;
	}

	acc_data.get_data = fis210x_acc_get_data;
	acc_data.get_raw_data = fis210x_acc_get_raw_data;
	acc_data.vender_div = 1000;
	err = acc_register_data_path(&acc_data);
	if(err)
	{
		FIS210X_ACC_ERR("register acc data path err\n");
		goto exit_attr;
	}
	
#if 0//def CONFIG_PM
	err = register_pm_notifier(&fis210x_acc_pm_notifier_func);
	if (err) {
		FIS210X_ACC_ERR("Failed to register PM notifier.\n");
		goto exit_kfree;
	}
#endif	/* CONFIG_PM */

	fis210x_acc_init_flag = 0;
	return 0;

	exit_attr:
#if defined(FIS210X_ACC_CREATE_MISC_DEV)
	misc_deregister(&fis210x_acc_misc_device);
#else
	accel_factory_device_deregister(&fis210x_factory_device);
#endif
	fis210x_acc_delete_attr(&(fis210x_acc_init_info.platform_diver_addr->driver));
	exit_kfree:
	kfree(obj);
	fis210x_acc = NULL;
	exit:
	fis210x_acc_init_flag = -1;

	return err;
}

static int fis210x_acc_spi_remove(struct spi_device *spi)
{
	fis210x_acc_delete_attr(&(fis210x_acc_init_info.platform_diver_addr->driver));
	
#if defined(FIS210X_ACC_CREATE_MISC_DEV)
	misc_deregister(&fis210x_acc_misc_device);
#else
	accel_factory_device_deregister(&fis210x_factory_device);
#endif
	spi_set_drvdata(spi, NULL);
	if(fis210x_acc)
	{
		if(fis210x_acc->spi_buffer)
		{
			kfree(fis210x_acc->spi_buffer);
			fis210x_acc->spi_buffer = NULL;
		}
		kfree(fis210x_acc);
		fis210x_acc=NULL;
	}

	return 0;
}

#else
static int fis210x_acc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;	
	fis210x_acc_t *obj = NULL;
	struct acc_control_path acc_ctl = {0};
	struct acc_data_path acc_data = {0};

	FIS210X_ACC_FUN();
	obj = kzalloc(sizeof(fis210x_acc_t), GFP_KERNEL);
	if (obj == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}
	fis210x_acc = obj;	
	fis210x_acc_i2c_client = client;

#if defined(FIS210X_ACC_MTK_8_1)
	err = get_accel_dts_func(client->dev.of_node, &obj->hw);
#elif defined(FIS210X_ACC_MTK_KK)
	memcpy(&obj->hw, fis210x_get_cust_acc_hw(), sizeof(struct acc_hw));
#else
	get_accel_dts_func("mediatek,gsensor", &obj->hw);
#endif
	if(err) 
	{
		FIS210X_ACC_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_kfree;
	}	

	fis210x_acc_power(&obj->hw, 1);
	client->addr = 0x6a;
	//client->addr = 0x6b;
	FIS210X_ACC_LOG("i2c addr =0x%x \n", fis210x_acc_i2c_client->addr);

	err = fis210x_acc_check_id();
	if(err)
	{
		FIS210X_ACC_ERR("fis210x_acc_check_id error\n");
		goto exit_kfree;
	}
	FIS210X_ACC_LOG("direction =0x%x \n", obj->hw.direction);
	if(0 != (err = hwmsen_get_convert(obj->hw.direction, &obj->cvt)))
	{
		FIS210X_ACC_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_kfree;
	}

	atomic_set(&obj->layout,obj->hw.direction);	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	i2c_set_clientdata(client, obj);
	fis210x_acc->uint = AccUnit_ms2;
	fis210x_acc->range = AccRange_4g;
	fis210x_acc->odr = AccOdr_256Hz;
	fis210x_acc->mode_acc = FIS_MODE_POWER_DOWN;
	fis210x_acc->mode_gyro = FIS_MODE_POWER_DOWN;

	if(fis210x_acc->uint == AccUnit_ms2)
		fis210x_acc->scale = 9807;
	else
		fis210x_acc->scale = 1000;
	err = fis210x_acc_init_client();	
	// after init set power down
	fis210x_acc_set_mode(FIS_MODE_POWER_DOWN, 0);
	// after init set power down
	if(err)
	{
		FIS210X_ACC_ERR("fis210x_acc_init_client error\n");
		goto exit_kfree;
	}
#if defined(FIS210X_ACC_CREATE_MISC_DEV)
	err = misc_register(&fis210x_acc_misc_device);
	if(err)
	{
		FIS210X_ACC_ERR("misc register failed\n");
		goto exit_kfree;
	}
#else
	err = accel_factory_device_register(&fis210x_factory_device);
	if(err) {
		FIS210X_ACC_ERR("fis210x_factory_device register failed.\n");
		goto exit_kfree;
	}
#endif
	err = fis210x_acc_create_attr(&(fis210x_acc_init_info.platform_diver_addr->driver));
	if (err) {
		FIS210X_ACC_ERR("create attribute failed.\n");
		goto exit_kfree;
	}

	acc_ctl.open_report_data= fis210x_acc_open_report_data;
#ifdef CUSTOM_KERNEL_SENSORHUB
	acc_ctl.enable_nodata = fis210x_acc_scp_enable_nodata;
#else
	acc_ctl.enable_nodata = fis210x_acc_enable_nodata;
#endif
	acc_ctl.set_delay = fis210x_acc_set_delay;
#if defined(FIS210X_ACC_MTK_8_1)
	acc_ctl.batch = fis210x_acc_batch;
	acc_ctl.flush = fis210x_acc_flush;
#endif
	acc_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	acc_ctl.is_support_batch = obj->hw.is_batch_supported;
#else
	acc_ctl.is_support_batch = false;
#endif
	err = acc_register_control_path(&acc_ctl);
	if(err)
	{
		FIS210X_ACC_ERR("register acc control path err\n");
		goto exit_attr;
	}

	acc_data.get_data = fis210x_acc_get_data;
	acc_data.get_raw_data = fis210x_acc_get_raw_data;
	acc_data.vender_div = 1000;
	err = acc_register_data_path(&acc_data);
	if(err)
	{
		FIS210X_ACC_ERR("register acc data path err\n");
		goto exit_attr;
	}
	
#if 0//def CONFIG_PM
	err = register_pm_notifier(&fis210x_acc_pm_notifier_func);
	if (err) {
		FIS210X_ACC_ERR("Failed to register PM notifier.\n");
		goto exit_kfree;
	}
#endif	/* CONFIG_PM */

	fis210x_acc_init_flag = 0;
	return 0;

	exit_attr:
#if defined(FIS210X_ACC_CREATE_MISC_DEV)
	misc_deregister(&fis210x_acc_misc_device);
#else
	accel_factory_device_deregister(&fis210x_factory_device);
#endif
	fis210x_acc_delete_attr(&(fis210x_acc_init_info.platform_diver_addr->driver));
	exit_kfree:
	kfree(obj);
	exit:
	fis210x_acc_init_flag = -1;

	return err;
}

static int fis210x_acc_i2c_remove(struct i2c_client *client)
{
	fis210x_acc_delete_attr(&(fis210x_acc_init_info.platform_diver_addr->driver));
	i2c_unregister_device(client);

#if defined(FIS210X_ACC_CREATE_MISC_DEV)
	misc_deregister(&fis210x_acc_misc_device);
#else
	accel_factory_device_deregister(&fis210x_factory_device);
#endif
	fis210x_acc_i2c_client = NULL;
	if(fis210x_acc)
	{
		kfree(fis210x_acc);
		fis210x_acc=NULL;
	}
	
	return 0;
}
#endif

static int fis210x_acc_local_init(void)
{
	FIS210X_ACC_FUN();
#if defined(USE_SPI)

	if(spi_register_driver(&fis210x_acc_spi_driver))		
	{
		printk("[acc-fis210x] add spi driver error!\n");
		return -1;
	}	
#else
	if(i2c_add_driver(&fis210x_acc_i2c_driver))
	{
		//FIS210X_ACC_ERR("add driver error\n");
		printk("[acc-fis210x] add i2c driver error!\n");
		return -1;
	}	
#endif
	if(-1 == fis210x_acc_init_flag)
		return -1;
	FIS210X_ACC_LOG("fis210x acc local init.\n");
	return 0;
}

static int fis210x_acc_remove(void)
{
#if defined(USE_SPI)
	spi_unregister_driver(&fis210x_acc_spi_driver);
#else
	i2c_del_driver(&fis210x_acc_i2c_driver);
#endif
	return 0;
}

static int __init fis210x_acc_init(void)
{
	FIS210X_ACC_FUN();
#if defined(FIS210X_ACC_MTK_KK)
	// add by yangzhiqiang for test
	//hwPowerOn(PMIC_APP_CAP_TOUCH_VDD, VOL_3300, "TP");
	//mdelay(10);
	// yangzhiqiang
	i2c_register_board_info(1, &i2c_fis210x, 1);
#endif
#if defined(USE_SPI)
	spi_register_board_info(fis210x_spi_info, ARRAY_SIZE(fis210x_spi_info));
#endif
	acc_driver_add(&fis210x_acc_init_info);
	return 0;
}

static void __exit fis210x_acc_exit(void)
{
	FIS210X_ACC_FUN();
}


module_init(fis210x_acc_init);
module_exit(fis210x_acc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("fis210x_acc I2C driver");
MODULE_AUTHOR("zhiqiang_yang@qstcorp.com");


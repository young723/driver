/* qmaX981 motion sensor driver
 *
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
 
#include <accel.h>
#include <cust_acc.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/sched.h>
#include "qmaX981.h"
#include <linux/of_gpio.h>

#define QMAX981_RETRY_COUNT 	3
#define QMAX981_BUFSIZE			256

#define QMAX981_DEBUG
#ifdef QMAX981_DEBUG
#define QMAX981_TAG                  "[QMAX981] "
#define QMAX981_ERR(fmt, args...)    printk(QMAX981_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#define QMAX981_FUN(f)               printk(QMAX981_TAG "%s\n", __FUNCTION__)
//#define QMAX981_LOG(fmt, args...)    pr_debug(QMAX981_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#define QMAX981_LOG(fmt, args...)    printk(QMAX981_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define QMAX981_ERR(fmt, args...)    do {} while (0)
#define QMAX981_FUN(f)               do {} while (0)
#define QMAX981_LOG(fmt, args...)    do {} while (0)
#endif

static int qmaX981_init_flag = -1;
//static DEFINE_MUTEX(qmaX981_init_mutex);
static DEFINE_MUTEX(qmaX981_mutex);
#ifdef CUSTOM_KERNEL_SENSORHUB
static DEFINE_MUTEX(qmaX981_scp_mutex);
#endif
static DECLARE_WAIT_QUEUE_HEAD(qmcX981_wq1);

#if defined(QMAX981_CHECK_ABNORMAL_DATA)
extern int qmaX981_check_abnormal_data(int data_in, int *data_out);
#endif
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
extern void qmaX981_step_debounce_reset(void);
extern int qmaX981_step_debounce_read_data(int result);
extern int qmaX981_step_debounce_int_work(int data, unsigned char irq_level);
#endif
#if defined(QMAX981_AUTO_CALI)
extern int qmaX981_check_flat_auto_cali(int acc_data[3], int delay);
extern void qmaX981_auto_cali_update(int *cali_data);
extern void qmaX981_auto_cali_reset(void);
#endif

static int qmaX981_initialize(void);
static int qmaX981_local_init(void);
static int qmaX981_local_uninit(void);

static struct acc_init_info qmaX981_init_info = {
	.name = QMAX981_ACC_DEV_NAME,
	.init = qmaX981_local_init,
	.uninit = qmaX981_local_uninit,	
};

#if defined(QMAX981_STEP_COUNTER)
static int qmaX981_step_c_local_init(void);
static int qmaX981_step_c_local_uninit(void);

static struct step_c_init_info qmaX981_step_c_init_info = {
	.name = QMAX981_STEP_C_DEV_NAME,
	.init = qmaX981_step_c_local_init,
	.uninit = qmaX981_step_c_local_uninit,	
};
#endif

typedef enum
{
	CHIP_TYPE_QMA6981 = 0,
	CHIP_TYPE_QMA7981,
	CHIP_TYPE_QMA6100,
	CHIP_TYPE_UNDEFINE,

	CHIP_TYPE_MAX
}qmaX981_type;

struct qmaX981_data{
	struct 	acc_hw 			hw;
	struct 	i2c_client 		*client;
	struct 	hwmsen_convert	cvt;	
	//int						sensitivity;
    atomic_t				layout;
	atomic_t				trace;
	atomic_t				suspend;

	unsigned char			mode;	
#ifdef CUSTOM_KERNEL_SENSORHUB
	int						scp_mode;
#endif
	short                   cali_sw[QMAX981_AXES_NUM+1];
	char                    offset[QMAX981_AXES_NUM+1];  /*+1: for 4-byte alignment*/
	short                   data[QMAX981_AXES_NUM+1];
	//unsigned char           bandwidth;
	unsigned char			chip_id;
	qmaX981_type			chip_type;
	int						lsb_1g;
	int						delay_ms;
// add by yangzhiqiang
	unsigned char			wq1_flag;
// yangzhiqiang
#if defined(QMAX981_INT1_FUNC)
	struct work_struct		irq1_work;
	struct device_node		*irq1_node;
	int						irq1_num;
	unsigned char			irq1_level;
#endif
#if defined(QMAX981_INT2_FUNC)
	struct work_struct		irq2_work;
	struct device_node		*irq2_node;
	int						irq2_num;
	unsigned char			irq2_level;
#endif
};


// yangzhiqiang add to read cic value to check stick, suggest from huifang
struct qmaX981_cic
{
	short	cic_z_00;
	short	cic_z_1f;
	int		static_status;
};
#define QMAX981_CIC_THRESHOLD		6000
// yangzhiqiang add to read cic value to check stick, suggest from huifang


static struct qmaX981_data *qmaX981 = NULL;
static struct i2c_client *qmaX981_i2c_client = NULL;
static struct qmaX981_cic g_qmxx981_cic;
#if defined(QMAX981_AUTO_CALI)
static int	auto_cali_data[3];
#endif
#if defined(GSENSOR_IOCTL_SET_STEPCOUNTER)
static u64 step_algo_num = 0;
#endif

const unsigned char qma6981_init_tbl[][2] = 
{
#if defined(QMAX981_STEP_COUNTER)
	{0x11, 0x80},
	{0x36, 0xb6},
	{QMAX981_DELAY, 5},
	{0x36, 0x00},
	{0x11, 0x80},
	{0x0f, QMAX981_RANGE_8G},
	{0x10, 0x2a},
	{0x12, 0x8f},
	{0x13, 0x10},
	{0x14, 0x14},
	{0x15, 0x10},	
	{0x16, 0x0c},
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	{0x19, 0x08},
#endif
	{0x32, 0x02},
	{0x27, QMA6981_OFFSET},
	{0x28, QMA6981_OFFSET},
	{0x29, QMA6981_OFFSET},
#else
	{0x11, 0x80},
	{0x36, 0xb6},
	{QMAX981_DELAY, 5},
	{0x36, 0x00},
	{0x11, 0x80},
	{0x0f, QMAX981_RANGE_4G},
	{0x10, QMA6981_ODR_125HZ},
#endif
#if defined(QMAX981_FIFO_FUNC)
	{0x0f, QMAX981_RANGE_4G},
	{0x10, QMA6981_ODR_250HZ},
	{0x11, 0x8b},
	{0x3E, 0x40},
	{0x17, 0x20},
	#if defined(QMAX981_FIFO_USE_INT)
	{0x1a, 0x20},	// fifo int map to int1
	#endif
#endif
#if defined(QMAX981_TAP_FUNC)
	{0x0f, QMAX981_RANGE_4G},
	{0x10, QMA6981_ODR_500HZ},
	{0x11, 0x80|QMA6981_SLEEP_DUR6},	// sleep 6ms
	// TAP_QUIET<7>: tap quiet time, 1: 30ms, 0: 20ms 
	// TAP_SHOCK<6>: tap shock time, 1: 50ms, 0: 75ms
	// TAP_DUR<2:0>: the time window of the second tap event for double tap
	{0x2a, 0x80},	
	{0x2b, 0x0f}, //62.5*9=562.5 mg, TAP_TH is 62.5mg in 2g-range, 125mg in 4g-range, 250mg in 8g-range.
	{0x16, 0x20}, //S_TAP_EN enable 
	{0x19, 0x20}, //map single tap interrupt to INT1 pin
#endif

	{QMAX981_DELAY, 1}
};

/*	
qma7981 odr setting
0x10<2:0>		ODR(Hz)					|	RANGE 0x0f<3:0>
000				65						|	0001	2g  		244ug/LSB
001				129						|	0010	4g  		488ug/LSB
002				258						|	0100	8g  		977ug/LSB
003				516						|	1000	16g  	1.95mg/LSB
     				               				|	1111	32g  	3.91mg/LSB
005				32						|	Others	2g  		244ug/LSB
006				16						|
007				8						|
*/

const unsigned char qma7981_init_tbl[][2] = 
{
	{0x36, 0xb6},
	{QMAX981_DELAY, 5},
	{0x36, 0x00},
#if defined(QMAX981_STEP_COUNTER)
	{QMAX981_REG_RANGE, QMAX981_RANGE_4G},	// 0.488 mg
	{QMAX981_REG_BW_ODR, 0xe1}, 			// ODR 129hz
#else
	{QMAX981_REG_RANGE, QMAX981_RANGE_4G},	// 0.488 mg
	{QMAX981_REG_BW_ODR, 0xe1},				// ODR 129hz
#endif
	//{0x4a, 0x08},				//Force I2C I2C interface.SPI is disabled,SENB can be used as ATB
	//{0x20, 0x05},
	{QMAX981_REG_POWER_CTL, 0x80},
	{0x5f, 0x80},		// enable test mode,take control the FSM
	{0x5f, 0x00},		//normal mode

	{QMAX981_DELAY, 1}
};


#ifndef CONFIG_MTK_I2C_EXTENSION
int qmaX981_TxData(char *txData, int length)
{
	int err, loop_i;

	err =0;
	if((!qmaX981_i2c_client)||(!txData)||(length >= C_I2C_FIFO_SIZE)) 
	{
		QMAX981_ERR(" client or length %d exceeds %d\n", length, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	
	mutex_lock(&qmaX981_mutex);
	//qmaX981_i2c_client->addr = qmaX981_i2c_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < QMAX981_RETRY_COUNT; loop_i++)
	{
		err = i2c_master_send(qmaX981_i2c_client, txData, length);
		if (err < 0)
		{
			QMAX981_ERR("try:%d,i2c_master_send error:%d\n",loop_i,  err);
		} 
		else
		{
			break;
		}
		mdelay(10);
	}
	mutex_unlock(&qmaX981_mutex);

	if((err < 0)||(loop_i>=QMAX981_RETRY_COUNT))
	 return -EFAULT;
	else
	 return 0;
}
 
int qmaX981_RxData(char *rxData, int length)
{
	unsigned char addr = rxData[0];
	int err, loop_i;
	struct i2c_msg msgs[2] = 
	{
		{
		.addr = qmaX981_i2c_client->addr,
		.flags = 0,
		.len = 1,	 
		.buf = &addr
		},
		{
		.addr = qmaX981_i2c_client->addr,
		.flags = I2C_M_RD,
		.len = length,  
		.buf = rxData,
		}
	};

	if((!qmaX981_i2c_client)||(!rxData)||(length > C_I2C_FIFO_SIZE))
	{
		QMAX981_ERR(" qmaX981_RxData para error \n");
		return -EINVAL;
	}
	
	mutex_lock(&qmaX981_mutex);

	for(loop_i = 0; loop_i < QMAX981_RETRY_COUNT; loop_i++)
	{
		err = i2c_transfer(qmaX981_i2c_client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
		if (err != 2) {
			QMAX981_ERR("try:%d,i2c_transfer error: (%d %d) %d\n",loop_i, addr, length, err);
			err = -EIO;
		} else {
			err = 0;
			break;
		}
		mdelay(10);
	}
	mutex_unlock(&qmaX981_mutex);

	return err;

}
#else
int qmaX981_RxData(char *rxData, int length)
{	 
	int loop_i;	
	int res = 0;

	if((rxData == NULL) || (length < 1))	 
	{	 
		QMAX981_ERR("qmaX981 qmaX981_RxData error");
		return -EINVAL; 
	}
	
	mutex_lock(&qmaX981_mutex);

	for(loop_i = 0; loop_i < QMAX981_RETRY_COUNT; loop_i++) 
	{		 
		qmaX981_i2c_client->addr = (qmaX981_i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG;		 
		res = i2c_master_send(qmaX981_i2c_client, (const char*)rxData, ((length<<0X08) | 0X01));		 
		if(res > 0)
			break;

		QMAX981_ERR("qmaX981 i2c_read retry %d times\n", loop_i);
		mdelay(10); 
	}		 
	qmaX981_i2c_client->addr = qmaX981_i2c_client->addr & I2C_MASK_FLAG;

	if(loop_i >= QMAX981_RETRY_COUNT)	 
	{		 
		QMAX981_ERR("qmaX981 %s retry over %d\n", __func__, QMAX981_RETRY_COUNT);
		mutex_unlock(&qmaX981_mutex);
		return -EIO;	 
	}
	mutex_unlock(&qmaX981_mutex);

	return 0;
}
 
int qmaX981_TxData(char *txData, int length)
{
	int loop_i;

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		return -EINVAL;
	}

	mutex_lock(&qmaX981_mutex);

	qmaX981_i2c_client->addr = qmaX981_i2c_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < QMAX981_RETRY_COUNT; loop_i++)
	{
		if(i2c_master_send(qmaX981_i2c_client, (const char*)txData, length) > 0)
			break;

		QMAX981_ERR("qmaX981 i2c_read retry %d times\n", loop_i);
		mdelay(10);
	}

	if(loop_i >= QMAX981_RETRY_COUNT)
	{
		QMAX981_ERR( "%s retry over %d\n", __func__, QMAX981_RETRY_COUNT);  
		mutex_unlock(&qmaX981_mutex);
		return -EIO;
	}
	mutex_unlock(&qmaX981_mutex);

	return 0;
}
#endif

static int qmaX981_write_reg(unsigned char reg, unsigned char value)
{
    unsigned char databuf[2];

	databuf[0] = reg;
	databuf[1] = value;
	return qmaX981_TxData(databuf,2);
}

static int qmaX981_set_mode(unsigned char enable)
{
    int res = 0;	
    unsigned char databuf[2];

#if defined(QMAX981_FIX_REG)
	return 0;
#endif
    if (enable == 0){
		databuf[1]=0x00;
    }else {
		databuf[1]=0x80;
	}
    databuf[0] = QMAX981_REG_POWER_CTL;
    res = qmaX981_TxData(databuf,2);
    mdelay(2);

	// for 7981
	if((qmaX981->chip_type == CHIP_TYPE_QMA7981)||(qmaX981->chip_type == CHIP_TYPE_QMA6100))
	{
	    databuf[0] = 0x5f;
	    databuf[1] = 0x80;
		res = qmaX981_TxData(databuf,2);
	    databuf[0] = 0x5f;
	    databuf[1] = 0x00;
		res = qmaX981_TxData(databuf,2);
	}
	// yangzhiqiang

    return res;
}

/*
static int qmaX981_Reset(void)
{
	int res = 0; 
	unsigned char databuf[2];

	databuf[0] = 0x36;
	databuf[1] = 0xb6;
	res = qmaX981_TxData(databuf,2);
	if(res)
	{
		QMAX981_ERR("qmaX981_Reset error!!! res=%d \n", res);
		return res;
	}
	mdelay(5);
	
	databuf[0] = 0x36;
	databuf[1] = 0x00;
	res = qmaX981_TxData(databuf,2);

	return res;	
}
*/

static int qmaX981_get_chip_id(void)
{
	unsigned char databuf[2] = {0};
	int ret;

	//ret = qmaX981_set_mode(1);
	databuf[0] = QMAX981_REG_POWER_CTL;	
#if defined(QMAX981_FIFO_FUNC)
	databuf[1] = 0x8b;
#else
	databuf[1] = 0x80;
#endif
    ret = qmaX981_TxData(databuf,2);
    mdelay(2);
	if(ret)
	{
		QMAX981_ERR("qmaX981_SetPower error!!!");
		return -EFAULT;	
	}
	databuf[0] = QMAX981_CHIP_ID;
	ret=qmaX981_RxData(databuf, 1);
	if(ret){
		QMAX981_ERR("read 0x00 error!!!");
		return -EFAULT;	
	}
	qmaX981->chip_id = databuf[0];
	QMAX981_LOG("chip id = 0x%x \n", qmaX981->chip_id);
	if((qmaX981->chip_id>=0xb0) && (qmaX981->chip_id<=0xb9))
	{
		QMAX981_LOG("qma6981 find \n");
		qmaX981->chip_type = CHIP_TYPE_QMA6981;
	}
	else if((qmaX981->chip_id>=0xe0) && (qmaX981->chip_id<=0xe7))
	{
		QMAX981_LOG("qma7981 find \n");
		qmaX981->chip_type = CHIP_TYPE_QMA7981;
	}	
	else if(qmaX981->chip_id == 0xe8)
	{
		qmaX981->chip_type = CHIP_TYPE_QMA6100;
	}
	else
	{
		QMAX981_LOG("qma acc chip id not defined!!! \n");
		qmaX981->chip_type = CHIP_TYPE_UNDEFINE;
	}

	return ret;
}

static int qmaX981_set_odr(int mdelay)
{
	unsigned char databuf[2] = {0};
	int ret=0;

#if defined(QMAX981_FIX_REG)
	return 0;
#endif
	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		databuf[0] = QMAX981_REG_BW_ODR;

		if(mdelay <= 5)
			databuf[1] = QMA6981_ODR_250HZ;
		else if(mdelay <= 10)
			databuf[1] = QMA6981_ODR_125HZ;
		else if(mdelay <= 20)
			databuf[1] = QMA6981_ODR_62HZ;
		else if(mdelay <= 50)
			databuf[1] = QMA6981_ODR_31HZ;
		else if(mdelay <= 100)
			databuf[1] = QMA6981_ODR_16HZ;
		else
			databuf[1] = QMA6981_ODR_16HZ;

		ret=qmaX981_TxData(databuf, 2);
	}
	else if((qmaX981->chip_type == CHIP_TYPE_QMA7981)||(qmaX981->chip_type == CHIP_TYPE_QMA6100))
	{	
		if(mdelay <= 5)
			databuf[1] = QMA7981_ODR_250HZ;
		else if(mdelay <= 10)
			databuf[1] = QMA7981_ODR_125HZ;
		else if(mdelay <= 20)
			databuf[1] = QMA7981_ODR_62HZ;
		else if(mdelay <= 50)
			databuf[1] = QMA7981_ODR_31HZ;
		else if(mdelay <= 100)
			databuf[1] = QMA7981_ODR_16HZ;
		else
			databuf[1] = QMA7981_ODR_16HZ;

		ret=qmaX981_TxData(databuf, 2);
	}
	else
	{
		QMAX981_LOG("chip type error! \n");
	}
	
	if(ret)
	{
		QMAX981_ERR("qmaX981_set_odr err ret=%d \n", ret);
	}
	return ret;
}


static int qmaX981_set_range(unsigned char range)
{
	unsigned char data[2] = {0};
	int ret=0;

	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		data[0] = QMAX981_REG_RANGE;
		data[1] = range;
		ret=qmaX981_TxData(data, 2);
		if(ret)
		{
			QMAX981_ERR("qma6981 set range error!\n");
			return ret;
		}

		if(data[1] == QMAX981_RANGE_4G)
			qmaX981->lsb_1g = 128;
		else if(data[1] == QMAX981_RANGE_8G)
			qmaX981->lsb_1g = 64;
		else					
			qmaX981->lsb_1g = 256;

	}
	else if((qmaX981->chip_type == CHIP_TYPE_QMA7981)||(qmaX981->chip_type == CHIP_TYPE_QMA6100))
	{	
		data[0] = QMAX981_REG_RANGE;
		data[1] = range;
		ret=qmaX981_TxData(data, 2);
		if(ret)
		{
			QMAX981_ERR("qma7981 set range error!\n");
			return ret;
		}
	
		if(data[1] == QMAX981_RANGE_4G)
			qmaX981->lsb_1g = 2048;
		else if(data[1] == QMAX981_RANGE_8G)
			qmaX981->lsb_1g = 1024;
		else if(data[1] == QMAX981_RANGE_16G)
			qmaX981->lsb_1g = 512;
		else if(data[1] == QMAX981_RANGE_32G)
			qmaX981->lsb_1g = 256;
		else
			qmaX981->lsb_1g = 4096;
	}
	else
	{
		QMAX981_LOG("chip type error! \n");
	}

	if(ret)
	{
		QMAX981_ERR("qmaX981_set_range err ret=%d \n", ret);
	}
	return ret;
}

static int qma6981_read_raw_xyz(int *data)
{
	//int res;	
	unsigned char databuf[6] = {0};		
	unsigned char i;	

	databuf[0] = QMAX981_XOUTL;		
	if(0 != qmaX981_RxData(databuf, 6)){
		QMAX981_ERR("read xyz error!!!");
		return -EFAULT;	
	}	

 	data[0]  = (short)((databuf[1]<<2) |( databuf[0]>>6));
	data[1]  = (short)((databuf[3]<<2) |( databuf[2]>>6));
	data[2]  = (short)((databuf[5]<<2) |( databuf[4]>>6));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data[i] == 0x0200 )	//so we want to calculate actual number here
			data[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data[i] & 0x0200 )	//transfor format
		{					//printk("data 0 step %x \n",data[i]);
			data[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
			data[i] = ~data[i];		//printk("data 2 step %x \n",data[i]);
			data[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
			data[i] = -data[i];		
		}
#if defined(QMAX981_STEP_COUNTER)
		data[i] -= QMA6981_OFFSET;
#endif
	}

	//printk("yzqaccraw	%d	%d	%d\n", data[0], data[1], data[2]);
	return 0;
}

static int qma7981_read_raw_xyz(int *data)
{
	int res;	
	unsigned char databuf[6] = {0}; 	
	//unsigned char i;
	//qma7981_acc_format data_14bit;

	databuf[0] = QMAX981_XOUTL; 	
	if((res = qmaX981_RxData(databuf, 6))){
		QMAX981_ERR("read xyz error!!!");
		return -EFAULT; 
	}	

#if 1
	data[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data[0] = data[0]>>2;
	data[1] = data[1]>>2;
	data[2] = data[2]>>2;
#else
	data[0] = (short)((databuf[1]<<6)|(databuf[0]>>2));
	data[1] = (short)((databuf[3]<<6)|(databuf[2]>>2));
	data[2] = (short)((databuf[5]<<6)|(databuf[4]>>2));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data[i] == 0x2000 )	//so we want to calculate actual number here
			data[i]= -8192; 	//10bit resolution, 512= 2^(10-1)
		else if ( data[i] & 0x2000 )	//transfor format
		{
			data[i] -= 0x1;
			data[i] = ~data[i];
			data[i] &= 0x1fff;
			data[i] = -data[i]; 	
		}
	}
#endif
	return 0;
}


static int qmaX981_read_raw_xyz(int *data)
{
	int ret = 0;

	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
		ret = qma6981_read_raw_xyz(data);
	else if((qmaX981->chip_type == CHIP_TYPE_QMA7981)||(qmaX981->chip_type == CHIP_TYPE_QMA6100))
		ret = qma7981_read_raw_xyz(data);
	else
		ret = -1;

	return ret;
}


static int qmaX981_read_acc_xyz(int *data){
	
	int raw[3]={0};
	int acc[3]={0};
	int ret = 0;

	ret = qmaX981_read_raw_xyz(raw);

	if(0 != ret ){
		QMAX981_ERR("qmaX981_read_acc_xyz error\n");
		return ret;
	}
	
	raw[QMAX981_AXIS_X] += qmaX981->cali_sw[QMAX981_AXIS_X];
	raw[QMAX981_AXIS_Y] += qmaX981->cali_sw[QMAX981_AXIS_Y];
	raw[QMAX981_AXIS_Z] += qmaX981->cali_sw[QMAX981_AXIS_Z];
	
	//remap coordinate
	acc[qmaX981->cvt.map[QMAX981_AXIS_X]] = qmaX981->cvt.sign[QMAX981_AXIS_X]*raw[QMAX981_AXIS_X];
	acc[qmaX981->cvt.map[QMAX981_AXIS_Y]] = qmaX981->cvt.sign[QMAX981_AXIS_Y]*raw[QMAX981_AXIS_Y];
	acc[qmaX981->cvt.map[QMAX981_AXIS_Z]] = qmaX981->cvt.sign[QMAX981_AXIS_Z]*raw[QMAX981_AXIS_Z];

	data[QMAX981_AXIS_X] = (acc[QMAX981_AXIS_X]*GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
	data[QMAX981_AXIS_Y] = (acc[QMAX981_AXIS_Y]*GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
	data[QMAX981_AXIS_Z] = (acc[QMAX981_AXIS_Z]*GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
	//QMAX981_LOG("qmaX981 AFTER x1:%d,y:%d,z:%d\n",data[0],data[1],data[2]);
	
#if defined(QMAX981_AUTO_CALI)
	data[QMAX981_AXIS_X] += auto_cali_data[0];
	data[QMAX981_AXIS_Y] += auto_cali_data[1];
	data[QMAX981_AXIS_Z] += auto_cali_data[2];
	if(qmaX981_check_flat_auto_cali(data, qmaX981->delay_ms))
	{
		qmaX981_auto_cali_update(auto_cali_data);
	}
#endif

	return ret;
}

#if defined(QMAX981_FIFO_FUNC)
static int qmaX981_fifo_data[32][3];

static int qma6981_read_fifo_raw(int *data)
{
	//int res;	
	unsigned char databuf[6] = {0};		
	unsigned char i;	

	databuf[0] = 0x3f;		
	if(0 != qmaX981_RxData(databuf, 6)){
		QMAX981_ERR("read xyz error!!!");
		return -EFAULT;	
	}	

 	data[0]  = (short)((databuf[1]<<2) |( databuf[0]>>6));
	data[1]  = (short)((databuf[3]<<2) |( databuf[2]>>6));
	data[2]  = (short)((databuf[5]<<2) |( databuf[4]>>6));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data[i] == 0x0200 )	//so we want to calculate actual number here
			data[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data[i] & 0x0200 )	//transfor format
		{					//printk("data 0 step %x \n",data[i]);
			data[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
			data[i] = ~data[i];		//printk("data 2 step %x \n",data[i]);
			data[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
			data[i] = -data[i];		
		}
#if defined(QMAX981_STEP_COUNTER)
		data[i] -= QMA6981_OFFSET;
#endif
	}
	//printk("qma6981 fifo raw: %d	%d	%d\n", data[0], data[1], data[2]);	

	return 0;	
}

static int qma7981_read_fifo_raw(int *data)
{
	int res;	
	unsigned char databuf[6] = {0};

	databuf[0] = 0x3f; 	
	if((res = qmaX981_RxData(databuf, 6))){
		QMAX981_ERR("read xyz error!!!");
		return -EFAULT; 
	}	

	data[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data[0] = data[0]>>2;
	data[1] = data[1]>>2;
	data[2] = data[2]>>2;

	//printk("qma7981 fifo raw: %d	%d	%d\n", data[0], data[1], data[2]);	
	return 0;
}

static int qmaX981_read_fifo_acc(int *acc_data)
{
	int ret = 0;
	int raw_data[3];

	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		ret = qma6981_read_fifo_raw(raw_data);
	}
	else if((qmaX981->chip_type == CHIP_TYPE_QMA7981)||(qmaX981->chip_type == CHIP_TYPE_QMA6100))
	{
		ret = qma7981_read_fifo_raw(raw_data);
	}
	else
	{
		ret = -1;
	}
	
	if(0 != ret ){
		QMAX981_ERR("qmaX981_read_fifo_acc error\n");
		return ret;
	}
	
	raw_data[QMAX981_AXIS_X] += qmaX981->cali_sw[QMAX981_AXIS_X];
	raw_data[QMAX981_AXIS_Y] += qmaX981->cali_sw[QMAX981_AXIS_Y];
	raw_data[QMAX981_AXIS_Z] += qmaX981->cali_sw[QMAX981_AXIS_Z];
	
	//remap coordinate
	acc_data[qmaX981->cvt.map[QMAX981_AXIS_X]] = qmaX981->cvt.sign[QMAX981_AXIS_X]*raw_data[QMAX981_AXIS_X];
	acc_data[qmaX981->cvt.map[QMAX981_AXIS_Y]] = qmaX981->cvt.sign[QMAX981_AXIS_Y]*raw_data[QMAX981_AXIS_Y];
	acc_data[qmaX981->cvt.map[QMAX981_AXIS_Z]] = qmaX981->cvt.sign[QMAX981_AXIS_Z]*raw_data[QMAX981_AXIS_Z];
	//QMAX981_LOG("qmaX981 AFTER x1:%d,y:%d,z:%d\n",data[0],data[1],data[2]);

	acc_data[QMAX981_AXIS_X] = (acc_data[QMAX981_AXIS_X]*GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
	acc_data[QMAX981_AXIS_Y] = (acc_data[QMAX981_AXIS_Y]*GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
	acc_data[QMAX981_AXIS_Z] = (acc_data[QMAX981_AXIS_Z]*GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
	
#if defined(QMAX981_AUTO_CALI)
	acc_data[QMAX981_AXIS_X] += auto_cali_data[0];
	acc_data[QMAX981_AXIS_Y] += auto_cali_data[1];
	acc_data[QMAX981_AXIS_Z] += auto_cali_data[2];
	if(qmaX981_check_flat_auto_cali(acc_data, qmaX981->delay_ms))
	{
		qmaX981_auto_cali_update(auto_cali_data);
	}
#endif

	return ret;
}

static int qmaX981_read_fifo(unsigned char is_raw)
{
	int ret = 0;
	unsigned char databuf[2];
	int acc_data[3];
	int icount;
	int fifo_depth = 32;
#if defined(QMAX981_FIFO_USE_INT)
	wait_event_interruptible(qmcX981_wq1, (qmaX981->wq1_flag > 0));
#endif
	if(qmaX981->chip_type == CHIP_TYPE_QMA6100)
		fifo_depth = 64;
	else
		fifo_depth = 32;

// read int status
	databuf[0] = QMAX981_INT_STAT1;
	ret = qmaX981_RxData(databuf, 1);
// read int status
	databuf[0] = QMAX981_FIFO_STATE;
	if((ret = qmaX981_RxData(databuf, 1)))
	{
		ret = qmaX981_RxData(databuf, 1);
		if(ret)
		{
			QMAX981_ERR("read %x error!\n", QMAX981_FIFO_STATE);
			return ret;
		}
	}
	//QMAX981_LOG("fifo frame count=%d  chip_type=%d\n", databuf[0], qmaX981->chip_type);

	if((databuf[0]&0x7f)==fifo_depth)
	{
		for(icount=0; icount<fifo_depth; icount++)
		{
			if(is_raw == 1)
			{
				if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
				{
					ret = qma6981_read_fifo_raw(acc_data);
				}
				else if(qmaX981->chip_type == CHIP_TYPE_QMA6100)
				{
					ret = qma7981_read_fifo_raw(acc_data);
				}
				else
				{
					ret = -1;
				}
			}
			else
			{
				ret = qmaX981_read_fifo_acc(acc_data);
			}
			
			if(ret)
			{
				QMAX981_ERR("read 0x3f error!\n");
				return ret;
			}
			qmaX981_fifo_data[icount][0] = acc_data[0];
			qmaX981_fifo_data[icount][1] = acc_data[1];
			qmaX981_fifo_data[icount][2] = acc_data[2];
			//QMAX981_LOG("fifo_data: %d	%d	%d\n", acc_data[0], acc_data[1], acc_data[2]);
		}
		// read status reg
		databuf[0] = QMAX981_INT_STAT1;
		if((ret = qmaX981_RxData(databuf, 1)))
		{
			ret = qmaX981_RxData(databuf, 1);
			if(ret)
			{
				QMAX981_ERR("fifo read %x error!\n", QMAX981_INT_STAT1);
				return ret;
			}
		}
		// write 0x3e
		databuf[0] = 0x3e;
		databuf[1] = 0x40;
		if((ret = qmaX981_TxData(databuf, 2)))
		{
			ret = qmaX981_RxData(databuf, 2);
			if(ret)
			{
				QMAX981_ERR("fifo write 0x3e error!\n");
				return ret;
			}
		}
	}
	else
	{
		ret = -1;
	}
#if defined(QMAX981_FIFO_USE_INT)
	qmaX981->wq1_flag = 0;
#endif
	return ret;
}
#endif

#if defined(QMAX981_STORE_CALI)
static void qmaX981_write_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDWR|O_CREAT, 0666);
	if (IS_ERR(fp))
	{
		printk("qmaX981_write_file open file error\n");
	}
	else
	{
		printk("qmaX981_write_file data=0x%x len=%d\n", data, len);
		//snprintf();
		fp->f_op->write(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
	}

	set_fs(fs);
}

static void qmaX981_read_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fp))
	{
		printk("qmaX981_read_file open file error\n");
	}
	else
	{
		printk("qmaX981_read_file data=0x%x len=%d\n", data, len);
		fp->f_op->read(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
	}

	set_fs(fs);
}
#endif


#if defined(QMAX981_WRITE_OFFSET_TO_REG)
static int qmaX981_WriteOffset(struct i2c_client *client, s8 x_offset, s8 y_offset, s8 z_offset)
{
	int ret;
	char data[2];

	printk("qmaX981_WriteOffset %d %d %d\n", x_offset, y_offset, z_offset);
	data[0] = 0x27;
	data[1] = x_offset;
	ret = qmaX981_TxData(data,2);

	data[0] = 0x28;
	data[1] = y_offset;
	ret = qmaX981_TxData(data,2);

	data[0] = 0x29;
	data[1] = z_offset;
	ret = qmaX981_TxData(data,2);

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int qmaX981_ResetCalibration(struct i2c_client *client)
{
	struct qmaX981_data *obj = i2c_get_clientdata(client);
	int err = 0;
	//QMAX981_LOG("qmaX981 qmaX981_ResetCalibration\n");
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
#if defined(QMAX981_WRITE_OFFSET_TO_REG)
	qmaX981_WriteOffset(client, obj->cali_sw[0], obj->cali_sw[1], obj->cali_sw[2]);
#endif
	return err;    
}

static int qmaX981_ReadCalibration(struct i2c_client *client, int dat[QMAX981_AXES_NUM])
{
    struct qmaX981_data *obj = i2c_get_clientdata(client);
    int mul;
	
	mul = 0;//only SW Calibration, disable HW Calibration
	
    dat[obj->cvt.map[QMAX981_AXIS_X]] = obj->cvt.sign[QMAX981_AXIS_X]*(obj->offset[QMAX981_AXIS_X]*mul + obj->cali_sw[QMAX981_AXIS_X]);
    dat[obj->cvt.map[QMAX981_AXIS_Y]] = obj->cvt.sign[QMAX981_AXIS_Y]*(obj->offset[QMAX981_AXIS_Y]*mul + obj->cali_sw[QMAX981_AXIS_Y]);
    dat[obj->cvt.map[QMAX981_AXIS_Z]] = obj->cvt.sign[QMAX981_AXIS_Z]*(obj->offset[QMAX981_AXIS_Z]*mul + obj->cali_sw[QMAX981_AXIS_Z]);                    
#if defined(QMAX981_STORE_CALI)
	//qmaX981_read_file("/data/misc/sensor/qmaX981_cali.txt", (char *)(cali_sw), sizeof(cali_sw));
	//printk("qmaX981 cali_sw %d %d %d\n", cali_sw[0],cali_sw[1],cali_sw[2]);
#endif
    return 0;
}

static int qmaX981_ReadCalibrationEx(struct i2c_client *client, int act[QMAX981_AXES_NUM], int raw[QMAX981_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct qmaX981_data *obj = i2c_get_clientdata(client);
	int mul;

	mul = 0;//only SW Calibration, disable HW Calibration
	QMAX981_LOG("qmaX981 qmaX981_ReadCalibrationEx\n");
	raw[QMAX981_AXIS_X] = obj->offset[QMAX981_AXIS_X]*mul + obj->cali_sw[QMAX981_AXIS_X];
	raw[QMAX981_AXIS_Y] = obj->offset[QMAX981_AXIS_Y]*mul + obj->cali_sw[QMAX981_AXIS_Y];
	raw[QMAX981_AXIS_Z] = obj->offset[QMAX981_AXIS_Z]*mul + obj->cali_sw[QMAX981_AXIS_Z];

	act[obj->cvt.map[QMAX981_AXIS_X]] = obj->cvt.sign[QMAX981_AXIS_X]*raw[QMAX981_AXIS_X];
	act[obj->cvt.map[QMAX981_AXIS_Y]] = obj->cvt.sign[QMAX981_AXIS_Y]*raw[QMAX981_AXIS_Y];
	act[obj->cvt.map[QMAX981_AXIS_Z]] = obj->cvt.sign[QMAX981_AXIS_Z]*raw[QMAX981_AXIS_Z];                        
	                       
	return 0;
}

/*----------------------------------------------------------------------------*/
static int qmaX981_WriteCalibration(struct i2c_client *client, int dat[QMAX981_AXES_NUM])
{
	struct qmaX981_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[QMAX981_AXES_NUM], raw[QMAX981_AXES_NUM];

	QMAX981_FUN();
	if(0 != qmaX981_ReadCalibrationEx(client, cali, raw))	/*offset will be updated in obj->offset*/
	{ 
		QMAX981_ERR("read offset fail, %d\n", err);
		return err;
	}

	/*calculate the real offset expected by caller*/
	cali[QMAX981_AXIS_X] += dat[QMAX981_AXIS_X];
	cali[QMAX981_AXIS_Y] += dat[QMAX981_AXIS_Y];
	cali[QMAX981_AXIS_Z] += dat[QMAX981_AXIS_Z];

	obj->cali_sw[QMAX981_AXIS_X] = obj->cvt.sign[QMAX981_AXIS_X]*(cali[obj->cvt.map[QMAX981_AXIS_X]]);
	obj->cali_sw[QMAX981_AXIS_Y] = obj->cvt.sign[QMAX981_AXIS_Y]*(cali[obj->cvt.map[QMAX981_AXIS_Y]]);
	obj->cali_sw[QMAX981_AXIS_Z] = obj->cvt.sign[QMAX981_AXIS_Z]*(cali[obj->cvt.map[QMAX981_AXIS_Z]]);	
#if defined(QMAX981_WRITE_OFFSET_TO_REG)
	qmaX981_WriteOffset(client, obj->cali_sw[0], obj->cali_sw[1], obj->cali_sw[2]);
	//printk("qmaX981 qmaX981_WriteCalibration dat= %d %d %d\n", obj->cali_sw[0],obj->cali_sw[1],obj->cali_sw[2]);
	//qmaX981_write_file("/data/misc/sensor/qmaX981_cali.txt", (char *)(obj->cali_sw), sizeof(obj->cali_sw));
#endif

	return err;
}
/*----------------------------------------------------------------------------*/

// yangzhiqiang add to read cic value to check stick, suggest from huifang
static void qmaX981_set_stick_check_reg(int flag)
{
	unsigned char data[2] = {0};
	unsigned char reg_59, reg_5c, reg_5d;
	//short cic_z_00, cic_z_1f;
	int ret=0;

	g_qmxx981_cic.static_status = 0;
	if(flag)
	{
		data[0] = 0x5f;
		data[1] = 0x80;
		ret = qmaX981_TxData(data,2);
		
		data[0] = 0x32;
		data[1] = 0x03;
		ret = qmaX981_TxData(data,2);

		data[0] = 0x59;
		ret = qmaX981_RxData(data, 1);
		reg_59 = data[0];
		data[0] = 0x59;
		data[1] = reg_59|0x80;
		ret = qmaX981_TxData(data,2);

		// read CIC_Z_00
		data[0] = 0x42;
		data[1] = 0x80;
		ret = qmaX981_TxData(data,2);
		data[0] = 0x5c;
		ret = qmaX981_RxData(data, 1);
		reg_5c = data[0];
		data[0] = 0x5d;
		ret = qmaX981_RxData(data, 1);
		reg_5d = data[0];
		g_qmxx981_cic.cic_z_00 = (short)((reg_5d<<8)|(reg_5c));
		g_qmxx981_cic.cic_z_00 = g_qmxx981_cic.cic_z_00>>2;
		
		// read CIC_Z_1F
		data[0] = 0x42;
		data[1] = 0x9f;
		ret = qmaX981_TxData(data,2);
		data[0] = 0x5c;
		ret = qmaX981_RxData(data, 1);
		reg_5c = data[0];
		data[0] = 0x5d;
		ret = qmaX981_RxData(data, 1);
		reg_5d = data[0];
		g_qmxx981_cic.cic_z_1f = (short)((reg_5d<<8)|(reg_5c));
		g_qmxx981_cic.cic_z_1f = g_qmxx981_cic.cic_z_1f>>2;

		printk("qmaX981 check stick cic_z_00=%d cic_z_1f=%d \n", g_qmxx981_cic.cic_z_00, g_qmxx981_cic.cic_z_1f);
		if(QMAX981_ABS(g_qmxx981_cic.cic_z_00)>QMAX981_CIC_THRESHOLD || QMAX981_ABS(g_qmxx981_cic.cic_z_1f)>QMAX981_CIC_THRESHOLD)
		{
			g_qmxx981_cic.static_status = 1;
		}
		else
		{
			g_qmxx981_cic.static_status = 0;
		}
	}
	else
	{
		printk("qmaX981 check stick end \n");
		data[0] = 0x36;
		data[1] = 0xb6;	//0x2a;	0x0b
		ret = qmaX981_TxData(data,2);	
		mdelay(5);
		data[0] = 0x36;
		data[1] = 0x00; //0x2a; 0x0b
		ret = qmaX981_TxData(data,2);
		data[0] = 0x11;
		data[1] = 0x80; //0x2a; 0x0b
		ret = qmaX981_TxData(data,2);
		qmaX981_initialize();
	}
}
// yangzhiqiang add to read cic value to check stick, suggest from huifang

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	int ret = 0;

	ret = qmaX981_get_chip_id();
	if(ret)
	{
		return sprintf(buf, "%s\n", "read chip id error!");
	}
	else
	{
		if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
			return sprintf(buf, "chipid:%02x qma6981\n", qmaX981->chip_id);
		else if(qmaX981->chip_type == CHIP_TYPE_QMA7981)
			return sprintf(buf, "chipid:%02x qma7981\n", qmaX981->chip_id);
		else if(qmaX981->chip_type == CHIP_TYPE_QMA6100)
			return sprintf(buf, "chipid:%02x qma6100\n", qmaX981->chip_id);
		else
			return sprintf(buf, "chipid:%02x undefined!\n", qmaX981->chip_id);
	}
}


static ssize_t show_waferid_value(struct device_driver *ddri, char *buf)
{
	int res;
	
	unsigned int chipid;
	unsigned char chipidh;
	unsigned char chipidl;
	
	unsigned char waferid;
	unsigned char waferid1;
	unsigned char waferid2;
	unsigned char waferid3;

	
	chipidh = 0x48;
	if((res = qmaX981_RxData(&chipidh, 1)))
	{
		QMAX981_LOG("read wafer chip h error!!!\n");
		return -EFAULT;
	}
	chipidl = 0x47;
	if((res = qmaX981_RxData(&chipidl, 1)))
	{
		QMAX981_LOG("read wafer chip l error!!!\n");
		return -EFAULT;
	}
	QMAX981_LOG("read wafer chip H:0x%x L:0x%x", chipidh, chipidl);
	chipid = (chipidh<<8)|chipidl;
	
	waferid1 = 0x59;
	if((res = qmaX981_RxData(&waferid1, 1)))
	{
		QMAX981_LOG("read wafer id 1 error!!!\n");
		return -EFAULT;
	}
	waferid2 = 0x41;
	if((res = qmaX981_RxData(&waferid2, 1)))
	{
		QMAX981_LOG("read wafer id 2 error!!!\n");
		return -EFAULT;
	}
	waferid3 = 0x40;
	if((res = qmaX981_RxData(&waferid3, 1)))
	{
		QMAX981_LOG("read wafer id 3 error!!!\n");
		return -EFAULT;
	}
	
	QMAX981_LOG("wafer ID: 0x%x 0x%x 0x%x\n", waferid1, waferid2, waferid3);
	
	waferid = (waferid1&0x10)|((waferid2>>4)&0x0c)|((waferid3>>6)&0x03);

	return sprintf(buf, " DieId:0x%x(%d) \n WaferId:0x%02x(%d)\n", chipid,chipid, waferid,waferid);
}



static ssize_t show_dumpallreg_value(struct device_driver *ddri, char *buf)
{
	int res;
	int i =0;
	int write_offset = 0;
	unsigned char databuf[2];

	write_offset = 0;
	for(i =0;i<=0x5f;i++)
	{
		databuf[0] = i;
		res = qmaX981_RxData(databuf, 1);
		if(res)
		{
			QMAX981_LOG("qmaX981 dump registers 0x%02x failed !\n", i);
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "I2C error!\n");
		}
		else
		{			
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "0x%2x=0x%2x\n", i, databuf[0]);
		}

	}

	return write_offset;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	int res = 0;

	if(NULL == qmaX981)
	{
		QMAX981_LOG("qmaX981_data is null!!\n");		
		//res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
		res = sprintf(buf, "error! \n");
	}
	else
	{
		//res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
		res = sprintf(buf, "%d\n", atomic_read(&qmaX981->trace));
	}

	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if(NULL == qmaX981)
	{
		QMAX981_LOG("qmaX981 is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "%d", &trace))
	{
		atomic_set(&qmaX981->trace, trace);
	}
	else
	{
		QMAX981_LOG("invalid content: '%s', length = %d\n", buf, (int)count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct qmaX981_data *obj;
	int err, len = 0;
	int tmp[QMAX981_AXES_NUM];

	if(NULL == qmaX981_i2c_client)
	{
		QMAX981_LOG("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(qmaX981_i2c_client);
	
	if(0 != (err = qmaX981_ReadCalibration(qmaX981_i2c_client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ] (%+3d, %+3d, %+3d)\n",
			obj->offset[QMAX981_AXIS_X], obj->offset[QMAX981_AXIS_Y], obj->offset[QMAX981_AXIS_Z]);
	
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ] (%+3d, %+3d, %+3d)\n",
			obj->cali_sw[QMAX981_AXIS_X], obj->cali_sw[QMAX981_AXIS_Y], obj->cali_sw[QMAX981_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL] (%+3d, %+3d, %+3d)\n", 
			tmp[QMAX981_AXIS_X], tmp[QMAX981_AXIS_Y], tmp[QMAX981_AXIS_Z]);
		
		return len;
    }
}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = qmaX981_i2c_client;  
	int err, x, y, z;
	int dat[QMAX981_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(0 != (err = qmaX981_ResetCalibration(client)))
		{
			QMAX981_LOG("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X,0x%02X,0x%02X", &x, &y, &z))
	{
		dat[QMAX981_AXIS_X] = x;
		dat[QMAX981_AXIS_Y] = y;
		dat[QMAX981_AXIS_Z] = z;
		if(0 != (err = qmaX981_WriteCalibration(client, dat)))
		{
			QMAX981_LOG("write calibration err = %d\n", err);
		}		
	}
	else
	{
		QMAX981_LOG("invalid format\n");
	}
	
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int sensordata[3];
	char strbuf[QMAX981_BUFSIZE];

	qmaX981_read_acc_xyz(sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}



/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = qmaX981_i2c_client;
	struct qmaX981_data *data = i2c_get_clientdata(client);
	int err;
		
	if(0 != (err = hwmsen_get_convert(data->hw.direction, &data->cvt)))
	{
		QMAX981_ERR("invalid direction: %d\n", data->hw.direction);
	}
	
	return scnprintf(buf, PAGE_SIZE, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw.direction,atomic_read(&data->layout),data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = qmaX981_i2c_client;
    struct qmaX981_data *data = i2c_get_clientdata(client);

	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			data->hw.direction = atomic_read(&data->layout);
			QMAX981_LOG("HWMSEN_GET_CONVERT function OK!\r\n");
		}
		else
		{
			QMAX981_ERR("invalid layout: (%d, %d)\n", layout, data->hw.direction);
			//hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		QMAX981_ERR("invalid format = '%s'\n", buf);
	}
	return count;
}


static ssize_t show_cic(struct device_driver *ddri, char *buf)
{
	qmaX981_set_stick_check_reg(1);
	mdelay(5);
	qmaX981_set_stick_check_reg(0);
	return sprintf(buf, "cic_z_00=%d cic_z_1f=%d stick=%d\n", 
		g_qmxx981_cic.cic_z_00, g_qmxx981_cic.cic_z_1f, g_qmxx981_cic.static_status);
}


// add by yangzhiqiang for debug reg use adb
static ssize_t store_setreg(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int addr, value;
	u8 data[C_I2C_FIFO_SIZE];
	int res = 0;
	
	QMAX981_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))
	{
		QMAX981_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);
		data[0] = (u8)addr;
		data[1] = (u8)value;
		if(data[0] == QMAX981_REG_RANGE)
			res = qmaX981_set_range(data[1]);
		else	
			res = qmaX981_TxData(data,2);
		if(res)
		{
			QMAX981_ERR("write reg 0x%02x fail\n", addr);
		}
	}
	else
	{
		QMAX981_ERR("store_reg get para error\n");
	}

	return count;
}

#ifdef QMAX981_STEP_COUNTER
static int qmaX981_reset_sc(void)
{
	unsigned char data[2] = {0};
	unsigned char value_13;
	int ret=0;

	QMAX981_FUN();
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	qmaX981_step_debounce_reset();
#endif
	data[0] = QMAX981_STEP_PRECISION;
	ret = qmaX981_RxData(data, 1);
	if(ret)
	{
		QMAX981_ERR("reset sc error = %d \n", ret);
		return ret;
	}
	value_13 = data[0];
	
	data[0] = QMAX981_STEP_PRECISION;
	data[1] = 0x80;
	ret = qmaX981_TxData(data,2);
	if(ret)
	{
		QMAX981_ERR("reset sc error = %d \n", ret);
		return ret;
	}
	
	data[0] = QMAX981_STEP_PRECISION;
	data[1] = value_13;
	ret = qmaX981_TxData(data,2);
	if(ret)
	{
		QMAX981_ERR("reset sc error = %d \n", ret);
		return ret;
	}
	QMAX981_LOG("reset sc OK \n");

	return ret;
}

static ssize_t show_resetsc(struct device_driver *ddri, char *buf)
{
	int err = 0;

	err = qmaX981_reset_sc();

	return sprintf(buf, "reset step = %d\n", err);
}
#endif


static DRIVER_ATTR(chipinfo,		S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(waferid,		S_IRUGO, show_waferid_value, NULL);
static DRIVER_ATTR(sensordata,	S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(dumpallreg,	S_IRUGO, show_dumpallreg_value, NULL);
static DRIVER_ATTR(cali,			S_IRUGO | S_IWUSR|S_IWGRP, show_cali_value, store_cali_value);
static DRIVER_ATTR(trace,		S_IRUGO | S_IWUSR|S_IWGRP, show_trace_value, store_trace_value);
static DRIVER_ATTR(layout,		S_IRUGO | S_IWUSR|S_IWGRP, show_layout_value, store_layout_value);
static DRIVER_ATTR(cic,			S_IRUGO, show_cic,        NULL);
static DRIVER_ATTR(setreg,		S_IWUSR|S_IWGRP, NULL, store_setreg);
#ifdef QMAX981_STEP_COUNTER
static DRIVER_ATTR(resetsc,		S_IRUGO, show_resetsc, NULL);
#endif


static struct driver_attribute *qmaX981_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_waferid,
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_dumpallreg,
	&driver_attr_layout,
	&driver_attr_trace,
	&driver_attr_cic,
#ifdef QMAX981_STEP_COUNTER
	&driver_attr_resetsc,
#endif
	&driver_attr_setreg
};

static int qmaX981_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(qmaX981_attr_list)/sizeof(qmaX981_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, qmaX981_attr_list[idx]);
	}
	return err;
}

static int qmaX981_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(qmaX981_attr_list)/sizeof(qmaX981_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver, qmaX981_attr_list[idx])))
		{
			QMAX981_LOG("attributes (%s) = %d\n", qmaX981_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}


static int qma6981_initialize(void)
{
	int ret = 0;
	int index, total;
	unsigned char data[2] = {0};

	total = sizeof(qma6981_init_tbl)/sizeof(qma6981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data[0] = qma6981_init_tbl[index][0];
		data[1] = qma6981_init_tbl[index][1];
		if(data[0] == QMAX981_DELAY)
		{
			mdelay(data[1]);
		}
		else
		{
			if(data[0] == QMAX981_REG_RANGE)
			{
				if(data[1] == QMAX981_RANGE_4G)
					qmaX981->lsb_1g = 128;
				else if(data[1] == QMAX981_RANGE_8G)
					qmaX981->lsb_1g = 64;
				else					
					qmaX981->lsb_1g = 256;
			}
			else if(data[0] == 0x32)	// set active axis for step counter
			{
				if(qmaX981->hw.direction%2)
					data[1] = 0x02;
				else
					data[1] = 0x01;
			}

			ret = qmaX981_TxData(data,2);
			if(ret)
			{
				QMAX981_ERR("ret=%d\n", ret);
				return ret;
			}
		}
	}

   	return 0;
}

static int qma7981_initialize(void)
{
	int ret = 0;
	int index, total;
	unsigned char data[2] = {0};
	unsigned char reg_0x10 = 0;
	unsigned char reg_0x16 = 0;
	unsigned char reg_0x18 = 0;
	unsigned char reg_0x19 = 0;
	unsigned char reg_0x1a = 0;
#if defined(QMA7981_ANY_MOTION)||defined(QMA7981_NO_MOTION)
	unsigned char reg_0x2c = 0;
#endif
#if defined(QMA7981_HAND_UP_DOWN)
	//unsigned char reg_0x42 = 0;
#endif

	total = sizeof(qma7981_init_tbl)/sizeof(qma7981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data[0] = qma7981_init_tbl[index][0];
		data[1] = qma7981_init_tbl[index][1];
		if(data[0] == QMAX981_DELAY)
		{
			mdelay(data[1]);
		}
		else
		{
			if(data[0] == QMAX981_REG_RANGE)
			{
				if(data[1] == QMAX981_RANGE_4G)
					qmaX981->lsb_1g = 2048;
				else if(data[1] == QMAX981_RANGE_8G)
					qmaX981->lsb_1g = 1024;
				else if(data[1] == QMAX981_RANGE_16G)
					qmaX981->lsb_1g = 512;
				else if(data[1] == QMAX981_RANGE_32G)
					qmaX981->lsb_1g = 256;
				else
					qmaX981->lsb_1g = 4096;
			}
			else if(data[0] == QMAX981_REG_BW_ODR)
			{
				reg_0x10 = data[1];
			}
			ret = qmaX981_TxData(data,2);
			if(ret)
			{
				QMAX981_ERR("ret=%d\n", ret);
				return ret;
			}
		}
	}

	reg_0x16 = 0x16;
	qmaX981_RxData(&reg_0x16, 1);
	reg_0x18 = 0x18;
	qmaX981_RxData(&reg_0x18, 1);
	reg_0x19 = 0x19;
	qmaX981_RxData(&reg_0x19, 1);
	reg_0x1a = 0x1a;
	qmaX981_RxData(&reg_0x1a, 1);

#if defined(QMAX981_STEP_COUNTER)
	if(reg_0x10 == 0xe0)
	{
		// ODR: 65hz 15.48 ms
		qmaX981_write_reg(0x12, 0x94);
		qmaX981_write_reg(0x13, 0x80);		// clear step
		qmaX981_write_reg(0x13, 0x00);		// 
		qmaX981_write_reg(0x14, 0x12);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_write_reg(0x15, 0x10);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	else if(reg_0x10 == 0xe1)
	{
		// ODR: 130hz 7.74 ms
		qmaX981_write_reg(0x12, 0x94);
		qmaX981_write_reg(0x13, 0x80);		// clear step
		qmaX981_write_reg(0x13, 0x00);		// 
		qmaX981_write_reg(0x14, 0x24);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_write_reg(0x15, 0x20);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	else if(reg_0x10 == 0xe2)
	{
		// ODR: 258Hz 3.87 ms
		qmaX981_write_reg(0x12, 0x94);
		qmaX981_write_reg(0x13, 0x80);		// clear step
		qmaX981_write_reg(0x13, 0x00);		// 
		qmaX981_write_reg(0x14, 0x48);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_write_reg(0x15, 0x40);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	
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
	reg_0x16 |= 0x02;
	reg_0x19 |= 0x02;
			
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
	// hand down
	reg_0x16 |= 0x04;
	reg_0x19 |= 0x04;
	qmaX981_write_reg(0x16, reg_0x16);
	qmaX981_write_reg(0x19, reg_0x19);
	// hand down	
	#if 0	// swap xy
	//read_reg(0x42, &reg_0x42, 1);
	reg_0x42 = 0x42;
	qmaX981_RxData(&reg_0x42, 1);
	reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
	qmaX981_write_reg(0x42, reg_0x42);
	#endif
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

   	return 0;
}

static int qma6100_initialize(void)
{
	qmaX981_write_reg(0x36, 0xb6);
	mdelay(5);
	qmaX981_write_reg(0x36, 0x00);
	qmaX981_set_range(QMAX981_RANGE_4G);
	qmaX981_write_reg(QMAX981_REG_BW_ODR, 0xe0);
#if defined(QMAX981_FIFO_FUNC)
	qmaX981_write_reg(0x31, 0x10);
	qmaX981_write_reg(0x3e, 0x40);		// fifo mode
	qmaX981_write_reg(0x17, 0x20);		// fifo full
	qmaX981_write_reg(0x1a, 0x20);		// map to int1
	qmaX981_write_reg(0x20, 0x05);		// int default low
	qmaX981_write_reg(0x21, 0x01);		// int latch modes
#endif
	qmaX981_write_reg(QMAX981_REG_POWER_CTL, 0x80);
	qmaX981_write_reg(0x5f, 0x80);
	qmaX981_write_reg(0x5f, 0x00);
	mdelay(5);

	return 0;
}

static int qmaX981_initialize(void)
{
	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
		return qma6981_initialize();
	else if(qmaX981->chip_type == CHIP_TYPE_QMA7981)
		return qma7981_initialize();
	else if(qmaX981->chip_type == CHIP_TYPE_QMA6100)
		return qma6100_initialize();
	else
		return -1;
}

#if defined(QMAX981_CREATE_MISC_DEV)
static long qmaX981_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int err = 0;
	void __user *val;
	char strbuf[QMAX981_BUFSIZE];
	int vec[3] = {0};
	struct SENSOR_DATA sensor_data= {0};
	int cali[3];

	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct qmaX981_data *obj = (struct qmaX981_data*)i2c_get_clientdata(client);	

	//QMAX981_LOG("qmaX981_unlocked_ioctl - cmd=%u, arg = %lu\n" , cmd, arg);
	if (&qmaX981->client == NULL) {
		QMAX981_LOG( "I2C driver not install\n");
		return -EFAULT;
	}

	switch (cmd) {
	
	case GSENSOR_IOCTL_INIT:
		qmaX981_initialize();
		break;
	case GSENSOR_IOCTL_READ_SENSORDATA:
		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		
		qmaX981_read_acc_xyz(vec);
		sprintf(strbuf, "%x %x %x", vec[0], vec[1], vec[2]);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}	
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		qmaX981_read_raw_xyz(vec);
		
		sprintf(strbuf, "%x %x %x", vec[0], vec[1], vec[2]);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}
		break;	  

	case GSENSOR_IOCTL_SET_CALI:
		QMAX981_LOG("qmaX981 ioctl QMAX981NSOR_IOCTL_SET_CALI\n");
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		if(copy_from_user(&sensor_data, val, sizeof(sensor_data)))
		{
			err = -EFAULT;
			break;	  
		}
		if(atomic_read(&obj->suspend))
		{
			QMAX981_LOG("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		}
		else
		{
			cali[QMAX981_AXIS_X] = (sensor_data.x * (qmaX981->lsb_1g))/GRAVITY_EARTH_1000;
			cali[QMAX981_AXIS_Y] = (sensor_data.y * (qmaX981->lsb_1g))/GRAVITY_EARTH_1000;
			cali[QMAX981_AXIS_Z] = (sensor_data.z * (qmaX981->lsb_1g))/GRAVITY_EARTH_1000;
			err = qmaX981_WriteCalibration(client, cali);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		err = qmaX981_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		if(0 != (err = qmaX981_ReadCalibration(client, cali)))
		{
			break;
		}
		sensor_data.x = (cali[QMAX981_AXIS_X] * GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
		sensor_data.y = (cali[QMAX981_AXIS_Y] * GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
		sensor_data.z = (cali[QMAX981_AXIS_Z] * GRAVITY_EARTH_1000)/(qmaX981->lsb_1g);
		if(copy_to_user(val, &sensor_data, sizeof(sensor_data)))
		{
			err = -EFAULT;
			break;
		}		
		break;

	case GSENSOR_IOCTL_RESET_SC:
#ifdef QMAX981_STEP_COUNTER
		err = qmaX981_reset_sc();
#endif
		break;

#if defined(GSENSOR_IOCTL_SET_STEPCOUNTER)
	case GSENSOR_IOCTL_SET_STEPCOUNTER:		
		step_algo_num = (void __user*)arg;
		break;
#endif
		
	case GSENSOR_IOCTL_SET_STICK_CHECK_REG:
		qmaX981_set_stick_check_reg((int)arg);
		break;

	case GSENSOR_IOCTL_GET_STICK_STATUS:
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		memset(strbuf, 0, sizeof(strbuf));
		sprintf(strbuf, "%d", g_qmxx981_cic.static_status);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}
		break;
#if defined(QMAX981_FIFO_FUNC)
	case GSENSOR_IOCTL_GET_FIFO_DATA:
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		err = qmaX981_read_fifo(0);
		if(err == 0)
		{		
			if(copy_to_user(val, qmaX981_fifo_data, sizeof(qmaX981_fifo_data)))
			{
				err = -EFAULT;
				break;	  
			}
		}
		break;
	case GSENSOR_IOCTL_GET_FIFO_RAW:
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		err = qmaX981_read_fifo(1);
		if(err == 0)
		{		
			if(copy_to_user(val, qmaX981_fifo_data, sizeof(qmaX981_fifo_data)))
			{
				err = -EFAULT;
				break;	  
			}
		}
		break;
#endif
	default:
		break;
	}
	return err;
}


#ifdef CONFIG_COMPAT
static long qmaX981_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
    	case COMPAT_GSENSOR_IOCTL_INIT:			
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_INIT, (unsigned long)arg32);
			if (err){
				QMAX981_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
				return err;
			}
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        QMAX981_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
        break;
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        QMAX981_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        QMAX981_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        QMAX981_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;		
        default:
            QMAX981_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif

static int qmaX981_open(struct inode *inode, struct file *file)
{
	file->private_data = qmaX981_i2c_client;

	if(file->private_data == NULL)
	{
		QMAX981_LOG("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int qmaX981_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static const struct file_operations qmaX981_fops = {
	.owner = THIS_MODULE,
	.open = qmaX981_open,
	.release = qmaX981_release,
	.unlocked_ioctl = qmaX981_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = qmaX981_compat_ioctl,
#endif
};

static struct miscdevice qmaX981_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &qmaX981_fops,
};

#else	//QMAX981_CREATE_MISC_DEV

/************************* For MTK factory mode ************************************/
static int qmaX981_factory_do_self_test(void)
{
	return 0;
}

static int qmaX981_factory_get_cali(int32_t data[3])
{
	//struct SENSOR_DATA sensor_data;	
	int cali[QMAX981_AXES_NUM];
	int err = -1;

	err = qmaX981_ReadCalibration(qmaX981_i2c_client, cali);
	if (err) {
		QMAX981_ERR("qmaX981_ReadCalibration fail\n");
		return -1;
	}
	data[0] = cali[0];
	data[1] = cali[1];
	data[2] = cali[2];

	return 0;
}

static int qmaX981_factory_set_cali(int32_t data[3])
{
	int err = 0;
	//struct SENSOR_DATA sensor_data;

	//sensor_data.x = data[0];
	//sensor_data.y = data[1];
	//sensor_data.z = data[2];
	err = qmaX981_WriteCalibration(qmaX981_i2c_client, data);
	if (err) {
		QMAX981_ERR("qmaX981_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}

static int qmaX981_factory_enable_calibration(void)
{
  return 0;
}
static int qmaX981_factory_clear_cali(void)
{
  int err = 0;

  err = qmaX981_ResetCalibration(qmaX981_i2c_client);
  if (err) {
    QMAX981_ERR("qmaX981_factory_clear_cali failed!\n");
    return -1;
  }
  return 0;
}

static int qmaX981_factory_get_raw_data(int32_t data[3])
{
	QMAX981_LOG("support qmaX981_i2c_client!\n");
	return qmaX981_read_raw_xyz(data);
}

static int qmaX981_factory_get_data(int32_t data[3], int *status)
{
	int ret = 0;

	ret= qmaX981_read_acc_xyz(data);
	*status = SENSOR_STATUS_ACCURACY_HIGH;	

	return ret;
}

static int qmaX981_enable_nodata(int en);
static int qmaX981_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs);

static int qmaX981_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = qmaX981_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		QMAX981_ERR("%s enable accel sensor failed!\n", __func__);
		return -1;
	}
	err = qmaX981_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		QMAX981_ERR("%s enable accel set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}

static struct accel_factory_fops qmaX981_factory_fops = {
	.enable_sensor = qmaX981_factory_enable_sensor,
	.get_data = qmaX981_factory_get_data,
	.get_raw_data = qmaX981_factory_get_raw_data,
	.enable_calibration = qmaX981_factory_enable_calibration,
	.clear_cali = qmaX981_factory_clear_cali,
	.set_cali = qmaX981_factory_set_cali,
	.get_cali = qmaX981_factory_get_cali,
	.do_self_test = qmaX981_factory_do_self_test,
};

static struct accel_factory_public qmaX981_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &qmaX981_factory_fops,
};

#endif	//QMAX981_CREATE_MISC_DEV

static int qmaX981_open_report_data(int open)
{
	return 0;
}


#ifdef CUSTOM_KERNEL_SENSORHUB
int qmaX981_scp_SetPowerMode(int enable, int sensorType)
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

static int qmaX981_scp_enable_nodata(int en)
{
	qmaX981_scp_SetPowerMode(en, ID_ACCELEROMETER);
}

#else
static int qmaX981_enable_nodata(int en)
{
    int err = 0;

	QMAX981_LOG("qmaX981_enable_nodata en:%d \n", en);
#if defined(QMAX981_STEP_COUNTER)
	return err;
#endif
	if((qmaX981->mode==0) && (en==1))
	{
		QMAX981_LOG("qmaX981_enable_nodata sensor mode on! \n");
		qmaX981->mode = 1;
		err = qmaX981_set_mode(1);
	}
	else if((qmaX981->mode==1) && (en==0))
	{
		QMAX981_LOG("qmaX981_enable_nodata sensor mode off! \n");
		qmaX981->mode = 0;
		err = qmaX981_set_mode(0);
	}
	else
	{
		QMAX981_LOG("qmaX981_enable_nodata return \n");
	}

    if(err != 0)
	{
		QMAX981_LOG("gsensor_enable_nodata fail! err=%d\n", err);
	}
	
	return err;
}
#endif				/* #ifdef CUSTOM_KERNEL_SENSORHUB */


static int qmaX981_set_delay(u64 ns)
{
    int msec;
	int err = 0;

    msec = (int)ns/1000/1000;
	qmaX981->delay_ms = msec;
	err = qmaX981_set_odr(msec);

	return err;
}

static int qmaX981_get_data(int* x ,int* y,int* z, int* status)
{
	int data[3]={0};
	int ret=0;

	ret = qmaX981_read_acc_xyz(data);
 
	*x = data[0];
	*y = data[1];
	*z = data[2]; 
	*status = SENSOR_STATUS_ACCURACY_HIGH;

	return ret;
}


static int qmaX981_get_raw_data(int *x, int *y, int *z)
{
	int data[3]={0};
	int ret=0;

	ret = qmaX981_read_raw_xyz(data);	
	*x = data[0];
	*y = data[1];
	*z = data[2]; 

	return ret;
}

#ifdef ANDROID80_ABOVE
static int qmaX981_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;
	QMAX981_LOG("mpu6515 acc set delay = (%d) ok.\n", value);

	return qmaX981_set_delay(samplingPeriodNs);
}


static int qmaX981_flush(void)
{
	return acc_flush_report();
}
#endif

/*----------------------------------------------------------------------------*/
static void qmaX981_power(struct acc_hw *hw, unsigned int on)
{
#if 0
	static unsigned int power_on = 0;
	//QMAX981_LOG("qmaX981_power id:%d on:%d power_on:%d\n",hw->power_id,on,power_on);
	if(hw->power_id != MT65XX_POWER_NONE)
	{
		QMAX981_LOG("qmaX981_power %s\n", on ? "on" : "off");
		if(power_on == on)
		{
			QMAX981_LOG("qmaX981_ignore power control: %d\n", on);
		}
		else if(on)
		{
			QMAX981_LOG("qmaX981_power_A %s\n", on ? "on" : "off");
			if(!hwPowerOn(hw->power_id, hw->power_vol, "qmaX981"))
			{
				QMAX981_LOG( "qmaX981_power on fails!!\n");
			}
		}
		else
		{	QMAX981_LOG("qmaX981_power_B %s\n", on ? "on" : "off");
			if(!hwPowerDown(hw->power_id, "qmaX981"))
			{
				QMAX981_LOG( "qmaX981_power off fail!!\n");
			}
		}
	}
	power_on = on;
#endif
}


/*----------------------------------------------------------------------------*/
#if defined(CONFIG_PM_SLEEP)&&defined(ANDROID80_ABOVE)
static int qmaX981_suspend(struct device *dev)
{
#if !defined(QMAX981_FIX_REG)
	qmaX981_set_mode(0);
	qmaX981_power(&qmaX981->hw, 0);
#endif
	return 0;
}

static int qmaX981_resume(struct device *dev)
{
#if !defined(QMAX981_FIX_REG)
	int err = 0;

	qmaX981_power(&qmaX981->hw, 1);
	qmaX981_set_mode(1);
	err = qmaX981_initialize();
#endif

	return 0;
}
#else
static int qmaX981_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct qmaX981_data *obj = i2c_get_clientdata(client);
	QMAX981_LOG("qmaX981_suspend");
	
	if(NULL == obj)
	{
		QMAX981_LOG("null pointer!!\n");
		return 0;
	}
	if(msg.event == PM_EVENT_SUSPEND)
	{
		atomic_set(&obj->suspend, 1);
#if !defined(QMAX981_FIX_REG)
		qmaX981_set_mode(0);
		qmaX981_power(&obj->hw, 0);
#endif
	}

	return 0;
}

static int qmaX981_resume(struct i2c_client *client)
{
	struct qmaX981_data *obj = i2c_get_clientdata(client);
	int err = 0;

	QMAX981_LOG("qmaX981_resume");
	
	if(NULL == obj)
	{
		QMAX981_LOG("null pointer!!\n");
		return 0;
	}
#if !defined(QMAX981_FIX_REG)
	qmaX981_power(&obj->hw, 1);
	qmaX981_set_mode(1);
	err = qmaX981_initialize();
#endif
	if(err)
	{
		QMAX981_ERR("failed to init qmaX981\n");
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
#endif

#if defined(QMAX981_INT1_FUNC)
static irqreturn_t qmaX981_irq1_handle(int irq, void *desc)
{
	if(qmaX981)
	{
		schedule_work(&qmaX981->irq1_work);
	}
	
	return IRQ_HANDLED;
}

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
static void qmaX981_irq1_work_step(struct work_struct *work)
{
	int res;
	char databuf[2];
	int result;

	QMAX981_FUN();
	//disable_irq(qmaX981->irq1_num);
	
	databuf[0] = QMAX981_STEP_CNT_L;
	if((res = qmaX981_RxData(databuf, 2))){
		if((res = qmaX981_RxData(databuf, 2)))
		{
			QMAX981_ERR("int read stepcounter error!!!\n");
			enable_irq(qmaX981->irq1_num);
			return;
		}
	}
	result = (databuf[1]<<8)|databuf[0];
	
	if(qmaX981->irq1_level == 0)
	{
		qmaX981->irq1_level = 1;
		qmaX981_step_debounce_int_work(result, qmaX981->irq1_level);
		//res = request_irq(qmaX981->irq1_num, qmaX981_irq1_handle, IRQF_TRIGGER_FALLING, "mediatek, gse_1-eint", NULL);
		res = irq_set_irq_type(qmaX981->irq1_num,IRQF_TRIGGER_FALLING);
	}
	else
	{
		qmaX981->irq1_level = 0;
		qmaX981_step_debounce_int_work(result, qmaX981->irq1_level);
		//res = request_irq(qmaX981->irq1_num, qmaX981_irq1_handle, IRQF_TRIGGER_RISING, "mediatek, gse_1-eint", NULL);
		res = irq_set_irq_type(qmaX981->irq1_num,IRQF_TRIGGER_RISING);
	}

	if(res)
		QMAX981_ERR("int request_irq error!!!\n");		

	enable_irq(qmaX981->irq1_num);
}
#else
static void qmaX981_irq1_work(struct work_struct *work)
{
	QMAX981_FUN();

	QMAX981_LOG("irq1 interrupt!!!");
	//kpd_send_powerkey(0);

	enable_irq(qmaX981->irq1_num);
}
#endif

static int qmaX981_irq1_config(void)
{
	struct qmaX981_data *obj = i2c_get_clientdata(qmaX981_i2c_client);        
	/*
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};
*/
	QMAX981_FUN();

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	INIT_WORK(&qmaX981->irq1_work, qmaX981_irq1_work_step);
#else
	INIT_WORK(&qmaX981->irq1_work, qmaX981_irq1_work);
#endif

	/*
	pinctrl = devm_pinctrl_get(&qma6981_dev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		QMAX981_ERR("Cannot find gsensor pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		QMAX981_ERR("Cannot find gsensor pinctrl default!\n");

	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "qma6981_eint_as_int");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		QMAX981_LOG("Cannot find gsensor pinctrl pin_cfg!\n");

	}
	pinctrl_select_state(pinctrl, pins_cfg);
*/
	obj->irq1_node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");
	if(obj->irq1_node)
	{
	
/*
		of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_request(ints[0], "gsensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		QMAX981_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
*/
/*
		obj->irq1_num = irq_of_parse_and_map(obj->irq1_node, 0);
		QMAX981_LOG("irq1_num = %d\n", obj->irq1_num);
		if (!obj->irq1_num) {
			QMAX981_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
*/
		obj->irq1_num = gpio_to_irq(of_get_named_gpio_flags(obj->irq1_node,"gse_1,gse_1_gpio", 0, NULL));
		QMAX981_LOG("irq1_num = %d\n", obj->irq1_num);
		if(request_irq(obj->irq1_num, qmaX981_irq1_handle, IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, "mediatek, gse_1-eint", NULL)) {
			QMAX981_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq_wake(obj->irq1_num);
		disable_irq(obj->irq1_num);
		enable_irq(obj->irq1_num);
	}
	else
	{
		QMAX981_ERR("null irq node!!\n");
		return -EINVAL;
	}

    return 0;
}
#endif


#if defined(QMAX981_INT2_FUNC)
static irqreturn_t qmaX981_irq2_handle(int irq, void *desc)
{
	if(qmaX981)
	{
		schedule_work(&qmaX981->irq2_work);
	}
	
	return IRQ_HANDLED;
}

static void qmaX981_irq2_work(struct work_struct *work)
{
	QMAX981_FUN();
	enable_irq(qmaX981->irq2_num);
}

static int qmaX981_irq2_config(void)
{
	struct qmaX981_data *obj = i2c_get_clientdata(qmaX981_i2c_client);		  
	/*
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};
*/
	QMAX981_FUN();

	INIT_WORK(&qmaX981->irq2_work, qmaX981_irq2_work);

	/*
	pinctrl = devm_pinctrl_get(&qma6981_dev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		QMAX981_ERR("Cannot find gsensor pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		QMAX981_ERR("Cannot find gsensor pinctrl default!\n");

	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "qma6981_eint_as_int");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		QMAX981_LOG("Cannot find gsensor pinctrl pin_cfg!\n");

	}
	pinctrl_select_state(pinctrl, pins_cfg);
*/
	obj->irq2_node = of_find_compatible_node(NULL, NULL, "mediatek, gse_2-eint");
	if(obj->irq2_node)
	{
/*
		of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_request(ints[0], "gsensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		QMAX981_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
*/
		obj->irq2_num = irq_of_parse_and_map(obj->irq2_node, 0);
		QMAX981_LOG("irq2_num = %d\n", obj->irq2_num);
		if (!obj->irq2_num) {
			QMAX981_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if(request_irq(obj->irq2_num, qmaX981_irq2_handle, IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, "mediatek, gse_2-eint", NULL)) {
			QMAX981_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq_wake(obj->irq2_num);
		disable_irq(obj->irq2_num);
		enable_irq(obj->irq2_num);
	}
	else
	{
		QMAX981_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return 0;
}
#endif

static int qmaX981_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	int err = 0;	
	static struct i2c_client *new_client;
	struct qmaX981_data *obj=NULL;
	struct acc_control_path acc_ctl = {0};
	struct acc_data_path acc_data = {0};
			
	QMAX981_FUN();
	obj = kzalloc(sizeof(struct qmaX981_data), GFP_KERNEL);
	if (obj == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(struct qmaX981_data));
	client->addr = QMAX981_I2C_SLAVE_ADDR;
#if defined(ANDROID80_ABOVE)
	err = get_accel_dts_func(client->dev.of_node, &obj->hw);
#else
	get_accel_dts_func("mediatek,qmaX981", &obj->hw);
#endif
	if (err < 0) {
		QMAX981_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_kfree;
	}	
	qmaX981 = obj;
	qmaX981_power(&obj->hw, 1);
	mdelay(3);
	atomic_set(&obj->layout,obj->hw.direction);
	if(0 != (err = hwmsen_get_convert(obj->hw.direction, &obj->cvt)))
	{
		QMAX981_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit;
	}
	
	obj->client = client;
	new_client = obj->client;	 
	qmaX981_i2c_client = new_client;
	i2c_set_clientdata(new_client, obj);	

	err = qmaX981_get_chip_id();
	if(err)
	{	
		client->addr = QMAX981_I2C_SLAVE_ADDR2;
		err = qmaX981_get_chip_id();
		if(err)
		{
			goto exit_kfree;
		}
	}
	if(0 > (err=qmaX981_initialize()))
	{
		err = -EFAULT;
		goto exit_kfree;
	}

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
#if defined(QMAX981_CREATE_MISC_DEV)
	err = misc_register(&qmaX981_miscdevice);
	if(err)
	{
		QMAX981_ERR("%s: misc register failed\n",__FUNCTION__);
		goto exit_kfree;
	}
#else
	err = accel_factory_device_register(&qmaX981_factory_device);
	if (err)
	{
		QMAX981_ERR("acc_factory register failed!\n");
		goto exit_kfree;
	}
#endif		
	if(0 != (err = qmaX981_create_attr(&qmaX981_init_info.platform_diver_addr->driver)))
	{	
		QMAX981_ERR("%s: create attribute\n",__FUNCTION__);
		goto qmaX981_create_attribute_failed;
	}
	
	acc_ctl.open_report_data= qmaX981_open_report_data;
#ifdef CUSTOM_KERNEL_SENSORHUB
	acc_ctl.enable_nodata = qmaX981_scp_enable_nodata;
#else
	acc_ctl.enable_nodata = qmaX981_enable_nodata;
#endif
	acc_ctl.set_delay = qmaX981_set_delay;
#if defined(ANDROID80_ABOVE)
	acc_ctl.batch = qmaX981_batch;
	acc_ctl.flush = qmaX981_flush;
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
		QMAX981_ERR("register acc control path err\n");
		goto qmaX981_local_init_failed;
	}

	acc_data.get_data = qmaX981_get_data;
	acc_data.get_raw_data = qmaX981_get_raw_data;
	acc_data.vender_div = 1000;
	err = acc_register_data_path(&acc_data);
	if(err)
	{
		QMAX981_ERR("register acc data path err\n");
		goto qmaX981_local_init_failed;
	}

	init_waitqueue_head(&qmcX981_wq1);
	qmaX981->wq1_flag=0;

	QMAX981_LOG("qmaX981 device created successfully\n");
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	qmaX981_step_debounce_reset();
#endif
#if defined(QMAX981_INT1_FUNC)
	qmaX981_irq1_config();
#endif
#if defined(QMAX981_INT2_FUNC)
	qmaX981_irq2_config();
#endif
#if defined(QMAX981_AUTO_CALI)
	qmaX981_auto_cali_reset();
	qmaX981_auto_cali_update(auto_cali_data);
#endif
	qmaX981_init_flag = 0;
	return 0;

qmaX981_local_init_failed:
	qmaX981_delete_attr(&qmaX981_init_info.platform_diver_addr->driver);
qmaX981_create_attribute_failed:
#if defined(QMAX981_CREATE_MISC_DEV)
	misc_deregister(&qmaX981_miscdevice);
#endif
exit_kfree:
	kfree(obj);
exit:
	qmaX981_init_flag = -2;
	return err;
}


static int qmaX981_i2c_remove(struct i2c_client *client)
{
//	int err = 0;
	
    qmaX981_delete_attr(&qmaX981_init_info.platform_diver_addr->driver);
#if defined(QMAX981_CREATE_MISC_DEV)
	misc_deregister(&qmaX981_miscdevice);
#endif
	qmaX981_i2c_client = NULL;
	kfree(i2c_get_clientdata(client));
	qmaX981 = NULL;
	i2c_unregister_device(client);
	return 0;
}

static const struct i2c_device_id qmaX981_id[] = {{QMAX981_ACC_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	//{.compatible = "mediatek,qmaX981"},
	{},
};
#endif
#if defined(CONFIG_PM_SLEEP)&&defined(ANDROID80_ABOVE)
static const struct dev_pm_ops qmaX981_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(qmaX981_suspend, qmaX981_resume)
};
#endif
static struct i2c_driver qmaX981_driver = {
	.probe = qmaX981_i2c_probe,
	.remove = qmaX981_i2c_remove,
	.id_table = qmaX981_id,
#ifndef ANDROID80_ABOVE
	.suspend = qmaX981_suspend,
	.resume = qmaX981_resume,
#endif
	.driver = {
	.owner = THIS_MODULE,
	.name = QMAX981_ACC_DEV_NAME,
#if defined(CONFIG_PM_SLEEP)&&defined(ANDROID80_ABOVE)
	.pm = &qmaX981_pm_ops,
#endif
#ifdef CONFIG_OF
	.of_match_table = gsensor_of_match,
#endif
	},
};

static int qmaX981_local_init(void)
{
	QMAX981_FUN();
	QMAX981_LOG("++++++++\n");

	if(-1 == qmaX981_init_flag)
	{
		if(i2c_add_driver(&qmaX981_driver))
		{
			QMAX981_ERR("add i2c driver error\n");
			return -1;
		}
	}

	if(-2 == qmaX981_init_flag)
	{
		QMAX981_ERR("qmaX981 already init error\n");
		return -1;
	}
	else if(0 == qmaX981_init_flag)
	{
		QMAX981_LOG("qmaX981 already init OK\n");
		return 0;
	}
	QMAX981_LOG("--------\n");

	return 0;
}

static int qmaX981_local_uninit(void)
{
    QMAX981_FUN();
    i2c_del_driver(&qmaX981_driver);
	
    return 0;
}

#ifdef QMAX981_STEP_COUNTER
int qmaX981_read_step(int *data)
{
	int res=0;	
	int result=0;
	char databuf[3];

	databuf[0] = QMAX981_STEP_CNT_L;
	if((res = qmaX981_RxData(databuf, 2))){
		QMAX981_ERR("read stepcounter error!!!\n");
		return -EFAULT;
	}
	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		result = (int)(((unsigned short)databuf[1]<<8)|databuf[0]);
	}
	else if(qmaX981->chip_type == CHIP_TYPE_QMA7981)
	{
		databuf[2] = 0x0e;
		if((res = qmaX981_RxData(&databuf[2], 1))){
			QMAX981_ERR("read stepcounter 0x0e error!!!\n");
			return -EFAULT;
		}
		result = (int)(((int)databuf[2]<<16)|((int)databuf[1]<<8)|databuf[0]);
	}

	*data = result;

	return 0;
}

static int qmaX981_step_c_open_report_data(int open)
{
	return 0;
}

static int qmaX981_step_c_enable_nodata(int en)
{
	if(en)
	{
	}
	else
	{
	}
	return 0;
}

static int qmaX981_step_c_enable_step_detect(int en)
{
	return 0;
}

static int qmaX981_step_c_enable_significant(int en)
{
	return 0;
}

static int qmaX981_step_c_enable_floor_c(int en)
{
	return 0;
}

static int qmaX981_step_c_set_delay(u64 ns)
{
    return 0;
}

static int qmaX981_step_d_set_delay(u64 ns)
{
    return 0;
}

static int qmaX981_floor_c_set_delay(u64 ns)
{
    return 0;
}


static int qmaX981_step_c_get_data_significant(u32 *value, int *status)			//(u64 *value, int *status)
{
	*value = 0;	
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
static int qmaX981_step_c_get_data_step_d(u32 *value, int *status)		//(u64 *value, int *status)
{
	*value = 0;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int qmaX981_step_c_read_stepCounter(int *data)
{
	int res;
//	char databuf[2];
	int result;
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	int de_result;
#endif

	if((res = qmaX981_read_step(&result))){
		QMAX981_ERR("read stepcounter error!!!\n");
		return -EFAULT;
	}
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
	if((res=qmaX981_check_abnormal_data(result, &result)))
	{
		QMAX981_ERR("qmaX981_check_abnormal_data error!!!\n");
		return -EFAULT;
	}
#endif
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	if(qmaX981->chip_type == CHIP_TYPE_QMA6981)
	{
		de_result = qmaX981_step_debounce_read_data(result);
		*data = de_result;
	}
	else
	{
		*data = result;
	}
#else
	*data = (u16)result;
#endif

	return 0;
}


static int qmaX981_step_c_get_sc(u32 *value, int *status)
{
#if defined(GSENSOR_IOCTL_SET_STEPCOUNTER)	
	*value = step_algo_num;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
#else
	int err=0;
	int pedo_data = 0;

	err = qmaX981_step_c_read_stepCounter(&pedo_data);
	if(err == 0)
	{
		*value = (u64)pedo_data;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
#endif
}



static int qmaX981_step_c_local_init(void)
{
	int err = 0;
	struct step_c_control_path step_ctl = {0};
	struct step_c_data_path step_data = {0};
	
	QMAX981_FUN();
	// if i2c not init
	if(qmaX981_init_flag == -1)
	{
		if(i2c_add_driver(&qmaX981_driver))
		{
			QMAX981_ERR("step_c add i2c driver error\n");
			return -1;
		}
	}
	// i2c not init
	if(-2 == qmaX981_init_flag)		// acc already init , error!
	{
		return -1;
	}
	else
	{
		//wake_lock_init(&sc_wakelock,WAKE_LOCK_SUSPEND,"sc wakelock");
#if 0
		if(0 != (err = qmaX981_create_attr(&qmaX981_step_c_init_info.platform_diver_addr->driver)))
		{
			QMAX981_ERR("%s: create attribute\n",__FUNCTION__);
			goto qmaX981_step_c_create_attribute_failed;
		}
#endif	
#if 1		// android O
		step_ctl.open_report_data = qmaX981_step_c_open_report_data;
		step_ctl.enable_nodata = qmaX981_step_c_enable_nodata;
		step_ctl.enable_significant = qmaX981_step_c_enable_significant;
		step_ctl.enable_step_detect = qmaX981_step_c_enable_step_detect;
		step_ctl.enable_floor_c = qmaX981_step_c_enable_floor_c;
		step_ctl.step_c_set_delay = qmaX981_step_c_set_delay;
		step_ctl.step_d_set_delay = qmaX981_step_d_set_delay;
		step_ctl.floor_c_set_delay = qmaX981_floor_c_set_delay;
		step_ctl.is_report_input_direct = false;
		step_ctl.is_counter_support_batch = false;
		step_ctl.is_detector_support_batch = false;
		step_ctl.is_smd_support_batch = false;
		step_ctl.is_floor_c_support_batch = false;
#elif 0		//android N
		step_ctl.open_report_data = qmaX981_step_c_open_report_data;
		step_ctl.enable_nodata = qmaX981_step_c_enable_nodata;
		step_ctl.enable_significant = qmaX981_step_c_enable_significant;
		step_ctl.enable_step_detect = qmaX981_step_c_enable_step_detect;
		step_ctl.step_c_set_delay = qmaX981_step_c_set_delay;
		step_ctl.step_d_set_delay = qmaX981_step_d_set_delay;
		step_ctl.is_report_input_direct = false;		
		step_ctl.is_support_batch = false;
#else		//android M
		step_ctl.open_report_data = qmaX981_step_c_open_report_data;
		step_ctl.enable_nodata=qmaX981_step_c_enable_nodata;
		step_ctl.enable_significant= qmaX981_step_c_enable_significant;
		step_ctl.enable_step_detect = qmaX981_step_c_enable_step_detect;
		step_ctl.set_delay =  qmaX981_step_c_set_delay;
		step_ctl.is_report_input_direct= false;
		step_ctl.is_support_batch = false;
#endif
		err=step_c_register_control_path(&step_ctl);
		if(err)
		{
		 	QMAX981_ERR("register step_counter control path err\n");
			goto qmaX981_step_c_local_init_failed;
		}

		step_data.get_data=qmaX981_step_c_get_sc;
		step_data.get_data_significant = qmaX981_step_c_get_data_significant;
		step_data.get_data_step_d = qmaX981_step_c_get_data_step_d;
		step_data.vender_div = 1;
		step_c_register_data_path(&step_data);

		if(err)
		{
		 	QMAX981_ERR("register step_counter data path err\n");
			goto qmaX981_step_c_local_init_failed;
		}
		
		return 0;
	}

qmaX981_step_c_local_init_failed:
	//qmaX981_delete_attr(&qmaX981_step_c_init_info.platform_diver_addr->driver);
	return err;
}


static int qmaX981_step_c_local_uninit(void)
{
    return 0;
}

#endif


static int __init qmaX981_init(void)
{
	acc_driver_add(&qmaX981_init_info);
#ifdef QMAX981_STEP_COUNTER
	step_c_driver_add(&qmaX981_step_c_init_info);
#endif
    QMAX981_FUN();

	return 0;
}

static void __exit qmaX981_exit(void)
{
	QMAX981_FUN();
}

module_init(qmaX981_init);
module_exit(qmaX981_exit);

MODULE_DESCRIPTION("QST qmaX981 Acc driver");
MODULE_AUTHOR("QST YZQ");
MODULE_LICENSE("GPL");


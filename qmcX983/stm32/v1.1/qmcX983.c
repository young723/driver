/**
  ******************************************************************************
  * @file    qmcX983.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief    qmcX983驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 指南者 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./qmcX983/qmcX983.h"
#include "./usart/bsp_usart.h"
#include "./i2c/bsp_i2c.h"
#define QST_CALI_2D
#if defined(QST_CALI_2D)
#include "./qstlib2d/ICAL2D.h"
#endif

#define QMCX983_LOG		console_write
#define QMCX983_ERR		console_write

struct hwmsen_convert {
	short sign[4];
	short map[4];
};


struct hwmsen_convert qmcX983_map[] = {
	{ { 1, 1, 1}, {0, 1, 2} },
	{ {-1, 1, 1}, {1, 0, 2} },
	{ {-1, -1, 1}, {0, 1, 2} },
	{ { 1, -1, 1}, {1, 0, 2} },

	{ {-1, 1, -1}, {0, 1, 2} },
	{ { 1, 1, -1}, {1, 0, 2} },
	{ { 1, -1, -1}, {0, 1, 2} },
	{ {-1, -1, -1}, {1, 0, 2} }

};


struct qmcX983_data
{
	u8			chip_id;
	u8			layout;
	//kal_int16			lsb_1g;	
	s16			xy_sensitivity;
	s16			z_sensitivity;
	s32			OTP_Kx;
	s32			OTP_Ky;
	
	struct hwmsen_convert *cvt;
};


static struct qmcX983_data g_qmcX983;



static void qmcX983_delay(unsigned int delay)
{
	int i,j;
	for(i=0;i<delay;i++)
	{
		for(j=0;j<1000;j++)
		{
			;
		}
	}
}

/**
  * @brief   写数据到qmcX983寄存器
  * @param   
  * @retval  
  */
u8 qmcX983_WriteReg(u8 reg_add,u8 reg_dat)
{
	I2C_Bus_set_slave_addr(QMCX983_SLAVE_ADDRESS);
	return I2C_ByteWrite(reg_dat,reg_add); 
}

/**
  * @brief   从qmcX983寄存器读取数据
  * @param   
  * @retval  
  */
u8 qmcX983_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
	I2C_Bus_set_slave_addr(QMCX983_SLAVE_ADDRESS);
	return I2C_BufferRead(Read,reg_add,num);
}


/**
  * @brief   读取qmcX983的ID
  * @param   
  * @retval  正常返回1，异常返回0
  */
static u8 qmcX983_device_check(void)
{
	unsigned char databuf[2] = {0};
	u8 ret = 0; 

	ret = qmcX983_ReadData(0x0d, databuf, 1);
	if(ret == 0){
		QMCX983_LOG("%s: read 0x0d failed\n",__func__);
		return ret;
	}

	QMCX983_LOG("qmcX983_device_check 0x0d=%02x \n",databuf[0]);
	if(0xff == databuf[0])
	{
		g_qmcX983.chip_id = QMC6983_A1_D1;
	}
	else if(0x31 == databuf[0])
	{
		g_qmcX983.chip_id = QMC6983_E1;
	}
	else if(0x32 == databuf[0])
	{
		//read otp 0x30
		ret = qmcX983_WriteReg(0x2e, 0x01);
		if(ret == 0)
		{
			QMCX983_LOG("write 0x2e=0x01 failed\n");
			return ret; 		
		}
		
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: read 0x2f failed\n",__func__);
			return ret;
		}		
		if(((databuf[0]&0x04 )>> 2))
		{
			g_qmcX983.chip_id = QMC6983_E1_Metal;
		}else
		{
			//read otp 0x3e
			ret = qmcX983_WriteReg(0x2e, 0x0f);
			if(ret == 0)
			{
				QMCX983_LOG("%s: write 0x2e=0x0f failed\n",__func__);
				return ret; 		
			}
			ret = qmcX983_ReadData(0x2f, databuf, 1);
			if(ret == 0)
			{
				QMCX983_LOG("%s: read 0x2f failed\n",__func__);
				return ret;
			}
			if(0x02 == ((databuf[0]&0x3c)>>2))
			{
				g_qmcX983.chip_id = QMC7983_Vertical;
			}
			if(0x03 == ((databuf[0]&0x3c)>>2))
			{
				g_qmcX983.chip_id = QMC7983_Slope;
			}
		}
	}
	
	QMCX983_LOG("%s: get chip_id OK  %d\n",__func__, g_qmcX983.chip_id);
	return ret;
}

static void qmcX983_start_measure(void)
{
	qmcX983_WriteReg(CTL_REG_ONE, 0x1d);

}

static void qmcX983_stop_measure(void)
{
	qmcX983_WriteReg(CTL_REG_ONE, 0x1c);
}

static void qmcX983_enable(void)
{
	u8 ret = 0;	

	unsigned char data[2];
	int err;

	//data[1] = 0x1;
	//data[0] = 0x21;
	//err = I2C_TxData(data, 2);
	ret = qmcX983_WriteReg(0x21, 0x01);

	//data[1] = 0x40;
	//data[0] = 0x20;
	//err = I2C_TxData(data, 2);
	ret = qmcX983_WriteReg(0x20, 0x40);

  	if(g_qmcX983.chip_id != QMC6983_A1_D1)
	{
		//data[1] = 0x80;
		//data[0] = 0x29;
		//err = I2C_TxData(data, 2); 		
		ret = qmcX983_WriteReg(0x29, 0x80);

		//data[1] = 0x0c;
		//data[0] = 0x0a;
		//err = I2C_TxData(data, 2);				
		ret = qmcX983_WriteReg(0x0a, 0x0c);
	}
	
	if((g_qmcX983.chip_id == QMC6983_E1_Metal) || (g_qmcX983.chip_id == QMC7983_Slope))
	{
		//data[1] = 0x80;
		//data[0] = 0x1b;
		//err = I2C_TxData(data, 2); 			
		ret = qmcX983_WriteReg(0x1b, 0x80);
	}
	
	QMCX983_LOG("start measure!\n");

	ret = qmcX983_WriteReg(0x0b, 0x01);				//the ratio must not be 0, different with qmc5983
	//usleep_range(20000,30000); //fixit for amr ready
	qmcX983_delay(30);
	qmcX983_start_measure();

}


static int qmcx983_get_OPT(void)
{
	unsigned char databuf[2] = {0};
	unsigned char value[2] = {0};
	u8 ret = 0;

	if(g_qmcX983.chip_id == QMC6983_A1_D1)
	{
		g_qmcX983.OTP_Kx = 0;
		g_qmcX983.OTP_Ky = 0;
		return 0;
	}
	else
	{
		//read otp_kx
		//databuf[0] = 0x2e;
		//databuf[1] = 0x0a;
		//ret = I2C_TxData(databuf,2);
		ret = qmcX983_WriteReg(0x2e, 0x0a);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        qmcX983_delay(10);
		//databuf[0] = 0x2f;
		//ret = I2C_RxData(databuf, 1);
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
    
		if(((value[0]&0x3f) >> 5) == 1)
		{
			g_qmcX983.OTP_Kx = (value[0]&0x1f)-32;
		}
		else
		{
			g_qmcX983.OTP_Kx = value[0]&0x1f;	
		}
		//read otp_ky
		//databuf[0] = 0x2e;
		//databuf[1] = 0x0d;
		//ret = I2C_TxData(databuf,2);
		ret = qmcX983_WriteReg(0x2e, 0x0d);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        qmcX983_delay(10);
		//databuf[0] = 0x2f;
		//ret = I2C_RxData(databuf, 1);
		//if(ret < 0)	
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
		qmcX983_delay(10);
		//databuf[0] = 0x2e;
		//databuf[1] = 0x0f;
		//ret = I2C_TxData(databuf,2);
		ret = qmcX983_WriteReg(0x2e, 0x0f);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        qmcX983_delay(10);
		//databuf[0] = 0x2f;
		//ret = I2C_RxData(databuf, 1);
		//if(ret < 0)
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[1] = databuf[0];
		QMCX983_LOG("otp-y value:[0x%x  0x%x] \n",value[0], value[1]);
		if((value[0] >> 7) == 1)
			g_qmcX983.OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6))-32;
		else
			g_qmcX983.OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6));	
	}
	QMCX983_LOG("kx:%d ky:%d \n",g_qmcX983.OTP_Kx, g_qmcX983.OTP_Ky);

	return ret;
}


u8 qmcX983_init(void)
{
	u8 ret;
	int i=0,j=0;
	//在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
	for(i=0;i<1000;i++)
	{
	  for(j=0;j<1000;j++)
	  {
		;
	  }
	}

	qmcX983_delay(100);
	ret = qmcX983_WriteReg(0x09,0x1d);

	ret = qmcX983_device_check();
	if(ret == 0)
	{	
		QMCX983_LOG("read chip id error!!! \n");
		return ret;
	}
	g_qmcX983.layout = 0;
	g_qmcX983.cvt = &qmcX983_map[g_qmcX983.layout];

	qmcX983_enable();
	qmcx983_get_OPT();
	
	return 1;
}


int qmcX983_read_mag_xyz(float *data)
{
	int res;
	u8 ret;
	unsigned char mag_data[6];
	unsigned char databuf[6];

	int hw_d[3] = {0};

	float output[3]={0};
	int t1 = 0;
	unsigned char rdy = 0;
	int i;

#if 0
	while(!(rdy & 0x07) && (t1<3)){
		ret = qmcX983_ReadData(STA_REG_ONE,&rdy,1);
		//QMCX983_LOG("%d: QMCX983 Status register is (%02x)\n",t1, rdy);
		t1 ++;
	}
#endif

	ret = qmcX983_ReadData(OUT_X_L,databuf,6);
	if(ret == 0)
    {
		return -1;
	}
	
	for(i=0;i<6;i++)
		mag_data[i]=databuf[i];

	//QMCX983_LOG("QMCX983 mag_data[%02x, %02x, %02x, %02x, %02x, %02x]\n",
	//	mag_data[0], mag_data[1], mag_data[2],
	//	mag_data[3], mag_data[4], mag_data[5]);

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);

	//QMCX983_LOG("QMCX983 hw_d[%d %d %d]\n",hw_d[0],hw_d[1],hw_d[2]);

#if 1
	output[0] = (float)((float)hw_d[0]/(short)25);		// ut,	//mgs  /2.5
	output[1] = (float)((float)hw_d[1]/(short)25);		// ut
	output[2] = (float)((float)hw_d[2]/(short)25);		// ut
#else
	output[0] = (float)((float)hw_d[0]/2.5);		// mgs  /2.5
	output[1] = (float)((float)hw_d[1]/2.5);		// mgs  
	output[2] = (float)((float)hw_d[2]/2.5);		// mgs  
#endif
	output[2] = output[2] - output[0]*(float)g_qmcX983.OTP_Kx*0.02f - output[1]*(float)g_qmcX983.OTP_Ky*0.02f;
	
	//QMCX983_LOG("QMCX983 output_A[%f %f %f]\n",output[0],output[1],output[2]);

#if 0
	data[0] = g_qmcX983.cvt->sign[QMCX983_AXIS_X]*output[g_qmcX983.cvt->map[QMCX983_AXIS_X]];
	data[1] = g_qmcX983.cvt->sign[QMCX983_AXIS_Y]*output[g_qmcX983.cvt->map[QMCX983_AXIS_Y]];
	data[2] = g_qmcX983.cvt->sign[QMCX983_AXIS_Z]*output[g_qmcX983.cvt->map[QMCX983_AXIS_Z]];
#else
	data[0] = output[0];
	data[1] = output[1];
	data[2] = output[2];
#endif	
	//QMCX983_LOG("QMCX983 output_mG[%f %f %f]\n",data[0],data[1],data[2]);

	//MSE_LOG("QMCX983 data [%d, %d, %d],otp,%d,%d\n", data[0], data[1], data[2],OTP_Kx,OTP_Ky);
	return (int)ret;
}


void qmcX983_get_orientation(float *yaw)
{
	float mag_data[3];

	qmcX983_read_mag_xyz(mag_data);
	mag_data[0] = mag_data[0]*3.125;
	mag_data[1] = mag_data[1]*3.125;
	mag_data[2] = mag_data[2]*3.125;
	process(mag_data);
	mag_data[0] = mag_data[0]/31.25;
	mag_data[1] = mag_data[1]/31.25;
	mag_data[2] = mag_data[2]/31.25;
	*yaw = (atan2(mag_data[1], mag_data[0])+3.1415926535897932f)*180/3.1415926535897932f;
	QMCX983_LOG("QMCX983 calied_uT[%f %f %f] yaw=%f\n",mag_data[0],mag_data[1],mag_data[2],*yaw);
}


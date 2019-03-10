/**
  ******************************************************************************
  * @file    qmp6988.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief    qmp6988驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 指南者 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./qmp6988/qmp6988.h"
#include "./usart/bsp_usart.h"
#include "./i2c/bsp_i2c.h"
#include <math.h>


#define QMP6988_LOG		printf
#define QMP6988_ERR		printf

static float Conv_A_S[10][2] = {
{-6.30E-03,4.30E-04},
{-1.90E-11,1.20E-10},
{1.00E-01,9.10E-02},
{1.20E-08,1.20E-06},
{3.30E-02,1.90E-02},
{2.10E-07,1.40E-07},
{-6.30E-10,3.50E-10},
{2.90E-13,7.60E-13},
{2.10E-15,1.20E-14},
{1.30E-16,7.90E-17},
};

static float a0,b00;
static float a1,a2,bt1,bt2,bp1,b11,bp2,b12,b21,bp3;

typedef struct bit_fields{
	s32 x:20;
	s32 y:12;
}BITFIELDS;

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


struct qmp6988_data
{
	u8			chip_id;
	u8			power_mode;
	double 		temperature;
	double		pressure;
	double		altitude;
	struct qmp6988_calibration_data qmp6988_cali;
};


static struct qmp6988_data g_qmp6988;



static void qmp6988_delay(unsigned int delay)
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
  * @brief   写数据到qmp6988寄存器
  * @param   
  * @retval  
  */
u8 qmp6988_WriteReg(u8 reg_add,u8 reg_dat)
{
	return I2C_ByteWrite(reg_dat,reg_add); 
}

/**
  * @brief   从qmp6988寄存器读取数据
  * @param   
  * @retval  
  */
u8 qmp6988_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
	return I2C_BufferRead(Read,reg_add,num);
}


/**
  * @brief   读取qmp6988的ID
  * @param   
  * @retval  正常返回1，异常返回0
  */
static u8 qmp6988_device_check(void)
{
	unsigned char databuf[2] = {0};
	u8 ret = 0; 

	ret = qmp6988_ReadData(QMP6988_CHIP_ID_REG, databuf, 1);
	if(ret == 0){
		QMP6988_LOG("%s: read 0xD1 failed\n",__func__);
		return ret;
	}
	g_qmp6988.chip_id = databuf[0];
	QMP6988_LOG("qmp6988 read chip id = 0x%x\n", g_qmp6988.chip_id);

	return ret;
}


static int qmp6988_get_calibration_data(void)
{
	int status = 0;
	//BITFIELDS temp_COE;
	u8 a_data_u8r[QMP6988_CALIBRATION_DATA_LENGTH] = {0};
	int len;

#if 1
	status = qmp6988_ReadData(QMP6988_CALIBRATION_DATA_START,a_data_u8r,QMP6988_CALIBRATION_DATA_LENGTH);
	if (status == 0)
	{
		QMP6988_LOG("qmp6988 read 0xA0 error!");
		return status;
	}
#else
	for(len = 0; len < QMP6988_CALIBRATION_DATA_LENGTH; len += 8)
	{
		if((QMP6988_CALIBRATION_DATA_LENGTH-len) >= 8 )
			status = qmp6988_ReadData(QMP6988_CALIBRATION_DATA_START+len,&a_data_u8r[len],8);
		else
			status = qmp6988_ReadData(QMP6988_CALIBRATION_DATA_START+len,&a_data_u8r[len],(QMP6988_CALIBRATION_DATA_LENGTH-len));
		if (status == 0)
		{
			QMP6988_LOG("qmp6988 read 0xA0 error!");
			return status;
		}
	}
#endif

#if 0
	temp_COE.x = (QMP6988_U32_t)((a_data_u8r[18] << \
		SHIFT_LEFT_12_POSITION) | (a_data_u8r[19] << \
		SHIFT_LEFT_4_POSITION) | (a_data_u8r[24] & 0x0f));
	g_qmp6988.qmp6988_cali.COE_a0 = 	temp_COE.x;
#else
	g_qmp6988.qmp6988_cali.COE_a0 = (QMP6988_S32_t)(((a_data_u8r[18] << SHIFT_LEFT_12_POSITION) \
							| (a_data_u8r[19] << SHIFT_LEFT_4_POSITION) \
							| (a_data_u8r[24] & 0x0f))<<12);

	g_qmp6988.qmp6988_cali.COE_a0 = g_qmp6988.qmp6988_cali.COE_a0>>12;
#endif
	
	g_qmp6988.qmp6988_cali.COE_a1 = (QMP6988_S16_t)(((a_data_u8r[20]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[21]);
	g_qmp6988.qmp6988_cali.COE_a2 = (QMP6988_S16_t)(((a_data_u8r[22]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[23]);
	
#if 0
	temp_COE.x = (QMP6988_U32_t)((a_data_u8r[0] << \
		SHIFT_LEFT_12_POSITION) | (a_data_u8r[1] << \
		SHIFT_LEFT_4_POSITION) | ((a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION));
	g_qmp6988.qmp6988_cali.COE_b00 = temp_COE.x;
#else
	g_qmp6988.qmp6988_cali.COE_b00 = (QMP6988_S32_t)(((a_data_u8r[0] << SHIFT_LEFT_12_POSITION) \
							| (a_data_u8r[1] << SHIFT_LEFT_4_POSITION) \
							| ((a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))<<12);
	g_qmp6988.qmp6988_cali.COE_b00 = g_qmp6988.qmp6988_cali.COE_b00>>12;
#endif
	g_qmp6988.qmp6988_cali.COE_bt1 = (QMP6988_S16_t)(((a_data_u8r[2]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[3]);
	g_qmp6988.qmp6988_cali.COE_bt2 = (QMP6988_S16_t)(((a_data_u8r[4]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[5]);
	g_qmp6988.qmp6988_cali.COE_bp1 = (QMP6988_S16_t)(((a_data_u8r[6]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[7]);
	g_qmp6988.qmp6988_cali.COE_b11 = (QMP6988_S16_t)(((a_data_u8r[8]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[9]);
	g_qmp6988.qmp6988_cali.COE_bp2 = (QMP6988_S16_t)(((a_data_u8r[10]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[11]);
	g_qmp6988.qmp6988_cali.COE_b12 = (QMP6988_S16_t)(((a_data_u8r[12]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[13]);		
	g_qmp6988.qmp6988_cali.COE_b21 = (QMP6988_S16_t)(((a_data_u8r[14]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[15]);
	g_qmp6988.qmp6988_cali.COE_bp3 = (QMP6988_S16_t)(((a_data_u8r[16]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[17]);			

	QMP6988_LOG("<-----------calibration data-------------->\n");
	QMP6988_LOG("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
			g_qmp6988.qmp6988_cali.COE_a0,g_qmp6988.qmp6988_cali.COE_a1,g_qmp6988.qmp6988_cali.COE_a2,g_qmp6988.qmp6988_cali.COE_b00);
	QMP6988_LOG("COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\n",
			g_qmp6988.qmp6988_cali.COE_bt1,g_qmp6988.qmp6988_cali.COE_bt2,g_qmp6988.qmp6988_cali.COE_bp1,g_qmp6988.qmp6988_cali.COE_b11);
	QMP6988_LOG("COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\n",
			g_qmp6988.qmp6988_cali.COE_bp2,g_qmp6988.qmp6988_cali.COE_b12,g_qmp6988.qmp6988_cali.COE_b21,g_qmp6988.qmp6988_cali.COE_bp3);
	QMP6988_LOG("<-----------calibration data-------------->\n");

	
	a0 = g_qmp6988.qmp6988_cali.COE_a0 /16.0f;
	b00 = g_qmp6988.qmp6988_cali.COE_b00 /16.0f;

	a1 = Conv_A_S[0][0] + Conv_A_S[0][1] * g_qmp6988.qmp6988_cali.COE_a1 / 32767.0f;
	a2 = Conv_A_S[1][0] + Conv_A_S[1][1] * g_qmp6988.qmp6988_cali.COE_a2 / 32767.0f;
	bt1 = Conv_A_S[2][0] + Conv_A_S[2][1] * g_qmp6988.qmp6988_cali.COE_bt1 / 32767.0f;
	bt2 = Conv_A_S[3][0] + Conv_A_S[3][1] * g_qmp6988.qmp6988_cali.COE_bt2 / 32767.0f;
	bp1 = Conv_A_S[4][0] + Conv_A_S[4][1] * g_qmp6988.qmp6988_cali.COE_bp1 / 32767.0f;
	b11 = Conv_A_S[5][0] + Conv_A_S[5][1] * g_qmp6988.qmp6988_cali.COE_b11 / 32767.0f;
	bp2 = Conv_A_S[6][0] + Conv_A_S[6][1] * g_qmp6988.qmp6988_cali.COE_bp2 / 32767.0f;
	b12 = Conv_A_S[7][0] + Conv_A_S[7][1] * g_qmp6988.qmp6988_cali.COE_b12 / 32767.0f;
	b21 = Conv_A_S[8][0] + Conv_A_S[8][1] * g_qmp6988.qmp6988_cali.COE_b21 / 32767.0f;
	bp3 = Conv_A_S[9][0] + Conv_A_S[9][1] * g_qmp6988.qmp6988_cali.COE_bp3 / 32767.0f;
	
	QMP6988_LOG("<----------- float calibration data -------------->\n");
	QMP6988_LOG("a0[%f]	a1[%f]	a2[%f]	b00[%f]\n",a0,a1,a2,b00);
	QMP6988_LOG("bt1[%f]	bt2[%f]	bp1[%f]	b11[%f]\n",bt1,bt2,bp1,b11);
	QMP6988_LOG("bp2[%f]	b12[%f]	b21[%f]	bp3[%f]\n",bp2,b12,b21,bp3);
	QMP6988_LOG("<----------- float calibration data -------------->\n");

	return 1;
}


static void qmp6988_software_reset(void)
{
	u8 ret = 0; 

	ret = qmp6988_WriteReg(QMP6988_RESET_REG, 0xe6);
	if(ret == 0)
	{
		QMP6988_LOG("qmp6988_software_reset fail!!! \n");
	}
	qmp6988_delay(20);
	ret = qmp6988_WriteReg(QMP6988_RESET_REG, 0x00);
}

static void qmp6988_set_powermode(int power_mode)
{
	u8 data;

	QMP6988_LOG("qmp_set_powermode %d \n", power_mode);
	if (power_mode == g_qmp6988.power_mode)
		return;

	qmp6988_ReadData(QMP6988_CTRLMEAS_REG, &data, 1);
	data = data&0xfc;
	if(power_mode == QMP6988_SLEEP_MODE)
	{
		data |= 0x00;
	}
	else if(power_mode == QMP6988_FORCED_MODE)
	{
		data |= 0x01;
	}
	else if(power_mode == QMP6988_NORMAL_MODE)
	{
		data |= 0x03;
	}
	qmp6988_WriteReg(QMP6988_CTRLMEAS_REG, data);

	QMP6988_LOG("qmp_set_powermode 0xf4=0x%x \n", data);
	
	qmp6988_delay(20);
}


static void qmp6988_set_filter(unsigned char filter)
{	
	u8 data;

	if((filter>=QMP6988_FILTERCOEFF_OFF) &&(filter<=QMP6988_FILTERCOEFF_32))
	{
		data = (filter&0x03);
		qmp6988_WriteReg(QMP6988_CONFIG_REG, data);
	}
	qmp6988_delay(20);
}

static void qmp6988_set_oversampling_p(unsigned char oversampling_p)
{
	u8 data;

	qmp6988_ReadData(QMP6988_CTRLMEAS_REG, &data, 1);
	if((oversampling_p>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_p<=QMP6988_OVERSAMPLING_64X))
	{
		data &= 0xe3;
		data |= (oversampling_p<<2);
		
		qmp6988_WriteReg(QMP6988_CTRLMEAS_REG, data);
	}	
	qmp6988_delay(20);
}

static void qmp6988_set_oversampling_t(unsigned char oversampling_t)
{
	u8 data;

	qmp6988_ReadData(QMP6988_CTRLMEAS_REG, &data, 1);
	if((oversampling_t>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_t<=QMP6988_OVERSAMPLING_64X))
	{
		data &= 0x1f;
		data |= (oversampling_t<<5);
		
		qmp6988_WriteReg(QMP6988_CTRLMEAS_REG, data);
	}	
	qmp6988_delay(20);
}


void qmp6988_calc_pressure(void)
{
	u8 err = 0;
	u8 retry_count = 0;
	QMP6988_U32_t P_read, T_read;
	QMP6988_S32_t P_raw, T_raw;
	u8 a_data_u8r[3] = {0};
	double Tr;

	a_data_u8r[0] = 0x08;
	retry_count = 0;
#if 0
	while(a_data_u8r[0]&0x08)
	{
		err = qmp6988_ReadData(QMP6988_DEVICE_STAT_REG, a_data_u8r, 1);
		if(err == 0)
		{
			QMP6988_LOG("qmp6988 read status reg error! \n");
			return;
		}		
		QMP6988_LOG("qmp6988 read status 0xf3 = 0x%02x \n", a_data_u8r[0]);
		qmp6988_delay(10);
		retry_count++;
		if(retry_count > 5)
			return;
	}
#endif
	// press
	err = qmp6988_ReadData(QMP6988_PRESSURE_MSB_REG, a_data_u8r, 3);
	if(err == 0)
	{
		QMP6988_LOG("qmp6988 read press raw error! \n");
		return;
	}
	P_read = (QMP6988_U32_t)(
	(((QMP6988_U32_t)(a_data_u8r[0])) << SHIFT_LEFT_16_POSITION) |
	(((QMP6988_U16_t)(a_data_u8r[1])) << SHIFT_LEFT_8_POSITION) |
	(a_data_u8r[2]));
	P_raw = (QMP6988_S32_t)(P_read - SUBTRACTOR);
	
	// temp
	err = qmp6988_ReadData(QMP6988_TEMPERATURE_MSB_REG, a_data_u8r, 3);
	if(err == 0)
	{
		QMP6988_LOG("qmp6988 read temp raw error! \n");
	}
	T_read = (QMP6988_U32_t)(
	(((QMP6988_U32_t)(a_data_u8r[0])) << SHIFT_LEFT_16_POSITION) |
	(((QMP6988_U16_t)(a_data_u8r[1])) << SHIFT_LEFT_8_POSITION) | 
	(a_data_u8r[2]));
	T_raw = (QMP6988_S32_t)(T_read - SUBTRACTOR);

	Tr = a0 + a1*T_raw + a2*T_raw*T_raw;
	//Unit centigrade
	g_qmp6988.temperature = Tr / 256.0f;  
	QMP6988_LOG("temperature = %f\n",g_qmp6988.temperature);
	//compensation pressure, Unit Pa
	g_qmp6988.pressure = b00+bt1*Tr+bp1*P_raw+b11*Tr*P_raw+bt2*Tr*Tr+bp2*P_raw*P_raw+b12*P_raw*Tr*Tr+b21*P_raw*P_raw*Tr+bp3*P_raw*P_raw*P_raw;
	QMP6988_LOG("Pressure = %f\n",g_qmp6988.pressure);

	//g_qmp6988.altitude = (pow((101325.00/g_qmp6988.pressure),1/5.257)-1)*(g_qmp6988.temperature+273.15)/0.0065;
	//QMP6988_LOG("altitude = %f\n",g_qmp6988.altitude);
}


void qmp6988_calc_altitude(double pressure)
{
	g_qmp6988.altitude = (pow((101325/pressure),1/5.257)-1)*(g_qmp6988.temperature+273.15)/0.0065;
	QMP6988_LOG("altitude = %f\n",g_qmp6988.pressure);
}

u8 qmp6988_init(void)
{
	u8 ret;
	int i=0,j=0;
	//在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
	I2C_Bus_set_slave_addr(QMP6988_SLAVE_ADDRESS);
	for(i=0;i<1000;i++)
	{
	  for(j=0;j<1000;j++)
	  {
		;
	  }
	}
	qmp6988_delay(100);

	qmp6988_device_check();
	if(g_qmp6988.chip_id != QMP6988_CHIP_ID)
	{
		return 0;
	}
	qmp6988_software_reset();
	qmp6988_get_calibration_data();
	
	//qmp6988_WriteReg(QMP6988_CTRLMEAS_REG, 0x31);
	qmp6988_delay(20);
	//qmp6988_WriteReg(QMP6988_CONFIG_REG, 0x02);
	qmp6988_set_powermode(QMP6988_NORMAL_MODE);
	qmp6988_set_filter(QMP6988_FILTERCOEFF_OFF);
	qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_2X);
	qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);

	//
#if 1
	{
		u8 reg_data;

		qmp6988_ReadData(0xf1,&reg_data,1);
		QMP6988_LOG("qmp6988 read reg: 0xf1 = 0x%02x \n", reg_data);
		qmp6988_ReadData(0xf2,&reg_data,1);
		QMP6988_LOG("qmp6988 read reg: 0xf2 = 0x%02x \n", reg_data);
		qmp6988_ReadData(0xf4,&reg_data,1);
		QMP6988_LOG("qmp6988 read reg: 0xf4 = 0x%02x \n", reg_data);
	}
#endif
	return 1;
}



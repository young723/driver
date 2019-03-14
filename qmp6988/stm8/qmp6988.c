
#include "stm8s.h"
#include "qst_i2c.h"
#include "qst_spi.h"

#if defined(QST_CONFIG_QMP6988)
//#define USE_SPI

#define QMP6988_IIC_ADDR		0x70

#define QMP6988_CHIP_ID_REG						0xD1
#define QMP6988_RESET_REG             			0xE0  /* Device reset register */
#define QMP6988_CONFIG_REG						0xF1
#define QMP6988_DEVICE_STAT_REG             	0xF3  /* Device state register */
#define QMP6988_CTRLMEAS_REG					0xF4  /* Measurement Condition Control Register */
#define QMP6988_IO_SETUP_REG					0xF5  /* IO SETUP Register */

#define QMP6988_CALIBRATION_DATA_START      0xA0 /* QMP6988 compensation coefficients */
#define QMP6988_CALIBRATION_DATA_LENGTH		25

#define QMP6988_SLEEP_MODE                    0x00
#define QMP6988_FORCED_MODE                   0x01
#define QMP6988_NORMAL_MODE                   0x03
#define QMP6988_OVERSAMPLING_SKIPPED          0x00
#define QMP6988_OVERSAMPLING_1X               0x01
#define QMP6988_OVERSAMPLING_2X               0x02
#define QMP6988_OVERSAMPLING_4X               0x03
#define QMP6988_OVERSAMPLING_8X               0x04
#define QMP6988_OVERSAMPLING_16X              0x05
#define QMP6988_OVERSAMPLING_32X              0x06
#define QMP6988_OVERSAMPLING_64X              0x07
#define QMP6988_FILTERCOEFF_OFF               0x00
#define QMP6988_FILTERCOEFF_2                 0x01
#define QMP6988_FILTERCOEFF_4                 0x02
#define QMP6988_FILTERCOEFF_8                 0x03
#define QMP6988_FILTERCOEFF_16                0x04
#define QMP6988_FILTERCOEFF_32                0x05
#define QMP6988_T_STANDBY_1MS				0x00
#define QMP6988_T_STANDBY_5MS				0x01
#define QMP6988_T_STANDBY_50MS				0x02
#define QMP6988_T_STANDBY_250MS				0x03
#define QMP6988_T_STANDBY_500MS				0x04
#define QMP6988_T_STANDBY_1S				0x05
#define QMP6988_T_STANDBY_2S				0x06
#define QMP6988_T_STANDBY_4S				0x07


#define SHIFT_RIGHT_4_POSITION				 4
#define SHIFT_LEFT_2_POSITION                2
#define SHIFT_LEFT_4_POSITION                4
#define SHIFT_LEFT_5_POSITION                5
#define SHIFT_LEFT_8_POSITION                8
#define SHIFT_LEFT_12_POSITION               12
#define SHIFT_LEFT_16_POSITION               16

#define QMP6988_U16_t uint16_t
#define QMP6988_S16_t int16_t
#define QMP6988_U32_t uint32_t
#define QMP6988_S32_t int32_t

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
	unsigned char		power_mode;
	double 				temperature;
	double				pressure;
	//double		altitude;
	struct qmp6988_calibration_data qmp6988_cali;
};


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
static struct qmp6988_data  g_qmp6988;

extern void qst_printf(const char *format, ...);

uint8_t qmp6988_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
#if defined(QMP6988_USE_SPI)
	return qmp6988_spi_write(reg_add, reg_dat);
#else
	return qst_iic_write((QMP6988_IIC_ADDR<<1), reg_add, reg_dat);
#endif
}

/**
  * @brief   从qmp6988寄存器读取数据
  * @param   
  * @retval  
  */
uint8_t qmp6988_ReadData(uint8_t reg_add,uint8_t* Read,uint8_t num)
{
#if defined(QMP6988_USE_SPI)
	return qmp6988_spi_read(reg_add, Read, num);
#else
	return qst_iic_read((QMP6988_IIC_ADDR<<1), reg_add, Read, num);
#endif
}


void qmp6988_set_power_mode(unsigned char power_mode)
{
	unsigned char reg_data;

//	qst_printf("qmp6988 mode:%d\n", power_mode);
	g_qmp6988.power_mode = power_mode;
	qmp6988_ReadData(QMP6988_CTRLMEAS_REG, &reg_data, 1);

	reg_data = reg_data&0xfc;
	reg_data |= power_mode;
	qmp6988_WriteReg(QMP6988_CTRLMEAS_REG, reg_data);
}

void qmp6988_set_t_standby(unsigned char standby)
{
	qmp6988_WriteReg(QMP6988_IO_SETUP_REG, standby<<5);
}

void qmp6988_set_filter(unsigned char filter)
{	
	unsigned char reg_data;

	if((filter>=QMP6988_FILTERCOEFF_OFF) &&(filter<=QMP6988_FILTERCOEFF_32))
	{
		reg_data = (filter&0x07);
		qmp6988_WriteReg(QMP6988_CONFIG_REG, reg_data);
		//g_qmp6988.iir_filter = filter;
	}
}


void qmp6988_set_oversampling_p(unsigned char oversampling_p)
{
	unsigned char reg_data;

	qmp6988_ReadData(QMP6988_CTRLMEAS_REG, &reg_data, 1);
	if((oversampling_p>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_p<=QMP6988_OVERSAMPLING_64X))
	{
		reg_data &= 0xe3;
		reg_data |= (oversampling_p<<2);
		qmp6988_WriteReg(QMP6988_CTRLMEAS_REG, reg_data);
		//g_qmp6988.p_oversampling = oversampling_p;
	}	
}

void qmp6988_set_oversampling_t(unsigned char oversampling_t)
{
	unsigned char reg_data;

	qmp6988_ReadData(QMP6988_CTRLMEAS_REG, &reg_data, 1);

	if((oversampling_t>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_t<=QMP6988_OVERSAMPLING_64X))
	{
		reg_data &= 0x1f;
		reg_data |= (oversampling_t<<5);
		qmp6988_WriteReg(QMP6988_CTRLMEAS_REG, reg_data);
		//g_qmp6988.t_oversampling= oversampling_t;
	}	
}

void qmp6988_set_app(unsigned char app)
{
	qst_printf("qmp6988 app:%d\n", app);
	if(app == 1)	//Weather monitoring
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_2X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_OFF);
	}
	else if(app == 2)		//Drop detection
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_2X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_OFF);
	}
	else if(app == 3)		//Elevator detection 
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_8X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_4);
	}
	else if(app == 4)		//Stair detection
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_16X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_2X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_8);
	}
	else if(app == 5)		//Indoor navigation
	{
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_32X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_4X);
		qmp6988_set_filter(QMP6988_FILTERCOEFF_32);
	}
	else
	{
	}
}

void qmp6988_get_calibration_data(void)
{
	int status = 1;
	//BITFIELDS temp_COE;
	unsigned char a_data_u8r[QMP6988_CALIBRATION_DATA_LENGTH] = {0};
	//int  len=0;

	status = qmp6988_ReadData(QMP6988_CALIBRATION_DATA_START,a_data_u8r,QMP6988_CALIBRATION_DATA_LENGTH);
	if(status == 0)
	{
		return;
	}
#if 0
	len = 0;	
	while(len < QMP6988_CALIBRATION_DATA_LENGTH)
	{
		qmp6988_ReadData(QMP6988_CALIBRATION_DATA_START+len,&a_data_u8r[len],1);
		len++;
	}
#endif

#if 0
	temp_COE.x = (QMP6988_U32_t)((a_data_u8r[18] << \
		SHIFT_LEFT_12_POSITION) | (a_data_u8r[19] << \
		SHIFT_LEFT_4_POSITION) | (a_data_u8r[24] & 0x0f));
	g_qmp6988.qmp6988_cali.COE_a0 = 	temp_COE.x;
#else
	g_qmp6988.qmp6988_cali.COE_a0 = (QMP6988_S32_t)((((QMP6988_S32_t)a_data_u8r[18] << SHIFT_LEFT_12_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[19] << SHIFT_LEFT_4_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[24] & 0x0f))<<12);
/*
	g_qmp6988.qmp6988_cali.COE_a0 = (QMP6988_S32_t)((((long)a_data_u8r[18] << 24) \
							| ((long)a_data_u8r[19] << 16) \
							| ((((long)a_data_u8r[24] & 0x0f))<<12)));
*/
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
	g_qmp6988.qmp6988_cali.COE_b00 = (QMP6988_S32_t)((((QMP6988_S32_t)a_data_u8r[0] << SHIFT_LEFT_12_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[1] << SHIFT_LEFT_4_POSITION) \
							| (((QMP6988_S32_t)a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))<<12);
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

//	QMP6988_LOG("<-----------calibration data-------------->\n");
//	QMP6988_LOG("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
//			g_qmp6988.qmp6988_cali.COE_a0,g_qmp6988.qmp6988_cali.COE_a1,g_qmp6988.qmp6988_cali.COE_a2,g_qmp6988.qmp6988_cali.COE_b00);
//	QMP6988_LOG("COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\n",
//			g_qmp6988.qmp6988_cali.COE_bt1,g_qmp6988.qmp6988_cali.COE_bt2,g_qmp6988.qmp6988_cali.COE_bp1,g_qmp6988.qmp6988_cali.COE_b11);
//	QMP6988_LOG("COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\n",
//			g_qmp6988.qmp6988_cali.COE_bp2,g_qmp6988.qmp6988_cali.COE_b12,g_qmp6988.qmp6988_cali.COE_b21,g_qmp6988.qmp6988_cali.COE_bp3);
//	QMP6988_LOG("<-----------calibration data-------------->\n");

	
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
	
//	QMP6988_LOG("<----------- float calibration data -------------->\n");
//	QMP6988_LOG("a0[%lle]	a1[%lle]	a2[%lle]	b00[%lle]\n",a0,a1,a2,b00);
//	QMP6988_LOG("bt1[%lle]	bt2[%lle]	bp1[%lle]	b11[%lle]\n",bt1,bt2,bp1,b11);
//	QMP6988_LOG("bp2[%lle]	b12[%lle]	b21[%lle]	bp3[%lle]\n",bp2,b12,b21,bp3);
//	QMP6988_LOG("<----------- float calibration data -------------->\n");

}

void qma6988_calc_press(void)
{
	unsigned char ret = 1;
	unsigned char reg_data[6];
	QMP6988_S32_t P_read, T_read;
	double Tr;
#if 0
	if(g_qmp6988.power_mode == QMP6988_NORMAL_MODE)
	{
		qmp6988_ReadData(0xF3, &reg_data[0], 1);
		if(reg_data[0]&0x08)
		{
		  	//qst_printf("qmp6988 measureing!\n");
			return;
		}
	}
#endif
	ret = qmp6988_ReadData(0xF7, reg_data, 6);
	if(ret == 0)
	{
		return;
	}
	P_read = (QMP6988_S32_t)(
		((QMP6988_S32_t)(reg_data[0]) << SHIFT_LEFT_16_POSITION) |
		((QMP6988_S32_t)(reg_data[1]) << SHIFT_LEFT_8_POSITION) |
		((QMP6988_S32_t)reg_data[2]));
	P_read = (QMP6988_S32_t)(P_read - 8388608);

	// temp
	T_read = (QMP6988_S32_t)(
		((QMP6988_S32_t)(reg_data[3]) << SHIFT_LEFT_16_POSITION) |
		((QMP6988_S32_t)(reg_data[4]) << SHIFT_LEFT_8_POSITION) | 
		((QMP6988_S32_t)reg_data[5]));

	//qst_printf("%d	%d	%d	%d	%d	%d \n", reg_data[0],reg_data[1],reg_data[2],reg_data[3],reg_data[4],reg_data[5]);
#if 1	
	T_read = (QMP6988_S32_t)(T_read - 8388608);
	Tr = a0 + a1*T_read + a2*T_read*T_read;
	//Unit centigrade
	g_qmp6988.temperature = Tr / 256.0f;	
	//compensation pressure, Unit Pa
	g_qmp6988.pressure = b00+bt1*Tr+bp1*P_read+b11*Tr*P_read+bt2*Tr*Tr+bp2*P_read*P_read+b12*P_read*Tr*Tr+b21*P_read*P_read*Tr+bp3*P_read*P_read*P_read;
	//g_qmp6988.elevation = ((pow((101.325/pressure), 1/5.257)-1)*(tempearture+273.15))/0.0065;
	qst_printf("pre %f %f\n", g_qmp6988.pressure, g_qmp6988.temperature);
#endif
	if(g_qmp6988.power_mode == QMP6988_FORCED_MODE)
	{	
		qmp6988_set_power_mode(QMP6988_FORCED_MODE);
	}

}


uint8_t qmp6988_init(void)
{
	uint8_t chip;

	qmp6988_ReadData(0xd1, &chip, 1);
	if(chip == 0x5c)
	{
		qmp6988_get_calibration_data();
		qmp6988_set_t_standby(QMP6988_T_STANDBY_1MS);
		// high speed
#if 1
		qmp6988_set_app(1);
#else
		qmp6988_set_filter(QMP6988_FILTERCOEFF_OFF);
		qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_2X);
		qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);
#endif
		qmp6988_set_power_mode(QMP6988_FORCED_MODE);	// QMP6988_NORMAL_MODE,QMP6988_FORCED_MODE
	}
	else
	{
		chip = 0;
	}

	return chip;
}
#endif


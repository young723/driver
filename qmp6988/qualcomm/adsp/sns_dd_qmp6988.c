/*	Date: 2016/2/23 14:00:00
 *	Revision: 1.4.5
 */






/*******************************************************************************
* Copyright (c) 2013, Bosch Sensortec GmbH
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     1. Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*     2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     3. Neither the name of Bosch Sensortec GmbH nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*-----------------------------------------------------------------------------
 * Copyright (c) 2013-2015 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
  -----------------------------------------------------------------------------*/

/*==============================================================================

    S E N S O R S   P R E S S U R E   D R I V E R

DESCRIPTION

   Impelements the driver for the QMP6988 driver


                      EDIT HISTORY FOR FILE


when			who			note
----------   ---     -----------------------------------------------------------
18/07/28		yzq			initial version

==============================================================================*/

#include "sns_dd_qmp6988.h"
#define QMP6988_SNS_DDF_ATTRIB_VERSION   1010100
/*!
 * @brief list all the standby duration
 * that could be set[unit:ms]
*/

static const float Conv_A_S[10][2] = {
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


#define QMP6988_REGV_STANDBY_DUR_4000MS		4000
#define QMP6988_REGV_STANDBY_DUR_2000MS		2000
#define QMP6988_REGV_STANDBY_DUR_1000MS		1000
#define QMP6988_REGV_STANDBY_DUR_500MS		500
#define QMP6988_REGV_STANDBY_DUR_250MS		250
#define QMP6988_REGV_STANDBY_DUR_50MS		50
#define QMP6988_REGV_STANDBY_DUR_5MS		5
#define QMP6988_REGV_STANDBY_DUR_1MS		1

#if 0
DD_ATTR_LOCAL struct bst_val_pair QMP6988_REGV_ODR_MAP[] =
{
 {
	 .l = 0,// 0hz
	 .r = QMP6988_REGV_STANDBY_DUR_4000MS
 },

 {
	 .l = 1,// 1hz
	 .r = QMP6988_REGV_STANDBY_DUR_1000MS
 },

 {
	 .l = 2,//2hz
	 .r = QMP6988_REGV_STANDBY_DUR_500MS
 },

 {
	 .l = 4,//4hz
	 .r = QMP6988_REGV_STANDBY_DUR_250MS
 },

 {
	 .l = 20,//20hz
	 .r = QMP6988_REGV_STANDBY_DUR_50MS
 },

 {
	 .l = 200,//10hz
	 .r = QMP6988_REGV_STANDBY_DUR_5MS
 },

 {
	 .l = 1000, //1000hz
	 .r = QMP6988_REGV_STANDBY_DUR_1MS
 }
};
#endif

sns_ddf_status_e sns_dd_qmp6988_init(
sns_ddf_handle_t*        dd_handle_ptr,
sns_ddf_handle_t         smgr_handle,
sns_ddf_nv_params_s*     nv_params,
sns_ddf_device_access_s  device_info[],
uint32_t                 num_devices,
sns_ddf_memhandler_s*    memhandler,
sns_ddf_sensor_e*        sensors[],
uint32_t*                num_sensors);

sns_ddf_status_e sns_dd_qmp6988_get_data(
sns_ddf_handle_t        dd_handle,
sns_ddf_sensor_e        sensors[],
uint32_t                num_sensors,
sns_ddf_memhandler_s*   memhandler,
sns_ddf_sensor_data_s*  data[]);

sns_ddf_status_e sns_dd_qmp6988_set_attr(
sns_ddf_handle_t     dd_handle,
sns_ddf_sensor_e     sensor,
sns_ddf_attribute_e  attrib,
void*                value);

sns_ddf_status_e sns_dd_qmp6988_get_attr(
sns_ddf_handle_t       dd_handle,
sns_ddf_sensor_e       sensor,
sns_ddf_attribute_e    attrib,
sns_ddf_memhandler_s*  memhandler,
void**                 value,
uint32_t*              num_elems);

sns_ddf_status_e sns_dd_qmp6988_reset(sns_ddf_handle_t dd_handle);
sns_ddf_status_e sns_dd_qmp6988_self_test(
            sns_ddf_handle_t dd_handle,
            sns_ddf_sensor_e sensor,
            sns_ddf_test_e test,
            uint32_t* err);

#if QMP6988_CONFIG_RUN_ON_OSSC
extern sns_ddf_driver_if_s sns_dd_vendor_if_1;
#else
extern sns_ddf_driver_if_s sns_qmp6988_driver_fn_list;
#endif
extern sns_ddf_handle_t qmp6988_port_handle;


typedef struct
{
    uint32_t conversion_time_usec;
    uint32_t avg_current_uA;
    uint32_t bits;
    uint32_t num_of_samples;
    uint32_t rms_nois_pa;
} qmp6988_mode_param_t;



static sns_ddf_sensor_e qmp6988_sensors[] =
{
    SNS_DDF_SENSOR_PRESSURE,
    SNS_DDF_SENSOR_TEMP
};


/*!
 * @brief Mode Prarameters.
 */

static const qmp6988_mode_param_t qmp6988_mode_param[QMP6988_NUM_MODES] =
{
//   usec , uA ,  bits ,  oversampling,    noise
  {  5500 ,  3 ,  16   , 1 ,  FX_FLTTOFIX_Q16(0.9)}, // ULTRA_LOW_POWER
  {  7500 ,  4 ,  17   , 2 ,  FX_FLTTOFIX_Q16(0.6)},	//LOW_POWER
  {  11500 , 7 ,  18   , 4 ,  FX_FLTTOFIX_Q16(0.5)}, // STANDARD
  {  19500 , 12 , 19   , 8 ,  FX_FLTTOFIX_Q16(0.4)}, // HIGH_RESOLUTION
  {  37500 , 25 , 20   , 16 , FX_FLTTOFIX_Q16(0.4)}, // ULTRA_HIGH_RESOLUTION
};

sns_ddf_handle_t qmp6988_port_handle;
sns_dd_qmp6988_state_t *qmp6988_data=NULL;
struct qmp6988_calibration_data qmp6988_cali;
static float a0,b00;
static float a1,a2,bt1,bt2,bp1,b11,bp2,b12,b21,bp3;

sns_ddf_status_e qmp6988_read_reg(uint8_t reg_addr, uint8_t *read_data, uint8_t len)
{

    sns_ddf_status_e stat;
    uint8_t out;

    if ( (stat = sns_ddf_read_port(qmp6988_port_handle,
                          reg_addr,
                          read_data,
                          len,
                          &out)) != SNS_DDF_SUCCESS)
    {
		QMP6988_MSG_1(HIGH, "qmp6988_read_reg 0x%x fail!!! \n", reg_addr);
        return stat;
    }
    return stat;
}

sns_ddf_status_e qmp6988_write_reg(uint8_t reg_addr, uint8_t *write_buffer, uint8_t len)
{

    sns_ddf_status_e stat;
    uint8_t out;

    if ( (stat = sns_ddf_write_port(qmp6988_port_handle,
                         reg_addr,
                         write_buffer,
                         len,
                         &out)) != SNS_DDF_SUCCESS)
    {    
		QMP6988_MSG_2(HIGH, "qmp6988_read_reg 0x%x value:0x%x fail!!! \n", reg_addr, write_buffer[0]);
        return stat;
    }
    return stat;
}

void qmp6988_delay(uint32_t msec)
{
    sns_ddf_delay(1000*msec);
}

sns_ddf_status_e qmp6988_get_calib_param()
{
    sns_ddf_status_e comres = SNS_DDF_SUCCESS;
    unsigned char a_data_u8r[QMP6988_CALIBRATION_DATA_LENGTH] = {0};

	comres = qmp6988_read_reg(QMP6988_CALIBRATION_DATA_START, a_data_u8r, QMP6988_CALIBRATION_DATA_LENGTH);
	if(comres!=SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_get_calib_param qmp6988_read_reg fail\n");
	}

	qmp6988_cali.COE_a0 = (QMP6988_S32_t)(((QMP6988_S32_t)a_data_u8r[18]<<SHIFT_LEFT_12_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[19]<<SHIFT_LEFT_4_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[24]&0x0f))<<12;
	qmp6988_cali.COE_a0 = qmp6988_cali.COE_a0>>12;

	qmp6988_cali.COE_a1 = (QMP6988_S16_t)(((a_data_u8r[20]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[21]);
	qmp6988_cali.COE_a2 = (QMP6988_S16_t)(((a_data_u8r[22]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[23]);
		
	qmp6988_cali.COE_b00 = (QMP6988_S32_t)(((QMP6988_S32_t)a_data_u8r[0]<<SHIFT_LEFT_12_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[1]<<SHIFT_LEFT_4_POSITION) \
							| (((QMP6988_S32_t)a_data_u8r[24]&0xf0)>>SHIFT_RIGHT_4_POSITION))<<12;;
	qmp6988_cali.COE_b00 = qmp6988_cali.COE_b00>>12;

	qmp6988_cali.COE_bt1 = (QMP6988_S16_t)(((a_data_u8r[2]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[3]);
	qmp6988_cali.COE_bt2 = (QMP6988_S16_t)(((a_data_u8r[4]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[5]);
	qmp6988_cali.COE_bp1 = (QMP6988_S16_t)(((a_data_u8r[6]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[7]);
	qmp6988_cali.COE_b11 = (QMP6988_S16_t)(((a_data_u8r[8]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[9]);
	qmp6988_cali.COE_bp2 = (QMP6988_S16_t)(((a_data_u8r[10]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[11]);
	qmp6988_cali.COE_b12 = (QMP6988_S16_t)(((a_data_u8r[12]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[13]); 	
	qmp6988_cali.COE_b21 = (QMP6988_S16_t)(((a_data_u8r[14]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[15]);
	qmp6988_cali.COE_bp3 = (QMP6988_S16_t)(((a_data_u8r[16]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[17]); 		
#if 0	
	QMP6988_MSG_0(HIGH, "<-----------calibration data-------------->\n");
	QMP6988_MSG_4(HIGH, "COE_a0[%d] COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
			qmp6988_cali.COE_a0,qmp6988_cali.COE_a1,qmp6988_cali.COE_a2,qmp6988_cali.COE_b00);
	QMP6988_MSG_4(HIGH, "COE_bt1[%d]	COE_bt2[%d] COE_bp1[%d] COE_b11[%d]\n",
			qmp6988_cali.COE_bt1,qmp6988_cali.COE_bt2,qmp6988_cali.COE_bp1,qmp6988_cali.COE_b11);
	QMP6988_MSG_4(HIGH, "COE_bp2[%d]	COE_b12[%d] COE_b21[%d] COE_bp3[%d]\n",
			qmp6988_cali.COE_bp2,qmp6988_cali.COE_b12,qmp6988_cali.COE_b21,qmp6988_cali.COE_bp3);
	QMP6988_MSG_0(HIGH, "<-----------calibration data-------------->\n");
#endif		
	a0 = qmp6988_cali.COE_a0 /16.0f;
	b00 = qmp6988_cali.COE_b00 /16.0f;

	a1 = Conv_A_S[0][0] + Conv_A_S[0][1] * qmp6988_cali.COE_a1 / 32767.0f;
	a2 = Conv_A_S[1][0] + Conv_A_S[1][1] * qmp6988_cali.COE_a2 / 32767.0f;
	bt1 = Conv_A_S[2][0] + Conv_A_S[2][1] * qmp6988_cali.COE_bt1 / 32767.0f;
	bt2 = Conv_A_S[3][0] + Conv_A_S[3][1] * qmp6988_cali.COE_bt2 / 32767.0f;
	bp1 = Conv_A_S[4][0] + Conv_A_S[4][1] * qmp6988_cali.COE_bp1 / 32767.0f;
	b11 = Conv_A_S[5][0] + Conv_A_S[5][1] * qmp6988_cali.COE_b11 / 32767.0f;
	bp2 = Conv_A_S[6][0] + Conv_A_S[6][1] * qmp6988_cali.COE_bp2 / 32767.0f;
	b12 = Conv_A_S[7][0] + Conv_A_S[7][1] * qmp6988_cali.COE_b12 / 32767.0f;
	b21 = Conv_A_S[8][0] + Conv_A_S[8][1] * qmp6988_cali.COE_b21 / 32767.0f;
	bp3 = Conv_A_S[9][0] + Conv_A_S[9][1] * qmp6988_cali.COE_bp3 / 32767.0f;
#if 1		
	QMP6988_MSG_0(HIGH, "<----------- float calibration data -------------->\n");
	QMP6988_MSG_4(HIGH, "a0[%f] a1[%f]	a2[%f]	b00[%f]\n",a0,a1,a2,b00);
	QMP6988_MSG_4(HIGH, "bt1[%f]	bt2[%f] bp1[%f] b11[%f]\n",bt1,bt2,bp1,b11);
	QMP6988_MSG_4(HIGH, "bp2[%f]	b12[%f] b21[%f] bp3[%f]\n",bp2,b12,b21,bp3);
	QMP6988_MSG_0(HIGH, "<----------- float calibration data -------------->\n");
#endif
    return comres;
}


sns_ddf_status_e qmp6988_software_reset(void)
{
	sns_ddf_status_e status = SNS_DDF_SUCCESS; 
	uint8_t data;

	data = 0xe6;
	status = qmp6988_write_reg(QMP6988_RESET_REG, &data, 1);
	if(status != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_software_reset fail!!! \n");
		return status;
	}
	qmp6988_delay(20);
	data = 0x00;
	status = qmp6988_write_reg(QMP6988_RESET_REG, &data, 1);
	if(status != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_software_reset fail!!! \n");
		return status;
	}

	return status;
}

sns_ddf_status_e qmp6988_set_powermode(int power_mode)
{
	uint8_t data;
	sns_ddf_status_e status = SNS_DDF_SUCCESS; 

	QMP6988_MSG_1(HIGH,"qmp_set_powermode %d \n", power_mode);

	status = qmp6988_read_reg(QMP6988_CTRLMEAS_REG, &data, 1);	
	if(status != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_set_powermode fail!!! \n");
		return status;
	}
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
	status = qmp6988_write_reg(QMP6988_CTRLMEAS_REG, &data, 1);

	QMP6988_MSG_1(HIGH,"qmp_set_powermode 0xf4=0x%x \n", data);
	//qmp6988_delay(20);

	return status;
}


static sns_ddf_status_e qmp6988_set_filter(unsigned char filter)
{	
	uint8_t data;
	sns_ddf_status_e status = SNS_DDF_SUCCESS; 

	if((filter>=QMP6988_FILTERCOEFF_OFF) &&(filter<=QMP6988_FILTERCOEFF_32))
	{
		data = (filter&0x07);
		status = qmp6988_write_reg(QMP6988_CONFIG_REG, &data, 1);
		if(status != SNS_DDF_SUCCESS)
		{
			QMP6988_MSG_0(HIGH, "qmp6988_set_powermode fail!!! \n");
			return status;
		}
	}
	//qmp6988_delay(20);

	return status;
}

static sns_ddf_status_e qmp6988_set_oversampling_p(unsigned char oversampling_p)
{
	uint8_t data;
	sns_ddf_status_e status = SNS_DDF_SUCCESS; 

	status = qmp6988_read_reg(QMP6988_CTRLMEAS_REG, &data, 1);
	if(status != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_set_oversampling_p fail!!! \n");
		return status;
	}
	if((oversampling_p>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_p<=QMP6988_OVERSAMPLING_64X))
	{
		data &= 0xe3;
		data |= (oversampling_p<<2);
		
		status = qmp6988_write_reg(QMP6988_CTRLMEAS_REG, &data, 1);
	}	
	//qmp6988_delay(20);

	return status;
}

static sns_ddf_status_e qmp6988_set_oversampling_t(unsigned char oversampling_t)
{
	uint8_t data;
	sns_ddf_status_e status = SNS_DDF_SUCCESS; 

	status = qmp6988_read_reg(QMP6988_CTRLMEAS_REG, &data, 1);
	if(status != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_set_oversampling_p fail!!! \n");
		return status;
	}
	if((oversampling_t>=QMP6988_OVERSAMPLING_SKIPPED)&&(oversampling_t<=QMP6988_OVERSAMPLING_64X))
	{
		data &= 0x1f;
		data |= (oversampling_t<<5);
		
		status = qmp6988_write_reg(QMP6988_CTRLMEAS_REG, &data, 1);
	}	
	//qmp6988_delay(20);	
	return status;
}


static sns_ddf_status_e qmp6988_set_standbydur(int32_t standby_dur)
{
	uint8_t	standby_data=0;
	uint8_t data;
    sns_ddf_status_e status = SNS_DDF_SUCCESS;

	if(standby_dur == QMP6988_REGV_STANDBY_DUR_1MS)
		standby_data = 0;
	else if(standby_dur == QMP6988_REGV_STANDBY_DUR_5MS)
		standby_data = 1;
	else if(standby_dur == QMP6988_REGV_STANDBY_DUR_50MS)
		standby_data = 2;
	else if(standby_dur == QMP6988_REGV_STANDBY_DUR_250MS)
		standby_data = 3;
	else if(standby_dur == QMP6988_REGV_STANDBY_DUR_500MS)
		standby_data = 4;
	else if(standby_dur == QMP6988_REGV_STANDBY_DUR_1000MS)
		standby_data = 5;
	else if(standby_dur == QMP6988_REGV_STANDBY_DUR_2000MS)
		standby_data = 6;
	else
		standby_data = 7;

	standby_data = (standby_data<<5);
	status = qmp6988_read_reg(QMP6988_IOSETUP_REG, &data, 1);	
	if(status != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_set_standbydur read fail!!! \n");
		return status;
	}
	data &= 0x1F;
	data |= standby_data;
	status = qmp6988_write_reg(QMP6988_CTRLMEAS_REG, &data, 1);
	if(status != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988_set_standbydur write fail!!! \n");
		return status;
	}

	return status;
}

static sns_ddf_status_e qmp6988_init_client(sns_dd_qmp6988_state_t *state)
{
    sns_ddf_status_e status = SNS_DDF_SUCCESS;

	qmp6988_set_powermode(QMP6988_NORMAL_MODE);
	qmp6988_set_filter(QMP6988_FILTERCOEFF_OFF);
	qmp6988_set_oversampling_p(QMP6988_OVERSAMPLING_2X);
	qmp6988_set_oversampling_t(QMP6988_OVERSAMPLING_1X);

    return status;
}


sns_ddf_status_e qmp6988_calc_pressure(void)
{
	sns_ddf_status_e err = SNS_DDF_SUCCESS;
	uint8_t retry_count = 0;
	QMP6988_U32_t P_read, T_read;
	QMP6988_S32_t P_raw, T_raw;
	uint8_t a_data_u8r[3] = {0};
	double Tr;

	a_data_u8r[0] = 0x08;
	retry_count = 0;
#if 0
	while(a_data_u8r[0]&0x08)
	{
		err = qmp6988_read_reg(QMP6988_DEVICE_STAT_REG, a_data_u8r, 1);
		if(err != SNS_DDF_SUCCESS)
		{
			QMP6988_MSG_0(HIGH, "qmp6988 read status reg error! \n");
			return;
		}		
		QMP6988_MSG_1(HIGH, "qmp6988 read status 0xf3 = 0x%02x \n", a_data_u8r[0]);
		qmp6988_delay(10);
		retry_count++;
		if(retry_count > 5)
			return;
	}
#endif
	// press
	err = qmp6988_read_reg(QMP6988_PRESSURE_MSB_REG, a_data_u8r, 3);
	if(err != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988 read press raw error! \n");
		return err;
	}
	P_read = (QMP6988_U32_t)(
	(((QMP6988_U32_t)(a_data_u8r[0])) << SHIFT_LEFT_16_POSITION) |
	(((QMP6988_U16_t)(a_data_u8r[1])) << SHIFT_LEFT_8_POSITION) |
	(a_data_u8r[2]));
	P_raw = (QMP6988_S32_t)(P_read - SUBTRACTOR);
	
	// temp
	err = qmp6988_read_reg(QMP6988_TEMPERATURE_MSB_REG, a_data_u8r, 3);
	if(err != SNS_DDF_SUCCESS)
	{
		QMP6988_MSG_0(HIGH, "qmp6988 read temp raw error! \n");
		return err;
	}
	T_read = (QMP6988_U32_t)(
	(((QMP6988_U32_t)(a_data_u8r[0])) << SHIFT_LEFT_16_POSITION) |
	(((QMP6988_U16_t)(a_data_u8r[1])) << SHIFT_LEFT_8_POSITION) | 
	(a_data_u8r[2]));
	T_raw = (QMP6988_S32_t)(T_read - SUBTRACTOR);

	Tr = a0 + a1*T_raw + a2*T_raw*T_raw;
	//Unit centigrade
	qmp6988_data->temperature = Tr / 256.0f;  
	QMP6988_MSG_1(HIGH, "temperature = %f\n",qmp6988_data->temperature);
	//compensation pressure, Unit Pa
	qmp6988_data->pressure = b00+bt1*Tr+bp1*P_raw+b11*Tr*P_raw+bt2*Tr*Tr+bp2*P_raw*P_raw+b12*P_raw*Tr*Tr+b21*P_raw*P_raw*Tr+bp3*P_raw*P_raw*P_raw;
	QMP6988_MSG_1(HIGH, "Pressure = %f\n",qmp6988_data->pressure);

	//g_qmp6988.altitude = (pow((101325.00/g_qmp6988.pressure),1/5.257)-1)*(g_qmp6988.temperature+273.15)/0.0065;
	//QMP6988_LOG("altitude = %f\n",g_qmp6988.altitude);
	return err;
}

sns_ddf_status_e qmp_get_pressure_temp(uint32_t *pressure, int32_t *temp)
{
	sns_ddf_status_e status=SNS_DDF_SUCCESS;

	status = qmp6988_calc_pressure();

	*temp = (uint32_t)(qmp6988_data->temperature*100);
	*pressure = (uint32_t)qmp6988_data->pressure;

	return status;
}

sns_ddf_status_e sns_dd_qmp6988_check_chip_id(sns_dd_qmp6988_state_t *drv)
{
    sns_ddf_status_e status;
    uint8_t chip_id = 0;
    uint8_t read_count = 0;

    if (drv == NULL)
    {
       QMP6988_MSG_0(ERROR, "sns_dd_qmp6988_check_chip_id - invalid state");
       return SNS_DDF_EINVALID_PARAM;
    }

    status = sns_ddf_read_port(drv->i2c_handle,QMP6988_CHIP_ID_REG,&chip_id, 1, &read_count);
    if (status != SNS_DDF_SUCCESS)
    {
        QMP6988_MSG_0(ERROR, "sns_ddf_read_port failed");
        return status;
    }
    QMP6988_MSG_1(HIGH, "sns_dd_qmp6988_check_chip_id - 0x%X", chip_id);

    if ((chip_id != QMP6988_CHIP_ID))
    {
        return SNS_DDF_EDEVICE;
    }

    drv->chip_id = chip_id;

    return SNS_DDF_SUCCESS;
}


/*!
*  @brief Resets the driver and device so they return to the state they were
*         in after init() was called.
*
* @detail
*
* @param[in] drv: pointer to the driver structure
*
* @return
*  The error code definition within the DDF
*  SNS_DDF_SUCCESS on success.
*/

sns_ddf_status_e sns_dd_qmp6988_reset(sns_ddf_handle_t dd_handle)
{
	sns_ddf_status_e status = SNS_DDF_SUCCESS;

    sns_ddf_delay(2000);

	return status;
}


/*!
* @brief Initializes the  QMP6988
*             determines the device to use
*
*
*  @detail
*   Allocates memory for driver drv structure.
*   Opens the device port by calling sns_ddf_open_port routine
*   Calls sns_dd_qmp6988_reset routine
*
*  @param[out] dd_handle_ptr  Pointer that this function must malloc and
*                             populate. This is a handle to the driver
*                            instance that will be passed in to all
*                             other functions.
*  @param[in]  smgr_handle    Handle used to identify this driver when it
*                             calls into Sensors Manager functions.
*  @param[in]  nv_params      NV parameters retrieved for the driver.
*  @param[in]  device_info    Information describing each of the physical
*                             devices controlled by this driver. This is
*                            used to configure the bus and talk to the
*                           device.
*  @param[in]  memhandler     Memory handler used to dynamically allocate
*                            output parameters, if applicable. NB: Do
*                           not use memhandler to allocate memory for
*                           @a dd_handle_ptr.
*  @param[in]  num_devices    Length of @a device_info.
*  @param[out] sensors        List of supported sensors, populated and
*                              returned by this function.
*  @param[out] num_sensors    Length of @a sensors.
*
*  @return
*    The error code definition within the DDF
*    SNS_DDF_SUCCESS on success.
*
*/

 sns_ddf_status_e sns_dd_qmp6988_init(
    sns_ddf_handle_t*        dd_handle_ptr,
    sns_ddf_handle_t         smgr_handle,
    sns_ddf_nv_params_s*     nv_params,
    sns_ddf_device_access_s  device_info[],
    uint32_t                 num_devices,
    sns_ddf_memhandler_s*    memhandler,
    sns_ddf_sensor_e*        sensors[],
    uint32_t*                num_sensors)
{
    sns_ddf_status_e status;
    sns_dd_qmp6988_state_t *drv;

#if QMP6988_CONFIG_ENABLE_UIMAGE
	sns_ddf_smgr_set_uimg_refac(smgr_handle);
#endif
    status = sns_ddf_malloc_ex((void **)&drv, sizeof(sns_dd_qmp6988_state_t), smgr_handle);
    if(status != SNS_DDF_SUCCESS)
    {
        QMP6988_MSG_0(HIGH, "sns_ddf_malloc_ex failed \n");
        return SNS_DDF_ENOMEM;
    }
    memset(drv, 0, sizeof(sns_dd_qmp6988_state_t));
	qmp6988_data = drv;
    *dd_handle_ptr = (sns_ddf_handle_t) drv;
    drv->smgr_handle = smgr_handle;
    status = sns_ddf_open_port(&(drv->i2c_handle),&(device_info->port_config));

    if(status != SNS_DDF_SUCCESS)
    {
        QMP6988_MSG_0(HIGH, "open_port failed");
        goto port_err;
    }
    qmp6988_port_handle = drv->i2c_handle;
    status = sns_dd_qmp6988_check_chip_id(drv);
    if(status != SNS_DDF_SUCCESS)
    {
        QMP6988_MSG_0(ERROR, "sns_dd_qmp6988_check_chip_id failed");
        goto init_err;
    }

	drv->mode = QMP6988_MODE_STANDARD;
	
	status = qmp6988_software_reset();
    if (status != SNS_DDF_SUCCESS)
    {
        QMP6988_MSG_0(ERROR, "qmp6988_software_reset failed");
        goto init_err;
    }
	status = qmp6988_get_calib_param();
    if (status != SNS_DDF_SUCCESS)
    {
        QMP6988_MSG_0(ERROR, "qmp6988_get_calib_param failed");
        goto init_err;
    }

    status = qmp6988_init_client(drv);
    if (status != SNS_DDF_SUCCESS)
    {
        QMP6988_MSG_0(ERROR, "qmp_init_client failed");
        goto init_err;
    }


    /* Fill out supported sensor info */
    *num_sensors = 2;
    *sensors = qmp6988_sensors;

    if (status != SNS_DDF_SUCCESS)
    {
        goto init_err;
    }
    return SNS_DDF_SUCCESS;

init_err:

    sns_ddf_close_port(drv->i2c_handle);
port_err:
    sns_ddf_mfree_ex(drv, smgr_handle);
	qmp6988_data = NULL;

    return status;
}

 sns_ddf_status_e sns_dd_qmp6988_config_mode
(
   sns_dd_qmp6988_state_t *state,
   sns_ddf_powerstate_e      *mode
)
{
   sns_ddf_status_e status = SNS_DDF_SUCCESS;

   switch (*mode)
   {
   case SNS_DDF_POWERSTATE_ACTIVE:
		//qmp6988_init_client(&(state->data));
		status = qmp6988_set_powermode(QMP6988_NORMAL_MODE);
		if(status != SNS_DDF_SUCCESS)
		{
			return status;
		}
		state->mode = QMP6988_MODE_STANDARD;
		break;
   case SNS_DDF_POWERSTATE_LOWPOWER:
	   status = qmp6988_set_powermode(QMP6988_SLEEP_MODE);
	   if(status != SNS_DDF_SUCCESS)
	   {
		   return status;
	   }
	   state->mode = QMP6988_MODE_ULTRA_LOW_POWER;
	   break;
   default:
      return status;

   }



   return SNS_DDF_SUCCESS;
}


static sns_ddf_status_e sns_dd_qmp6988_config_odr(
        sns_dd_qmp6988_state_t   *state,
        uint32_t                odr_value)
	{
		sns_ddf_status_e stat;
		int32_t standby_dur = 0;
		uint32_t odr_reported = 0;

		if (odr_value < 1)
		{
			standby_dur = QMP6988_REGV_STANDBY_DUR_4000MS;
			odr_reported = 0;
		}
		else if (odr_value < 2)
		{
			standby_dur = QMP6988_REGV_STANDBY_DUR_1000MS;
			odr_reported = 1;
		}
		else if (odr_value < 3)
		{
			standby_dur = QMP6988_REGV_STANDBY_DUR_500MS;
			odr_reported = 2;
		}
		else if (odr_value < 5)
		{
			standby_dur = QMP6988_REGV_STANDBY_DUR_250MS;
			odr_reported = 4;
		}
		else if (odr_value < 21)
		{
			standby_dur = QMP6988_REGV_STANDBY_DUR_50MS;
			odr_reported = 20;
		}
		else if (odr_value < 201)
		{
			standby_dur = QMP6988_REGV_STANDBY_DUR_5MS;
			odr_reported = 200;
		}
		else
		{
			standby_dur = QMP6988_REGV_STANDBY_DUR_1MS;
			odr_reported = 1000;
		}
		stat = qmp6988_set_standbydur(standby_dur);
		if (stat != SNS_DDF_SUCCESS)
		{
			 return stat;
		}
		state->odr_reported = odr_reported;

		return SNS_DDF_SUCCESS;
	}




/*!
*  @brief Sets an attribute of the qmp6988
*
* @detail
*    Called by SMGR to set certain device attributes that are
*    programmable. Curently its the power mode and range.
*
*  @param[in] dd_handle   Handle to a driver instance.
*  @param[in] sensor Sensor for which this attribute is to be set.
*  @param[in] attrib      Attribute to be set.
*  @param[in] value      Value to set this attribute.
*
*  @return
*   The error code definition within the DDF
*    SNS_DDF_SUCCESS on success.
*
*/

 sns_ddf_status_e sns_dd_qmp6988_set_attr(
    sns_ddf_handle_t     dd_handle,
    sns_ddf_sensor_e     sensor,
    sns_ddf_attribute_e  attrib,
    void*                value_ptr)
{
    sns_ddf_status_e ret_val = SNS_DDF_SUCCESS;
	sns_dd_qmp6988_state_t *state = (sns_dd_qmp6988_state_t *)dd_handle;
	QMP6988_MSG_4(HIGH, "<qmp6988_if_ set_attrib> 0x%x sensor: %d attr: %d val: %d",
			  dd_handle, sensor, attrib, *(uint8_t *)value_ptr);

    if((sensor != SNS_DDF_SENSOR_PRESSURE) && (sensor != SNS_DDF_SENSOR__ALL)
        && (sensor != SNS_DDF_SENSOR_TEMP))
    {
        QMP6988_MSG_0(HIGH, "sns_dd_qmp6988_set_attr invalid sensor");
        return SNS_DDF_EINVALID_PARAM;
    }

    if(dd_handle == NULL)
    {
       QMP6988_MSG_0(HIGH, "sns_dd_qmp6988_set_attr - invalid state");
       return SNS_DDF_EINVALID_PARAM;
    }
    switch(attrib)
    {
    case SNS_DDF_ATTRIB_POWER_STATE:
		QMP6988_MSG_0(HIGH, "SNS_DDF_ATTRIB_POWER_STATE");
		ret_val = sns_dd_qmp6988_config_mode(state, (sns_ddf_powerstate_e *)value_ptr);
		break;
    case SNS_DDF_ATTRIB_RANGE:
		QMP6988_MSG_1(HIGH, "SNS_DDF_ATTRIB_RANGE, %d", *((uint32_t*)value_ptr));
		return SNS_DDF_SUCCESS;
        break;
    case SNS_DDF_ATTRIB_LOWPASS:
		QMP6988_MSG_0(HIGH, "SNS_DDF_ATTRIB_LOWPASS");
		return SNS_DDF_SUCCESS;
        break;
	case SNS_DDF_ATTRIB_ODR:
	   {
		   sns_ddf_odr_t odr_value = *(sns_ddf_odr_t *)value_ptr;
		   QMP6988_MSG_1(HIGH, "SNS_DDF_ATTRIB_ODR, %d", odr_value);
#if 0
		   if ((ret_val = sns_dd_qmp6988_config_odr(state,odr_value))
				   != SNS_DDF_SUCCESS)
		   {
			   return ret_val;
		   }
#endif
		   return SNS_DDF_SUCCESS;
	   }

    case SNS_DDF_ATTRIB_RESOLUTION_ADC:
		QMP6988_MSG_0(HIGH, "SNS_DDF_ATTRIB_RESOLUTION_ADC");
		return SNS_DDF_SUCCESS;
        break;

    default:
        QMP6988_MSG_0(HIGH, "sns_dd_qmp6988_set_attr invalid attrib");
        return SNS_DDF_EINVALID_ATTR;
    }

    return ret_val;
}


/*!
*  @brief Called by the SMGR to retrieves the value of an attribute of
*  the sensor.
*
*  @detail
*   range and resolution info is from the device data sheet.
*
*  @param[in] dd_handle   Handle to a driver instance.
*  @param[in] sensor       Sensor whose attribute is to be retrieved.
*  @param[in] attrib      Attribute to be retrieved.
*  @param[in] memhandler  Memory handler used to dynamically allocate
*                         output parameters, if applicable.
*  @param[out] value      Pointer that this function will allocate or set
*                         to the attribute's value.
*  @param[out] num_elems  Number of elements in @a value.
*
*  @return
*    The error code definition within the DDF
*    SNS_DDF_SUCCESS on success.
*
*/


 sns_ddf_status_e sns_dd_qmp6988_get_attr(
    sns_ddf_handle_t       dd_handle,
    sns_ddf_sensor_e       sensor,
    sns_ddf_attribute_e    attrib,
    sns_ddf_memhandler_s*  memhandler,
    void**                 value,
    uint32_t*              num_elems)
{
    sns_dd_qmp6988_state_t *drv = (sns_dd_qmp6988_state_t *)dd_handle;

    QMP6988_MSG_2(HIGH, "sns_dd_qmp6988_get_attr sensor=%d attrib=%d", sensor, attrib);

    if((sensor != SNS_DDF_SENSOR_PRESSURE) && (sensor != SNS_DDF_SENSOR_TEMP))
    {
        QMP6988_MSG_0(HIGH, "sns_dd_qmp6988_get_attr invalid sensor");
        return SNS_DDF_EINVALID_PARAM;
    }

    if(drv == NULL)
    {
       QMP6988_MSG_0(HIGH, "sns_dd_qmp6988_get_attr - invalid state");
       return SNS_DDF_EINVALID_PARAM;
    }

    switch (attrib)
    {
    case SNS_DDF_ATTRIB_POWER_INFO:
        {
            sns_ddf_power_info_s* power_attrib;
            power_attrib = sns_ddf_memhandler_malloc_ex(memhandler,
                                                     sizeof(sns_ddf_power_info_s), drv->smgr_handle);
            if (power_attrib == NULL)
            {
                QMP6988_MSG_0(HIGH, "malloc failure power_attrib");
                return SNS_DDF_ENOMEM;
            }
			if(drv->mode < QMP6988_MODE_ULTRA_LOW_POWER || drv->mode >= QMP6988_NUM_MODES)
				drv->mode = QMP6988_MODE_ULTRA_LOW_POWER;

            power_attrib->active_current = qmp6988_mode_param[drv->mode].avg_current_uA;
            power_attrib->lowpower_current = 0;
            *num_elems = 1;
            *value = power_attrib;
        }
        break;

    case SNS_DDF_ATTRIB_RANGE:
        {
            sns_ddf_range_s *device_ranges;

            *num_elems = 1;
            device_ranges = sns_ddf_memhandler_malloc_ex(memhandler,
                                                      (*num_elems)*sizeof(sns_ddf_range_s), drv->smgr_handle);
            if (device_ranges == NULL)
            {
                QMP6988_MSG_0(HIGH,"malloc failure power_attrib");
                return SNS_DDF_ENOMEM;
            }

            if (sensor == SNS_DDF_SENSOR_PRESSURE)
            {
             // Pressure Range is 300 to 1100 hPa = 30,000 to 110,000 Pascals.
             // Which is +9000 meters to -500 meters at sea level.
             // Pressure at sea level is 1013.25 hPa.
             // A pressure change of 1 hPa (100 Pascals) corresponds to 8.43 meters
             // at sea level.
                device_ranges[0].min = FX_FLTTOFIX_Q16(300.0);
                device_ranges[0].max = FX_FLTTOFIX_Q16(1100.0);
            }
            else
            {
                //Temperature Operational range is -40 to +85
                device_ranges[0].min = FX_FLTTOFIX_Q16(-40.0);
                device_ranges[0].max = FX_FLTTOFIX_Q16(85.0);
            }

            *value = device_ranges;
        }
        break;

    case SNS_DDF_ATTRIB_RESOLUTION_ADC:
        {
            sns_ddf_resolution_adc_s* resp;
            *num_elems = 1;
            if((resp = sns_ddf_memhandler_malloc_ex(memhandler, (*num_elems)*sizeof(sns_ddf_resolution_adc_s), drv->smgr_handle)) == NULL )
            {
                 return SNS_DDF_ENOMEM;
            }
			
			if(drv->mode < QMP6988_MODE_ULTRA_LOW_POWER || drv->mode >= QMP6988_NUM_MODES)
				drv->mode = QMP6988_MODE_ULTRA_LOW_POWER;
            resp->bit_len = qmp6988_mode_param[drv->mode].bits;
			resp->max_freq = 26;
            *value = resp;
        }
        break;
	case SNS_DDF_ATTRIB_ODR:
		{
		sns_ddf_odr_t *odr = sns_ddf_memhandler_malloc_ex(memhandler ,sizeof(sns_ddf_odr_t),drv->smgr_handle);
		if(odr == NULL)
		{		
			QMP6988_MSG_0(HIGH, "SNS_DDF_ATTRIB_ODR memory fail");
			return SNS_DDF_ENOMEM;
		}
		*odr = 1;
		*value = odr;		
		*num_elems = 1;
		}
		break;
	case SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST:
	    {
	        uint32_t *odr_list;
			*num_elems = 1;

	        odr_list = sns_ddf_memhandler_malloc_ex(memhandler,
	                sizeof(sns_ddf_odr_t)*(*num_elems),
	                drv->smgr_handle);
	        if (NULL == odr_list) {
	            QMP6988_MSG_0(HIGH, "No memory for SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST");
	            return SNS_DDF_ENOMEM;
	        }
			odr_list[0] = 1;

	        *value = odr_list;
	    }
	    break;

    case SNS_DDF_ATTRIB_RESOLUTION:
        {
            sns_ddf_resolution_t *device_res;
            int res_num = 1;

            device_res = sns_ddf_memhandler_malloc_ex(memhandler,
                                                   res_num*sizeof(sns_ddf_resolution_t), drv->smgr_handle);
            if (device_res == NULL)
            {
                QMP6988_MSG_0(HIGH, "malloc failure device_res");
                return SNS_DDF_ENOMEM;
            }

            *num_elems = res_num;

			if (sensor == SNS_DDF_SENSOR_PRESSURE)
            {
                device_res[0] =  FX_FLTTOFIX_Q16(0.0072); /* 0.0072 hPa   at standard resolution */

            }
            else
            {
                device_res[0] =  FX_FLTTOFIX_Q16(0.005); /* 0.005°C */
            }

            *value = device_res;
        }
        break;

    case SNS_DDF_ATTRIB_LOWPASS:
        return SNS_DDF_EINVALID_PARAM;
    case SNS_DDF_ATTRIB_DELAYS:
        break;
    case SNS_DDF_ATTRIB_DEVICE_INFO:
        {
            sns_ddf_device_info_s *device_info;
            device_info = sns_ddf_memhandler_malloc_ex(memhandler,
                                                    sizeof(sns_ddf_device_info_s), drv->smgr_handle);
            if(device_info == NULL)
            {
                QMP6988_MSG_0(HIGH,"malloc failure device_info");
                return SNS_DDF_ENOMEM;
            }

            device_info->vendor = "QST";
            device_info->model = "QMP6988";
            if (sensor == SNS_DDF_SENSOR_PRESSURE)
            {
               device_info->name = "Pressure";
            }
            else
            {
               device_info->name = "Temperature";
            }
            device_info->version = QMP6988_SNS_DDF_ATTRIB_VERSION;

            *value = device_info;
            *num_elems = 1;
        }
        break;

    default:
        QMP6988_MSG_0(HIGH, "sns_dd_qmp6988_get_attr invalid attrib");
        return SNS_DDF_EINVALID_ATTR;
    }

    return SNS_DDF_SUCCESS;
}



 sns_ddf_status_e sns_dd_qmp6988_self_test(
                sns_ddf_handle_t dd_handle,
                sns_ddf_sensor_e sensor,
                sns_ddf_test_e test,
                uint32_t* err)
{
    sns_dd_qmp6988_state_t *state = (sns_dd_qmp6988_state_t *)dd_handle;
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    //uint32_t pressure;
    //int32_t temperature;

    if (test != SNS_DDF_TEST_SELF && test != SNS_DDF_TEST_SELF_SW)
    {
       return SNS_DDF_EINVALID_PARAM;
    }

    if (state == NULL)
    {
       return SNS_DDF_EINVALID_PARAM;
    }

    return status;
}

sns_ddf_status_e sns_dd_qmp6988_probe(
                        sns_ddf_device_access_s* device_info,
                        sns_ddf_memhandler_s*    memhandler,
                        uint32_t*                num_sensors,
                        sns_ddf_sensor_e**       sensors )
{
  sns_ddf_status_e status;
  sns_ddf_handle_t port_handle;
  uint8_t chip_id;
  uint8_t read_count;

  *num_sensors = 0;
  *sensors = NULL;

  status = sns_ddf_open_port(&port_handle, &(device_info->port_config));
  if(status != SNS_DDF_SUCCESS)
  {
  	QMP6988_MSG_0(HIGH, "sns_ddf_open_port failed \n");
    return status;
  }

  /* Read & Verify Device ID */
  status = sns_ddf_read_port(port_handle,QMP6988_CHIP_ID_REG,
                               &chip_id, 1, &read_count);

  if(status != SNS_DDF_SUCCESS)
  {
     sns_ddf_close_port(port_handle);
     QMP6988_MSG_0(HIGH, "sns_ddf_read_port failed \n");
     return status;
  }

  if((chip_id != QMP6988_CHIP_ID))
  {
     sns_ddf_close_port(port_handle);
     return SNS_DDF_EDEVICE;
  }

  /* Registers are correct. This is probably a LPS25H */
  *num_sensors = 2;
  *sensors = sns_ddf_memhandler_malloc( memhandler,
                 sizeof(sns_ddf_sensor_e) * *num_sensors );
  if(*sensors != NULL )
  {
    (*sensors)[0] = SNS_DDF_SENSOR_PRESSURE;
    (*sensors)[1] = SNS_DDF_SENSOR_TEMP;
    status = SNS_DDF_SUCCESS;
  }
  else
  {
    status = SNS_DDF_ENOMEM;
  }
  sns_ddf_close_port(port_handle);
  return status;
}


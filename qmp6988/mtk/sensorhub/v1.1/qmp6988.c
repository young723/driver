/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <seos.h>
#include <util.h>
#include <sensors.h>

#include <barometer.h>
#include <contexthub_core.h>
#include <cust_baro.h>

//#define BUILD_ON_ANDROID_P

#define BARO_NAME                    "qmp6988"
#define I2C_SPEED                    100000

#define QMP6988_IIC_ADDR		                0x70
#define QMP6988_CHIP_ID                         0x5c
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

#define QMP6988_U16_t unsigned short
#define QMP6988_S16_t short
#define QMP6988_U32_t unsigned int
#define QMP6988_S32_t int

enum qmp6988State {
    STATE_SAMPLE = CHIP_SAMPLING,
    STATE_CONVERT = CHIP_CONVERT,
    STATE_SAMPLE_DONE = CHIP_SAMPLING_DONE,
    STATE_ENABLE = CHIP_ENABLE,
    STATE_ENABLE_DONE = CHIP_ENABLE_DONE,
    STATE_DISABLE = CHIP_DISABLE,
    STATE_DISABLE_DONE = CHIP_DISABLE_DONE,
    STATE_RATECHG = CHIP_RATECHG,
    STATE_RATECHG_DONE = CHIP_RATECHG_DONE,
    STATE_INIT_DONE = CHIP_INIT_DONE,
    STATE_IDLE = CHIP_IDLE,
    STATE_RESET = CHIP_RESET,
    STATE_POWER_ON,
    STATE_POWER_OFF,
    STATE_VERIFY_ID,
    STATE_AWAITING_COMP_PARAMS,
    STATE_MEAS,
    STATE_CONFIG,
    STATE_CORE,
};

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
} __attribute__((packed));

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
static struct qmp6988_calibration_data qmp6988_cali;

static unsigned char reg_f4_value = 0x2b;
static uint8_t cali_read_reg = 0;
static uint8_t cali_reg[QMP6988_CALIBRATION_DATA_LENGTH];
void qmp6988_calc_calibration_data(const uint8_t *a_data_u8r);

static struct qmp6988Task {
    /* i2c operation read some data which is only use in prative driver */
    /* txBuf for i2c operation, fill register and fill value */
    uint8_t txBuf[8];
    /* rxBuf for i2c operation, receive rawdata */
    struct transferDataInfo dataInfo;
    struct baroDataPacket baroPacket;
    uint8_t rxBuf[25];
    uint8_t verifyId;
    struct baro_hw *hw;
    uint8_t i2c_addr;
    /* data for factory */
    struct SingleAxisDataPoint factoryData;
    uint32_t i32kP;
    uint32_t i32kT;
} mTask;

static struct qmp6988Task *qmp6988DebugPoint;


static int qmp6988_set_powermode(I2cCallbackF i2cCallBack, void *next_state, uint8_t power_mode)
{

    mTask.txBuf[0] = QMP6988_CTRLMEAS_REG;
    mTask.txBuf[1] = (reg_f4_value&0xfc)|power_mode;
    osLog(LOG_ERROR, "qmp6988_set_powermoder %x %x\n", mTask.txBuf[0], mTask.txBuf[1]);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
}

static int qmp6988_power_on(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    return qmp6988_set_powermode(i2cCallBack, next_state, QMP6988_NORMAL_MODE);
}

static int qmp6988_power_off(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
     return qmp6988_set_powermode(i2cCallBack, next_state,  QMP6988_SLEEP_MODE);
}

static int qmp6988_read_config(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
     mTask.txBuf[0] = QMP6988_CTRLMEAS_REG;
     return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, i2cCallBack,
                         next_state);
}

static int qmp6988_sample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "qmp6988 Sample, rx dataInfo error\n");
        return -1;
    }
    
    if(cali_read_reg == 0)
    {
        mTask.txBuf[0] = QMP6988_CALIBRATION_DATA_START;
        ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[0], 1,
                            (uint8_t*)&cali_reg[0], 6, i2cCallBack,
                            (void *)STATE_IDLE);
        mTask.txBuf[1] = QMP6988_CALIBRATION_DATA_START + 6;
        ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[1], 1,
                            (uint8_t*)&cali_reg[6], 6, i2cCallBack,
                            (void *)STATE_IDLE);
        mTask.txBuf[2] = QMP6988_CALIBRATION_DATA_START + 12;
        ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[2], 1,
                            (uint8_t*)&cali_reg[12], 6, i2cCallBack,
                            (void *)STATE_IDLE);
        mTask.txBuf[3] = QMP6988_CALIBRATION_DATA_START+ 18;
        ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[3], 1,
                            (uint8_t*)&cali_reg[18], 6, i2cCallBack,
                            (void *)STATE_IDLE);
        mTask.txBuf[4] = QMP6988_CALIBRATION_DATA_START+ 24;
        ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[4], 1,
                            (uint8_t*)&cali_reg[24], 1, i2cCallBack,
                            next_state);

        qmp6988_calc_calibration_data(cali_reg);
        if(ret >= 0)
            cali_read_reg = 1;
    }
    
    mTask.txBuf[0] = 0xF7;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                        mTask.rxBuf, 6, i2cCallBack,
                        next_state);
    return ret;
}


static void getTempAndBaro(const uint8_t *reg_data, float *pressure_Pa, float *temp_centigrade)
{
//    int32_t upressure = ((int32_t)tmp[0] << 16) | ((int32_t)tmp[1] << 8) | tmp[2];
//    int32_t utemp = ((int32_t)tmp[3] << 16) | ((int32_t)tmp[4] << 8) | tmp[5];
//    float fTsc, fPsc;
//    float qua2, qua3;
//    float fpressure;
	QMP6988_S32_t P_read, T_read;
	float Tr;      // double
    float temperature;
	float pressure;
	
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
	T_read = (QMP6988_S32_t)(T_read - 8388608);
	
	Tr = a0 + a1*T_read + a2*T_read*T_read;
	//Unit centigrade
	temperature = (float)(Tr / 256.0f);	
	//compensation pressure, Unit Pa
	pressure = (float)(b00+bt1*Tr+bp1*P_read+b11*Tr*P_read+bt2*Tr*Tr+bp2*P_read*P_read+b12*P_read*Tr*Tr+b21*P_read*P_read*Tr+bp3*P_read*P_read*P_read);
	//g_qmp6988.elevation = ((pow((101.325/pressure), 1/5.257)-1)*(tempearture+273.15))/0.0065;

    *temp_centigrade = (float)temperature;
    *pressure_Pa = (float)pressure/100.0f;
}

static int qmp6988_convert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct baroData *data = mTask.baroPacket.outBuf;
    float pressure_Pa, temp_centigrade;

    getTempAndBaro(mTask.rxBuf, &pressure_Pa, &temp_centigrade);
    data[0].fdata = pressure_Pa;
    data[0].sensType = SENS_TYPE_BARO;
    mTask.factoryData.idata = (uint32_t)(pressure_Pa * BAROMETER_INCREASE_NUM_AP);
    txTransferDataInfo(&mTask.dataInfo, 1, data);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    osLog(LOG_ERROR,"qmp6988_convert %d  %d\n",(int)pressure_Pa, (int)temp_centigrade);
    return 0;
}
static void baroGetCalibration(int32_t *cali, int32_t size)
{

}
static void baroSetCalibration(int32_t *cali, int32_t size)
{

}
static void baroGetData(void *sample)
{
    struct SingleAxisDataPoint *singleSample = (struct SingleAxisDataPoint *)sample;
    singleSample->idata = mTask.factoryData.idata;
}

#if defined(BUILD_ON_ANDROID_P)
static void qmp6988_getSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, BARO_NAME, sizeof(data->name));
}
#endif

void qmp6988_calc_calibration_data(const uint8_t *a_data_u8r)
{
#if 0
	temp_COE.x = (QMP6988_U32_t)((a_data_u8r[18] << \
		SHIFT_LEFT_12_POSITION) | (a_data_u8r[19] << \
		SHIFT_LEFT_4_POSITION) | (a_data_u8r[24] & 0x0f));
	 qmp6988_cali.COE_a0 = 	temp_COE.x;
#else
	 qmp6988_cali.COE_a0 = (QMP6988_S32_t)((((QMP6988_S32_t)a_data_u8r[18] << SHIFT_LEFT_12_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[19] << SHIFT_LEFT_4_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[24] & 0x0f))<<12);
	 qmp6988_cali.COE_a0 =  qmp6988_cali.COE_a0>>12;
#endif
	
	 qmp6988_cali.COE_a1 = (QMP6988_S16_t)(((a_data_u8r[20]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[21]);
	 qmp6988_cali.COE_a2 = (QMP6988_S16_t)(((a_data_u8r[22]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[23]);
	
#if 0
	temp_COE.x = (QMP6988_U32_t)((a_data_u8r[0] << \
		SHIFT_LEFT_12_POSITION) | (a_data_u8r[1] << \
		SHIFT_LEFT_4_POSITION) | ((a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION));
	 qmp6988_cali.COE_b00 = temp_COE.x;
#else
	 qmp6988_cali.COE_b00 = (QMP6988_S32_t)((((QMP6988_S32_t)a_data_u8r[0] << SHIFT_LEFT_12_POSITION) \
							| ((QMP6988_S32_t)a_data_u8r[1] << SHIFT_LEFT_4_POSITION) \
							| (((QMP6988_S32_t)a_data_u8r[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))<<12);
	 qmp6988_cali.COE_b00 =  qmp6988_cali.COE_b00>>12;
#endif
	 qmp6988_cali.COE_bt1 = (QMP6988_S16_t)(((a_data_u8r[2]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[3]);
	 qmp6988_cali.COE_bt2 = (QMP6988_S16_t)(((a_data_u8r[4]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[5]);
	 qmp6988_cali.COE_bp1 = (QMP6988_S16_t)(((a_data_u8r[6]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[7]);
	 qmp6988_cali.COE_b11 = (QMP6988_S16_t)(((a_data_u8r[8]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[9]);
	 qmp6988_cali.COE_bp2 = (QMP6988_S16_t)(((a_data_u8r[10]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[11]);
	 qmp6988_cali.COE_b12 = (QMP6988_S16_t)(((a_data_u8r[12]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[13]);		
	 qmp6988_cali.COE_b21 = (QMP6988_S16_t)(((a_data_u8r[14]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[15]);
	 qmp6988_cali.COE_bp3 = (QMP6988_S16_t)(((a_data_u8r[16]) << SHIFT_LEFT_8_POSITION) | a_data_u8r[17]);			

	osLog(LOG_ERROR,"<-----------calibration data-------------->\n");
	osLog(LOG_ERROR,"COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
			 qmp6988_cali.COE_a0, qmp6988_cali.COE_a1, qmp6988_cali.COE_a2, qmp6988_cali.COE_b00);
	osLog(LOG_ERROR,"COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\n",
			 qmp6988_cali.COE_bt1, qmp6988_cali.COE_bt2, qmp6988_cali.COE_bp1, qmp6988_cali.COE_b11);
	osLog(LOG_ERROR,"COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\n",
			 qmp6988_cali.COE_bp2, qmp6988_cali.COE_b12, qmp6988_cali.COE_b21, qmp6988_cali.COE_bp3);
	osLog(LOG_ERROR,"<-----------calibration data-------------->\n");

	
	a0 =  qmp6988_cali.COE_a0 /16.0f;
	b00 =  qmp6988_cali.COE_b00 /16.0f;

	a1 = Conv_A_S[0][0] + Conv_A_S[0][1] * ((float)qmp6988_cali.COE_a1) / 32767.0f;
	a2 = Conv_A_S[1][0] + Conv_A_S[1][1] * ((float)qmp6988_cali.COE_a2) / 32767.0f;
	bt1 = Conv_A_S[2][0] + Conv_A_S[2][1] * ((float)qmp6988_cali.COE_bt1) / 32767.0f;
	bt2 = Conv_A_S[3][0] + Conv_A_S[3][1] * ((float)qmp6988_cali.COE_bt2) / 32767.0f;
	bp1 = Conv_A_S[4][0] + Conv_A_S[4][1] * ((float)qmp6988_cali.COE_bp1) / 32767.0f;
	b11 = Conv_A_S[5][0] + Conv_A_S[5][1] * ((float)qmp6988_cali.COE_b11) / 32767.0f;
	bp2 = Conv_A_S[6][0] + Conv_A_S[6][1] * ((float)qmp6988_cali.COE_bp2) / 32767.0f;
	b12 = Conv_A_S[7][0] + Conv_A_S[7][1] * ((float)qmp6988_cali.COE_b12) / 32767.0f;
	b21 = Conv_A_S[8][0] + Conv_A_S[8][1] * ((float)qmp6988_cali.COE_b21) / 32767.0f;
	bp3 = Conv_A_S[9][0] + Conv_A_S[9][1] *  ((float)qmp6988_cali.COE_bp3) / 32767.0f;
	
//	osLog(LOG_ERROR,"<----------- float calibration data -------------->\n");
//	osLog(LOG_ERROR,"a0[%lle]	a1[%lle]	a2[%lle]	b00[%lle]\n",a0,a1,a2,b00);
//	osLog(LOG_ERROR,"bt1[%lle]	bt2[%lle]	bp1[%lle]	b11[%lle]\n",bt1,bt2,bp1,b11);
//	osLog(LOG_ERROR,"bp2[%lle]	b12[%lle]	b21[%lle]	bp3[%lle]\n",bp2,b12,b21,bp3);
//	osLog(LOG_ERROR,"<----------- float calibration data -------------->\n");
}


static int qmp6988_compParams(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;

    if (mTask.verifyId != QMP6988_CHIP_ID) {
        osLog(LOG_INFO, "qmp6988: not detected\n");
        return -1;
    }
    /* Get compensation parameters */
	mTask.txBuf[0] = QMP6988_CTRLMEAS_REG;
	mTask.txBuf[1] = reg_f4_value;
	ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, (void *)STATE_IDLE);
                       
    mTask.txBuf[0] = QMP6988_CONFIG_REG;
	mTask.txBuf[1] = 0x00;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, (void *)STATE_IDLE);
                       
    mTask.txBuf[0] = QMP6988_IO_SETUP_REG;
	mTask.txBuf[1] = 0x00;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
    //mdelay(15);
    
    return ret;
}

static int qmp6988_config(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;

    mTask.txBuf[0] = QMP6988_CALIBRATION_DATA_START;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[0], 1,
                        (uint8_t*)&cali_reg[0], 6, i2cCallBack,
                        (void *)STATE_IDLE);
    mTask.txBuf[1] = QMP6988_CALIBRATION_DATA_START + 6;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[1], 1,
                        (uint8_t*)&cali_reg[6], 6, i2cCallBack,
                        (void *)STATE_IDLE);
    mTask.txBuf[2] = QMP6988_CALIBRATION_DATA_START + 12;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[2], 1,
                        (uint8_t*)&cali_reg[12], 6, i2cCallBack,
                        (void *)STATE_IDLE);
    mTask.txBuf[3] = QMP6988_CALIBRATION_DATA_START+ 18;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[3], 1,
                        (uint8_t*)&cali_reg[18], 6, i2cCallBack,
                        (void *)STATE_IDLE);
    mTask.txBuf[4] = QMP6988_CALIBRATION_DATA_START+ 24;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, (uint8_t*)&mTask.txBuf[4], 1,
                        (uint8_t*)&cali_reg[24], 1, i2cCallBack,
                        next_state);

    qmp6988_calc_calibration_data(cali_reg);
    {
        int  i;
        for(i=0; i<QMP6988_CALIBRATION_DATA_LENGTH; i++)
        {
            osLog(LOG_ERROR,"cali_reg[%d] = %d\n", i, cali_reg[i]);
        }
    }

    return ret;
}


static int qmp6988_registerCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_BARO;
    mInfo.getCalibration = baroGetCalibration;
    mInfo.setCalibration = baroSetCalibration;
    mInfo.getData = baroGetData;
#if defined(BUILD_ON_ANDROID_P)
    mInfo.getSensorInfo = qmp6988_getSensorInfo;
#endif
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int qmp6988_rate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct baroCntlPacket cntlPacket;
    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "qmp6988 Rate, rx inSize and elemSize error\n");
        return -1;
    }
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static struct sensorFsm qmp6988Fsm[] = {
    /* sample state */
    sensorFsmCmd(STATE_SAMPLE, STATE_CONVERT, qmp6988_sample),
    sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, qmp6988_convert),
    /* enable state */
    sensorFsmCmd(STATE_ENABLE, STATE_POWER_ON, qmp6988_read_config),
    sensorFsmCmd(STATE_POWER_ON, STATE_ENABLE_DONE, qmp6988_power_on),
    /* disable state*/
    sensorFsmCmd(STATE_DISABLE, STATE_POWER_OFF, qmp6988_read_config),
    sensorFsmCmd(STATE_POWER_OFF, STATE_DISABLE_DONE, qmp6988_power_off),

    sensorFsmCmd(STATE_RATECHG, STATE_RATECHG_DONE, qmp6988_rate),

    sensorFsmCmd(STATE_RESET, STATE_CONFIG, qmp6988_compParams),
    sensorFsmCmd(STATE_CONFIG, STATE_CORE, qmp6988_config),
    sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, qmp6988_registerCore),
};

static int qmp6988Init(void)
{
    int ret = 0;
    qmp6988DebugPoint = &mTask;

    osLog(LOG_ERROR, "qmp6988Init\n\r");
    mTask.hw = get_cust_baro("qmp6988");
    if (NULL == mTask.hw) {
        osLog(LOG_ERROR, "qmp6988 get_cust_baro_hw fail\n\r");
        ret = -1;
        goto err_out;
    }

    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    osLog(LOG_ERROR, "i2c_num: %d, i2c_addr: 0x%x\n\r", mTask.hw->i2c_num, mTask.i2c_addr);
    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);
    mTask.txBuf[0] = QMP6988_CHIP_ID_REG;

    for (uint8_t i = 0; i < 3; i++) {
        ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
        (uint8_t *)&mTask.verifyId, 1, NULL, NULL);
        if (ret >= 0)
            break;
    }

    if (ret < 0) {
        ret = -1;
#if defined(BUILD_ON_ANDROID_P)
        sendSensorErrToAp(ERR_SENSOR_BARO, ERR_CASE_BARO_INIT, BARO_NAME);
#endif
        i2cMasterRelease(mTask.hw->i2c_num);
        goto err_out;
    }

    if (mTask.verifyId == QMP6988_CHIP_ID) {
        osLog(LOG_INFO, "qmp6988 auto detect success %x\n", mTask.verifyId);
        goto success_out;
    } else {
        ret = -1;
        osLog(LOG_INFO, "qmp6988 auto detect fail %x\n", mTask.verifyId);
#if defined(BUILD_ON_ANDROID_P)
        sendSensorErrToAp(ERR_SENSOR_BARO, ERR_CASE_BARO_INIT, BARO_NAME);
#endif
        i2cMasterRelease(mTask.hw->i2c_num);
        goto err_out;
    }

success_out:
    baroSensorRegister();
    baroRegisterInterruptMode(BARO_UNINTERRUPTIBLE);
    registerBaroDriverFsm(qmp6988Fsm, ARRAY_SIZE(qmp6988Fsm));
err_out:
    return ret;
}


#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(qmp6988, SENS_TYPE_BARO, qmp6988Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(qmp6988, OVERLAY_WORK_03, qmp6988Init);
#endif

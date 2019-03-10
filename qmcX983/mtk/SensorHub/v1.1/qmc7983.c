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
#include"qmc7983.h"

//#define LOG_ENABLE
#ifdef LOG_ENABLE
#define MSE_LOG(fmt, args...)    osLog(LOG_DEBUG, "[QMCX983]: "fmt, ##args)
#define MSE_ERR(fmt, args...)    osLog(LOG_ERROR, "[QMCX983]: "fmt, ##args)
#else
#define MSE_LOG(fmt, args...)
#define MSE_ERR(fmt, args...)	
#endif
	
#define I2C_SPEED                       400000

uint8_t otpMatrix[3] = {0,0,0};
static short OTP_K[2] = {0,0};

enum qmcX983State {
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
	
    STATE_CHIP_OPTW1 = CHIP_RESET,
    STATE_CHIP_OPTW2,
	STATE_CHIP_OPTW3,
	STATE_CHIP_OTPR1,
	STATE_CHIP_OTPR2,
	STATE_CHIP_OTPR3,
    STATE_CORE,
    STATE_CHIP_CTRL1,
    STATE_CHIP_CTRL2,
    STATE_CHIP_CTRL3,
    STATE_CHIP_CTRL4,
	STATE_CHIP_CTRL5,
};

static struct qmcX983Task {
    /* txBuf for i2c operation, fill register and fill value */
    uint8_t txBuf[8];
    /* rxBuf for i2c operation, receive rawdata */
    uint8_t rxBuf[8];
	
    uint8_t chipId;
	uint8_t verifyId;
	
    uint64_t hwSampleTime;
    struct transferDataInfo dataInfo;
    struct magDataPacket magPacket;
    /* data for factory */
    struct TripleAxisDataPoint factoryData;
    struct mag_hw *hw;
	
    struct sensorDriverConvert cvt;
    uint8_t i2c_addr;
#ifdef CFG_MAG_CALIBRATION_IN_AP	
	struct mag_dev_info_t mag_dev_info;
#endif	
} mTask;

static struct qmcX983Task *qmcX983DebugPoint;

void qmcX983TimerCbkF(uint64_t time)
{
    mTask.hwSampleTime = time;
}
static void magGetCalibration(int32_t *cali, int32_t size)
{
}
static void magSetCalibration(int32_t *cali, int32_t size)
{
}
static void magGetData(void *sample)
{
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mTask.factoryData.ix;
    tripleSample->iy = mTask.factoryData.iy;
    tripleSample->iz = mTask.factoryData.iz;
}

static int qmcX983SetOTP1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x2e;
    mTask.txBuf[1] = 0x0a;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);	
}

static int qmcX983GetOTP1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x2f;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         (uint8_t *)&otpMatrix[0], 1, i2cCallBack,
                         next_state);	
}


static int qmcX983SetOTP2(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x2e;
    mTask.txBuf[1] = 0x0d;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);	
}

static int qmcX983GetOTP2(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x2f;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         (uint8_t *)&otpMatrix[1], 1, i2cCallBack,
                         next_state);	
}


static int qmcX983SetOTP3(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x2e;
    mTask.txBuf[1] = 0x0f;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);	
}

static int qmcX983GetOTP3(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x2f;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         (uint8_t *)&otpMatrix[2], 1, i2cCallBack,
                         next_state);	
}

static int qmcX983ChipCtrl1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x20;
    mTask.txBuf[1] = 0x40;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);		
}

static int qmcX983ChipCtrl2(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x21;
    mTask.txBuf[1] = 0x01;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);		
}

static int qmcX983ChipCtrl3(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x29;
    mTask.txBuf[1] = 0x80;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);		
}

static int qmcX983ChipCtrl4(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = QMCX983_CTL_REG_TWO;
    mTask.txBuf[1] = 0x0c;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);		
}

static int qmcX983ChipCtrl5(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = QMCX983_SR_PERIOD_REG;
    mTask.txBuf[1] = 0x01;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);	
}

static void qmcX983_convert_otp(uint8_t *data)
{
	uint8_t temp; 
	
	if(((data[0]&0x3f) >> 5) == 1)
		OTP_K[0] = (data[0]&0x1f)-32;
	else
		OTP_K[0] = data[0]&0x1f;	
	
	if((data[1] >> 7) == 1)
		OTP_K[1] = (((data[1]&0x70) >> 4)*4 + (data[2] >> 6))-32;
	else
		OTP_K[1] = (((data[1]&0x70) >> 4)*4 + (data[2] >> 6));	
	
	temp = (data[2]&0x3c) >> 2;
	if(0x02 == temp){
		mTask.chipId = QMC7983_Vertical;
	}else if(0x03 == temp){
		mTask.chipId = QMC7983_Slope;
	}	
}

#ifdef CFG_MAG_CALIBRATION_IN_AP
static void magGetSensorInfo(struct sensorInfo_t *data )
{
    memcpy(&data->mag_dev_info, &mTask.mag_dev_info, sizeof(struct mag_dev_info_t));
}
#endif
static int qmcX983RegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
	
	qmcX983_convert_otp(otpMatrix);
	
    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_MAG;
    mInfo.getCalibration = magGetCalibration;
    mInfo.setCalibration = magSetCalibration;
    mInfo.getData = magGetData;
#ifdef CFG_MAG_CALIBRATION_IN_AP	
    mInfo.getSensorInfo = magGetSensorInfo;
#endif	
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int qmcX983Enable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	MSE_LOG( "qmcX983Enable \n");	
	
	if(mTask.chipId == QMC7983_Slope)
	{
		mTask.txBuf[0] = 0x1b;
		mTask.txBuf[1] = 0x80;
		i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,i2cCallBack, (void *)STATE_IDLE);			
	}
    mTask.txBuf[0] = QMCX983_CTL_REG_ONE;
    mTask.txBuf[1] = QMCX983_MODE_200HZ_MEASURE;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);				   
}

static int qmcX983Disable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	MSE_LOG( "qmcX983Disable \n");
	
    mTask.txBuf[0] = QMCX983_CTL_REG_ONE;
    mTask.txBuf[1] = QMCX983_MODE_POWERDOWN;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);			   
}

static int qmcX983Rate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
//    uint32_t sample_rate, water_mark;
    struct magCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        MSE_ERR( "qmcX983Rate, rx inSize and elemSize error\n");
        return -1;
    }
	
 //   sample_rate = cntlPacket.rate;
 //   water_mark = cntlPacket.waterMark;
	
//    MSE_LOG( "qmcX983Rate: %d, water_mark:%d\n", sample_rate, water_mark);
	
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int qmcX983Sample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
	int t1 = 0;
	uint8_t rdy;
	
    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        MSE_ERR( "qmcX983Sample, rx dataInfo error\n");
        return -1;
    }
	while(!(rdy & 0x07) && t1<3){
		mTask.txBuf[0] = QMCX983_STATUS_REG;
		ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, (uint8_t *)&rdy, 1, i2cCallBack, (void *)STATE_IDLE);
		MSE_LOG( "qmcX983 Status Register is (0x%02X)\n",rdy);
		t1 ++;
	}
	
    mTask.txBuf[0] = OUTPUT_X_L_REG;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 6, i2cCallBack,
                         next_state);
}

static int qmcX983Convert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct magData *data = mTask.magPacket.outBuf;

    int32_t remap_data[3];
    int32_t idata[3];
	int32_t hw_data[3] = {0};
    uint64_t timestamp = 0;

    idata[AXIS_X] = (int16_t)((mTask.rxBuf[AXIS_X*2+1] << 8) | (mTask.rxBuf[AXIS_X*2]));
    idata[AXIS_Y] = (int16_t)((mTask.rxBuf[AXIS_Y*2+1] << 8) | (mTask.rxBuf[AXIS_Y*2]));
    idata[AXIS_Z] = (int16_t)((mTask.rxBuf[AXIS_Z*2+1] << 8) | (mTask.rxBuf[AXIS_Z*2]));

	hw_data[AXIS_X] = idata[AXIS_X] * 1000 / QST_8G_SENSITIVITY;
	hw_data[AXIS_Y] = idata[AXIS_Y] * 1000 / QST_8G_SENSITIVITY;
	hw_data[AXIS_Z] = idata[AXIS_Z] * 1000 / QST_8G_SENSITIVITY;
	
	hw_data[AXIS_Z] = hw_data[AXIS_Z] - (int32_t)(hw_data[AXIS_X]*OTP_K[0]*0.02f) - (int32_t)(hw_data[AXIS_Y]*OTP_K[1]*0.02f);

	MSE_LOG( "qmcX983Convert,hw_data: %d, %d, %d \n", hw_data[AXIS_X], hw_data[AXIS_Y], hw_data[AXIS_Z]);
	
    remap_data[AXIS_X] = mTask.cvt.sign[AXIS_X] * hw_data[mTask.cvt.map[AXIS_X]];
    remap_data[AXIS_Y] = mTask.cvt.sign[AXIS_Y] * hw_data[mTask.cvt.map[AXIS_Y]];
    remap_data[AXIS_Z] = mTask.cvt.sign[AXIS_Z] * hw_data[mTask.cvt.map[AXIS_Z]];
	
    data[0].x = remap_data[AXIS_X] / 10.0f;
    data[0].y = remap_data[AXIS_Y] / 10.0f;
    data[0].z = remap_data[AXIS_Z] / 10.0f;
	
    mTask.factoryData.ix = (int32_t)(data[0].x);
    mTask.factoryData.iy = (int32_t)(data[0].y);
    mTask.factoryData.iz = (int32_t)(data[0].z);
	
    MSE_LOG( "qmcX983Convert,factoryData: %d, %d, %d \n",mTask.factoryData.ix,mTask.factoryData.iy,mTask.factoryData.iz);
	
    timestamp = addThenRetreiveAverageMagTimeStamp(mTask.hwSampleTime);
    txTransferDataInfo(&mTask.dataInfo, 1, timestamp, data);
	
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static struct sensorFsm qmcX983Fsm[] = {
    sensorFsmCmd(STATE_SAMPLE, STATE_CONVERT, qmcX983Sample),
    sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, qmcX983Convert),

    sensorFsmCmd(STATE_ENABLE, STATE_ENABLE_DONE, qmcX983Enable),
    sensorFsmCmd(STATE_DISABLE, STATE_DISABLE_DONE, qmcX983Disable),

    sensorFsmCmd(STATE_RATECHG, STATE_RATECHG_DONE, qmcX983Rate),

	/* init state */
    sensorFsmCmd(STATE_CHIP_OPTW1, STATE_CHIP_OTPR1, qmcX983SetOTP1),
    sensorFsmCmd(STATE_CHIP_OTPR1, STATE_CHIP_OPTW2, qmcX983GetOTP1),
	sensorFsmCmd(STATE_CHIP_OPTW2, STATE_CHIP_OTPR2, qmcX983SetOTP2),
	sensorFsmCmd(STATE_CHIP_OTPR2, STATE_CHIP_OPTW3, qmcX983GetOTP2),
	sensorFsmCmd(STATE_CHIP_OPTW3, STATE_CHIP_OTPR3, qmcX983SetOTP3),
	sensorFsmCmd(STATE_CHIP_OTPR3, STATE_CHIP_CTRL1, qmcX983GetOTP3),
    sensorFsmCmd(STATE_CHIP_CTRL1, STATE_CHIP_CTRL2, qmcX983ChipCtrl1),
	sensorFsmCmd(STATE_CHIP_CTRL2, STATE_CHIP_CTRL3, qmcX983ChipCtrl2),
	sensorFsmCmd(STATE_CHIP_CTRL3, STATE_CHIP_CTRL4, qmcX983ChipCtrl3),
	sensorFsmCmd(STATE_CHIP_CTRL4, STATE_CHIP_CTRL5, qmcX983ChipCtrl4),
	sensorFsmCmd(STATE_CHIP_CTRL5, STATE_CORE, qmcX983ChipCtrl5),
	sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, qmcX983RegisterCore),
};
#ifndef CFG_MAG_CALIBRATION_IN_AP
static int qmcX983CaliApiGetOffset(float offset[AXES_NUM])
{
    return 0;
}
static int qmcX983CaliApiSetOffset(float offset[AXES_NUM])
{
    return 0;
}
static int qmcX983CaliApiSetGyroData(struct magCaliDataInPut *inputData)
{
    return 0;
}

static int qmcX983DoCaliAPI(struct magCaliDataInPut *inputData,
        struct magCaliDataOutPut *outputData)
{
    int8_t accuracy = 0;
    float mag_CaliData[3] = {0.0f};
    float magData[3] = {0.0f};
	
	magData[0] = inputData->x;
	magData[1] = inputData->y;
	magData[2] = inputData->z;
	
	convert_magnetic(magData, mag_CaliData, &accuracy);
	
	outputData->x = mag_CaliData[AXIS_X];
	outputData->y = mag_CaliData[AXIS_Y];
	outputData->z = mag_CaliData[AXIS_Z];
	outputData->x_bias = magData[AXIS_X] - mag_CaliData[AXIS_X];
	outputData->y_bias = magData[AXIS_Y] - mag_CaliData[AXIS_Y];
	outputData->z_bias = magData[AXIS_Z] - mag_CaliData[AXIS_Z];
	outputData->status = accuracy;

	return 0;
}

static struct magCalibrationLibAPI qmcX983CaliAPI = {
    .caliApiGetOffset = qmcX983CaliApiGetOffset,
    .caliApiSetOffset = qmcX983CaliApiSetOffset,
    .caliApiSetGyroData = qmcX983CaliApiSetGyroData,
    .doCaliApi = qmcX983DoCaliAPI
};
#endif

int qmcX983Init(void)
{
    int ret = 0;
	
    qmcX983DebugPoint = &mTask;
    insertMagicNum(&mTask.magPacket);
    mTask.hw = get_cust_mag("qmc7983");
    if (NULL == mTask.hw) {
        MSE_ERR( "qmc7983 get_cust_mag_hw fail\n");
		ret = -1;
       goto err_out;
    }
    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    MSE_LOG( "qmcX983Init,mag i2c_num: %d, i2c_addr: 0x%x\n", mTask.hw->i2c_num, mTask.i2c_addr);

    if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt))) {
        MSE_ERR( "invalid direction: %d\n", mTask.hw->direction);
    }
	
    MSE_LOG( "qmcX983Init,mag map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
        mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z],
        mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);
 
    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);
	
    mTask.txBuf[0] = QMCX983_DEVICE_ID_REG;
	ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
		(uint8_t *)&mTask.verifyId, 1, NULL, NULL);
    if (ret < 0) {
		MSE_ERR( "qmcX983Init,qmcX983 Read Chip_ID fail!\n");
		ret = -1;
		i2cMasterRelease(mTask.hw->i2c_num);
		goto err_out;    
	}

	if(0x32 == mTask.verifyId)
	{
		MSE_LOG("qmcX983 detected successfully\n");
		goto success_out;
	}else{
        i2cMasterRelease(mTask.hw->i2c_num);
		MSE_ERR("qmcX983Init,qst unkonwn ID\n");
		ret = -1;
		goto err_out;		
	}
success_out:	
	magSensorRegister();
    magRegisterInterruptMode(MAG_UNFIFO);
    registerMagDriverFsm(qmcX983Fsm, ARRAY_SIZE(qmcX983Fsm));
    registerMagTimerCbk(qmcX983TimerCbkF);
#ifndef CFG_MAG_CALIBRATION_IN_AP
	registerMagCaliAPI(&qmcX983CaliAPI);	
	qst_ical_init();
#else
	mTask.mag_dev_info.layout = mTask.hw->direction;
    mTask.mag_dev_info.deviceid = mTask.verifyId;
    strncpy(mTask.mag_dev_info.libname, "QstAlgo", sizeof(mTask.mag_dev_info.libname));		
#endif

    MSE_LOG("qmcX983Init, auto detect success,ret = %d\n",ret);
err_out:
	return ret;
	
}
#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(qmc7983, SENS_TYPE_MAG, qmcX983Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(qmc7983, OVERLAY_WORK_01, qmcX983Init);	
#endif
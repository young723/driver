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
#include"qmc6308.h"

#define QST_MAG_NAME	"qmc6308"

//#define LOG_ENABLE
#ifdef LOG_ENABLE
#define MSE_LOG(fmt, args...)    osLog(LOG_ERROR, "[QMC6308]: "fmt, ##args)
#define MSE_ERR(fmt, args...)    osLog(LOG_ERROR, "[QMC6308]: "fmt, ##args)
#else
#define MSE_LOG(fmt, args...)
#define MSE_ERR(fmt, args...)	
#endif
	
#define I2C_SPEED                       400000

SENSOR_MASK mask = {
	.mask0 = 0x80,
	.mask1 = 0xA0,
	.mask2 = 0xB0,
	.mask3 = 0xC0,
};

enum qmc6308State {
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
	
    STATE_CHIP_CTRL1 = CHIP_RESET,
    STATE_CHIP_CTRL2,
	STATE_CORE,
};

static struct qmc6308Task {
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
	struct mag_dev_info_t mag_dev_info;
} mTask;

static struct qmc6308Task *qmc6308DebugPoint;

void qmc6308TimerCbkF(uint64_t time)
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

static int qmc6308ChipCtrl1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = 0x0d;
    mTask.txBuf[1] = 0x40;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);		
}

static int qmc6308ChipCtrl2(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = QMC6308_CTL_REG_TWO;
    mTask.txBuf[1] = 0x08;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);		
}

static void magGetSensorInfo(struct sensorInfo_t *data )
{
    memcpy(&data->mag_dev_info, &mTask.mag_dev_info, sizeof(struct mag_dev_info_t));
}

static int qmc6308RegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));

    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_MAG;
    mInfo.getCalibration = magGetCalibration;
    mInfo.setCalibration = magSetCalibration;
    mInfo.getData = magGetData;
    mInfo.getSensorInfo = magGetSensorInfo;
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int qmc6308Enable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	MSE_LOG( "qmc6308Enable \n");	

    mTask.txBuf[0] = QMC6308_CTL_REG_ONE;
    mTask.txBuf[1] = 0x63;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);				   
}

static int qmc6308Disable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	MSE_LOG( "qmc6308Disable \n");
	
	mTask.txBuf[0] = QMC6308_CTL_REG_ONE;
	i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, (uint8_t *)&mTask.txBuf[1], 1, i2cCallBack, (void *)STATE_IDLE);		
		
    mTask.txBuf[0] = QMC6308_CTL_REG_ONE;
    mTask.txBuf[1] = (mTask.txBuf[1]& ~0x03) | QMC6308_SUSPEND_MODE;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);			   
}

static int qmc6308Rate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
//    uint32_t sample_rate, water_mark;
    struct magCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        MSE_ERR( "qmc6308Rate, rx inSize and elemSize error\n");
        return -1;
    }
	
 //   sample_rate = cntlPacket.rate;
 //   water_mark = cntlPacket.waterMark;
	
//    MSE_LOG( "qmc6308Rate: %d, water_mark:%d\n", sample_rate, water_mark);
	
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int qmc6308Sample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
	int t1 = 0;
	uint8_t rdy = 0;
	
    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        MSE_ERR( "qmc6308Sample, rx dataInfo error\n");
        return -1;
    }
	while(!(rdy & 0x03) && t1<3){
		mTask.txBuf[0] = QMC6308_STATUS_REG;
		ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, (uint8_t *)&rdy, 1, i2cCallBack, (void *)STATE_IDLE);
		MSE_LOG( "qmc6308 Status Register is (0x%02X)\n",rdy);
		t1 ++;
	}
	
    mTask.txBuf[0] = QMC6308_DATA_OUT_X_LSB_REG;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 6, i2cCallBack,
                         next_state);
}

static int qmc6308Convert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct magData *data = mTask.magPacket.outBuf;

    int32_t remap_data[3];
    int32_t idata[3];
    uint64_t timestamp = 0;

    idata[AXIS_X] = (int16_t)((mTask.rxBuf[AXIS_X*2+1] << 8) | (mTask.rxBuf[AXIS_X*2]));
    idata[AXIS_Y] = (int16_t)((mTask.rxBuf[AXIS_Y*2+1] << 8) | (mTask.rxBuf[AXIS_Y*2]));
    idata[AXIS_Z] = (int16_t)((mTask.rxBuf[AXIS_Z*2+1] << 8) | (mTask.rxBuf[AXIS_Z*2]));

	MSE_LOG( "qmc6308Convert,hw_data: %d, %d, %d \n", idata[AXIS_X], idata[AXIS_Y], idata[AXIS_Z]);
	
    remap_data[AXIS_X] = mTask.cvt.sign[AXIS_X] * idata[mTask.cvt.map[AXIS_X]];
    remap_data[AXIS_Y] = mTask.cvt.sign[AXIS_Y] * idata[mTask.cvt.map[AXIS_Y]];
    remap_data[AXIS_Z] = mTask.cvt.sign[AXIS_Z] * idata[mTask.cvt.map[AXIS_Z]];
	
    data[0].x = remap_data[AXIS_X] * 0.1f;
    data[0].y = remap_data[AXIS_Y] * 0.1f;
    data[0].z = remap_data[AXIS_Z] * 0.1f;
	
    mTask.factoryData.ix = (int32_t)(data[0].x);
    mTask.factoryData.iy = (int32_t)(data[0].y);
    mTask.factoryData.iz = (int32_t)(data[0].z);
	
    MSE_LOG( "qmc6308Convert,factoryData: %d, %d, %d \n",mTask.factoryData.ix,mTask.factoryData.iy,mTask.factoryData.iz);
	
    timestamp = addThenRetreiveAverageMagTimeStamp(mTask.hwSampleTime);
    txTransferDataInfo(&mTask.dataInfo, 1, timestamp, data);
	
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static struct sensorFsm qmc6308Fsm[] = {
    sensorFsmCmd(STATE_SAMPLE, STATE_CONVERT, qmc6308Sample),
    sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, qmc6308Convert),

    sensorFsmCmd(STATE_ENABLE, STATE_ENABLE_DONE, qmc6308Enable),
    sensorFsmCmd(STATE_DISABLE, STATE_DISABLE_DONE, qmc6308Disable),

    sensorFsmCmd(STATE_RATECHG, STATE_RATECHG_DONE, qmc6308Rate),

	/* init state */
    sensorFsmCmd(STATE_CHIP_CTRL1, STATE_CHIP_CTRL2, qmc6308ChipCtrl1),
	sensorFsmCmd(STATE_CHIP_CTRL2, STATE_CORE, qmc6308ChipCtrl2),
	sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, qmc6308RegisterCore),
};
#ifndef CFG_MAG_CALIBRATION_IN_AP
static int qmc6308CaliInitLib(int hwGyroSupported)
{
	return 0;
}

static int qmc6308CaliApiGetOffset(float offset[AXES_NUM])
{
    return 0;
}
static int qmc6308CaliApiSetOffset(float offset[AXES_NUM])
{
    return 0;
}

static int qmc6308CaliApiGetCaliParam(int32_t caliParameter[6])
{
    return 0;
}
static int qmc6308CaliApiSetCaliParam(int32_t caliParameter[6])
{
    return 0;
}

static int qmc6308CaliApiSetGyroData(struct magCaliDataInPut *inputData)
{
    return 0;
}

static int qmc6308DoCaliAPI(struct magCaliDataInPut *inputData,
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

static struct magCalibrationLibAPI qmc6308CaliAPI = {	
	.initLib = qmc6308CaliInitLib,
    .caliApiGetOffset = qmc6308CaliApiGetOffset,
    .caliApiSetOffset = qmc6308CaliApiSetOffset,
    .caliApiGetCaliParam = qmc6308CaliApiGetCaliParam,
    .caliApiSetCaliParam = qmc6308CaliApiSetCaliParam,
    .caliApiSetGyroData = qmc6308CaliApiSetGyroData,
    .doCaliApi = qmc6308DoCaliAPI
};
#endif

int qmc6308Init(void)
{
    int ret = 0;
	
    qmc6308DebugPoint = &mTask;
    insertMagicNum(&mTask.magPacket);
    mTask.hw = get_cust_mag("qmc6308");
    if (NULL == mTask.hw) {
        MSE_ERR( "qmc6308 get_cust_mag_hw fail\n");
		ret = -1;
       goto err_out;
    }
    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    MSE_LOG( "qst qmc6308Init,mag i2c_num: %d, i2c_addr: 0x%x\n", mTask.hw->i2c_num, mTask.i2c_addr);

    if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt))) {
        MSE_ERR( "invalid direction: %d\n", mTask.hw->direction);
    }
	
    MSE_LOG( "qmc6308Init,mag map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
        mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z],
        mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);
 
    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);
	
    mTask.txBuf[0] = QMC6308_CHIP_ID_REG;
	ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
		(uint8_t *)&mTask.verifyId, 1, NULL, NULL);
    if (ret < 0) {
		MSE_ERR( "qmc6308Init,qmc6308 Read Chip_ID fail!\n");
		ret = -1;
		i2cMasterRelease(mTask.hw->i2c_num);		
        sendSensorErrToAp(ERR_SENSOR_MAG, ERR_CASE_MAG_INIT, QST_MAG_NAME);
		goto err_out;    
	}

	if((mTask.verifyId & mask.mask1) != mask.mask1 &&
		(mTask.verifyId & mask.mask2) != mask.mask2 &&
		(mTask.verifyId & mask.mask3) != mask.mask3 &&
		(mTask.verifyId & mask.mask0) != mask.mask0)
	{
		i2cMasterRelease(mTask.hw->i2c_num);
        sendSensorErrToAp(ERR_SENSOR_MAG, ERR_CASE_MAG_INIT, QST_MAG_NAME);		
		MSE_ERR("qmc6308Init,qst unkonwn ID\n");
		ret = -1;
		goto err_out;
	}else{
		MSE_LOG("qmc6308 detected successfully\n");
		goto success_out;		
	}
success_out:
	mTask.mag_dev_info.layout = mTask.hw->direction;
    mTask.mag_dev_info.deviceid = mTask.verifyId;
    strncpy(mTask.mag_dev_info.libname, "QstAlgo", sizeof(mTask.mag_dev_info.libname));
	
	magSensorRegister();
    magRegisterInterruptMode(MAG_UNFIFO);
    registerMagDriverFsm(qmc6308Fsm, ARRAY_SIZE(qmc6308Fsm));
    registerMagTimerCbk(qmc6308TimerCbkF);
#ifndef CFG_MAG_CALIBRATION_IN_AP
	registerMagCaliAPI(&qmc6308CaliAPI);	
	qst_ical_init();
#endif

    MSE_LOG("qmc6308Init, auto detect success,ret = %d\n",ret);
err_out:
	return ret;
	
}
#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(qmc6308, SENS_TYPE_MAG, qmc6308Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(qmc6308, OVERLAY_ID_MAG, qmc6308Init);	
#endif

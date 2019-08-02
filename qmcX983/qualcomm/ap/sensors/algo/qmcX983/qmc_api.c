#include <hardware/sensors.h>
#include <string.h>
#include "qmc_api.h"
#include "qmcLog.h"
#define UT2MG 10
#define QMC_FORMAT		3.125
#define QMCFORMAT2ANDROID (1.f/(UT2MG*QMC_FORMAT))
#define FILTERNUM       10
#define PI				3.1415926
#define DEG2RAD			( PI / 180.0)

static sensors_event_t mAccelEvent;
static sensors_event_t mMagneticEvent;
static sensors_event_t mCaliMagEvent;
static sensors_event_t mOrientationEvent;

static float qmc_RV[4];
static float qmc_gyro[3];
static float qmc_LA[3];
static float qmc_GR[3];
#define ID_A  (0)
#define ID_M  (1)
#define ID_O  (2)

float MagsumX = 0;
float fMagX[FILTERNUM] = { 0 };
float MagsumY = 0;
float fMagY[FILTERNUM] = { 0 };
float MagsumZ = 0;
float fMagZ[5*FILTERNUM] = { 0 };

float AccsumX = 0;
float fAccX[FILTERNUM] = { 0 };
float AccsumY = 0;
float fAccY[FILTERNUM] = { 0 };
float AccsumZ = 0;
float fAccZ[FILTERNUM] = { 0 };

void Filter_Acc(sensors_event_t* acc)
{
	int i=0;
	AccsumX = 0;
	AccsumY = 0;
	AccsumZ = 0;
	for( i=0;i<FILTERNUM-1;i++)
	{
		fAccX[i] = fAccX[i+1];
		fAccY[i] = fAccY[i+1];
		fAccZ[i] = fAccZ[i+1];
	}

	fAccX[FILTERNUM-1] = acc->data[0];
	fAccY[FILTERNUM-1] = acc->data[1];
	fAccZ[FILTERNUM-1] = acc->data[2];

	for( i=0;i<FILTERNUM;i++)
	{
		AccsumX += fAccX[i];
		AccsumY += fAccY[i];
		AccsumZ += fAccZ[i];
	}
	acc->data[0] = AccsumX / FILTERNUM;
	acc->data[1] = AccsumY / FILTERNUM;
	acc->data[2] = AccsumZ / FILTERNUM;

}

void Filter_Mag(sensors_event_t* mag)
{

	int i=0;

	MagsumX = 0;
	MagsumY = 0;
	MagsumZ = 0;
	for( i=0;i<FILTERNUM-1;i++)
	{
		fMagX[i] = fMagX[i+1];
		fMagY[i] = fMagY[i+1];
	}

	for (i = 0; i < (FILTERNUM * 5 - 1); i++)
		fMagZ[i] = fMagZ[i + 1];

	fMagX[FILTERNUM - 1] = mag->data[0];
	fMagY[FILTERNUM - 1] = mag->data[1];
	fMagZ[FILTERNUM * 5 - 1] = mag->data[2];

	for( i=0;i<FILTERNUM;i++)
	{
		MagsumX += fMagX[i];
		MagsumY += fMagY[i];
	}
	for (i = 0; i < FILTERNUM * 5; i++)
		MagsumZ += fMagZ[i];

	mag->data[0] = MagsumX / FILTERNUM;
	mag->data[1] = MagsumY / FILTERNUM;
	mag->data[2] = MagsumZ / FILTERNUM / 5;

}

void QMC_Init(void)
{
	mAccelEvent.version = sizeof(sensors_event_t);
	mAccelEvent.sensor = ID_A;
	mAccelEvent.type = SENSOR_TYPE_ACCELEROMETER;
	mAccelEvent.acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
	memset(mAccelEvent.data, 0, sizeof(mAccelEvent.data));

	mMagneticEvent.version = sizeof(sensors_event_t);
	mMagneticEvent.sensor = ID_M;
	mMagneticEvent.type = SENSOR_TYPE_MAGNETIC_FIELD;
	mMagneticEvent.magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
	memset(mMagneticEvent.data, 0, sizeof(mMagneticEvent.data));

	mOrientationEvent.version = sizeof(sensors_event_t);
	mOrientationEvent.sensor = ID_O;
	mOrientationEvent.type = SENSOR_TYPE_ORIENTATION;
	mOrientationEvent.magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
	memset(mOrientationEvent.data, 0, sizeof(mOrientationEvent.data));
	mcal(0);
#ifdef QST_VIRTUAL_SENSORS
	dummyGyroInit();
#endif
}


void QMC_CalibMagProcess(sensors_event_t* raw,sensors_event_t* cali)
{
#ifdef QST_VIRTUAL_SENSORS
	float qst_fusionMag[3];
	float qst_fusionAcc[3];
	int pg_out[3];
	float qst_fusionRes[10];
#endif
	mMagneticEvent.magnetic.x = raw->magnetic.x * QMC_FORMAT * UT2MG;
	mMagneticEvent.magnetic.y = raw->magnetic.y * QMC_FORMAT * UT2MG;
	mMagneticEvent.magnetic.z = raw->magnetic.z * QMC_FORMAT * UT2MG;
	mMagneticEvent.timestamp = raw->timestamp;
	process(&mMagneticEvent);
	mCaliMagEvent.magnetic.x = mMagneticEvent.magnetic.x;
	mCaliMagEvent.magnetic.y = mMagneticEvent.magnetic.y;
	mCaliMagEvent.magnetic.z = mMagneticEvent.magnetic.z;
	mCaliMagEvent.magnetic.status = mMagneticEvent.magnetic.status;
	cali->magnetic.x = mCaliMagEvent.magnetic.x * QMCFORMAT2ANDROID;
	cali->magnetic.y = mCaliMagEvent.magnetic.y * QMCFORMAT2ANDROID;
	cali->magnetic.z = mCaliMagEvent.magnetic.z * QMCFORMAT2ANDROID;
	cali->magnetic.status = mCaliMagEvent.magnetic.status;
#ifdef QST_VIRTUAL_SENSORS
	qst_fusionMag[0] = mCaliMagEvent.magnetic.x;
	qst_fusionMag[1] = mCaliMagEvent.magnetic.y;
	qst_fusionMag[2] = mCaliMagEvent.magnetic.z;
	qst_fusionAcc[0] = mAccelEvent.acceleration.x;
	qst_fusionAcc[1] = mAccelEvent.acceleration.y;
	qst_fusionAcc[2] = mAccelEvent.acceleration.z;
	process_PG(qst_fusionMag,qst_fusionAcc,pg_out);
	QST_fusion(mMagneticEvent.data, mAccelEvent.data, qst_fusionRes);
	qmc_gyro[0] = (float)pg_out[0] * DEG2RAD;
	qmc_gyro[1] = (float)pg_out[1] * DEG2RAD;
	qmc_gyro[2] = (float)pg_out[2] * DEG2RAD;
	ALOGE("gyro,%f,%f,%f",qmc_gyro[0],qmc_gyro[1],qmc_gyro[2]);
	qmc_RV[0] = qst_fusionRes[0];
	qmc_RV[1] = qst_fusionRes[1];
	qmc_RV[2] = qst_fusionRes[2];
	qmc_RV[3] = qst_fusionRes[9];
	ALOGE("rotation,%f,%f,%f,%f",qmc_RV[0],qmc_RV[1],qmc_RV[2],qmc_RV[3]);
	qmc_GR[0] = qst_fusionRes[3];
	qmc_GR[1] = qst_fusionRes[4];
	qmc_GR[2] = qst_fusionRes[5];
	ALOGE("GR,%f,%f,%f",qmc_GR[0],qmc_GR[1],qmc_GR[2]);
	qmc_LA[0] = qst_fusionRes[6];
	qmc_LA[1] = qst_fusionRes[7];
	qmc_LA[2] = qst_fusionRes[8];
	ALOGE("LA,%f,%f,%f",qmc_LA[0],qmc_LA[1],qmc_LA[2]);
#endif
	Filter_Acc(&mAccelEvent);
	Filter_Mag(&mMagneticEvent);
	push2mcal(&mAccelEvent, 1);
	push2mcal(&mMagneticEvent, 1);
	get(&mOrientationEvent);
}

void QMC_ORIProcess(sensors_event_t *sensor, sensors_event_t *ori)
{
	if(sensor->type == SENSOR_TYPE_ACCELEROMETER) 
	{
		mAccelEvent.acceleration.x = sensor->acceleration.x;
		mAccelEvent.acceleration.y = sensor->acceleration.y;
		mAccelEvent.acceleration.z = sensor->acceleration.z;
	}
	if(sensor->type == SENSOR_TYPE_MAGNETIC_FIELD) 
	{
		ALOGE("QMC_ORI,%f,%f,%f,%d",sensor->magnetic.x,sensor->magnetic.y,
			 sensor->magnetic.z,sensor->magnetic.status);
	}
	*ori = mOrientationEvent;
}

void QMC_GyroProcess(sensors_event_t *sensor, sensors_event_t *pg)
{
	if(sensor->type == SENSOR_TYPE_ACCELEROMETER) 
	{
		mAccelEvent.acceleration.x = sensor->acceleration.x;
		mAccelEvent.acceleration.y = sensor->acceleration.y;
		mAccelEvent.acceleration.z = sensor->acceleration.z;
	}
	if(sensor->type == SENSOR_TYPE_MAGNETIC_FIELD) 
	{

	}
	pg->gyro.x = qmc_gyro[0];
	pg->gyro.y = qmc_gyro[1];
	pg->gyro.z = qmc_gyro[2];
	pg->gyro.status = mCaliMagEvent.magnetic.status;
}

void QMC_RVProcess(sensors_event_t *sensor, sensors_event_t *rv)
{
	if(sensor->type == SENSOR_TYPE_ACCELEROMETER) 
	{
		mAccelEvent.acceleration.x = sensor->acceleration.x;
		mAccelEvent.acceleration.y = sensor->acceleration.y;
		mAccelEvent.acceleration.z = sensor->acceleration.z;
	}
	if(sensor->type == SENSOR_TYPE_MAGNETIC_FIELD) 
	{

	}
	rv->data[0] = qmc_RV[0];
	rv->data[1] = qmc_RV[1];
	rv->data[2] = qmc_RV[2];
	rv->data[3] = qmc_RV[3];
}

void QMC_GRAProcess(sensors_event_t *sensor, sensors_event_t *ga)
{
	if(sensor->type == SENSOR_TYPE_ACCELEROMETER) 
	{
		mAccelEvent.acceleration.x = sensor->acceleration.x;
		mAccelEvent.acceleration.y = sensor->acceleration.y;
		mAccelEvent.acceleration.z = sensor->acceleration.z;
	}
	if(sensor->type == SENSOR_TYPE_MAGNETIC_FIELD) 
	{

	}
	ga->data[0] = qmc_GR[0];
	ga->data[1] = qmc_GR[1];
	ga->data[2] = qmc_GR[2];
	ga->data[3] = mCaliMagEvent.magnetic.status;
}

void QMC_LAProcess(sensors_event_t *sensor, sensors_event_t *la)
{
	if(sensor->type == SENSOR_TYPE_ACCELEROMETER) 
	{
		mAccelEvent.acceleration.x = sensor->acceleration.x;
		mAccelEvent.acceleration.y = sensor->acceleration.y;
		mAccelEvent.acceleration.z = sensor->acceleration.z;
	}
	if(sensor->type == SENSOR_TYPE_MAGNETIC_FIELD) 
	{

	}
	la->data[0] = qmc_LA[0];
	la->data[1] = qmc_LA[1];
	la->data[2] = qmc_LA[2];
	la->data[3] = mCaliMagEvent.magnetic.status;
}

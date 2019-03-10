#include "QMCD_API.h"

#define ACC_FILTERNUM       10
#define MAG_FILTERNUM       50

float MagsumX = 0;
float fMagX[MAG_FILTERNUM] = { 0 };
float MagsumY = 0;
float fMagY[MAG_FILTERNUM] = { 0 };
float MagsumZ = 0;
float fMagZ[MAG_FILTERNUM] = { 0 };

float AccsumX = 0;
float fAccX[ACC_FILTERNUM] = { 0 };
float AccsumY = 0;
float fAccY[ACC_FILTERNUM] = { 0 };
float AccsumZ = 0;
float fAccZ[ACC_FILTERNUM] = { 0 };

void Filter_Acc(sensors_event_t* acc)
{
	int i=0;
	AccsumX = 0;
	AccsumY = 0;
	AccsumZ = 0;
	for(i = 0;i < ACC_FILTERNUM-1;i++)
	{
		fAccX[i] = fAccX[i+1];
		fAccY[i] = fAccY[i+1];
		fAccZ[i] = fAccZ[i+1];
	}

	fAccX[ACC_FILTERNUM-1] = acc->data[0];
	fAccY[ACC_FILTERNUM-1] = acc->data[1];
	fAccZ[ACC_FILTERNUM-1] = acc->data[2];

	for( i = 0;i < ACC_FILTERNUM;i++)
	{
		AccsumX += fAccX[i];
		AccsumY += fAccY[i];
		AccsumZ += fAccZ[i];
	}
	acc->data[0] = AccsumX / ACC_FILTERNUM;
	acc->data[1] = AccsumY / ACC_FILTERNUM;
	acc->data[2] = AccsumZ / ACC_FILTERNUM;
}

void Filter_Mag(sensors_event_t* mag)
{
	int i=0;
	
	MagsumX = 0;
	MagsumY = 0;
	MagsumZ = 0;
	for(i = 0;i < MAG_FILTERNUM-1;i++)
	{
		fMagX[i] = fMagX[i+1];
		fMagY[i] = fMagY[i+1];
		fMagZ[i] = fMagZ[i+1];
	}

	fMagX[MAG_FILTERNUM - 1] = mag->data[0];
	fMagY[MAG_FILTERNUM - 1] = mag->data[1];
	fMagZ[MAG_FILTERNUM - 1] = mag->data[2];

	for( i = 0;i < MAG_FILTERNUM;i++)
	{
		MagsumX += fMagX[i];
		MagsumY += fMagY[i];
		MagsumZ += fMagZ[i];
	}

	mag->data[0] = MagsumX / MAG_FILTERNUM;
	mag->data[1] = MagsumY / MAG_FILTERNUM;
	mag->data[2] = MagsumZ / MAG_FILTERNUM;
}

int QMCD_GetSensorsData(sensors_event_t *raw_h, sensors_event_t *acc,
					 sensors_event_t *pg, sensors_event_t *rv ,
					 sensors_event_t *ori, sensors_event_t *ga,
					 sensors_event_t *la){
						 
	static sensors_event_t mOrientationEvent;

#ifdef QST_VIRTUAL_SENSOR
	float qst_fusionMag_pg[3] = {0};
	float qst_fusionAcc_pg[3] = {0};
	int pg_out[3]; //output for pg

	float qst_fusionMag[3] = {0};
	float qst_fusionAcc[3] = {0};
	float qst_fusionRes[10]={0}; //rv 4 elements
	
	qst_fusionMag_pg[0] = raw_h->magnetic.x;
	qst_fusionMag_pg[1] = raw_h->magnetic.y;
	qst_fusionMag_pg[2] = raw_h->magnetic.z;
	qst_fusionAcc_pg[0] = acc->acceleration.x;
	qst_fusionAcc_pg[1] = acc->acceleration.y;
	qst_fusionAcc_pg[2] = acc->acceleration.z;
	//gyro  Algorithm interface  
	QST_fussion_PG(qst_fusionMag_pg,qst_fusionAcc_pg, pg_out);
	D("QST_PseudoGyro: [%d %d %d] \n",pg_out[0],pg_out[1],pg_out[2]);

	qst_fusionMag[0] = raw_h->magnetic.x;
	qst_fusionMag[1] = raw_h->magnetic.y;
	qst_fusionMag[2] = raw_h->magnetic.z;
	qst_fusionAcc[0] = acc->acceleration.x;
	qst_fusionAcc[1] = acc->acceleration.y;
	qst_fusionAcc[2] = acc->acceleration.z;
	//rotation vector/gravity/linear acceleration  Algorithm interface 
	QST_fusion(qst_fusionMag,qst_fusionAcc,qst_fusionRes);
	D("QST_VirtualSensors: RV[%f  %f  %f  %f],Gravity[%f  %f  %f  ],LA[%f  %f  %f  ]\n",
	qst_fusionRes[0],qst_fusionRes[1],qst_fusionRes[2],qst_fusionRes[9],
	qst_fusionRes[3],qst_fusionRes[4],qst_fusionRes[5],
	qst_fusionRes[6],qst_fusionRes[7],qst_fusionRes[8]);
#endif

	Filter_Acc(acc);

	Filter_Mag(raw_h);

	//compute pitch/roll
	push2mcal(acc, 1);
	//compute orientation
	push2mcal(raw_h, 1);
	//get orientation
	get(&mOrientationEvent);				 
	
	/* Gyroscope */
	if(pg){
		pg->gyro.x = pg_out[0];
		pg->gyro.y = pg_out[1];
		pg->gyro.z = pg_out[2];
		pg->gyro.status = raw_h->magnetic.status;		
	}
	/* Orientation */
	if(ori) {
		ori->orientation.x = mOrientationEvent.orientation.x;	/* Azimuth */
		ori->orientation.y = mOrientationEvent.orientation.y; /* Pitch */
		ori->orientation.z = mOrientationEvent.orientation.z;	/* Roll */
		ori->orientation.status = mOrientationEvent.orientation.status;
	}

	/* Rotation Vector */
	if(rv) {
		rv->data[0] = qst_fusionRes[0];
		rv->data[1] = qst_fusionRes[1];
		rv->data[2] = qst_fusionRes[2];
		rv->data[3] = qst_fusionRes[9];
		rv->data[4] = raw_h->magnetic.status;
	}

	/* Gravity */
	if(ga) {
		ga->data[0] = qst_fusionRes[3];
		ga->data[1] = qst_fusionRes[4];
		ga->data[2] = qst_fusionRes[5];
		ga->data[3] = 3;
	}

	/* Linear Accelerometer */
	if(la) {
		la->data[0] = qst_fusionRes[6];
		la->data[1] = qst_fusionRes[7];
		la->data[2] = qst_fusionRes[8];
		la->data[3] = 3;
	}
	
	return 0;
}

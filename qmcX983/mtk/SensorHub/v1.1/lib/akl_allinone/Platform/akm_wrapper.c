
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include "akm_apis.h"
#include "akm_wrapper.h"

#define AKM_ALGO_9D 0x84 //AKL_MODE_FUSION_9D | AKL_MODE_CALIB_DOEAG
#define AKM_ALGO_6D 0x22 //AKL_MODE_FUSION_6D_PG_ON | AKL_MODE_CALIB_DOEEX

#define PI     ((float)3.14159265358979323846264338)

#define RAD2DEG             (180.f/PI)
#define DEG2PAD             (PI/180.f)

#define Q16_TO_STD(x)       (x / 65536.f)
#define STD_TO_Q16(x)       (x * 65536.f)

static int init_flag = 0;
uint8_t akm_wia[2] = {0};
uint32_t akm_hasgyro = 0;
void AKM_SetDevice(uint8_t *wia) {
    akm_wia[0] = wia[0];    // akm eric modify for mult lib
    akm_wia[1] = wia[1];    // akm eric modify for mult lib
    // printf("AKM_SetDevice %x,%x\n",akm_wia[0],akm_wia[1]);
}

int AKM_ConfigMode(void)
{
    uint32_t akm_cali_mode = 0;
    uint32_t akm_fusion_mode = 0;
    if (akm_hasgyro)
    {
        akm_cali_mode = AKL_MODE_CALIB_DOEAG;
        akm_fusion_mode = AKL_MODE_FUSION_9D;
    }
    else
    {
        akm_cali_mode = AKL_MODE_CALIB_DOEEX;
        akm_fusion_mode = AKL_MODE_FUSION_6D_PG_ON;
    }
    return (akm_fusion_mode | akm_cali_mode);
}
int AKM_Open(void)
{
    int ret = 0;
    uint32_t akm_mode = 0;
    if (init_flag == 0) {
        akm_mode = AKM_ConfigMode();
        //AKM_LOG_DBG("AKM_Open, akm_mode:%x\n",akm_mode);
        ret = AKM_LibraryInit(akm_wia[1], 0, akm_mode);
        ret = AKM_LoadAndStart();
        if(ret == AKM_SUCCESS)
            init_flag = 1;
    }
    return ret;
}
int AKM_SetGyroFlag(int32_t hasGyro)
{
    akm_hasgyro = hasGyro;
    if (init_flag == 1)
    {
        init_flag = 0;
        AKM_Open();
    }
    return 0;
}
int AKM_Close(void)
{
    int16_t ret;
    ret = AKM_StopAndSave();
    return ret;
}
int AKM_SetMagData(int32_t data_x, int32_t data_y, int32_t data_z, int64_t time_stamp)
{
    struct AKM_SENSOR_DATA sensordata;
    int ret;
    sensordata.stype = AKM_ST_MAG;
    sensordata.time_stamp = time_stamp;

    sensordata.u.v[0] = data_x;
    sensordata.u.v[1] = data_y;
    sensordata.u.v[2] = data_z;

    ret = AKM_SetData(&sensordata);
    return ret;
}
int AKM_SetGyroData(double data_x, double data_y, double data_z, int64_t time_stamp)
{
    struct AKM_SENSOR_DATA sensordata;
    int16_t ret;
    sensordata.stype = AKM_ST_GYR;
    sensordata.time_stamp = time_stamp;

    sensordata.u.v[0] = (int32_t)((float)data_x * RAD2DEG);
    sensordata.u.v[1] = (int32_t)((float)data_y * RAD2DEG);
    sensordata.u.v[2] = (int32_t)((float)data_z * RAD2DEG);

    ret = AKM_SetData(&sensordata);
    return ret;
}
int AKM_GetMagData(double *cali_data, double *offset, int16_t *accuracy)
{
    float32_t sensor_result[6] = {0};
    int ret;
    int32_t tmp_accuracy = 0;
    ret = AKM_GetData(sensor_result,AKM_VT_MAG,&(tmp_accuracy));

    cali_data[0] = (float)(sensor_result[0]);
    cali_data[1] = (float)(sensor_result[1]);
    cali_data[2] = (float)(sensor_result[2]);

    offset[0] = (float)(sensor_result[3]);
    offset[1] = (float)(sensor_result[4]);
    offset[2] = (float)(sensor_result[5]);

    //AKM_LOG_DBG("AKM_GetMagData calidata:%f,%f,%f\n",cali_data[0],cali_data[1],cali_data[2]);
    *accuracy = tmp_accuracy;

    return ret;
}

#ifdef CFG_AKM_FUSION_SUPPORT
int AKM_FusionSetAccData(double data_x, double data_y, double data_z, int64_t time_stamp)
{
    struct AKM_SENSOR_DATA sensordata;
    int ret;
    sensordata.stype = AKM_ST_FUSION_ACC;
    sensordata.time_stamp = time_stamp;

    sensordata.u.v[0] = (float)data_x ;
    sensordata.u.v[1] = (float)data_y ;
    sensordata.u.v[2] = (float)data_z ;
    AKM_LOG_DBG("AKM_FusionSetAccData acc data:%f,%f,%f, acc time:%lld\n",data_x,data_y,data_z,time_stamp);

    ret = AKM_SetFusionData(&sensordata);
    return ret;
}
int AKM_FusionSetMagData(double data_x, double data_y, double data_z, int64_t time_stamp)
{
    struct AKM_SENSOR_DATA sensordata;
    int ret;
    sensordata.stype = AKM_ST_FUSION_MAG;
    sensordata.time_stamp = time_stamp;

    sensordata.u.v[0] = (float)data_x ;
    sensordata.u.v[1] = (float)data_y ;
    sensordata.u.v[2] = (float)data_z ;

    //AKM_LOG_DBG("AKM_FusionSetMagData mag data:%f,%f,%f,  mag time:%lld\n",data_x,data_y,data_z,time_stamp);
    ret = AKM_SetFusionData(&sensordata);
    return ret;
}
int AKM_FusionSetGyroData(double data_x, double data_y, double data_z, int64_t time_stamp)
{
    struct AKM_SENSOR_DATA sensordata;
    int ret;
    sensordata.stype = AKM_ST_FUSION_GYR;
    sensordata.time_stamp = time_stamp;

    sensordata.u.v[0] = (float)data_x * RAD2DEG;
    sensordata.u.v[1] = (float)data_y * RAD2DEG;
    sensordata.u.v[2] = (float)data_z * RAD2DEG;

    ret = AKM_SetFusionData(&sensordata);
    return ret;
}
int AKM_GetOri(double *cali_data, int16_t *accuracy)
{
    float32_t sensor_result[6] = {0};
    int ret;
    int32_t tmp_accuracy = 0;
    ret = AKM_GetData(sensor_result,AKM_VT_ORI,&(tmp_accuracy));

    cali_data[0] = (float)(sensor_result[0]);
    cali_data[1] = (float)(sensor_result[1]);
    cali_data[2] = (float)(sensor_result[2]);

    *accuracy = tmp_accuracy;
    return ret;
}
#ifdef CFG_AKM_PG_SUPPORT
int AKM_GetGyroscope(double *cali_data, int16_t *accuracy)
{
    float32_t sensor_result[6] = {0};
    int ret;
    int32_t tmp_accuracy = 0;
    ret = AKM_GetData(sensor_result,AKM_VT_GYR,&(tmp_accuracy));

    cali_data[0] = (float)(sensor_result[0]);
    cali_data[1] = (float)(sensor_result[1]);
    cali_data[2] = (float)(sensor_result[2]);
    cali_data[0] = cali_data[0] * (double)DEG2PAD;
    cali_data[1] = cali_data[1] * (double)DEG2PAD;
    cali_data[2] = cali_data[2] * (double)DEG2PAD;

    //printf("AKM_GetGyroscope data output:%f,%f,%f\n",(double)sensor_result[0],(double)sensor_result[1],(double)sensor_result[2]);
    *accuracy = tmp_accuracy;
    return ret;
}
int AKM_GetRotationVector(double *cali_data, int16_t *accuracy)
{
    float32_t sensor_result[6] = {0};
    int ret;
    int32_t tmp_accuracy = 0;
    ret = AKM_GetData(sensor_result,AKM_VT_QUAT,&(tmp_accuracy));

    cali_data[0] = (float)(sensor_result[0]);
    cali_data[1] = (float)(sensor_result[1]);
    cali_data[2] = (float)(sensor_result[2]);
    cali_data[3] = (float)(sensor_result[3]);

    *accuracy = tmp_accuracy;
    return ret;
}
int AKM_GetLinearAccelerometer(double *cali_data, int16_t *accuracy)
{
    float32_t sensor_result[6] = {0};
    int ret;
    int32_t tmp_accuracy = 0;
    ret = AKM_GetData(sensor_result,AKM_VT_LACC,&(tmp_accuracy));

    cali_data[0] = (float)(sensor_result[0]);
    cali_data[1] = (float)(sensor_result[1]);
    cali_data[2] = (float)(sensor_result[2]);

    *accuracy = tmp_accuracy;
    return ret;
}
int AKM_GetGravity(double *cali_data, int16_t *accuracy)
{
    float32_t sensor_result[6] = {0};
    int ret;
    int32_t tmp_accuracy = 0;
    ret = AKM_GetData(sensor_result,AKM_VT_GRAVITY,&(tmp_accuracy));

    cali_data[0] = (float)(sensor_result[0]);
    cali_data[1] = (float)(sensor_result[1]);
    cali_data[2] = (float)(sensor_result[2]);

    *accuracy = tmp_accuracy;
    return ret;
}
#endif
#endif



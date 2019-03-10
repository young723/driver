
#include <stdio.h>
#include "MemsicConfig.h"

////for save the calibration parameters
#define SAVE_CAL_DATA 0

#if SAVE_CAL_DATA
#include <fcntl.h>

#define NVM_SIZE 1024
#define NVM_PATH "/data/misc/magpara"
#endif
#if 0
static float fMagSP[9] = {1,0,0,
0,1,0,
0,0,1};

#else
static float fMagSP[9] = {0.999660338205083   ,    0.00645666449110527  ,   0.0196073668194122,
-0.0113381901912553   ,  0.996774155833341    ,   0.0315612241215416,
0.0234660108004154 ,     -0.0288085468422545 ,    1.00760148652868};
#endif

#if SAVE_CAL_DATA
int ReadDataFromFile(float *pa);
int SaveDataInFile(float *pa, float *pb);
#endif


int SetMagSensorPara(float *pa,float *pb)
{
        int i;
        float magHP[4] = {0.0f,0.0f,0.0f,0.5f};

        for(i=0;i<9;i++){
        pa[i] = fMagSP[i];
          //printf("memsicalgoinit pa[%d] = %f\n",i,pa[i]);
}

        #if SAVE_CAL_DATA
        if(ReadDataFromFile(magHP)<0){
        ALOGE("memsicalgoinit config.c %s: fail to load the calibration file\n",__FUNCTION__);
        }
        #endif

        for(i=0;i<4;i++){
        pb[i] = magHP[i];
        //printf("memsicalgoinit pb[%d] = %f\n",i,pb[i]);
        }

        return 0;
}

/*******************************************************************************************
* Function Name: SaveMagSensorPara
* Description: Save the magnetic sensor parameter.
********************************************************************************************/
int SaveMagSensorPara(float *pa,float *pb)
{
#if SAVE_CAL_DATA
if(SaveDataInFile(pa,pb)<0){
ALOGE("memsicalgo config.c %s: fail to save the calibration file\n",__FUNCTION__);
}
#endif

return 1;
}

/*******************************************************************************************
* Function Name: SetOriSensorPara
* Description: Set the orientation sensor parameter
********************************************************************************************/
int SetOriSensorPara(float *pa, float *pb, int *pc)
{
pa[0] = 15; //FirstLevel= can not be more than 15
pa[1] = 24; //SecondLevel= can not be more than 35
pa[2] = 35;//ThirdLevel= can not be more than 50
pa[3] = 0.09f; //dCalPDistLOne
pa[4] = 0.08f; //dCalPDistLTwo
pa[5] = 0.85f;   //dPointToCenterLow
pa[6] = 1.15f; //dPointToCenterHigh
pa[7] = 1.4f; //dMaxPiontsDistance

pb[0] = 0.1f; //dSatAccVar = threshold for saturation judgment
pb[1] = 0.00002f; //dSatMagVar = threshold for saturation judgment
pb[2] = 0.0002f; //dMovAccVar= threshold for move judgment
  pb[3] = 0.0001f;//dMovMagVar = threshold for move judgment

pc[0] = 10; //delay_ms = sampling interva
pc[1] = 15; //yawFirstLen= can not be more than 31, must be odd number.
pc[2] = 10; //yawFirstLen= can not be more than 30
pc[3] = 2; //corMagLen= can not be more than 20

#if 0
int i;
for(i=0;i<8;i++){
printf("memsicalgoinit pa[%d] = %f\n",i,pa[i]);
}
for(i=0;i<4;i++){
printf("memsicalgoinit pb[%d] = %f\n",i,pb[i]);
}
for(i=0;i<4;i++){
printf("memsicalgoinit pc[%d] = %d\n",i,pc[i]);
}
#endif

return 1;
}
/*****************************************************************
* Description:
*****************************************************************/
#if SAVE_CAL_DATA
int SaveDataInFile(float *pa, float *pb)
{
int fd = -1;
char buf[NVM_SIZE];

fd = open(NVM_PATH, O_WRONLY | O_CREAT, 0666);
if (fd < 0){
ALOGE("memsicpara config.c %s: fail to open %s\n", __FUNCTION__, NVM_PATH);
return -1;
}

memset(buf, '\0', NVM_SIZE);
sprintf(buf, "%f %f %f %f\r\n",pa[0], pa[1],pa[2],pb[0]);
write(fd, buf, strlen(buf));
close(fd);

//ALOGE("memsicpara config.c %s: save data %f\t %f\t %f\t %f\t\n", __FUNCTION__, pa[0], pa[1], pa[2],pb[0]);

return fd;
}
#endif
/*****************************************************************
* Description:
*****************************************************************/
#if SAVE_CAL_DATA
int ReadDataFromFile(float *pa)
{
int fd;
int n;
char buf[NVM_SIZE];

fd = open(NVM_PATH, O_RDONLY);
if (fd == -1){
ALOGE("memsicpara config.c %s: fail to open %s\n", __FUNCTION__, NVM_PATH);
return -1;
}

n = read(fd, buf, NVM_SIZE);
if (n <= 0){
ALOGE("memsicpara config.c %s: there is no data in file %s\n", __FUNCTION__, NVM_PATH);
close(fd);
return -1;
}

sscanf(buf, "%f %f %f %f\r\n",&pa[0], &pa[1], &pa[2],&pa[3]);
close(fd);

//ALOGE("memsicpara config.c %s: read data %f\t %f\t %f\t %f\t\n", __FUNCTION__, pa[0], pa[1], pa[2], pa[3]);

return fd;
}
#endif


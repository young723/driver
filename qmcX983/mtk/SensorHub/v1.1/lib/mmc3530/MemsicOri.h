/*****************************************************************************
* Copyright (C), 2016, MEMSIC Inc.
* File Name: MemsicOri.h
* Autho: MEMSIC AE Algoritm Team
* Description: This file is the head file of MemsicAlgo.lib. It provides the
*interface function declarations of the lib.
* History: 1. Data: 2016/11/01
*2. Autho: Yan Guopu
*3. Modification:
*****************************************************************************/
#ifndef _MEMSICALGORITHM_H_
#define _MEMSICALGORITHM_H_

#define ANDROID_PLATFORM 1
#define EVB_PLATFORM 2
#define WINDOWS_PLATFORM 3
#define PLATFORM ANDROID_PLATFORM

#if((PLATFORM == ANDROID_PLATFORM)||(PLATFORM == EVB_PLATFORM))
#define DLL_API
#elif(PLATFORM == WINDOWS_PLATFORM)
#define DLL_API extern "C" __declspec(dllexport)
#endif

#if(PLATFORM == ANDROID_PLATFORM)
#include "MemsicConfig.h"
#endif

#ifndef MEMSIC_ALGORITHM_DATA_TYPES
#define MEMSIC_ALGORITHM_DATA_TYPES
typedef   signed char int8; // signed 8-bit number    (-128 to +127)
typedef unsigned char uint8; // unsigned 8-bit number  (+0 to +255)
typedef   signed short  int16; // signed 16-bt number    (-32,768 to +32,767)
typedef unsigned short  uint16; // unsigned 16-bit number (+0 to +65,535)
typedef   signed int int32; // signed 32-bit number   (-2,147,483,648 to +2,147,483,647)
typedef unsigned int uint32; // unsigned 32-bit number (+0 to +4,294,967,295)
#endif

#if(PLATFORM == ANDROID_PLATFORM)
/*******************************************************************************************
* Function Name: AlgoInitial
* Description: Initialize the algorithm.
* Inpu: None
* Output: None.
* Return: 1 --- succeed.
*-1 --- fail.
********************************************************************************************/
int AlgoInitial(void);
#elif((PLATFORM == EVB_PLATFORM)||(PLATFORM == WINDOWS_PLATFORM))
/*******************************************************************************************
* Function Name: InitialAlgorithm
* Description: Initial the algorithm.
* Input: sim[0-8] ---soft iron matrix;
*mag_para[0-2]---mag offset;
*mag_para[3]  ---mag radius;
* Output: None.
* Retur: 1 --- succeed.
* -1 --- fail.
********************************************************************************************/
int InitialAlgorithm(float *sim, float *mag_para);
#endif

int MainAlgorithmProcess(float *acc, float *mag, float *reg_mag);


int GetMagAccuracy(void);


int GetMagSaturation(void);


int GetCalPara(float *cp);


int GetCalMag(float *cm);


int GetCalOri(float *co);

#endif


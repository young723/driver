/******************************************************************************
 *
 *  $Id: $
 *
 * -- Copyright Notice --
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 * -- End Asahi Kasei Microdevices Copyright Notice --
 *
 ******************************************************************************/
#ifndef AKMD_INC_FILEIO_H
#define AKMD_INC_FILEIO_H

// Common include files.
#include "AKM_Common.h"
#include "AKM_CustomerSpec.h"
// Include file for SmartCompass library.
#include "akl_smart_compass.h"
#include "stdio.h"
/*** Constant definition ******************************************************/


typedef struct _AKSCSAVEPRMS{
    int16vec        HSUC_HO[AKM_CUSTOM_NUM_FORM];
    int16vec        HFLUCV_HREF[AKM_CUSTOM_NUM_FORM];
    AKSC_HDST       HSUC_HDST[AKM_CUSTOM_NUM_FORM];
    int32vec        HSUC_HBASE[AKM_CUSTOM_NUM_FORM];
} AKSCSAVEPRMS;


/*** Type declaration *********************************************************/

/*** Global variables *********************************************************/

/*** Prototype of function ****************************************************/
int16 AKL_LoadParameter(struct AKL_NV_PRMS *nv_data);

int16 AKL_LoadPDC(uint8 *pdc);

int16 AKL_SaveParameter(struct AKL_NV_PRMS *nv_data);

#endif


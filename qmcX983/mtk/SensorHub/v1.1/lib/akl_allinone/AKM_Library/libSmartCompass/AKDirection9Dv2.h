/******************************************************************************
 *
 *  $Id: AKDirection9Dv2.h 367 2016-04-01 06:42:49Z suzuki.thj $
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
#ifndef AKSC_INC_AKDIRECTION9DV2_H
#define AKSC_INC_AKDIRECTION9DV2_H

#include "AKHDOE.h"
#include "AKMDevice.h"
#include "AKMDeviceF.h"

//========================= Type declaration  ===========================//
typedef struct _AKSC_D9DVAR AKSC_D9DVAR; // v2

typedef enum _AKSC_D9DV2_SENSOR_RELIABILITY {
    AKSC_RELIABILITY_LV1,
    AKSC_RELIABILITY_LV2,
    AKSC_RELIABILITY_LV3,
} AKSC_D9DV2_SENSOR_RELIABILITY;

typedef enum D9DV2_MODE_ {
    D9DV2_MODE_9D = 0, // use mag, gyro and acc
    D9DV2_MODE_GYRO,   // use gyro only
    D9DV2_MODE_6D      // use mag and acc
} D9DV2_MODE;

//========================= Prototype of Function =======================//
AKLIB_C_API_START

int16 AKSC_GetD9DVerSize(void); //(o) size of 9DFusion v2 parameters

void AKSC_InitD9Dv2(
    AKSC_D9DVAR *var             // (i/o) data structure
);

int16 AKSC_D9Dv2(
    const AKSC_HDST  hdst,       // (i) DOE level
    const uint8      licenser[], // (i) licenser
    const uint8      licensee[], // (i) licensee
    const int16      key[],      // (i) key
    const D9DV2_MODE mode,       // (i) mode for DOE
    const int16vec   *h,         // (i) calibrated mag vector, 16.67[uT]
    const int16vec   *a,         // (i) calibrated acc vector, 720[G]
    const AKSC_FVEC  *g,         // (i) calibrated gyro vector, Q4[dps]
    const int16      hdt,        // (i) sampling time for mag, Q4[msec]
    const int16      adt,        // (i) sampling time for acc, Q4[msec]
    const int16      gdt,        // (i) sampling time for gyro, Q4[msec]
    const int16vec   *dvec,      // (i) vector to define reference axis of the azimuth on the terminal coordinate system
    const I16MATRIX  *hlayout,   // (i) layout matrix for mag
    const I16MATRIX  *alayout,   // (i) layout matrix for acc
    const I16MATRIX  *glayout,   // (i) layout matrix for gyro
    AKSC_D9DVAR      *var,       // (i/o) data structure
    int16            *theta,     // (o) azimuth angle, Q6[deg]
    int16            *delta,     // (o) inclination angle, Q6[deg]
    int16            *hr,        // (o) Geomagnetic vector size, 16.67[uT]
    int16            *hrhoriz,   // (o) Horizontal element of geomagnetic vector, 16.67[uT]
    int16            *ar,        // (o) Acceleration vector size, 720[G]
    int16            *phi180,    // (o) Pitch angle (-180 to +180 degree), Q6[deg]
    int16            *phi90,     // (o) Pitch angle (-90 to +90 degree), Q6[deg]
    int16            *eta180,    // (o) Roll angle  (-180 to +180 degree), Q6[deg]
    int16            *eta90,     // (o) Roll angle  (-90 to +90 degree), Q6[deg]
    AKSC_FMAT        *mat,       // (o) Rotation matrix
    AKSC_FQUAT       *quat,      // (o) Rotation Quaternion
    AKSC_FVEC        *gravity,   // (o) Gravity, 720[G]
    AKSC_FVEC        *linacc     // (o) Linear acceleration, 720[G]
);

// === mag ===
// LV1: strick to use mag data
// LV2: use mag as appropriate
// Lv3: positively use mag data
// === acc ===
// LV1: never use acc data
// Lv2: use acc. offset is over 0.2[G]
// LV3: use acc. offset is less than 0.2[G]
// === gyro ===
// LV1: use gyro. offset is over 5[dps]
// LV2: use gyro. offset is less than 5[dps]
// Lv3: use gyro. offset is less than 1[dps]
void AKSC_SetD9Dv2PrmsSet(
    const AKSC_D9DV2_SENSOR_RELIABILITY magReliability,
    const AKSC_D9DV2_SENSOR_RELIABILITY accReliability,
    const AKSC_D9DV2_SENSOR_RELIABILITY gyrReliability,
    AKSC_D9DVAR                         *var
);

void AKSC_SetD9Dv2Prms(
    const int16      index,  // (i) index for parameters
    const AKSC_FLOAT val,    // (i) input value
    AKSC_D9DVAR      *var    // (i/o) data structure
);

// function for debug
void AKSC_GetD9Dv2DebugInfo(
    const AKSC_D9DVAR *var,  // (i) data structure
    int16vec          *dbg1, // (o) debug information
    int32vec          *dbg2, // (o) debug information
    int32vec          *dbg3, // (o) debug information
    int16vec          *dbg4, // (o) debug information
    int16vec          *dbg5  // (o) debug information
);

AKLIB_C_API_END
#endif

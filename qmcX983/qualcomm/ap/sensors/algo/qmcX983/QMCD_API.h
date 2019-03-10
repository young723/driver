/******************************************************************************
 *
 *  $Id: AKMD_APIs.h 1032 2013-06-12 09:23:43Z yamada.rj $
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
#pragma once

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <hardware/sensors.h>

#include "ical.h"
#include "dummyGyro.h"
#include "qmcLog.h"
#include "CalibrationModule.h"

#if defined(__cplusplus)
extern "C" {
#endif

int QMCD_GetSensorsData(sensors_event_t *raw_h, sensors_event_t *acc,
					 sensors_event_t *pg, sensors_event_t *rv ,
					 sensors_event_t *ori, sensors_event_t *ga,
					 sensors_event_t *la);

#if defined(__cplusplus)
}
#endif



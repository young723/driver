
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
#ifndef AKMD_INC_AKMLOG_H
#define AKMD_INC_AKMLOG_H
//#include <vendor_cust.h>

#define AKM_ENABLE_LOG_INFO
#define AKM_ENABLE_LOG_ERR

#ifdef AKM_ENABLE_LOG_INFO
#define AKM_MSG_INFO_0(msg)             printf(msg)
#define AKM_MSG_INFO_1(msg,p1)          printf(msg,p1)
#define AKM_MSG_INFO_2(msg,p1,p2)       printf(msg,p1,p2)
#define AKM_MSG_INFO_3(msg,p1,p2,p3)    printf(msg,p1,p2,p3)
#define AKM_MSG_INFO_4(msg,p1,p2,p3,p4) printf(msg,p1,p2,p3,p4)
#else
#define AKM_MSG_INFO_0(msg)
#define AKM_MSG_INFO_1(msg,p1)
#define AKM_MSG_INFO_2(msg,p1,p2)
#define AKM_MSG_INFO_3(msg,p1,p2,p3)
#define AKM_MSG_INFO_4(msg,p1,p2,p3,p4)
#endif

#ifdef AKM_ENABLE_LOG_ERR
#define AKM_MSG_ERR_0(msg)              printf(msg)
#define AKM_MSG_ERR_1(msg,p1)           printf(msg,p1)
#define AKM_MSG_ERR_2(msg,p1,p2)        printf(msg,p1,p2)
#define AKM_MSG_ERR_3(msg,p1,p2,p3)     printf(msg,p1,p2,p3)
#define AKM_MSG_ERR_4(msg,p1,p2,p3,p4)  printf(msg,p1,p2,p3,p4)
#else
#define AKM_MSG_ERR_0(msg)
#define AKM_MSG_ERR_1(msg,p1)
#define AKM_MSG_ERR_2(msg,p1,p2)
#define AKM_MSG_ERR_3(msg,p1,p2,p3)
#define AKM_MSG_ERR_4(msg,p1,p2,p3,p4)
#endif

#define AKM_LOG_DBG     printf
#define AKMDEBUG(flag, format, ...)
#define AKMERROR


#endif //AKMD_INC_AKMLOG_H




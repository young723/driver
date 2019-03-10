/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *    motion_sensor_custom.h
 *
 * Project:
 * --------
 *   Maui_Software
 *
 * Description:
 * ------------
 *   This Module is for motion sensor driver.
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#ifndef _MOTION_SENSOR_CUSTOM_H
#define _MOTION_SENSOR_CUSTOM_H

#include "motion_sensor.h"
/*========================================================================================================
										D E F I N E
========================================================================================================*/
#ifdef _MOTION_SENSOR_CUSTOM_C_
  #define _MOTION_SENSOR_CUSTOM_DEC_
#else
  #define _MOTION_SENSOR_CUSTOM_DEC_ extern
#endif

/******************************************************
	Motion sensor controller register macro define
*******************************************************/
//#define QMAX981_HAND_LIGHT
#define QMAX981_STEP_COUNTER
#define QMAX981_USE_INT1
//#define QMAX981_USE_INT2
#if defined(QMAX981_STEP_COUNTER)
#if defined(QMAX981_USE_INT1)
//#define QMAX981_STEP_COUNTER_USE_INT
#endif
#define QMAX981_CHECK_ABNORMAL_DATA
#endif

/***********QST TEAM***************************************/
#define GRAVITY_1G			9807

#define QMAX981_RANGE_2G        (1<<0)
#define QMAX981_RANGE_4G        (1<<1)
#define QMAX981_RANGE_8G        (1<<2)
#define QMAX981_RANGE_16G       (1<<3)
	
#if defined(QMAX981_STEP_COUNTER)
#define QMAX981_OFFSET_X		0x60
#define QMAX981_OFFSET_Y		0x60
#define QMAX981_OFFSET_Z		0x60
#else
#define QMAX981_OFFSET_X		0x00
#define QMAX981_OFFSET_Y		0x00
#define QMAX981_OFFSET_Z		0x00
#endif
	
#define QMAX981_CHIP_ID			0x00
#define QMAX981_XOUTL			0x01	// 4-bit output value X
#define QMAX981_XOUTH			0x02	// 6-bit output value X
#define QMAX981_YOUTL			0x03	
#define QMAX981_YOUTH			0x04	
#define QMAX981_ZOUTL			0x05	
#define QMAX981_ZOUTH			0x06
#define QMAX981_STEP_CNT_L		0x07
	
#define QMAX981_RANGE			0x0f
#define QMAX981_ODR				0x10
#define QMAX981_MODE			0x11
	
#define QMAX981_INT_MAP0		0x19	// INT MAP
#define QMAX981_INT_STAT		0x0a    //interrupt statues
	
#define QMAX981_FIFO_WTMK		0x31	// FIFO water mark level
#define QMAX981_FIFO_CONFIG		0x3e	// fifo configure
#define QMAX981_FIFO_DATA		0x3f	//fifo data out 
	
#define SINGLE_TAP 1
#define DOUBLE_TAP 2
	
#define YZQ_ABS(X) ((X) < 0 ? (-1 * (X)) : (X))
	
//-------------------------------------
// 中断处理
//-------------------------------------
// add by qst for test
#define QMAX981_LOW_G_FLAG		0x08
#define QMAX981_HIGH_G_Z_FLAG	0x04
#define QMAX981_HIGH_G_Y_FLAG	0x02
#define QMAX981_HIGH_G_X_FLAG	0x01
	
//#define QMAX981_FUNC		QMAX981_LOW_G_FLAG
	
//-------------------------------------
// 中断处理
//-------------------------------------
#define QMAX981_D_TAP_FLAG	0x10
#define QMAX981_S_TAP_FLAG	0x20
#define QMAX981_ORIENT_FLAG	0x40
#define QMAX981_FOB_FLAG	0x80
	
#define QMAX981_LOW_G_FLAG		0x08
#define QMAX981_HIGH_G_Z_FLAG	0x04
#define QMAX981_HIGH_G_Y_FLAG	0x02
#define QMAX981_HIGH_G_X_FLAG	0x01
	
//#define QMA6891_EXTRA_FUNC_1		(QMAX981_FOB_FLAG|QMAX981_ORIENT_FLAG)
//#define QMA6891_EXTRA_FUNC_2		(QMAX981_HIGH_G_Z_FLAG|QMAX981_HIGH_G_Y_FLAG|QMAX981_HIGH_G_X_FLAG)
//#define QMA6891_EXTRA_FUNC_3		(QMAX981_S_TAP_FLAG)

/******************************************************
	I2C header (slaver address|W/R)   macro define
*******************************************************/

/******************************************************
	GPIO macro define for software I2C
*******************************************************/

//==============================
// 供上层查询用的+-2G 范围
//==============================
#if defined(QMAX981_STEP_COUNTER)
#define ACC_0G_X      2048
#define ACC_1G_X      (2048+64)
#define ACC_MINUS1G_X (2048-64)
#define ACC_0G_Y      2048   
#define ACC_1G_Y      (2048+64)
#define ACC_MINUS1G_Y (2048-64)
#define ACC_0G_Z      2048       
#define ACC_1G_Z      (2048+64)
#define ACC_MINUS1G_Z (2048-64)
#else
#define ACC_0G_X      2048
#define ACC_1G_X      (2048+128)
#define ACC_MINUS1G_X (2048-128)
#define ACC_0G_Y      2048   
#define ACC_1G_Y      (2048+128)
#define ACC_MINUS1G_Y (2048-128)
#define ACC_0G_Z      2048       
#define ACC_1G_Z      (2048+128)
#define ACC_MINUS1G_Z (2048-128)
#endif
/*
//ADC value configure of 0g 1g and -1g in X,Y,Z axis

#define ACC_0G_X      (2048) 			
#define ACC_1G_X      (2048+256)
#define ACC_MINUS1G_X (2048-256)
#define ACC_0G_Y      (2048)
#define ACC_1G_Y      (2048+256)
#define ACC_MINUS1G_Y (2048-256)
#define ACC_0G_Z      (2048)
#define ACC_1G_Z      (2048+256)
#define ACC_MINUS1G_Z (2048-256)
*/
//==============================
// 供底层轮询模拟中断用的+-16G 范围
//==============================
#define ACC_POLL_0G_X      16384
#define ACC_POLL_1G_X      (16384+1024)
#define ACC_POLL_MINUS1G_X (16384-1024)
#define ACC_POLL_0G_Y      16384   
#define ACC_POLL_1G_Y      (16384+1024)
#define ACC_POLL_MINUS1G_Y (16384-1024)
#define ACC_POLL_0G_Z      16384       
#define ACC_POLL_1G_Z      (16384+1024)
#define ACC_POLL_MINUS1G_Z (16384-1024)

/*========================================================================================================
										E X T E R N 
========================================================================================================*/
extern MotionSensor_customize_function_struct *ms_GetFunc(void);
extern MotionSensor_custom_data_struct *ms_get_data(void);


/*========================================================================================================
										T Y P E D E F
========================================================================================================*/
typedef struct 
{
    kal_uint16  X;
    kal_uint16  Y;
    kal_uint16  Z;  
} SPARAMETERS;

/*========================================================================================================
										F U N C T I O N
========================================================================================================*/

/*========================================================================================================
										end
========================================================================================================*/
#endif	//_MOTION_SENSOR_CUSTOM_H

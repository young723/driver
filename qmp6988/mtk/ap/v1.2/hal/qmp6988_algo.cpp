/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2012. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <string.h>

#include <log/log.h>

#include "Pressure.h"
#include <utils/SystemClock.h>
#include <utils/Timers.h>
#include <inttypes.h>
#include "qmp6988_algo.h"

#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "qmp6988_algo"
#endif


#define IGNORE_EVENT_TIME 0
#define CALIDATA_PATH           "/sys/bus/platform/drivers/barometer/calidata"
#define RAWDATA_PATH           "/sys/bus/platform/drivers/barometer/sensordata"

static float Conv_A_S[10][2] = {
	{-6.30E-03,4.30E-04},
	{-1.90E-11,1.20E-10},
	{1.00E-01,9.10E-02},
	{1.20E-08,1.20E-06},
	{3.30E-02,1.90E-02},
	{2.10E-07,1.40E-07},
	{-6.30E-10,3.50E-10},
	{2.90E-13,7.60E-13},
	{2.10E-15,1.20E-14},
	{1.30E-16,7.90E-17},
};

static struct qmp6988_calibration_data qmp6988_cali;
static float a0,b00;
static float a1,a2,bt1,bt2,bp1,b11,bp2,b12,b21,bp3;
static float press, tempearture;
static int qmp6988_cali_fd = -1;
static int qmp6988_data_fd = -1;

float qmp6988_calc_press(void)
{
	char databuf[64];
	double Tr = 0;
	double Pr = 0;
	int Dt, Dp;

	if(qmp6988_data_fd < 0)
	{
		qmp6988_data_fd = TEMP_FAILURE_RETRY(open(RAWDATA_PATH, O_RDONLY));
		//ALOGD("qmp6988_data_fd = %d", qmp6988_data_fd);
		memset(databuf, 0, sizeof(databuf));
		TEMP_FAILURE_RETRY(read(qmp6988_data_fd, databuf, sizeof(databuf)-1));
		sscanf(databuf, "%d %d\n", &Dp, &Dt);
		close(qmp6988_data_fd);
		qmp6988_data_fd = -1;
	}

	Tr = a0 + a1*Dt + a2*Dt*Dt;
	//Unit centigrade
	tempearture = (float)(Tr / 256.0f);
	//compensation pressure, Unit Pa
	Pr = b00 + bt1*Tr + bp1*Dp + b11*Tr*Dp + bt2*Tr*Tr + bp2*Dp*Dp + b12*Dp*Tr*Tr + b21*Dp*Dp*Tr + bp3*Dp*Dp*Dp;
	press = (float)Pr/100.0f;
	ALOGD("Dp[%d] Dt[%d] T[%f] Pr [%f]",Dp, Dt, tempearture, press);

	return press;
}

float qmp6988_calc_temperature(void)
{
	char databuf[64];
	double Tr = 0;
	int Dt, Dp;

	if(qmp6988_data_fd < 0)
	{
		qmp6988_data_fd = TEMP_FAILURE_RETRY(open(RAWDATA_PATH, O_RDONLY));
		ALOGD("qmp6988_data_fd = %d", qmp6988_data_fd);
		memset(databuf, 0, sizeof(databuf));
		TEMP_FAILURE_RETRY(read(qmp6988_data_fd, databuf, sizeof(databuf)-1));
		sscanf(databuf, "%d %d\n", &Dp, &Dt);
		close(qmp6988_data_fd);
		qmp6988_data_fd = -1;
	}

	Tr = a0 + a1*Dt + a2*Dt*Dt;
	//Unit centigrade
	tempearture = (float)(Tr / 256.0f);

	return tempearture;
}


float qmp6988_calc(int Dp, int Dt)
{
	double Tr = 0;
	double Pr = 0;

	Tr = a0 + a1*Dt + a2*Dt*Dt;
	//Unit centigrade
	tempearture = (float)(Tr / 256.0f);
	//compensation pressure, Unit Pa
	Pr = b00 + bt1*Tr + bp1*Dp + b11*Tr*Dp + bt2*Tr*Tr + bp2*Dp*Dp + b12*Dp*Tr*Tr + b21*Dp*Dp*Tr + bp3*Dp*Dp*Dp;
	press = (float)Pr/100.0f;
	ALOGD("Dp[%d] Dt[%d] T[%f] Pr [%f]",Dp, Dt, tempearture, press);

	return press;
}


int qmp6988_algo_init(void)
{
	char calibuf[256];

	ALOGD("qmp6988_algo_init");
	if(qmp6988_cali_fd<0)
	{
		qmp6988_cali_fd = TEMP_FAILURE_RETRY(open(CALIDATA_PATH, O_RDONLY));
		ALOGD("qmp6988_cali_fd=%d", qmp6988_cali_fd);
	}
	else
	{
		return 0;
	}
	
	if(qmp6988_cali_fd>0)
	{
		int err;
		//err = ioctl(fd, BAROMETER_GET_CALI, &qmp6988_cali);
		//ALOGD("ioctl err = %d", err);
		memset(calibuf, 0, sizeof(calibuf));
		TEMP_FAILURE_RETRY(read(qmp6988_cali_fd, calibuf, sizeof(calibuf)-1));
		sscanf(calibuf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
			&qmp6988_cali.COE_a0,&qmp6988_cali.COE_a1,&qmp6988_cali.COE_a2,
			&qmp6988_cali.COE_b00,&qmp6988_cali.COE_bt1,&qmp6988_cali.COE_bt2,
			&qmp6988_cali.COE_bp1,&qmp6988_cali.COE_b11,
			&qmp6988_cali.COE_bp2,&qmp6988_cali.COE_b12,
			&qmp6988_cali.COE_b21,&qmp6988_cali.COE_bp3);

		close(qmp6988_cali_fd);
		qmp6988_cali_fd = -1;

		a0 = qmp6988_cali.COE_a0 /16.0f;
		b00 = qmp6988_cali.COE_b00 /16.0f;

		a1 = Conv_A_S[0][0] + Conv_A_S[0][1] * qmp6988_cali.COE_a1 / 32767.0f;
		a2 = Conv_A_S[1][0] + Conv_A_S[1][1] * qmp6988_cali.COE_a2 / 32767.0f;
		bt1 = Conv_A_S[2][0] + Conv_A_S[2][1] * qmp6988_cali.COE_bt1 / 32767.0f;
		bt2 = Conv_A_S[3][0] + Conv_A_S[3][1] * qmp6988_cali.COE_bt2 / 32767.0f;
		bp1 = Conv_A_S[4][0] + Conv_A_S[4][1] * qmp6988_cali.COE_bp1 / 32767.0f;
		b11 = Conv_A_S[5][0] + Conv_A_S[5][1] * qmp6988_cali.COE_b11 / 32767.0f;
		bp2 = Conv_A_S[6][0] + Conv_A_S[6][1] * qmp6988_cali.COE_bp2 / 32767.0f;
		b12 = Conv_A_S[7][0] + Conv_A_S[7][1] * qmp6988_cali.COE_b12 / 32767.0f;
		b21 = Conv_A_S[8][0] + Conv_A_S[8][1] * qmp6988_cali.COE_b21 / 32767.0f;
		bp3 = Conv_A_S[9][0] + Conv_A_S[9][1] * qmp6988_cali.COE_bp3 / 32767.0f;
#if 1
		ALOGD("<-----------get calibration data-------------->\n");
		ALOGD("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
			qmp6988_cali.COE_a0,qmp6988_cali.COE_a1,qmp6988_cali.COE_a2,qmp6988_cali.COE_b00);
		ALOGD("COE_bt1[%d] COE_bt2[%d] COE_bp1[%d] COE_b11[%d]\n",
			qmp6988_cali.COE_bt1,qmp6988_cali.COE_bt2,qmp6988_cali.COE_bp1,qmp6988_cali.COE_b11);
		ALOGD("COE_bp2[%d] COE_b12[%d] COE_b21[%d] COE_bp3[%d]\n",
			qmp6988_cali.COE_bp2,qmp6988_cali.COE_b12,qmp6988_cali.COE_b21,qmp6988_cali.COE_bp3);
		ALOGD("<-----------calibration data done-------------->\n");

		ALOGD("<----------- float calibration data -------------->\n");
		ALOGD("a0[%lle] a1[%lle] a2[%lle] b00[%lle]\n",a0,a1,a2,b00);
		ALOGD("bt1[%lle]	bt2[%lle]	bp1[%lle]	b11[%lle]\n",bt1,bt2,bp1,b11);
		ALOGD("bp2[%lle]	b12[%lle]	b21[%lle]	bp3[%lle]\n",bp2,b12,b21,bp3);
		ALOGD("<----------- float calibration data dones-------------->\n");
#endif
		return 1;
	}
	else
	{
		ALOGE("can not open %s", CALIDATA_PATH);
		return 0;
	}
}


void qmp6988_algo_deinit(void)
{
	if(qmp6988_cali_fd > 0)
	{
		close(qmp6988_cali_fd);
		qmp6988_cali_fd = -1;
	}
}



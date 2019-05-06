/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>

#include "PressureSensor.h"
#include "sensors.h"
#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG		"QMP6988"

#define FETCH_FULL_EVENT_BEFORE_RETURN 	1

#define	EVENT_TYPE_PRESSURE		ABS_X	//ABS_PRESSURE

#define CONVERT_PRESSURE		(0.01)

#define IGNORE_EVENT_TIME		0

/*****************************************************************************/
#define QMP6988_U16_t	unsigned short
#define QMP6988_S16_t	short
#define QMP6988_U32_t	unsigned int
#define QMP6988_S32_t	int
#define QMP6988_U64_t	long
#define QMP6988_S64_t	unsigned long

struct qmp6988_calibration_data {
	QMP6988_S32_t COE_a0;
	QMP6988_S16_t COE_a1;
	QMP6988_S16_t COE_a2;
	QMP6988_S32_t COE_b00;
	QMP6988_S16_t COE_bt1;
	QMP6988_S16_t COE_bt2;
	QMP6988_S16_t COE_bp1;
	QMP6988_S16_t COE_b11;
	QMP6988_S16_t COE_bp2;
	QMP6988_S16_t COE_b12;
	QMP6988_S16_t COE_b21;
	QMP6988_S16_t COE_bp3;
};

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

struct qmp6988_calibration_data qmp6988_cali;
static float a0,b00;
static float a1,a2,bt1,bt2,bp1,b11,bp2,b12,b21,bp3;
static int press_raw, temp_raw;
static float press_data, temp_data;

#define BROMETER							0X87
#define BAROMETER_GET_CALIBRATION_DATA		_IOR(BROMETER, 0x06, struct qmp6988_calibration_data)

// add by yangzhiqiang
void qmp6988_get_calibration(void)
{
	int fd;
	char calibuf[256];

#if 0
	fd = open("/dev/qmp6988_misc", O_RDWR);
#else
	fd = open("/sys/class/misc/qmp6988_misc", O_RDONLY);
#endif
	if(fd>0)
	{
		int err;
#if 0
		err = ioctl(fd, BAROMETER_GET_CALIBRATION_DATA, &qmp6988_cali);
		ALOGD("ioctl err = %d", err);
#else
		memset(calibuf, 0, sizeof(calibuf));
		read(fd, calibuf, sizeof(calibuf)-1);
		sscanf(calibuf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
			&qmp6988_cali.COE_a0,&qmp6988_cali.COE_a1,&qmp6988_cali.COE_a2,
			&qmp6988_cali.COE_b00,&qmp6988_cali.COE_bt1,&qmp6988_cali.COE_bt2,
			&qmp6988_cali.COE_bp1,&qmp6988_cali.COE_b11,
			&qmp6988_cali.COE_bp2,&qmp6988_cali.COE_b12,
			&qmp6988_cali.COE_b21,&qmp6988_cali.COE_bp3);
#endif
		close(fd);
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
	}
	else
	{
		ALOGD("can not open /dev/qmp6988_misc");
	}
}

void qmp6988_convert(int Dp, int Dt) 
{
	double Tr = 0,T = 0;
	double Pr = 0;

	Tr = a0 + a1*Dt + a2*Dt*Dt;
	//Unit centigrade
	T = Tr / 256.0f;  

	ALOGD("Tr [%f] T[%f]\n",Tr,T);
	//compensation pressure, Unit Pa
	Pr = b00 + bt1*Tr + bp1*Dp + b11*Tr*Dp + bt2*Tr*Tr + bp2*Dp*Dp + b12*Dp*Tr*Tr + b21*Dp*Dp*Tr + bp3*Dp*Dp*Dp;
	ALOGD("Pr [%f]",Pr);

	press_data = (float)Pr/100.0f;
	temp_data = T;
}
// yangzhiqiang

PressureSensor::PressureSensor()
	: SensorBase(NULL, "qmp6988"),
	  mInputReader(4),
	  mHasPendingEvent(false),
	  mEnabledTime(0)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = SENSORS_PRESSURE_HANDLE;
	mPendingEvent.type = SENSOR_TYPE_PRESSURE;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	if(data_fd) {
		strlcpy(input_sysfs_path, "/sys/class/input/", sizeof(input_sysfs_path));
		strlcat(input_sysfs_path, input_name, sizeof(input_sysfs_path));
		strlcat(input_sysfs_path, "/device/device/", sizeof(input_sysfs_path));
		input_sysfs_path_len = strlen(input_sysfs_path);
		enable(0, 1);
	}	
	qmp6988_get_calibration();
}

PressureSensor::PressureSensor(struct SensorContext *context)
	: SensorBase(NULL, NULL, context),
	  mInputReader(4),
	  mHasPendingEvent(false),
	  mEnabledTime(0)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = context->sensor->handle;
	mPendingEvent.type = SENSOR_TYPE_PRESSURE;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
	data_fd = context->data_fd;
	strlcpy(input_sysfs_path, context->enable_path, sizeof(input_sysfs_path));
	input_sysfs_path_len = strlen(input_sysfs_path);
	mUseAbsTimeStamp = false;
	enable(0, 1);
	qmp6988_get_calibration();
}


PressureSensor::PressureSensor(char *name)
	: SensorBase(NULL, "qmp6988"),
	  mInputReader(4),
	  mHasPendingEvent(false),
	  mEnabledTime(0)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = SENSORS_PRESSURE_HANDLE;
	mPendingEvent.type = SENSOR_TYPE_PRESSURE;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	if (data_fd) {
		strlcpy(input_sysfs_path, SYSFS_CLASS, sizeof(input_sysfs_path));
		strlcat(input_sysfs_path, name, sizeof(input_sysfs_path));
		strlcat(input_sysfs_path, "/", sizeof(input_sysfs_path));
		input_sysfs_path_len = strlen(input_sysfs_path);
		ALOGI("The pressure sensor path is %s",input_sysfs_path);
		enable(0, 1);
	}
	qmp6988_get_calibration();
}

PressureSensor::~PressureSensor() {
	if (mEnabled) {
		enable(0, 0);
	}
}

int PressureSensor::setInitialState() {
	struct input_absinfo absinfo;
	float value;
	if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_PRESSURE), &absinfo)) {
		value = absinfo.value;
		mPendingEvent.pressure = value * CONVERT_PRESSURE;
		mHasPendingEvent = true;
	}
	return 0;
}

int PressureSensor::enable(int32_t, int en) {
	int flags = en ? 1 : 0;
	if (flags != mEnabled) {
		int fd;
		strlcpy(&input_sysfs_path[input_sysfs_path_len],
				SYSFS_ENABLE, SYSFS_MAXLEN);
		fd = open(input_sysfs_path, O_RDWR);
		if (fd >= 0) {
			char buf[2];
			int err;
			buf[1] = 0;
			if (flags) {
				buf[0] = '1';
				mEnabledTime = getTimestamp() + IGNORE_EVENT_TIME;
			} else {
				buf[0] = '0';
			}
			err = write(fd, buf, sizeof(buf));
			close(fd);
			mEnabled = flags;
			return 0;
		}
		return -1;
	}
	return 0;
}

bool PressureSensor::hasPendingEvents() const {
	return mHasPendingEvent || mHasPendingMetadata;
}

int PressureSensor::setDelay(int32_t, int64_t delay_ns)
{
	int fd;
	int delay_ms = delay_ns / 1000000;
	strlcpy(&input_sysfs_path[input_sysfs_path_len],
			SYSFS_POLL_DELAY, SYSFS_MAXLEN);
	fd = open(input_sysfs_path, O_RDWR);
	if (fd >= 0) {
		char buf[80];
		snprintf(buf, sizeof(buf), "%d", delay_ms);
		write(fd, buf, strlen(buf)+1);
		close(fd);
		return 0;
	}
	return -1;
}

int PressureSensor::readEvents(sensors_event_t* data, int count)
{
	if (count < 1)
		return -EINVAL;

	if (mHasPendingEvent) {
		mHasPendingEvent = false;
		mPendingEvent.timestamp = getTimestamp();
		*data = mPendingEvent;
		return mEnabled ? 1 : 0;
	}

	if (mHasPendingMetadata) {
		mHasPendingMetadata--;
		meta_data.timestamp = getTimestamp();
		*data = meta_data;
		return mEnabled ? 1 : 0;
	}

	ssize_t n = mInputReader.fill(data_fd);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;

#if FETCH_FULL_EVENT_BEFORE_RETURN
again:
#endif
	while(count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS) {
			float value = event->value;
			if(event->code == ABS_X)
			{
				press_raw = value;
			}
			else if(event->code == ABS_Y)
			{
				temp_raw = value;
				qmp6988_convert(press_raw, temp_raw);
			}
			mPendingEvent.pressure = press_data;
		} else if (type == EV_SYN) {
			switch (event->code) {
				case SYN_TIME_SEC:
					mUseAbsTimeStamp = true;
					report_time = event->value*1000000000LL;
					break;
				case SYN_TIME_NSEC:
					mUseAbsTimeStamp = true;
					mPendingEvent.timestamp = report_time+event->value;
					break;
				case SYN_REPORT:
					if(mUseAbsTimeStamp != true) {
						mPendingEvent.timestamp = timevalToNano(event->time);
					}
					if (mEnabled) {
						if (mPendingEvent.timestamp >= mEnabledTime) {
							*data++ = mPendingEvent;
							numEventReceived++;
						}
						count--;
					}
					break;
			}
		} else {
			ALOGE("PressureSensor: unknown event (type=%d, code=%d)",
					type, event->code);
		}
		mInputReader.next();
	}

#if FETCH_FULL_EVENT_BEFORE_RETURN
	/* if we didn't read a complete event, see if we can fill and
	   try again instead of returning with nothing and redoing poll. */
	if (numEventReceived == 0 && mEnabled == 1) {
		n = mInputReader.fill(data_fd);
		if (n)
			goto again;
	}
#endif

	return numEventReceived;
}


/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
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

#include "StepCounter.h"
#include <utils/SystemClock.h>
#include <utils/Timers.h>
#ifdef LOG_TAG
#undef   LOG_TAG
#define LOG_TAG "STEP_COUNTER"
#endif

#define QMAX981_ACC_DEV_PATH_NAME    "/dev/qmax981"
#define IGNORE_EVENT_TIME 350000000

#define QMAX981_ACC_IOCTL_BASE                   77
/** The following define the IOCTL command values via the ioctl macros */
#define QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE		_IOW(QMAX981_ACC_IOCTL_BASE,5,int)
#define QMAX981_ACC_IOCTL_GET_STEPCOUNT_ENABLE		_IOR(QMAX981_ACC_IOCTL_BASE,6,int)


static struct sensor_t sSensorList[] = {
#if 1
	{
		.name		= "QMAX981 Step Counter",
		.vendor 	= "QST",
		.version	= 1,
		.handle 	= SENSORS_STEP_COUNTER_HANDLE,
		.type		= SENSOR_TYPE_STEP_COUNTER,
		.maxRange	= 65535.0f,//32.0f,
		.resolution = 1.0f,//4.0f/1024.0f,
		.power		= 5000,//130.0f/1000.0f,
		.minDelay	= 5,
		.fifoReservedEventCount = 0,
		.fifoMaxEventCount = 0,
		.reserved	= {}
	}
#endif
#if 0
	{
		"QMAX981 Step Counter",
		"QST",
		1,
		SENSORS_STEP_COUNTER_HANDLE,
		SENSOR_TYPE_STEP_COUNTER,
		65535.0f,
		1.0f,
		0.2f,
		5000,
		0,
		0,
		SENSOR_STRING_TYPE_STEP_COUNTER,
		"",
		1000000,
		SENSOR_FLAG_CONTINUOUS_MODE,
		{}
	},
#endif
};
/*****************************************************************************/
StepCounterSensor::StepCounterSensor() : SensorBase(QMAX981_ACC_DEV_PATH_NAME, "step_counter"),
      mEnabled(0),
	  mInputReader(32),
	  mHasPendingEvent(false),
      mDelay(-1),
      mSensorCoordinate()
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_STEP_COUNTER;
    mPendingEvent.type = SENSOR_TYPE_STEP_COUNTER;
    memset(mPendingEvent.data, 0x00, sizeof(mPendingEvent.data));
	
    mEnabledTime =0;
    mPendingEvent.timestamp =0;

	open_device();

    ALOGD("StepCounterSensor::StepCounterSensor(StepCounter.cpp): construct called");
}


StepCounterSensor::~StepCounterSensor() {
	close_device();
}

int StepCounterSensor::setInitialState()
{
	struct input_absinfo absinfo;
	int clockid = CLOCK_BOOTTIME;

	if (mEnabled) {
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_STEP_COUNTER_VALUE), &absinfo)) {
			mPendingEvent.u64.step_counter = (absinfo.value);
		}
	}
	
	if (!ioctl(data_fd, EVIOCSCLOCKID, &clockid))
	{
		ALOGD("StepCounterSensor: set EVIOCSCLOCKID = %d\n", clockid);
	}
	else
	{
		ALOGE("StepCounterSensor: set EVIOCSCLOCKID failed \n");
	}	
		
	return 0;
}

int StepCounterSensor::setEnable(int32_t handle, int enabled)
{
	int err;
    int flags = enabled ? 1 : 0;

    mEnabled = flags;
	
	if (flags) 
	{
		setInitialState();
		ALOGD("setEnable dev_fd[%d]\n",dev_fd);
		mEnabledTime = getTimestamp() + IGNORE_EVENT_TIME;
		err = ioctl(dev_fd, QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE,
				  &enabled);
		ALOGD("StepCounterSensor:enable ioctl true");
	}
	else  
	{
		ALOGD("StepCounterSensor:enable ioctl false");
		err = ioctl(dev_fd, QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE,
				  &enabled);
	}
	 
	if (err != 0) {
			ALOGE("StepCounterSensor: IOCTL failed (%s)", strerror(errno));
			return err;
	}
	
    ALOGD("step counter enable(%d) done", mEnabled );
    return 0;
}

int StepCounterSensor::getEnable(int32_t handle)
{
	return (handle == ID_STEP_COUNTER) ? mEnabled : 0;
}

int StepCounterSensor::setDelay(int32_t handle, int64_t ns)
{
       //int fd;
	mDelay = ns;
    ALOGD("setDelay: regardless of the setDelay() value (handle=%d, ns=%lld)",handle, ns);

    return 0;

}

int64_t StepCounterSensor::getDelay(int32_t handle)
{
	return (handle == ID_STEP_COUNTER ) ? mDelay : 0;
}

bool StepCounterSensor::hasPendingEvents() const
{
        return mHasPendingEvent;
}

int StepCounterSensor::readEvents(sensors_event_t* data, int count)
{

    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
	ALOGD("StepCounterSensor %d %d\r\n",n,data_fd);
    if (n < 0)
        return n;
    int numEventReceived = 0;
    input_event const* event;

	ALOGD("StepCounterSensor.readEvents\r\n");
    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        //ALOGE("StepCounterSensor.readEvents\r\n");
        if (type == EV_ABS)
        {
            processEvent(event->code, event->value);
			 mInputReader.next();
            //ALOGE("fwq2....\r\n");
        }
        else if (type == EV_SYN)
        {
            ALOGE("readEvents....EV_SYN\r\n");
            int64_t time = timevalToNano(event->time);//systemTime(SYSTEM_TIME_MONOTONIC);//timevalToNano(event->time);
            mPendingEvent.timestamp = time;
            if (mEnabled)
            {
                 ALOGE("readEvents....mEnabled[%d]\r\n",mEnabled);
                // if (mPendingEvent.timestamp >= mEnabledTime)
                 {
                    //ALOGE("fwq5....\r\n");
                     *data++ = mPendingEvent;
                    numEventReceived++;
                 }
                 count--;

            }
			 mInputReader.next();
        }
        else
        {
            ALOGE("step: unknown event (type=%d, code=%d)",
                    type, event->code);
			mInputReader.next();
        }

    }

    //ALOGE("fwq read Event 2\r\n");
    return numEventReceived;
}

void StepCounterSensor::processEvent(int code, int value)
{
    ALOGD("processEvent code=%d,value=%d\r\n",code, value);
    //uint64_t stepcount=0;
    switch (code) {
    case EVENT_TYPE_STEP_COUNTER_VALUE:
        mPendingEvent.sensor = ID_STEP_COUNTER;
        mPendingEvent.type = SENSOR_TYPE_STEP_COUNTER;
        mPendingEvent.u64.step_counter= value;
        break;
    }
}

int StepCounterSensor::populateSensorList(struct sensor_t *list)
{
        memcpy(list, sSensorList, sizeof(struct sensor_t) * numSensors);
        return numSensors;
}

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
#include <dlfcn.h>
#include <string.h>

#include <hardware/sensors.h>

#include <pthread.h>

#include <linux/input.h>

#include <cutils/atomic.h>


#include <cutils/sockets.h>
#include <cutils/atomic.h>
#include <cutils/log.h>
#include "mir3daSensor.h"

#define INPUT_DEV_NAME_BMA 			"accelerometer"     //Bosch Sensortec bma220 product
#define SYSFS_PATH_CLASS_INPUT 		"/sys/class/input/input"
#define SYSFS_NODE_BMP_ENABLE 		"enable"
#define SYSFS_NODE_BMP_DELAY 		"poll_delay"
#define CONVERT                     (GRAVITY_EARTH / 1024)
#define CONVERT_X                   (CONVERT)
#define CONVERT_Y                   -(CONVERT)
#define CONVERT_Z                   -(CONVERT)

static struct sensor_t sSensorList[] = {
	{ 	"Mir3da 3-axis Accelerometer",
        "Miramems",
        1,
        SENSORS_ACCELERATION_HANDLE,
        SENSOR_TYPE_ACCELEROMETER,
		(float)4.0f*9.81f,
		(float)(9.81f)/1024.0f,
		(float)0.2f,
		10000,
		0,
		0,
		SENSOR_STRING_TYPE_ACCELEROMETER,
		"",
		1000000,
		SENSOR_FLAG_CONTINUOUS_MODE,
		{ }
	},
};


#if 0
static  struct sensor_t sSensorList[] = {

	{
		"MiraMEMS  3-axis Accelerometer",
		"MiraMEMS.",
		1, SENSORS_ACCELERATION_HANDLE,
		SENSOR_TYPE_ACCELEROMETER, (GRAVITY_EARTH * 2.0f),
		(GRAVITY_EARTH)/ 1024.0f, 0.145f, 10000, { }
	},
};
#endif

static int input_get_event_num(const char *pname)
{
        int num = -1;
        int i;
        char filename[64];
        int err;
        char ev_name[64];
        int fd;

        for (i = 0; i < 256; i++) {
                sprintf(filename, "/dev/input/event%d", i);
                fd = open(filename, O_RDONLY);
                if (fd >= 0) {
                        err = ioctl(fd, EVIOCGNAME(sizeof(ev_name) - 1), &ev_name);
                        ALOGI("err: %d ev_name: %s", err, ev_name);
                        if (err < 1) {
                                ev_name[0] = '\0';
                        }

                        if (0 == strcmp(ev_name, pname)) {
                                num = i;
                                close(fd);
                                break;
                        }

                        close(fd);
                }
        }

        return num;
}

static int txt_file_write_int(const char *path, int value)
{
        int fd;

        fd = open(path, O_RDWR);
        if (fd >= 0) {
                char buffer[32] = "";
                int bytes = sprintf(buffer, "%d\n", value) + 1;
                int amt = write(fd, buffer, bytes);
                close(fd);
                return amt == -1 ? -errno : 0;
        } else {
                ALOGE("failed to open %s, reason: %s\n",
                                path, (char *)strerror(errno));
                return -errno;
        }
}

AccSensor::AccSensor()
: SensorBase(NULL, INPUT_DEV_NAME_BMA),
        mInputReader(32)        //TODO
{
        ALOGI("init of mir3da sensor\n");
        mInputDevNum = input_get_event_num(INPUT_DEV_NAME_BMA);

        if (-1 == mInputDevNum) {
                ALOGE("error init mir3da sensor\n");
                return;
        }

        ALOGI("mir3da sensor input dev num: %d\n",
                        mInputDevNum);

        memset(&mPendingEvents, 0, sizeof(sensors_event_t));
        mPendingEvents.version = sizeof(sensors_event_t);
        mPendingEvents.sensor = ID_A;
        mPendingEvents.type = SENSOR_TYPE_ACCELEROMETER;
        mDelay = 200;
        mEnabled = 0;
}

AccSensor::~AccSensor()
{
        ALOGI("deinit of mir3da sensor");
        close_device();
}

int AccSensor::getEnable(int32_t handle)
{
	return (handle == ID_A) ? mEnabled : 0;
}

int AccSensor::setEnable(int32_t handle, int en)
{
        int err = 0;
        char buf[68] = "";
        int clockid = CLOCK_BOOTTIME;

        ALOGD("mir3da sensor enable %d \n", en);

        if (1 == en) {
               if (!ioctl(data_fd, EVIOCSCLOCKID, &clockid))
                       ALOGD("AccSensor: set EVIOCSCLOCKID = %d", clockid);
               else
                       ALOGE("AccSensor: set EVIOCSCLOCKID failed");
        }
        if (-1 == mInputDevNum) {
                ALOGE("no bma220 sensor\n");
                return -ENODEV;
        }

        if (handle != ID_A) {
                ALOGW("unexpected command, discarded\n");
                return -EINVAL;
        }

        en = en ? 1 : 0;
        if (mEnabled == en) {
                ALOGW("duplicate command to enable: %d\n", en);
                return 0;
        }

        sprintf(buf, "%s%d/%s",
                        SYSFS_PATH_CLASS_INPUT, mInputDevNum, SYSFS_NODE_BMP_ENABLE);

        ALOGV("path of delay: %s", buf);

        err = txt_file_write_int(buf, en);
        if (err) {
                ALOGE("fail to enable: %d\n", en);
        } else {
                mEnabled = en;
        }

        return err;
}


int64_t AccSensor::getDelay(int32_t handle)
{
	return (handle == ID_A) ? mDelay : 0;
}

int AccSensor::setDelay(int32_t handle, int64_t ns)
{
        int err = 0;
        char buf[68] = "";

        ALOGD("mir3da sensor setDelay %d\n",
                        (int)(ns / 1000000));

        if (-1 == mInputDevNum) {
                ALOGE("no mir3da sensor\n");
                return -ENODEV;
        }

        if (handle != ID_A) {
                ALOGW("unexpected command, discarded\n");
                return -EINVAL;
        }

        sprintf(buf, "%s%d/%s",
                        SYSFS_PATH_CLASS_INPUT, mInputDevNum, SYSFS_NODE_BMP_DELAY);

        ALOGV("path of delay: %s", buf);

        err = txt_file_write_int(buf, (int)(ns / 1000000));
        if (err) {
                ALOGE("fail to set delay: %d, err: %d\n",
                                mDelay, err);
        } else {
                mDelay = (int)(ns / 1000000);
        }

        return err;
}


int AccSensor::readEvents(sensors_event_t* data, int count)
{
        int numEventReceived = 0;
     //   int type;
     //   int64_t time;
      //  int code;

       // LOGD("readEvent count = %d\n", count);

        if (-1 == mInputDevNum) {
                ALOGE("no bma220 sensor\n");
                return 0;
        }

        if (count < 0) {
                ALOGE("buffer size error: %d\n", count);
                return 0;
        }

        ssize_t n = mInputReader.fill(data_fd);
        if (n < 0) {
                ALOGE("no mir3da event available: %d", (int)n);
                return 0;
        }

        input_event const *event;

       while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
            float value = event->value;
            if (event->code == EVENT_TYPE_ACCEL_X) {
                mPendingEvents.data[0] = value * CONVERT_X;
            } else if (event->code == EVENT_TYPE_ACCEL_Y) {
                mPendingEvents.data[1] = value * CONVERT_Y;
            } else if (event->code == EVENT_TYPE_ACCEL_Z) {
                mPendingEvents.data[2] = value * CONVERT_Z;
            }
        } else if (type == EV_SYN) {
            mPendingEvents.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                *data++ = mPendingEvents;
                count--;
                numEventReceived++;
            }
        } else {
            ALOGE("AccelSensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mInputReader.next();
    }

        return numEventReceived;
}

int AccSensor::populateSensorList(struct sensor_t *list)
{
	memcpy(list, sSensorList, sizeof(struct sensor_t) * numSensors);
	return numSensors;
}


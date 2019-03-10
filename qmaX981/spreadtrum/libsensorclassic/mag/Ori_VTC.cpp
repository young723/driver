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
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <dlfcn.h>

#include <cutils/log.h>
#include <cutils/properties.h>

#include "OriSensor.h"
#include "vtc_library/vtc_lib.h"
//#include "vtc_library/vtc_type_def.h"

#include "vtc_library/vtc_cust.h"
#include "vtc_library/vtc_chip.h"
#include "vtc_library/vtc_math.h"
#include "vtc_library/vtc_type_def.h"

#define VTC_SYSFS_PATH	   "/sys/class/compass/msensor/"

#define MAG_INPUT_DEV_NAME "compass"

#define EVENT_TYPE_MAG_X		ABS_X
#define EVENT_TYPE_MAG_Y		ABS_Y
#define EVENT_TYPE_MAG_Z		ABS_Z
//=======================================================================
float acc_data[3];
float mag_data[3];
float gyro_data[3];
//int odr;
//=======================================================================

#if 0
static struct sensor_t sSensorList[] = {
#if 0 //for AF7133E/AF8133i/AF8133J/AF9133
  {
    "AFx133 Magnetic field sensor",
	  "Voltafield",
	  1,
	  SENSORS_MAGNETIC_FIELD_HANDLE,
	  SENSOR_TYPE_MAGNETIC_FIELD,
	  2200.0f,
	  0.036,
	  0.8f,
	  10000,
	  0,
	  0,
	  {}
	},
#else //for AF6133

	{
    "AF6133 Magnetic field sensor",
	  "Voltafield",
	  1,
	  SENSORS_MAGNETIC_FIELD_HANDLE,
	  SENSOR_TYPE_MAGNETIC_FIELD,
	  1600.0f,
	  0.2,
	  0.8f,
	  10000,
	  0,
	  0,
	  {}
	},
#endif
	{
    "VTC Orientation sensor",
	  "Voltafield",
	  1,
    SENSORS_ORIENTATION_HANDLE,
    SENSOR_TYPE_ORIENTATION,
	  360.0f,
	  1.0,
	  0.27f,
	  10000,
	  0,
	  0,
	  {}
	},
};
#endif
static struct sensor_t sSensorList[] = {
	{
      "AFx133 Magnetic field sensor",
	  "Voltafield",
	  1,
	  SENSORS_MAGNETIC_FIELD_HANDLE,
	  SENSOR_TYPE_MAGNETIC_FIELD,
	  1600.0f,
	  0.2,
	  0.8f,
	  10000,
	  0,
	  0,
	  SENSOR_STRING_TYPE_MAGNETIC_FIELD,
	  "",
	  100000000,
	  0,
	  {}
	},
	{
      "VTC Orientation sensor",
	  "Voltafield",
	  1,
      SENSORS_ORIENTATION_HANDLE,
      SENSOR_TYPE_ORIENTATION,
	  360.0f,
	  1.0,
	  0.27f,
	  10000,
	  0,
	  0,
	  SENSOR_STRING_TYPE_MAGNETIC_FIELD,
	  "",
	  100000000,
	  0,
	  {}
	  },
};
OriSensor::OriSensor() :
	SensorBase(NULL, MAG_INPUT_DEV_NAME),
    mPendingMask(0),
    mInputReader(4)
{
	INT16 map_axis[3] = {1, 2, 3};

    ALOGE("HJDDbgMag, SensorBase");
	for (int i = 0; i < __numSensors; i++) {
		mEnabled[i] = 0;
		mDelay[i] = -1;
	}

	for (int i = 0; i < 3; i++)
	{
	  acc_data[i] = 0;
	  mag_data[i] = 0;
	  gyro_data[i] = 0;
	}

    mMagEnabled = 0;

	mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
	mPendingEvents[MagneticField].sensor = ID_M;
	mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
	mPendingEvents[MagneticField].magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;

	mPendingEvents[Orientation].version = sizeof(sensors_event_t);
	mPendingEvents[Orientation].sensor = ID_O;
	mPendingEvents[Orientation].type = SENSOR_TYPE_ORIENTATION;
	mPendingEvents[Orientation].orientation.status = SENSOR_STATUS_ACCURACY_HIGH;

	strncpy(input_sysfs_path, VTC_SYSFS_PATH, strlen(VTC_SYSFS_PATH));
	input_sysfs_path_len = strlen(input_sysfs_path);

    //initial calibrating parameters
    vtc_algo_initial();
    vtc_odr_change(10);
    vtc_set_mag_axis(map_axis);
}

OriSensor::~OriSensor() {
}

int OriSensor::setEnable(int32_t handle, int enabled)
{
	int idx = handle2id(handle);
	int err = 0;
	int flags = -1;
	int i;

    ALOGE("HJDDbgMag, setEnable, handle=%d, en=%d, idx=%d", handle, enabled, idx);

  if (idx >= __numSensors)
	return -EINVAL;

	if (enabled) {
		(mEnabled[idx])++;
		if (mEnabled[idx] > 32767)
			mEnabled[idx] = 32767;
	}
	else {
		(mEnabled[idx])--;
		if (mEnabled[idx] < 0)
			mEnabled[idx] = 0;
	}

  for(i=0;i<__numSensors;i++)
    if(mEnabled[i] > 0)
    {
      if(mMagEnabled == 0)
        flags = 1;
      break;
    }

    ALOGE("HJDDbgMag, setEnable, flags=%d", flags);
  if(flags != -1) {
    int fd;

    strcpy(&input_sysfs_path[input_sysfs_path_len], "enable_device");
    fd = open(input_sysfs_path, O_RDWR);

    ALOGE("HJDDbgMag, setEnable, fd=%d, input_sysfs_path=%s", fd, input_sysfs_path); // -1
    if (fd >= 0) {
      char buf[2];
      buf[1] = '\0';
      if (flags) {
        buf[0] = '1';
        mMagEnabled = 1;
      } else {
        buf[0] = '0';
        mMagEnabled = 0;
      }
      err = write(fd, buf, sizeof(buf));
    ALOGE("HJDDbgMag, setEnable, write, err=%d", err);

      close(fd);
      return 0;
    }
    return fd;
  }
  return 0;
}

int OriSensor::getEnable(int32_t handle)
{
	int idx = handle2id(handle);

  if (idx >= __numSensors)
	  return -EINVAL;

	return (mEnabled[idx]);
}

int OriSensor::setDelay(int32_t handle, int64_t ns)
{
	int idx = handle2id(handle);

    ALOGE("HJDDbgMag, setDelay, idx=%d", idx);

	if (idx >= __numSensors)
	  return -EINVAL;

  strcpy(&input_sysfs_path[input_sysfs_path_len], "pollrate_ms");
  int fd = open(input_sysfs_path, O_RDWR);

    ALOGE("HJDDbgMag, setDelay, fd=%d, input_sysfs_path=%s", fd, input_sysfs_path);
  if (fd >= 0) {
	  char buf[80];
	  int delay_ms = ns / 1000000;
	  int odr;

    if(delay_ms > 100)
      delay_ms = 100;
    else if(delay_ms < 10)
      delay_ms = 10;

    mDelay[idx] = ns;

   odr = (int)(1000 / delay_ms);
  // odr = (int)(1000 / 100);
    vtc_odr_change(odr);
  //  ALOGE("afx133, odr=%d\n", odr);

    int length = sprintf(buf, "%d\n", delay_ms);
    int err = write(fd, buf, length);
    ALOGE("HJDDbgMag, setDelay, write, err=%d", err);
    close(fd);
    //LOGD("%s: err = %d\n", __func__, err);
    return 0;
  }

	return fd;
}

int64_t OriSensor::getDelay(int32_t handle)
{
	int idx = handle2id(handle);

	if (idx >= __numSensors)
	  return -EINVAL;

	return (mDelay[idx]);
}

int OriSensor::setAccel(sensors_event_t * data)
{
	int err=0;

	/* Input data is already formated to Android definition. */
	acc_data[0] = (data->acceleration.x);
	acc_data[1] = (data->acceleration.y);
	acc_data[2] = (data->acceleration.z);

	return err;
}

int OriSensor::readEvents(sensors_event_t * data, int count)
{
	static AF_ValueTypeDef event_val;

    // ALOGE("HJDDbgMag, readEvents, count=%d", count);

	if (count < 1)
		return -EINVAL;

	ssize_t n = mInputReader.fill(data_fd);

    // ALOGE("HJDDbgMag, readEvents, n=%d 1", n);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;

    // ALOGE("HJDDbgMag, readEvents, n=%d 2", n);
	while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS) {
			float value = event->value;
			if (event->code == EVENT_TYPE_MAG_X) {
				event_val.mx=(float)((value > 32767) ? (value-65536) : value);
			} else if (event->code == EVENT_TYPE_MAG_Y) {
				event_val.my=(float)((value > 32767) ? (value-65536) : value);
			} else if (event->code == EVENT_TYPE_MAG_Z) {
				event_val.mz=(float)((value > 32767) ? (value-65536) : value);
			}
		} else if (type == EV_SYN) {
			int64_t time = timevalToNano(event->time);

      //==================VTC algorithm==========================================
     #if 0
      mag_data[0] = event_val.mx * RESOLUTION_M;
      mag_data[1] = event_val.my * RESOLUTION_M;
      mag_data[2] = event_val.mz * RESOLUTION_M;
      // ALOGE("HJDDbgMag, readEvents, mag_data[0]=%d, mag_data[1]=%d, mag_data[2]=%d", mag_data[0], mag_data[1], mag_data[2]);
    #endif
      mag_data[0] = event_val.my * RESOLUTION_M;
      mag_data[1] = event_val.mx * RESOLUTION_M;
      mag_data[2] = event_val.mz * RESOLUTION_M;

      if(mEnabled[MagneticField])
      {
        float mag_cali[3];
        int16_t accuracy_level;
        uint32_t utime = time / 1000;

        vtc_algo_run(acc_data, mag_data, gyro_data, utime);
        vtc_get_magneticfield(mag_cali, &accuracy_level);

        mPendingEvents[MagneticField].magnetic.x = mag_cali[0];//mag_cali[0];
        mPendingEvents[MagneticField].magnetic.y = mag_cali[1];//mag_cali[1];
        mPendingEvents[MagneticField].magnetic.z = mag_cali[2];//mag_cali[2];
        mPendingEvents[MagneticField].magnetic.status = accuracy_level;
        mPendingMask |= 1<<MagneticField;
      }

      if(mEnabled[Orientation])
      {
        float ori_data[3];
        int16_t accuracy_level;

        vtc_get_orientation(ori_data, &accuracy_level);

        mPendingEvents[Orientation].orientation.x = ori_data[0];
        mPendingEvents[Orientation].orientation.y = ori_data[1];
        mPendingEvents[Orientation].orientation.z = ori_data[2];
        mPendingEvents[Orientation].orientation.status = accuracy_level;
        mPendingMask |= 1<<Orientation;
      }
      //ALOGE("afx133, odr=%d\n", odr);
      #if 1
        ALOGE("afx133, sfi_array[X]=%f %f %f\n", sfi_array[0],sfi_array[1],sfi_array[2]);
        ALOGE("afx133, sfi_array[Y]=%f %f %f\n", sfi_array[3],sfi_array[4],sfi_array[5]);
        ALOGE("afx133, sfi_array[Z]=%f %f %f\n", sfi_array[6],sfi_array[7],sfi_array[8]);
        ALOGE("afx133, vtc_cali log 00-02=%d %d %d\n", vtc_cali_log[0],vtc_cali_log[1],vtc_cali_log[2]);
        ALOGE("afx133, vtc_cali log 03-05=%d %d %d\n", vtc_cali_log[3],vtc_cali_log[4],vtc_cali_log[5]);
        ALOGE("afx133, vtc_cali log 06-08=%d %d %d\n", vtc_cali_log[6],vtc_cali_log[7],vtc_cali_log[8]);
        ALOGE("afx133, vtc_cali log 09-11=%d %d %d\n", vtc_cali_log[9],vtc_cali_log[10],vtc_cali_log[11]);
        ALOGE("afx133, vtc_cali log 12-14=%d %d %d\n", vtc_cali_log[12],vtc_cali_log[13],vtc_cali_log[14]);
        ALOGE("afx133, mag_cali_check_err = %d\n", mag_cali_check_err);

      #endif

      //========================================================================

			for (int j = 0;   count && mPendingMask && j < __numSensors; j++) {
				if (mPendingMask & (1 << j)) {
					mPendingMask &= ~(1 << j);
					mPendingEvents[j].timestamp = time;

				  *data++ = mPendingEvents[j];
					count--;
					numEventReceived++;
				}
            }
          }
		else {
          ALOGD("VTC debug VTCSensor: unknown event (type=%d, code=%d)", type, event->code);
		}

		mInputReader.next();
	} //while(1)

	return numEventReceived;
}

int OriSensor::handle2id(int32_t handle)
{
	switch (handle) {
	case ID_M:
		return MagneticField;
	case ID_O:
		return Orientation;
	default:
		ALOGE("OriSensor: unknown handle (%d)", handle);
		return -EINVAL;
	}
}

int OriSensor::populateSensorList(struct sensor_t *list)
{
	memcpy(list, sSensorList, sizeof(struct sensor_t) * __numSensors);
	return __numSensors;
}

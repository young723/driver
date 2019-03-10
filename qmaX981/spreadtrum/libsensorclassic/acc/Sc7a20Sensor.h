/*
 * Copyright (C) 2011 The Android Open Source Project
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

#ifndef ANDROID_Sc7a20_SENSOR_H
#define ANDROID_Sc7a20_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"

//huafeizhou160823 add-s
#define SENSORS_ACCELERATION_HANDLE     0
#define SENSORS_MAGNETIC_FIELD_HANDLE   1
#define SENSORS_ORIENTATION_HANDLE      2
#define SENSORS_LIGHT_HANDLE            3
#define SENSORS_PROXIMITY_HANDLE        4
//huafeizhou160823 add-e
/*****************************************************************************/

struct input_event;

class AccSensor : public SensorBase {
    int			mEnabled;
    int64_t		mDelay;
    int 		mMinPollDelay;
    int 		mMaxPollDelay;
    int			mLayout;
    InputEventCircularReader	mInputReader;
    sensors_event_t	mPendingEvent;
    bool		mHasPendingEvent;
    char		input_sysfs_path[PATH_MAX];
    int			input_sysfs_path_len;
	char 		sysfs_enable[PATH_MAX];
	char 		sysfs_poll[PATH_MAX];
	char 		sysfs_poll_min[PATH_MAX];
	char 		sysfs_poll_max[PATH_MAX];

private:    
    int getLayout();
	int write_sysfs(char * filename,char * buf,int size);
	int read_sysfs(char * filename,char * buf,int size);
	int sensorBaseGetPollMin();
	int sensorBaseGetPollMax();
	int sensorBaseGetSysfsPath(const char* inputName);

public:
            AccSensor();
    virtual ~AccSensor();
    virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents() const;
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int setEnable(int32_t handle, int enabled);
    virtual int64_t getDelay(int32_t handle);
    virtual int getEnable(int32_t handle);
    virtual int populateSensorList(struct sensor_t *list);//huafeizhou60823 add

//huafeizhou160823 add-s
#ifndef ACC_NULL
    enum {
		Accelerometer = 0,
        numSensors
    };
#else
	static const int numSensors = 0;
#endif
//huafeizhou160823 add-e
};

/*****************************************************************************/

#endif  // ANDROID_Sc7a20_SENSOR_H

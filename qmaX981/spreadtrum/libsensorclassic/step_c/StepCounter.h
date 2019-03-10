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

#ifndef ANDROID_STEP_C_SENSOR_H
#define ANDROID_STEP_C_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"
#include "SensorCoordinate.h"


#define SENSORS_STEP_COUNTER_HANDLE		ID_STEP_COUNTER
#define EVENT_TYPE_STEP_COUNTER_VALUE	ABS_X

/*****************************************************************************/
struct input_event;

class StepCounterSensor : public SensorBase  {
public:
            StepCounterSensor();
    virtual ~StepCounterSensor();
    enum {
		Stepcounter = 0,
        numSensors
    };
	
	virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents() const;
    virtual int setDelay(int32_t handle, int64_t ns);
	virtual int64_t getDelay(int32_t handle);
    virtual int setEnable(int32_t handle, int enabled);
    virtual int getEnable(int32_t handle);
    virtual int populateSensorList(struct sensor_t *list);
private:
    int mEnabled;
    InputEventCircularReader mInputReader;
	sensors_event_t mPendingEvent;
	int64_t mDelay;
    int64_t mEnabledTime;
	bool mHasPendingEvent;
    char input_sysfs_path[PATH_MAX];
    int input_sysfs_path_len;
	SensorCoordinate mSensorCoordinate;	
	void processEvent(int code, int value);
    int setInitialState();
};

/*****************************************************************************/

#endif  // ANDROID_STEP_C_SENSOR_H

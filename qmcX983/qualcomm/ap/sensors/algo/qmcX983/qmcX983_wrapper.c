/*--------------------------------------------------------------------------
Copyright (c) 2014, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <float.h>
#include "CalibrationModule.h"
#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <fcntl.h>
#include <dlfcn.h>

#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/select.h>
#include <linux/input.h>

#include <hardware/sensors.h>

#define LOG_TAG "sensor_cal.common"
#include <utils/Log.h>
#include "qmc_common.h"
#include "qmc_api.h"

#define SENSOR_CAL_ALGO_VERSION 1
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))



struct sensor_cal_module_t SENSOR_CAL_MODULE_INFO;
static struct sensor_cal_algo_t algo_list[];

static int convert_magnetic(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{
	ALOGD("convert_magnetic mag[%f %f %f]\n",raw->magnetic.x,raw->magnetic.y,raw->magnetic.z);
	QMC_CalibMagProcess(raw,result);
	ALOGD("convert_magnetic calib_Mag,%f,%f,%f\n",result->magnetic.x,result->magnetic.y,result->magnetic.z);
	
	return 0;
}

static int get_orientation(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{
	QMC_ORIProcess(raw,result);
	return 0;
}

static int get_virtual_gyro(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{
	QMC_GyroProcess(raw,result);
	return 0;
}

static int get_rotation_vector(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{
	QMC_RVProcess(raw,result);
	return 0;
}

static int get_gravity(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{
	QMC_GRAProcess(raw,result);
	return 0;
}

static int get_la(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{
	QMC_LAProcess(raw,result);
	return 0;
}

static int cal_init(const struct sensor_cal_module_t *module)
{
	if(NULL == module)
	{
		ALOGD("qst call init,NULL module pointer.\n");
	}
	QMC_Init();
	return 0;
}

static void cal_deinit()
{
	ALOGD("%s called\n", __func__);
}

static int cal_get_algo_list(const struct sensor_cal_algo_t **algo)
{
	*algo = algo_list;
	return 0;
} 

static struct sensor_algo_methods_t algo_methods = {
	.convert = convert_magnetic,
	.config = NULL,
};

static struct sensor_algo_methods_t or_methods = {
	.convert = get_orientation,
	.config = NULL,
};

static struct sensor_algo_methods_t pg_methods = {
	.convert = get_virtual_gyro,
	.config = NULL,
};

static struct sensor_algo_methods_t rv_methods = {
	.convert = get_rotation_vector,
	.config = NULL,
};

static struct sensor_algo_methods_t gr_methods = {
	.convert = get_gravity,
	.config = NULL,
};

static struct sensor_algo_methods_t la_methods = {
	.convert = get_la,
	.config = NULL,
};

static const char* mag_match_table[] = {
	COMPASS_NAME,
	NULL
};

static const char* or_match_table[] = {
	ORIENTATION_NAME,
	NULL
};

static const char* pg_match_table[] = {
	"oem-pseudo-gyro",
	NULL
};

static const char* rv_match_table[] = {
	"oem-rotation-vector",
	NULL
};

static const char* gr_match_table[] = {
	"oem-gravity",
	NULL
};

static const char* la_match_table[] = {
	"oem-linear-acceleration",
	NULL
};

static struct sensor_cal_algo_t algo_list[] = {
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_MAGNETIC_FIELD,
		.compatible = mag_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &algo_methods,
	},
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_ORIENTATION,
		.compatible = or_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &or_methods,
	},
//#ifdef QMC_VIRTUAL_SENSORS
#if 0
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_GYROSCOPE,
		.compatible = pg_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &pg_methods,
	},
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_ROTATION_VECTOR,
		.compatible = rv_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &rv_methods,
	},
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_GRAVITY,
		.compatible = gr_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &gr_methods,
	},
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_LINEAR_ACCELERATION,
		.compatible = la_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &la_methods,
	},
#endif
};

static struct sensor_cal_methods_t cal_methods = {
	.init = cal_init,
	.deinit = cal_deinit,
	.get_algo_list = cal_get_algo_list,
};

struct sensor_cal_module_t SENSOR_CAL_MODULE_INFO = {
	.tag = SENSOR_CAL_MODULE_TAG,
	.id = "cal_module_qmcX983",
	.version = SENSOR_CAL_MODULE_VERSION,
	.vendor = "qst",
	.dso = NULL,
	.number = ARRAY_SIZE(algo_list),
	.methods = &cal_methods,
	.reserved = {0},
};

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
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <fcntl.h>
#include <dlfcn.h>

#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <linux/input.h>

#include "QMCD_API.h"


#define SENSOR_CAL_ALGO_VERSION 1

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define scalar 3.125
#define scalar2uT 31.25

struct sensor_cal_module_t SENSOR_CAL_MODULE_INFO;
static struct sensor_cal_algo_t algo_list[];

struct hwmsen_convert {
	signed char sign[4];
	unsigned char map[4];
};

struct hwmsen_convert map[] = {
    { { 1, 1, 1}, {0, 1, 2} },
    { {-1, 1, 1}, {1, 0, 2} },
    { {-1,-1, 1}, {0, 1, 2} },
    { { 1,-1, 1}, {1, 0, 2} },

    { {-1, 1,-1}, {0, 1, 2} },
    { { 1, 1,-1}, {1, 0, 2} },
    { { 1,-1,-1}, {0, 1, 2} },
    { {-1,-1,-1}, {1, 0, 2} },      

};
struct hwmsen_convert cvt;
static sensors_event_t mAccelEvent;
static sensors_event_t mMagneticEvent;
static int direction = 0;
static int OTP[2] = {0};

float mag_raw[3] = {0};

static char *sys_input_layout = "/sys/class/misc/msensor/mag_layout";
static char *sys_input_otp = "/sys/class/misc/msensor/mag_otp";

static int convert_magnetic(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{	
	D("convert_magnetic mag[%f %f %f]\n",raw->magnetic.x,raw->magnetic.y,raw->magnetic.z);
	
	raw->magnetic.z = raw->magnetic.z - 0.02*OTP[0]*raw->magnetic.x - 0.02*OTP[1]*raw->magnetic.y;
	mag_raw[0] = raw->magnetic.x;
	mag_raw[1] = raw->magnetic.y;
	mag_raw[2] = raw->magnetic.z;
	
	//translate coordinate axis
	mMagneticEvent.magnetic.x = (cvt.sign[0]*mag_raw[cvt.map[0]]) * scalar;
	mMagneticEvent.magnetic.y = (cvt.sign[1]*mag_raw[cvt.map[1]]) * scalar;
	mMagneticEvent.magnetic.z = (cvt.sign[2]*mag_raw[cvt.map[2]]) * scalar;
	
	process(&mMagneticEvent);

	result->magnetic.x = mMagneticEvent.magnetic.x/scalar2uT;
	result->magnetic.y = mMagneticEvent.magnetic.y/scalar2uT;
	result->magnetic.z = mMagneticEvent.magnetic.z/scalar2uT;
	result->magnetic.status = mMagneticEvent.magnetic.status;
	
	D("convert_magnetic result->x=%f,result->y=%f,result->z=%f\n",result->magnetic.x,result->magnetic.y,result->magnetic.z);
	
	return 0;
}

static int convert_orientation(sensors_event_t *raw, sensors_event_t *result,
		struct sensor_algo_args *args)
{	
	if (raw->type == SENSOR_TYPE_ACCELEROMETER) {
		mAccelEvent.acceleration.x = raw->acceleration.x;
		mAccelEvent.acceleration.y = raw->acceleration.y;
		mAccelEvent.acceleration.z = raw->acceleration.z;
	}
	D("convert_orientation acc[%f %f %f]\n",raw->acceleration.x,raw->acceleration.y,raw->acceleration.z);
	if (raw->type == SENSOR_TYPE_MAGNETIC_FIELD) {
		mMagneticEvent.magnetic.x = raw->magnetic.x;
		mMagneticEvent.magnetic.y = raw->magnetic.y;
		mMagneticEvent.magnetic.z = raw->magnetic.z;
	}
	D("convert_orientation mag[%f %f %f]\n",raw->magnetic.x,raw->magnetic.y,raw->magnetic.z);
	
	QMCD_GetSensorsData(&mMagneticEvent, &mAccelEvent, NULL, NULL, result, NULL, NULL);

	D("convert_orientation result->x=%f,result->y=%f,result->z=%f\n",result->orientation.azimuth,result->orientation.pitch,result->orientation.roll);

	return 0;
}

#ifdef QST_VIRTUAL_SENSOR
static int get_rotation_vector(sensors_event_t *raw, sensors_event_t *result, struct sensor_algo_args *args)
{
	if (raw->type == SENSOR_TYPE_ACCELEROMETER) {
		mAccelEvent.acceleration.x = raw->acceleration.x;
		mAccelEvent.acceleration.y = raw->acceleration.y;
		mAccelEvent.acceleration.z = raw->acceleration.z;
	}

	if (raw->type == SENSOR_TYPE_MAGNETIC_FIELD) {
		mMagneticEvent.magnetic.x = raw->magnetic.x;
		mMagneticEvent.magnetic.y = raw->magnetic.y;
		mMagneticEvent.magnetic.z = raw->magnetic.z;
	}

	QMCD_GetSensorsData(&mMagneticEvent, &mAccelEvent, NULL, result, NULL, NULL, NULL);
	
	D("get_rotation_vector result[%f %f %f %f]\n",result->data[0],result->data[1],result->data[2],result->data[3]);
	return 0;
}

static int get_virtual_gyro(sensors_event_t *raw, sensors_event_t *result, struct sensor_algo_args *args)
{
	if (raw->type == SENSOR_TYPE_ACCELEROMETER) {
		mAccelEvent.acceleration.x = raw->acceleration.x;
		mAccelEvent.acceleration.y = raw->acceleration.y;
		mAccelEvent.acceleration.z = raw->acceleration.z;
	}

	if (raw->type == SENSOR_TYPE_MAGNETIC_FIELD) {
		mMagneticEvent.magnetic.x = raw->magnetic.x;
		mMagneticEvent.magnetic.y = raw->magnetic.y;
		mMagneticEvent.magnetic.z = raw->magnetic.z;
	}

	QMCD_GetSensorsData(&mMagneticEvent, &mAccelEvent, result, NULL, NULL, NULL, NULL);
	
	D("get_virtual_gyro result[%f %f %f]\n",result->data[0],result->data[1],result->data[2]);
	return 0;
}

static int get_gravity(sensors_event_t *raw, sensors_event_t *result, struct sensor_algo_args *args)
{
	if (raw->type == SENSOR_TYPE_ACCELEROMETER) {
		mAccelEvent.acceleration.x = raw->acceleration.x;
		mAccelEvent.acceleration.y = raw->acceleration.y;
		mAccelEvent.acceleration.z = raw->acceleration.z;
	}

	if (raw->type == SENSOR_TYPE_MAGNETIC_FIELD) {
		mMagneticEvent.magnetic.x = raw->magnetic.x;
		mMagneticEvent.magnetic.y = raw->magnetic.y;
		mMagneticEvent.magnetic.z = raw->magnetic.z;
	}

	QMCD_GetSensorsData(&mMagneticEvent, &mAccelEvent, NULL, NULL, NULL, result, NULL);
	
	D("get_gravity result[%f %f %f]\n",result->data[0],result->data[1],result->data[2]);
	return 0;
}

static int get_la(sensors_event_t *raw, sensors_event_t *result, struct sensor_algo_args *args)
{
	if (raw->type == SENSOR_TYPE_ACCELEROMETER) {
		mAccelEvent.acceleration.x = raw->acceleration.x;
		mAccelEvent.acceleration.y = raw->acceleration.y;
		mAccelEvent.acceleration.z = raw->acceleration.z;
	}

	if (raw->type == SENSOR_TYPE_MAGNETIC_FIELD) {
		mMagneticEvent.magnetic.x = raw->magnetic.x;
		mMagneticEvent.magnetic.y = raw->magnetic.y;
		mMagneticEvent.magnetic.z = raw->magnetic.z;
	}

	QMCD_GetSensorsData(&mMagneticEvent, &mAccelEvent, NULL, NULL, NULL, result, NULL);
	
	D("get_la result[%f %f %f]\n",result->data[0],result->data[1],result->data[2]);
	return 0;
}
#endif

static int cal_init(const struct sensor_cal_module_t *module)
{
	int fd_layout;
	int fd_otp;
	int ret = 0;
	char buf[30] = {0};

	fd_layout = open(sys_input_layout,O_RDONLY);
	if(fd_layout >= 0){
		ret = read(fd_layout, buf, sizeof(buf));
		if(1 == sscanf(buf,"%d",&direction)){
			if(direction < 0)	
			direction = 0;
		}
		close(fd_layout);
	}
	
	memset(buf,0,sizeof(buf));

	fd_otp = open(sys_input_otp,O_RDONLY);
	if(fd_otp >= 0){
		ret = read(fd_otp, buf, sizeof(buf));
		sscanf(buf,"%d,%d",&OTP[0],&OTP[1]);
		close(fd_otp);
	}
	if(direction > sizeof(map)/sizeof(map[0]))
		direction = 0;
	cvt = map[direction];
	
	mcal(0);
#ifdef QST_VIRTUAL_SENSOR
	dummyGyroInit();
#endif
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

static struct sensor_algo_methods_t compass_methods = {
	.convert = convert_magnetic,
};

static const char* compass_match_table[] = {
	"QST-mag",
	NULL
};

static struct sensor_algo_methods_t orientation_methods = {
	.convert = convert_orientation,
};

static const char* orientation_match_table[] = {
	"QST-orientation",
	NULL
};

#ifdef QST_VIRTUAL_SENSOR
static struct sensor_algo_methods_t rv_methods = {
	.convert = get_rotation_vector,
	.config = NULL,
};

static struct sensor_algo_methods_t pg_methods = {
	.convert = get_virtual_gyro,
	.config = NULL,
};

static struct sensor_algo_methods_t gv_methods = {
	.convert = get_gravity,
	.config = NULL,
};

static struct sensor_algo_methods_t la_methods = {
	.convert = get_la,
	.config = NULL,
};

static const char* rv_match_table[] = {
	"QST-rotation-vector",
	NULL
};

static const char* pg_match_table[] = {
	"QST-pseudo-gyro",
	NULL
};

static const char* gv_match_table[] = {
	"QST-gravity",
	NULL
};

static const char* la_match_table[] = {
	"QST-linear-acceleration",
	NULL
};
#endif

static struct sensor_cal_algo_t algo_list[] = {
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_MAGNETIC_FIELD,
		.compatible = compass_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &compass_methods,
	},
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_ORIENTATION,
		.compatible = orientation_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &orientation_methods,
	},
#ifdef QST_VIRTUAL_SENSOR
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
		.type = SENSOR_TYPE_GYROSCOPE,
		.compatible = pg_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &pg_methods,
	},
	{
		.tag = SENSOR_CAL_ALGO_TAG,
		.version = SENSOR_CAL_ALGO_VERSION,
		.type = SENSOR_TYPE_GRAVITY,
		.compatible = gv_match_table,
		.module = &SENSOR_CAL_MODULE_INFO,
		.methods = &gv_methods,
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

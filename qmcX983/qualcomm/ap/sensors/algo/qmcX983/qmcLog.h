#ifndef QMC_LOG_H
#define QMC_LOG_H

#include <cutils/log.h>

#undef LOG_TAG
#define LOG_TAG "QMCX983D"

#define ENBALE_DEBUG

#ifdef ENBALE_DEBUG
#define ALOGE(...)	ALOGD(__VA_ARGS__)
#define D(...)  	ALOGD(__VA_ARGS__) //open debug log
#else
#define  D(...)
#define ALOGE
#endif
#endif

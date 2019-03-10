#ifndef QMC_LOG_H
#define QMC_LOG_H
#include <cutils/log.h>
#undef LOG_TAG
#define LOG_TAG "QMCX983D"

#ifdef ENBALE_DEBUG_WRAPPER
#define E(...)		ALOGE(__VA_ARGS__)
#define D(...)  	ALOGD(__VA_ARGS__) //open debug log
#else
#define  D(...)  ((void)0)
#define  E(...)  ((void)0)
#endif
#endif

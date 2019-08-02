#ifndef QMC_COMMON_H
#define QMC_COMMON_H
#include "ical.h"
#define QST_VIRTUAL_SENSORS
#ifdef QST_VIRTUAL_SENSORS
#include "dummyGyro.h"
#endif

#define ID_A  (0)
#define ID_M  (1)
#define ID_O  (2)

#endif

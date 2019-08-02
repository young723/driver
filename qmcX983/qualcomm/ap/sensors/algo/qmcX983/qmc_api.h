#ifndef QMC_API_H
#define QMC_API_H
#include "qmc_common.h"
void QMC_Init(void);
void QMC_CalibMagProcess(sensors_event_t* raw, sensors_event_t* cali);
void QMC_ORIProcess(sensors_event_t *raw, sensors_event_t *ori);
void QMC_GyroProcess(sensors_event_t *raw, sensors_event_t *ori);
void QMC_RVProcess(sensors_event_t *raw, sensors_event_t *ori);
void QMC_GRAProcess(sensors_event_t *raw, sensors_event_t *ori);
void  QMC_LAProcess(sensors_event_t *raw, sensors_event_t *ori);

#endif

#ifndef __QSICAL_H__
#define __QSICAL_H__

void mcal(int KxOrKy);
int process(sensors_event_t* mMagneticEvent);
int push2mcal(sensors_event_t* data, int count);
int get(sensors_event_t* data);

void QST_fusion(float * mag,float *acc,float * resbuff);

#endif


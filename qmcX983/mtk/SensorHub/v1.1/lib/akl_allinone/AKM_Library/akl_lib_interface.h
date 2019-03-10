#ifndef _INCLUDE_AKL_LIB_INTERFACE_H
#define _INCLUDE_AKL_LIB_INTERFACE_H

#include "AKM_CustomerSpec.h"
#include "akl_smart_compass.h"
#include "AKM_Common.h"


int16 AKL_DOEAG_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_Calibrate(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst);


int16 AKL_DOEEX_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_Calibrate(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst);


int16 AKL_DOE_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_Calibrate(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst);


int16 AKL_D9D_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D9D_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D9D_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_D9D_CalcFusion(struct AKL_SCL_PRMS *prms);


int16 AKL_D6D_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D6D_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D6D_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_D6D_CalcFusion(struct AKL_SCL_PRMS *prms);

int16 AKL_PG_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_PG_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_PG_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_PG_CalcAngularRate(struct AKL_SCL_PRMS *prms);

int16 AKL_NULL_Function(struct AKL_SCL_PRMS *prms);

#endif

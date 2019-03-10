/****************************************************************
*
* @file vtc_chip.h
*
* Copyright(C), 2016, Voltafield Corporation
*
****************************************************************/

#ifndef __VTC_CHIP_H__
#define __VTC_CHIP_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

//================================================================
//field range (uT)
#define MAG_FIELD_RANGE_AF7133    1200
#define MAG_FIELD_RANGE_AF7133B   1600
#define MAG_FIELD_RANGE_AF7133C   1600
#define MAG_FIELD_RANGE_AF7133E   2200
#define MAG_FIELD_RANGE_AF8133I   2200
#define MAG_FIELD_RANGE_AF8133J   2200
#define MAG_FIELD_RANGE_AF9133    2200
#define MAG_FIELD_RANGE_AF6133    1600

enum mag_chip_device {
	MAG_DEV_AF7133 = 0,
	MAG_DEV_AF7133B,
	MAG_DEV_AF7133C,
	MAG_DEV_AF7133E,
	MAG_DEV_AF8133I,
	MAG_DEV_AF8133J,
	MAG_DEV_AF9133,
	MAG_DEV_AF6133,
	MAG_DEV_MAX
};

enum mag_chip_layout {
	MAG_LAYOUT_TOP_LEFT = 0,
	MAG_LAYOUT_BTN_LEFT = 1,
	MAG_LAYOUT_TOP_RIGHT = 2,
	MAG_LAYOUT_BTN_RIGHT = 3,
	MAG_LAYOUT_TOP_LEFT_INV = 4,
	MAG_LAYOUT_BTN_LEFT_INV = 5,
	MAG_LAYOUT_TOP_RIGHT_INV = 6,
	MAG_LAYOUT_BTN_RIGHT_INV = 7
};

//================================================================
#define AF7133_EXTRA_ZGAIN   10
//================================================================

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* __VTC_CHIP_H__ */


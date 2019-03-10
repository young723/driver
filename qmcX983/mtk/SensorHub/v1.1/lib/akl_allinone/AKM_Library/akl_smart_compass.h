/******************************************************************************
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 ******************************************************************************/
#ifndef INCLUDE_AKL_SMART_COMPASS_H
#define INCLUDE_AKL_SMART_COMPASS_H
#include "AKM_CustomerSpec.h"

/* Root file of AKL module. */
#include "akl_apis.h"
/* Headers of SmartCompass Library */
#include "libSmartCompass/AKCertification.h"
#include "libSmartCompass/AKConfigure.h"
#include "libSmartCompass/AKDecomp.h"
#include "libSmartCompass/AKMDevice.h"
#include "libSmartCompass/AKMDeviceF.h"
#include "libSmartCompass/AKDirection6D.h"
#include "libSmartCompass/AKDirection9D.h"
#include "libSmartCompass/AKDirection9Dv2.h"
#include "libSmartCompass/AKHDOE.h"
#include "libSmartCompass/AKHDOEaG.h"
#include "libSmartCompass/AKHDOEaGF.h"
#include "libSmartCompass/AKHFlucCheck.h"
#include "libSmartCompass/AKManualCal.h"
#include "libSmartCompass/AKVersion.h"
#include "libSmartCompass/AKHDOEEX.h"
#include "libSmartCompass/AKPseudoGyroF.h"

/*! Parameter for #AKSC_HFlucCheck function. */
#define HFLUCV_TH          2500
#define THETAFILTER_SCALE  4128

struct akl_libapi_calib{
    int16 calib_mode;
    int16 (*calib_set_pointer)(struct AKL_SCL_PRMS *prms);
    int16 (*calib_free_pointer)(struct AKL_SCL_PRMS *prms);
    int16 (*calib_init)(struct AKL_SCL_PRMS *prms);
    int16 (*calib_calc)(struct AKL_SCL_PRMS *prms);
    int16 (*calib_set_level)(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst);
};

struct akl_libapi_fusion{
    int16 fusion_mode;
    int16 (*fusion_set_pointer)(struct AKL_SCL_PRMS *prms);
    int16 (*fusion_free_pointer)(struct AKL_SCL_PRMS *prms);
    int16 (*fusion_init)(struct AKL_SCL_PRMS *prms);
    int16 (*fusion_calc)(struct AKL_SCL_PRMS *prms);
    int16 (*fusion_pg_set_pointer)(struct AKL_SCL_PRMS *prms);
    int16 (*fusion_pg_free_pointer)(struct AKL_SCL_PRMS *prms);
    int16 (*fusion_pg_init)(struct AKL_SCL_PRMS *prms);
    int16 (*fusion_pg_calc)(struct AKL_SCL_PRMS *prms);
};


/*
 * The following prefix joined this order.
 * e.g. A pointer to a struct is: 'ps_foo'
 *      An array of struct is   : 'sa_bar'
 * p_ : pointer
 * v_ : vector (a special type of struct)
 * s_ : struct
 * a_ : array
 * m_ : integer/float/enum etc..
 */

/*!
 * A parameter struct which is saved to non-volatile area for warm start up.
 */
struct AKL_NV_PRMS {
    /*! This value is used to identify the data area is AKL_NV_PRMS.
     * This value should be #AKL_NV_MAGIC_NUMBER. */
    uint32     magic;

    int16vec   va_hsuc_ho;                      /*!< fine offset of magnetic vector */
    int16vec   va_hflucv_href;                  /*!< rough value of magnetic vector */
    AKSC_HDST  a_hsuc_hdst;                     /*!< status of magnetic offset */
    int32vec   va_hsuc_hbase;                   /*!< rough offset of magnetic vector */
#ifdef AKM_ENABLE_PDC
    uint8      a_pdc[AKL_PDC_SIZE];             /*!< a parameter for PDC */
#endif
};

struct AKL_PG_PRMS{
    int16           pg_dt;
    int16           pg_filter;
    AKPG_COND_F     pg_cond;
    AKPG_VAR_F      pg_var;
    AKSC_FVEC       pg_out;
    AKSC_FVEC       pg_gravity;
    AKSC_FVEC       pg_linacc;
    AKSC_FQUAT      pg_quat;
};
struct AKL_FUSION_DATA{
    /*! Magnetic vector (offset subtracted) */
    int16vec                      v_hvec_fusion;
    /*! Acceleration vector (offset subtracted) */
    int16vec                      v_avec_fusion;
    /*! Angular rate vector (offset subtracted) */
    int16vec                      v_gvec_fusion;
    /*! Time stamp for v_hvec */
    int64_t                       m_ts_hvec_fusion;
    /*! Time stamp for v_avec */
    int64_t                       m_ts_avec_fusion;
    /*! Time stamp for v_gvec */
    int64_t                       m_ts_gvec_fusion;
    /*! Time stamp for v_hvec_pre */
    int64_t                       m_ts_hvec_fusion_pre;
    /*! Time stamp for v_avec */
    int64_t                       m_ts_avec_fusion_pre;
    /*! Time stamp for v_gvec */
    int64_t                       m_ts_gvec_fusion_pre;

    int64_t                       m_ts_fusion;
};
/*! A parameter structure which is needed for SmartCompass Library. */
struct AKL_SCL_PRMS {
    struct AKL_NV_PRMS            *ps_nv;
    uint32_t                      init;
    struct AKL_CERTIFICATION_INFO s_cert;

    /* Variables for magnetic sensor. */
    int32vec                      v_ho32;
    int16vec                      v_hs;
    AKSC_HFLUCVAR                 s_hflucv;
    /* base */
    int32vec                      v_hbase;

    /* Variables for DecompS3. */
    int16                         m_hn;
    int16                         m_hnave;
    int16vec                      v_have;
    int16vec                      v_asa;
    uint8                         *pa_pdcptr;
    /* Variables for HDOEEX. */
    AKSC_DOEEXVAR                 *ps_doeex_var;
    /* Variables for HDOE. */
    AKSC_HDOEVAR                  *ps_doe_var;
    /* Variables for HDOEaG. */
    AKSC_DOEAGVARF                 *ps_doeag_varf;
    AKSC_HDST                     m_hdst;
    int16                         m_mode;
    int16                         m_oedt;
    AKSC_HDST           m_pre_hdst;     //add by eric for save parameters when hdst change to 3

    /* Variables for formation change */
    uint8                         m_form;
    uint8                         m_maxForm;
    uint8                         m_curForm;
    /* layout */
    I16MATRIX     m_hlayout;
    I16MATRIX     m_alayout;
    I16MATRIX     m_glayout;

    /* Variables for Direction9Dv2 */
    AKSC_D9DVAR                   *m_D9Dvar;
    int16                         m_D9Dmode;
    int16                         m_theta;
    int16                         m_phi180;
    int16                         m_eta90;

    /* vector collection */
    /*! Magnetic vector (offset subtracted) */
    int16vec                      v_hvec;
    /*! Acceleration vector (offset subtracted) */
    int16vec                      v_avec;
    /*! Angular rate vector (offset subtracted) */
    int16vec                      v_gvec;
    AKSC_FVEC                     v_gvecf;
    /*! An offset of magnetic vector */
    int16vec                      v_ho;
    /*! An offset of angular rate vector */
    int16vec                      v_go;
    /*! A buffer of raw magnetic vector (with offset) */
    int16vec                      va_hdata[AKSC_HDATA_SIZE];
    /*! A buffer of raw angular rate vector (with offset) */
    int16vec                      va_gdata[AKSC_GDATA_SIZE];
    /*! Quaternion */
    I16QUAT                       s_quat;
    AKSC_FQUAT                    s_quatf;
    /*! Gravity vector */
    AKSC_FVEC                      v_gravity;
    /*! Linear acceleration vector */
    AKSC_FVEC                      v_lacc;

    uint8                         m_data_type;
    /* time stamp collection */
    /*! Time during for two mag data */
    int16_t                       m_hdt_ag;
    /*! Time during for gyro data and last mag data*/
    int16_t                       m_gdt_ag;
    /*! Time during for two mag data*/
    int16_t                       m_hdt;
    /*! Time during for two gyro data*/
    int16_t                       m_gdt;
    /*! Time during for two acc data*/
    int16_t                       m_adt;
    /*! Time stamp for 'formation chage' event. */
    int64_t                       m_ts_ev_form;
    /*! Time stamp for 'HDOE' event. */
    int64_t                       m_ts_ev_doe;
    /*! Time stamp for 'Direction9D' event. */
    int64_t                       m_ts_ev_d9d;
    /*! Time stamp for v_hvec */
    int64_t                       m_ts_hvec;
    /*! Time stamp for v_avec */
    int64_t                       m_ts_avec;
    /*! Time stamp for v_gvec */
    int64_t                       m_ts_gvec;
    /*! Time stamp for D9D hvec */
    int64_t                       m_ts_ev_9d_hvec;
    /*! Time stamp for D9D avec */
    int64_t                       m_ts_ev_9d_avec;
    /*! Time stamp for D9D gvec */
    int64_t                       m_ts_ev_9d_gvec;
    /*! Time stamp for v_hvec, temp store data time stamp */
    int64_t                       m_ts_hvec_set;
    /*! Time stamp for v_gvec, temp store data time stamp */
    int64_t                       m_ts_gvec_set;

    struct AKL_PG_PRMS            *ps_PGvar;

    /*! Layout pattern  number */
    int16_t        m_pat;

    /*! AKM device type */
    AKM_DEVICE_TYPE m_device;
    struct akl_libapi_calib m_lib_calib;
    struct akl_libapi_fusion m_lib_fusion;
    struct AKL_FUSION_DATA m_fusion_data;
};
#endif /* INCLUDE_AKL_SMART_COMPASS_H */

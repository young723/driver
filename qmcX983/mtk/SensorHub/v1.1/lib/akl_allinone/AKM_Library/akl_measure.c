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
#include "akl_measure.h"
#include "akl_lib_interface.h"

void InitAKSCPRMS(struct AKL_SCL_PRMS *prms)
{
    /* Sensitivity */
    prms->v_hs.u.x = AKSC_HSENSE_TARGET;
    prms->v_hs.u.y = AKSC_HSENSE_TARGET;
    prms->v_hs.u.z = AKSC_HSENSE_TARGET;
    /* ASA */
    prms->v_asa.u.x = 0x80;
    prms->v_asa.u.y = 0x80;
    prms->v_asa.u.z = 0x80;

    prms->m_hdst = AKSC_HDST_UNSOLVED;

    /* (m_hdata is initialized with AKSC_InitDecomp) */
    prms->m_hnave = AKM_DEFAULT_HNAVE;

    /* Formation */
    prms->m_form = prms->m_curForm;

    /* Conversion matrix from Android to SmartCompass coordination */
    prms->m_hlayout.m[0][0] = 0;
    prms->m_hlayout.m[0][1] = 1;
    prms->m_hlayout.m[0][2] = 0;
    prms->m_hlayout.m[1][0] = -1;
    prms->m_hlayout.m[1][1] = 0;
    prms->m_hlayout.m[1][2] = 0;
    prms->m_hlayout.m[2][0] = 0;
    prms->m_hlayout.m[2][1] = 0;
    prms->m_hlayout.m[2][2] = 1;

    prms->m_alayout.m[0][0] = 0;
    prms->m_alayout.m[0][1] = -1;
    prms->m_alayout.m[0][2] = 0;
    prms->m_alayout.m[1][0] = 1;
    prms->m_alayout.m[1][1] = 0;
    prms->m_alayout.m[1][2] = 0;
    prms->m_alayout.m[2][0] = 0;
    prms->m_alayout.m[2][1] = 0;
    prms->m_alayout.m[2][2] = -1;

    prms->m_glayout.m[0][0] = 0;
    prms->m_glayout.m[0][1] = 1;
    prms->m_glayout.m[0][2] = 0;
    prms->m_glayout.m[1][0] = -1;
    prms->m_glayout.m[1][1] = 0;
    prms->m_glayout.m[1][2] = 0;
    prms->m_glayout.m[2][0] = 0;
    prms->m_glayout.m[2][1] = 0;
    prms->m_glayout.m[2][2] = 1;
}


int16 InitMeasure(struct AKL_SCL_PRMS *prms)
{
    int16 ret;

    prms->m_form = 0;
    /* Restore the value when succeeding in estimating of HOffset. */
    prms->v_ho = prms->ps_nv[prms->m_form].va_hsuc_ho;
    prms->v_ho32.u.x = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.x;
    prms->v_ho32.u.y = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.y;
    prms->v_ho32.u.z = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.z;
    prms->s_hflucv.href = prms->ps_nv[prms->m_form].va_hflucv_href;
    prms->m_hdst = prms->ps_nv[prms->m_form].a_hsuc_hdst;
    prms->v_hbase = prms->ps_nv[prms->m_form].va_hsuc_hbase;

    /* Initialize the decompose parameters */
    AKSC_InitDecomp(prms->va_hdata);

    prms->m_lib_calib.calib_init(prms);

    ret = AKSC_InitHFlucCheck(
            &(prms->s_hflucv),
            &(prms->ps_nv[prms->m_form].va_hflucv_href),
            HFLUCV_TH
        );

    if (ret == 0) {
        return AKRET_PROC_FAIL;
    }

    prms->m_lib_fusion.fusion_init(prms);
    prms->m_lib_fusion.fusion_pg_init(prms);

#ifdef AKM_ENABLE_PDC
    /* Set PDC parameter pointer */
    prms->pa_pdcptr = prms->ps_nv[prms->m_form].a_pdc;
#else
    prms->pa_pdcptr = 0;
#endif

    /* Reset time stamp. */
    prms->m_ts_hvec = 0;
    prms->m_ts_avec = 0;
    prms->m_ts_gvec = 0;
    prms->m_ts_ev_form = 0;
    prms->m_ts_ev_d9d = 0;

    return AKRET_PROC_SUCCEED;
}


int16 GetMagneticVectorDecomp(
    const int16         bData[],
    struct AKL_SCL_PRMS *prms,
    const int64_t       ts_mag)
{
    const int16vec hrefZero = {{0, 0, 0}};
    int16    temperature, dor, derr, hofl, cb, dc;
    int32vec preHbase;
    int16    overflow;
    int16    aksc_ret;
    int16    ret;

    temperature = 0;
    dor = 0;
    derr = 0;
    hofl = 0;
    cb = 0;
    dc = 0;

    preHbase = prms->v_hbase;
    overflow = 0;
    ret = AKRET_PROC_SUCCEED;

    /* Subtract the formation suspend counter */
    if (prms->m_ts_ev_form > 0U) {
        /* If suspended time exceeds the threshold, reset parameters. */
        if ((ts_mag - prms->m_ts_ev_form) >= AKM_INTERVAL_SUSPEND) {
            /* Restore the value when succeeding in estimating of HOffset. */
            prms->v_ho = prms->ps_nv[prms->m_form].va_hsuc_ho;
            prms->v_ho32.u.x = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.x;
            prms->v_ho32.u.y = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.y;
            prms->v_ho32.u.z = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.z;

            prms->m_hdst = prms->ps_nv[prms->m_form].a_hsuc_hdst;
            prms->v_hbase = prms->ps_nv[prms->m_form].va_hsuc_hbase;

            /* Initialize the decompose parameters */
            AKSC_InitDecomp(prms->va_hdata);

#ifdef AKM_ENABLE_PDC
            prms->pa_pdcptr = prms->ps_nv[prms->m_form].a_pdc;
#endif

            prms->m_lib_calib.calib_init(prms);

            /* Initialize HFlucCheck parameters */
            aksc_ret = AKSC_InitHFlucCheck(
                    &(prms->s_hflucv),
                    &(prms->ps_nv[prms->m_form].va_hflucv_href),
                    HFLUCV_TH
                );
            /* Reset timestamp */
            prms->m_ts_ev_form = 0;
        }
    }

    /* Decompose one block data into each Magnetic sensor's data */
    aksc_ret = AKSC_DecompS3(
            prms->m_device,
            bData,
            prms->m_hnave,
            &prms->v_asa,
            prms->pa_pdcptr,
            prms->va_hdata,
            &prms->v_hbase,
            &prms->m_hn,
            &prms->v_have,
            &temperature,
            &dor,
            &derr,
            &hofl,
            &cb,
            &dc
        );

    //printf("AKSC_DecompS3 prms->m_device = %d\n",prms->m_device);
    if (aksc_ret == 0) {
        AKM_MSG_ERR_0("akm_err AKSC_DecompS3 failed\n");
        return AKRET_PROC_FAIL;
    }

    //AKM_MSG_ERR_3("akm_err LogForMDA: %d, %d, %d", prms->va_hdata[0].u.x, prms->va_hdata[0].u.y, prms->va_hdata[0].u.z);

    if (prms->m_form != prms->m_curForm) {
        /* Formation changed! */
        prms->m_form = prms->m_curForm;
        prms->m_ts_ev_form = ts_mag;
        ret |= AKRET_FORMATION_CHANGED;
        return ret;
    }

    /* Check derr */
    if (derr == 1) {
        ret |= AKRET_DATA_READERROR;
        return ret;
    }

    /* Check hofl */
    if (hofl == 1) {
        if (prms->m_ts_ev_form == 0) {
            prms->m_lib_calib.calib_set_level(prms, AKSC_HDST_UNSOLVED);
        }

        ret |= AKRET_DATA_OVERFLOW;
        return ret;
    }

    /* Check hbase */
    if (cb == 1) {
        /* Translate HOffset */
        AKSC_TransByHbase(
            &(preHbase),
            &(prms->v_hbase),
            &(prms->v_ho),
            &(prms->v_ho32),
            &overflow
        );

        if (overflow == 1) {
            ret |= AKRET_OFFSET_OVERFLOW;
        }

        /* Set hflucv.href to 0 */
        aksc_ret = AKSC_InitHFlucCheck(
                &(prms->s_hflucv),
                &(hrefZero),
                HFLUCV_TH
            );

        if (prms->m_ts_ev_form == 0) {
            prms->m_lib_calib.calib_set_level(prms, AKSC_HDST_UNSOLVED);
        }

        ret |= AKRET_HBASE_CHANGED;
        return ret;
    }

    return AKRET_PROC_SUCCEED;
}

/*****************************************************************************/
int16 GetMagneticVectorOffset(
    struct AKL_SCL_PRMS *prms,
    const int64_t       ts_mag)
{
    int16           hfluc;
    int16           hdSucc;
    int16           aksc_ret;

    if (prms->m_ts_ev_form == 0U) {
        /* Detect a fluctuation of magnetic field. */
        hfluc = AKSC_HFlucCheck(&(prms->s_hflucv), &(prms->va_hdata[0]));

        if (hfluc == 1) {
            prms->m_lib_calib.calib_set_level(prms, AKSC_HDST_UNSOLVED);
            return AKRET_HFLUC_OCCURRED;
        } else {
            hdSucc = prms->m_lib_calib.calib_calc(prms);

            if (hdSucc == AKSC_CERTIFICATION_DENIED) {
                AKM_MSG_ERR_0("akm_err prms->m_lib_calib.calib_calc Failed\n");
                return AKRET_CERTIFICATION_ERROR;
            }

            if (hdSucc > 0) {
                prms->ps_nv[prms->m_form].va_hsuc_ho = prms->v_ho;
                prms->v_ho32.u.x = (int32)prms->v_ho.u.x;
                prms->v_ho32.u.y = (int32)prms->v_ho.u.y;
                prms->v_ho32.u.z = (int32)prms->v_ho.u.z;

                prms->ps_nv[prms->m_form].a_hsuc_hdst = prms->m_hdst;
                prms->ps_nv[prms->m_form].va_hflucv_href =prms->s_hflucv.href;
                prms->ps_nv[prms->m_form].va_hsuc_hbase = prms->v_hbase;
            }
        }
    }

    /* Subtract offset and normalize magnetic field vector. */
    aksc_ret = AKSC_VNorm(
            &prms->v_have,
            &prms->v_ho,
            &prms->v_hs,
            AKSC_HSENSE_TARGET,
            &prms->v_hvec
        );

    if (aksc_ret == 0) {
        return AKRET_VNORM_ERROR;
    }

    return AKRET_PROC_SUCCEED;
}

int16 CalcDirection(struct AKL_SCL_PRMS *prms)
{
    return prms->m_lib_fusion.fusion_calc(prms);
}
int16 CalcAngularRate(struct AKL_SCL_PRMS *prms)
{
    return prms->m_lib_fusion.fusion_pg_calc(prms);
}



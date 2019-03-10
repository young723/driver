/*
 * =====================================================================================
 *
 *       Filename:  akm_apis.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2017年06月07日 18时50分58秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */ 
#include "akm_apis.h"
#include "FileIO.h"
#include "AKM_CustomerSpec.h"
#include "AKM_DataSort.h"

static struct AKL_SCL_PRMS *g_prm = NULL;
static struct AKL_CERTIFICATION_INFO g_info;

static float g_offset[3] = {0.f};
#define CONVERT_AKSC_TO_MICROTESLA (0.06f)

 /**
  * @brief A simple implementation of string copy function.
  * This is really simple, so argument check is not done in the function.
  * Wrong parameter may destroy memory area.
  *
  * @param dst A pointer to destination buffer.
  * @param src A pointer to an original string.
  * @param dst_len The size of destination buffer.
  */
 static void string_copy(
     char          dst[],
     const char    src[],
     const int16_t dst_len)
 {
     int16_t i = 0;

     while (src[i] != '\0') {
         dst[i] = src[i];
         i++;

         if (i >= dst_len) {
             i--;
             break;
         }
     }

     dst[i] = '\0';
 }


 /**
  * @brief Set certification information to enable DOE function.
  * This function sets LICENSER and LICENSEE string, then
  * get hardware information from driver.
  *
  * @param info A pointer to #AKL_CERTIFICATION_INFO struct.
  *
  * @return When function succeeds, #AKM_SUCCESS is returned.
  */
 static int16_t AKM_SetCertificationInfo(struct AKL_CERTIFICATION_INFO *info)
 {
     string_copy((char *)info->a_licenser,
                 AKM_CUSTOM_LICENSER, AKL_CI_MAX_CHARSIZE + 1);
     string_copy((char *)info->a_licensee,
                 AKM_CUSTOM_LICENSEE, AKL_CI_MAX_CHARSIZE + 1);

     info->a_key[0] = 8;
     info->a_key[1] = 0x48;
     info->a_key[2] = 0x80;
     info->a_key[3] = 0x80;
     info->a_key[4] = 0x80;

     info->a_key[5] = (int16_t)AKM_CUSTOM_MAG_AXIS_ORDER_X;
     info->a_key[6] = (int16_t)AKM_CUSTOM_MAG_AXIS_ORDER_Y;
     info->a_key[7] = (int16_t)AKM_CUSTOM_MAG_AXIS_ORDER_Z;
     info->a_key[8] = (int16_t)AKM_CUSTOM_MAG_AXIS_SIGN_X;
     info->a_key[9] = (int16_t)AKM_CUSTOM_MAG_AXIS_SIGN_Y;
     info->a_key[10] = (int16_t)AKM_CUSTOM_MAG_AXIS_SIGN_Z;

     return AKM_SUCCESS;
 }

 void AKM_SetDefaultNV(
     struct AKL_NV_PRMS nv[],
     const uint8        num_formation)
 {
     int16 i;
#ifdef AKM_ENABLE_PDC
     int16 k;
     /* UNIT parameter for PDC */
     const uint8 U_PDC[AKL_PDC_SIZE] = {
         32, 82, 186, 174, 49, 255, 255, 96, 214,
         55, 231, 85, 38, 206, 255, 242, 255, 255,
         127, 154, 191, 252, 255, 255, 9, 38, 255
     };
#endif

     /* Set parameter to HDST, HO, HREF, HBASE */
     for (i = 0; i < (int16_t)num_formation; i++) {
         nv[i].a_hsuc_hdst = AKSC_HDST_UNSOLVED;
         nv[i].va_hsuc_ho.u.x = 0;
         nv[i].va_hsuc_ho.u.y = 0;
         nv[i].va_hsuc_ho.u.z = 0;
         nv[i].va_hflucv_href.u.x = 0;
         nv[i].va_hflucv_href.u.y = 0;
         nv[i].va_hflucv_href.u.z = 0;
         nv[i].va_hsuc_hbase.u.x = 0;
         nv[i].va_hsuc_hbase.u.y = 0;
         nv[i].va_hsuc_hbase.u.z = 0;
#ifdef AKM_ENABLE_PDC
         /* Set unit parameter */
         for (k = 0; k < AKL_PDC_SIZE; k++) {
             nv[i].a_pdc[k] = U_PDC[k];
         }
#endif
     }
 }


 /*! Library initialize function.
  * This function allocate new memory area for #AKL_SCL_PRMS.
  * When this function succeeds, #AKL_SCL_PRMS parameter is ready to
  * start measurement. But please note that this function does
  * not restore previously saved data, for example, offset and its
  * accuracy etc.\n
  * When this function fails, \c prm is cleaned up internally,
  * so you don't need to \c 'free' outside of this function.
  * \return When function succeeds, #AKM_SUCCESS is returned.
  * \param prm A pointer of pointer to #AKL_SCL_PRMS struct. */
// int16_t library_init(struct AKL_SCL_PRMS **prm)
 int16_t AKM_LibraryInit(uint32_t device, uint32_t layout_pat, uint32_t mode)
 {
     uint32_t temp_size;
     int16_t  fret;
#ifdef AKM_ENABLE_PDC
     uint8_t pdc[AKL_PDC_SIZE];
#endif

     temp_size = AKL_GetParameterSize((uint8_t)AKM_CUSTOM_NUM_FORM);
     if (temp_size == 0U) {
         fret = AKM_ERROR;
         goto EXIT_LIBRARY_INIT;
     }
     /* malloc library data parameter */
     g_prm = (struct AKL_SCL_PRMS *)AKM_MALLOC(temp_size);
     if(g_prm == NULL)
     {
        AKM_MSG_ERR_0("akm_err AKM_LibraryInit malloc g_prm failed");
        fret = AKM_ERROR;
        goto EXIT_LIBRARY_INIT;
     }
     AKM_MEMSET(g_prm, 0, (size_t)temp_size);
     /* malloc NV data*/
     temp_size = AKL_GetNVdataSize((uint8_t)AKM_CUSTOM_NUM_FORM);
     if (temp_size == 0U) {
         fret = AKM_ERROR;
         goto EXIT_LIBRARY_INIT;
     }
     g_prm->ps_nv = (struct AKL_NV_PRMS *)AKM_MALLOC(temp_size);
     if(g_prm->ps_nv == NULL)
     {
        AKM_MSG_ERR_0("akm_err AKM_LibraryInit malloc ps_nv failed");
        fret = AKM_ERROR;
        goto EXIT_LIBRARY_INIT;
     }
     AKM_SetDefaultNV(g_prm->ps_nv, AKM_CUSTOM_NUM_FORM);


     g_prm->m_device = AKL_GetDeviceType(device);
     g_prm->m_pat = layout_pat;

     /* certification */
     fret = AKM_SetCertificationInfo(&g_info);
     if (fret != AKM_SUCCESS) {
        AKM_MSG_ERR_0("akm_err AKM_SetCertificationInfo failed");
        goto EXIT_LIBRARY_INIT;
     }

     /* load data from storage */
     fret = AKL_LoadParameter(g_prm->ps_nv);
     if (fret != AKM_SUCCESS) {
            AKM_MSG_ERR_0("akm_err AKL_LoadParameter failed");
     }

     fret = AKL_SetMode(g_prm, mode);

     if (fret != AKM_SUCCESS) {
            AKM_MSG_ERR_0("akm_err AKL_SetMode failed");
            goto EXIT_LIBRARY_INIT;
     }
#ifdef AKM_ENABLE_PDC
     /* Use customized PDC parameter */
     AKL_LoadPDC(pdc);
     fret = AKL_SetPDC(g_prm, pdc, 0U);
#endif
     akm_data_sort_init();
     return AKM_SUCCESS;

 EXIT_LIBRARY_INIT:
     AKM_MSG_ERR_0("akm_err AKM_LibraryInit failed");
     if (g_prm != NULL) {
        if(g_prm->ps_nv != NULL){
            AKM_FREE(g_prm->ps_nv);
        }
        AKM_FREE(g_prm);
        g_prm = NULL;
     }
     return fret;
 }



 int16_t AKM_LoadAndStart(void)
 {
     int16_t  fret;

     AKM_MSG_INFO_0("akm_log AKM_LoadAndStart");
     //* Initialize AKM library. */
     fret = AKL_Init(g_prm, &g_info, (uint8_t)AKM_CUSTOM_NUM_FORM);
     if (fret != AKM_SUCCESS) {
         goto LOAD_QUIT;
     }

     /* AKL can accept NULL pointer for nv_data */
     fret = AKL_StartMeasurement(g_prm);

     if (fret != AKM_SUCCESS) {
         goto LOAD_QUIT;
     }


 LOAD_QUIT:


     return fret;
 }

/*------WARNING------
 *   SET DATA UNIT
 *	MAG: uT    
 *	ACC: m/s2  
 *	GYR: rad/s 
 *	timestamp: us
 */
 int16_t AKM_SetData(struct AKM_SENSOR_DATA *data)
 {
     int16_t ret = 0;
     if (data == NULL)
     {
         return ret;
     }
     akm_data_sort(data);
    ret = AKL_SetVector(g_prm,data,1);
    return ret;
 }
  int16_t AKM_SetFusionData(struct AKM_SENSOR_DATA *data)
  {
      int16_t ret = 0;
      if (data == NULL)
      {
          return ret;
      }
      akm_fusion_data_sort(data);
     ret = AKL_SetVector(g_prm,data,1);
#ifndef AKM_DISABLE_D9D
     if(data->stype == AKM_ST_FUSION_MAG || data->stype == AKM_ST_FUSION_GYR)
#else
     if(data->stype == AKM_ST_FUSION_MAG)
#endif
     {
         ret = AKL_CalcFusion(g_prm);
         if (ret != 0)
         {
             AKM_MSG_ERR_1("akm_err AKL_CalcFusion ERROR: %d\n",ret);
         }
     }
     return ret;
  }
 /*------WARNING------
 *   GET DATA UNIT
 *	MAG: uT
 *	ACC: m/s2
 *	GYR: deg/s
 *	ORI: deg
 *	GRAVITY: m/s2
 *	Linear ACC: m/s2
 *	QUAT: 1
 *	timestamp: us
 */
 int16_t AKM_GetData(float32_t result[6], int senor_type, int32_t *accuracy)
 {
    int64_t time_stamp;
    int16_t ret = 0;

    ret = AKL_GetVector(senor_type,g_prm,result,AKM_VT_MAG_SIZE,accuracy,&time_stamp);
    if (ret != 0)
    {
        AKM_MSG_ERR_2("akm_err AKL_GetData senor type: %d, ERROR: %d\n",senor_type, ret);
    }
    return ret;
 }

 void AKM_ReloadContext(float *offset)
 {
     g_offset[0]= offset[0];
     g_offset[1]= offset[1];
     g_offset[2]= offset[2];
 }
 int16 AKL_LoadParameter(struct AKL_NV_PRMS *nv_data)
 {
     nv_data->va_hsuc_ho.u.x = (uint16_t) (g_offset[0] / CONVERT_AKSC_TO_MICROTESLA);
     nv_data->va_hsuc_ho.u.y = (uint16_t) (g_offset[1] / CONVERT_AKSC_TO_MICROTESLA);
     nv_data->va_hsuc_ho.u.z = (uint16_t) (g_offset[2] / CONVERT_AKSC_TO_MICROTESLA);
     nv_data->va_hflucv_href.u.x = (uint16_t) (g_offset[0] / CONVERT_AKSC_TO_MICROTESLA);
     nv_data->va_hflucv_href.u.y = (uint16_t) (g_offset[1] / CONVERT_AKSC_TO_MICROTESLA);
     nv_data->va_hflucv_href.u.z = (uint16_t) (g_offset[2] / CONVERT_AKSC_TO_MICROTESLA);
     nv_data->va_hsuc_hbase.u.x = 0;
     nv_data->va_hsuc_hbase.u.y = 0;
     nv_data->va_hsuc_hbase.u.z = 0;
     nv_data->a_hsuc_hdst = AKSC_HDST_L0;
     return 0;
 }

 const uint8 pdc_init[27]={64 , 81 , 68 , 141, 225, 236, 0  , 235, 214,
                           54 , 236, 237, 148, 129, 0  , 57 , 255, 176,
                           193, 110, 188, 252, 44 , 222, 11 , 92 , 78 };
 int16 AKL_LoadPDC(uint8 *pdc)
 {
     int i = 0;
     for(i=0; i<27; i++){
         pdc[i] = pdc_init[i];
     }

     return 0;
 }


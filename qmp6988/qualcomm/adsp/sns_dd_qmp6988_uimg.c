/*	Date: 2015/12/22 14:00:00
 *	Revision: 1.4.4
 */







/*******************************************************************************
* Copyright (c) 2013, Bosch Sensortec GmbH
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     1. Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*     2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     3. Neither the name of Bosch Sensortec GmbH nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*-----------------------------------------------------------------------------
 * Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
  -----------------------------------------------------------------------------*/


/*==============================================================================

    S E N S O R S   P R E S S U R E   D R I V E R

DESCRIPTION

   Impelements the driver for the QMP6988 driver


                      EDIT HISTORY FOR FILE


when			who			note
----------   ---     -----------------------------------------------------------
18/07/28		yzq			initial version
==============================================================================*/

#include "sns_dd_qmp6988.h"

const q16_t qmp6988_temperature_resolution = FX_FLTTOFIX_Q16(0.01);
const q16_t qmp6988_pressure_resolution = FX_FLTTOFIX_Q16(0.01);

extern sns_ddf_status_e sns_dd_qmp6988_init(
   sns_ddf_handle_t*		dd_handle_ptr,
   sns_ddf_handle_t 		smgr_handle,
   sns_ddf_nv_params_s* 	nv_params,
   sns_ddf_device_access_s	device_info[],
   uint32_t 				num_devices,
   sns_ddf_memhandler_s*	memhandler,
   sns_ddf_sensor_e*		sensors[],
   uint32_t*				num_sensors);


extern sns_ddf_status_e sns_dd_qmp6988_set_attr(
   sns_ddf_handle_t 	dd_handle,
   sns_ddf_sensor_e 	sensor,
   sns_ddf_attribute_e	attrib,
   void*				value);

extern sns_ddf_status_e sns_dd_qmp6988_get_attr(
   sns_ddf_handle_t 	  dd_handle,
   sns_ddf_sensor_e 	  sensor,
   sns_ddf_attribute_e	  attrib,
   sns_ddf_memhandler_s*  memhandler,
   void**				  value,
   uint32_t*			  num_elems);

extern sns_ddf_status_e sns_dd_qmp6988_reset(sns_ddf_handle_t dd_handle);
extern sns_ddf_status_e sns_dd_qmp6988_self_test(
               sns_ddf_handle_t dd_handle,
               sns_ddf_sensor_e sensor,
               sns_ddf_test_e test,
               uint32_t* err);
extern sns_ddf_status_e sns_dd_qmp6988_probe(
                        sns_ddf_device_access_s* device_info,
                        sns_ddf_memhandler_s*    memhandler,
                        uint32_t*                num_sensors,
                        sns_ddf_sensor_e**       sensors );



extern sns_ddf_status_e qmp_get_pressure_temp(uint32_t *pressure, int32_t *temp);

 /*===========================================================================

 FUNCTION:	 sns_dd_qmp6988_log_data

 ===========================================================================*/

 /*!
  *  @brief log sensor data
  *
  *  @detail
  *   Logs latest set of sensor data sampled from the sensor
  *
  *  @param[in] state: ptr to the driver structure
  *  @param[in] sample_time: Time that the sensor was sampled
  *
  *
  *  @return
  *
  */
 /*=========================================================================*/
 void sns_dd_qmp6988_log_data(
         sns_dd_qmp6988_state_t *state,
         sns_ddf_time_t sample_time,
         sns_ddf_sensor_e sensor)
 {
     sns_err_code_e err_code;
     sns_log_sensor_data_pkt_s* log_struct_ptr;
     log_pkt_t						 log_pkt_type;

#if QMP6988_CONFIG_RUN_ON_OSSC
     log_pkt_type = SNS_LOG_CONVERTED_SENSOR_DATA_PUBLIC;
#else
     log_pkt_type = SNS_LOG_CONVERTED_SENSOR_DATA;
#endif

     QMP6988_MSG_0(HIGH, "Data[0] : Pressure in hPa (Q16)");

     //Allocate log packet
     err_code = sns_logpkt_malloc(log_pkt_type,
             sizeof(sns_log_sensor_data_pkt_s),
             (void**)&log_struct_ptr);

     if((err_code == SNS_SUCCESS) && (log_struct_ptr != NULL))
     {
         log_struct_ptr->version = SNS_LOG_SENSOR_DATA_PKT_VERSION;
         log_struct_ptr->sensor_id = sensor;
         log_struct_ptr->vendor_id = SNS_DDF_VENDOR_BOSCH;

         //Timestamp the log with sample time
         log_struct_ptr->timestamp = sample_time;

         //Log the sensor data
         log_struct_ptr->num_data_types = 1;

         if (sensor == SNS_DDF_SENSOR_PRESSURE)
         {
            log_struct_ptr->data[0]  = state->data_cache[0];
         }
         else
         {
            log_struct_ptr->data[0]  = state->data_cache[1];
         }

         //Commit log (also frees up the log packet memory)
         err_code = sns_logpkt_commit(log_pkt_type,
                 log_struct_ptr);
     }

     if (err_code != SNS_SUCCESS)
     {
         QMP6988_MSG_1(ERROR, "QMP6988 pressure Log Data - logpkt_malloc failed with err: %d", err_code);
     }
 }



/*!
*  @brief Called by the SMGR to get data
*
*  @detail
*   Requests a single sample of sensor data from each of the specified
*    sensors.
*
*  @param[in] dd_handle     Handle to a driver instance.
*  @param[in] sensors       List of sensors for which data isrequested.
*  @param[in] num_sensors   Length of @a sensors.
*  @param[in] memhandler    Memory handler used to dynamically allocate
*                           output parameters, if applicable.
*  @param[out] data         Sampled sensor data. The number of elements
*                           must match @a num_sensors.
*
*  @return
*    The error code definition within the DDF
*    SNS_DDF_SUCCESS on success.
*
*/


 sns_ddf_status_e sns_dd_qmp6988_get_data
(
  sns_ddf_handle_t        dd_handle,
  sns_ddf_sensor_e        sensors[],
  uint32_t                num_sensors,
  sns_ddf_memhandler_s*   memhandler,
  sns_ddf_sensor_data_s*  data[]
)
{
  uint8_t i;
  sns_ddf_status_e status = SNS_DDF_SUCCESS;
  sns_ddf_sensor_data_s *data_ptr;

  sns_dd_qmp6988_state_t *state = (sns_dd_qmp6988_state_t *)dd_handle;
  uint32_t pressure;
  int32_t temperature;

  if (state == NULL)
  {
     return SNS_DDF_EINVALID_PARAM;
  }
  for (i = 0; i < num_sensors; i++)
  {
    if (sensors[i] != SNS_DDF_SENSOR_PRESSURE &&
        sensors[i] != SNS_DDF_SENSOR_TEMP)
    {
      return SNS_DDF_EINVALID_PARAM;
    }
  }

  if((data_ptr = sns_ddf_memhandler_malloc_ex(memhandler,
        (num_sensors)*(sizeof(sns_ddf_sensor_data_s)), state->smgr_handle)) == NULL)
  {
    return SNS_DDF_ENOMEM;
  }
  *data = data_ptr;
  for (i=0; i <num_sensors; i++)
  {
    data_ptr[i].sensor = sensors[i];
    data_ptr[i].status = SNS_DDF_SUCCESS;
    data_ptr[i].timestamp = sns_ddf_get_timestamp();

	status = qmp_get_pressure_temp(&pressure, &temperature);
    if(sensors[i] == SNS_DDF_SENSOR_PRESSURE)
    {
      if( (data_ptr[i].samples = sns_ddf_memhandler_malloc_ex(memhandler,
            sizeof(sns_ddf_sensor_sample_s), state->smgr_handle)) == NULL)
      {
        return SNS_DDF_ENOMEM;
      }

	  data_ptr[i].samples[0].sample = FX_FLTTOFIX_Q16(pressure/100.0);
	  state->data_cache[0] = FX_FLTTOFIX_Q16(pressure/100.0);
      data_ptr[i].samples[0].status = SNS_DDF_SUCCESS;
      data_ptr[i].num_samples = 1;

      sns_dd_qmp6988_log_data(state, data_ptr[i].timestamp, sensors[i]);
	  QMP6988_MSG_2(HIGH, " pressure value in Q16 is %d, TS is %d \n",state->data_cache[0], data_ptr[i].timestamp);
    }
    else
    {
      if((data_ptr[i].samples = sns_ddf_memhandler_malloc_ex(memhandler,
                                   sizeof(sns_ddf_sensor_sample_s), state->smgr_handle)) == NULL)
      {
        return SNS_DDF_ENOMEM;
      }
      data_ptr[i].samples[0].sample = FX_FLTTOFIX_Q16(temperature/100.0);
      data_ptr[i].samples[0].status = SNS_DDF_SUCCESS;
      data_ptr[i].num_samples = 1;
      state->data_cache[1] = FX_FLTTOFIX_Q16(temperature/100.0);
      sns_dd_qmp6988_log_data(state, data_ptr[i].timestamp, sensors[i]);
	  QMP6988_MSG_2(HIGH, " temperature value in Q16 is %d, TS is %d \n",state->data_cache[1], data_ptr[i].timestamp);
     }
  }

  return status;
}


#if QMP6988_CONFIG_RUN_ON_OSSC
sns_ddf_driver_if_s sns_dd_vendor_if_1 =
#else
sns_ddf_driver_if_s sns_qmp6988_driver_fn_list =
#endif
{
    .init                 = &sns_dd_qmp6988_init,
    .get_data             = &sns_dd_qmp6988_get_data,
    .set_attrib           = &sns_dd_qmp6988_set_attr,
    .get_attrib           = &sns_dd_qmp6988_get_attr,
    .handle_irq           = NULL,
    .reset                = &sns_dd_qmp6988_reset,
    .run_test             = &sns_dd_qmp6988_self_test,
    .enable_sched_data    = NULL,
    .probe                = sns_dd_qmp6988_probe,
    .trigger_fifo_data    = NULL
};


/*==============================================================================

EDIT HISTORY FOR FILE

This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.

when         who     what, where, why
----------   ---     -----------------------------------------------------------
==============================================================================*/

/*============================================================================
INCLUDE FILES
============================================================================*/

#include "sns_dd_mag_qmc7983.h"
#include <stdint.h>

/*============================================================================
STATIC VARIABLE DEFINITIONS
============================================================================*/
#ifdef QMCX983_SOFTIRON_COMP
extern void softIronCompen(float data[]);
#endif

void sns_dd_qmcx983_log_sensor_data(sns_ddf_sensor_data_s *sensor_data)
{
   sns_log_sensor_data_pkt_s *log_struct_ptr;
   sns_err_code_e err_code;

   /* Allocate log packet */
   if( (err_code = sns_logpkt_malloc(
		QMCX983_LOG_PACKET,
        sizeof(sns_log_sensor_data_pkt_s),
		(void**)&log_struct_ptr)) != SNS_SUCCESS)
   	{
     	// Do nothing
		QMC7983_DMSG_1(HIGH, "logpkg malloc failed, err_code = %d", err_code);
	 	return;
   	}
   	else
	{
		log_struct_ptr->version   = SNS_LOG_SENSOR_DATA_PKT_VERSION;
		log_struct_ptr->sensor_id = SNS_DDF_SENSOR_MAG;
		log_struct_ptr->vendor_id = SNS_DDF_VENDOR_QST;

		log_struct_ptr->timestamp = sensor_data->timestamp;

		log_struct_ptr->num_data_types = 3;
		log_struct_ptr->data[0]   = sensor_data->samples[0].sample;
		log_struct_ptr->data[1]   = sensor_data->samples[1].sample;
		log_struct_ptr->data[2]   = sensor_data->samples[2].sample;
		QMC7983_DMSG_3(HIGH, "sns_dd_qmcx983_log_sensor_data, [%d, %d, %d]", sensor_data->samples[0].sample, sensor_data->samples[1].sample, sensor_data->samples[2].sample);
		/* Commit log (also frees up the log packet memory) */
		sns_logpkt_commit(QMCX983_LOG_PACKET, log_struct_ptr);
	}
}

void sns_dd_qmcx983_report_data(
	sns_dd_qmc7983_state_t *state, 
	sns_ddf_time_t timestamp)
{
	sns_ddf_status_e status;
	uint8_t read_count[1]  = { 0 };
	uint8_t read_buffer[6] = { 0 };
	float tmp_data[3] = { 0.f };
	int16_t idata[3] = { 0 };
	sns_ddf_sensor_data_s*    sensor_data_report;
	sns_ddf_sensor_sample_s*  data_ptr;
  
  	/* allocate memory for sensor data */
  	if ((status = sns_ddf_malloc_ex(
		(void **)&sensor_data_report,
		sizeof(sns_ddf_sensor_data_s) , 
		state->smgr_handle)) != SNS_DDF_SUCCESS)
  	{
    	return;
  	}
	/* allocate memory for sensor sub-samples */
  	if ((status = sns_ddf_malloc_ex(
		  (void **)&data_ptr,
		  3 * sizeof(sns_ddf_sensor_sample_s) , 
		  state->smgr_handle)) != SNS_DDF_SUCCESS)
  	{
    	sns_ddf_mfree_ex(sensor_data_report, state->smgr_handle);
    	return;
  	} 
  	if((status = sns_ddf_read_port(
		state->port_handle, 
		QMCX983_X_LSB_REG, 
		read_buffer, 
		6, 
		read_count)) != SNS_DDF_SUCCESS)
  	{
  		sns_ddf_mfree_ex(data_ptr, state->smgr_handle);
		sns_ddf_mfree_ex(sensor_data_report, state->smgr_handle);
		return;
	}

	idata[QMCX983_AXIS_X] = (int16_t)((((uint16_t)read_buffer[1]) << 8) | 
										(uint16_t)read_buffer[0]);
	idata[QMCX983_AXIS_Y] = (int16_t)((((uint16_t)read_buffer[3]) << 8) | 
										(uint16_t)read_buffer[2]);
	idata[QMCX983_AXIS_Z] = (int16_t)((((uint16_t)read_buffer[5]) << 8) | 
										(uint16_t)read_buffer[4]);
  
	tmp_data[QMCX983_AXIS_X] = (float)idata[QMCX983_AXIS_X];
	tmp_data[QMCX983_AXIS_Y] = (float)idata[QMCX983_AXIS_Y];
	tmp_data[QMCX983_AXIS_Z] = (float)idata[QMCX983_AXIS_Z];

  	tmp_data[QMCX983_AXIS_Z] = 
		tmp_data[QMCX983_AXIS_Z] - 
		tmp_data[QMCX983_AXIS_X] * state->opt_k[OPT_KX] * 0.02f - 
		tmp_data[QMCX983_AXIS_Y] * state->opt_k[OPT_KY] *0.02f;
  	idata[2] = (int16_t)tmp_data[QMCX983_AXIS_Z];
  	QMC7983_DMSG_3(HIGH,"RawMagData, %d,%d,%d",idata[0],idata[1],idata[2]);
#ifdef QMCX983_SOFTIRON_COMP
  	softIronCompen(tmp_data);
#endif	
	/* create a report */
	sensor_data_report->sensor      = SNS_DDF_SENSOR_MAG;
	sensor_data_report->status      = SNS_DDF_SUCCESS;
	sensor_data_report->samples     = data_ptr;
	sensor_data_report->num_samples = QMCX983_AXIS_NUMS;
	sensor_data_report->timestamp   = timestamp;

	state->data_cache[QMCX983_AXIS_X] = FX_FLTTOFIX_Q16(tmp_data[QMCX983_AXIS_X]/2500);
	state->data_cache[QMCX983_AXIS_Y] = FX_FLTTOFIX_Q16(tmp_data[QMCX983_AXIS_Y]/2500);
	state->data_cache[QMCX983_AXIS_Z] = FX_FLTTOFIX_Q16(tmp_data[QMCX983_AXIS_Z]/2500);

	sns_ddf_map_axes(&state->axes_map, &state->data_cache[QMCX983_AXIS_X]);

	sensor_data_report->samples[QMCX983_AXIS_X].sample = state->data_cache[QMCX983_AXIS_X];
	sensor_data_report->samples[QMCX983_AXIS_Y].sample = state->data_cache[QMCX983_AXIS_Y];
	sensor_data_report->samples[QMCX983_AXIS_Z].sample = state->data_cache[QMCX983_AXIS_Z];

	sensor_data_report->samples[QMCX983_AXIS_X].status = SNS_DDF_SUCCESS;
	sensor_data_report->samples[QMCX983_AXIS_Y].status = SNS_DDF_SUCCESS;
	sensor_data_report->samples[QMCX983_AXIS_Z].status = SNS_DDF_SUCCESS;

	sns_ddf_smgr_notify_data(state->smgr_handle, sensor_data_report, 1);

	sns_dd_qmcx983_log_sensor_data(sensor_data_report);

	sns_ddf_mfree_ex(sensor_data_report, state->smgr_handle);
	sns_ddf_mfree_ex(data_ptr, state->smgr_handle);
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_handle_irq

===========================================================================*/
/**
* @brief Called in response to an interrupt for this driver.
*
* @note This function will be called within the context of the SMGR task,
*       *not* the ISR.
*
* @param[in] dd_handle  Handle to a driver instance.
* @param[in] gpio_num   GPIO number that triggered this interrupt.
* @param[in] timestamp  Time at which interrupt happened.
*/
/*=========================================================================*/
void sns_dd_qmcx983_handle_irq
(
    sns_ddf_handle_t  dd_handle,
    uint32_t          gpio_num,
    sns_ddf_time_t    timestamp
)
{
  /* Read status, handle interrupt, read data and return data to SMGR */
  sns_dd_qmc7983_state_t* state = (sns_dd_qmc7983_state_t *)dd_handle;
  QMC7983_DMSG_0(HIGH, "sns_dd_qmcx983_handle_irq");

  if (state == NULL)
  {
    return;
  }

  sns_dd_qmcx983_report_data(state, timestamp);

}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_handle_timer

===========================================================================*/
/**
* @brief Called when the timer set by this driver has expired. This must be
*        the callback function submitted when initializing a timer.
*
* @note This will be called within the context of the Sensors Manager task.
*
* @param[in] dd_handle  Handle to a driver instance.
* @param[in] arg        The argument submitted when the timer was set.
*
*/
/*=========================================================================*/
void sns_dd_qmcx983_handle_timer
(
    sns_ddf_handle_t dd_handle,
    void* arg
)
{
	sns_dd_qmc7983_state_t* state = (sns_dd_qmc7983_state_t *)dd_handle;
	//QMC7983_DMSG_0(HIGH, "sns_dd_qmcx983_handle_timer");
	if (state == NULL)
	{
		return;
	}

	sns_dd_qmcx983_report_data(state, sns_ddf_get_timestamp());
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_get_data

===========================================================================*/
/**
* @brief Retrieves a single set of sensor data.
*
* @param[in]  dd_handle    Handle to a driver instance.
* @param[in]  sensors      List of sensors for which data is requested.
* @param[in]  num_sensors  Number of elements in @a sensors.
* @param[in]  memhandler   Memory handler used to dynamically allocate
*                          output parameters, if applicable.
* @param[out] data         Sampled sensor data. The number of elements must
*                          match @a num_sensors.
*
* @return SNS_DDF_SUCCESS if data was populated successfully. If any of the
*         sensors queried are to be read asynchronously SNS_DDF_PENDING is
*         returned and data is via @a sns_ddf_smgr_data_notify() when
*         available. Otherwise a specific error code is returned.
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_get_data
    (
    sns_ddf_handle_t        dd_handle,
    sns_ddf_sensor_e        sensors[],
    uint32_t                num_sensors,
    sns_ddf_memhandler_s*   memhandler,
    sns_ddf_sensor_data_s*  data[] /* ignored by this async driver */
)
{
	sns_dd_qmc7983_state_t* state = (sns_dd_qmc7983_state_t *) dd_handle;
		
	QMC7983_DMSG_0(HIGH, "get_data");

	if (state == NULL) {
		return SNS_DDF_EINVALID_PARAM;
	}
	
	sns_ddf_timer_start(state->sns_dd_tmr_obj, 1000);

	return SNS_DDF_PENDING;
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_trigger_fifo_data

===========================================================================*/
/**
* @brief Retrieves a set of sensor data. Asynchronous API
*
* @note If a sensor has failed or
*       isn't available, @a sns_ddf_sensor_data_s.status must be used to
*       reflect this status.
*
* @param[in]  dd_handle    	Handle to a driver instance.
* @param[in]  sensor      	sensor for which data is requested.
*
* @param[in]  num_samples  	number of samples to retrieve as available. Drain the FIFO if value is set to Zero.
* @param[in]  trigger now  	trigger notify fifo data now or
*       later when trigger_now is set to true.
*
*
* @return SNS_DDF_SUCCESS 	if data was populated successfully.
*         via sns_ddf_smgr_data_notify() or if trigger_now is
*         set to false; Otherwise a specific error code is
*         returned.
*
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_trigger_fifo_data
(
    sns_ddf_handle_t         dd_handle,
    sns_ddf_sensor_e         sensor,
    uint16_t                 num_samples,
    bool                     trigger_now
)
{
  sns_ddf_status_e status = SNS_DDF_SUCCESS;
  QMC7983_DMSG_0(HIGH, "sns_dd_qmcx983_trigger_fifo_data");
  return status;
}

/*============================================================================
Callbacks setting
============================================================================*/
sns_ddf_driver_if_s SNS_DD_QMCX983_IF =
{
    .init = &sns_dd_qmcx983_init,
    .get_data = &sns_dd_qmcx983_get_data,
    .set_attrib = &sns_dd_qmcx983_set_attrib,
    .get_attrib = &sns_dd_qmcx983_get_attrib,
    .handle_timer = &sns_dd_qmcx983_handle_timer,
    .handle_irq = &sns_dd_qmcx983_handle_irq,
    .reset = &sns_dd_qmcx983_reset,
    .run_test = &sns_dd_qmcx983_run_test,
    .enable_sched_data = &sns_dd_qmcx983_enable_sched_data,
    .probe = &sns_dd_qmcx983_probe,
    .trigger_fifo_data = &sns_dd_qmcx983_trigger_fifo_data
};


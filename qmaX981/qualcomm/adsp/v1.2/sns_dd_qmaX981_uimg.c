#include "sns_dd_qmax981.h"

sns_ddf_driver_if_s sns_dd_qmaX981_if =
{
    .init                 = &sns_dd_qmaX981_init,
    .get_data             = &sns_dd_qmaX981_get_data,
    .set_attrib           = &sns_dd_qmaX981_set_attr,
    .get_attrib           = &sns_dd_qmaX981_get_attr,
    .handle_timer         = &sns_dd_qmaX981_handle_timer,
    .handle_irq           = sns_dd_qmaX981_handle_irq,
    .reset                = &sns_dd_qmaX981_reset,
    .run_test             = &sns_dd_qmaX981_self_test,
    .enable_sched_data    = NULL,
    .probe                = &sns_dd_qmaX981_probe,
    .trigger_fifo_data    = NULL
};

/*==============================================================================

  FUNCTION:   sns_dd_qma6981_notify_smgr_err

==============================================================================*/
/*!
  @brief Called SMGR needs to be notified of error

  @note This will  be called from sns_dd_handle_timer  
  when an error is seen in memory  allocation or 
  problem in reading the data from the Sensor. 

  @param[in] *state               ptr to the driver state structure
  @param[in] err_code             error code for the problem
  @param[in] sensor_data_report   Sensor data to pass to the Sensors Manager.
  @param[in] num_sensors          Length of sensor_data_report.

  @return
   None

  $TODO:
*/
/*============================================================================*/
static void sns_dd_qma6981_notify_smgr_err(
	sns_dd_qmaX981_state_t* state,
	sns_ddf_status_e            err_code,
	sns_ddf_sensor_data_s      data[],
	uint32_t                      data_len)
{
	uint8_t i;

	for(i = 0; i < (uint8_t)data_len; i++)
	{
		data[i].sensor    = SNS_DDF_SENSOR_ACCEL;
		data[i].status    = err_code;
		data[i].timestamp = sns_ddf_get_timestamp();
	}
	sns_ddf_smgr_notify_data(state->smgr_hndl, data, data_len);
}


void sns_dd_qmaX981_log_data(sns_dd_qmaX981_state_t *state, sns_ddf_time_t sample_time)
{
#ifndef QST_COMPILE_FOR_HD22
	sns_log_sensor_data_pkt_s *log_struct_ptr;
	sns_err_code_e err_code;

	if((err_code = sns_logpkt_malloc(QMAX981_LOG_PACKET,
   		sizeof(sns_log_sensor_data_pkt_s),(void**)&log_struct_ptr)) != SNS_SUCCESS)
   	{
		QMAX981_MSG_1(HIGH, "logpkg malloc failed, err_code = %d", err_code);
	 	return;
   	}
   	else
	{
		log_struct_ptr->version   = SNS_LOG_SENSOR_DATA_PKT_VERSION;
		log_struct_ptr->sensor_id = SNS_DDF_SENSOR_ACCEL;
		log_struct_ptr->vendor_id = SNS_DDF_VENDOR_QST;
		log_struct_ptr->timestamp = sample_time;
		log_struct_ptr->num_data_types = QST_ACC_NUM_AXIS;
		log_struct_ptr->data[0]   = state->data_cache[0];
		log_struct_ptr->data[1]   = state->data_cache[1];
		log_struct_ptr->data[2]   = state->data_cache[2];
		QMAX981_MSG_3(HIGH, "sns_dd_qmaX981_log_data, [%d, %d, %d]",state->data_cache[0], state->data_cache[1], state->data_cache[2]);
		sns_logpkt_commit(QMAX981_LOG_PACKET, log_struct_ptr);
	}
#endif
}


sns_ddf_status_e sns_dd_qmaX981_report_data(sns_dd_qmaX981_state_t *state, sns_ddf_time_t timestamp)
{
	sns_ddf_status_e status;
	uint8_t read_buffer[6] = { 0 };
	int16_t hw_d[QST_ACC_NUM_AXIS] = { 0 };
	sns_ddf_sensor_data_s*    sensor_data_report;
	sns_ddf_sensor_sample_s*  data_ptr;
	
	QMAX981_MSG_0(HIGH,"sns_dd_qmaX981_report_data, entry");
  	/* allocate memory for sensor data */
  	if((status = sns_ddf_malloc_ex((void **)&sensor_data_report,sizeof(sns_ddf_sensor_data_s),state->smgr_hndl)) != SNS_DDF_SUCCESS)
  	{
		QMAX981_MSG_0(ERROR,"sns_dd_qmaX981_report_data, sensor_data_report allocate fail");
		sns_ddf_smgr_notify_data(state, NULL, 0);
    	return status;
  	}
	/* allocate memory for sensor sub-samples */
  	if((status = sns_ddf_malloc_ex((void **)&data_ptr,QST_ACC_NUM_AXIS * sizeof(sns_ddf_sensor_sample_s),state->smgr_hndl)) != SNS_DDF_SUCCESS)
  	{
		sns_dd_qma6981_notify_smgr_err(state, status, sensor_data_report, 1);
    	sns_ddf_mfree_ex(sensor_data_report, state->smgr_hndl);
    	return status;
  	} 
	status = qmaX981_read_bytes(state->port_handle,QMAX981_X_AXIS_LSB_REG,read_buffer,6);
  	if(status != SNS_DDF_SUCCESS)
  	{
  		sns_ddf_mfree_ex(data_ptr, state->smgr_hndl);
		sns_ddf_mfree_ex(sensor_data_report, state->smgr_hndl);
		return status;
	}

    hw_d[0] = (int16_t)(read_buffer[0] | (read_buffer[1] << 8));
    hw_d[1] = (int16_t)(read_buffer[2] | (read_buffer[3] << 8));
    hw_d[2] = (int16_t)(read_buffer[4] | (read_buffer[5] << 8));
	if(state->qma_chip == QMA_6981)
	{
		hw_d[0] = hw_d[0]>>6;
		hw_d[1] = hw_d[1]>>6;
		hw_d[2] = hw_d[2]>>6;
		
		state->data_cache[0] = FX_FLTTOFIX_Q16(hw_d[0]*2*state->sens*G/1024);
		state->data_cache[1] = FX_FLTTOFIX_Q16(hw_d[1]*2*state->sens*G/1024);
		state->data_cache[2] = FX_FLTTOFIX_Q16(hw_d[2]*2*state->sens*G/1024);
	}
	else if((state->qma_chip == QMA_7981)||(state->qma_chip == QMA_6100))
	{
		hw_d[0] = hw_d[0]>>2;
		hw_d[1] = hw_d[1]>>2;
		hw_d[2] = hw_d[2]>>2;
		state->data_cache[0] = FX_FLTTOFIX_Q16(hw_d[0]*2*state->sens*G/16384);
		state->data_cache[1] = FX_FLTTOFIX_Q16(hw_d[1]*2*state->sens*G/16384);
		state->data_cache[2] = FX_FLTTOFIX_Q16(hw_d[2]*2*state->sens*G/16384);
	}
	QMAX981_MSG_3(HIGH,"report_data hw_d[%d %d %d]",hw_d[0],hw_d[1],hw_d[2]);
	sns_ddf_map_axes(&state->axes_map, state->data_cache);
	data_ptr[0].sample = state->data_cache[0];
	data_ptr[1].sample = state->data_cache[1];
	data_ptr[2].sample = state->data_cache[2];
	data_ptr[0].status = SNS_DDF_SUCCESS;
	data_ptr[1].status = SNS_DDF_SUCCESS;
	data_ptr[2].status = SNS_DDF_SUCCESS;
	
	sensor_data_report->sensor = SNS_DDF_SENSOR_ACCEL;
    sensor_data_report->status = SNS_DDF_SUCCESS;
    sensor_data_report->timestamp = timestamp;
	sensor_data_report->samples  = data_ptr;
    sensor_data_report->num_samples = QST_ACC_NUM_AXIS;
	QMAX981_MSG_0(HIGH,"sns_dd_qmaX981_report_data, done");
	status = sns_ddf_smgr_notify_data(state->smgr_hndl, sensor_data_report, 1);
	if(status != SNS_DDF_SUCCESS)
	{
		return status;
	}
	sns_dd_qmaX981_log_data(state, sensor_data_report->timestamp);
	sns_ddf_mfree_ex(data_ptr, state->smgr_hndl);
	sns_ddf_mfree_ex(sensor_data_report, state->smgr_hndl);
	
	return SNS_DDF_SUCCESS;
}


void sns_dd_qmaX981_handle_timer(sns_ddf_handle_t dd_handle, void* arg)
{
    QMAX981_MSG_0(HIGH, "sns_dd_qmaX981_handle_timer, entry");

    sns_dd_qmaX981_state_t *state = (sns_dd_qmaX981_state_t *)dd_handle;

	if(state == NULL)
	{
		return;
	}
	
	sns_dd_qmaX981_report_data(state,sns_ddf_get_timestamp());
}


/*===========================================================================

FUNCTION:   sns_dd_qmaX981_get_data

===========================================================================*/
sns_ddf_status_e sns_dd_qmaX981_get_data(
    sns_ddf_handle_t        dd_handle,
    sns_ddf_sensor_e        sensors[],
    uint32_t                num_sensors,
    sns_ddf_memhandler_s*   memhandler,
    sns_ddf_sensor_data_s*  data[]
)
{
	sns_dd_qmaX981_state_t* state = (sns_dd_qmaX981_state_t *) dd_handle;
	sns_ddf_status_e status;
	uint8_t i;
	QMAX981_MSG_0(HIGH, "sns_dd_qmaX981_get_data, entry");

	/* Sanity check*/
	for (i = 0; i < num_sensors; i++)
	{
		if (sensors[i] != SNS_DDF_SENSOR_ACCEL )
		{
			QMAX981_MSG_0(FATAL, "in qmaX981_get_data main err, not accel requirement");     
			return SNS_DDF_EINVALID_PARAM;
		}
	}
	
	status = sns_ddf_timer_start(state->sns_dd_tmr_obj, 10000);
	
	if(status != SNS_DDF_SUCCESS)
	{
		QMAX981_MSG_0(FATAL, "in qmaX981_get_data, timerstart error");
		return status;
	}

	return SNS_DDF_PENDING;
}


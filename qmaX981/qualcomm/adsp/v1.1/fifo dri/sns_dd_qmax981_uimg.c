#include "sns_dd_qmax981.h"


sns_ddf_driver_if_s sns_dd_qmax981_if =
{
    .init                 = &sns_dd_qmax981_init,
    .get_data             = &sns_dd_qmax981_get_data,
    .set_attrib           = &sns_dd_qmax981_set_attr,
    .get_attrib           = &sns_dd_qmax981_get_attr,
    .handle_timer         = &sns_dd_qmax981_handle_timer,
    .handle_irq           = &sns_dd_qmax981_handle_irq,
    .reset                = &sns_dd_qmax981_reset,
    .run_test             = &sns_dd_qmax981_self_test,
    .enable_sched_data    = &sns_dd_qmax981_enable_sched_data,
    .probe                = &sns_dd_qmax981_probe,
    .trigger_fifo_data    = &sns_dd_qmax981_trigger_fifo_data
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
    sns_dd_qmax981_state_t* state,
    sns_ddf_status_e            err_code,
    sns_ddf_sensor_data_s      data[],
    uint32_t                      data_len)
{
    uint8_t i;

    for (i = 0; i < (uint8_t)data_len; i++)
    {
        data[i].sensor    = SNS_DDF_SENSOR_ACCEL;
        data[i].status    = err_code;
        data[i].timestamp = sns_ddf_get_timestamp();
    }
    sns_ddf_smgr_notify_data(state->smgr_hndl, data, data_len);
}

/*===========================================================================
FUNCTION      sns_dd_qmax981_log_data

DESCRIPTION  Logs latest set of sensor data sampled from the sensor.
============================================================================*/
void sns_dd_qmax981_log_data(
    sns_dd_qmax981_state_t *state,
    sns_ddf_time_t  sample_time)
{
   sns_log_sensor_data_pkt_s *log_struct_ptr;
   sns_err_code_e err_code;

#ifdef QMAX981_ENABLE_UIMAGE_SUPPORT
	if (sns_ddf_smgr_is_in_uimage()) {
		return;
	}
#endif
   /* Allocate log packet */
   if( (err_code = sns_logpkt_malloc(
        QMAX981_LOG_PACKET,
        sizeof(sns_log_sensor_data_pkt_s),
        (void**)&log_struct_ptr)) != SNS_SUCCESS)
    {
        // Do nothing
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
        QMAX981_MSG_3(HIGH, "sns_dd_qmax981_log_data, [%d, %d, %d]",
        state->data_cache[0], state->data_cache[1], state->data_cache[2]);
        /* Commit log (also frees up the log packet memory) */
        sns_logpkt_commit(QMAX981_LOG_PACKET, log_struct_ptr);
    }
}

sns_ddf_status_e sns_dd_qmax981_report_data(
    sns_dd_qmax981_state_t *state,
    sns_ddf_time_t timestamp)
{
    sns_ddf_status_e status;
    uint8_t read_buffer[6] = { 0 };
    int16_t hw_d[QST_ACC_NUM_AXIS] = { 0 };
    sns_ddf_sensor_data_s*    sensor_data_report;
    sns_ddf_sensor_sample_s*  data_ptr;

    QMAX981_MSG_0(HIGH,"sns_dd_qmax981_report_data, entry");
    /* allocate memory for sensor data */
    if ((status = sns_ddf_malloc_ex(
        (void **)&sensor_data_report,
        sizeof(sns_ddf_sensor_data_s) ,
        state->smgr_hndl)) != SNS_DDF_SUCCESS)
    {
        QMAX981_MSG_0(ERROR,"sns_dd_qmax981_report_data, sensor_data_report allocate fail");
        sns_ddf_smgr_notify_data(state, NULL, 0);
        return status;
    }
    /* allocate memory for sensor sub-samples */
    if ((status = sns_ddf_malloc_ex(
        (void **)&data_ptr,
        QST_ACC_NUM_AXIS * sizeof(sns_ddf_sensor_sample_s) ,
        state->smgr_hndl)) != SNS_DDF_SUCCESS)
    {
        sns_dd_qma6981_notify_smgr_err(state, status, sensor_data_report, 1);
        sns_ddf_mfree_ex(sensor_data_report, state->smgr_hndl);
        return status;
    }

    status = qmax981_read_bytes(state->port_handle,QMAX981_X_AXIS_LSB_REG,read_buffer,6);

    if(status != SNS_DDF_SUCCESS)
    {
        sns_ddf_mfree_ex(data_ptr, state->smgr_hndl);
        sns_ddf_mfree_ex(sensor_data_report, state->smgr_hndl);
        return status;
    }

    hw_d[0] = (int16_t) (read_buffer[2] | (read_buffer[3] << 8));
    hw_d[1] = -(int16_t) (read_buffer[0] | (read_buffer[1] << 8));
    hw_d[2] = -(int16_t) (read_buffer[4] | (read_buffer[5] << 8));
    hw_d[0] = hw_d[0]>>6;
    hw_d[1] = hw_d[1]>>6;
    hw_d[2] = hw_d[2]>>6;
    QMAX981_MSG_3(HIGH,"report_data hw_d[%d %d %d]",hw_d[0],hw_d[1],hw_d[2]);

    state->data_cache[0] = FX_FLTTOFIX_Q16(hw_d[0] * 2 * state->sstvt_adj * G/1024);
    state->data_cache[1] = FX_FLTTOFIX_Q16(hw_d[1] * 2 * state->sstvt_adj * G/1024);
    state->data_cache[2] = FX_FLTTOFIX_Q16(hw_d[2] * 2 * state->sstvt_adj * G/1024);
    QMAX981_MSG_3(HIGH,"report_data hw_d[%d %d %d]",hw_d[0],hw_d[1],hw_d[2]);
    sns_ddf_map_axes(&state->axes_map, state->data_cache);

    /* create a report */
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

    QMAX981_MSG_0(HIGH,"sns_dd_qmax981_report_data, done");
    status =  sns_ddf_smgr_notify_data(state->smgr_hndl, sensor_data_report, 1);
    if(status != SNS_DDF_SUCCESS)
        return status;

    sns_dd_qmax981_log_data(state,sensor_data_report->timestamp);

    sns_ddf_mfree_ex(data_ptr, state->smgr_hndl);
    sns_ddf_mfree_ex(sensor_data_report, state->smgr_hndl);

    return SNS_DDF_SUCCESS;
}


sns_ddf_status_e qmax981_read_fifo_frames(sns_dd_qmax981_state_t* state, uint8_t fifo_level);
void sns_dd_qma6981_log_fifo(sns_ddf_sensor_data_s *accel_data_ptr);

void sns_dd_qmax981_handle_timer(
            sns_ddf_handle_t dd_handle,
            void* arg)
{
    sns_ddf_status_e status;
	uint8_t fifo_status;

    QMAX981_MSG_0(HIGH, "sns_dd_qmax981_handle_timer, entry");

    sns_dd_qmax981_state_t *state = (sns_dd_qmax981_state_t *)dd_handle;

    if(state == NULL)
    {
        return;
    }

    sns_dd_qmax981_report_data(state,sns_ddf_get_timestamp());
}


/*===========================================================================

FUNCTION:   sns_dd_qmax981_get_data

===========================================================================*/
sns_ddf_status_e sns_dd_qmax981_get_data(
    sns_ddf_handle_t        dd_handle,
    sns_ddf_sensor_e        sensors[],
    uint32_t                num_sensors,
    sns_ddf_memhandler_s*   memhandler,
    sns_ddf_sensor_data_s*  data[]
)
{
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *) dd_handle;
    sns_ddf_status_e status;
    unsigned char i;
    QMAX981_MSG_0(HIGH, "sns_dd_qmax981_get_data, entry");

    /* Sanity check*/
    for (i = 0; i < num_sensors; i++)
    {
        if (sensors[i] != SNS_DDF_SENSOR_ACCEL )
        {
            QMAX981_MSG_0(FATAL, "in qmax981_get_data main err, not accel requirement");
            return SNS_DDF_EINVALID_PARAM;
        }
    }

    status = sns_ddf_timer_start(state->sns_dd_tmr_obj, 10000);

    if(status != SNS_DDF_SUCCESS)
    {
        QMAX981_MSG_0(FATAL, "in qmax981_get_data, timerstart error");
        return status;
    }

    return SNS_DDF_PENDING;
}


void sns_dd_qma6981_log_fifo(sns_ddf_sensor_data_s *accel_data_ptr)
{
#if 1//
    sns_err_code_e err_code;
    sns_log_sensor_data_pkt_s* log_struct_ptr;
    uint16 idx = 0;
    log_pkt_t                  log_pkt_type;

#ifdef QMAX981_ENABLE_UIMAGE_SUPPORT
	if (sns_ddf_smgr_is_in_uimage()) {
		return;
	}
#endif

#if 0//BMA2X2_CONFIG_RUN_ON_OSSC || BMA2X2_CONFIG_ENABLE_COMPILE_CHECK
    log_pkt_type = SNS_LOG_CONVERTED_SENSOR_DATA_PUBLIC;
#else
    log_pkt_type = SNS_LOG_CONVERTED_SENSOR_DATA;
#endif


    //Allocate log packet
    err_code = sns_logpkt_malloc(log_pkt_type,
            sizeof(sns_log_sensor_data_pkt_s) + (accel_data_ptr->num_samples -1)*sizeof(int32_t),
            (void**)&log_struct_ptr);
    if ((err_code == SNS_SUCCESS) && (log_struct_ptr != NULL))
    {
        log_struct_ptr->version = SNS_LOG_SENSOR_DATA_PKT_VERSION;
        log_struct_ptr->sensor_id = SNS_DDF_SENSOR_ACCEL;
        log_struct_ptr->vendor_id = SNS_DDF_VENDOR_QST;

        //Timestamp the log with sample time
        log_struct_ptr->timestamp = accel_data_ptr->timestamp;
        log_struct_ptr->end_timestamp = accel_data_ptr->end_timestamp;

        log_struct_ptr->num_data_types = QST_ACC_NUM_AXIS;
        log_struct_ptr->num_samples = accel_data_ptr->num_samples / QST_ACC_NUM_AXIS;

        //Log the sensor fifo data
        log_struct_ptr->data[0]  = accel_data_ptr->samples[0].sample;
        log_struct_ptr->data[1]  = accel_data_ptr->samples[1].sample;
        log_struct_ptr->data[2]  = accel_data_ptr->samples[2].sample;

        for(idx=0; idx<accel_data_ptr->num_samples; idx++)
        {
            log_struct_ptr->samples[idx]  = accel_data_ptr->samples[idx].sample;
        }

        //Commit log (also frees up the log packet memory)
        (void) sns_logpkt_commit(log_pkt_type,
                log_struct_ptr);
    }
#endif  //#if BMA2X2_CONFIG_FIFO_LOG

}


sns_ddf_status_e qmax981_read_fifo_frames(sns_dd_qmax981_state_t* state, uint8_t fifo_level)
{
    sns_ddf_status_e status;
    unsigned char val;
	unsigned char	iCount;	
    unsigned char offset = 0;
    unsigned char fifo_data_out[QMA6981_MAX_FIFO_LEVEL*6] = {0};
    int16_t hw_d[QST_ACC_NUM_AXIS] = { 0 };
    sns_ddf_time_t ts_gap;

    //state->irq_ts_end = sns_ddf_get_timestamp();
    if((status = qmax981_read_bytes(state->port_handle,
           			QMAX981_FIFO_DATA_OUTPUT_REG,
                    fifo_data_out, fifo_level*6))
            != SNS_DDF_SUCCESS) {
        QMAX981_MSG_0(FATAL, "qmax981_read_fifo_frames error");
        return status;
    }

	for(iCount=0; iCount<fifo_level; iCount++)
	{
		offset = iCount*6;
	    hw_d[0] = (int16_t) (fifo_data_out[offset+2] | (fifo_data_out[offset+3] << 8));
	    hw_d[1] = -(int16_t) (fifo_data_out[offset+0] | (fifo_data_out[offset+1] << 8));
	    hw_d[2] = -(int16_t) (fifo_data_out[offset+4] | (fifo_data_out[offset+5] << 8));
	    hw_d[0] = hw_d[0]>>6;
	    hw_d[1] = hw_d[1]>>6;
	    hw_d[2] = hw_d[2]>>6;

		
		state->data_cache_fifo[0] = FX_FLTTOFIX_Q16(hw_d[0] * 2 * state->sstvt_adj * G/1024);
		state->data_cache_fifo[1] = FX_FLTTOFIX_Q16(hw_d[1] * 2 * state->sstvt_adj * G/1024);
		state->data_cache_fifo[2] = FX_FLTTOFIX_Q16(hw_d[2] * 2 * state->sstvt_adj * G/1024);
        //QMAX981_MSG_1(HIGH,"read_fifo[%d]",iCount);
		QMAX981_MSG_3(HIGH,"hw_d[%d %d %d]", state->data_cache_fifo[0],state->data_cache_fifo[1],state->data_cache_fifo[2]);
		sns_ddf_map_axes(&state->axes_map, state->data_cache_fifo);

        state->f_frames_cache.samples[QST_ACC_NUM_AXIS*iCount+0].sample = state->data_cache_fifo[0];
        state->f_frames_cache.samples[QST_ACC_NUM_AXIS*iCount+1].sample = state->data_cache_fifo[1];
        state->f_frames_cache.samples[QST_ACC_NUM_AXIS*iCount+2].sample = state->data_cache_fifo[2];

        state->f_frames_cache.samples[QST_ACC_NUM_AXIS*iCount+0].status = SNS_DDF_SUCCESS;
        state->f_frames_cache.samples[QST_ACC_NUM_AXIS*iCount+1].status = SNS_DDF_SUCCESS;
        state->f_frames_cache.samples[QST_ACC_NUM_AXIS*iCount+2].status = SNS_DDF_SUCCESS;
	}
	
    state->f_frames_cache.status = SNS_DDF_SUCCESS;
    state->f_frames_cache.sensor = SNS_DDF_SENSOR_ACCEL;
    state->f_frames_cache.num_samples = fifo_level * QST_ACC_NUM_AXIS;
	if(fifo_level == 1)
		state->f_frames_cache.timestamp = state->irq_ts_end;
	else
		state->f_frames_cache.timestamp = state->irq_ts_start;
	state->f_frames_cache.end_timestamp = state->irq_ts_end;
#if 0    
    //if(((state->irq_ts_end) > (state->irq_ts_start))&&(fifo_level>1))
    //{
    //    ts_gap = (state->irq_ts_end - state->irq_ts_start)/(fifo_level-1);
    //    state->f_frames_cache.timestamp  = state->f_frames_cache.end_timestamp + ts_gap;
    //    state->f_frames_cache.end_timestamp = state->f_frames_cache.timestamp + ts_gap * (fifo_level - 1);
    //}
    ts_gap = (state->irq_ts_end - state->irq_ts_start)/(fifo_level-1);
    state->f_frames_cache.end_timestamp = state->irq_ts_end - ts_gap;
    state->f_frames_cache.timestamp = state->f_frames_cache.end_timestamp-ts_gap*(fifo_level - 1);
#endif    
    QMAX981_MSG_2(HIGH,"timestamp[%lld, %lld]", state->f_frames_cache.timestamp,state->f_frames_cache.end_timestamp);
	// start fifo int again
	val = 0x40;
	status = qmax981_write_byte(state->port_handle,0x3e,&val);
    return SNS_DDF_SUCCESS;
}

void sns_dd_qmax981_handle_irq(
        sns_ddf_handle_t       dd_handle,
        uint32_t               gpio_num,
        sns_ddf_time_t         timestamp)
{
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *) dd_handle;	
    sns_ddf_status_e status;
	uint8_t fifo_status, int0_status;

    if(state == NULL)
    {
    	QMAX981_MSG_0(HIGH,"qmax981_irq return state NULL!");
        return;
    }	
#if defined(QMA6981_TAP_SUPPORT)
	if(state->tap_irq_flag)
	{
	    status = qmax981_read_bytes(state->port_handle,QMAX981_INT_STAT0_REG,&int0_status,1);
    	QMAX981_MSG_1(HIGH,"qmax981_irq  state0 0x%x", int0_status);
		if(int0_status&0x20)
		{
			QMAX981_MSG_0(HIGH,"qmax981_irq  report SNS_DDF_EVENT_MOTION!");
			status = sns_ddf_smgr_notify_event(state->smgr_hndl,
						SNS_DDF_SENSOR_ACCEL,
						SNS_DDF_EVENT_MOTION);
			if(status != SNS_DDF_SUCCESS)
			{
				QMAX981_MSG_0(HIGH,"qmax981_irq SNS_DDF_EVENT_MOTION error!");
			}
		}
		return;
	}
#endif
	state->irq_ts_end = sns_ddf_get_timestamp();

    status = qmax981_read_bytes(state->port_handle,QMAX981_STATUS_FIFO_REG,&fifo_status,1);
	QMAX981_MSG_3(HIGH,"qmax981_irq OR:%d count:%d status:%d", fifo_status&0x80, fifo_status&0x7f, status);
	if(fifo_status&0x80)
	{
        status = sns_ddf_smgr_notify_event(state->smgr_hndl,
		            SNS_DDF_SENSOR_ACCEL,
		            SNS_DDF_EVENT_FIFO_OVERFLOW);
		if(status != SNS_DDF_SUCCESS)
		{
			QMAX981_MSG_0(HIGH,"qmax981_irq EVENT_FIFO_OVERFLOW error!");
		}
	}
	if((fifo_status&0x7f)>=(state->fifo_level))
	{
		status = sns_ddf_smgr_notify_event(state->smgr_hndl,
					SNS_DDF_SENSOR_ACCEL,
					SNS_DDF_EVENT_FIFO_WM_INT);
		if(status != SNS_DDF_SUCCESS)
		{
			QMAX981_MSG_0(HIGH,"qmax981_irq EVENT_FIFO_WM_INT error!");
		}

		status = qmax981_read_fifo_frames(state, fifo_status&0x7f);
		QMAX981_MSG_1(HIGH,"qmax981_irq qmax981_read_fifo_frames %d!", status);
		status = sns_ddf_smgr_notify_data(state->smgr_hndl, &state->f_frames_cache, 1);
        sns_dd_qma6981_log_fifo(&state->f_frames_cache);
		QMAX981_MSG_1(HIGH,"qmax981_irq sns_ddf_smgr_notify_data %d!", status);

		state->irq_ts_start = state->irq_ts_end;
	}
}

sns_ddf_status_e sns_dd_qmax981_trigger_fifo_data(
        sns_ddf_handle_t    dd_handle,
        sns_ddf_sensor_e    sensor,
        uint16_t            num_samples,
        bool                trigger_now)
{
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *) dd_handle;	
    sns_ddf_status_e status;
	uint8_t fifo_status;
#if 1
	QMAX981_MSG_2(HIGH,"qmax981_trigger_fifo_data %d %d", num_samples, trigger_now);
    status = qmax981_read_bytes(state->port_handle,QMAX981_STATUS_FIFO_REG,&fifo_status,1);
	
	QMAX981_MSG_3(HIGH,"qmax981_trigger_fifo_data OR:%d count:%d status:%d", fifo_status&0x80, fifo_status&0x7f, status);
	
	if(fifo_status&0x80)
	{
        status = sns_ddf_smgr_notify_event(state->smgr_hndl,
		            SNS_DDF_SENSOR_ACCEL,
		            SNS_DDF_EVENT_FIFO_OVERFLOW);
		if(status != SNS_DDF_SUCCESS)
		{
			QMAX981_MSG_0(HIGH,"qmax981_trigger_fifo_data EVENT_FIFO_OVERFLOW error!");
		}
	}
	if((fifo_status&0x7f)>=1&&(fifo_status&0x7f)<QMA6981_MAX_FIFO_LEVEL)	//((fifo_status&0x7f)>=(state->fifo_level))
	{
		//status = sns_ddf_smgr_notify_event(state->smgr_hndl,
		//			SNS_DDF_SENSOR_ACCEL,
		//			SNS_DDF_EVENT_FIFO_WM_INT);
		//if(status != SNS_DDF_SUCCESS)
		//{
		//	QMAX981_MSG_0(HIGH,"qmax981_trigger_fifo_data EVENT_FIFO_WM_INT error!");
		//}
		state->irq_ts_end = sns_ddf_get_timestamp();
		status = qmax981_read_fifo_frames(state, fifo_status&0x7f);
		QMAX981_MSG_1(HIGH,"qmax981_trigger_fifo_data qmax981_read_fifo_frames %d!", status);
		status = sns_ddf_smgr_notify_data(state->smgr_hndl, &state->f_frames_cache, 1);
		QMAX981_MSG_1(HIGH,"qmax981_trigger_fifo_data sns_ddf_smgr_notify_data %d!", status);

		state->irq_ts_start = state->irq_ts_end;
	}
#endif
	return SNS_DDF_SUCCESS;
}


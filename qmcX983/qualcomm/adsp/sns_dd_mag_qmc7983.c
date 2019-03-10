/*==============================================================================    

 * Copyright (c) 2010-2014 Qualcomm Technologies, Inc. All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 
 ==============================================================================*/    

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
STATIC FUNCTION PROTOTYPES
============================================================================*/

void sns_dd_qmcx983_get_OPT(
    sns_ddf_handle_t dd_handle
)
{
	sns_dd_qmc7983_state_t* state = (sns_dd_qmc7983_state_t *)dd_handle;
	int8_t otp;
	uint8_t reg_addr, reg_val;
	uint8_t value[2] = { 0 };
	uint8_t read_buffer[1] = { 0 };
	uint8_t read_count[1]  = { 0 };
	sns_ddf_status_e status;

	if(state->device_select == QMC6983_A1_D1)
	{
		return ;
	}
	if (state->device_select == QMC7983 || 
		state->device_select == QMC7983_SLOW_SETRESET)
	{
		//read Kx
		reg_addr = 0x2e;
		reg_val = 0x0a;
		sns_ddf_write_port_byte(
			state->port_handle, 
			reg_addr, 
			reg_val, 
			&status);
		QMC7983_DMSG_1(HIGH, "Reg[0x2e] write, status = %d", status);

		reg_addr = 0x2f;
	  	status = sns_ddf_read_port(
			state->port_handle, 
			reg_addr, 
			read_buffer, 
			QMCX983_READ_ONE_BYTE, 
			read_count);
	  	QMC7983_DMSG_2(HIGH, "Reg[0x2f] read, status = %d, value = 0x%x", status, read_buffer[0]);

		value[0] = read_buffer[0];

		if(((value[0]&0x3f) >> 5) == 1)
		{
			otp = (value[0]&0x1f)-32;
		}
		else
		{
			otp = value[0]&0x1f;
		}
		state->opt_k[OPT_KX]= otp ;
	}

	//read Ky
	reg_addr = 0x2e;
	reg_val = 0x0d;
	sns_ddf_write_port_byte(
		state->port_handle, 
		reg_addr, 
		reg_val, 
		&status);
	QMC7983_DMSG_1(HIGH, "Reg[0x2e] write, status = %d", status);

	reg_addr = 0x2f;
  	status = sns_ddf_read_port(
		state->port_handle, 
		reg_addr, 
		read_buffer, 
		QMCX983_READ_ONE_BYTE, 
		read_count);
  	QMC7983_DMSG_3(HIGH, "Reg[0x2f] read, status = %d,  count = %d, value[0] = 0x%x", status, read_count[0], read_buffer[0]);

	value[0] = read_buffer[0];

	reg_addr = 0x2e;
	reg_val = 0x0f;
	sns_ddf_write_port_byte(
		state->port_handle, 
		reg_addr, 
		reg_val, 
		&status);
	QMC7983_DMSG_1(HIGH, "Reg[0x2e] write, status = %d", status);
	reg_addr = 0x2f;
  	status = sns_ddf_read_port(
		state->port_handle, 
		reg_addr, 
		read_buffer, 
		QMCX983_READ_ONE_BYTE, 
		read_count);
  	QMC7983_DMSG_3(HIGH, "Reg[0x2f] read, status = %d,  count = %d, value[1] = 0x%x", status, read_count[0], read_buffer[0]);

	value[1] = read_buffer[0];

	if((value[0] >> 7) == 1)
		otp = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6))-32;
	else
		otp = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6));
	state->opt_k[OPT_KY] = otp ;

	QMC7983_DMSG_2(HIGH,"OPT_KXY,%d,%d",state->opt_k[OPT_KX],state->opt_k[OPT_KY]);
}

/*==============================================================================
  STATIC FUNCTION:   sns_mgt_akm_check_device
==============================================================================*/
/*!
  @brief Check device id if the device is supported one or not.

  @param[in]  port_handle  Handle to a communication port.
  @param[out] *device_id   ptr to uint8_t array which size must be larger than 2.
                           When the function succeeded, device information
                           (i.e. WIA1, WIA2) is stored to this variable.

  @return
   The error code definition within the DDF
   SNS_DDF_SUCCESS on success;
==============================================================================*/
static sns_ddf_status_e sns_dd_qmcx983_check_device(
    sns_ddf_handle_t dd_handle
)
{
	sns_dd_qmc7983_state_t* state = (sns_dd_qmc7983_state_t *)dd_handle;
  	uint8_t          out;
	uint8_t			device_id[1] = {0};
	sns_ddf_status_e status;
	
  	/* Read Device ID, to make sure device is working properly */
  	if ((status = sns_ddf_read_port(
        state->port_handle,
        QMCX983_CHIPID_REG,
        device_id,
        QMCX983_READ_ONE_BYTE,
        &out)) != SNS_DDF_SUCCESS) 
    {
    	QMC7983_DMSG_1(HIGH, "Failed to read device id. status = %d.", status);
    	return status;
  	}
  	QMC7983_DMSG_1(MED, " Id = 0x%02X.", device_id[0]);

	if (device_id[0] == 0xff )
  	{
  		state->device_select = QMC6983_A1_D1;
  	}
  	else if (device_id[0] == 0x31) //qmc7983 asic
  	{
  	   	if ((status = sns_ddf_read_port(
			state->port_handle, 
			0x3E, 
			device_id, 
			QMCX983_READ_ONE_BYTE, 
			&out)))
  		{
  			QMC7983_DMSG_1(FATAL,"Reg[0x3e] read failed, status = %d",status);
  			return status;
  		}
  		if ((device_id[0] & 0x20) == 1)
  		{
  			state->device_select = QMC6983_E1;
  		}
		else if((device_id[0] & 0x20) == 0)
		{
  			state->device_select = QMC7983;
		}
  	}
  	else if(device_id[0] == 0x32) //qmc7983 asic low setreset
  	{
  			state->device_select = QMC7983_SLOW_SETRESET;
  	}
  	else
  	{
  		QMC7983_DMSG_0(FATAL,"Unknow device ID!");
  		return SNS_DDF_EDEVICE;;
  	}
	return SNS_DDF_SUCCESS;
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_init

===========================================================================*/
/** 
* @brief Initializes the driver and sets up devices.
*  
* @param[out] dd_handle_ptr  Pointer that this function must malloc and 
*                            populate. This is a handle to the driver
*                            instance that will be passed in to all other
*                            functions. NB: Do not use @a memhandler to
*                            allocate this memory.
* @param[in]  smgr_handle    Handle used to identify this driver when it 
*                            calls into Sensors Manager functions.
* @param[in]  nv_params      NV parameters retrieved for the driver.
* @param[in]  device_info    Access info for physical devices controlled by 
*                            this driver. Used to configure the bus
*                            and talk to the devices.
* @param[in]  num_devices    Number of elements in @a device_info. 
* @param[in]  memhandler     Memory handler used to dynamically allocate 
*                            output parameters, if applicable. NB: Do not
*                            use memhandler to allocate memory for
*                            @a dd_handle_ptr.
* @param[in/out] sensors     List of supported sensors, allocated, 
*                            populated, and returned by this function.
* @param[in/out] num_sensors Number of elements in @a sensors.
*
* @return Success if @a dd_handle_ptr was allocated and the driver was 
*         configured properly. Otherwise a specific error code is returned.
*/
/*=========================================================================*/                                             
sns_ddf_status_e sns_dd_qmcx983_init
(
    sns_ddf_handle_t*        dd_ptr,
    sns_ddf_handle_t         smgr_handle,
    sns_ddf_nv_params_s*     nv_params,
    sns_ddf_device_access_s  device_info[],
    uint32_t                 num_devices,
    sns_ddf_memhandler_s*    memhandler,
    sns_ddf_sensor_e*        sensors[],
    uint32_t*                num_sensors
)
{
	sns_dd_qmc7983_state_t *state;
	sns_ddf_status_e status;
	static sns_ddf_sensor_e dd_qst_sensors[] = { SNS_DDF_SENSOR_MAG };
  
#ifdef QMCX983_ENABLE_UIMG
	/* Update smgr sensor data for the driver to indicate uImage support */
	sns_ddf_smgr_set_uimg_refac(smgr_handle);
#endif
  	QMC7983_DMSG_0(FATAL, "sns_dd_qmcx983_init");

  	/* Input sanity check */
  	if (num_sensors == NULL)
  	{
	  	QMC7983_DMSG_0(FATAL, "SNS_DDF_EINVALID_PARAM");
	 	return SNS_DDF_EINVALID_PARAM;
  	}

  	if ((status = sns_ddf_malloc_ex(
        (void **)&state,
        sizeof(sns_dd_qmc7983_state_t), 
        smgr_handle)) != SNS_DDF_SUCCESS)
  	{
	  	QMC7983_DMSG_0(FATAL, "SNS_DDF_ENOMEM");
    	goto errorCleanUp_0;
  	}
  	memset(state, 0, sizeof(sns_dd_qmc7983_state_t));

  	*dd_ptr = state;

  	if ((status = sns_ddf_open_port(
          &(state->port_handle),
          &(device_info->port_config)))!= SNS_DDF_SUCCESS)
  	{
    	goto errorCleanUp_1;
  	}

  	state->smgr_handle    = smgr_handle;
  	state->gpio1          = device_info->first_gpio;

  	/* Init timer object */
  	if ((status = sns_ddf_timer_init(
  		&state->sns_dd_tmr_obj, 
  		(sns_ddf_handle_t)state, 
  		&SNS_DD_QMCX983_IF, 
  		state, 
  		false)) != SNS_DDF_SUCCESS)
  	{
  		QMC7983_DMSG_0(HIGH, "Failed to initialize timer.");
    	goto errorCleanUp_2;
  	}

  	/* Fill out supported sensor info */
  	*sensors        = dd_qst_sensors;
  	*num_sensors    = QMCX983_SUPPORTED_SENSORS_NUM;

    sns_ddf_axes_map_init(
    	&state->axes_map, 
       	((nv_params != NULL) ? nv_params->data : NULL));
  	
  	/* Read Device ID, to make sure device is working properly */
  	if ((status = sns_dd_qmcx983_check_device(state)) != SNS_DDF_SUCCESS) 
  	{
    	goto errorCleanUp_3;
  	}

  	sns_dd_qmcx983_get_OPT(state);
	/* Perform reset */
	sns_dd_qmcx983_reset(state);

  return SNS_DDF_SUCCESS;
  
errorCleanUp_3:
  sns_ddf_timer_release(state->sns_dd_tmr_obj);
errorCleanUp_2:
  sns_ddf_close_port(state->port_handle);
errorCleanUp_1:
  sns_ddf_mfree_ex((void *)state, smgr_handle);
errorCleanUp_0:
  /* Clear output parameter */
  *dd_ptr      = NULL;
  *sensors     = NULL;
  *num_sensors = 0;

  return status;
}




/*===========================================================================

FUNCTION:   sns_dd_qmcx983_reset

===========================================================================*/
/**
* @brief Resets the driver and device so they return to the state they were 
*        in after init() was called.
*
* @param[in] dd_handle  Handle to a driver instance.
* 
* @return Success if the driver was able to reset its state and the device.
*         Otherwise a specific error code is returned. 
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_reset
(
    sns_ddf_handle_t dd_handle
)
{
	sns_dd_qmc7983_state_t* state = (sns_dd_qmc7983_state_t *)dd_handle;
	sns_ddf_status_e status;

   	/* Input sanity check */
  	if (state == NULL) {
    	return SNS_DDF_EINVALID_PARAM;
  	}
	QMC7983_DMSG_0(HIGH, "sns_dd_qmcx983_reset");
	
	/* Stop timer interrupt before reset the device. */
  	sns_ddf_timer_cancel(state->sns_dd_tmr_obj);
	
  	sns_ddf_write_port_byte(
		state->port_handle,
		QMCX983_CONTROL_1_REG,
		0x1c,
		&status);
	if(status != SNS_DDF_SUCCESS)
	{
		return status;
	}
  	state->odr = 0;
  	return SNS_DDF_SUCCESS;
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_set_attrib

===========================================================================*/
/**
* @brief Sets a sensor attribute to a specific value.
*
* @param[in] dd_handle  Handle to a driver instance.
* @param[in] sensor     Sensor for which this attribute is to be set. When 
*                       addressing an attribute that refers to the driver
*                       this value is set to SNS_DDF_SENSOR__ALL.
* @param[in] attrib     Attribute to be set.
* @param[in] value      Value to set this attribute.
*
* @return Success if the value of the attribute was set properly. Otherwise 
*         a specific error code is returned.
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_set_attrib
(
    sns_ddf_handle_t     dd_handle,
    sns_ddf_sensor_e     sensor,
    sns_ddf_attribute_e  attrib,
    void*                value
)
{
	//uint8_t* read_count = 0;
	sns_ddf_status_e status = SNS_DDF_SUCCESS;
	sns_dd_qmc7983_state_t* state = (sns_dd_qmc7983_state_t *) dd_handle;
	sns_ddf_powerstate_e power_state;

	QMC7983_DMSG_1(HIGH, "set_attrib,attrib = %d", attrib);

	/* Input sanity check */
	if (state == NULL) {
		return SNS_DDF_EINVALID_PARAM;
	}

	if ((sensor != SNS_DDF_SENSOR_MAG) && (sensor != SNS_DDF_SENSOR__ALL)) {
		return SNS_DDF_EINVALID_PARAM;
	}

	//Get the handle
	switch (attrib) {
		case SNS_DDF_ATTRIB_POWER_STATE:
		{
			power_state = *(sns_ddf_powerstate_e *) value;

			if (power_state == SNS_DDF_POWERSTATE_ACTIVE)
			{
				/* Activate sensor power */
				sns_ddf_write_port_byte(state->port_handle, 0x21, 0x01, &status);
				sns_ddf_write_port_byte(state->port_handle, 0x20, 0x40, &status);
				sns_ddf_write_port_byte(state->port_handle, 0x0b, 0x01, &status);
				if( state->device_select == QMC6983_E1 || 
					state->device_select == QMC7983 || 
					state->device_select == QMC7983_SLOW_SETRESET)
				{
					sns_ddf_write_port_byte(state->port_handle, 0x29, 0x80, &status);

					sns_ddf_write_port_byte(state->port_handle, 0x0a, 0x0c, &status);

				}
				sns_ddf_write_port_byte(state->port_handle, 0x09, 0x1D, &status);
				QMC7983_DMSG_1(HIGH, "sns_dd_qmcx983_set_attrib, write status = %d active_mode", status);
				if (status != SNS_DDF_SUCCESS)
				{
					return SNS_DDF_EBUS;
				}
			}
			else
			{
				sns_ddf_write_port_byte(state->port_handle, 0x09, 0x1C, &status);
				QMC7983_DMSG_1(HIGH, "sns_dd_qmcx983_set_attrib, write status = %d lowpower_mode", status);
				if (status != SNS_DDF_SUCCESS)
					return SNS_DDF_EBUS;
			}
		}
		break;
		case SNS_DDF_ATTRIB_ODR:
		/* Set ODR */
		{
			sns_ddf_odr_t desired_odr = *(sns_ddf_odr_t *) value;
			QMC7983_DMSG_2(HIGH, "sns_dd_qmcx983_set_attrib, SNS_DDF_ATTRIB_ODR, state->odr = %d, desired_odr = %d", state->odr, desired_odr);
			state->odr = desired_odr;
		}
		break;
		case SNS_DDF_ATTRIB_FIFO:
		/* Set FIFO watermark */
		{
			QMC7983_DMSG_0(HIGH, "sns_dd_qmcx983_set_attrib, SNS_DDF_ATTRIB_FIFO");
			sns_ddf_fifo_set_s wm = *(sns_ddf_fifo_set_s *) value;
			state->water_mark = wm.watermark;
		}
		break;
		case SNS_DDF_ATTRIB_RANGE:
		/* Set range (typically used only for accel and gyro sensors) */
		{
			state->range = *(uint16_t *) value;
			QMC7983_DMSG_1(HIGH, "sns_dd_qmcx983_set_attrib, SNS_DDF_ATTRIB_RANGE, range = %d", state->range);
		}
		break;
		default:
			return SNS_DDF_EINVALID_ATTR;
	}

	return status;
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_get_attrib

===========================================================================*/
/**
* @brief Retrieves the value of an attribute for a sensor.
* 
* @param[in]  dd_handle   Handle to a driver instance.
* @param[in]  sensor      Sensor whose attribute is to be retrieved. When 
*                         addressing an attribute that refers to the driver
*                         this value is set to SNS_DDF_SENSOR__ALL.
* @param[in]  attrib      Attribute to be retrieved.
* @param[in]  memhandler  Memory handler used to dynamically allocate 
*                         output parameters, if applicable.
* @param[out] value       Pointer that this function will allocate or set 
*                         to the attribute's value.
* @param[out] num_elems   Number of elements in @a value.
*  
* @return Success if the attribute was retrieved and the buffer was 
*         populated. Otherwise a specific error code is returned.
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_get_attrib
(
    sns_ddf_handle_t     dd_handle,
    sns_ddf_sensor_e     sensor,
    sns_ddf_attribute_e  attrib,
    sns_ddf_memhandler_s* memhandler,
    void**                value,
    uint32_t*             num_elems
)
{
  	sns_dd_qmc7983_state_t *state = (sns_dd_qmc7983_state_t *)dd_handle;

  	QMC7983_DMSG_1(HIGH, "get_attrib, attrib = %d", attrib);

  	/* Input sanity check */
  	if (state == NULL)
  	{
	 	return SNS_DDF_EINVALID_PARAM;
  	}

	if (sensor != SNS_DDF_SENSOR_MAG)
	{
		return SNS_DDF_EINVALID_PARAM;
	}

  	switch(attrib)
  	{
  		case SNS_DDF_ATTRIB_POWER_INFO:
    	{
      		sns_ddf_power_info_s* power_attrib;
      		if ((*value = sns_ddf_memhandler_malloc_ex(
              	memhandler,
              	sizeof(sns_ddf_power_info_s), 
              	state->smgr_handle)) == NULL)
			{
			return SNS_DDF_ENOMEM;
			}
			power_attrib  = (sns_ddf_power_info_s *)(*value);
			*num_elems    = 1;

			/* Value is unit of uA */
			power_attrib->active_current = 250;
			power_attrib->lowpower_current = 3;
		}
	    break;

  		case SNS_DDF_ATTRIB_RANGE:
    	{
      		sns_ddf_range_s *device_ranges;
      		if ((*value = sns_ddf_memhandler_malloc_ex(
              	memhandler,
              	sizeof(sns_ddf_range_s), 
              	state->smgr_handle)) == NULL)
			{
				return SNS_DDF_ENOMEM;
			}

      		device_ranges  = (sns_ddf_range_s *)(*value);
      		*num_elems     = 1;
      		/* Value is unit of Gauss in Q16 format */
      		device_ranges->min  = FX_FLTTOFIX_Q16(-8.0);
      		device_ranges->max  = FX_FLTTOFIX_Q16(8.0);
    	}
    	break;
  		case SNS_DDF_ATTRIB_RESOLUTION_ADC:
		{
			sns_ddf_resolution_adc_s *device_res_adc;
			if ((*value = sns_ddf_memhandler_malloc_ex(
		      	memhandler,
		      	sizeof(sns_ddf_resolution_adc_s) , 
		      	state->smgr_handle)) == NULL)
			{
				return SNS_DDF_ENOMEM;
			}

			device_res_adc  = (sns_ddf_resolution_adc_s *)(*value);
			*num_elems      = 1;

			/* Value is integer */
			device_res_adc->bit_len  = 16;  // bits
			device_res_adc->max_freq = 100;  //Hz
		}
    	break;
  		case SNS_DDF_ATTRIB_RESOLUTION:
    	{
      		sns_ddf_resolution_t *device_res;
      		if ((*value = sns_ddf_memhandler_malloc_ex(
              	memhandler,
              	sizeof(sns_ddf_resolution_t) , 
             	state->smgr_handle)) == NULL)
      		{
        		return SNS_DDF_ENOMEM;
      		}
      		/* Value is in Q16 in sensor unit */
      		device_res  = (sns_ddf_resolution_t *)(*value);
      		*num_elems  = 1;
      		*device_res = FX_FLTTOFIX_Q16(0.1);
    	}
    	break;
  		case SNS_DDF_ATTRIB_DELAYS:
    	{
      		sns_ddf_delays_s *device_delay;
      		if ((*value = sns_ddf_memhandler_malloc_ex(
              	memhandler,
              	sizeof(sns_ddf_delays_s), 
              	state->smgr_handle)) == NULL)
      		{
        		return SNS_DDF_ENOMEM;
      		}	

      		device_delay  = (sns_ddf_delays_s *)(*value);
      		*num_elems    = 1;
      		/* Value is in int in usec */
      		device_delay->time_to_active  = 10;
      		device_delay->time_to_data    = 100;
    	}
    	break;
  		case SNS_DDF_ATTRIB_DRIVER_INFO:
    	{
      		sns_ddf_driver_info_s *driver_info;
      		if ((*value = sns_ddf_memhandler_malloc_ex(
              	memhandler,
              	sizeof(sns_ddf_driver_info_s), 
              	state->smgr_handle)) == NULL)
     	 	{
        		return SNS_DDF_ENOMEM;
      		}

      		driver_info  = (sns_ddf_driver_info_s *)(*value);
      		*num_elems   = 1;
      		driver_info->name = "QMX7983";
      		driver_info->version = 1;
    	}
    	break;
  		case SNS_DDF_ATTRIB_DEVICE_INFO:
		{
			sns_ddf_device_info_s *device_info;
			if ((*value = sns_ddf_memhandler_malloc_ex(
		      	memhandler,
		      	sizeof(sns_ddf_device_info_s), 
		      	state->smgr_handle)) == NULL)
			{
				return SNS_DDF_ENOMEM;
			}

			device_info  = (sns_ddf_device_info_s *)(*value);
			*num_elems   = 1;
			device_info->model = "QMCX983";
			device_info->vendor = "QST Corp";
			device_info->name = "QMCX983";
			device_info->version = 1;
		}
    	break;
  		case SNS_DDF_ATTRIB_ODR:
		{
			sns_ddf_odr_t *curr_odr;
			if((curr_odr = sns_ddf_memhandler_malloc_ex(
				memhandler,
				sizeof(sns_ddf_odr_t), 
				state->smgr_handle)) == NULL)
			{
				return SNS_DDF_ENOMEM;
			}
			*curr_odr = state->odr;
			*value = curr_odr;
		}
    	break;
  		case SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST:
		{
			uint32_t *odr_list;

			if((odr_list = (uint32_t *)sns_ddf_memhandler_malloc_ex(
				memhandler,
				4 * sizeof(uint32_t), 
				state->smgr_handle)) == NULL)
			{
				return SNS_DDF_ENOMEM;
			}
			odr_list[0] = 10;
			odr_list[1] = 20;
			odr_list[2] = 50;
			odr_list[3] = 100;
			*value = (uint32_t *)odr_list;
			*num_elems = 4;
		}
    	break;
  		case SNS_DDF_ATTRIB_FIFO:
		{
			sns_ddf_fifo_attrib_get_s *fifo_info;

			if ((fifo_info = (sns_ddf_fifo_attrib_get_s *)sns_ddf_memhandler_malloc_ex(
				memhandler,
				sizeof(sns_ddf_fifo_attrib_get_s) , 
				state->smgr_handle)) == NULL)
			{
				return SNS_DDF_ENOMEM;
			}

			fifo_info->is_supported = false;
			fifo_info->is_sw_watermark = false;
			fifo_info->max_watermark = 0; 
			fifo_info->share_sensor_cnt = 0;
			fifo_info->share_sensors[0] = NULL;

			*value = fifo_info;
		}
		break;
  
  		default:
    		return SNS_DDF_EINVALID_PARAM;
  	}
  	return SNS_DDF_SUCCESS;
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_run_test

===========================================================================*/
/**
* @brief Runs a factory test case.
*  
* @param[in]  dd_handle  Handle to a driver instance. 
* @param[in]  sensor     Sensor on which to run the test. 
* @param[in]  test       Test case to run. 
* @param[out] err        Optional driver-specific error code.
*  
* @return One of the following error codes:
*     SNS_DDF_SUCCESS        - Test passed.
*     SNS_DDF_PENDING        - Test result will be sent as an event.
*     SNS_DDF_EDEVICE_BUSY   - Device is busy streaming, cannot run test.
*     SNS_DDF_EINVALID_TEST  - Test is not defined for this sensor.
*     SNS_DDF_EINVALID_PARAM - One of the parameters is invalid.
*     SNS_DDF_EFAIL          - Unknown error occurred.
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_run_test
(
    sns_ddf_handle_t    dd_handle,
    sns_ddf_sensor_e    sensor_type,
    sns_ddf_test_e      test,
    uint32_t*           err
)
{
  sns_dd_qmc7983_state_t *state = (sns_dd_qmc7983_state_t *)dd_handle;
  QMC7983_DMSG_0(HIGH, "sns_dd_qmcx983_run_test");
  /* Sanity check input */
  if (state == NULL)
  {
     return SNS_DDF_EINVALID_PARAM;
  }

  if (sensor_type != SNS_DDF_SENSOR_MAG)
  {
     return SNS_DDF_EINVALID_PARAM;
  }

  switch (test)
  {
    case SNS_DDF_TEST_SELF:
      /* Perform comm test with sensor, read ID and verify sensor presence.
         Return failure if comm test fails */
      return SNS_DDF_SUCCESS;
    default:
      return SNS_DDF_EINVALID_PARAM;
  }
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_probe

===========================================================================*/
/**
* @brief Probes for the device with a given configuration.

* @param[in]  dev_info    Access info for physical devices controlled by 
*                         this driver. Used to determine if the device is
*                         physically present.
* @param[in]  memhandler  Memory handler used to dynamically allocate 
*                         output parameters, if applicable.
* @param[out] num_sensors Number of sensors supported. 0 if none.
* @param[out] sensor_type Array of sensor types supported, with num_sensor
*                         elements. Allocated by this function.
*
* @return SNS_DDF_SUCCESS if the part was probed function completed, even
*         if no device was found (in which case num_sensors will be set to
*         0).
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_probe
(
    sns_ddf_device_access_s* device_info,
    sns_ddf_memhandler_s*    memhandler,
    uint32_t*                num_sensors,
    sns_ddf_sensor_e**       sensors
)
{
  	sns_ddf_status_e status;
  	sns_ddf_handle_t port_handle;

  	*num_sensors = 0;
  	*sensors = NULL;

  	if ((status = sns_ddf_open_port( 
			&port_handle, 
			&(device_info->port_config))) != SNS_DDF_SUCCESS)
  	{
	  	QMC7983_DMSG_1(HIGH, "Port open failed. status = %d.", status);
    	return status;
  	}

	*num_sensors = QMCX983_SUPPORTED_SENSORS_NUM;
  	*sensors = sns_ddf_memhandler_malloc(memhandler,
      	sizeof(sns_ddf_sensor_e) * (*num_sensors));
	if (NULL != *sensors) 
	{
		(*sensors)[0] = SNS_DDF_SENSOR_MAG;
		status = SNS_DDF_SUCCESS;
	}
	else
  	{
    	status = SNS_DDF_ENOMEM;
  	}
	
	sns_ddf_close_port(port_handle);
  	return status;
}

/*===========================================================================

FUNCTION:   sns_dd_qmcx983_enable_sched_data

===========================================================================*/
/**
* @brief Begins device-scheduled sampling and enables notification via Data 
*        Ready Interrupts (DRI).
*  
* @param[in] handle  Handle to the driver's instance.
* @param[in] sensor  Sensor to be sampled.
* @param[in] enable  True to enable or false to disable data stream.
* 
* @return SNS_DDF_SUCCESS if sensor was successfully configured and 
*         internal sampling has commenced or ceased. Otherwise an
*         appropriate error code.
*/
/*=========================================================================*/
sns_ddf_status_e sns_dd_qmcx983_enable_sched_data
    (
    sns_ddf_handle_t  handle,
    sns_ddf_sensor_e  sensor,
    bool              enable
    )
{
	sns_ddf_status_e status = SNS_DDF_SUCCESS;
	sns_dd_qmc7983_state_t *state = (sns_dd_qmc7983_state_t *)handle;
	
	QMC7983_DMSG_0(HIGH, "enable_sched_data");
	/* Input sanity check */
	if (state == NULL)
	{
		return SNS_DDF_EINVALID_PARAM;
	}

	/* check parameter - sensor */
	if (sensor != SNS_DDF_SENSOR_MAG)
	{
		return SNS_DDF_EINVALID_PARAM;
	}

	/* Check if the platform supports interrupts. */
	if( !sns_ddf_signal_irq_enabled() || (state->gpio1 == 0))
	{
		return SNS_DDF_EFAIL;
	}

  	if(enable)
	{
    	/* Enables DRI */
		status = sns_ddf_signal_register(
			state->gpio1,
			state,
			&SNS_DD_QMCX983_IF,
			SNS_DDF_SIGNAL_IRQ_RISING);
  	}
  	else
  	{
		/* Disables DRI */
		sns_ddf_signal_deregister(state->gpio1);
  	}
  	return status;
}


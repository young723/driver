/*  Date: 2017/06/01
 *  Revision: 1.0.0
 */


/*******************************************************************************
 * Copyright (c) 2013, QST Corp.
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


/*==============================================================================

  S E N S O R S   A C C E L E R O M E T E R    D R I V E R

  DESCRIPTION

  Impelements the driver for the accelerometer driver

  EDIT HISTORY FOR FILE


  version when   who        what
  ----------   ---     -----------------------------------------------------------
  17/06/01       FDP    basic values, bandwidth, g range, power mode, motion detect interrupt
  19/05/22       yzq    add qma7981

==============================================================================*/

/*============================================================================

                      INCLUDE FILES

============================================================================*/
#include "sns_dd_qmax981.h"

extern sns_ddf_driver_if_s sns_dd_qmaX981_if;
//static unsigned char sns_dd_int_init_flag = 0;

static sns_ddf_sensor_e sns_dd_qmaX981_sensor_types[] =
{
	SNS_DDF_SENSOR_ACCEL
#if 0
	,SNS_DDF_SENSOR_STEP_COUNT
#endif
};


static const qmaX981_odr_map_t qma6981_odr_reg_tbl[] =
{
    {QMAX981_ACCEL_BW_VALUE_8HZ, QMA6981_ODR_7_8HZ},
    {QMAX981_ACCEL_BW_VALUE_16HZ, QMA6981_ODR_15_6HZ},
    {QMAX981_ACCEL_BW_VALUE_32HZ, QMA6981_ODR_31_2HZ},
    {QMAX981_ACCEL_BW_VALUE_64HZ, QMA6981_ODR_62_5HZ},
    {QMAX981_ACCEL_BW_VALUE_128HZ, QMA6981_ODR_125HZ},
    {QMAX981_ACCEL_BW_VALUE_256HZ, QMA6981_ODR_250HZ},
    {QMAX981_ACCEL_BW_VALUE_512HZ, QMA6981_ODR_500HZ},
	
    {QMAX981_ACCEL_BW_VALUE_MAX, QMA6981_ODR_500HZ}
};

static const qmaX981_odr_map_t qma7981_odr_reg_tbl[] =
{
    {QMAX981_ACCEL_BW_VALUE_8HZ, QMA7981_ODR_8HZ},
    {QMAX981_ACCEL_BW_VALUE_16HZ, QMA7981_ODR_16HZ},
    {QMAX981_ACCEL_BW_VALUE_32HZ, QMA7981_ODR_32HZ},
    {QMAX981_ACCEL_BW_VALUE_64HZ, QMA7981_ODR_64HZ},
    {QMAX981_ACCEL_BW_VALUE_128HZ, QMA7981_ODR_128HZ},
    {QMAX981_ACCEL_BW_VALUE_256HZ, QMA7981_ODR_256HZ},
    {QMAX981_ACCEL_BW_VALUE_512HZ, QMA7981_ODR_512HZ},
    
    {QMAX981_ACCEL_BW_VALUE_MAX, QMA7981_ODR_512HZ}
};

static const uint32_t qmaX981_accel_bw[] =
{
	QMAX981_BW_8HZ,
	QMAX981_BW_16HZ,
	QMAX981_BW_32HZ,
	QMAX981_BW_64HZ,
	QMAX981_BW_128HZ,
	QMAX981_BW_256HZ,
	QMAX981_BW_512HZ
};

static sns_ddf_lowpass_freq_t qmaX981_accel_freq[] =
{
	FX_FLTTOFIX_Q16(QMAX981_ACCEL_BW_VALUE_8HZ),
	FX_FLTTOFIX_Q16(QMAX981_ACCEL_BW_VALUE_16HZ),
	FX_FLTTOFIX_Q16(QMAX981_ACCEL_BW_VALUE_32HZ),
	FX_FLTTOFIX_Q16(QMAX981_ACCEL_BW_VALUE_64HZ),
	FX_FLTTOFIX_Q16(QMAX981_ACCEL_BW_VALUE_128HZ),
	FX_FLTTOFIX_Q16(QMAX981_ACCEL_BW_VALUE_256HZ),
	FX_FLTTOFIX_Q16(QMAX981_ACCEL_BW_VALUE_512HZ)
};

sns_ddf_status_e qmaX981_read_bytes(sns_ddf_handle_t port_handle, uint8_t reg_addr, uint8_t*data, uint8_t len)
{
    sns_ddf_status_e status;
    uint8_t out;

    if((status = sns_ddf_read_port(port_handle,reg_addr,data,len,&out)) != SNS_DDF_SUCCESS)
    {
		QMAX981_MSG_2(HIGH, "qmaX981_read_bytes addr 0x%x error status=%d !!!", reg_addr, status);    	
        return status;
    }
    return status;
}

sns_ddf_status_e qmaX981_read_byte(sns_ddf_handle_t port_handle, uint8_t reg_addr, uint8_t*data)
{
    sns_ddf_status_e status;
    uint8_t out;

    status = sns_ddf_read_port(port_handle,reg_addr,data,1,&out);
    if(status != SNS_DDF_SUCCESS)
    {
		QMAX981_MSG_2(HIGH, "qmaX981_read_byte addr 0x%x error status=%d !!!", reg_addr, status);    	
        return status;
    }

    return status;
}

sns_ddf_status_e qmaX981_write_byte(sns_ddf_handle_t port_handle, uint8_t reg_addr, uint8_t*data)
{
    sns_ddf_status_e status;
    uint8_t out;

    status = sns_ddf_write_port(port_handle,reg_addr,data,1,&out);	
    if(status != SNS_DDF_SUCCESS)
    {
		QMAX981_MSG_2(HIGH, "qmaX981_write_byte addr 0x%x error status=%d !!!", reg_addr, status);    	
    }

    return status;
}


sns_ddf_status_e qmaX981_set_mode(sns_dd_qmaX981_state_t *state, sns_ddf_powerstate_e mode)
{
	sns_ddf_status_e status = SNS_DDF_SUCCESS;
	uint8_t reg_data;

	if(mode == SNS_DDF_POWERSTATE_LOWPOWER)
	{
		reg_data = 0x00;
		status = qmaX981_write_byte(state->port_handle, QMAX981_PWR_CTRL_REG, &reg_data);
	}
	else if(mode == SNS_DDF_POWERSTATE_ACTIVE)
	{
		reg_data = 0x80;
		status = qmaX981_write_byte(state->port_handle, QMAX981_PWR_CTRL_REG, &reg_data);
		if(state->qma_chip == QMA_7981)
		{
			reg_data = 0x80;
			status = qmaX981_write_byte(state->port_handle, 0x5f, &reg_data);
			reg_data = 0x00;
			status = qmaX981_write_byte(state->port_handle, 0x5f, &reg_data);
			sns_ddf_delay(20*1000);
		}
	}

	return status;
}


/*!
 *  @brief Initializes the 3 axis QST accelerometer QMAX981
 *              determines the device to use
 *
 *  @detail
 *  - Allocates memory for driver state structure.
 *  Opens the device port by calling sns_ddf_open_port routine
 *  Calls sns_dd_qmaX981_reset routine
 *
 *
 * @param[out] dd_handle_ptr  Pointer that this function must malloc and
 *                             populate. This is a handle to the driver
 *                             instance that will be passed in to all
 *                             other functions.
 *  @param[in]  smgr_handle    Handle used to identify this driver when it
 *                             calls into Sensors Manager functions.
 *  @param[in]  nv_params      NV parameters retrieved for the driver.
 *  @param[in]  device_info    Information describing each of the physical
 *                             devices controlled by this driver. This is
 *                             used to configure the bus and talk to the
 *                             device.
 *  @param[in]  memhandler     Memory handler used to dynamically allocate
 *                             output parameters, if applicable. NB: Do
 *                             not use memhandler to allocate memory for
 *                             @a dd_handle_ptr.
 * @param[in]  num_devices    Length of @a device_info.
 * @param[out] sensors        List of supported sensors, populated and
 returned by this function.
 *  @param[out] num_sensors    Length of @a sensors.
 *
 *
 *  @return
 *    The error code definition within the DDF
 *    SNS_DDF_SUCCESS on success; Otherwise SNS_DDF_EBUS
 *
 */
sns_ddf_status_e sns_dd_qmaX981_init(
  sns_ddf_handle_t* dd_handle_ptr,
  sns_ddf_handle_t smgr_handle,
  sns_ddf_nv_params_s* nv_params,
  sns_ddf_device_access_s device_info[],
  uint32_t num_devices,
  sns_ddf_memhandler_s* memhandler,
  sns_ddf_sensor_e* sensors[],
  uint32_t* num_sensors)
{
    sns_ddf_status_e status;
    sns_dd_qmaX981_state_t 	*state;
    uint8_t device_id;

    QMAX981_MSG_0(HIGH,"sns_dd_qmaX981_init, entry");
    /* Input sanity check */
    if(dd_handle_ptr == NULL || smgr_handle == NULL || num_sensors == NULL 
		||nv_params == NULL || memhandler == NULL || sensors == NULL)
    {
        QMAX981_MSG_0(ERROR, "sns_dd_qmaX981_init, Null Pointer argument found.");
        return SNS_DDF_EINVALID_PARAM;
    }

#ifdef QMAX981_ENABLE_UIMAGE_SUPPORT
    sns_ddf_smgr_set_uimg_refac(smgr_handle);
#endif

    /* Allocate a driver instance.*/
    status = sns_ddf_malloc_ex((void **)&state, sizeof(sns_dd_qmaX981_state_t),smgr_handle);
    if(status != SNS_DDF_SUCCESS)
    {
        status = SNS_DDF_ENOMEM;
        goto ErrorExit_0;
    }

    memset(state, 0, sizeof(sns_dd_qmaX981_state_t));
    *dd_handle_ptr = (sns_ddf_handle_t)state;
    // add for fifo
    status = sns_ddf_malloc_ex((void**)&(state->f_frames_cache.samples),
    sizeof(sns_ddf_sensor_sample_s)* QST_ACC_NUM_AXIS* QMA6981_MAX_FIFO_LEVEL, 
    smgr_handle);
    if(status != SNS_DDF_SUCCESS)
    {
        sns_ddf_mfree_ex((void *)state, smgr_handle);
        status = SNS_DDF_ENOMEM;
        goto ErrorExit_0;
    }
    // fifo
    status = sns_ddf_open_port(&(state->port_handle), &(device_info->port_config));
    if(status != SNS_DDF_SUCCESS)
    {
        goto ErrorExit_1;
    }
    state->smgr_hndl = smgr_handle;
    state->dev_info = device_info;
    state->gpio_num = device_info->first_gpio;
    QMAX981_MSG_1(HIGH, "state->gpio_num=%d", state->gpio_num);

    /* Init timer object */
	if((status = sns_ddf_timer_init(&(state->sns_dd_tmr_obj),*dd_handle_ptr,&sns_dd_qmaX981_if,NULL,0)) != SNS_DDF_SUCCESS)
	{
		QMAX981_MSG_0(ERROR, "sns_dd_qmaX981_init,Failed to initialize timer.");
		goto ErrorExit_2;
	}
    *num_sensors = 2;
    *sensors = sns_dd_qmaX981_sensor_types;
	sns_ddf_axes_map_init(&state->axes_map, ((nv_params != NULL) ? nv_params->data : NULL));

    /*Check sensor hardware*/
	status = qmaX981_read_byte(state->port_handle, QMAX981_CHIP_ID_REG, &device_id);
	if(status !=  SNS_DDF_SUCCESS)
	{
		QMAX981_MSG_0(ERROR, "sns_dd_qmaX981_init, read chip id fail !!!");
		goto ErrorExit_3;
	}

	state->qma_chip = QMA_START;
	if(device_id == 0xb0)
		state->qma_chip = QMA_6981;
	else if(device_id == 0xe7)
		state->qma_chip = QMA_7981;
	else if(device_id == 0xe8)
		state->qma_chip = QMA_6100;

	if(state->qma_chip == QMA_START)
    {
        QMAX981_MSG_1(ERROR, "qmaX981 wrong device_id 0x%x",device_id);
        status = SNS_DDF_EDEVICE;
        goto ErrorExit_3;
    }

    status = sns_dd_qmaX981_reset(state);
    if(status != SNS_DDF_SUCCESS)
    {
        status = SNS_DDF_EDEVICE;
        goto ErrorExit_3;
    }

    QMAX981_MSG_0(HIGH,"sns_dd_qmaX981_init, done");
    return SNS_DDF_SUCCESS;

ErrorExit_3:
    sns_ddf_timer_release(state->sns_dd_tmr_obj);
ErrorExit_2:
    sns_ddf_close_port(state->port_handle);
ErrorExit_1:
	sns_ddf_mfree_ex((void *)state->f_frames_cache.samples, smgr_handle);
    sns_ddf_mfree_ex((void *)state, smgr_handle);
ErrorExit_0:
    *dd_handle_ptr = NULL;
    *num_sensors = 0;

    return status;
}

static sns_ddf_status_e sns_dd_qmaX981_chip_init(sns_dd_qmaX981_state_t *state)
{
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    uint8_t reg_data = 0x00;

	if(state->qma_chip == QMA_6981)
	{
	    reg_data = QMAX981_RANGE_4G;
		status = qmaX981_write_byte(state->port_handle, QMAX981_RANGE_SEL_REG, &reg_data);
	    reg_data = QMA6981_ODR_62_5HZ;
		status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG, &reg_data);		
		state->acc_rate = 64;	//32
	    //sns_dd_qmaX981_set_odr(state, 32);
	    //databuf[0] = 0x31;
	    //reg_data = 32; // watermark level
	    //status = qmaX981_write_byte(state->port_handle, databuf[0],&reg_data);
		//databuf[0] = 0x3e;
		//reg_data = 0x40;		// fifo mode
		//status = qmaX981_write_byte(state->port_handle, databuf[0],&reg_data);
		//databuf[0] = 0x17;
		//reg_data = 0x20;		// 0x20:fifo full enable, 0x40:fifo watermark enable
		//status = qmaX981_write_byte(state->port_handle, databuf[0],&reg_data);
		//databuf[0] = 0x1a;
		//reg_data = 0x20;		// 0x20:fifo full enable, 0x40:fifo watermark enable
		//status = qmaX981_write_byte(state->port_handle, databuf[0],&reg_data);
	}
	else if(state->qma_chip == QMA_7981)
	{
		reg_data = QMAX981_RANGE_4G;
		status = qmaX981_write_byte(state->port_handle, QMAX981_RANGE_SEL_REG, &reg_data);
		reg_data = QMA7981_ODR_64HZ;		// 130 hz
		status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG, &reg_data);
		state->acc_rate = 64;	//32
	}
	status = qmaX981_set_mode(state, SNS_DDF_POWERSTATE_ACTIVE);

    return status;
}


/*===========================================================================
FUNCTION      sns_dd_qmaX981_reset

DESCRIPTION  Resets the driver and device so they return to the state
                    they were in after init() was called.
============================================================================*/
sns_ddf_status_e sns_dd_qmaX981_reset(sns_ddf_handle_t dd_handle)
{
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
	sns_dd_qmaX981_state_t *state = (sns_dd_qmaX981_state_t *)dd_handle;
    uint8_t i;

    QMAX981_MSG_0(HIGH, "sns_dd_qmaX981_reset, entry");
	if(state == NULL)
	{
		QMAX981_MSG_0(HIGH, "sns_dd_qmaX981_reset, state == NULL");
		return SNS_DDF_EINVALID_PARAM;
	}

    sns_ddf_timer_cancel(state->sns_dd_tmr_obj);
    for(i = 0; i < QST_ACC_NUM_AXIS; i++)
    {
        state->bias[i] = 0;
        state->data_cache[i] = 0;
    }
    //state->range = QMAX981_RANGE_4G;
    state->sens = QMAX981_SENS_4G;
    //state->data_rate = QMAX981_REGV_BW_125HZ;
    state->power_state = SNS_DDF_POWERSTATE_ACTIVE;
    state->acc_rate = 0;
    status = sns_dd_qmaX981_chip_init(state);

    return status;
}

sns_ddf_status_e sns_dd_qmaX981_set_odr(sns_dd_qmaX981_state_t* state, sns_ddf_odr_t odr)
{
    uint32_t i, tbl_size;
    uint8_t reg_data = 0x00;
    sns_ddf_status_e status = SNS_DDF_SUCCESS;

	if(state->qma_chip == QMA_6981)
	{
	    tbl_size = sizeof(qma6981_odr_reg_tbl)/sizeof(qma6981_odr_reg_tbl[0]);
	    for(i=0; i<tbl_size; i++)
	    {
	        if(odr <= qma6981_odr_reg_tbl[i].rate)
	        break;
	    }
	    if(i < tbl_size)
	    {
	        reg_data = qma6981_odr_reg_tbl[i].reg;
			QMAX981_MSG_2(HIGH, "qmaX981_set_odr 0x%x=0x%x", QMAX981_BW_SEL_REG, reg_data);
	        status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG, &reg_data);
			if(status == SNS_DDF_SUCCESS)
	        	state->acc_rate = qma6981_odr_reg_tbl[i].rate;
	    }
	}
	else if(state->qma_chip == QMA_7981)
	{
		tbl_size = sizeof(qma7981_odr_reg_tbl)/sizeof(qma7981_odr_reg_tbl[0]);
	    for(i=0; i<tbl_size; i++)
	    {
	        if(odr <= qma7981_odr_reg_tbl[i].rate)
	        break;
	    }
	    if(i < tbl_size)
	    {
	        reg_data = qma7981_odr_reg_tbl[i].reg;
			QMAX981_MSG_2(HIGH, "qmaX981_set_odr 0x%x=0x%x", QMAX981_BW_SEL_REG, reg_data);
	        status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG, &reg_data);
			if(status == SNS_DDF_SUCCESS)
	        	state->acc_rate = qma7981_odr_reg_tbl[i].rate;
	    }
	}

    return status;
}


/*!
 *  @brief Sets an attribute of the QST accelerometer
 *
 * @detail
 *  Called by SMGR to set certain device attributes that are
 *  programmable. Curently its the power mode and range.
 *
 *  @param[in] dd_handle   Handle to a driver instance.
 *  @param[in] sensor Sensor for which this attribute is to be set.
 *  @param[in] attrib      Attribute to be set.
 *  @param[in] value      Value to set this attribute.
 *
 *  @return
 *    The error code definition within the DDF
 *    SNS_DDF_SUCCESS on success; Otherwise SNS_DDF_EBUS or
 *    SNS_DDF_EINVALID_PARAM
 *
 */
sns_ddf_status_e sns_dd_qmaX981_set_attr(sns_ddf_handle_t dd_handle,sns_ddf_sensor_e sensor,
													sns_ddf_attribute_e attrib,void* value)
{
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    sns_dd_qmaX981_state_t* state = (sns_dd_qmaX981_state_t *)dd_handle;
    uint8_t reg_data = 0x00;

    if((sensor != SNS_DDF_SENSOR_ACCEL) && (sensor != SNS_DDF_SENSOR__ALL))
        return SNS_DDF_EINVALID_PARAM;
	
    QMAX981_MSG_2(HIGH, "sns_dd_qmaX981_set_attr ,sensor,%d,attr,%d",sensor, attrib);
    switch(attrib)
    {
        case SNS_DDF_ATTRIB_POWER_STATE:
        {
            sns_ddf_powerstate_e* power_state = value;
			
			QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_POWER_STATE power=%d", *power_state);
			return SNS_DDF_SUCCESS;
            if((SNS_DDF_POWERSTATE_ACTIVE == *power_state)&&(SNS_DDF_POWERSTATE_LOWPOWER == state->power_state))
            {            
				status = qmaX981_set_mode(state, SNS_DDF_POWERSTATE_ACTIVE);
                if(status != SNS_DDF_SUCCESS)
                {
                	QMAX981_MSG_0(ERROR, "SNS_DDF_POWERSTATE_ACTIVE error!");
                    return SNS_DDF_EBUS;
                }
                state->power_state = SNS_DDF_POWERSTATE_ACTIVE;
            }
            else if((SNS_DDF_POWERSTATE_LOWPOWER == *power_state)&&(SNS_DDF_POWERSTATE_ACTIVE == state->power_state))
            {
				qmaX981_set_mode(state, SNS_DDF_POWERSTATE_LOWPOWER);
                if(status != SNS_DDF_SUCCESS)
                {
					QMAX981_MSG_0(ERROR, "SNS_DDF_POWERSTATE_LOWPOWER error!");
                    return SNS_DDF_EBUS;
                }
                state->power_state = SNS_DDF_POWERSTATE_LOWPOWER;
            }
            return SNS_DDF_SUCCESS;
        }
        //break;

        case SNS_DDF_ATTRIB_RANGE:
        {
            uint32_t* range_idx = value;
			
			if(state->qma_chip == QMA_6981)
			{
	            if(*range_idx == 0)
	            {
	                reg_data = (uint8_t)QMAX981_RANGE_2G;
	                state->sens = QMAX981_SENS_2G;
	            }
	            else if(*range_idx == 1)
	            {
	                reg_data = (uint8_t)QMAX981_RANGE_4G;
	                state->sens = QMAX981_SENS_4G;
	            }
	            else if(*range_idx == 2)
	            {
	                reg_data = (uint8_t)QMAX981_RANGE_8G;
	                state->sens = QMAX981_SENS_8G;
	            }
	            else
	            {
	                reg_data = (uint8_t)QMAX981_RANGE_2G;
	                state->sens = QMAX981_SENS_2G;
	            }
			}			
			else if(state->qma_chip == QMA_7981)
			{
				if(*range_idx == 0)
				{
					reg_data = (uint8_t)QMAX981_RANGE_2G;
					state->sens = QMAX981_SENS_2G;
				}
				else if(*range_idx == 1)
				{
					reg_data = (uint8_t)QMAX981_RANGE_4G;
					state->sens = QMAX981_SENS_4G;
				}
				else if(*range_idx == 2)
				{
					reg_data = (uint8_t)QMAX981_RANGE_8G;
					state->sens = QMAX981_SENS_8G;
				}				
				else if(*range_idx == 3)
				{
					reg_data = (uint8_t)QMAX981_RANGE_16G;
					state->sens = QMAX981_SENS_16G;
				}				
				else if(*range_idx == 4)
				{
					reg_data = (uint8_t)QMAX981_RANGE_32G;
					state->sens = QMAX981_SENS_32G;
				}
				else
				{
					reg_data = (uint8_t)QMAX981_RANGE_2G;
					state->sens = QMAX981_SENS_2G;
				}
			}

			//reg_data = (uint8_t)state->range;
            status = qmaX981_write_byte(state->port_handle, QMAX981_RANGE_SEL_REG, &reg_data);		
            QMAX981_MSG_2(HIGH, "SNS_DDF_ATTRIB_RANGE, range_idx = %d status = %d",*range_idx, status);
            return SNS_DDF_SUCCESS;
           // break;
        }
		case SNS_DDF_ATTRIB_LOWPASS:
		{
			uint8_t bw_index = *((uint8_t *)value);
			if(bw_index > QMAX981_BW_MAX)
			{
				QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RESOLUTION ERROR");
				return SNS_DDF_EINVALID_PARAM;
			}
			sns_dd_qmaX981_set_odr(state, qma7981_odr_reg_tbl[bw_index].rate);
            return SNS_DDF_SUCCESS;
		}
        case SNS_DDF_ATTRIB_ODR:
        {
            sns_ddf_odr_t odr = *(sns_ddf_odr_t *) value;

            state->acc_rate = odr;
            status = sns_dd_qmaX981_set_odr(state, odr);
            QMAX981_MSG_2(HIGH, "SNS_DDF_ATTRIB_ODR, odr = %d, status=%d", odr, status);
#if defined(QMA6981_TAP_SUPPORT)
			if(sns_dd_int_init_flag == 0)
			{
	            status = sns_ddf_signal_register(state->gpio_num,dd_handle,&sns_dd_qmaX981_if,SNS_DDF_SIGNAL_IRQ_RISING);
	            reg_data = 0x80;
	            status = qmaX981_write_byte(state->port_handle, 0x2a, &reg_data);
	            reg_data = 0x01;
	            status = qmaX981_write_byte(state->port_handle, 0x2b, &reg_data);
	            reg_data = 0x20;
	            status = qmaX981_write_byte(state->port_handle, 0x16, &reg_data);
	            reg_data = 0x20;
	            status = qmaX981_write_byte(state->port_handle, 0x19, &reg_data);
				// enable fifo set high ODR!
	            reg_data = QMAX981_REGV_BW_62_5HZ;
	            status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG, &reg_data);
				// disable fifo
				reg_data = 0x40;	   // fifo mode
				status = qmaX981_write_byte(state->port_handle, 0x3e, &reg_data);
				reg_data = 0x00;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	            status = qmaX981_write_byte(state->port_handle, 0x17, &reg_data);
	            reg_data = 0x00;    // disable fifo interrupt
	            status = qmaX981_write_byte(state->port_handle, 0x1a, &reg_data);
				sns_dd_int_init_flag = 1;
			}
#endif
            return SNS_DDF_SUCCESS;
            //break;
        }
        case SNS_DDF_ATTRIB_MOTION_DETECT:
			if(!sns_ddf_signal_irq_enabled())
			{                
				QMAX981_MSG_0(ERROR, "SNS_DDF_ATTRIB_MOTION_DETECT return");
				return SNS_DDF_EINVALID_PARAM;
			}
			QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_MOTION_DETECT = %d", *((bool *)value));
            state->am_irq_flag = *((bool *)value);
			if(state->qma_chip == QMA_7981)
			{
				if(state->am_irq_flag)
				{
					reg_data = 0x07;
					qmaX981_write_byte(state->port_handle, 0x18, &reg_data);
					reg_data = 0x01;
					qmaX981_write_byte(state->port_handle, 0x1a, &reg_data);
					reg_data = 0x00;
					qmaX981_write_byte(state->port_handle, 0x2c, &reg_data);
					reg_data = 0x40;
					qmaX981_write_byte(state->port_handle, 0x2e, &reg_data);
					//qmaX981_write_byte(state->port_handle,0x2e, 0x18);		// 0.488*16*20 = 156mg
					//qmaX981_write_byte(state->port_handle,0x2e, 0xc0);		// 0.488*16*196= 1.5g
					//qmaX981_write_byte(state->port_handle,0x2e, 0x80);		// 0.488*16*128 = 1g
					//qmaX981_write_byte(state->port_handle,0x2e, 0x60);		// 0.488*16*96 = 750mg
					//qmaX981_write_byte(state->port_handle,0x2e, 0x40);		// 0.488*16*64 = 500mg
					//qmaX981_write_byte(state->port_handle,0x2e, 0x20);		// 0.488*16*32 = 250mg
					sns_ddf_signal_register(state->gpio_num,dd_handle,&sns_dd_qmaX981_if,SNS_DDF_SIGNAL_IRQ_RISING);
				}
				else
				{					
					reg_data = 0x00;
					qmaX981_write_byte(state->port_handle, 0x18, &reg_data);
					reg_data = 0x00;
					qmaX981_write_byte(state->port_handle, 0x1a, &reg_data);
				}
			}
			else if(state->qma_chip == QMA_6981)
			{
#if defined(QMA6981_TAP_SUPPORT)
	            if(state->am_irq_flag)
	            {
	                reg_data = 0x80;
	                status = qmaX981_write_byte(state->port_handle, 0x2a, &reg_data);
	                reg_data = 0x01;
	                status = qmaX981_write_byte(state->port_handle, 0x2b, &reg_data);
	                reg_data = 0x20;
	                status = qmaX981_write_byte(state->port_handle, 0x16, &reg_data);
	                reg_data = 0x20;
	                status = qmaX981_write_byte(state->port_handle, 0x19, &reg_data);
					// enable fifo set high ODR!
	                reg_data = QMAX981_REGV_BW_62_5HZ;
	                status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG, &reg_data);
					// disable fifo
					reg_data = 0x40;	   // fifo mode
					status = qmaX981_write_byte(state->port_handle, 0x3e, &reg_data);
					reg_data = 0x00;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	                status = qmaX981_write_byte(state->port_handle, 0x17, &reg_data);
	                reg_data = 0x00;    // disable fifo interrupt
	                status = qmaX981_write_byte(state->port_handle, 0x1a, &reg_data);
					// disable fifo
	            }
	            else
	            {
					// set low odr
	                reg_data = QMAX981_REGV_BW_7_8HZ;
	                status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG, &reg_data);
	                reg_data = 0x00;
	                status = qmaX981_write_byte(state->port_handle, 0x19, &reg_data);
	            }
#endif
			}
            return SNS_DDF_SUCCESS;

        case SNS_DDF_ATTRIB_FIFO:
        {
            uint8_t fifo_level = *((uint8_t *)value);
            QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_FIFO, fifo_level = %d", fifo_level);
			if(state->qma_chip == QMA_6981)
			{
	            if(fifo_level >= QMA6981_MAX_FIFO_LEVEL)
	            {
	                return SNS_DDF_EINVALID_PARAM;
	            }
	            else
	            {
	                if(fifo_level == 0)
	                	fifo_level = 1;

					if(fifo_level<5)
						fifo_level = 5;
	                state->fifo_level = fifo_level;

	                //reg_data = QMAX981_REGV_BW_15_6HZ;		// 62.5hz
	                //status = qmaX981_write_byte(state->port_handle, QMAX981_BW_SEL_REG,  &reg_data);
	                reg_data = state->fifo_level; // watermark level
	                status = qmaX981_write_byte(state->port_handle, 0x31, &reg_data);
				}
				
				return SNS_DDF_SUCCESS;
			}
			else
			{
				return SNS_DDF_EINVALID_PARAM;
			}
        }

        default:
            return SNS_DDF_EINVALID_PARAM;
    }
    return status;
}


/*===========================================================================
FUNCTION      sns_dd_qmaX981_get_attr

DESCRIPTION  Retrieves the value of an attribute for QMA6981.
============================================================================*/
sns_ddf_status_e sns_dd_qmaX981_get_attr(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_attribute_e attrib,
    sns_ddf_memhandler_s* memhandler,
    void** value,
    uint32_t* num_elems)
{
    sns_dd_qmaX981_state_t* state = (sns_dd_qmaX981_state_t *)dd_handle;

    if(sensor != SNS_DDF_SENSOR_ACCEL)
    {
        return SNS_DDF_EINVALID_PARAM;
    }
    QMAX981_MSG_2(HIGH, "sns_dd_qmaX981_get_attr ,sensor,%d,attr,%d",sensor, attrib);

    switch(attrib)
    {
        case SNS_DDF_ATTRIB_POWER_INFO:
        {
            sns_ddf_power_info_s* power_attr = sns_ddf_memhandler_malloc_ex(
                memhandler, sizeof(sns_ddf_power_info_s),state->smgr_hndl);
            if(NULL == power_attr)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_POWER_INFO fail");
                return SNS_DDF_ENOMEM;
            }
            power_attr->active_current = 220;
            power_attr->lowpower_current = 2;
            *value = power_attr;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_POWER_INFO OK");
            return SNS_DDF_SUCCESS;
            //break;
        }
        case SNS_DDF_ATTRIB_RANGE:
        {
			sns_ddf_range_s* ranges;

			if(state->qma_chip == QMA_6981)
				*num_elems = 3;
			else if((state->qma_chip == QMA_7981)||(state->qma_chip == QMA_6100))
				*num_elems = 4;

			ranges = sns_ddf_memhandler_malloc_ex(memhandler, (*num_elems)*sizeof(sns_ddf_range_s), state->smgr_hndl);
            if(NULL == ranges)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RANGE fail");
                return SNS_DDF_ENOMEM;
            }

			if(state->qma_chip == QMA_6981)
			{
				ranges[0].min = QMAX981_ACCEL_SENSOR_RANGE_2G_MIN;
				ranges[0].max = QMAX981_ACCEL_SENSOR_RANGE_2G_MAX;
				ranges[1].min = QMAX981_ACCEL_SENSOR_RANGE_4G_MIN;
				ranges[1].max = QMAX981_ACCEL_SENSOR_RANGE_4G_MAX;
				ranges[2].min = QMAX981_ACCEL_SENSOR_RANGE_8G_MIN;
				ranges[2].max = QMAX981_ACCEL_SENSOR_RANGE_8G_MAX;
			}
			else if((state->qma_chip == QMA_7981)||(state->qma_chip == QMA_6100))
			{
				ranges[0].min = QMAX981_ACCEL_SENSOR_RANGE_2G_MIN;
				ranges[0].max = QMAX981_ACCEL_SENSOR_RANGE_2G_MAX;
				ranges[1].min = QMAX981_ACCEL_SENSOR_RANGE_4G_MIN;
				ranges[1].max = QMAX981_ACCEL_SENSOR_RANGE_4G_MAX;
				ranges[2].min = QMAX981_ACCEL_SENSOR_RANGE_8G_MIN;
				ranges[2].max = QMAX981_ACCEL_SENSOR_RANGE_8G_MAX;
				ranges[3].min = QMAX981_ACCEL_SENSOR_RANGE_16G_MIN;
				ranges[3].max = QMAX981_ACCEL_SENSOR_RANGE_16G_MAX;
			}
            *value = ranges;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RANGE OK");
            return SNS_DDF_SUCCESS;
            //break;
        }
        case SNS_DDF_ATTRIB_RESOLUTION_ADC:
        {
            sns_ddf_resolution_adc_s *adc_res = sns_ddf_memhandler_malloc_ex(
                memhandler ,sizeof(sns_ddf_resolution_adc_s),state->smgr_hndl);
            if(NULL == adc_res)
            { 
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RESOLUTION_ADC fail");
                return SNS_DDF_ENOMEM;
            }

			if(state->qma_chip == QMA_6981)
			{
	            adc_res->bit_len = 10;
	            adc_res->max_freq = 500;
			}
			else if(state->qma_chip == QMA_7981)
			{
	            adc_res->bit_len = 14;
	            adc_res->max_freq = 500;
			}
            *value = adc_res;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RESOLUTION_ADC OK");
            return SNS_DDF_SUCCESS;
            //break;
        }
        case SNS_DDF_ATTRIB_RESOLUTION:
        {
            sns_ddf_resolution_t *res = sns_ddf_memhandler_malloc_ex(
                memhandler ,sizeof(sns_ddf_resolution_t),state->smgr_hndl);
            if(NULL == res)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RESOLUTION fail");
                return SNS_DDF_ENOMEM;
            }
			if(state->qma_chip == QMA_6981)
            	*res = FX_FLTTOFIX_Q16(2 * QMAX981_SENS_2G * G / 1024);
			else if(state->qma_chip == QMA_7981)
            	*res = FX_FLTTOFIX_Q16(2 * QMAX981_SENS_2G * G / 16384);
            *value = res;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RESOLUTION OK");
            return SNS_DDF_SUCCESS;
            //break;
        }
		case SNS_DDF_ATTRIB_LOWPASS:
		{
			QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_LOWPASS");
			*num_elems = 7;
			*value = qmaX981_accel_freq;
			return SNS_DDF_SUCCESS;
		}
        case SNS_DDF_ATTRIB_ODR:
        {
            sns_ddf_odr_t *odr = sns_ddf_memhandler_malloc_ex(
                memhandler ,sizeof(sns_ddf_odr_t),state->smgr_hndl);
            if(NULL == odr)
			{
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_ODR fail");
                return SNS_DDF_ENOMEM;
            }

            /*always return the current accel ODR to SMGR.*/
            if((SNS_DDF_SENSOR_ACCEL == sensor)||(SNS_DDF_SENSOR__ALL == sensor)){
                *odr = state->acc_rate;
            }
            else 
			{
                return SNS_DDF_EINVALID_PARAM;
			}

            *value = odr;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_ODR OK");
            return SNS_DDF_SUCCESS;
            //break;
        }
        case SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST:
        {
            uint32_t *odr_list;

			*num_elems = 7;
            if((odr_list = (uint32_t *)sns_ddf_memhandler_malloc_ex(
                memhandler,
                (*num_elems) * sizeof(uint32_t),
                state->smgr_hndl)) == NULL)  //dean modify smgr_hndl smgr_handle
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST fail");
                return SNS_DDF_ENOMEM;
            }
            odr_list[0] = 8;
            odr_list[1] = 16;
            odr_list[2] = 32;
            odr_list[3] = 64;
            odr_list[4] = 128; 
            odr_list[5] = 256; 
            odr_list[6] = 512; 
            *value = (uint32_t *)odr_list;
            
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST OK");
            return SNS_DDF_SUCCESS;
        }
    	//break;
// add for fifo
        case SNS_DDF_ATTRIB_FIFO:
		if(state->qma_chip == QMA_6981)
        {
            sns_ddf_fifo_attrib_get_s *fifo_attrib_info = sns_ddf_memhandler_malloc_ex(
            memhandler ,sizeof(sns_ddf_fifo_attrib_get_s), state->smgr_hndl);//dean modify smgr_hndl smgr_handle
            if (NULL == fifo_attrib_info)
                return SNS_DDF_ENOMEM;

            fifo_attrib_info->is_supported = true;
            fifo_attrib_info->is_sw_watermark = false;
            fifo_attrib_info->max_watermark = 32;
            fifo_attrib_info->share_sensor_cnt = 0;
            fifo_attrib_info->share_sensors[0] =  NULL;
            *value = fifo_attrib_info;
            *num_elems = 1;

            return SNS_DDF_SUCCESS;
        }
		else
		{
			return SNS_DDF_EINVALID_ATTR;
		}
// fifo
        case SNS_DDF_ATTRIB_DELAYS:
        {
            sns_ddf_delays_s *qmaX981_delays = sns_ddf_memhandler_malloc_ex(
                memhandler, sizeof(sns_ddf_delays_s),state->smgr_hndl);
            if(NULL == qmaX981_delays)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DELAYS fail");
                return SNS_DDF_ENOMEM;
            }

            qmaX981_delays->time_to_active = 5;
            qmaX981_delays->time_to_data = 1;

            *value = qmaX981_delays;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DELAYS OK");

            return SNS_DDF_SUCCESS;
        }

        case SNS_DDF_ATTRIB_DRIVER_INFO:
        {
            sns_ddf_driver_info_s *driver_info = sns_ddf_memhandler_malloc_ex(
                memhandler, sizeof(sns_ddf_driver_info_s),state->smgr_hndl);
            if(NULL == driver_info)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DRIVER_INFO fali");
                return SNS_DDF_ENOMEM;
            }
			if(state->qma_chip == QMA_6981)
            	driver_info->name = "QMA6981";
			else if(state->qma_chip == QMA_7981)
            	driver_info->name = "QMA7981";
            driver_info->version = 1;
            *value = driver_info;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DRIVER_INFO OK");
            return SNS_DDF_SUCCESS;
            //break;
        }

        case SNS_DDF_ATTRIB_DEVICE_INFO:
        {
            sns_ddf_device_info_s *device_info = sns_ddf_memhandler_malloc_ex(
                memhandler, sizeof(sns_ddf_device_info_s),state->smgr_hndl);
            if(NULL == device_info)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DEVICE_INFO fali");
                return SNS_DDF_ENOMEM;
            }

            device_info->name = "Accelerometer";
            device_info->vendor = "QST Corp";			
			if(state->qma_chip == QMA_6981)
	            device_info->model = "QMA6981";
			else if(state->qma_chip == QMA_7981)
	            device_info->model = "QMA7981";
            device_info->version = 1;
            *value = device_info;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DEVICE_INFO OK");
            return SNS_DDF_SUCCESS;
            //break;
        }
        default:
            return SNS_DDF_EINVALID_ATTR;
    }
    return SNS_DDF_SUCCESS;
}


/*===========================================================================
FUNCTION      sns_dd_qmaX981_self_test

DESCRIPTION  Runs self test and returns the test result.
============================================================================*/
sns_ddf_status_e sns_dd_qmaX981_self_test(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_test_e test,
    uint32_t* err)
{
    sns_dd_qmaX981_state_t* state = (sns_dd_qmaX981_state_t *)dd_handle;
    uint8_t rw_buffer = 0;
    sns_ddf_status_e status = SNS_DDF_SUCCESS;

    if(test == SNS_DDF_TEST_SELF)
    {
        status = qmaX981_read_byte(state->port_handle, QMAX981_CHIP_ID_REG, &rw_buffer);
        if(status != SNS_DDF_SUCCESS)
        	return SNS_DDF_EINVALID_TEST;
		else if(rw_buffer != 0xb0)			
			return SNS_DDF_EINVALID_TEST;
		else if(rw_buffer != 0xe7)
			return SNS_DDF_EINVALID_TEST;
    }
    else
    {
        return SNS_DDF_EINVALID_TEST;
    }

    return status;
}

sns_ddf_status_e sns_dd_qmaX981_probe(
    sns_ddf_device_access_s*  device_info,
    sns_ddf_memhandler_s*     memhandler,
    uint32_t*                 num_sensors,
    sns_ddf_sensor_e**        sensors )
{
    sns_ddf_status_e status;
    sns_ddf_handle_t port_handle;
    uint8_t rw_buffer = 0, rw_bytes = 0;

    /*Sanity check IN*/
    if((device_info == NULL) || (memhandler == NULL))
        return SNS_DDF_EINVALID_PARAM;

    status = SNS_DDF_SUCCESS;
    *num_sensors = 0;
    *sensors = NULL;

    QMAX981_MSG_0(HIGH,"sns_dd_qmaX981_probe, entry");

    /*Open COM port*/
    status = sns_ddf_open_port(&port_handle, &(device_info->port_config));
    if (status != SNS_DDF_SUCCESS)
    {
        return status;
    }

    /* Read and Verify Sensor ID */
    status = sns_ddf_read_port(port_handle,QMAX981_CHIP_ID_REG,&rw_buffer,1,&rw_bytes);
    if(status != SNS_DDF_SUCCESS || rw_bytes != 1)
    {
        sns_ddf_close_port(port_handle);
        return status;
    }
    QMAX981_MSG_1(HIGH,"sns_dd_qmaX981_probe CHIP_ID:0x%02x",rw_buffer);
    if(rw_buffer != QMA6981_CHIP_ID)
	{
        sns_ddf_close_port(port_handle);
        return status;
    }
    *num_sensors = 1;
    *sensors = sns_ddf_memhandler_malloc(memhandler, sizeof(sns_ddf_sensor_e)*(*num_sensors));

    if(*sensors == NULL)
    {
        status = SNS_DDF_ENOMEM;
        sns_ddf_close_port(port_handle);
        return status;
    }

    (*sensors)[0] = SNS_DDF_SENSOR_ACCEL;
    status = SNS_DDF_SUCCESS;
    QMAX981_MSG_0(HIGH,"sns_dd_qmaX981_probe ,done");
    sns_ddf_close_port( port_handle );

    return status;
}

#if 0
sns_ddf_status_e sns_dd_qmaX981_enable_sched_data(sns_ddf_handle_t dd_handle,sns_ddf_sensor_e sensor_type,bool enable)
{
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    sns_dd_qmaX981_state_t* state = (sns_dd_qmaX981_state_t *)dd_handle;
	uint8_t reg_data = 0x00;

    QMAX981_MSG_3(HIGH, "enable_sched_data fifo_level=%d, type=%d enable=%d",state->fifo_level, sensor_type, enable);

    if(!sns_ddf_signal_irq_enabled())
	{
        QMAX981_MSG_0(HIGH,"enable_sched_data return 0!!");
        return SNS_DDF_EINVALID_PARAM;
    }
    if(SNS_DDF_SENSOR_ACCEL != sensor_type)
	{
		QMAX981_MSG_0(HIGH,"enable_sched_data return 1!!");
        return SNS_DDF_SUCCESS;
    }

    if(enable)
    {
        if(state->fifo_int_config == 1)
        {
            if((status = sns_ddf_signal_deregister(state->gpio_num)) != SNS_DDF_SUCCESS) 
			{
                QMAX981_MSG_0(HIGH,"enable_sched_data sns_ddf_signal_deregister error!!");
                return status;
            }
            state->fifo_int_config = 0;
        }
        if(state->fifo_int_config != 1)
        {
            if((status = sns_ddf_signal_register(
                        state->gpio_num,
                        dd_handle,
                        &sns_dd_qmaX981_if,
                        SNS_DDF_SIGNAL_IRQ_RISING)) != SNS_DDF_SUCCESS) {
                        QMAX981_MSG_0(HIGH,"enable_sched_data sns_ddf_signal_register error!!");
                        return status;
            }
            state->fifo_int_config = 1;
        }
	    if(state->am_irq_flag)
	    {
	        return SNS_DDF_SUCCESS;
	    }
	    reg_data = 0x40;		// fifo mode
	    status = qmaX981_write_byte(state->port_handle, 0x3e, &reg_data);
	    reg_data = 0x40;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	    status = qmaX981_write_byte(state->port_handle, 0x17, &reg_data);
	    reg_data = 0x40;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	    status = qmaX981_write_byte(state->port_handle, 0x1a, &reg_data);
	    state->irq_ts_start = sns_ddf_get_timestamp();
    }
    else
    {
        if(state->fifo_int_config == 1)
        {
            if((status = sns_ddf_signal_deregister(state->gpio_num)) != SNS_DDF_SUCCESS) {
                QMAX981_MSG_0(HIGH,"enable_sched_data sns_ddf_signal_deregister error!!");
                return status;
            }
            state->fifo_int_config = 0;
        }
        if(state->fifo_int_config != 1)
        {
            if((status = sns_ddf_signal_register(state->gpio_num,dd_handle,&sns_dd_qmaX981_if,SNS_DDF_SIGNAL_IRQ_RISING)) != SNS_DDF_SUCCESS)
			{
                QMAX981_MSG_0(HIGH,"enable_sched_data sns_ddf_signal_register error!!");
                return status;
            }
            state->fifo_int_config = 1;
        }
	    reg_data = 0x00;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	    status = qmaX981_write_byte(state->port_handle, 0x17, &reg_data);
	    reg_data = 0x00;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	    status = qmaX981_write_byte(state->port_handle, 0x1a, &reg_data);
    }

    return status;
}
#endif

void sns_dd_qmaX981_handle_irq(sns_ddf_handle_t dd_handle, uint32_t gpio_num,sns_ddf_time_t timestamp)
{
    sns_ddf_status_e status;
	uint8_t rw_buffer = 0;
	uint8_t reg_data = 0;
	sns_dd_qmaX981_state_t* state = dd_handle;
	
    status = qmaX981_read_byte(state->port_handle, 0x09, &rw_buffer);
    if(status != SNS_DDF_SUCCESS)
    {
        return;
    }
	if(rw_buffer&0x0f)
	{
		// disable any motion
		reg_data = 0x00;
		qmaX981_write_byte(state->port_handle, 0x18, &reg_data);
		reg_data = 0x00;
		qmaX981_write_byte(state->port_handle, 0x1a, &reg_data);
	    
        status = sns_ddf_signal_deregister(gpio_num);
        if(status != SNS_DDF_SUCCESS)
            return;

         sns_ddf_smgr_notify_event(state->smgr_hndl,SNS_DDF_SENSOR_ACCEL,SNS_DDF_EVENT_MOTION);
	}
	else
	{
		//status = sns_dd_acc_mc3410_report_data(dd_handle, timestamp);
		//if(status != SNS_DDF_SUCCESS)
			return;
	}
}



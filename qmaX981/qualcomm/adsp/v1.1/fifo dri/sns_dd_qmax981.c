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

==============================================================================*/

/*============================================================================

                      INCLUDE FILES

============================================================================*/
#include "sns_dd_qmax981.h"

extern sns_ddf_driver_if_s sns_dd_qmax981_if;
sns_ddf_status_e sns_dd_qmax981_set_odr(sns_dd_qmax981_state_t* state, sns_ddf_odr_t odr);


static sns_ddf_sensor_e sns_dd_accel_sensor_types[] =
{
	SNS_DDF_SENSOR_ACCEL
};


const qma6981_odr_bw_map_t qma6981_odr_bw_tbl[] =
{
    {16, QMAX981_REGV_BW_7_8HZ},
    {32, QMAX981_REGV_BW_15_6HZ},
    {64, QMAX981_REGV_BW_31_2HZ},
    {125, QMAX981_REGV_BW_62_5HZ},
    {250, QMAX981_REGV_BW_125HZ},
    {500, QMAX981_REGV_BW_250HZ},
    {1000, QMAX981_REGV_BW_500HZ},
    {2000, QMAX981_REGV_BW_500HZ}
};


sns_ddf_status_e qmax981_read_bytes(
        sns_ddf_handle_t 	port_handle,
        uint8_t	 			reg_addr,
        uint8_t*			data,
        uint8_t 			len)
{
    sns_ddf_status_e status;
    uint8_t out;

    if ((status = sns_ddf_read_port(port_handle,
                    reg_addr,
                    data,
                    len,
                    &out)) != SNS_DDF_SUCCESS)
    {
        return status;
    }
    return status;
}

sns_ddf_status_e qmax981_read_byte(
        sns_ddf_handle_t 	port_handle,
        uint8_t 			reg_addr,
        uint8_t*			data)
{
    sns_ddf_status_e status;
    uint8_t out;

    status = sns_ddf_read_port(port_handle,
            reg_addr,
            data,
            1,
            &out);

    QMAX981_MSG_3(MED, "sbus_read@0x%x = 0x%x stat: %d",
            reg_addr, data[0], status);

    if(status != SNS_DDF_SUCCESS)
    {
        return status;
    }

    return status;
}

sns_ddf_status_e qmax981_write_byte(
        sns_ddf_handle_t 	port_handle,
        uint8_t 			reg_addr,
        uint8_t*			data)
{
    sns_ddf_status_e status;
    uint8_t out;

    status = sns_ddf_write_port(port_handle,
            reg_addr,
            data,
            1,
            &out);

    QMAX981_MSG_3(HIGH, "sbus_write@0x%x = 0x%x stat: %d",
            reg_addr, data[0], status);

    return status;
}

/*!
 *  @brief Initializes the 3 axis QST accelerometer QMAX981
 *              determines the device to use
 *
 *  @detail
 *  - Allocates memory for driver state structure.
 *  Opens the device port by calling sns_ddf_open_port routine
 *  Calls sns_dd_qmax981_reset routine
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
sns_ddf_status_e sns_dd_qmax981_init(
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
    sns_dd_qmax981_state_t 		*state;
    uint8_t device_id, read_byte;

    QMAX981_MSG_0(HIGH,"sns_dd_qmax981_init, entry");
    /* Input sanity check */
    if( dd_handle_ptr == NULL ||
        smgr_handle == NULL ||
        num_sensors == NULL ||
        nv_params == NULL ||
        memhandler == NULL ||
        sensors == NULL )
    {
        QMAX981_MSG_0(ERROR, "sns_dd_qmax981_init,Null Pointer argument found.");
        return SNS_DDF_EINVALID_PARAM;
    }

#ifdef QMAX981_ENABLE_UIMAGE_SUPPORT
    sns_ddf_smgr_set_uimg_refac(smgr_handle);
#endif

    /* Allocate a driver instance.*/
    status = sns_ddf_malloc_ex((void **)&state, sizeof(sns_dd_qmax981_state_t),smgr_handle);
    if(status != SNS_DDF_SUCCESS)
    {
        status = SNS_DDF_ENOMEM;
        goto ErrorExit_0;
    }

    memset(state, 0, sizeof(sns_dd_qmax981_state_t));
    *dd_handle_ptr = (sns_ddf_handle_t)state;
    // add for fifo
    status = sns_ddf_malloc_ex((void**)&(state->f_frames_cache.samples),
    sizeof(sns_ddf_sensor_sample_s)
    * QST_ACC_NUM_AXIS* QMA6981_MAX_FIFO_LEVEL, smgr_handle);
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
    if ((status = sns_ddf_timer_init(
        &(state->sns_dd_tmr_obj),
        *dd_handle_ptr,
        &sns_dd_qmax981_if,
        NULL,
        0)) != SNS_DDF_SUCCESS)
        {
            QMAX981_MSG_0(ERROR, "sns_dd_qmax981_init,Failed to initialize timer.");
            goto ErrorExit_2;
        }

    *num_sensors = 1;
    *sensors = sns_dd_accel_sensor_types;

    sns_ddf_axes_map_init(
        &state->axes_map, ((nv_params != NULL) ? nv_params->data : NULL));

    /*Check sensor hardware*/
    status = sns_ddf_read_port(
        (sns_ddf_handle_t)state->port_handle,
        QMAX981_CHIP_ID_REG,
        &device_id,
        1,
        &read_byte);

    if(status !=  SNS_DDF_SUCCESS){
        QMAX981_MSG_0(ERROR, "sns_dd_qmax981_init, read chpid fail");
        goto ErrorExit_3;
        }

    if(device_id != QMA6981_CHIP_ID){
        QMAX981_MSG_1(FATAL, "qmax981 wrong device_id 0x%x",device_id);
        status = SNS_DDF_EDEVICE;
        goto ErrorExit_3;
        }

    state->device_select = device_id;

    /* Resets the qma6981*/
    status = sns_dd_qmax981_reset(state);
    if(status != SNS_DDF_SUCCESS)
    {
        status = SNS_DDF_EDEVICE;
        goto ErrorExit_3;
    }

    QMAX981_MSG_0(HIGH,"sns_dd_qmax981_init, done");
    return SNS_DDF_SUCCESS;

ErrorExit_3:
    sns_ddf_timer_release(state->sns_dd_tmr_obj);
ErrorExit_2:
    sns_ddf_close_port(state->port_handle);
ErrorExit_1:
    sns_ddf_mfree_ex((void *)state, smgr_handle);
ErrorExit_0:
    *dd_handle_ptr = NULL;
    *num_sensors = 0;

    return status;
}

/*============================================================================
FUNCTION: sns_dd_qmax981_device_initialization
============================================================================*/
static sns_ddf_status_e sns_dd_qmax981_device_initialization(
    sns_ddf_handle_t dd_handle
)
{
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *)dd_handle;
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    uint8_t databuf[2] = {0};
    uint8_t out = 0;

    databuf[0] = QMAX981_RANGE_SEL_REG;
    databuf[1] = QMA6981_RANGE_4G;
    status = sns_ddf_write_port(state->port_handle,databuf[0],&databuf[1],1,&out);
    if(status != SNS_DDF_SUCCESS)
    {
        QMAX981_MSG_1(HIGH, "QMAX981_RANGE_SEL_REG write failed, status = %d", status);
        return status;
    }

    databuf[0] = QMAX981_BW_SEL_REG;
    databuf[1] = QMAX981_REGV_BW_62_5HZ;//QMAX981_REGV_BW_15_6HZ;    //QMAX981_REGV_BW_3_9HZ;
    status = sns_ddf_write_port(state->port_handle,databuf[0],&databuf[1],1,&out);
    if(status != SNS_DDF_SUCCESS)
    {
        QMAX981_MSG_1(HIGH, "QMAX981_BW_SEL_REG write failed, status = %d", status);
        return status;
    }
    state->acc_cur_rate = 0;	//32

    //sns_dd_qmax981_set_odr(state, 32);
    //databuf[0] = 0x31;
    //databuf[1] = 32; // watermark level
    //status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
	//databuf[0] = 0x3e;
	//databuf[1] = 0x40;		// fifo mode
	//status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
	//databuf[0] = 0x17;
	//databuf[1] = 0x20;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	//status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
	//databuf[0] = 0x1a;
	//databuf[1] = 0x20;		// 0x20:fifo full enable, 0x40:fifo watermark enable
	//status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);

    return status;
}


/*===========================================================================
FUNCTION      sns_dd_qmax981_reset

DESCRIPTION  Resets the driver and device so they return to the state
                    they were in after init() was called.
============================================================================*/
sns_ddf_status_e sns_dd_qmax981_reset(sns_ddf_handle_t dd_handle)
{
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *)dd_handle;
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    uint8_t i;

    if(dd_handle == NULL)
        return SNS_DDF_EINVALID_PARAM;

    for (i = 0; i < QST_ACC_NUM_AXIS; i++)
    {
        state->bias[i] = 0;
        state->data_cache[i] = 0;
    }

    QMAX981_MSG_0(HIGH, "sns_dd_qmax981_reset, entry");

    state->range = QMA6981_RANGE_4G;
    state->sstvt_adj = QMA6981_SENS_4G;
    state->data_rate = QMAX981_REGV_BW_125HZ;
    state->power_state = SNS_DDF_POWERSTATE_LOWPOWER;
    state->acc_cur_rate = 20;

    /* Stop timer interrupt before reset the device. */
    sns_ddf_timer_cancel(state->sns_dd_tmr_obj);

    status = sns_dd_qmax981_device_initialization(dd_handle);

    return status;
}

sns_ddf_status_e sns_dd_qmax981_set_odr(sns_dd_qmax981_state_t* state, sns_ddf_odr_t odr)
{
    uint32_t    i, tbl_size;
    uint8_t databuf[2] = {0};
    sns_ddf_status_e status = SNS_DDF_SUCCESS;

    tbl_size = sizeof(qma6981_odr_bw_tbl)/sizeof(qma6981_odr_bw_tbl[0]);
    for(i=0; i<tbl_size; i++)
    {
        if(odr <= qma6981_odr_bw_tbl[i].rate)
        break;
    }
    if(i < tbl_size)
    {
        databuf[0] = QMAX981_BW_SEL_REG;
        databuf[1] = qma6981_odr_bw_tbl[i].reg;
        status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
        state->acc_cur_rate = qma6981_odr_bw_tbl[i].rate;
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
sns_ddf_status_e sns_dd_qmax981_set_attr(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_attribute_e attrib,
    void* value)
{
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    sns_ddf_odr_t new_rate = 0;
    uint32_t new_rate_index = 0;
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *)dd_handle;

    QMAX981_MSG_2(HIGH, "sns_dd_qmax981_set_attr ,sensor,%d,attr,%d",sensor, attrib);

    if((sensor != SNS_DDF_SENSOR_ACCEL) && (sensor != SNS_DDF_SENSOR__ALL))
        return SNS_DDF_EINVALID_PARAM;

    uint8_t databuf[2] = {0};

    switch (attrib)
    {
        case SNS_DDF_ATTRIB_POWER_STATE:
        {
            sns_ddf_powerstate_e* power_state = value;
            databuf[0] = QMAX981_PWR_CTRL_REG;
            if (( SNS_DDF_POWERSTATE_ACTIVE == *power_state)
                &&(SNS_DDF_POWERSTATE_LOWPOWER == state->power_state))
            {
                databuf[1] = 0x80;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
                QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_POWER_STATE, write status = %d active_mode", status);
                if (status != SNS_DDF_SUCCESS)
                {
                    return SNS_DDF_EBUS;
                }
                state->power_state = SNS_DDF_POWERSTATE_ACTIVE;
            }
            else if((SNS_DDF_POWERSTATE_LOWPOWER == *power_state)
                &&(SNS_DDF_POWERSTATE_ACTIVE == state->power_state))
            {
                databuf[1] = 0x80;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
                QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_POWER_STATE, write status = %d powerdown_mode", status);
                if (status != SNS_DDF_SUCCESS)
                {
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

            QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_RANGE, range_idx = %d", *range_idx);
            databuf[0] = QMAX981_RANGE_SEL_REG;
            if(*range_idx == 0)
            {
                databuf[1] = 0x01;
                state->range = QMA6981_RANGE_2G;
                state->sstvt_adj = QMA6981_SENS_2G;
            }
            else if(*range_idx == 1)
            {
                databuf[1] = 0x02;
                state->range = QMA6981_RANGE_4G;
                state->sstvt_adj = QMA6981_SENS_4G;
            }
            else if(*range_idx == 2)
            {
                databuf[1] = 0x04;
                state->range = QMA6981_RANGE_8G;
                state->sstvt_adj = QMA6981_SENS_8G;
            }
            else
            {
                databuf[1] = 0x01;
                state->range = QMA6981_RANGE_2G;
                state->sstvt_adj = QMA6981_SENS_2G;
            }
            status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
            QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_RANGE, status = %d", status);
            return SNS_DDF_SUCCESS;
           // break;
        }

        case SNS_DDF_ATTRIB_ODR:
        {
            sns_ddf_odr_t odr = *(sns_ddf_odr_t *) value;
            QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_ODR, odr = %d", odr);
            state->acc_cur_rate = odr;
            //sns_dd_qmax981_set_odr(state, odr);
            return SNS_DDF_SUCCESS;
            //break;
        }
        case SNS_DDF_ATTRIB_MOTION_DETECT:
            state->tap_irq_flag = *((bool *)value);
#if defined(QMA6981_TAP_SUPPORT)
            if(state->tap_irq_flag)
            {
                QMAX981_MSG_0(HIGH, "enable MD interrupt = %d");
#if 0
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
                    if((status = sns_ddf_signal_register(
                        state->gpio_num,
                        dd_handle,
                        &sns_dd_qmax981_if,
                        SNS_DDF_SIGNAL_IRQ_RISING)) != SNS_DDF_SUCCESS) {
                            QMAX981_MSG_0(HIGH,"enable_sched_data sns_ddf_signal_register error!!");
                            return status;
                        }
                    state->fifo_int_config = 1;
                }
#endif
#if defined(QMA6981_TAP_SUPPORT)
                databuf[0] = 0x2a;
                databuf[1] = 0x80;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
                databuf[0] = 0x2b;
                databuf[1] = 0x01;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
                databuf[0] = 0x16;
                databuf[1] = 0x20;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
                databuf[0] = 0x19;
                databuf[1] = 0x20;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
#endif
				// enable fifo set high ODR!
                databuf[0] = QMAX981_BW_SEL_REG;
                databuf[1] = QMAX981_REGV_BW_62_5HZ;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);

				// disable fifo
				databuf[0] = 0x3e;
				databuf[1] = 0x40;	   // fifo mode
				status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
				databuf[0] = 0x17;
				databuf[1] = 0x00;		// 0x20:fifo full enable, 0x40:fifo watermark enable
                databuf[0] = 0x1a;
                databuf[1] = 0x00;    // disable fifo interrupt
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
				// disable fifo
            }
            else
            {
                QMAX981_MSG_0(HIGH, "disable MD interrupt = %d");
				// set low odr
                databuf[0] = QMAX981_BW_SEL_REG;
                databuf[1] = QMAX981_REGV_BW_7_8HZ;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);

                databuf[0] = 0x19;
                databuf[1] = 0x00;
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
            }
#endif
            return SNS_DDF_SUCCESS;
        case SNS_DDF_ATTRIB_FIFO:
        /* we are using FIFO mode */
        {
            uint8_t fifo_level = *((uint8_t *)value);
            QMAX981_MSG_1(HIGH, "SNS_DDF_ATTRIB_FIFO, fifo_level = %d", fifo_level);
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
                state->fifo_level = fifo_level; //fifo_level;

                //databuf[0] = QMAX981_BW_SEL_REG;
                //databuf[1] = QMAX981_REGV_BW_15_6HZ;		// 62.5hz
                //status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
          
                databuf[0] = 0x31;
                databuf[1] = state->fifo_level; // watermark level
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
#if 0
                databuf[0] = 0x3e;
                databuf[1] = 0x40;		// fifo mode
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
                databuf[0] = 0x17;
                databuf[1] = 0x40;		// 0x20:fifo full enable, 0x40:fifo watermark enable
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
                
                databuf[0] = 0x1a;
                databuf[1] = 0x40;		// 0x20:fifo full enable, 0x40:fifo watermark enable
                status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);                
                state->irq_ts_start = sns_ddf_get_timestamp();
#endif
                return SNS_DDF_SUCCESS;
                }
        }
        default:
            return SNS_DDF_EINVALID_PARAM;
    }
    return status;
}


/*===========================================================================
FUNCTION      sns_dd_qmax981_get_attr

DESCRIPTION  Retrieves the value of an attribute for QMA6981.
============================================================================*/
sns_ddf_status_e sns_dd_qmax981_get_attr(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_attribute_e attrib,
    sns_ddf_memhandler_s* memhandler,
    void** value,
    uint32_t* num_elems)
{
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *)dd_handle;

    QMAX981_MSG_2(HIGH, "sns_dd_qmax981_get_attr ,sensor,%d,attr,%d",sensor, attrib);
    if(sensor != SNS_DDF_SENSOR_ACCEL)
        return SNS_DDF_EINVALID_PARAM;

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

            /*current consumption, unit uA*/
            power_attr->active_current = QMA6981_ACTIVE_CURRENT;
            power_attr->lowpower_current = QMA6981_LOWPW_CURRENT;
            *value = power_attr;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_POWER_INFO OK");
            return SNS_DDF_SUCCESS;

            //break;
        }

        case SNS_DDF_ATTRIB_RANGE:
        {
            sns_ddf_range_s* ranges = sns_ddf_memhandler_malloc_ex(
                memhandler, QMA6981_RANGE_NUM * sizeof(sns_ddf_range_s),state->smgr_hndl);
            if(NULL == ranges)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RANGE fail");
                return SNS_DDF_ENOMEM;
            }

            ranges[0].min = QMAX981_ACCEL_SENSOR_RANGE_2G_MIN;
            ranges[0].max = QMAX981_ACCEL_SENSOR_RANGE_2G_MAX;
            ranges[1].min = QMAX981_ACCEL_SENSOR_RANGE_4G_MIN;
            ranges[1].max = QMAX981_ACCEL_SENSOR_RANGE_4G_MAX;
            ranges[2].min = QMAX981_ACCEL_SENSOR_RANGE_8G_MIN;
            ranges[2].max = QMAX981_ACCEL_SENSOR_RANGE_8G_MAX;

            *num_elems = QMA6981_RANGE_NUM;
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

            adc_res->bit_len = QMA6981_BIT_LEN;
            adc_res->max_freq = QMA6981_MAX_FREQ;
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

            *res = FX_FLTTOFIX_Q16(2 * QMA6981_SENS_2G * G / 1024);
            *value = res;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_RESOLUTION OK");
            return SNS_DDF_SUCCESS;

            //break;
        }

        case SNS_DDF_ATTRIB_ODR:
        {
            sns_ddf_odr_t *odr = sns_ddf_memhandler_malloc_ex(
                memhandler ,sizeof(sns_ddf_odr_t),state->smgr_hndl);
            if(NULL == odr){
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_ODR fail");
                return SNS_DDF_ENOMEM;
            }

            /*always return the current accel ODR to SMGR.*/
            if((SNS_DDF_SENSOR_ACCEL == sensor)||(SNS_DDF_SENSOR__ALL == sensor)){
                *odr = state->acc_cur_rate;
            }
            else {
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

	    *num_elems = 8;
            if((odr_list = (uint32_t *)sns_ddf_memhandler_malloc_ex(
                memhandler,
                (*num_elems) * sizeof(uint32_t),
                state->smgr_hndl)) == NULL)  //dean modify smgr_hndl smgr_handle
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST fail");
                return SNS_DDF_ENOMEM;
            }
#if 1
            odr_list[0] = 2;
            odr_list[1] = 4;
            odr_list[2] = 8;
            odr_list[3] = 16;
	    odr_list[4] = 32; 
            odr_list[5] = 64; 
            odr_list[6] = 128; 
            odr_list[7] = 256; 
            *value = (uint32_t *)odr_list;
            //*num_elems = 4;
#else
            odr_list[0] = 50;    //50;
            *value = (uint32_t *)odr_list;
            *num_elems = 1;
#endif
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_SUPPORTED_ODR_LIST OK");
            return SNS_DDF_SUCCESS;
        }
    //break;
    // add for fifo
        case SNS_DDF_ATTRIB_FIFO:
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
            //break;
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
            QMAX981_MSG_0(HIGH,"get_attr,SNS_DDF_ATTRIB_DRIVER_INFO");
            sns_ddf_driver_info_s *driver_info = sns_ddf_memhandler_malloc_ex(
                memhandler, sizeof(sns_ddf_driver_info_s),state->smgr_hndl);
            if(NULL == driver_info)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DRIVER_INFO fali");
                return SNS_DDF_ENOMEM;
            }

            driver_info->name = "QMA6981";
            driver_info->version = 1;
            *value = driver_info;
            *num_elems = 1;
            QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DRIVER_INFO OK");
            return SNS_DDF_SUCCESS;

            //break;
        }

        case SNS_DDF_ATTRIB_DEVICE_INFO:
        {
            QMAX981_MSG_0(HIGH,"get_attr,SNS_DDF_ATTRIB_DEVICE_INFO");
            sns_ddf_device_info_s *device_info = sns_ddf_memhandler_malloc_ex(
                memhandler, sizeof(sns_ddf_device_info_s),state->smgr_hndl);
            if(NULL == device_info)
            {
                QMAX981_MSG_0(HIGH, "SNS_DDF_ATTRIB_DEVICE_INFO fali");
                return SNS_DDF_ENOMEM;
            }

            device_info->name = "Accelerometer";
            device_info->vendor = "QST Corp";
            device_info->model = "QMA6981";
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
FUNCTION      sns_dd_qmax981_self_test

DESCRIPTION  Runs self test and returns the test result.
============================================================================*/
sns_ddf_status_e sns_dd_qmax981_self_test(
    sns_ddf_handle_t dd_handle,
    sns_ddf_sensor_e sensor,
    sns_ddf_test_e test,
    uint32_t* err)
{
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *)dd_handle;
    uint8_t rw_buffer = 0;
    sns_ddf_status_e status = SNS_DDF_SUCCESS;

    if(test == SNS_DDF_TEST_SELF)
    {
        status = qmax981_read_byte(state->port_handle, QMAX981_CHIP_ID_REG, &rw_buffer);
        if( status != SNS_DDF_SUCCESS && rw_buffer != QMA6981_CHIP_ID)
        return SNS_DDF_EINVALID_TEST;
    }
    else
    {
        return SNS_DDF_EINVALID_TEST;
    }
    return status;
}

sns_ddf_status_e sns_dd_qmax981_probe(
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

    QMAX981_MSG_0(HIGH,"sns_dd_qmax981_probe, entry");

    /*Open COM port*/
    status = sns_ddf_open_port(&port_handle, &(device_info->port_config));
    if (status != SNS_DDF_SUCCESS)
    {
        return status;
    }

    /* Read and Verify Sensor ID */
    status = sns_ddf_read_port(port_handle,
                    QMAX981_CHIP_ID_REG,
                    &rw_buffer,
                    1,
                    &rw_bytes);

    if(status != SNS_DDF_SUCCESS || rw_bytes != 1)
    {
        sns_ddf_close_port(port_handle);
        return status;
    }

    QMAX981_MSG_1(HIGH,"sns_dd_qmax981_probe CHIP_ID:0x%02x",rw_buffer);

    if(rw_buffer != QMA6981_CHIP_ID){
        sns_ddf_close_port(port_handle);
        return status;
    }

    QMAX981_MSG_0(HIGH,"sns_dd_qmax981_probe ,found sensor");

    *num_sensors = 1;
    *sensors = sns_ddf_memhandler_malloc( memhandler,
            sizeof(sns_ddf_sensor_e) * *num_sensors );

    if( *sensors == NULL )
    {
        status = SNS_DDF_ENOMEM;
        sns_ddf_close_port(port_handle);
        return status;
    }

    (*sensors)[0] = SNS_DDF_SENSOR_ACCEL;
    status = SNS_DDF_SUCCESS;

    QMAX981_MSG_0(HIGH,"sns_dd_qmax981_probe ,done");

    sns_ddf_close_port( port_handle );
    return status;
}


sns_ddf_status_e sns_dd_qmax981_enable_sched_data(
        sns_ddf_handle_t    dd_handle,
        sns_ddf_sensor_e    sensor_type,
        bool                enable)
{
    sns_ddf_status_e status = SNS_DDF_SUCCESS;
    sns_dd_qmax981_state_t* state = (sns_dd_qmax981_state_t *)dd_handle;	
    uint8_t databuf[2] = {0};

    QMAX981_MSG_3(HIGH, "enable_sched_data fifo_level=%d, type=%d enable=%d",
            state->fifo_level, sensor_type, enable);

    if (!sns_ddf_signal_irq_enabled()) {
        QMAX981_MSG_0(HIGH,"enable_sched_data return 0!!");
        return SNS_DDF_EINVALID_PARAM;
    }
    if (SNS_DDF_SENSOR_ACCEL != sensor_type) {
    QMAX981_MSG_0(HIGH,"enable_sched_data return 1!!");
        return SNS_DDF_SUCCESS;
    }

    if(enable)
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
            if((status = sns_ddf_signal_register(
                        state->gpio_num,
                        dd_handle,
                        &sns_dd_qmax981_if,
                        SNS_DDF_SIGNAL_IRQ_RISING)) != SNS_DDF_SUCCESS) {
                        QMAX981_MSG_0(HIGH,"enable_sched_data sns_ddf_signal_register error!!");
                        return status;
            }
            state->fifo_int_config = 1;
        }
    if(state->tap_irq_flag)
    {
        return SNS_DDF_SUCCESS;
    }
    databuf[0] = 0x3e;
    databuf[1] = 0x40;     // fifo mode
    status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
    databuf[0] = 0x17;
    databuf[1] = 0x40;		// 0x20:fifo full enable, 0x40:fifo watermark enable
    status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
    databuf[0] = 0x1a;
    databuf[1] = 0x40;		// 0x20:fifo full enable, 0x40:fifo watermark enable
    status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
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
            if((status = sns_ddf_signal_register(
                        state->gpio_num,
                        dd_handle,
                        &sns_dd_qmax981_if,
                        SNS_DDF_SIGNAL_IRQ_RISING)) != SNS_DDF_SUCCESS) {
                        QMAX981_MSG_0(HIGH,"enable_sched_data sns_ddf_signal_register error!!");
                        return status;
            }
            state->fifo_int_config = 1;
        }
    databuf[0] = 0x17;
    databuf[1] = 0x00;		// 0x20:fifo full enable, 0x40:fifo watermark enable
    status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
    databuf[0] = 0x1a;
    databuf[1] = 0x00;		// 0x20:fifo full enable, 0x40:fifo watermark enable
    status = qmax981_write_byte(state->port_handle, databuf[0],&databuf[1]);
    }

    return status;
}


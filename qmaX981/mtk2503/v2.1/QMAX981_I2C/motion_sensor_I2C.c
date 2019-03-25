/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   motion_sensor_I2C.c
 *
 *
 * Description:
 * ------------
 *   qmaX981 I2C read/write Driver
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 *****************************************************************************/
#ifdef __CUST_NEW__
#include "kal_release.h"
#include "dcl.h"
#include "motion_sensor_I2C.h"
#include "us_timer.h"               /* For ust_get_current_time() API*/

kal_bool ms_i2c_configure_done = KAL_FALSE;
	
extern const char gpio_ms_i2c_data_pin;
extern const char gpio_ms_i2c_clk_pin;
extern kal_uint32 MS_DELAY_TIME;

DCL_HANDLE ms_i2c_handle;
DCL_HANDLE ms_pmu_handle;

#define MS_I2C_DELAY \
{\
	volatile int count=0;\
	for(;count<MS_DELAY_TIME;count++);\
}

void ms_i2c_udelay(kal_uint32 delay)
{
	kal_uint32 ust = 0; //ust_get_current_time
	kal_uint32 count = 0;
	kal_uint32 break_count = 0;
	
	ust = ust_get_current_time();
	do{
		if(ust_get_current_time() != ust)
			count++;
		else
			break_count++;
	}while((count < delay) && (break_count < 0xFFFFFF));
}

void ms_i2c_configure(kal_uint32 slave_addr, kal_uint32 speed)
{
	I2C_CONFIG_T cfg;
	
	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{
		if(!ms_i2c_configure_done)
		{
			ms_i2c_handle = DclSI2C_Open(DCL_I2C, DCL_I2C_OWNER_MS);
		}
		cfg.eOwner = DCL_I2C_OWNER_MS;
		cfg.fgGetHandleWait = KAL_TRUE;
		cfg.u1SlaveAddress = slave_addr;
		cfg.u1DelayLen = 0;
		cfg.eTransactionMode = DCL_I2C_TRANSACTION_FAST_MODE;
		cfg.u4FastModeSpeed = speed;
		cfg.u4HSModeSpeed = 0;
		cfg.fgEnableDMA = KAL_FALSE;
  	
		DclSI2C_Configure(ms_i2c_handle, (DCL_CONFIGURE_T *)&cfg);
	}
	
	ms_i2c_configure_done = KAL_TRUE;
}

void ms_i2c_power_on(kal_bool ON, kal_uint32 ldo, kal_uint32 ldo_volt)
{
	PMU_CTRL_LDO_BUCK_SET_VOLTAGE_EN pmu_ldo_voltage_en;
	PMU_CTRL_LDO_BUCK_SET_EN pmu_ldo_en;
		
	if (ms_pmu_handle==DCL_HANDLE_NONE)
		ms_pmu_handle = DclPMU_Open(DCL_PMU, FLAGS_NONE);
	
	if( ON && (ldo_volt != 0) )
	{
		pmu_ldo_voltage_en.mod = (PMU_LDO_BUCK_LIST_ENUM)ldo;
		pmu_ldo_voltage_en.voltage = (PMU_VOLTAGE_ENUM)ldo_volt;
		DclPMU_Control(ms_pmu_handle, LDO_BUCK_SET_VOLTAGE_EN,(DCL_CTRL_DATA_T *)& pmu_ldo_voltage_en);
	}
	else if( ON && (ldo_volt == 0) )
	{
		pmu_ldo_en.mod = (PMU_LDO_BUCK_LIST_ENUM)ldo;
		pmu_ldo_en.enable = KAL_TRUE;
		DclPMU_Control(ms_pmu_handle, LDO_BUCK_SET_EN,(DCL_CTRL_DATA_T *)&pmu_ldo_en);
	}
	else
	{
		pmu_ldo_en.mod = (PMU_LDO_BUCK_LIST_ENUM)ldo;
		pmu_ldo_en.enable = KAL_FALSE;
		DclPMU_Control(ms_pmu_handle, LDO_BUCK_SET_EN,(DCL_CTRL_DATA_T *)&pmu_ldo_en);
	}
}

// Start bit of I2C waveform
void ms_i2c_start(void)
{
	DCL_HANDLE sda_handle, scl_handle;
	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{

	}
	else
	{		
		sda_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_data_pin);
		scl_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_clk_pin);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_MODE_0, NULL);
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_MODE_0, NULL);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_OUT, NULL);		
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_HIGH, NULL);
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		MS_I2C_DELAY;
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
		MS_I2C_DELAY;
	
		DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_LOW, NULL);
		MS_I2C_DELAY;
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
		MS_I2C_DELAY;
	
		DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_LOW, NULL);
		MS_I2C_DELAY;
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_LOW, NULL);
		MS_I2C_DELAY;
	}
}

// Stop bit of I2C waveform
void ms_i2c_stop(void)
{
	DCL_HANDLE sda_handle, scl_handle;
	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{

	}
	else
	{		
		sda_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_data_pin);
		scl_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_clk_pin);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_MODE_0, NULL);
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_MODE_0, NULL);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_LOW, NULL);
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		MS_I2C_DELAY;
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
		MS_I2C_DELAY;
	
		DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_HIGH, NULL);
		MS_I2C_DELAY;
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
		MS_I2C_DELAY;
	}
}

// Send one byte from host to client
kal_bool ms_i2c_send_byte(kal_uint8 ucData)
{
	//kal_bool bRet;
	int i;
	kal_bool ret;
	kal_uint8 ucMask;
	DCL_STATUS status = STATUS_INVALID_OPERATION;
	GPIO_CTRL_READ_T sda_read;
	I2C_CTRL_CONT_WRITE_T write;
	DCL_HANDLE sda_handle, scl_handle;

	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{
		if(ms_i2c_configure_done)
		{
			write.pu1Data = &ucData;
			write.u4DataLen = 1;
			write.u4TransferNum = 1;
			status = DclSI2C_Control(ms_i2c_handle, I2C_CMD_CONT_WRITE, (DCL_CTRL_DATA_T *)&write);
		}
		ret = (status == STATUS_OK)?KAL_TRUE:KAL_FALSE;
	}
	else
	{
		sda_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_data_pin);
		scl_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_clk_pin);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_MODE_0, NULL);
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_MODE_0, NULL);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		
		for(i = 0, ucMask = 0x80; i < 8; i++, ucMask >>= 1)
		{
			if(ucData & ucMask)
				DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_HIGH, NULL);
			else
				DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_LOW, NULL);
			MS_I2C_DELAY;
			DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
			MS_I2C_DELAY;
			DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_LOW, NULL);
			MS_I2C_DELAY;
		}
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_IN, NULL);
	
		DclGPIO_Control(sda_handle, GPIO_CMD_READ, (DCL_CTRL_DATA_T *)&sda_read);
		
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
		MS_I2C_DELAY;
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_LOW, NULL);
		MS_I2C_DELAY;
	
		ret = (sda_read.u1IOData == GPIO_IO_HIGH)?KAL_TRUE:KAL_FALSE;
	}
	return ret;
}

// Receive one byte form client to host 
kal_uint8 ms_i2c_receive_byte(kal_bool bAck)
{
	kal_uint8 ucRet = 0;
	int i;
	I2C_CTRL_CONT_READ_T read;
	GPIO_CTRL_READ_T sda_read;
	DCL_STATUS status;
	DCL_HANDLE sda_handle, scl_handle;
	
	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{
		if(ms_i2c_configure_done)
		{
			read.pu1Data = &ucRet;
			read.u4DataLen = 1;
			read.u4TransferNum = 1;
			status = DclSI2C_Control(ms_i2c_handle, I2C_CMD_CONT_READ, (DCL_CTRL_DATA_T *)&read);
			if(status != STATUS_OK)
				return 0;
		}
	}
	else
	{
		sda_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_data_pin);
		scl_handle = DclGPIO_Open(DCL_GPIO, gpio_ms_i2c_clk_pin);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_MODE_0, NULL);
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_MODE_0, NULL);
		
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_IN, NULL);
		DclGPIO_Control(scl_handle, GPIO_CMD_SET_DIR_OUT, NULL);
		
		for(i = 7; i >= 0; i--)
		{
			DclGPIO_Control(sda_handle, GPIO_CMD_READ, (DCL_CTRL_DATA_T *)&sda_read);
			ucRet |= sda_read.u1IOData << i;
	
			DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
			MS_I2C_DELAY;
			DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_LOW, NULL);
			MS_I2C_DELAY;
		}
	
		DclGPIO_Control(sda_handle, GPIO_CMD_SET_DIR_OUT, NULL);
	
		if(bAck)
			DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_HIGH, NULL);
		else
			DclGPIO_Control(sda_handle, GPIO_CMD_WRITE_LOW, NULL);
		MS_I2C_DELAY;
	
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_HIGH, NULL);
		MS_I2C_DELAY;
		DclGPIO_Control(scl_handle, GPIO_CMD_WRITE_LOW, NULL);
		MS_I2C_DELAY;

	}
	return ucRet;
}

// I2C send data fuction
kal_bool ms_i2c_send(kal_uint8 ucDeviceAddr, kal_uint8 ucBufferIndex, kal_uint8* pucData, kal_uint32 unDataLength)
{
	kal_uint32 i;
	kal_uint8 write_buf[9];
	kal_bool bRet = KAL_TRUE;
	I2C_CTRL_CONT_WRITE_T write;
	DCL_STATUS status;

	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{
		if(ms_i2c_configure_done)
		{
			write_buf[0] = ucBufferIndex;
			for(i=0;i<unDataLength;i++)
			{
				write_buf[i+1] = *(pucData+i);
			}
			write.pu1Data = write_buf;
			write.u4DataLen = unDataLength+1;
			write.u4TransferNum = 1;
			status = DclSI2C_Control(ms_i2c_handle, I2C_CMD_CONT_WRITE, (DCL_CTRL_DATA_T *)&write);
			if(status != STATUS_OK)
				return KAL_FALSE;
		}
	}
	else
	{
		ms_i2c_start();
	
		if(ms_i2c_send_byte(ucDeviceAddr & 0xFE) == MS_I2C_ACK)
		{
			if(ms_i2c_send_byte(ucBufferIndex) == MS_I2C_ACK)
			{
				for(i = 0; i < unDataLength; i++)
				{
					if(ms_i2c_send_byte(pucData[i]) == MS_I2C_NAK)
					{
						bRet = KAL_FALSE;
						break;
					}
				}
			}
			else
			{
				bRet = KAL_FALSE;
			}
		}
		else
		{
			bRet = KAL_FALSE;
		}
		ms_i2c_stop();
	}
	return bRet;
}

kal_bool ms_i2c_send_ext(kal_uint8 ucDeviceAddr, kal_uint16 ucBufferIndex, kal_uint8* pucData, kal_uint32 unDataLength)
{
	kal_uint32 i;
	kal_uint8 write_buf[10];
	kal_bool bRet = KAL_TRUE;
	I2C_CTRL_CONT_WRITE_T write;
	DCL_STATUS status;
	kal_uint8 addr_h = ( ucBufferIndex >> 8 )& 0xFF;
	kal_uint8 addr_l = ucBufferIndex&0xFF;
	kal_uint32 offset = 0;
	kal_uint8  pkt_len;

	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{
		if(ms_i2c_configure_done)
		{
			while ( offset <= unDataLength )
			{
				write_buf[0] = ((ucBufferIndex + offset)>>8)&0xFF;
				write_buf[1] = (ucBufferIndex + offset)&0xFF;
          			if ( unDataLength - offset > 6 )
				{
					pkt_len = 6;
				}
				else
				{
					pkt_len = unDataLength - offset;
				}
				memcpy( &write_buf[2], &pucData[offset], pkt_len );
				offset += pkt_len;
				write.pu1Data = write_buf;
				write.u4DataLen = pkt_len+2;
				write.u4TransferNum = 1;
				status = DclSI2C_Control(ms_i2c_handle, I2C_CMD_CONT_WRITE, (DCL_CTRL_DATA_T *)&write);
				if(status != STATUS_OK)
					return KAL_FALSE;
					
				if ( offset == unDataLength )
					break;					
			}
		}
	}
	else
	{
		ms_i2c_start();
	
		if(ms_i2c_send_byte(ucDeviceAddr & 0xFE) == MS_I2C_ACK)
		{
			if ( (ms_i2c_send_byte(addr_h) == MS_I2C_ACK) &&
				(ms_i2c_send_byte(addr_l) == MS_I2C_ACK) )
			{
				for(i = 0; i < unDataLength; i++)
				{
					if(ms_i2c_send_byte(pucData[i]) == MS_I2C_NAK)
					{
						bRet = KAL_FALSE;
						break;
					}
				}
			}
			else
			{
				bRet = KAL_FALSE;
			}
		}
		else
		{
			bRet = KAL_FALSE;
		}
		ms_i2c_stop();
	}
	return bRet;
}

// I2C receive data function
kal_bool ms_i2c_receive(kal_uint8 ucDeviceAddr, kal_uint8 ucBufferIndex, kal_uint8* pucData, kal_uint32 unDataLength)
{
	kal_uint32 i;
	kal_bool bRet = KAL_TRUE;
	I2C_CTRL_WRITE_AND_READE_T write_read;

	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{
		if(ms_i2c_configure_done)
		{
			write_read.pu1InData = pucData;
			write_read.u4InDataLen = unDataLength;
			write_read.pu1OutData = &ucBufferIndex;			
			write_read.u4OutDataLen = 1;
			DclSI2C_Control(ms_i2c_handle, I2C_CMD_WRITE_AND_READ, (DCL_CTRL_DATA_T *)&write_read);
		}
	}
	else
	{
		ms_i2c_start();
  	
		if(ms_i2c_send_byte(ucDeviceAddr & 0xFE) == MS_I2C_ACK)
		{
			if(ms_i2c_send_byte(ucBufferIndex) == MS_I2C_ACK)
			{
				ms_i2c_start();
  	
				if(ms_i2c_send_byte(ucDeviceAddr | 0x01) == MS_I2C_ACK)
				{
					for(i = 0; i < unDataLength - 1; i++)
					{
						pucData[i] = ms_i2c_receive_byte((kal_bool)MS_I2C_ACK);
					}
					pucData[unDataLength - 1] = ms_i2c_receive_byte((kal_bool)MS_I2C_NAK);
				}
				else
				{
					bRet = KAL_FALSE;
				}
			}
			else
			{
				bRet = KAL_FALSE;
			}
		}
		else
		{
			bRet = KAL_FALSE;
		}
		ms_i2c_stop();
	}
	return bRet;
}

// I2C receive data function
kal_bool ms_i2c_receive_ext(kal_uint8 ucDeviceAddr, kal_uint16 ucBufferIndex, kal_uint8* pucData, kal_uint32 unDataLength)
{
	kal_uint32 i;
	kal_bool bRet = KAL_TRUE;
	I2C_CTRL_WRITE_AND_READE_T write_read;
	kal_uint8 write_buf[2];
	kal_uint16 reg_addr = ucBufferIndex;
	kal_uint32 offset = 0;
	kal_uint8  pkt_len;
	kal_uint8 addr_h = ( ucBufferIndex >> 8 )& 0xFF;
	kal_uint8 addr_l = ucBufferIndex&0xFF;

	if(gpio_ms_i2c_data_pin == 0xFF) // HW I2C
	{
		if(ms_i2c_configure_done)
		{
			while ( offset < unDataLength )
			{
				write_buf[0] = ( reg_addr >> 8 )& 0xFF;
				write_buf[1] = reg_addr&0xFF;
				if ( unDataLength - offset > 8 )
				{
					pkt_len = 8;
				}
				else
				{
					pkt_len = unDataLength - offset;
				}
				write_read.pu1InData = pucData + offset;
				write_read.u4InDataLen = pkt_len;
				write_read.pu1OutData = write_buf;			
				write_read.u4OutDataLen = 2;
				DclSI2C_Control(ms_i2c_handle, I2C_CMD_WRITE_AND_READ, (DCL_CTRL_DATA_T *)&write_read);
				offset += pkt_len;
				reg_addr = ucBufferIndex + offset;
			}
		}
	}
	else
	{
		ms_i2c_start();
  	
		if(ms_i2c_send_byte(ucDeviceAddr & 0xFE) == MS_I2C_ACK)
		{
			if ( (ms_i2c_send_byte(addr_h) == MS_I2C_ACK) &&
				(ms_i2c_send_byte(addr_l) == MS_I2C_ACK) )
			{
				ms_i2c_start();
  	
				if(ms_i2c_send_byte(ucDeviceAddr | 0x01) == MS_I2C_ACK)
				{
					for(i = 0; i < unDataLength - 1; i++)
					{
						pucData[i] = ms_i2c_receive_byte((kal_bool)MS_I2C_ACK);
					}
					pucData[unDataLength - 1] = ms_i2c_receive_byte((kal_bool)MS_I2C_NAK);
				}
				else
				{
					bRet = KAL_FALSE;
				}
			}
			else
			{
				bRet = KAL_FALSE;
			}
		}
		else
		{
			bRet = KAL_FALSE;
		}
		ms_i2c_stop();
	}
	return bRet;
}

#endif //#ifdef __CUST_NEW__


#ifdef __CUST_NEW__
extern const char gpio_ms_i2c_data_pin;
extern const char gpio_ms_i2c_clk_pin;

#define MS_SCL	gpio_ms_i2c_clk_pin
#define MS_SDA	gpio_ms_i2c_data_pin
#else
#define MS_SCL	16
#define MS_SDA	17
#endif

#if defined(USE_MTK_SW_IIC)

#define MS_CLK_PIN_GPIO_MODE		GPIO_ModeSetup(MS_SCL,0)
#define	MS_DATA_PIN_GPIO_MODE		GPIO_ModeSetup(MS_SDA,0)
#define MS_I2C_CLK_OUTPUT			GPIO_InitIO(OUTPUT,MS_SCL)
#define MS_I2C_DATA_OUTPUT			GPIO_InitIO(OUTPUT,MS_SDA)
#define MS_I2C_DATA_INPUT		   	GPIO_InitIO(INPUT,MS_SDA)
#define MS_I2C_CLK_HIGH				GPIO_WriteIO(1,MS_SCL)
#define MS_I2C_CLK_LOW				GPIO_WriteIO(0,MS_SCL)
#define MS_I2C_DATA_HIGH			GPIO_WriteIO(1,MS_SDA)
#define MS_I2C_DATA_LOW				GPIO_WriteIO(0,MS_SDA)
#define MS_I2C_GET_BIT				GPIO_ReadIO(MS_SDA)

static void SW_i2c_udelay(kal_uint32 delay)
{
    kal_uint32 ust = 0; //ust_get_current_time
    kal_uint32 count = 0;
    kal_uint32 break_count = 0;

    ust = ust_get_current_time();
    do{
        if(ust_get_current_time() != ust)
            count++;
        else
            break_count++;
    }while((count < delay) && (break_count < 0xFFFFFF));
}

static void SW_i2c_start(void)
{
	MS_CLK_PIN_GPIO_MODE;
	MS_I2C_CLK_OUTPUT;

	MS_DATA_PIN_GPIO_MODE;
	MS_I2C_DATA_OUTPUT;
	
	MS_I2C_DATA_HIGH;
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(40);		//20
	MS_I2C_DATA_LOW;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_LOW;
	SW_i2c_udelay(20);		//10
}

/******************************************
	software I2C stop bit
*******************************************/
static void SW_i2c_stop(void)
{
	MS_I2C_CLK_OUTPUT;
	MS_I2C_DATA_OUTPUT;
	
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(20);		//10
	MS_I2C_DATA_HIGH;
}

/******************************************
	software I2C one clock
*******************************************/
static void SW_i2c_one_clk(void)
{
	SW_i2c_udelay(10);		//5
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_LOW;
	SW_i2c_udelay(10);		//5
}

/******************************************
	software I2C read byte with ack
*******************************************/
static kal_uint8 ms_ReadByteAck(void)
{
	kal_int8 i;
	kal_uint8 data;

	MS_I2C_DATA_INPUT; 
	data = 0; 
	
	for (i=7; i>=0; i--) 
	{
		if (MS_I2C_GET_BIT)
		{
			data |= (0x01<<i);
		}
		SW_i2c_one_clk();
	}			                                

	MS_I2C_DATA_OUTPUT;                    
	MS_I2C_DATA_LOW;                       
	SW_i2c_one_clk();                         

	return data;
}

/******************************************
	software I2C read byte without ack
*******************************************/
static kal_uint8 ms_ReadByteNAck(void)
{
	kal_int8 i;
	kal_uint8 data;

	MS_I2C_DATA_INPUT; 
	data = 0; 
	
	for (i=7; i>=0; i--) 
	{
		if (MS_I2C_GET_BIT)
		{
			data |= (0x01<<i);
		}
		SW_i2c_one_clk();
	}			                                

	MS_I2C_DATA_OUTPUT;                                           
	MS_I2C_DATA_HIGH;
	SW_i2c_one_clk();                         
	
	return data;
}

/******************************************
	software I2C send byte
*******************************************/
static void ms_SendByte(kal_uint8 sData) 
{
	kal_int8 i;
	MS_I2C_DATA_OUTPUT;       
	for (i=7; i>=0; i--) 
	{            
		if ((sData>>i)&0x01) 
		{               
			MS_I2C_DATA_HIGH;	              
		} 
		else 
		{ 
			MS_I2C_DATA_LOW;                  
		}
		SW_i2c_one_clk();                        
	}		
}
/******************************************
	software I2C check ack bit
*******************************************/
static kal_bool ms_ChkAck(void)//
{
	MS_I2C_DATA_INPUT;
	SW_i2c_udelay(10);		//5
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(10);		//5

	if(MS_I2C_GET_BIT)		//Non-ack
	{
		SW_i2c_udelay(10);	//5
		MS_I2C_CLK_LOW;
		SW_i2c_udelay(10);	//5
		MS_I2C_DATA_OUTPUT;
		MS_I2C_DATA_LOW;
		
		return KAL_FALSE;
	}
	else					//Ack
	{
		SW_i2c_udelay(10);	//5
		MS_I2C_CLK_LOW;
		SW_i2c_udelay(10);	//5
		MS_I2C_DATA_OUTPUT;
		MS_I2C_DATA_LOW;

		return KAL_TRUE;
	}
}

/******************************************
	software I2C restart bit
*******************************************/
static void ms_Restart(void)
{
	MS_I2C_CLK_OUTPUT;
	MS_I2C_DATA_OUTPUT;

	SW_i2c_udelay(40);
	MS_I2C_DATA_HIGH;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_HIGH;
	SW_i2c_udelay(40);
	MS_I2C_DATA_LOW;
	SW_i2c_udelay(20);		//10
	MS_I2C_CLK_LOW;
	SW_i2c_udelay(20);		//10
}

/******************************************
	QMA6981 ms delay function
		uint: ms
*******************************************/
/*
void qmaX981_DelayMS(kal_uint16 delay)
{
	kal_uint16 i=0;

	for(i=0; i<delay; i++)
	{
		SW_i2c_udelay(1000);
	}
}
*/

kal_bool qst_read_one_byte(kal_uint8 slave_addr, kal_uint8 RegAddr, kal_uint8* Data)
{
	SW_i2c_start();						//start bit
	ms_SendByte(slave_addr);		//slave address|write bit
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		SW_i2c_stop();
		return KAL_FALSE;
	}
		
	ms_SendByte(RegAddr);				//send RegAddr
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_Restart();						//restart bit

	ms_SendByte(slave_addr|I2C_READ_FLAG);		//slave address|read bit
	if(KAL_FALSE == ms_ChkAck())
	{
		SW_i2c_stop();
		return KAL_FALSE;
	}

	*Data = ms_ReadByteNAck();

	SW_i2c_stop();						//stop bit

	//TO_DO: add debug code to display the data received

	return KAL_TRUE;
	
}


kal_bool qst_read_bytes(kal_uint8 slave_addr, kal_uint8 RegAddr, kal_uint8* Data, kal_uint32 Length)
{
	kal_uint8* Data_ptr;
	kal_uint16 i;

	Data_ptr = Data;
	
	SW_i2c_start();						//start bit
	ms_SendByte(slave_addr);		//slave address|write bit
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send write id		
		SW_i2c_stop();
		return KAL_FALSE;
	}
		
	ms_SendByte(RegAddr);				//send RegAddr
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display ack check fail when send RegAddr		
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_Restart();						//restart bit

	ms_SendByte(slave_addr|I2C_READ_FLAG);		//slave address|read bit
	if(KAL_FALSE == ms_ChkAck())
	{
		//TO_DO: display ack check fail when send read id		
		SW_i2c_stop();
		return KAL_FALSE;
	}

	for(i=Length; i>1; i--)
	{
		*Data_ptr = ms_ReadByteAck();	//read byte with ack
		Data_ptr++;
	}
	
	*Data_ptr = ms_ReadByteNAck();		//read byte with non-ack to stop reading

	SW_i2c_stop();						//stop bit

	//TO_DO: add debug code to display the data received

	return KAL_TRUE;
}


kal_bool qst_write_byte(kal_uint8 slave_addr, kal_uint8 RegAddr, kal_uint8 Data)
{
	SW_i2c_start();						//start bit

	ms_SendByte(slave_addr);		//slave address|write bit
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send write id
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_SendByte(RegAddr);				//send RegAddr
	if(KAL_FALSE == ms_ChkAck())		//check Ack bit
	{
		//TO_DO: display check ack fail when send RegAddr
		SW_i2c_stop();
		return KAL_FALSE;
	}

	ms_SendByte(Data);					//send parameter
	if(KAL_FALSE == ms_ChkAck())
	{
		//TO_DO: display check ack fail when send data
		SW_i2c_stop();
		return KAL_FALSE;
	}

	SW_i2c_stop();						//stop bit

	return KAL_TRUE;
}
#endif


#if defined(USE_QST_SW_IIC)
#define QST_IIC_DELAY	10

#define MS_CLK_PIN_GPIO_MODE		GPIO_ModeSetup(MS_SCL,0)
#define	MS_DATA_PIN_GPIO_MODE		GPIO_ModeSetup(MS_SDA,0)

#define I2C_SCL_OUTPUT()			GPIO_InitIO(OUTPUT,MS_SCL)
#define I2C_SCL_INPUT()				GPIO_InitIO(INPUT,MS_SCL)
#define I2C_SDA_OUTPUT()			GPIO_InitIO(OUTPUT,MS_SDA)
#define I2C_SDA_INPUT()				GPIO_InitIO(INPUT,MS_SDA)

#define I2C_SCL_1()					GPIO_WriteIO(1,MS_SCL)
#define I2C_SCL_0()					GPIO_WriteIO(0,MS_SCL)
#define I2C_SDA_1()					GPIO_WriteIO(1,MS_SDA)
#define I2C_SDA_0()					GPIO_WriteIO(0,MS_SDA)
#define I2C_SDA_READ()				GPIO_ReadIO(MS_SDA)

void i2c_Ack(void);
void i2c_NAck(void);

static void i2c_Delay(void)
{
    kal_uint32 ust = 0; //ust_get_current_time
    kal_uint32 count = 0;
    kal_uint32 break_count = 0;

    ust = ust_get_current_time();
    do{
        if(ust_get_current_time() != ust)
            count++;
        else
            break_count++;
    }while((count < QST_IIC_DELAY) && (break_count < 0xFFFFFF));
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();

	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参：_ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_SendByte(kal_uint8 _ucByte)
{
	kal_uint8 i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参：无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
kal_uint8 i2c_ReadByte(kal_uint8 ack)
{
	kal_uint8 i;
	kal_uint8 value;

	/* 读到第1个bit为数据的bit7 */
	I2C_SDA_INPUT();	// set data input	
	i2c_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		//I2C_SCL_1();
		//i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_1();
		i2c_Delay();
		I2C_SCL_0();
		i2c_Delay();
	}
	
	I2C_SDA_OUTPUT();	// set data output	
	i2c_Delay();
	if(ack==0)
		i2c_NAck();
	else
		i2c_Ack();
	return value;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
kal_uint8 i2c_WaitAck(void)
{
	kal_uint8 re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	I2C_SDA_INPUT();	//set data input
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if(I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	I2C_SDA_OUTPUT();	//set data input
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************
*	函 数 名: i2c_GPIO_Config
*	功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_GPIO_Config(void)
{
	GPIO_ModeSetup(MS_SCL,0);
	GPIO_ModeSetup(MS_SDA,0);
	GPIO_WriteIO(1,MS_SCL);
	GPIO_WriteIO(1,MS_SDA);

	i2c_Stop();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参：_Address：设备的I2C总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
kal_uint8 i2c_CheckDevice(kal_uint8 _Address)
{
	kal_uint8 ucAck;

	i2c_GPIO_Config();		/* 配置GPIO */
	i2c_Start();		/* 发送启动信号 */
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_Address);
	ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */
	i2c_Stop();			/* 发送停止信号 */

	return ucAck;
}


kal_uint8 qst_sw_writereg(kal_uint8 slave, kal_uint8 reg_add, kal_uint8 reg_dat)
{
	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_dat);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_Stop();

	return 1;
}

kal_uint8 qst_sw_readreg(kal_uint8 slave, kal_uint8 reg_add, kal_uint8 *buf, kal_uint8 num)
{
	//kal_uint8 ret;
	kal_uint8 i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);
	if(i2c_WaitAck())
	{
		return 0;
	}

	i2c_Start();
	i2c_SendByte(slave|I2C_READ_FLAG);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(num-1);i++){
		*buf=i2c_ReadByte(1);
		buf++;
	}
	*buf=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
}

#endif


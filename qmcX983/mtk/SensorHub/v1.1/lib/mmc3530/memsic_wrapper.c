/*===========================================================================
*
* MAGNETIC SENSOR DRIVER
* Copyright (c) 2016, "Memsic Inc."
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. Neither the name of "Memsic Inc." nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
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

============================================================================*/
/*===========================================================================

REVISON HISTORY FOR FILE
This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.

when            who       what, where, why
----------      ----      ------------------------------------------------------------------------------
Jan  06, 2016     Miao        V1.0 Create
=============================================================================*/

//++add by memsic

#include "sensor_manager.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <FreeRTOSConfig.h>
#include <platform.h>
#include "FreeRTOS.h"
#include "task.h"
#include "hwsen.h"

#include "memsic_wrapper.h"
#include "MagCaliAlgo.h"

#define WRAPPER_MAG_LOG_MSG

#define WRAPPER_TAG                "[MAG_WRAPPER] "
#define WRAPPER_ERR(fmt, arg...)   PRINTF_D(MAG_TAG"%d: "fmt, __LINE__, ##arg)

#ifdef WRAPPER_MAG_LOG_MSG
#define WRAPPER_LOG(fmt, arg...)   PRINTF_D(WRAPPER_TAG fmt, ##arg)
#define WRAPPER_FUN(f)             PRINTF_D("%s\n", __FUNCTION__)
#else
#define WRAPPER_LOG(fmt, arg...)
#define WRAPPER_FUN(f)
#endif
static float dRawMag[3];
//static double dRawGyr[3];


int MEMSIC_SetAData(int16_t Ax, int16_t Ay, int16_t Az,  int64_t  timestamp)
{
       //here need konw the datatype about acc;

        return 1;
}
int MEMSIC_SetGData(int16_t Ax, int16_t Ay, int16_t Az, int16_t Gx, int16_t Gy, int16_t Gz, int64_t  timestamp)
{
        return 1;
}

int MEMSIC_SetMagData(double RawX, double RawY, double RawZ,  int64_t  timestamp)
{
        //WRAPPER_LOG("MEMSIC_SetMagData87\n");
        dRawMag[0] = RawX;
        dRawMag[1] = RawY;
        dRawMag[2] = RawZ;
        //WRAPPER_LOG("MEMSIC_SetMagData91\n");
        MainAlgorithmProcess(dRawMag);
      //  WRAPPER_LOG("MEMSIC_SetMagData out\n");
        return 1;
}

int MEMSIC_Calibrate(float data_cali[3], float data_offset[3], int8_t *sf_status)
{
       // WRAPPER_LOG("MEMSIC_Calibrate\n");
        GetCalMagdata(data_cali);
        GetOffsetdata(data_offset);
        *sf_status = GetMagAccuracy();
        //WRAPPER_LOG("MEMSIC_Calibrate out\n");

        return 1;
}



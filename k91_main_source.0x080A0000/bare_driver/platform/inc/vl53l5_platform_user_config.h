/*******************************************************************************
Copyright (C) 2020, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/**
 * @file  vl53l5_platform_user_config.h
 *
 * @brief EwokPlus compile time user modifiable configuration
 */

#ifndef _VL53L5_PLATFORM_USER_CONFIG_H_
#define _VL53L5_PLATFORM_USER_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
#ifdef VL53L5_API_EXPORTS
#define VL53L5_API  __declspec(dllexport)
#else
#define VL53L5_API
#endif
#else
#define VL53L5_API
#endif

#define VL53L5_RAW_RANGE_ID_STRING "rawrange"

#define VL53L5_DEFAULT_COMMS_POLLING_DELAY_MS 10
#define VL53L5_STOP_COMMAND_TIMEOUT 1000
#define VL53L5_MCU_BOOT_WAIT_DELAY 50
#define VL53L5_HP_IDLE_WAIT_DELAY 0
#define VL53L5_LP_IDLE_WAIT_DELAY 0
#define VL53L5_RANGE_WAIT 1000

#ifdef __cplusplus
}
#endif
#endif                          /* _VL53L5_PLATFORM_USER_CONFIG_H_ */

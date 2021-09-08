/*
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of Stm32 F407 Firmware and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, Stm32 F407 Firmware may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/
#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_
#include <stdio.h>
#include <stdint.h>
//#include "main.h"

#define LOBYTE(x)  ((uint8_t)(x & 0x00FFU))
#define HIBYTE(x)  ((uint8_t)((x & 0xFF00U) >> 8U))
#define SPIx_TIMEOUT_MAX        0x1000
#define SPI_16I_READWRITE_BIT 0x8000
#define SPI_16I_MAX_INDEX     0xFFFF
#define SPI_16I_WRITE(x) (uint16_t)(x | SPI_16I_READWRITE_BIT)  // 1
#define SPI_16I_READ(x)  (uint16_t)(x & ~SPI_16I_READWRITE_BIT) // 0

int32_t BSP_SPI_16M_Write_Sequence(void* SPIinterfaces, void* csport, uint16_t* cspin, uint16_t index, uint8_t * values,
                                   uint32_t count);
int32_t BSP_SPI_16M_Read_Sequence(void* SPIinterfaces, void* csport, uint16_t* cspin, uint16_t index, uint8_t * values,
                                  uint32_t count);

typedef enum {
  BSPControlDisable = 0,
  BSPControlEnable,
} BSPControl;

#endif /* INC_SPI_DRIVER_H_ */

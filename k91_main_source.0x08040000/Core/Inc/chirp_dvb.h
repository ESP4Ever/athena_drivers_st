/*
Copyright (c) 2016-2019, Chirp Microsystems
All rights reserved.

Chirp Microsystems CONFIDENTIAL

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

You can contact the authors of this program by email at support@chirpmicro.com
or by mail at 2070 Allston Way Suite 300, Berkeley, CA 94704.
*/

/**
  * Portions copyright ST Microelectronics
  *
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHIRP_DVB_H
#define __CHIRP_DVB_H

#include "sensor_device.h"

#include "chirp_board_config.h"
#include "soniclib.h"
#include "chirp_bsp.h"

#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <stdio.h>
#include <string.h>

/* Sensor and I2C bus counts - normally taken from chirp_config.h */
#ifdef CHIRP_MAX_NUM_SENSORS
#define CHBSP_MAX_DEVICES 		CHIRP_MAX_NUM_SENSORS
#else
#define CHBSP_MAX_DEVICES 		1
#endif

#ifdef CHIRP_NUM_I2C_BUSES
#define CHBSP_NUM_I2C_BUSES		CHIRP_NUM_I2C_BUSES
#else
#define CHBSP_NUM_I2C_BUSES 	1
#endif

/* RTC calibration pulse length */
#define CHBSP_RTC_CAL_PULSE_MS	100     // length of pulse applied to sensor INT line during clock cal, in ms

/* I2C Addresses and bus index for each possible device, indexed by device number */
#define CHIRP_I2C_ADDRS		{ 45 }
#define CHIRP_I2C_BUSES		{ 0 }
/* I2C bus speed */
#define I2C_BUS_SPEED		(400000)
#define I2C_TIMEOUT	2000    // XXX move

/* Flags for special I2C handling by Chirp driver */
#define I2C_DRV_FLAGS	(I2C_DRV_FLAG_RESET_AFTER_NB | I2C_DRV_FLAG_USE_PROG_NB)        // reset i2c interface after non-blocking,
                                                                                                                                                                        // use programming interface for non-blocking

extern ch_group_t *sensor_group_ptr;
extern ch_io_int_callback_t io_int_callback_ptr;

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
///* User can use this section to tailor TIMx instance used and associated
//   resources */
///* Definition for TIMx clock resources */
//#define TIMx                           TIM2
//#define TIMx_CLK_ENABLE                __HAL_RCC_TIM3_CLK_ENABLE
//
///* Definition for TIMx's NVIC */
//#define TIMx_IRQn                      TIM2_IRQn
//#define TIMx_IRQHandler                TIM2_IRQHandler
//#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()
/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */

/*
 * Interrupt priorities
 */
#define CH_SENSOR_IRQ_PRIORITY			15      // interrupt priority for external I/O line connected to Chirp sensor

/* IO configuration */
//check this define along with the main.h defines
#define RESET_N_PORT 				GPIOC
#define RESET_N_PORT_ENABLE() 		__HAL_RCC_GPIOC_CLK_ENABLE()
#define RESET_N_PIN 				GPIO_PIN_2
#define RESET_N_MODE 				GPIO_MODE_OUTPUT_PP
#define RESET_N_ASSERTED_LEVEL 		GPIO_PIN_RESET
#define RESET_N_DEASSERTED_LEVEL	GPIO_PIN_SET

#define PROG_DEASSERTED_LEVEL GPIO_PIN_RESET
#define PROG_ASSERTED_LEVEL GPIO_PIN_SET

#define PROG0_PORT 				GPIOC
#define PROG0_PORT_ENABLE() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define PROG0_PIN 				GPIO_PIN_1

#define PROG1_PORT				GPIOC
#define PROG1_PORT_ENABLE() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define PROG1_PIN				GPIO_PIN_0

/* IO pins must be on the same port! */
#define CM_IO_PORT				GPIOC
#define CM_IO_PORT_ENABLE()		__HAL_RCC_GPIOC_CLK_ENABLE()

#define IO_PIN					GPIO_PIN_3

#define IO_PIN_ALL (IO_PIN)

#define IO_EXTI_IRQn		EXTI3_IRQn

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions ------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c[CHBSP_NUM_I2C_BUSES];

//extern TIM_HandleTypeDef tim3;                                                // XXX rename

/**
 * @brief Unique ID register address location
 */
#define ID_UNIQUE_ADDRESS		0x1FFF7A10

/**
 * @brief  Get unique ID number in 32-bit format
 * @note   STM32F4xx has 96bits long unique ID, so 3 4-bytes values are available for read in 32-bit format
 * @note   Defined as macro to get maximal response time
 * @param  x: Byte number: specify which part of 16 bits you want to read
 *               - Values between 0 and 2 are allowed
 * @retval Unique ID address
 */
#define TM_ID_GetUnique32(x)	((x >= 0 && x < 3) ? (*(uint32_t *) (ID_UNIQUE_ADDRESS + 4 * (x))) : 0)

/* ================================================================================= */
/* Hello Chirp application version */
#define	APP_VERSION_MAJOR	1       // major version
#define APP_VERSION_MINOR	0       // minor version
#define APP_VERSION_REV 	4       // revision

/*========================= Sensor Firmware Selection ===========================*/

/* Select sensor firmware to use 
 *   The sensor firmware is specified during the call to ch_init(), by
 *   giving the name (address) of the firmware initialization function 
 *   that will be called.
 *
 *   Uncomment ONE of the following lines to use that sensor firmware type.
 *   Note that you must choose a firmware image that is appropriate for the 
 *   sensor model you are using (CH101 or CH201).  
 *
 *   To use a different sensor firmware type (e.g. a new distribution from
 *   Chirp), simply define CHIRP_SENSOR_FW_INIT_FUNC to equal the name of
 *   the init routine for the new firmware.
 */

//#define        CHIRP_SENSOR_FW_INIT_FUNC      ch101_gpr_open_init             /* CH101 GPR OPEN firmware */
#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprmt_init        /* CH201 GPR Multi-Threshold firmware */

/* SHORT RANGE OPTION:
 *   Uncomment the following line to use different CH101 sensor f/w optimized for 
 *   short range. The short range firmware has 4 times the resolution, but 
 *   only 1/4 the maximum range.  If you use this option, you should redefine 
 *   the CHIRP_SENSOR_MAX_RANGE_MM symbol, below, to 250mm or less.
 */
// #define      USE_SHORT_RANGE                 /* use short-range firmware */

#ifdef USE_SHORT_RANGE
#undef	 CHIRP_SENSOR_FW_INIT_FUNC
#define	 CHIRP_SENSOR_FW_INIT_FUNC		ch101_gpr_sr_open_init  /* CH101 GPR SR OPEN firmware (short range) */
#endif

/*============================ Sensor Configuration =============================*/

/* Define configuration settings for the Chirp sensors 
 *   The following symbols define configuration values that are used to 
 *   initialize the ch_config_t structure passed during the ch_set_config() 
 *   call.  
 */
#define	CHIRP_SENSOR_MAX_RANGE_MM		2000    /* maximum range, in mm */

#define	CHIRP_SENSOR_STATIC_RANGE		0       /* static target rejection sample 
                                                           range, in samples (0=disabled) */
#define CHIRP_SENSOR_SAMPLE_INTERVAL	0       /* internal sample interval - 
                                                   NOT USED IF TRIGGERED */

/*============================= Application Timing ==============================*/

/* Define how often the application will get a new sample from the sensor(s) 
 *   This macro defines the sensor sample interval, in milliseconds.  The 
 *   application will use a timer to trigger a sensor measurement after 
 *   this period elapses.
 */
#define	MEASUREMENT_INTERVAL_MS		100     // 100ms interval = 10Hz sampling

/*===================  Application Storage for Sensor Data ======================*/

/* Define how many I/Q samples are expected by this application
 *   The following macro is used to allocate space for I/Q data in the 
 *   "chirp_data_t" structure, defined below.  Because a Chirp CH201 sensor 
 *   has more I/Q data than a CH101 device, the CH201 sample count is used 
 *   here.
 *   If you are ONLY using CH101 devices with this application, you may 
 *   redefine the following define to equal CH101_MAX_NUM_SAMPLES to use 
 *   less memory.
 */
#define IQ_DATA_MAX_NUM_SAMPLES  CH201_MAX_NUM_SAMPLES  // use CH201 I/Q size

/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from 
 *   a sensor.  The data values include the measured range, the ultrasonic 
 *   signal amplitude, the number of valid samples (I/Q data pairs) in the 
 *   measurement, and the raw I/Q data from the measurement.
 *
 *  The format of this data structure is specific to this application, so 
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor, 
 *  is declared in the hello_chirp.c file.  The sensor's device number is 
 *  used to index the array.
 */
typedef struct {
  uint32_t range;               // from ch_get_range()
  uint16_t amplitude;           // from ch_get_amplitude()
  uint16_t num_samples;         // from ch_get_num_samples()
  ch_iq_sample_t iq_data[IQ_DATA_MAX_NUM_SAMPLES];      // from ch_get_iq_data()
} chirp_data_t;

extern chirp_data_t chirp_data[];

/*===================  Build Options for I/Q Data Handling ======================*/

/* The following build options control if and how the raw I/Q data is read 
 * from the device after each measurement cycle, in addition to the standard 
 * range and amplitude.  Comment or un-comment the various definitions, as 
 * appropriate.
 *
 * Note that reading the I/Q data is not required for most basic sensing 
 * applications - the reported range value is typically all that is required. 
 * However, the full data set may be read and analyzed for more advanced 
 * sensing needs.
 *
 * By default, this application will read the I/Q data in blocking mode 
 * (i.e. READ_IQ_DATA_BLOCKING is defined by default).  The data will be read 
 * from the device and placed in the I/Q data array field in application's 
 * chirp_data structure.
 *
 * Normally, the I/Q data is read in blocking mode, so the call to 
 * ch_get_iq_data() will not return until the data has actually been read from 
 * the device.  However, if READ_IQ_DATA_NONBLOCK is defined instead, the I/Q 
 * data will be read in non-blocking mode. The ch_get_iq_data() call will 
 * return immediately, and a separate callback function will be called to 
 * notify the application when the read operation is complete.
 *
 * Finally, if OUTPUT_IQ_DATA_CSV is defined, the application will write the 
 * I/Q data bytes out through the serial port in ascii form as comma-separated 
 * numeric value pairs.  This can make it easier to take the data from the 
 * application and analyze it in a spreadsheet or other program.
 */

#define READ_IQ_DATA_BLOCKING   /* define for blocking I/Q data read */
// #define READ_IQ_DATA_NONBLOCK        /* define for non-blocking I/Q data read */

// #define OUTPUT_IQ_DATA_CSV           /* define to output I/Q data in CSV format*/

#endif /* __CHIRP_DVB_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

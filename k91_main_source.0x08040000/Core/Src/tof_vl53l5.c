/*
 * tof_vl53l5.c
 *
 *  Created on: Dec 22, 2020
 *      Author: sangweilin@xiaomi.com
 */

/*
Copyright (C) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5 Bare Driver and is dual licensed,
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
* Alternatively, the VL53L5 Bare Driver may be distributed under the terms of
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

#include "tof_vl53l5.h"
#include "sensor_device.h"

#include "vl53l5_types.h"
#include "vl53l5_error_codes.h"
#include "vl53l5_platform_log.h"

#include "vl53l5_api_core.h"
#include "vl53l5_api_power.h"
#include "vl53l5_api_ranging.h"
#include "vl53l5_api_range_decode.h"

#include "vl53l5_platform_init.h"
#include "vl53l5_platform.h"

#include "vl53l5_fw_data.h"
#include "vl53l5_customer_config_data.h"
#include "vl53l5_customer_calibration_data.h"

#include "vl53l5_core_map_bh.h"
#include "vl53l5_calibration_map_bh.h"

#include "vl53l5_api_calibration_decode.h"

#include "calibration_data.h"

/* Private macro -------------------------------------------------------------*/

/*************** Global Data ******************/
////////////////////////////////////////////////////////////////

/* Private variables ---------------------------------------------------------*/
static float tof_range_data = 0;
static int32_t range_sum = 0;
static stmdev_ctx_t tof_dev_ctx;
static platform_prams *tof_init_parms;
static uint8_t sensor_activate_count = 0;
static bool tof_activated = false;

struct vl53l5_dev_handle_t dev = { 0 };

static float last_tof_range_data = 0.0;
static tof_cali_data mtof_cali_data = { 0 };

/* Extern variables ----------------------------------------------------------*/

/*
 * In this example, the firmware buffer is assigned using a #define of a full
 * buffer. In a real example, this may be stripped from file or copied from a
 * pointer to a known location e.g. flash storage.
 * NOTE: Implement as appropriate for current platform to run examples
 */
#define MAX_FW_FILE_SIZE 21510*4        // CHECK THIS!!!
static const uint8_t _fw_buffer[MAX_FW_FILE_SIZE] = EWOKMZ_STXP70_TCPM_RAM_FULL;
uint32_t _fw_buff_count = sizeof(_fw_buffer);

/*
 * Max count for general parameter buffer. This is set very large in this
 * example and could be easily optimised per use-case.
 */
#define PARAMETER_BUFF_MAX_COUNT 2048

/*
 * A comms buffer must be provided for all driver -> device interactions. The
 * default size is given by VL53L5_COMMS_BUFFER_SIZE_BYTES. However, this can be
 * dynamically altered and user-specified through the max_count parameter.
 */
uint8_t _comms_buffer[VL53L5_COMMS_BUFFER_SIZE_BYTES] = { 0 };

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
#define CAL_BLOCK_HEADERS_XTALK_SHAPE_CALIBRATION { \
    VL53L5_MAP_VERSION_BH, \
    VL53L5_PXTALK_SHAPE_META_BH, \
    VL53L5_PXTALK_SHAPE_DATA_CAL__XTALK_SHAPE__BIN_DATA_BH}

#define CAL_BLOCK_HEADERS_XTALK_GRID_CALIBRATION { \
    VL53L5_MAP_VERSION_BH, \
    VL53L5_PXTALK_GRID_META_BH, \
    VL53L5_PXTALK_PHASE_STATS_BH, \
    VL53L5_PXTALK_TEMPERATURE_STATS_BH, \
    VL53L5_PXTALK_GRID_RATE_CAL__GRID_DATA__RATE_KCPS_PER_SPAD_BH, \
    VL53L5_PXTALK_GRID_SPADS_CAL__GRID_DATA_EFFECTIVE_SPAD_COUNT_BH, \
    VL53L5_PXTALK_ERROR_STATUS_BH, \
    VL53L5_PXTALK_WARNING_STATUS_BH, \
    VL53L5_PXTALK_MON_META_BH, \
    VL53L5_PXTALK_MON_ZONES_CAL__XMON__ZONE__X_OFF_BH, \
    VL53L5_PXTALK_MON_ZONES_CAL__XMON__ZONE__Y_OFF_BH, \
    VL53L5_PXTALK_MON_ZONES_CAL__XMON__ZONE__WIDTH_BH, \
    VL53L5_PXTALK_MON_ZONES_CAL__XMON__ZONE__HEIGHT_BH, \
    VL53L5_PXTALK_MON_DATA_CAL__XMON__ZONE__RATE_KCPS_SPAD_BH, \
    VL53L5_PXTALK_MON_DATA_CAL__XMON__ZONE__AVG_COUNT_BH}

uint32_t _cal_data_shape_block_list[] =
    CAL_BLOCK_HEADERS_XTALK_SHAPE_CALIBRATION;
uint32_t _num_cal_shape_blocks =
    sizeof(_cal_data_shape_block_list) / sizeof(uint32_t);

uint32_t _cal_data_grid_block_list[] = CAL_BLOCK_HEADERS_XTALK_GRID_CALIBRATION;
uint32_t _num_cal_grid_blocks =
    sizeof(_cal_data_grid_block_list) / sizeof(uint32_t);

uint32_t *cal_results_bh;

FILE *fptr;

int32_t send_device_xtalk_characterisation_config(struct vl53l5_dev_handle_t
                                                  *pdev)
{
  int32_t status = STATUS_OK;
  uint8_t params_buff[] = VL53L5_CFG__XTALK_GEN1_1000__8X8_DATA;

  printf("Setting config for xtalk characterisation...\r\n");

  status = vl53l5_set_device_parameters(pdev, params_buff, sizeof(params_buff));

  return status;
}

int32_t calibrating_loop(struct vl53l5_dev_handle_t * pdev)
{
  int32_t status = STATUS_OK;
  uint8_t n = 0;
  int cal_complete = 0;

  printf("asking device to start ranging...\r\n");

  status = vl53l5_start(pdev, NULL);
  if (status < STATUS_OK) {
    printf("Start failed: %d\r\n", (int)status);
    goto exit;
  }

  while (cal_complete == 0) {
    /* Simulate a wait between subsequent checks of range results */
    vl53l5_wait_ms(pdev, 100);

    /* now check to see if range data is available */
    status = vl53l5_check_data_ready(pdev);
    if (status < STATUS_OK) {
      if (status == VL53L5_NO_NEW_RANGE_DATA_ERROR) {
        printf("waiting for new range data...\r\n");
        continue;
      }
      printf("check_data_ready() error: status %d\r\n", (int)status);
      goto exit;
    } else {
      cal_complete = 1;
    }
    goto exit;
  }

exit:
  printf("asking device to stop ranging...\r\n");
  status = vl53l5_stop(pdev, NULL);

  if (status < STATUS_OK) {
    printf("Stop command failed: status %d\r\n", (int)status);
  }

  return status;
}

int save_cal_data(struct vl53l5_dev_handle_t *pdev)
{
  int i, j;
  int32_t status = STATUS_OK;
  struct vl53l5_calibration_data_t cal_data = { 0 };

// shape cali data
  status =
      vl53l5_get_device_parameters(pdev, _cal_data_shape_block_list,
                                   _num_cal_shape_blocks);
  if (status < STATUS_OK) {
    printf("Getting calibration data failed: %d\r\n", (int)status);
    goto exit;
  } else
    printf("Calibration data received \r\n");

  /* The data is available in the comms buffer. How the data is then
     stored is customer specific.  The last 4 bytes of data are not required
     and should be removed [00, 00, 00, 0f].
   */
  for (i = 3; i < VL53L5_COMMS_BUFF_COUNT(pdev) - 4; i += 4) {
    printf(fptr, "%02x, %02x, %02x, %02x \n", VL53L5_COMMS_BUFF(pdev)[i - 3],
           VL53L5_COMMS_BUFF(pdev)[i - 2],
           VL53L5_COMMS_BUFF(pdev)[i - 1], VL53L5_COMMS_BUFF(pdev)[i]);
    mtof_cali_data.shape_cali_data[i - 3] = VL53L5_COMMS_BUFF(pdev)[i - 3];
    mtof_cali_data.shape_cali_data[i - 2] = VL53L5_COMMS_BUFF(pdev)[i - 2];
    mtof_cali_data.shape_cali_data[i - 1] = VL53L5_COMMS_BUFF(pdev)[i - 1];
    mtof_cali_data.shape_cali_data[i] = VL53L5_COMMS_BUFF(pdev)[i];
  }

  printf("Decoding calibration data \r\n");
  status = vl53l5_decode_calibration_data(pdev,
                                          &cal_data,
                                          VL53L5_COMMS_BUFF(pdev),
                                          VL53L5_COMMS_BUFF_COUNT(pdev));

  if (status != STATUS_OK) {
    printf("Decode calibration failed: %d\n", status);
    goto exit;
  }

    /****************************************************************************************
    **
    ** The next section prints ALL the calibration data but in reality you are only interested
    ** in the xtalk grid rates to define CG xtalk values
    **
    ****************************************************************************************/

    /****************************************************************************************
    **
    ** Xtalk data
    **
    ****************************************************************************************/

  printf("Xtalk cal data \r\n");
  printf("\r\n");

  printf("decoded cal__xtalk_cal_shape_median_phase = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__median_phase);
  printf("decoded cal__xtalk_cal_shape_avg_count = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__avg_count);
  printf("decoded cal__xtalk_cal_shape_no_of_bins = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__no_of_bins);
  printf("decoded cal__xtalk_cal_shape_normalisation_power = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__normalisation_power);
  printf("decoded cal__xtalk_cal_shape_silicon_temp_degC = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__silicon_temp_degc);
  printf("decoded cal__xtalk_cal_shape_spare_0 = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__spare_0);
  printf("decoded cal__xtalk_cal_shape_spare_1 = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__spare_1);
  printf("decoded cal__xtalk_cal_shape_spare_2 = %4d  \r\n",
         cal_data.core.pxtalk_shape_meta.cal__xtalk_shape__spare_2);
  printf("\r\n");

  for (j = 0; j < 144; j++) {
    printf("decoded data xtalk shape bin data[ %02d", j);
    printf("] =  %04d  \r\n",
           cal_data.core.pxtalk_shape_data.cal__xtalk_shape__bin_data[j]);
  }
  printf("\r\n");

//   grid cali data
  status =
      vl53l5_get_device_parameters(pdev, _cal_data_grid_block_list,
                                   _num_cal_grid_blocks);
  if (status < STATUS_OK) {
    printf("Getting calibration data failed: %d\r\n", (int)status);
    goto exit;
  } else
    printf("Calibration data received \r\n");

  /* The data is available in the comms buffer. How the data is then
     stored is customer specific.  The last 4 bytes of data are not required
     and should be removed [00, 00, 00, 0f].
   */

  for (i = 3; i < VL53L5_COMMS_BUFF_COUNT(pdev) - 4; i += 4) {
    printf(fptr, "%02x, %02x, %02x, %02x \n", VL53L5_COMMS_BUFF(pdev)[i - 3],
           VL53L5_COMMS_BUFF(pdev)[i - 2],
           VL53L5_COMMS_BUFF(pdev)[i - 1], VL53L5_COMMS_BUFF(pdev)[i]);
    mtof_cali_data.grid_cali_data[i - 3] = VL53L5_COMMS_BUFF(pdev)[i - 3];
    mtof_cali_data.grid_cali_data[i - 2] = VL53L5_COMMS_BUFF(pdev)[i - 2];
    mtof_cali_data.grid_cali_data[i - 1] = VL53L5_COMMS_BUFF(pdev)[i - 1];
    mtof_cali_data.grid_cali_data[i] = VL53L5_COMMS_BUFF(pdev)[i];
  }
  printf("Decoding calibration data \r\n");
  status = vl53l5_decode_calibration_data(pdev,
                                          &cal_data,
                                          VL53L5_COMMS_BUFF(pdev),
                                          VL53L5_COMMS_BUFF_COUNT(pdev));

  if (status != STATUS_OK) {
    printf("Decode calibration failed: %d\n", status);
    goto exit;
  }

  printf("\n----- Calibration xtalk grid -----\n");
  uint32_t max_cross_talk = 0;
  for (j = 0; j < 64; j++) {
    if (cal_data.core.pxtalk_grid_rate.cal__grid_data__rate_kcps_per_spad[j] /
        2048 > max_cross_talk)
      max_cross_talk =
          cal_data.core.pxtalk_grid_rate.cal__grid_data__rate_kcps_per_spad[j] /
          2048;
    printf("%4x,",
           cal_data.core.pxtalk_grid_rate.
           cal__grid_data__rate_kcps_per_spad[j] / 2048);
    if ((j + 1) % 8 == 0)
      printf("\n");
  }
  mtof_cali_data.max_cross_talk = max_cross_talk;
exit:
  return status;
}

int load_cali_to_dev(struct vl53l5_dev_handle_t *pdev)
{
  int32_t status = STATUS_OK;
  if (mtof_cali_data.max_cross_talk != 0) {
    status =
        vl53l5_set_device_parameters(pdev, mtof_cali_data.shape_cali_data, 316);
    if (status < STATUS_OK) {
      printf("Set shape cal data failed %d\n", status);
      return -1;
    }
    status =
        vl53l5_set_device_parameters(pdev, mtof_cali_data.grid_cali_data, 600);
    if (status < STATUS_OK) {
      printf("Set grid cal data failed %d\n", status);
      return -1;
    }
  } else
    printf("mtof_cali_data.max_cross_talk = %u", mtof_cali_data.max_cross_talk);
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */

static int32_t tof_platform_write(void *handle, uint16_t reg, uint8_t * bufp,
                                  uint16_t len)
{
  if (handle == &tof_init_parms->spi_handle) {
    BSP_SPI_16M_Write_Sequence(handle, dev.tof_cs_gpio_port, dev.tof_cs_pin, reg, bufp, len);
  }

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */

static int32_t tof_platform_read(void *handle, uint16_t reg, uint8_t * bufp,
                                 uint16_t len)
{
  if (handle == &tof_init_parms->spi_handle) {
    BSP_SPI_16M_Read_Sequence(handle, dev.tof_cs_gpio_port, dev.tof_cs_pin, reg, bufp, len);
  }

  return 0;
}

static void tof_get_range_data(struct vl53l5_dev_handle_t *pdev)
{
  int32_t status = ranging_loop(pdev);
  if (status < STATUS_OK) {
    printf("ranging loop failed: %d\r\n", (int)status);
  }
}

/*
 * @brief  platform specific initialization (platform dependent)
 */

int tof_init(void *para, void *para2)
{

  tof_init_parms = (platform_prams *) para;
  /* Initialize mems driver interface */
  tof_dev_ctx.write_reg = tof_platform_write;
  tof_dev_ctx.read_reg = tof_platform_read;
  dev.spi_handle = &tof_init_parms->spi_handle;
  dev.timer_handle = sensor_get_timestamp;
  dev.tof_cs_gpio_port = tof_init_parms->tof_cs_gpio_port;
  dev.tof_cs_pin = &tof_init_parms->tof_cs_pin;
  /*power on */
  HAL_GPIO_WritePin(TOF_EN_GPIO_Port, TOF_EN_Pin, GPIO_PIN_SET);
  osDelay(1);
  /*set cs high by default */
  HAL_GPIO_WritePin(TOF_CS_GPIO_Port, TOF_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  //uint8_t status = HAL_GPIO_ReadPin(TOF_CS_GPIO_Port, TOF_CS_Pin);
  //printf("TOF cs status: %u \r\n", status);

  uint8_t current = 0x0;

  printf("TOF current page: %u \r\n", current);
  tof_platform_write(dev.spi_handle, (uint16_t) INITIAL_PAGE_ID,
                     &current, 1);

  /* Check ID */
  uint8_t id_0 = 0, id_1 = 0;
  tof_platform_read(dev.spi_handle, (uint16_t) TOF_DEVICE_ID, &id_0, 1);

  tof_platform_read(dev.spi_handle, (uint16_t) TOF_REVISION_ID, &id_1, 1);

  printf("TOF whoamI: 0x%x 0x%x %p\r\n", id_0, id_1, dev.spi_handle);

  if (id_0 != TOF_DEVICE_ID_VALUE || id_1 != TOF_REVISION_ID_VALUE) {
    printf("init failed for TOF!\r\n");
    return SENSOR_FAILED;
  }
  return SENSOR_SUCCESS;
}

static int32_t tof_enable()
{
  int32_t status = 0;
  if (sensor_activate_count == 0) {
    tof_activated = true;
    int32_t status = STATUS_OK;

    /* Platform specific data: running in SPI mode */
    dev.comms_type = VL53L5_SPI;

    /* set the fw buffer to be loaded into device RAM */
    printf("Setting up fw buffer.\r\n");
    VL53L5_ASSIGN_FW_BUFF(&dev, (uint8_t *) _fw_buffer, _fw_buff_count);

    /* Setup the comms buffer */
    VL53L5_ASSIGN_COMMS_BUFF(&dev, _comms_buffer, sizeof(_comms_buffer));

    /* Initialise platform and comms */
    printf("Initialising.\r\n");
    status = vl53l5_platform_init(&dev);
    if (status < STATUS_OK) {
      printf("Platform init failed: %d\r\n", status);
      goto exit;
    }
    // -------------------- set the high power mode ----------------
    // move the device in HP whatever its state is high power
    // already or low power with comms. But in case it has been set in LP
    // at the latest termination stage this call is mandatory
    dev.host_dev.power_state = VL53L5_POWER_STATE_LP_IDLE_COMMS;
    status = vl53l5_set_power_mode(&dev, VL53L5_POWER_STATE_HP_IDLE);
    if (status < STATUS_OK) {
      printf
          ("Could not put the device in High Power. Is it already powered up ?\n");
      goto exit;
    }
    /* Initialise device */
    status = vl53l5_init(&dev);
    if (status < STATUS_OK) {
      printf("Init failed: %d\r\n", status);
      goto exit;
    }
    status = load_cali_to_dev(&dev);
    if (status < STATUS_OK) {
      printf("load calidata to device failed: %d\r\n", status);
      goto exit;
    }

    status = send_device_config(&dev);
    if (status < STATUS_OK) {
      printf("setting config failed: %d\r\n", status);
      goto exit;
    }
    status = vl53l5_start(&dev, NULL);

    if (status < STATUS_OK) {
      printf("Start failed: %d\r\n", status);
      goto exit;
    }
  }
  sensor_activate_count++;

  return SENSOR_SUCCESS;

exit:
  if (tof_activated) {
    if (status < STATUS_OK) {
      status = vl53l5_read_device_error(&dev, status);
      printf("tof enable failed: vl53l5_read_device_error status %d\r\n",
             status);
    }
    status = vl53l5_stop(&dev, NULL);
    if (status < STATUS_OK) {
      printf("tof_enable failed vl53l5_stop failed: status %d\r\n", status);
    }
    vl53l5_term(&dev);
    if (status < STATUS_OK) {
      printf("tof_enable failed  vl53l5_term failed: status %d\r\n", status);
    }
    vl53l5_platform_terminate(&dev);
    if (status < STATUS_OK) {
      printf("tof_enable failed  platform term failed: status %d\r\n", status);
    }
  }

  return status;
}

static int32_t tof_disable()
{
  int32_t status = 0;
  sensor_activate_count--;
  if (sensor_activate_count == 0) {
    tof_activated = false;
    printf("asking device to stop ranging...\r\n");
    status = vl53l5_stop(&dev, NULL);

    if (status < STATUS_OK) {
      printf("Stop command failed: status %d\r\n", status);
    }
    status = vl53l5_set_power_mode(&dev, VL53L5_POWER_STATE_LP_IDLE_COMMS);
    if (status < STATUS_OK) {
      printf("set device in low power idle fails : %d\n", status);
    }
    status = vl53l5_term(&dev);
    if (status < STATUS_OK) {
      printf("vl53l5_term fails : %d\n", status);
    }

    status = vl53l5_platform_terminate(&dev);
    if (status < STATUS_OK) {
      printf("vl53l5_platform_terminate fails : %d\n", status);
    }
  }

  return status;
}

int tof_activate(bool activate)
{
  int res = 0;
  printf("tof_activate: %d \r\n", activate);
  if (activate) {
    res = tof_enable();
  } else {
    res = tof_disable();
  }
  return res;
}

/* Main Example --------------------------------------------------------------*/
int tof_publish_sensor_data(void *para)
{
  /* Read samples in polling mode (no int) */
  sensors_event_t sensor_data = { 0 };
  //printf("tof_publish_sensor_data: %d \r\n", tof_activated);
  if (tof_activated) {
    // core process
    tof_get_range_data(&dev);
    //send related messages to SensorDataQ
    sensor_data.sensor_type = SENSOR_TYPE_PROXIMITY_BOT;
    sensor_data.accuracy = 3;
    sensor_data.timestamp = sensor_get_timestamp();
    sensor_data.sensor_data_t.vec.data[0] = tof_range_data;
    sensor_data.sensor_data_t.vec.data[1] =
        (float)mtof_cali_data.max_cross_talk;
    osMessageQueuePut(tof_init_parms->SensorDataQHandle, &sensor_data, 0, 0);
  }
  return 0;
}

int tof_calibration()
{
  int status = 0;

  /* Platform specific data: running in SPI mode */
  dev.comms_type = VL53L5_SPI;

  /* set the fw buffer to be loaded into device RAM */
  printf("Setting up fw buffer.\r\n");
  VL53L5_ASSIGN_FW_BUFF(&dev, (uint8_t *) _fw_buffer, _fw_buff_count);

  /* Setup the comms buffer */
  VL53L5_ASSIGN_COMMS_BUFF(&dev, _comms_buffer, sizeof(_comms_buffer));

  /* Initialise platform and comms */
  printf("Initialising.\r\n");
  status = vl53l5_platform_init(&dev);
  if (status < STATUS_OK) {
    printf("Platform init failed: %d\r\n", (int)status);
    goto exit;
  }

  /* Initialise device */
  status = vl53l5_init(&dev);
  if (status < STATUS_OK) {
    printf("Init failed: %d\r\n", (int)status);
    goto exit;
  }

  /*reset device to clear memory */
  status = vl53l5_set_power_mode(&dev, VL53L5_POWER_STATE_ULP_IDLE);
  if (status < STATUS_OK) {
    printf("set_power_mode() failed: %d\r\n", (int)status);
    goto exit;
  }

  status = vl53l5_set_power_mode(&dev, VL53L5_POWER_STATE_HP_IDLE);
  if (status < STATUS_OK) {
    printf("set_power_mode() failed: %d\r\n", (int)status);
    goto exit;
  }

  status = send_device_xtalk_characterisation_config(&dev);
  if (status < STATUS_OK) {
    printf("setting config failed: %d\r\n", (int)status);
    goto exit;
  }

  status = calibrating_loop(&dev);
  if (status < STATUS_OK) {
    printf("ranging loop failed: %d\r\n", (int)status);
    goto exit;
  }

  status = save_cal_data(&dev);
  if (status < STATUS_OK) {
    printf("saving cal data failed: %d\r\n", (int)status);
    goto exit;
  }

exit:
  /*
   * If an error has occurred, run the error handler. In certain cases,
   * this error handler will return a more detailed error code due to
   * the failure occurring within device firmware. Otherwise, the original
   * status will be returned.
   */
  if (status < STATUS_OK) {
    status = vl53l5_read_device_error(&dev, status);
  }
  /* end session */
  printf("Example complete with status %d... terminating.\r\n", (int)status);

  vl53l5_term(&dev);
  if (status < STATUS_OK) {
    printf("saving cal data failed: %d\r\n", (int)status);
  }
  vl53l5_platform_terminate(&dev);
  if (status < STATUS_OK) {
    printf("saving cal data failed: %d\r\n", (int)status);
  }
  return status;
}

int tof_init_complete(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

int tof_config(uint8_t config_type, void *para)
{
  //send init complete event to smgr?
  int res = SENSOR_SUCCESS;
  uint8_t id_0 = 0, id_1 = 0;
  sensors_event_t sensor_data = { 0 };

  switch (config_type) {
  case SENSOR_CONFIG_SELFTEST:
    printf("VL53L5 selftest message \r\n");
    tof_platform_read(dev.spi_handle, (uint16_t) TOF_DEVICE_ID, &id_0, 1);

    tof_platform_read(dev.spi_handle, (uint16_t) TOF_REVISION_ID, &id_1, 1);

    //printf("TOF whoamI: 0x%x 0x%x %p\r\n", id_0, id_1,
    //        tof_dev_ctx.handle);

    if (id_0 != TOF_DEVICE_ID_VALUE || id_1 != TOF_REVISION_ID_VALUE) {
      printf("selftest failed for VL53L5!\r\n");
      res = SENSOR_FAILED;
    }
    printf("selftest success for VL53L5!\r\n");
    break;
  case SENSOR_CONFIG_CALIBRATION:
    res = tof_calibration();
    if (res == 0) {
      memcpy(&tof_init_parms->board_calidata->tof_cali_t, &mtof_cali_data,
             sizeof(tof_cali_data));
      if (tof_init_parms->board_calidata->tof_cali_ver != DEFAULT_VERSION)
        tof_init_parms->board_calidata->tof_cali_ver++;
      else
        tof_init_parms->board_calidata->tof_cali_ver = 1;
    }
    break;
  case SENSOR_CONFIG_BIAS:
    //store calibrate sensor data to driver when boot up
    if (tof_init_parms->board_calidata->tof_cali_ver != DEFAULT_VERSION) {
      memcpy(&mtof_cali_data, &tof_init_parms->board_calidata->tof_cali_t,
             sizeof(tof_cali_data));
    }
    break;
  case SENSOR_CALIBRATION_RESULT:
    sensor_data.sensor_type = SENSOR_TYPE_PROXIMITY_BOT;
    sensor_data.accuracy = 0xFF;
    sensor_data.timestamp = sensor_get_timestamp();
    sensor_data.sensor_data_t.vec.data[0] =
        (float)tof_init_parms->board_calidata->tof_cali_t.max_cross_talk;
    sensor_data.sensor_data_t.vec.data[1] =
        (float)tof_init_parms->board_calidata->tof_cali_ver;
    sensor_data.sensor_data_t.vec.data[2] = 0;
    sensor_data.sensor_data_t.vec.data[3] = 0;
    osMessageQueuePut(tof_init_parms->SensorDataQHandle, &sensor_data, 0, 0);
    break;
  default:
    break;
  }
  return res;
}

int tof_publish_config_resp(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

int32_t send_device_config(struct vl53l5_dev_handle_t * pdev)
{
  int32_t status = STATUS_OK;
  uint8_t params_buff[] = VL53L5_CFG__BACK_TO_BACK__4X4__15HZ_DATA;

  printf("Setting config for 4x4 @ 15Hz...\r\n");

  status = vl53l5_set_device_parameters(pdev, params_buff, sizeof(params_buff));

  return status;
}

int32_t ranging_loop(struct vl53l5_dev_handle_t * pdev)
{
  int32_t status = STATUS_OK;
  struct vl53l5_range_data_t range = { 0 };
  range_sum = 0;
  int valid_range_num = 0;
  int64_t valid_range_peak_rate_sum = 0;
  int64_t valid_range[16] = {0};
  int64_t valid_range_peak_rate[16] = {0};

  if (!tof_activated) {
    printf("tof is not enabled in range loop check, return");
    return -1;
  }
  do {
    /* now check to see if range data is available *///swl check < ok
    status = vl53l5_check_data_ready(pdev);
    if (status == VL53L5_NO_NEW_RANGE_DATA_ERROR) {
      //printf("waiting for new range data...\r\n");
      vl53l5_wait_ms(pdev, 25); //may be not proper
    } else
      break;
  } while (tof_activated);
  if (!tof_activated) {
    printf("tof is not enabled after check data ready, return");
    return -1;
  }
  /* when new data is ready, get the range data from the device */
  status = vl53l5_get_range_data(pdev);
  if (status < STATUS_OK) {
    printf("get_range_data() error: status %d\r\n", (int)status);
  }

  /* Decode range data in comms buffer (OPTIONAL) */
  status = vl53l5_decode_range_data(pdev, &range);
  if (status < STATUS_OK) {
    printf("decode_range_data() failed %d\r\n", (int)status);
  } else {
    //printf("decode_range_data() succeeded\r\n");

    //printf("common data: \r\n");
    //printf("max targets per zone  %d\r\n", (int)range.core.common_data.rng__max_targets_per_zone);
    //printf("no. of zones  %d\r\n", (int)range.core.common_data.rng__no_of_zones);
    //printf("wrap dmax  %2d\r\n", (int)range.core.common_data.wrap_dmax_mm);

    //printf("meta data: \r\n");
    //printf("buffer_ID  %d\r\n", (int)range.core.meta_data.buffer_id);
    //printf("device status  %d\r\n", (int)range.core.meta_data.device_status);
    //printf("silicon tempC  %d\r\n", (int)range.core.meta_data.silicon_temp_degc);
    //printf("stream_count  %d\r\n", (int)range.core.meta_data.stream_count);
    //printf("timestamp  %d\r\n", (int)range.core.meta_data.time_stamp);

    //printf("per zone data: \r\n");
    //printf("amb_dmax         amb_rate        rng_no. targets \r\n");
/*
        for(int x = 0; x < 4; x++){                 // Warning : Print for 4x4 ranging - this line controls the rows
            for(int y = 0; y < 4 ;y++){             //this line controls the columns
                uint32_t idy = (y + x * 4);

                printf("%6d,%4d,%2d", (int)range.core.per_zone_results.amb_dmax_mm[idy],
                                  (int)range.core.per_zone_results.amb_rate_kcps_per_spad[idy]/2048,  //data format is 19.11
                                  (int)range.core.per_zone_results.rng__no_of_targets[idy]);

            }

            //printf(" \r\n");
        }
*/
    //printf("per target data - 1st target: \r\n");
    //printf("status  median   peak_rate     \r\n");

    for (int i = 0; i < 4; i++) {       // Warning : Print for 4x4 ranging - this line controls the rows
      for (int j = 0; j < 8; j = j + 2) {       //this line controls the columns
        uint32_t idx = (j + i * 8);     //This line gets every second value - 2 targets - byte 0, 2, 4,...30
        //Z0T0, Z1T0, Z2T0, Z3T0
        //Z4T0, Z5T0, Z6T0, Z7T0
        //Z8T0, Z9T0, Z10T0, Z11T0
        //Z12T0, Z13T0, Z14T0, Z15T0
        if ((int)range.core.per_tgt_results.target_status[idx] == 5 || (int)range.core.per_tgt_results.target_status[idx] == 9) {
          if (range.core.per_tgt_results.median_range_mm[idx] / 4 > 0) {
            valid_range[valid_range_num] = range.core.per_tgt_results.median_range_mm[idx] / 4;
            valid_range_peak_rate[valid_range_num] = (int)range.core.per_tgt_results.peak_rate_kcps_per_spad[idx] / 2048;
            valid_range_num++;
          }
        }
        /*printf(" %4d,%4d,%4d",
           (int)range.core.per_tgt_results.target_status[idx],
           (int)range.core.per_tgt_results.median_range_mm[idx]/4, //);        //divide by 4-due to 14.2 data format for non fractional parts
           (int)range.core.per_tgt_results.peak_rate_kcps_per_spad[idx]/2048); //divide by 2048 due to 19.11 data format */
      }
      //printf(" \r\n");
    }
  }

  if (valid_range_num != 0) {
    for (int i = 0; i < valid_range_num; i++) {
      if (valid_range_peak_rate[i] != 0)
        valid_range_peak_rate_sum += valid_range_peak_rate[i] * valid_range_peak_rate[i];
    }
    tof_range_data = 0.0;
    for (int i = 0; i < valid_range_num; i++)
      tof_range_data += (valid_range_peak_rate[i] * valid_range_peak_rate[i] * 1.0 / valid_range_peak_rate_sum) * valid_range[i];
    last_tof_range_data = tof_range_data;
  } else
    tof_range_data = last_tof_range_data;
  return status;
}

/******************************************************************************
 *
 * COPYRIGHT 2017 ASAHI KASEI MICRODEVICES CORPORATION ("AKM")
 * All Rights Reserved.
 *
 * This software is licensed to you under the Apache License, Version 2.0
 * (http://www.apache.org/licenses/LICENSE-2.0) except for using, copying,
 * modifying, merging, publishing and/or distributing in combination with
 * AKM's Proprietary Software defined below. 
 *
 * "Proprietary Software" means the software and its related documentations
 * which AKM will provide only to those who have entered into the commercial
 * license agreement with AKM separately. If you wish to use, copy, modify,
 * merge, publish and/or distribute this software in combination with AKM's
 * Proprietary Software, you need to request AKM to enter into such agreement
 * and grant commercial license to you.
 *
 ******************************************************************************/

#include "ak09918_reg.h"

#include "sensor_device.h"

#define AKM_CUSTOM_MAG_AXIS_ORDER_X  0
#define AKM_CUSTOM_MAG_AXIS_ORDER_Y  1
#define AKM_CUSTOM_MAG_AXIS_ORDER_Z  2
#define AKM_CUSTOM_MAG_AXIS_SIGN_X   0
#define AKM_CUSTOM_MAG_AXIS_SIGN_Y   0
#define AKM_CUSTOM_MAG_AXIS_SIGN_Z   0

/* Global variable for ASA value. */
static uint8_t g_mag_axis_order[3];
static uint8_t g_mag_axis_sign[3];

static int32_t platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t * bufp,
                             uint16_t len);

static stmdev_ctx_t dev_ctx;
static platform_prams *mag_init_parms;
static uint8_t sensor_activate_count = 0;
static bool mag_activated = false;

void AKS_ConvertCoordinate(int32_t vec[3], const uint8_t axis_order[3],
                           const uint8_t axis_sign[3])
{
  int32_t val32[3];
  uint8_t i;

  /* Axis conversion */
  for (i = 0; i < 3; i++) {
    val32[i] = vec[axis_order[i]];

    if (axis_sign[i]) {
      val32[i] *= -1;
    }
  }

  /* Copy to argument */
  for (i = 0; i < 3; i++) {
    vec[i] = val32[i];
  }
}

/******************************************************************************/
/***** AKS public APIs ********************************************************/
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                              uint16_t len)
{
  if (handle == &mag_init_parms->i2c_handle) {
    //printf("start i2c write, \r\n");
    HAL_I2C_Mem_Write(handle, MAGNETOMETER_SLAVE_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
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
static int32_t platform_read(void *handle, uint8_t reg, uint8_t * bufp,
                             uint16_t len)
{
  if (handle == &mag_init_parms->i2c_handle) {
    //printf("start i2c read, \r\n");
    HAL_I2C_Mem_Read(handle, MAGNETOMETER_SLAVE_ADDR, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }

  return 0;
}

int ak09918_mag_init_complete(void *para)
{
  return AKM_SUCCESS;
}

int16_t ak099xx_start(stmdev_ctx_t * ctx)
{
  int32_t ret = 0;
  if (sensor_activate_count == 0) {
    mag_activated = true;
    /* 50 - 100 Hz */
    ret = ak099xx_set_mode(ctx, AK099XX_MODE_CONT_MEASURE_MODE4);
  }
  sensor_activate_count++;
  return ret;
}

int16_t ak099xx_stop(stmdev_ctx_t * ctx)
{
  int32_t ret = 0;
  sensor_activate_count--;
  if (sensor_activate_count == 0) {
    mag_activated = false;
    /* 50 - 100 Hz */
    ret = ak099xx_set_mode(ctx, AK099XX_MODE_POWER_DOWN);
  }
  return ret;
}

int ak09918_mag_activate(bool activate)
{
  int res = 0;
  printf("ak09918_mag_activate: %d \r\n", activate);
  if (activate) {
    res = ak099xx_start(&dev_ctx);
  } else {
    res = ak099xx_stop(&dev_ctx);
  }
  return res;
}

int ak09918_mag_publish_sensor_data(void *para)
{
  struct AKM_SENSOR_DATA data;
  int fret = AKM_SUCCESS, ready = 0;
  uint8_t num = 1;
  sensors_event_t sensor_data = { 0 };

  ready = ak099xx_check_rdy(&dev_ctx);
  if (ready == 1 && mag_activated) {
    fret = ak099xx_get_data(&dev_ctx, &data, &num);
    AKS_ConvertCoordinate(data.u.v, g_mag_axis_order, g_mag_axis_sign);

    //send related messages to SensorDataQ
    sensor_data.sensor_type = SENSOR_TYPE_MAGNETIC_FIELD;
    sensor_data.accuracy = 3;
    sensor_data.timestamp = sensor_get_timestamp();
    sensor_data.sensor_data_t.vec.data[0] = data.u.v[0] / 65535.0f;     //convert Q16 to uT
    sensor_data.sensor_data_t.vec.data[1] = data.u.v[1] / 65535.0f;
    sensor_data.sensor_data_t.vec.data[2] = data.u.v[2] / 65535.0f;
    osMessageQueuePut(mag_init_parms->SensorDataQHandle, &sensor_data, 0, 0);
  }

  return fret;
}

int ak09918_mag_init(void *para1, void *para2)
{
  mag_init_parms = (platform_prams *) para1;
  int fret;
  uint16_t WhoAmI = 0;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &mag_init_parms->i2c_handle;

  //call who I am function here to check if HW pressent
  fret = ak099xx_get_WhoAmI(&dev_ctx, &WhoAmI);
  if (fret != AKM_SUCCESS) {
    return fret;
  }

  printf("ak09918_mag_init WhoAmI: 0x%x\r\n", WhoAmI);
  if (WhoAmI != AK09918_WIA_VAL) {
    printf("ak09918_mag_init failed!\r\n");
    return AKM_ERROR;
  }

  fret = ak099xx_soft_reset(&dev_ctx);
  if (fret != AKM_SUCCESS) {
    return fret;
  }

  /* axis conversion parameter */
  g_mag_axis_order[0] = AKM_CUSTOM_MAG_AXIS_ORDER_X;
  g_mag_axis_order[1] = AKM_CUSTOM_MAG_AXIS_ORDER_Y;
  g_mag_axis_order[2] = AKM_CUSTOM_MAG_AXIS_ORDER_Z;
  g_mag_axis_sign[0] = AKM_CUSTOM_MAG_AXIS_SIGN_X;
  g_mag_axis_sign[1] = AKM_CUSTOM_MAG_AXIS_SIGN_Y;
  g_mag_axis_sign[2] = AKM_CUSTOM_MAG_AXIS_SIGN_Z;

  return AKM_SUCCESS;
}

int ak09918_mag_config(uint8_t config_type, void *para)
{
  int res = 0;
  switch (config_type) {
  case SENSOR_CONFIG_SELFTEST:
    printf("mag selftest message \r\n");
    res = ak09918_self_test(&dev_ctx);
    break;
  default:
    break;
  }
  return res;
}

int ak09918_mag_publish_config_resp(void *para)
{
  return AKM_SUCCESS;
}

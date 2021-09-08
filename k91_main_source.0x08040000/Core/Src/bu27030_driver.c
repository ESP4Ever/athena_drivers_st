/*
 * bu27030_driver.c
 *
 *  Created on: Nov 28, 2020
 *      Author: sangweilin@xiaomi.com
 */
#include <string.h>
#include "bu27030_reg.h"
#include "sensor_device.h"

/* Private macro -------------------------------------------------------------*/

/*************** Global Data ******************/
////////////////////////////////////////////////////////////////
#define     COEFFICIENT_SIZE               (19)
#define		NUM_FOR_CALI_SCALE			(10)

const float data_coefficient[COEFFICIENT_SIZE] = {
  0.29,
  0.001646, -0.000253, -0.29, 0.0,
  0.35,
  0.001646, -0.000253, -0.29, 5.833,
  0.40,
  0.001646, -0.00253, -0.285, -10.0,
  0.001646, -0.00253, -0.294, -1.417
};

static const uint8_t time_table[] = { 100, 50 };

static const uint16_t gain_table[] = { 0,       // 0
  0,                            // 1
  BU27030_1X,                   // 2
  0,                            // 3
  0,                            // 4
  0,                            // 5
  0,                            // 6
  0,                            // 7
  0,                            // 8
  0,                            // 9
  BU27030_32X,                  // 10
  0,                            // 11
  BU27030_256X,                 // 12
  0,                            // 13
  0,                            // 14
  0                             // 15
};

/* Private variables ---------------------------------------------------------*/
static float light_data;
static uint8_t whoamI;
static stmdev_ctx_t light_dev_ctx;
static platform_prams *light_init_parms;
static uint8_t sensor_activate_count = 0;
static bool light_activated = false;

static float light_scale = 1.0f;

static int skip_num = 5;

static float target_lux = 200.0f;

static READ_DATA_ARG data = { 0 };

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */

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
static int32_t light_platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                                    uint16_t len)
{
  if (handle == &light_init_parms->i2c_handle) {
    //printf("start i2c write, \r\n");
    HAL_I2C_Mem_Write(handle, (uint16_t) BU27030_I2C_COMUNICATE_ADD, reg,
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
static int32_t light_platform_read(void *handle, uint8_t reg, uint8_t * bufp,
                                   uint16_t len)
{
  if (handle == &light_init_parms->i2c_handle) {
    //printf("start i2c read, \r\n");
    HAL_I2C_Mem_Read(handle, (uint16_t) BU27030_I2C_COMUNICATE_ADD, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }

  return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/******************************************************************************
 * NAME       : bu27030_driver_reset
 * FUNCTION   : reset BU27030 register
 * REMARKS    :
 *****************************************************************************/
static int bu27030_driver_reset(stmdev_ctx_t * ctx)
{
  int result = 0;
  /* set soft reset */
  uint8_t BU27030_RST = (1 << 7);
  result |= light_platform_write(ctx->handle, BU27030_REG_SOFT_RST,
                                 &BU27030_RST, 1);
  return (result);
}

/*************** Initialze Functions ******************/
static int bu27030_init_client(stmdev_ctx_t * ctx)
{
  int result = 0;

  /* execute software reset */
  result = bu27030_driver_reset(ctx);
  if (result != 0) {
    return (result);
  }
  uint8_t MEASURE_100MS = 0;    //100ms
  uint8_t MEASURE_DEFAULT_GAIN = (0x02 << 4) | (0x02);

  result = light_platform_write(ctx->handle, BU27030_REG_TIMING,
                                &MEASURE_100MS, 1);
  if (result == 0) {
    //gain0 gain1 : 1X, 1X
    result = light_platform_write(ctx->handle, BU27030_REG_GAIN,
                                  &MEASURE_DEFAULT_GAIN, 1);
  } else {
    printf(" I2c write failed! \r\n");
  }

  return (result);
}

/**
 * @Brief: bu27030_calculate_light Calculate lux base on rgbc
 *
 * @Param: data RGBC value from sensor
 *
 * @Returns: lux value or failed.
 */
static int bu27030_calculate_light(READ_DATA_ARG data, unsigned char gain,
                                   unsigned char time)
{
  float lx, tmp1, tmp2;
  CALC_DATA calculated_data;

  /* set the value of measured als data */
  calculated_data.als_time = time_table[time & 0x1];
  calculated_data.gain0 = gain_table[gain >> 4];
  calculated_data.gain1 = gain_table[gain & DATA1_GAIN_MASK];
  calculated_data.als_data0 = data.data0;
  calculated_data.als_data1 = data.data1;

  if (data.data0 == 0xFFFF) {
    printf("Data0 is 0xFFFF, return max lux 65535.\r\n");
    return 65535;
  }

  if (!calculated_data.als_time || !calculated_data.gain0
      || !calculated_data.gain1) {
    printf("parameter error, als_time:%d, gain0:%d, gain1:%d",
           calculated_data.als_time, calculated_data.gain0,
           calculated_data.gain1);
    return 0;
  }

  calculated_data.als_data0 = data.data0 * DATA_TRANSFER_COFF
      / calculated_data.als_time / calculated_data.gain0;
  calculated_data.als_data1 = data.data1 * DATA_TRANSFER_COFF
      / calculated_data.als_time / calculated_data.gain1;

  if (calculated_data.als_data1 <
      calculated_data.als_data0 * data_coefficient[0]) {
    tmp1 =
        data_coefficient[1] * calculated_data.als_data0 +
        data_coefficient[2] * calculated_data.als_data1;
    tmp2 =
        ((calculated_data.als_data1 / calculated_data.als_data0 -
          data_coefficient[3]) * data_coefficient[4] + 1.0);
  } else if (calculated_data.als_data1 <
             calculated_data.als_data0 * data_coefficient[5]) {
    tmp1 =
        data_coefficient[6] * calculated_data.als_data0 +
        data_coefficient[7] * calculated_data.als_data1;
    tmp2 =
        ((calculated_data.als_data1 / calculated_data.als_data0 -
          data_coefficient[8]) * data_coefficient[9] + 1.0);
  } else if (calculated_data.als_data1 <
             calculated_data.als_data0 * data_coefficient[10]) {
    tmp1 =
        data_coefficient[11] * calculated_data.als_data0 +
        data_coefficient[12] * calculated_data.als_data1;
    tmp2 =
        ((calculated_data.als_data1 / calculated_data.als_data0 -
          data_coefficient[13]) * data_coefficient[14] + 1.0);
  } else {
    tmp1 =
        data_coefficient[15] * calculated_data.als_data0 +
        data_coefficient[16] * calculated_data.als_data1;
    tmp2 =
        ((calculated_data.als_data1 / calculated_data.als_data0 -
          data_coefficient[17]) * data_coefficient[18] + 1.0);
  }

  lx = tmp1 * tmp2;

  if (lx < 0) {
    lx = 0;
    printf("lx is minus, error!!!\r\n");
  }

  printf("lux:%ld, data0=%d, data1=%d, gain0=%d, gain1=%d, als_time:%d\r\n",
         lx, data.data0, data.data1, calculated_data.gain0,
         calculated_data.gain1, calculated_data.als_time);

  return (int)(lx);
}

int bu27030_auto_change_gain0(stmdev_ctx_t * ctx, unsigned int data0)
{
  unsigned char target_gain = 0;
  unsigned short curret_gain0 = 0;
  uint8_t buffer;

  int gain_changed = 0;

  if (!ctx) {
    printf("Parameter error !!! \r\n");
    return gain_changed;
  }
  //get gain reg_value
  light_platform_read(ctx->handle, BU27030_REG_GAIN, &buffer, 1);
  if (buffer < 0) {
    printf("Read data from IC error.\r\n");
    return gain_changed;
  }
  //gain0
  curret_gain0 = gain_table[buffer >> 4];
  if (data0 > BU27030_SATURATION_THRESH) {
    if (curret_gain0 > BU27030_32X) {   //current is  256X
      target_gain = DATA0_GAIN_X32 | (buffer & DATA1_GAIN_MASK);
    } else if (curret_gain0 > BU27030_1X) {     //current is 32X
      target_gain = DATA0_GAIN_X1 | (buffer & DATA1_GAIN_MASK);
    }
  } else if (data0 < BU27030_INSUFFICIENCE_THRESH) {
    if (curret_gain0 < BU27030_32X) {   //current is  1X
      target_gain = DATA0_GAIN_X32 | (buffer & DATA1_GAIN_MASK);
    } else if (curret_gain0 < BU27030_256X) {   //current is 32X
      target_gain = DATA0_GAIN_X256 | (buffer & DATA1_GAIN_MASK);
    }
  }

  if (target_gain) {
    gain_changed = 1;
    light_platform_write(ctx->handle, BU27030_REG_GAIN, &target_gain, 1);
    printf("bu27030_auto_change_gain current_gain0=%d, target=%d\r\n",
           curret_gain0, gain_table[target_gain >> 4]);
  }

  return gain_changed;
}

int bu27030_auto_change_gain1(stmdev_ctx_t * ctx, unsigned int data1)
{

  unsigned char target_gain = 0;
  unsigned short curret_gain1 = 0;
  uint8_t buffer;
  int gain_changed = 0;
  if (!ctx) {
    printf("Parameter error !!! \r\n");
    return gain_changed;
  }
  //get gain reg_value
  light_platform_read(ctx->handle, BU27030_REG_GAIN, &buffer, 1);
  if (buffer < 0) {
    printf("Read data from IC error.\r\n");
    return gain_changed;
  }

  curret_gain1 = gain_table[buffer & DATA1_GAIN_MASK];
  if (data1 > BU27030_SATURATION_THRESH) {
    if (curret_gain1 > BU27030_32X) {   //current is  256X
      target_gain = DATA1_GAIN_X32 | (buffer & DATA0_GAIN_MASK);
    } else if (curret_gain1 > BU27030_1X) {     //current is 32X
      target_gain = DATA1_GAIN_X1 | (buffer & DATA0_GAIN_MASK);
    }
  } else if (data1 < BU27030_INSUFFICIENCE_THRESH) {
    if (curret_gain1 < BU27030_32X) {   //current is  1X
      target_gain = DATA1_GAIN_X32 | (buffer & DATA0_GAIN_MASK);
    } else if (curret_gain1 < BU27030_256X) {   //current is 32X
      target_gain = DATA1_GAIN_X256 | (buffer & DATA0_GAIN_MASK);
    }
  }

  if (target_gain) {
    gain_changed = 1;
    light_platform_write(ctx->handle, BU27030_REG_GAIN, &target_gain, 1);
    printf("bu27030_auto_change_gain1 current_gain1=%d, target=%d\r\n",
           curret_gain1, gain_table[target_gain & DATA1_GAIN_MASK]);
  }

  return gain_changed;
}

/* ALS polling routine */
static void bu27030_get_raw_data_and_calculated_lux_without_scale(stmdev_ctx_t *
                                                                  ctx)
{
  uint8_t tmp = 0;
  int gain_changed = 0;

  //get valid from BU27030_REG_CONTROL(0x43)
  light_platform_read(ctx->handle, BU27030_REG_CONTROL, &tmp, 1);
  if (tmp < 0) {
    printf("Read data from IC error.\r\n");
    return;
  }
  uint8_t POWER_ON = 1;
  if (0 == (tmp & POWER_ON)) {
    printf(" ic is abnormal, re-initialize, and re-enable \r\n");
    bu27030_init_client(ctx);
    light_platform_write(ctx->handle, BU27030_REG_CONTROL, &POWER_ON, 1);
  }
  //BU27030_WARNING("Data valid BU27030_REG_CONTROL(0x%x) = 0x%x\n", BU27030_REG_CONTROL, result);
  if ((tmp & ALS_VALID_HIGH) == 0) {    //not valid
    printf("Data Not valid. But it does not matter, please ignore it.\r\n");
  } else {
    unsigned char gain = 0;
    unsigned char time = 0;

    //read data0
    uint8_t data0_0, data0_1;
    light_platform_read(ctx->handle, BU27030_REG_DATA0_0, &data0_0, 1);
    light_platform_read(ctx->handle, BU27030_REG_DATA0_1, &data0_1, 1);
    uint16_t tmp_data = (uint16_t) data0_1;
    tmp_data = (tmp_data << 8) | data0_0;

    if (tmp_data < 0) {
      printf("%s: i2c read data0 fail.\r\n", __func__);
      return;
    }
    data.data0 = (unsigned int)tmp_data;

    //read data1
    uint8_t data1_0, data1_1;
    light_platform_read(ctx->handle, BU27030_REG_DATA1_0, &data1_0, 1);
    light_platform_read(ctx->handle, BU27030_REG_DATA1_1, &data1_1, 1);
    tmp_data = (uint16_t) data1_1;
    tmp_data = (tmp_data << 8) | data1_0;

    if (tmp_data < 0) {
      printf("%s: i2c read data1 fail.\r\n", __func__);
      return;
    }
    data.data1 = (unsigned int)tmp_data;

    //read gain
    light_platform_read(ctx->handle, BU27030_REG_GAIN, &tmp, 1);
    if (tmp < 0) {
      printf("%s: i2c read gain fail.\r\n", __func__);
      return;
    }
    gain = (unsigned char)tmp;

#if AGC_SUPPORT
    //auto change gain
    //Be noted: if agc was enabled, you must make sure that agc can not be happened on the first time.
    //Then you should set the default value carefullly in bu27030_init_client() to avoid data overflow on the first time.
    gain_changed = bu27030_auto_change_gain0(ctx, data.data0);
    gain_changed |= bu27030_auto_change_gain1(ctx, data.data1);
    if (gain_changed) {
      printf("%s: gain changed return.\r\n", __func__);
      return;
    }
#endif

    //read time
    light_platform_read(ctx->handle, BU27030_REG_TIMING, &tmp, 1);
    if (tmp < 0) {
      printf("%s: i2c read time fail.\r\n", __func__);
      return;
    }

    time = (unsigned char)tmp;
    light_data = (float)bu27030_calculate_light(data, gain, time);
  }
}

static int bu27030_light_calibration(stmdev_ctx_t * ctx, float *lux_scale)
{
  int cali_loop_count = 0;
  float raw_data_for_cali[NUM_FOR_CALI_SCALE];
  memset(raw_data_for_cali, 0, sizeof(raw_data_for_cali));
  do {
    bu27030_get_raw_data_and_calculated_lux_without_scale(&light_dev_ctx);
    cali_loop_count++;
    platform_delay(100);        //mesasure_time is 100ms one time
    if (cali_loop_count > skip_num) {
      raw_data_for_cali[cali_loop_count - skip_num - 1] = light_data;   // need consider again or atime changed
    }
  } while (cali_loop_count - skip_num <= NUM_FOR_CALI_SCALE);

  float sumed_raw_data = 0;
  for (int i = 0; i < NUM_FOR_CALI_SCALE; i++) {
    sumed_raw_data += raw_data_for_cali[i];
  }

  float average = sumed_raw_data / NUM_FOR_CALI_SCALE;
  float var = 0;
  for (int j = 0; j <= NUM_FOR_CALI_SCALE; j++) {
    var += pow(raw_data_for_cali[j] - average, 2) / NUM_FOR_CALI_SCALE; //var
  }

  //float standard = pow(var,0.5);//std

  if (var > 100 || average < 100) {
    printf("data does not meet requiement!");
    return -1;
  }
  *lux_scale = target_lux / average;
  return 0;
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
int bu27030_light_init(void *para, void *para2)
{
  light_init_parms = (platform_prams *) para;
  /* Initialize mems driver interface */
  light_dev_ctx.write_reg = light_platform_write;
  light_dev_ctx.read_reg = light_platform_read;
  light_dev_ctx.handle = &light_init_parms->i2c_handle;

  /* Check device ID */
  light_platform_read(light_dev_ctx.handle, (uint8_t) BU27030_REG_PART_ID,
                      &whoamI, 1);

  printf("bu27030 whoamI: 0x%x %p %x \r\n", whoamI, light_dev_ctx.handle,
         (uint8_t) BU27030_REG_PART_ID);

  if (whoamI != BU27030_PART_ID_VALUE) {
    printf("init failed for bu27030!\r\n");
    return SENSOR_FAILED;
  }

  /* Restore default configuration */
  bu27030_init_client(&light_dev_ctx);

  return SENSOR_SUCCESS;
}

static int32_t bu27030_enable()
{
  int32_t ret = 0;
  uint8_t POWER_ON = 1;
  if (sensor_activate_count == 0) {
    ret = light_platform_write(&light_dev_ctx, BU27030_REG_CONTROL,
                               &POWER_ON, 1);
    light_activated = true;
  }
  sensor_activate_count++;
  return ret;
}

static int32_t bu27030_disable()
{
  int32_t ret = 0;
  uint8_t POWER_OFF = 0;
  sensor_activate_count--;

  if (sensor_activate_count == 0) {
    ret = light_platform_write(&light_dev_ctx, BU27030_REG_CONTROL,
                               &POWER_OFF, 1);
    light_activated = false;
  }
  return ret;
}

int bu27030_light_activate(bool activate)
{
  int res = 0;
  printf("bu27030_light_activate: %d \r\n", activate);
  if (activate) {
    res = bu27030_enable();
  } else {
    res = bu27030_disable();
  }
  return res;
}

/* Main Example --------------------------------------------------------------*/
int bu27030_publish_sensor_data(void *para)
{
  /* Read samples in polling mode (no int) */
  sensors_event_t sensor_data = { 0 };
  //printf("bu27030_publish_sensor_data: %d \r\n", light_activated);
  if (light_activated) {
    // core process
    bu27030_get_raw_data_and_calculated_lux_without_scale(&light_dev_ctx);
    //send related messages to SensorDataQ
    sensor_data.sensor_type = SENSOR_TYPE_LIGHT;
    sensor_data.accuracy = 3;
    sensor_data.timestamp = sensor_get_timestamp();
    sensor_data.sensor_data_t.vec.data[0] = light_data * light_scale;
    sensor_data.sensor_data_t.vec.data[1] = light_data;
    sensor_data.sensor_data_t.vec.data[2] = data.data0;
    sensor_data.sensor_data_t.vec.data[3] = data.data1;

    osMessageQueuePut(light_init_parms->SensorDataQHandle, &sensor_data, 0, 0);
  }
  return 0;
}

int bu27030_init_complete(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

int bu27030_config(uint8_t config_type, void *para)
{
  //send init complete event to smgr?
  int res = 0;
  sensors_event_t sensor_data = { 0 };

  switch (config_type) {
  case SENSOR_CONFIG_SELFTEST:
    printf("bu27030 selftest message \r\n");
    break;
  case SENSOR_CONFIG_CALIBRATION:
    printf("bu27030 cali message \r\n");
    bu27030_init_client(&light_dev_ctx);
    uint8_t POWER_ON = 1;
    uint8_t POWER_OFF = 0;
    light_platform_write(&light_dev_ctx, BU27030_REG_CONTROL, &POWER_ON, 1);
    res = bu27030_light_calibration(&light_dev_ctx, &light_scale);

    light_platform_write(&light_dev_ctx, BU27030_REG_CONTROL, &POWER_OFF, 1);
    if (res == 0) {
      memcpy(&light_init_parms->board_calidata->lux_scale, &light_scale,
             sizeof(float));
      if (light_init_parms->board_calidata->light_cali_ver != DEFAULT_VERSION)
        light_init_parms->board_calidata->light_cali_ver++;
      else
        light_init_parms->board_calidata->light_cali_ver = 1;
    }
    break;
  case SENSOR_CONFIG_BIAS:
    //store calibrate sensor data to driver when boot up
    if (light_init_parms->board_calidata->light_cali_ver != DEFAULT_VERSION) {
      memcpy(&light_scale, &light_init_parms->board_calidata->lux_scale,
             sizeof(float));
      printf("light_scale data: {%f} \r\n", light_scale);
    }
    break;
  case SENSOR_CALIBRATION_RESULT:
    sensor_data.sensor_type = SENSOR_TYPE_LIGHT;
    sensor_data.accuracy = 0xFF;
    sensor_data.timestamp = sensor_get_timestamp();
    sensor_data.sensor_data_t.vec.data[0] = light_scale;
    sensor_data.sensor_data_t.vec.data[1] =
        (float)light_init_parms->board_calidata->light_cali_ver;
    sensor_data.sensor_data_t.vec.data[2] = 0;
    sensor_data.sensor_data_t.vec.data[3] = 0;
    osMessageQueuePut(light_init_parms->SensorDataQHandle, &sensor_data, 0, 0);
    break;
  default:
    break;
  }
  return res;
}

int bu27030_publish_config_resp(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

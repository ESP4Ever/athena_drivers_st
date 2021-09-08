/*
 * bf2092_driver.c
 *
 *  Created on: Dec 1, 2020
 *      Author: sangweilin@xiaomi.com
 */
#include <string.h>
#include "bf2092_reg.h"
#include "sensor_device.h"

/* Private macro -------------------------------------------------------------*/
#define HIGHESTBITMASK 0x80
/*************** Global Data ******************/
////////////////////////////////////////////////////////////////
#define maxCount (5)
/* Private variables ---------------------------------------------------------*/
static uint8_t bf2092_raw_data[2];
static uint8_t last_bf2092_raw_data[2];
static stmdev_ctx_t optical_dev_ctx;
static platform_prams *optical_init_parms;
static uint8_t sensor_activate_count = 0;
static bool optical_activated = false;
static int is_env_good = -1;
static const float coeff_of_bf2092 = 0.002754;
static int complete_data_array_init = 0;
static int idx = 0;
static uint64_t current_timestamp = 0;
static uint64_t last_timestamp = 0;

static float x_history_values[maxCount] = {0.0f};
static int x_history_ages[maxCount] = {0};

static float y_history_values[maxCount] = {0.0f};
static int y_history_ages[maxCount] = {0};

/* Extern variables ----------------------------------------------------------*/

/* Swap the contents of 'a' and 'b' */
void swapint(int *a, int *b)
{
  int temp = *a;
  *a = *b;
  *b = temp;
}

/* Swap the contents of 'a' and 'b' */
void swapfloat(float *a, float *b)
{
  float temp = *a;
  *a = *b;
  *b = temp;
}

/* Returns the median of the values stored in data */
int median(int *data, int len)
{
  if (len & 1) {
    return data[len >> 1];
  } else {
    return (data[len >> 1] + data[(len >> 1) - 1] + 1) / 2;
  }
}

/* Figures out where to insert a new value for our streaming median */
int get_insertion_index(int *ages, int len)
{
  for (int i = 0; i < len; i++) {
    // Replace an invalid value index or the oldest value index
    if ((0 == ages[i]) || (len + 1 == ages[i])) {
      return i;
    }
  }
  return 0;// We should never reach this return statement
}

/*
Return a new median based on the new input value
Leverage the fact that we don't need to sort the entire history array each time
*/
float streaming_median(float new_val, float *values, int *ages, int len)
{
  int i;
  int num_valid = len;  // Initially assume that all stored values are valid
  int index;

  for (i = 0; i < len; i++) {
    if (0 != ages[i])
      ages[i]++;      // An age of '0' means there is not a valid value for that index
    else
      num_valid--;    // We have one fewer valid values than initially assumed
  }                   // All of the values are now older by one sample

  index = get_insertion_index(ages, len);    // We want to replace the oldest value with our new value
  ages[index] = 1;                           // Indicate that the value at index 'index' is new - '1' is the lowest valid age
  values[index] = new_val;                   // Store the new value in it's proper location
  if (num_valid < len)
    num_valid++;                             // We just replaced an invalid value index

  while ((index > 0) && (values[index] > values[index - 1])) {
    swapfloat(&values[index], &values[index - 1]);    // Swap the values
    swapint(&ages[index], &ages[index - 1]);          // Swap the ages corresponding to each value
    index--;                                          // Now check the next value down
  }

  while ((index < len - 1) && (values[index] < values[index + 1])) {
    swapfloat(&values[index], &values[index + 1]);    // Swap the values
    swapint(&ages[index], &ages[index + 1]);          // Swap the ages corresponding to each value
    index++;                                          // Now check the next value up
  }

  return (values[1]+values[2]+values[3]) / 3;
}

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
static int32_t bf2092_platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                                     uint16_t len)
{
  if (handle == &optical_init_parms->i2c_handle) {
    //printf("start i2c write, \r\n");
	  HAL_I2C_Mem_Write(handle, (uint16_t) BF2092_I2C_COMUNICATE_ADD, reg,
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
static int32_t bf2092_platform_read(void *handle, uint8_t reg, uint8_t * bufp,
                                    uint16_t len)
{
  if (handle == &optical_init_parms->i2c_handle) {
    //printf("start i2c read, \r\n");
    HAL_I2C_Mem_Read(handle, (uint16_t) BF2092_I2C_COMUNICATE_ADD, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }

  return 0;
}

/******************************************************************************
 * NAME       : bf2092_driver_reset
 * FUNCTION   : reset bf2092 register
 * REMARKS    :
 *****************************************************************************/
static int bf2092_driver_reset(stmdev_ctx_t * ctx)
{
  int result = 0;
  /* set soft reset */
  uint8_t reset_value = 0x01;
  bf2092_platform_write(ctx->handle, RESET_REG, &reset_value, 1);
  return (result);
}

static int bf2092_check_raw_data_quality(stmdev_ctx_t * ctx)
{
  uint8_t imagequality;
  bf2092_platform_read(ctx->handle, IMAGE_Q_REG, &imagequality, 1);
  if (imagequality < 5)
    return -1;
  uint8_t lightquality;
  bf2092_platform_read(ctx->handle, LIGHT_Q_REG, &lightquality, 1);
  if (lightquality < 10)
    return -1;
  return 0;
}

static void bf2092_get_raw_data(stmdev_ctx_t * ctx)
{
  uint8_t dataready;
  bf2092_platform_read(ctx->handle, DATA_READY_REG, &dataready, 1);
  if (dataready & HIGHESTBITMASK) {     //check bit7, if 1, data ready, if 0 not ready
    bf2092_platform_read(ctx->handle, DATA_X_REG, &bf2092_raw_data[0], 1);
    bf2092_platform_read(ctx->handle, DATA_Y_REG, &bf2092_raw_data[1], 1);
    is_env_good = bf2092_check_raw_data_quality(ctx);
    if (is_env_good < 0) {
      bf2092_raw_data[0] = last_bf2092_raw_data[0];
      bf2092_raw_data[1] = last_bf2092_raw_data[1];
      return;
    }
	if ((bf2092_raw_data[0]&(~HIGHESTBITMASK)) < 3)
		bf2092_raw_data[0] = 0;
	if ((bf2092_raw_data[1]&(~HIGHESTBITMASK)) < 3)
		bf2092_raw_data[1] = 0;

    last_bf2092_raw_data[0] = bf2092_raw_data[0];
    last_bf2092_raw_data[1] = bf2092_raw_data[1];
  } else
    printf("bf2092 data not ready!!!\r\n");
}

/*************** Initialze Functions ******************/
static int bf2092_init_client(stmdev_ctx_t * ctx)
{
  int result = 0;
  uint8_t tmp = 0x00;
  bf2092_platform_write(ctx->handle, COM7, &tmp, 1);
  tmp = 0x03;
  bf2092_platform_write(ctx->handle, ISPBYPS, &tmp, 1);
  tmp = 0xa5;
  bf2092_platform_write(ctx->handle, REG_CTR0, &tmp, 1);
  tmp = 0x44;
  bf2092_platform_write(ctx->handle, REG_CTR2, &tmp, 1);
  tmp = 0x24;
  bf2092_platform_write(ctx->handle, REG_CTR3, &tmp, 1);
  tmp = 0x09;
  bf2092_platform_write(ctx->handle, REG_CTR4, &tmp, 1);
  tmp = 0x7c;
  bf2092_platform_write(ctx->handle, REG_CTR5, &tmp, 1);
  tmp = 0x7e;
  bf2092_platform_write(ctx->handle, REG_CTR6, &tmp, 1);
  tmp = 0xc0;
  bf2092_platform_write(ctx->handle, REG_CTR7, &tmp, 1);
  tmp = 0x12;
  bf2092_platform_write(ctx->handle, DIN_SET1, &tmp, 1);
  tmp = 0x23;
  bf2092_platform_write(ctx->handle, COUTER3, &tmp, 1);
  tmp = 0x24;
  bf2092_platform_write(ctx->handle, COUTER9, &tmp, 1);
  tmp = 0x25;
  bf2092_platform_write(ctx->handle, COUTER10, &tmp, 1);
  tmp = 0x06;
  bf2092_platform_write(ctx->handle, INT_TIM_TH, &tmp, 1);
  tmp = 0x00;
  bf2092_platform_write(ctx->handle, BPS_PARAH, &tmp, 1);
  tmp = 0xe4;
  bf2092_platform_write(ctx->handle, BPS_PARAL, &tmp, 1);
  tmp = 0x08;
  bf2092_platform_write(ctx->handle, UART_REG, &tmp, 1);
  tmp = 0x40;
  bf2092_platform_write(ctx->handle, SC_CNTL1, &tmp, 1);
  tmp = 0xb3;
  bf2092_platform_write(ctx->handle, MODE_CNTL, &tmp, 1);
  tmp = 0x01;
  bf2092_platform_write(ctx->handle, INT_MAX_I2C, &tmp, 1);
  tmp = 0xca;
  bf2092_platform_write(ctx->handle, INT_STEP_50, &tmp, 1);
  tmp = 0xd6;
  bf2092_platform_write(ctx->handle, INT_STEP_60, &tmp, 1);
  tmp = 0x19;
  bf2092_platform_write(ctx->handle, DM_ROWL_AF_PIX, &tmp, 1);
  tmp = 0xff;
  bf2092_platform_write(ctx->handle, GLB_MAX3, &tmp, 1);
  tmp = 0xf4;
  bf2092_platform_write(ctx->handle, SPEED_PLL, &tmp, 1);

  return (result);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
int bf2092_optical_init(void *para, void *para2)
{
  optical_init_parms = (platform_prams *) para;
  /* Initialize mems driver interface */
  optical_dev_ctx.write_reg = bf2092_platform_write;
  optical_dev_ctx.read_reg = bf2092_platform_read;
  optical_dev_ctx.handle = &optical_init_parms->i2c_handle;
  /* power up */
  HAL_GPIO_WritePin(GUANGLIU_EN_GPIO_Port, GUANGLIU_EN_Pin, GPIO_PIN_SET);
  osDelay(2);
  /* Check device ID */
  uint8_t id_0, id_1;
  bf2092_platform_read(optical_dev_ctx.handle, (uint8_t) BF2092_REG_PART_ID_0,
                       &id_0, 1);

  bf2092_platform_read(optical_dev_ctx.handle, (uint8_t) BF2092_REG_PART_ID_1,
                       &id_1, 1);

  printf("bF2092 whoamI: 0x%x 0x%x %p \r\n", id_0, id_1,
         optical_dev_ctx.handle);

  if (id_0 != BF2092_REG_PART_ID_0_VALUE || id_1 != BF2092_REG_PART_ID_1_VALUE) {
    printf("init failed for bf2092!\r\n");
    return SENSOR_FAILED;
  }
  // soft reset
  bf2092_driver_reset(&optical_dev_ctx);

  /* Restore default configuration */
  bf2092_init_client(&optical_dev_ctx);
  return SENSOR_SUCCESS;
}

static int32_t bf2092_enable()
{
  int32_t ret = 0;
  uint8_t STATUS;
  ret = bf2092_platform_read(optical_dev_ctx.handle, ENABLE_REG, &STATUS, 1);
  if (sensor_activate_count == 0) {
    if ((STATUS & 0x01) == 1) {
	  STATUS &= (~0x01);        //bit 0 set to 0
	  ret = bf2092_platform_write(optical_dev_ctx.handle, ENABLE_REG, &STATUS, 1);

    }
    optical_activated = true;
  }
  sensor_activate_count++;
  return ret;
}

static int32_t bf2092_disable()
{
  int32_t ret = 0;
  uint8_t STATUS;
  sensor_activate_count--;
  ret = bf2092_platform_read(optical_dev_ctx.handle, ENABLE_REG, &STATUS, 1);
  if (sensor_activate_count == 0) {
	if ((STATUS & 0x01) != 1) {
	  STATUS |= 0X01;           //bit 0 set to 1
	  ret = bf2092_platform_write(optical_dev_ctx.handle, ENABLE_REG, &STATUS, 1);
	  last_bf2092_raw_data[0] = 0;
	  last_bf2092_raw_data[1] = 0;
	}
    optical_activated = false;
  }
  return ret;
}

int bf2092_optical_activate(bool activate)
{
  int res = 0;
  printf("bf2092_optical_activate: %d \r\n", activate);
  if (activate) {
    res = bf2092_enable();
  } else {
    res = bf2092_disable();
  }
  return res;
}

int bf2092_publish_sensor_data(void *para)
{
  /* Read samples in polling mode (no int) */
  sensors_event_t sensor_data = { 0 };
  float x= 0.0f;
  float y = 0.0f;
  float xaverage = 0.0f;
  float yaverage = 0.0f;
  if (optical_activated) {
    // core process
    bf2092_get_raw_data(&optical_dev_ctx);
    //send related messages to SensorDataQ
    sensor_data.sensor_type = SENSOR_TYPE_LIGHT_SPEED;
    sensor_data.accuracy = 3;
    sensor_data.timestamp = sensor_get_timestamp();
    current_timestamp = sensor_data.timestamp;
    if (is_env_good >= 0) {

      x = -(bf2092_raw_data[0] & (~HIGHESTBITMASK)) * coeff_of_bf2092;
      if (((bf2092_raw_data[0] & HIGHESTBITMASK) >> 7) == 1)    //back, -;front +;
        x = -x;

      y = -(bf2092_raw_data[1] & (~HIGHESTBITMASK)) * coeff_of_bf2092;
      if (((bf2092_raw_data[1] & HIGHESTBITMASK) >> 7) == 1)    //left, -;right +;
        y = -y;

      if (last_timestamp == 0)
        last_timestamp = current_timestamp;
      else {
        xaverage = streaming_median(x / (current_timestamp - last_timestamp) * 1000, x_history_values, x_history_ages, maxCount);
        yaverage = streaming_median(y / (current_timestamp - last_timestamp) * 1000, y_history_values, y_history_ages, maxCount);
        idx++;
        if (idx == maxCount && complete_data_array_init == 0) {
          idx = 0;
          complete_data_array_init = 1;
        }
        if (idx == maxCount)
          idx = 0;

        if (complete_data_array_init) {
          sensor_data.sensor_data_t.vec.data[0] = xaverage;
          sensor_data.sensor_data_t.vec.data[1] = yaverage;
        } else {
          sensor_data.sensor_data_t.vec.data[0] = x;
          sensor_data.sensor_data_t.vec.data[1] = y;
        }
        osMessageQueuePut(optical_init_parms->SensorDataQHandle, &sensor_data,
                        0, 0);
        last_timestamp = current_timestamp;
      }
    }
  }
  return 0;
}

int bf2092_init_complete(void *para)
{
  return SENSOR_SUCCESS;
}

int bf2092_config(uint8_t config_type, void *para)
{
  int res = SENSOR_SUCCESS;
  uint8_t id_0, id_1;

  switch (config_type) {
  case SENSOR_CONFIG_SELFTEST:
    /* Check device ID */
    bf2092_platform_read(optical_dev_ctx.handle, (uint8_t) BF2092_REG_PART_ID_0,
                         &id_0, 1);

    bf2092_platform_read(optical_dev_ctx.handle, (uint8_t) BF2092_REG_PART_ID_1,
                         &id_1, 1);

    if (id_0 != BF2092_REG_PART_ID_0_VALUE
        || id_1 != BF2092_REG_PART_ID_1_VALUE) {
      printf("selftest failed for bf2092!\r\n");
      res = SENSOR_FAILED;
    }
    printf("selftest success for bf2092!\r\n");
    break;
  default:
    printf("unsupported config command\n\r");
    break;
  }
  return res;
}

int bf2092_publish_config_resp(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

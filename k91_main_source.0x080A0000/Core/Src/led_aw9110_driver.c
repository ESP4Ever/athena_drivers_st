/*
 * led_driver.c
 *
 *  Created on: Dec 1, 2020
 *      Author: sangweilin@xiaomi.com
 */

#include <string.h>
#include "led_aw9110_reg.h"
#include "sensor_device.h"

/* Private macro -------------------------------------------------------------*/

/*************** Global Data ******************/
////////////////////////////////////////////////////////////////

/* Private variables ---------------------------------------------------------*/
static stmdev_ctx_t led_dev_ctx;
static platform_prams *led_init_parms;
static uint8_t sensor_activate_count = 0;
static bool led_activated = false;
#define BRIGHTNESS 255
static struct aw9110B aw9110 = { 3, 2, 2, 2, 2 };
static struct aw9110B aw9110_on = { 3, 0, 2, 0, 0 };
static struct aw9110B aw9110_power = { 3, 2, 2, 4, 2 };
static struct aw9110B aw9110_breath = { 3, 2, 2, 2, 1 };
static struct aw9110B aw9110_blink = { 3, 0, 2, 0, 2 };
static uint8_t brightnessarray[6] = { 255, 255, 255, 255, 255, 255 };

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
static int32_t led_platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                                  uint16_t len)
{
  if (handle == &led_init_parms->i2c_handle) {
    if (led_init_parms->boardID == REAR_BOARD) {
      HAL_I2C_Mem_Write(handle, (uint16_t) REAR_LED_I2C_COMUNICATE_ADD, reg,
                        I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    } else {
      HAL_I2C_Mem_Write(handle, (uint16_t) HEAD_LED_I2C_COMUNICATE_ADD, reg,
                        I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    }
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
static int32_t led_platform_read(void *handle, uint8_t reg, uint8_t * bufp,
                                 uint16_t len)
{
  if (handle == &led_init_parms->i2c_handle) {
    if (led_init_parms->boardID == REAR_BOARD) {
      HAL_I2C_Mem_Read(handle, (uint16_t) REAR_LED_I2C_COMUNICATE_ADD, reg,
                       I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    } else {
      HAL_I2C_Mem_Read(handle, (uint16_t) HEAD_LED_I2C_COMUNICATE_ADD, reg,
                       I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    }
  }

  return 0;
}

static void aw9110_led_blink(stmdev_ctx_t * ctx, struct aw9110B *aw9110,
                             uint8_t * brightness, uint8_t enablebits)
{
  uint8_t tmp;

  if (enablebits) {
    tmp = 0x00;
    led_platform_write(ctx->handle, REG_WORK_MODE_P0, &tmp, 1); // led mode
    led_platform_write(ctx->handle, REG_WORK_MODE_P1, &tmp, 1); // led mode
    tmp = enablebits;
    led_platform_write(ctx->handle, REG_EN_BREATH, &tmp, 1);    // enable breath
    tmp = (enablebits >> 4) & 0x03;
    led_platform_write(ctx->handle, REG_CONFIG_P0, &tmp, 1);    // blink mode
    tmp = enablebits & 0x0f;
    led_platform_write(ctx->handle, REG_CONFIG_P1, &tmp, 1);    // blink mode
    tmp = (aw9110->fall_time << 3) | (aw9110->rise_time);
    led_platform_write(ctx->handle, REG_FADE_TIME, &tmp, 1);    // fade time
    tmp = (aw9110->off_time << 3) | (aw9110->on_time);
    led_platform_write(ctx->handle, REG_FULL_TIME, &tmp, 1);    // on/off time
    for (int i = 0; i < 6; i++) {
      if ((enablebits >> i) & 0x01) {
        led_platform_write(ctx->handle, REG_DIM00 + i, &brightness[i], 1);      // dimming
      }
      else {
    	  tmp = 0x00;
    	  led_platform_write(ctx->handle, REG_DIM00 + i, &tmp, 1);      // dimming
      }
    }
    tmp = 0x83;                 //| aw9110->imax;
    led_platform_write(ctx->handle, REG_CTRL, &tmp, 1); // blink enable | imax
  }
}

static int aw9110_led_off(stmdev_ctx_t * ctx)
{
  uint8_t tmp = 0x00;
  led_platform_write(ctx->handle, REG_WORK_MODE_P0, &tmp, 1);   // led mode
  led_platform_write(ctx->handle, REG_WORK_MODE_P1, &tmp, 1);   // led mode
  led_platform_write(ctx->handle, REG_EN_BREATH, &tmp, 1);      // disable breath
  tmp = 0x03;
  led_platform_write(ctx->handle, REG_CTRL, &tmp, 1);   // imax
  tmp = 0x00;
  for (int i = 0; i < 6; i++) {
    led_platform_write(ctx->handle, REG_DIM00 + i, &tmp, 1);    // dimming
  }
}

static int aw9110_led_on(stmdev_ctx_t * ctx, uint8_t * brightness,
                         uint8_t enablebits)
{
  uint8_t tmp = 0x00;
  led_platform_write(ctx->handle, REG_WORK_MODE_P0, &tmp, 1);   // led mode
  led_platform_write(ctx->handle, REG_WORK_MODE_P1, &tmp, 1);   // led mode
  led_platform_write(ctx->handle, REG_EN_BREATH, &tmp, 1);    // enable breath
  tmp = 0x03;
  led_platform_write(ctx->handle, REG_CTRL, &tmp, 1);   // imax
  for (int i = 0; i < 6; i++) {
    if ((enablebits >> i) & 0x01) {
      led_platform_write(ctx->handle, REG_DIM00 + i, &brightness[i], 1);        // lighting on
    }
  }
}

static int aw9110_hw_on(stmdev_ctx_t * ctx)
{
  if (ctx != NULL) {
    HAL_GPIO_WritePin(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port,
                      HEAD_AND_REAR_LED_DRIVER_EN_Pin, GPIO_PIN_SET);
    osDelay(1);
  } else {
    printf("%s:  failed\n", __func__);
  }
  return 0;
}

static int aw9110_hw_off(stmdev_ctx_t * ctx)
{
  if (ctx != NULL) {
    HAL_GPIO_WritePin(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port,
                      HEAD_AND_REAR_LED_DRIVER_EN_Pin, GPIO_PIN_RESET);
    osDelay(1);
  } else {
    printf("%s:  failed\n", __func__);
  }
  return 0;
}

/******************************************************************************
 * NAME       : led_hw_reset
 * FUNCTION   : reset led
 * REMARKS    :
 *****************************************************************************/
static int led_hw_reset(stmdev_ctx_t * ctx)
{
  if (ctx != NULL) {
    HAL_GPIO_WritePin(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port,
                      HEAD_AND_REAR_LED_DRIVER_EN_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port,
                      HEAD_AND_REAR_LED_DRIVER_EN_Pin, GPIO_PIN_SET);
    osDelay(1);
  } else {
    printf("%s: failed\n", __func__);
  }
  return 0;
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
int led_init(void *para, void *para2)
{
  led_init_parms = (platform_prams *) para;
  /* Initialize mems driver interface */
  led_dev_ctx.write_reg = led_platform_write;
  led_dev_ctx.read_reg = led_platform_read;
  led_dev_ctx.handle = &led_init_parms->i2c_handle;

  /* hardware reset */
  led_hw_reset(&led_dev_ctx);
  /* aw9110 chip id */
  uint8_t reg_val;
  led_platform_read(led_dev_ctx.handle, REG_ID, &reg_val, 1);
  printf("led_init reg_id 0x%x\r\n", reg_val);
  osDelay(1);
  if (reg_val != AW9110_ID)
    return SENSOR_FAILED;
  aw9110_led_on(&led_dev_ctx, brightnessarray, 0x09);

  return SENSOR_SUCCESS;
}

static int32_t led_enable()
{
  int32_t ret = 0;

  if (sensor_activate_count == 0) {
    led_activated = true;
  }
  sensor_activate_count++;
  return ret;
}

static int32_t led_disable()
{
  int32_t ret = 0;
  sensor_activate_count--;
  if (sensor_activate_count == 0) {
    led_activated = false;
  }
  return ret;
}

int led_activate(bool activate)
{
  int res = 0;
  printf("led_activate: %d \r\n", activate);
  if (activate) {
    res = led_enable();
  } else {
    res = led_disable();
  }
  return res;
}

int led_init_complete(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

//out5-->out0
int led_config(uint8_t config, void *para)
{
  //send init complete event to smgr?

  uint8_t *config_data = NULL;
  config_data = (uint8_t *) para;

  aw9110_led_off(&led_dev_ctx);
  uint8_t config_type = config_data[0] & 0x0F;
  if (((config_data[0] >> 4) & 0x0F) == 0) {
    switch (config_type) {
    case 0:
      aw9110_led_off(&led_dev_ctx);
      break;
    case 1:
      aw9110_led_on(&led_dev_ctx, brightnessarray, 0x09);       // red lighting
      break;
    case 2:
      aw9110_led_blink(&led_dev_ctx, &aw9110_breath, brightnessarray, 0x09);    // red breathing
      break;
    case 3:
      aw9110_led_blink(&led_dev_ctx, &aw9110_blink, brightnessarray, 0x09);     // red blinking
      break;
    case 4:
      aw9110_led_on(&led_dev_ctx, brightnessarray, (0x09 << 1));       // green lighting
      break;
    case 5:
      aw9110_led_blink(&led_dev_ctx, &aw9110_breath, brightnessarray, (0x09 << 1));    // green breathing
      break;
    case 6:
      aw9110_led_blink(&led_dev_ctx, &aw9110_blink, brightnessarray, (0x09 << 1));     // green blinking
      break;
    default:
      aw9110_led_on(&led_dev_ctx, brightnessarray, 0x09);       // red lighting
      break;
    }
  } else if (((config_data[0] >> 4) & 0x0F) == 2) {
	  if (config_type == 0) {
		  aw9110_led_on(&led_dev_ctx, brightnessarray, 0x3f);       // White lighting
	  } else {
		  aw9110_led_on(&led_dev_ctx, brightnessarray, 0x09);       // red lighting
	  }
  }
  return SENSOR_SUCCESS;
}

int led_publish_config_resp(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

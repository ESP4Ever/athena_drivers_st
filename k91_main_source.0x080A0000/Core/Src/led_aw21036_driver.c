/*
 * led_aw21036_driver.c
 *
 *  Created on: Dec 1, 2020
 *      Author: sangweilin@xiaomi.com
 */

#include <string.h>
#include "led_aw21036_reg.h"
#include "sensor_device.h"

/* Private macro -------------------------------------------------------------*/

/*************** Global Data ******************/
////////////////////////////////////////////////////////////////

/* Private variables ---------------------------------------------------------*/
static stmdev_ctx_t headled_dev_ctx;
static platform_prams *headled_init_parms;
static uint8_t sensor_activate_count = 0;
static bool headled_activated = false;
static bool first_start_head_flow_water = true;
static uint8_t head_flow_water_tick = 0;

bool head_flow_water_effect = false;

AW21036_CFG aw21036_cfg_array[] = {
  {aw21036_cfg_led_off, sizeof(aw21036_cfg_led_off)}
  ,
  {aw21036_led_darkblue_power_on, sizeof(aw21036_led_darkblue_power_on)}
  ,
  {aw21036_led_darkblue_power_off, sizeof(aw21036_led_darkblue_power_off)}
  ,
  {aw21036_led_darkblue_on, sizeof(aw21036_led_darkblue_on)}
  ,
  {aw21036_led_skyblue_on, sizeof(aw21036_led_skyblue_on)}
  ,
  {aw21036_led_orange_on, sizeof(aw21036_led_orange_on)}
  ,
  {aw21036_led_red_on, sizeof(aw21036_led_red_on)}
  ,
  {aw21036_led_darkblue_breath_forever,
   sizeof(aw21036_led_darkblue_breath_forever)}
  ,
  {aw21036_led_skyblue_breath_forever,
   sizeof(aw21036_led_skyblue_breath_forever)}
  ,
  {aw21036_led_darkblue_blink_forever,
   sizeof(aw21036_led_darkblue_blink_forever)}
  ,
  {aw21036_led_orange_blink_forever, sizeof(aw21036_led_orange_blink_forever)}
  ,
  {aw21036_led_red_blink_forever, sizeof(aw21036_led_red_blink_forever)}
    ,
};

static struct aw21036 maw21036 = { 0 };

/*
 * tim3 is used for led blinking related function
 */

TIM_HandleTypeDef *tim3;        // XXX move and rename

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
static int32_t headled_platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                                      uint16_t len)
{
  if (handle == &headled_init_parms->i2c_handle) {
    HAL_I2C_Mem_Write(handle, (uint16_t) HEAD_LED_AW21036_I2C_COMUNICATE_ADD,
                      reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
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

static int32_t headled_platform_read(void *handle, uint8_t reg, uint8_t * bufp,
                                     uint16_t len)
{
  if (handle == &headled_init_parms->i2c_handle) {
    HAL_I2C_Mem_Read(handle, (uint16_t) HEAD_LED_AW21036_I2C_COMUNICATE_ADD,
                     reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }

  return 0;
}

static int aw21036_i2c_write_bits(void *handle,
                                  unsigned char reg_addr, unsigned int mask,
                                  unsigned char reg_data)
{
  uint8_t reg_val;
  headled_platform_read(handle, reg_addr, &reg_val, 1);
  reg_val &= mask;
  reg_val |= reg_data;
  headled_platform_write(handle, reg_addr, &reg_val, 1);
  return 0;
}

static int aw21036_chip_enable(void *handle, bool flag)
{
  if (flag)
    aw21036_i2c_write_bits(handle, AW21036_REG_GCR,
                           AW21036_BIT_GCR_CHIPEN_MASK,
                           AW21036_BIT_GCR_CHIPEN_ENABLE);
  else
    aw21036_i2c_write_bits(handle, AW21036_REG_GCR,
                           AW21036_BIT_GCR_CHIPEN_MASK,
                           AW21036_BIT_GCR_CHIPEN_DISABLE);

  return 0;
}

static int aw21036_pwm_freq_cfg(void *handle, struct aw21036 *aw21036)
{
  printf("%s: enter\r\n", __func__);

  aw21036_i2c_write_bits(handle, AW21036_REG_GCR,
                         AW21036_BIT_GCR_CLKFRQ_MASK, aw21036->pwm_freq);
  printf("%s: osc clk freq: 0x%x\r\n", __func__, aw21036->pwm_freq);

  return 0;
}

/*****************************************************
 *
 * firmware/cfg update
 *
 *****************************************************/
static void aw21036_update_cfg_array(void *handle,
                                     unsigned char *p_cfg_data,
                                     unsigned int cfg_size)
{
  unsigned int i = 0;

  for (i = 0; i < cfg_size; i += 2)
    headled_platform_write(handle, p_cfg_data[i], &p_cfg_data[i + 1], 1);
}

static int aw21036_cfg_update(void *handle, struct aw21036 *aw21036)
{
  printf("%s: enter\r\n", __func__);
  aw21036_update_cfg_array(handle,
                           (aw21036_cfg_array[aw21036->effect].p),
                           aw21036_cfg_array[aw21036->effect].count);
  return 0;
}

/*****************************************************
 *
 * aw21036 led cfg
 *
 *****************************************************/
/*
static int aw21036_led_update(void* handle)
{
	uint8_t reg_val = 0x00;
	headled_platform_write(handle, AW21036_REG_UPDATE, &reg_val, 1);
	return 0;
}

static void aw21036_brightness_work(struct work_struct *work)
{
	struct aw21036 *aw21036 = container_of(work, struct aw21036,
					       brightness_work);

	printf("%s: enter\n", __func__);

	if (aw21036->cdev.brightness > aw21036->cdev.max_brightness)
		aw21036->cdev.brightness = aw21036->cdev.max_brightness;

	aw21036_i2c_write(aw21036, AW21036_REG_GCCR, aw21036->cdev.brightness);
}

static int aw21036_rgbcolor_store(void * handle,
				      const uint8_t *buf, size_t len)
{
	unsigned int databuf[2] = { buf[0], buf[1] };
	int temp = 0x00;
	headled_platform_write(handle, AW21036_REG_GCFG0, &temp, 1);// GEn = 0
	temp = 0x10;
	headled_platform_write(handle, AW21036_REG_GCFG1, &temp, 1);// GCOLDIS = 1 GEn = 0
	temp = 0x01;
	headled_platform_write(handle, AW21036_REG_GCR2, &temp, 1);//RGBMD = 1
	temp = 0xff;
	headled_platform_write(handle, AW21036_REG_BR0 + databuf[0], &temp, 1);
	temp = 0x0f;
	headled_platform_write(handle, AW21036_REG_GCCR, &temp, 1);

	temp = (databuf[1] & 0x00ff0000) >> 16;
	headled_platform_write(handle, AW21036_REG_COL0 + databuf[0] * 3,
			  &temp, 1);

	temp = (databuf[1] & 0x0000ff00) >> 8;
	headled_platform_write(handle, AW21036_REG_COL0 + databuf[0] * 3 + 1,
			  &temp, 1);

	temp = (databuf[1] & 0x000000ff);
	headled_platform_write(handle, AW21036_REG_COL0 + databuf[0] * 3 + 2,
			  &temp, 1);

	aw21036_led_update(handle);
	return len;
}

static void aw21036_led_clear(void * handle)
{
	printf("%s: enter\r\n", __func__);
	uint8_t reg_val = 0x00;
	for (int i =0; i < 36; i++) {
		headled_platform_write(handle, AW21036_REG_BR0 + i, &reg_val, 1);
		headled_platform_write(handle, AW21036_REG_COL0 + i, &reg_val, 1);
	}
}

*/

/******************************************************
 *
 * led class dev
 ******************************************************/
static int aw21036_led_init(void *handle, struct aw21036 *aw21036)
{
  printf("%s: enter\r\n", __func__);
  uint8_t reg_val = 0x00;
  headled_platform_write(handle, AW21036_REG_RESET, &reg_val, 1);
  osDelay(5);
  aw21036_chip_enable(handle, true);
  osDelay(5);

  aw21036_pwm_freq_cfg(handle, aw21036);

  aw21036_i2c_write_bits(handle, AW21036_REG_GCR,
                         AW21036_BIT_GCR_APSE_MASK,
                         AW21036_BIT_GCR_APSE_ENABLE);
  printf("%s: DONE!\r\n", __func__);
  return 0;
}

/******************************************************************************
 * NAME       : led_hw_reset
 * FUNCTION   : reset led
 * REMARKS    :
 *****************************************************************************/
static int headled_hw_reset(stmdev_ctx_t * ctx)
{
  if (ctx != NULL) {
    HAL_GPIO_WritePin(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port,
                      HEAD_AND_REAR_LED_DRIVER_EN_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port,
                      HEAD_AND_REAR_LED_DRIVER_EN_Pin, GPIO_PIN_SET);
    osDelay(1);
  } else {
    printf("%s: failed\r\n", __func__);
  }
  return 0;
}

int headled_init(void *para, void *para2)
{
  headled_init_parms = (platform_prams *) para;
  /* Initialize mems driver interface */
  headled_dev_ctx.write_reg = headled_platform_write;
  headled_dev_ctx.read_reg = headled_platform_read;
  headled_dev_ctx.handle = &headled_init_parms->i2c_handle;

  /* hardware reset */
  headled_hw_reset(&headled_dev_ctx);

  //osDelay(1);
  /* aw21036 chip id */
  uint8_t reg_val, ver_val;
  headled_platform_read(headled_dev_ctx.handle, AW21036_REG_RESET, &reg_val, 1);
  printf("led_init reg_id 0x%x\r\n", reg_val);
  osDelay(1);
  headled_platform_read(headled_dev_ctx.handle, AW21036_REG_VER, &ver_val, 1);
  printf("led_init ver_id 0x%x\r\n", ver_val);
  osDelay(1);
  if (reg_val != AW21036_CHIPID || ver_val != AW21036_CHIP_VERSION)
    return SENSOR_FAILED;
  maw21036.chipid = reg_val;
  maw21036.led_current = LED_CURRENT_MAX;
  maw21036.pwm_freq = 1;
  maw21036.imax = 1;
  aw21036_led_init(headled_dev_ctx.handle, &maw21036);
  maw21036.effect = 1;
  aw21036_cfg_update(headled_dev_ctx.handle, &maw21036);
  return SENSOR_SUCCESS;
}

static int32_t headled_enable()
{
  int32_t ret = 0;

  if (sensor_activate_count == 0) {
    headled_activated = true;
  }
  sensor_activate_count++;
  return ret;
}

static int32_t headled_disable()
{
  int32_t ret = 0;
  sensor_activate_count--;
  if (sensor_activate_count == 0) {
    headled_activated = false;
  }
  return ret;
}

int headled_activate(bool activate)
{
  int res = 0;
  printf("head led_activate: %d\r\n", activate);
  if (activate) {
    res = headled_enable();
  } else {
    res = headled_disable();
  }
  return res;
}

int headled_init_complete(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

int headled_config(uint8_t config, void *para)
{
  //send init complete event to smgr?

  uint8_t *config_data = NULL;
  config_data = (uint8_t *) para;

  switch (config) {
  case SENSOR_CONFIG_TIMEOUT:
    printf("@@@SENSOR_CONFIG_TIMEOUT!\r\n");
    headled_flow_water();
    break;
  default:
    maw21036.effect = 0;
    aw21036_cfg_update(headled_dev_ctx.handle, &maw21036);
    uint8_t config_type = config_data[0] & 0x0F;
    if (((config_data[0] >> 4) & 0x0F) == 0) {
      //0 off;1 white;2 red;3 green;4 blue; 5 white breathforever; 6 red breathforever; 7 green breathforever; 8 blue breathforever
      if (config_type == FLOW_WATER_COFIG_TYPE) {
        sensor_op_timer_enable(tim3);
        head_flow_water_effect = true;
        head_flow_water_tick = 0;
      } else {
        sensor_op_timer_disable(tim3);
        head_flow_water_effect = false;
        maw21036.effect = config_type;

        if (maw21036.effect < ARRAY_SIZE(aw21036_cfg_array))
          aw21036_cfg_update(headled_dev_ctx.handle, &maw21036);
        else {
          printf("%s: effect out of range!\r\n", __func__);
          maw21036.effect = 1;
          aw21036_cfg_update(headled_dev_ctx.handle, &maw21036);
        }
      }
    }
    break;
  }
  return SENSOR_SUCCESS;
}

int headled_publish_config_resp(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

void headled_flow_water(void)
{
  //printf("Enter headled_flow_water!");
  if (head_flow_water_effect) {
    if (first_start_head_flow_water) {
      //printf("head_flow_water_effect:%d !\r\n",head_flow_water_effect);
      first_start_head_flow_water = false;
      aw21036_update_cfg_array(headled_dev_ctx.handle,
                               (aw21036_cfg_array[1].p),
                               aw21036_cfg_array[1].count);
    } else {
      //printf("head_flow_water_tick:%d !\r\n",head_flow_water_tick);
      if (head_flow_water_tick >= 0 && head_flow_water_tick < 9) {
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick],
                               &COL_REG_value[1], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick] - 1,
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick] - 2,
                               &COL_REG_value[0], 1);
      } else if (head_flow_water_tick >= 9 && head_flow_water_tick < 18) {
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[17 - head_flow_water_tick],
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[17 - head_flow_water_tick] - 1,
                               &COL_REG_value[1], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[17 - head_flow_water_tick] - 2,
                               &COL_REG_value[0], 1);
      } else if (head_flow_water_tick >= 18 && head_flow_water_tick < 27) {
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick - 18],
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick - 18] - 1,
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick - 18] - 2,
                               &COL_REG_value[1], 1);
      } else if (head_flow_water_tick >= 27 && head_flow_water_tick < 36) {
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[35 - head_flow_water_tick],
                               &COL_REG_value[1], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[35 - head_flow_water_tick] - 1,
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[35 - head_flow_water_tick] - 2,
                               &COL_REG_value[0], 1);
      } else if (head_flow_water_tick >= 36 && head_flow_water_tick < 45) {
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick - 36],
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick - 36] - 1,
                               &COL_REG_value[1], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[head_flow_water_tick - 36] - 2,
                               &COL_REG_value[0], 1);
      } else if (head_flow_water_tick >= 45 && head_flow_water_tick < 54) {
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[53 - head_flow_water_tick],
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[53 - head_flow_water_tick] - 1,
                               &COL_REG_value[0], 1);
        headled_platform_write(headled_dev_ctx.handle,
                               LED_R_COL_REG[53 - head_flow_water_tick] - 2,
                               &COL_REG_value[1], 1);
      }
      head_flow_water_tick = head_flow_water_tick + 1;
      if (head_flow_water_tick >= 54) {
        head_flow_water_tick = 0;
      }
    }

  } else {

  }

}

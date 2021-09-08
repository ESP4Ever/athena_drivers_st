/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI196V1
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A3
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <lsm6dso_reg.h>

#include "sensor_device.h"

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

/* Private macro -------------------------------------------------------------*/

#define ST_CUSTOM_IMU_AXIS_ORDER_X  1
#define ST_CUSTOM_IMU_AXIS_ORDER_Y  2
#define ST_CUSTOM_IMU_AXIS_ORDER_Z  0
#define ST_CUSTOM_IMU_AXIS_SIGN_X   0
#define ST_CUSTOM_IMU_AXIS_SIGN_Y   0
#define ST_CUSTOM_IMU_AXIS_SIGN_Z   0

/* Global variable for ASA value. */
uint8_t g_imu_axis_order[3];
uint8_t g_imu_axis_sign[3];

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;

static float acceleration_offset[3] = { 0 };
static float angular_rate_offset[3] = { 0 };

/* Extern variables ----------------------------------------------------------*/
PID imu_pid_struct = { 0 };

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t * bufp,
                             uint16_t len);

static stmdev_ctx_t dev_ctx;
static platform_prams *acc_gyro_init_parms;
static uint8_t acc_activate_count = 0;
static uint8_t gyro_activate_count = 0;

static bool acc_activated = false;
static bool gyro_activated = false;

static bool acc_data_report_in_uart_mode = false;
static bool gyro_data_report_in_uart_mode = false;

TIM_HandleTypeDef *tim1;        // XXX move and rename

void pid_pwm_control(uint8_t command, uint16_t data)
{
  return;
  static uint16_t current_ccr1 = 0;
  switch (command) {
  case PWM_ACTIVATE:
    if (tim1 != NULL) {
      tim1->Instance->CCR1 = 1;
      current_ccr1 = 1;
      if (HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_1) != HAL_OK) {
        /* Starting Error */
        Error_Handler();
      }
    } else {
      //assert here
    }
    break;
  case PWM_DEACTIVATE:
    if (tim1 != NULL) {
      if (HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_1) != HAL_OK) {
        /* Starting Error */
        Error_Handler();
      }
    } else {
      //assert here
    }
    break;
  case PWM_CONFIG:
    if (tim1 != NULL) {
      if (data <= 0xFFFF && data >= 0) {
        if (current_ccr1 != data) {
          tim1->Instance->CCR1 = data;
          current_ccr1 = data;
        }
      }
    } else {
      //assert here
    }
    break;
  }
}

void PID_Init(PID * Pid, float SETtemp)
{
  Pid->Set_temperature = SETtemp;
  Pid->proportion = 0.5;
  Pid->integral = 4;
  Pid->differential = 1.5;
  Pid->T = 100;
  Pid->error_current = 0.0;
  Pid->error_last = 0;
  Pid->error_sum = 0;
  Pid->pid_proportion_out = 0;
  Pid->pid_integral_out = 0;
  Pid->pid_differential_out = 0;
  Pid->pid_out = 0;
}

float pid_temp_control(PID * PP, float current_temp)
{
  static float PID_ZL = 0.0;
  float result = 0.0;
  float A0, A1, A2;
  PP->error_current = PP->Set_temperature - current_temp;
  //printf("error_c:%f, current_c: %f \r\n", PP->error_current, current_temp);

  if (PP->error_current >= PID_TEMP_THRES)
    pid_pwm_control(PWM_CONFIG, 1);
  else if (PP->error_current <= -PID_TEMP_THRES)
    pid_pwm_control(PWM_CONFIG, 0xFFFF);
  else if (PP->error_current < PID_TEMP_THRES
           && PP->error_current > -PID_TEMP_THRES) {
    A0 = PP->proportion * (1 + PP->T / PP->integral + PP->differential / PP->T);
    A1 = -PP->proportion * (1 + 2 * PP->differential / PP->T);
    A2 = PP->proportion * (PP->differential / PP->T);
    result = A0 * PP->error_current + A1 * PP->error_last + A2 * PP->error_sum;
    result += PID_ZL;
    float pid_control_data = 0;
    pid_control_data =
        (PID_RESULT_MAX - result) / (PID_RESULT_MAX -
                                     PID_RESULT_MIN) * 65535.0f;
    //printf("A0 = %f \r\n", A0);
    //printf("A1 = %f \r\n", A1);
    //printf("A2 = %f \r\n", A2);
    //printf("control: %f \r\n", pid_control_data);
    pid_pwm_control(PWM_CONFIG, (uint16_t) pid_control_data);
  }

  PID_ZL = result;
  //printf("PID_ZL: %f\r\n", PID_ZL);

  PP->error_sum = PP->error_last;
  PP->error_last = PP->error_current;
  return result;
}

/*!
 * \brief Busy wait delay for us microseconds
 *
 * \note
 */
void lsm6dso_delay_us(uint32_t us)
{
  uint32_t cycles_per_us = HAL_RCC_GetSysClockFreq() / 10000000;
  volatile uint32_t i;

  for (i = 0; i < (us * cycles_per_us); i++) {
    ;
  }
}

void ConvertCoordinate(float vec[3], const uint8_t axis_order[3],
                       const uint8_t axis_sign[3])
{
  float val32[3];
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

/* Main Example --------------------------------------------------------------*/
int lsm6dso_publish_sensor_data(void *para)
{
  /* Read samples in polling mode (no int) */
  sensors_event_t sensor_data = { 0 };
  lsm6dso_status_reg_t reg;

  lsm6dso_status_reg_get(&dev_ctx, &reg);

  if (reg.xlda) {
    /* Read acceleration field data */
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
    lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
    acceleration_mg[0] =
        lsm6dso_from_fs8_to_mg(data_raw_acceleration.i16bit[0]);
    acceleration_mg[1] =
        lsm6dso_from_fs8_to_mg(data_raw_acceleration.i16bit[1]);
    acceleration_mg[2] =
        lsm6dso_from_fs8_to_mg(data_raw_acceleration.i16bit[2]);

    if (acc_gyro_init_parms->use_uart_mode == 1) {
      //set a flag to report acc data
      acc_data_report_in_uart_mode = true;
    }
    //apply axis transfer first
    ConvertCoordinate(acceleration_mg, g_imu_axis_order, g_imu_axis_sign);

    if (acc_activated) {
      //send related messages to SensorDataQ
      sensor_data.sensor_type = SENSOR_TYPE_ACCELEROMETER;
      sensor_data.accuracy = 3;
      sensor_data.timestamp = sensor_get_timestamp();
      sensor_data.sensor_data_t.vec.data[0] = acceleration_mg[0]
          * GRAVITY_EARTH / 1000.0f - acceleration_offset[0];
      sensor_data.sensor_data_t.vec.data[1] = acceleration_mg[1]
          * GRAVITY_EARTH / 1000.0f - acceleration_offset[1];
      sensor_data.sensor_data_t.vec.data[2] = acceleration_mg[2]
          * GRAVITY_EARTH / 1000.0f - acceleration_offset[2];
      sensor_data.sensor_data_t.vec.data[3] = temperature_degC;
      osMessageQueuePut(acc_gyro_init_parms->SensorDataQHandle,
                        &sensor_data, 0, 0);
    }
  }

  if (reg.gda) {
    /* Read angular rate field data */
    memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
    lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
    angular_rate_mdps[0] =
        lsm6dso_from_fs1000_to_mdps(data_raw_angular_rate.i16bit[0]);
    angular_rate_mdps[1] =
        lsm6dso_from_fs1000_to_mdps(data_raw_angular_rate.i16bit[1]);
    angular_rate_mdps[2] =
        lsm6dso_from_fs1000_to_mdps(data_raw_angular_rate.i16bit[2]);

    if (acc_gyro_init_parms->use_uart_mode) {
      //set a flag to report gyro data
      gyro_data_report_in_uart_mode = true;
    }
    //apply axis transfer first
    ConvertCoordinate(angular_rate_mdps, g_imu_axis_order, g_imu_axis_sign);

    if (gyro_activated) {
      //send related messages to SensorDataQ
      sensor_data.sensor_type = SENSOR_TYPE_GYROSCOPE;
      sensor_data.accuracy = 3;
      sensor_data.timestamp = sensor_get_timestamp();
      sensor_data.sensor_data_t.vec.data[0] = angular_rate_mdps[0]
          / 1000.0f * DPS2RPS - angular_rate_offset[0];
      sensor_data.sensor_data_t.vec.data[1] = angular_rate_mdps[1]
          / 1000.0f * DPS2RPS - angular_rate_offset[1];
      sensor_data.sensor_data_t.vec.data[2] = angular_rate_mdps[2]
          / 1000.0f * DPS2RPS - angular_rate_offset[2];
      sensor_data.sensor_data_t.vec.data[3] = temperature_degC;
      osMessageQueuePut(acc_gyro_init_parms->SensorDataQHandle,
                        &sensor_data, 0, 0);
    }
  }

  if (gyro_activated || acc_activated) {
    //lsm6dso_temp_flag_data_ready_get(&dev_ctx, &reg);
    if (reg.tda) {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lsm6dso_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC =
          lsm6dso_from_lsb_to_celsius(data_raw_temperature.i16bit);
      pid_temp_control(&imu_pid_struct, temperature_degC);
    }
  }

  if (gyro_data_report_in_uart_mode && acc_data_report_in_uart_mode) {
    //notify uart2 receive task to report sensor raw
    uint8_t uart_sensor_data[UART_TX_CODE_LENGTH] = { 0 };
    uint32_t currentu32timestamp = 0;
    currentu32timestamp = osKernelGetTickCount();

    uint16_t range_acc = 8000;
    uint16_t range_gyro = 1000;
    uart_sensor_data[0] = 0x5A;
    uart_sensor_data[1] = 0xA5;
    uart_sensor_data[2] = 0x18;
    uart_sensor_data[3] = 0x1;
    uart_sensor_data[4] = 0;
    memcpy(&uart_sensor_data[5], &currentu32timestamp, sizeof(uint32_t));
    memcpy(&uart_sensor_data[9], &range_acc, sizeof(uint16_t));
    memcpy(&uart_sensor_data[11], data_raw_acceleration.i16bit,
           sizeof(uint16_t) * 3);
    memcpy(&uart_sensor_data[17], &range_gyro, sizeof(uint16_t));
    memcpy(&uart_sensor_data[19], data_raw_angular_rate.i16bit,
           sizeof(uint16_t) * 3);
    memcpy(&uart_sensor_data[25], data_raw_temperature.i16bit,
           sizeof(uint16_t));

    osMessageQueuePut(acc_gyro_init_parms->UartReportQHandle,
                      uart_sensor_data, 0, 0);

    gyro_data_report_in_uart_mode = false;
    acc_data_report_in_uart_mode = false;
  }
}

int lsm6dso_publish_fake_sensor_data(void *para) {
	return SENSOR_SUCCESS;
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t * bufp,
                              uint16_t len)
{
  if (handle == &acc_gyro_init_parms->i2c_handle) {
    //printf("start i2c write, \r\n");
    HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_L, reg,
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
  HAL_StatusTypeDef ret = HAL_OK;
  if (handle == &acc_gyro_init_parms->i2c_handle) {
    //printf("start i2c read, \r\n");
    ret = HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    lsm6dso_delay_us(2);
    if(ret != HAL_OK) {
      //notify host to do reset!
      sensor_message_event_t sensor_msg_event;
      sensor_msg_event.message_event_type = SENSOR_BUS_ERROR_EVENT;
      sensor_msg_event.message_event_t.resp_event.sensor_type = SENSOR_TYPE_ACCELEROMETER;
      sensor_msg_event.message_event_t.resp_event.config_type = SENSOR_BUS_ERROR_CONFIG;
      //notifiy smgr that bus is abnormal
      osMessageQueuePut(acc_gyro_init_parms->SensorMessageQHandle,
                          &sensor_msg_event, 0, 0);
      osDelay(1);
    }
  }

  return 0;
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
int lsm6dso_acc_init(void *para1, void *para2)
{
  acc_gyro_init_parms = (platform_prams *) para1;
  /* Initialize mems driver interface */

  if (acc_gyro_init_parms != NULL) {
    tim1 = &acc_gyro_init_parms->PWMTimerHandle;
  } else {
    Error_Handler();
  }

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &acc_gyro_init_parms->i2c_handle;

  /* Check device ID */
  lsm6dso_device_id_get(&dev_ctx, &whoamI);
  printf("lsm6dso whoamI: 0x%x \r\n", whoamI);

  if (whoamI != LSM6DSO_ID) {
    printf("lsm6dso acc init failed!\r\n");
    return SENSOR_FAILED;
  }

  /* Restore default configuration */
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(&dev_ctx, &rst);
  } while (rst);

  lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);
  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
  /* Set Output Data Rate */
  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);
  lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_104Hz);
  /* Set full scale */
  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_8g);
  lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_1000dps);
  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
  lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  /* axis conversion parameter */
  g_imu_axis_order[0] = ST_CUSTOM_IMU_AXIS_ORDER_X;
  g_imu_axis_order[1] = ST_CUSTOM_IMU_AXIS_ORDER_Y;
  g_imu_axis_order[2] = ST_CUSTOM_IMU_AXIS_ORDER_Z;
  g_imu_axis_sign[0] = ST_CUSTOM_IMU_AXIS_SIGN_X;
  g_imu_axis_sign[1] = ST_CUSTOM_IMU_AXIS_SIGN_Y;
  g_imu_axis_sign[2] = ST_CUSTOM_IMU_AXIS_SIGN_Z;

  return SENSOR_SUCCESS;
}

int lsm6dso_gyro_init(void *para1, void *para2)
{
  if (whoamI != LSM6DSO_ID) {
    printf("lsm6dso gyro init failed!\r\n");
    return SENSOR_FAILED;
  }

  return SENSOR_SUCCESS;
}

static int32_t lsm6dso_enable(uint8_t SENSOR_TYPE)
{
  int32_t ret = 0;

  if (SENSOR_TYPE == SENSOR_TYPE_ACCELEROMETER && acc_activate_count == 0) {
    acc_activate_count++;
    acc_activated = true;
  } else if (SENSOR_TYPE == SENSOR_TYPE_ACCELEROMETER)
    acc_activate_count++;

  if (SENSOR_TYPE == SENSOR_TYPE_GYROSCOPE && gyro_activate_count == 0) {
    gyro_activate_count++;
    gyro_activated = true;
  } else if (SENSOR_TYPE == SENSOR_TYPE_GYROSCOPE)
    gyro_activate_count++;

  if ((acc_activate_count == 1 && gyro_activate_count == 0)
      || (gyro_activate_count == 1 && acc_activate_count == 0)) {
    ret = lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    PID_Init(&imu_pid_struct, PID_TARGET_TEMP);
    pid_pwm_control(PWM_ACTIVATE, 0);
  }

  return ret;
}

static int32_t lsm6dso_disable(uint8_t SENSOR_TYPE)
{
  int32_t ret = 0;
  if (SENSOR_TYPE == SENSOR_TYPE_ACCELEROMETER && acc_activate_count > 0) {
    acc_activate_count--;
  }

  if (SENSOR_TYPE == SENSOR_TYPE_GYROSCOPE && gyro_activate_count > 0) {
    gyro_activate_count--;
  }

  if (gyro_activate_count == 0)
    gyro_activated = false;

  if (acc_activate_count == 0)
    acc_activated = false;

  if (acc_activate_count == 0 && gyro_activate_count == 0) {
    ret = lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);
    pid_pwm_control(PWM_DEACTIVATE, 0);
  }
  return ret;
}

int lsm6dso_acc_activate(bool activate)
{
  int res = 0;
  printf("lsm6dso_acc_activate: %d \r\n", activate);
  if (activate) {
    res = lsm6dso_enable((uint8_t) SENSOR_TYPE_ACCELEROMETER);
  } else {
    res = lsm6dso_disable((uint8_t) SENSOR_TYPE_ACCELEROMETER);
  }
  return res;
}

int lsm6dso_gyro_activate(bool activate)
{
  int res = 0;
  printf("lsm6dso_gyro_activate: %d \r\n", activate);
  if (activate) {
    res = lsm6dso_enable((uint8_t) SENSOR_TYPE_GYROSCOPE);
  } else {
    res = lsm6dso_disable((uint8_t) SENSOR_TYPE_GYROSCOPE);
  }
  return res;
}

int lsm6dso_init_complete(void *para)
{
  //send init complete event to smgr?
  return SENSOR_SUCCESS;
}

int lsm6dso_acc_config(uint8_t config_type, void *para)
{
  int res = 0;
  sensors_event_t sensor_data = { 0 };

  switch (config_type) {
  case SENSOR_CONFIG_SELFTEST:
    res = lsm6dso_acc_self_test(&dev_ctx);
    break;
  case SENSOR_CONFIG_CALIBRATION:
    printf("acc cali message \r\n");
    res = lsm6dso_acc_calibration(&dev_ctx, acceleration_offset);
    if (res == 0) {
      memcpy(acc_gyro_init_parms->board_calidata->acc_bias, acceleration_offset,
             sizeof(float[3]));
      if (acc_gyro_init_parms->board_calidata->acc_cali_ver != DEFAULT_VERSION)
        acc_gyro_init_parms->board_calidata->acc_cali_ver++;
      else
        acc_gyro_init_parms->board_calidata->acc_cali_ver = 1;
    }
    break;
  case SENSOR_CONFIG_BIAS:
    if (acc_gyro_init_parms->board_calidata->acc_cali_ver != DEFAULT_VERSION) {
      memcpy(acceleration_offset, acc_gyro_init_parms->board_calidata->acc_bias,
             sizeof(float[3]));
      printf("acc bias data: {%f, %f, %f} \r\n", acceleration_offset[0],
             acceleration_offset[1], acceleration_offset[2]);
    }
    break;
  case SENSOR_CALIBRATION_RESULT:
    sensor_data.sensor_type = SENSOR_TYPE_ACCELEROMETER;
    sensor_data.accuracy = 0xFF;
    sensor_data.timestamp = osKernelGetTickCount();
    sensor_data.sensor_data_t.vec.data[0] = acceleration_offset[0];
    sensor_data.sensor_data_t.vec.data[1] = acceleration_offset[1];
    sensor_data.sensor_data_t.vec.data[2] = acceleration_offset[2];
    osMessageQueuePut(acc_gyro_init_parms->SensorDataQHandle,
                      &sensor_data, 0, 0);
    break;
  default:
    break;
  }

  return res;
}

int lsm6dso_gyro_config(uint8_t config_type, void *para)
{
  int res = 0;
  sensors_event_t sensor_data = { 0 };

  switch (config_type) {
  case SENSOR_CONFIG_SELFTEST:
    res = lsm6dso_gyro_self_test(&dev_ctx);
    break;
  case SENSOR_CONFIG_CALIBRATION:
    printf("gyro cali message \r\n");
    res = lsm6dso_gyro_calibration(&dev_ctx, angular_rate_offset);
    if (res == 0) {
      memcpy(acc_gyro_init_parms->board_calidata->gyro_bias,
             angular_rate_offset, sizeof(float[3]));
      if (acc_gyro_init_parms->board_calidata->gyr_cali_ver != DEFAULT_VERSION)
        acc_gyro_init_parms->board_calidata->gyr_cali_ver++;
      else
        acc_gyro_init_parms->board_calidata->gyr_cali_ver = 1;
    }
    break;
  case SENSOR_CONFIG_BIAS:
    if (acc_gyro_init_parms->board_calidata->gyr_cali_ver != DEFAULT_VERSION) {
      memcpy(angular_rate_offset,
             acc_gyro_init_parms->board_calidata->gyro_bias, sizeof(float[3]));
      printf("gyro bias data: {%f, %f, %f} \r\n", angular_rate_offset[0],
             angular_rate_offset[1], angular_rate_offset[2]);
    }
    break;
  case SENSOR_CALIBRATION_RESULT:
    sensor_data.sensor_type = SENSOR_TYPE_GYROSCOPE;
    sensor_data.accuracy = 0xFF;
    sensor_data.timestamp = sensor_get_timestamp();
    sensor_data.sensor_data_t.vec.data[0] = angular_rate_offset[0];
    sensor_data.sensor_data_t.vec.data[1] = angular_rate_offset[1];
    sensor_data.sensor_data_t.vec.data[2] = angular_rate_offset[2];
    osMessageQueuePut(acc_gyro_init_parms->SensorDataQHandle,
                      &sensor_data, 0, 0);
    break;
  default:
    break;
  }
  return res;
}

int lsm6dso_publish_config_resp(void *para)
{
  //send calibration result to AP using this function
  return SENSOR_SUCCESS;
}

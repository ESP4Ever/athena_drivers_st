/*
 ******************************************************************************
 * @file    lsm6dso_self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements the self test procedure.
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
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */
//#define STEVAL_MKI109V3
#define NUCLEO_F411RE

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE)
/* NUCLEO_F411RE: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <lsm6dso_reg.h>
#include "sensor_base.h"
#define MAX_CAL_COL 3

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

extern uint8_t g_imu_axis_order[3];
extern uint8_t g_imu_axis_sign[3];
extern void ConvertCoordinate(float vec[3], const uint8_t axis_order[3],
                              const uint8_t axis_sign[3]);

extern void pid_pwm_control(uint8_t command, uint16_t data);
extern void PID_Init(PID * Pid, float SETtemp);
extern float pid_temp_control(PID * PP, float current_temp);
extern PID imu_pid_struct;

/* Private macro -------------------------------------------------------------*/
/* Self test limits. */
#define    MIN_ST_LIMIT_mg        50.0f
#define    MAX_ST_LIMIT_mg      1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Private variables ---------------------------------------------------------*/
#define GRAVITY_EARTH		   (9.80665f)

#define SKIP_COUNT 300
#define SKIP_COUNT_GYRO 3
enum cal_state_e {
  CAL_STATE_START = 0,
  CAL_STATE_UNKNOWN,
  CAL_STATE_SUCCESS,
  CAL_STATE_MOVING,
  CAL_STATE_NO_VAR,
  CAL_STATE_HI_BIAS
};

/* calibration configuration */
struct cal_config_t {
  int32_t cols;                 // columms or axes, maximum 3
  int32_t num_samples;          // number of samples
  float variance_threshold;     // stationary detect variance threshold
  bool check_zero_variance;
  float bias_thresholds[MAX_CAL_COL];
  int32_t count;
};

/* calibration dynamic data */
struct cal_dynm_t {
  float variance[MAX_CAL_COL];  // sensor variance
  float sample_sqsum[MAX_CAL_COL];      // sum of square of sensor sample data
  float sample_sum[MAX_CAL_COL];        // sum of sensor sample data
  int32_t sample_count;         // count of sensor samples
};

typedef struct _sensor_info {
  float offset[MAX_CAL_COL];
  enum cal_state_e cal_state;
  struct cal_config_t cal_config;
  struct cal_dynm_t cal_dynm_data;
} sensor_info_type;
/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static enum cal_state_e cal_process_sample(sensor_info_type * cal_data,
                                           float *input)
{
  uint8_t i;

  if (cal_data->cal_state != CAL_STATE_START) {
    return CAL_STATE_UNKNOWN;
  }

  if (cal_data->cal_dynm_data.sample_count == 0) {
    for (i = 0; i < cal_data->cal_config.cols; i++) {
      cal_data->cal_dynm_data.sample_sum[i] = 0;
      cal_data->cal_dynm_data.sample_sqsum[i] = 0;
      cal_data->cal_dynm_data.variance[i] = 0;
    }
  }

  cal_data->cal_dynm_data.sample_count++;

  for (i = 0; i < cal_data->cal_config.cols; i++) {
    cal_data->cal_dynm_data.sample_sum[i] += input[i];
    cal_data->cal_dynm_data.sample_sqsum[i] += (input[i] * input[i]);
  }

  if (cal_data->cal_dynm_data.sample_count == cal_data->cal_config.num_samples) {
    float varT;

    for (i = 0; i < cal_data->cal_config.cols; i++) {
      varT = (cal_data->cal_dynm_data.sample_sum[i]
              * cal_data->cal_dynm_data.sample_sum[i]);

      cal_data->cal_dynm_data.variance[i] =
          (cal_data->cal_dynm_data.sample_sqsum[i]
           - (varT / cal_data->cal_config.num_samples))
          / cal_data->cal_config.num_samples;

      if (cal_data->cal_dynm_data.variance[i]
          > cal_data->cal_config.variance_threshold) {
        printf
            ("Variance exceeded, restarting algorithm. index: %i; variance: %9.4f; variance threshold: %9.4f",
             i, cal_data->cal_dynm_data.variance[i],
             cal_data->cal_config.variance_threshold);
        //indicate motion state detected, reset algorithm state
        cal_data->cal_dynm_data.sample_count = 0;
        return CAL_STATE_MOVING;
      } else if ((0 == cal_data->cal_dynm_data.variance[i])
                 && cal_data->cal_config.check_zero_variance) {
        printf
            ("Zero variance found, restarting algorithm. index: %i; variance: %9.4f",
             i, cal_data->cal_dynm_data.variance[i]);

        cal_data->cal_dynm_data.sample_count = 0;
        return CAL_STATE_NO_VAR;
      } else if (fabsf(cal_data->cal_dynm_data.sample_sum[i]
                       / cal_data->cal_config.num_samples)
                 > cal_data->cal_config.bias_thresholds[i]) {
        printf
            ("High BIAS found, restarting algorithm. index: %i; bias: %9.4f, threshold: %9.4f",
             i, cal_data->cal_dynm_data.sample_sum[i]
             / cal_data->cal_config.num_samples,
             cal_data->cal_config.bias_thresholds[i]);

        cal_data->cal_dynm_data.sample_count = 0;
        return CAL_STATE_HI_BIAS;
      }
    }

    for (i = 0; i < cal_data->cal_config.cols; i++) {
      cal_data->offset[i] = cal_data->cal_dynm_data.sample_sum[i]
          / cal_data->cal_config.num_samples;
    }

    cal_data->cal_state = CAL_STATE_SUCCESS;
    return CAL_STATE_SUCCESS;
  }

  return CAL_STATE_UNKNOWN;
}

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static void platform_delay(uint32_t ms);

/* Main function --------------------------------------------------------------*/
static void lsm6dso_reinit_device(stmdev_ctx_t * ctx)
{
  uint8_t rst;

  lsm6dso_reset_set(ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(ctx, &rst);
  } while (rst);

  lsm6dso_block_data_update_set(ctx, PROPERTY_DISABLE);
  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(ctx, LSM6DSO_I3C_DISABLE);
  /* Set Output Data Rate */
  lsm6dso_xl_data_rate_set(ctx, LSM6DSO_XL_ODR_104Hz);
  lsm6dso_gy_data_rate_set(ctx, LSM6DSO_GY_ODR_104Hz);
  /* Set full scale */
  lsm6dso_xl_full_scale_set(ctx, LSM6DSO_8g);
  lsm6dso_gy_full_scale_set(ctx, LSM6DSO_1000dps);
  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  lsm6dso_xl_hp_path_on_out_set(ctx, LSM6DSO_LP_ODR_DIV_100);
  lsm6dso_xl_filter_lp2_set(ctx, PROPERTY_ENABLE);
}

int32_t lsm6dso_acc_self_test(stmdev_ctx_t * ctx)
{
  axis3bit16_t data_raw;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;

  /* Restore default configuration */
  lsm6dso_reset_set(ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(ctx, LSM6DSO_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(ctx, PROPERTY_ENABLE);

  /*
   * Accelerometer Self Test
   */
  /* Set Output Data Rate */
  lsm6dso_xl_data_rate_set(ctx, LSM6DSO_XL_ODR_52Hz);
  /* Set full scale */
  lsm6dso_xl_full_scale_set(ctx, LSM6DSO_4g);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dso_xl_flag_data_ready_get(ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dso_acceleration_raw_get(ctx, data_raw.u8bit);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dso_xl_flag_data_ready_get(ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dso_acceleration_raw_get(ctx, data_raw.u8bit);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dso_from_fs4_to_mg(data_raw.i16bit[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dso_xl_self_test_set(ctx, LSM6DSO_XL_ST_NEGATIVE);
  //lsm6dso_xl_self_test_set(ctx, LSM6DSO_XL_ST_POSITIVE);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dso_xl_flag_data_ready_get(ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dso_acceleration_raw_get(ctx, data_raw.u8bit);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dso_xl_flag_data_ready_get(ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dso_acceleration_raw_get(ctx, data_raw.u8bit);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dso_from_fs4_to_mg(data_raw.i16bit[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if ((MIN_ST_LIMIT_mg > test_val[i])
        || (test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dso_xl_self_test_set(ctx, LSM6DSO_XL_ST_DISABLE);
  /* Disable sensor. */
  lsm6dso_xl_data_rate_set(ctx, LSM6DSO_XL_ODR_OFF);

  lsm6dso_reinit_device(ctx);

  printf("lsm6dso_acc_self_test result: %d\r\n", st_result);
  if (st_result == ST_PASS) {
    printf("lsm6dso_acc_self_test success\r\n");
    return 0;
  } else {
    printf("lsm6dso_acc_self_test failed\r\n");
    return -1;
  }
}

int32_t lsm6dso_gyro_self_test(stmdev_ctx_t * ctx)
{
  axis3bit16_t data_raw;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;

  /* Restore default configuration */
  lsm6dso_reset_set(ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(ctx, LSM6DSO_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(ctx, PROPERTY_ENABLE);

  /*
   * Gyroscope Self Test
   */
  /* Set Output Data Rate */
  lsm6dso_gy_data_rate_set(ctx, LSM6DSO_GY_ODR_208Hz);
  /* Set full scale */
  lsm6dso_gy_full_scale_set(ctx, LSM6DSO_2000dps);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dso_gy_flag_data_ready_get(ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dso_angular_rate_raw_get(ctx, data_raw.u8bit);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dso_xl_flag_data_ready_get(ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dso_angular_rate_raw_get(ctx, data_raw.u8bit);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dso_from_fs2000_to_mdps(data_raw.i16bit[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dso_gy_self_test_set(ctx, LSM6DSO_GY_ST_POSITIVE);
  //lsm6dso_gy_self_test_set(ctx, LIS2DH12_GY_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(100);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dso_xl_flag_data_ready_get(ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dso_angular_rate_raw_get(ctx, data_raw.u8bit);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dso_from_fs2000_to_mdps(data_raw.i16bit[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if ((MIN_ST_LIMIT_mdps > test_val[i])
        || (test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dso_gy_self_test_set(ctx, LSM6DSO_GY_ST_DISABLE);
  /* Disable sensor. */
  lsm6dso_xl_data_rate_set(ctx, LSM6DSO_GY_ODR_OFF);

  lsm6dso_reinit_device(ctx);

  printf("lsm6dso_gyro_self_test result: %d\r\n", st_result);
  if (st_result == ST_PASS) {
    printf("lsm6dso_gyro_self_test success\r\n");
    return 0;
  } else {
    printf("lsm6dso_gyro_self_test failed\r\n");
    return -1;
  }
}

int32_t lsm6dso_acc_calibration(stmdev_ctx_t * ctx, float *offset)
{
  uint8_t reg;
  axis3bit16_t data_raw_acceleration;
  axis1bit16_t data_raw_temperature;
  float temperature_degC;
  float acceleration[3];
  sensor_info_type acc_info;

  acc_info.cal_config.cols = 3;
  acc_info.cal_config.num_samples = 50;
  acc_info.cal_config.variance_threshold = 10.0;
  acc_info.cal_config.check_zero_variance = true;
  acc_info.cal_config.bias_thresholds[0] = 2.0;
  acc_info.cal_config.bias_thresholds[1] = 2.0;
  acc_info.cal_config.bias_thresholds[2] = 2.0;
  acc_info.cal_config.count = 0;
  acc_info.cal_dynm_data.sample_count = 0;
  acc_info.cal_state = CAL_STATE_START;

  lsm6dso_reinit_device(ctx);
  lsm6dso_block_data_update_set(ctx, PROPERTY_ENABLE);
  PID_Init(&imu_pid_struct, PID_TARGET_TEMP);
  pid_pwm_control(PWM_ACTIVATE, 0);

  platform_delay(20);

  do {
    //printf("reading acc data\r\n");
    lsm6dso_xl_flag_data_ready_get(ctx, &reg);
    if (reg) {
      /* Read acceleration field data */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      lsm6dso_acceleration_raw_get(ctx, data_raw_acceleration.u8bit);
      acceleration[0] =
          lsm6dso_from_fs8_to_mg(data_raw_acceleration.i16bit[0]) *
          GRAVITY_EARTH / 1000.0f;
      acceleration[1] =
          lsm6dso_from_fs8_to_mg(data_raw_acceleration.i16bit[1]) *
          GRAVITY_EARTH / 1000.0f;
      acceleration[2] =
          lsm6dso_from_fs8_to_mg(data_raw_acceleration.i16bit[2]) *
          GRAVITY_EARTH / 1000.0f;

      printf("%d, {%f, %f, %f}\r\n", acc_info.cal_config.count,
             acceleration[0], acceleration[1], acceleration[2]);

      //apply axis transfer first
      ConvertCoordinate(acceleration, g_imu_axis_order, g_imu_axis_sign);

      printf("%d, {%f, %f, %f}\r\n", acc_info.cal_config.count,
             acceleration[0], acceleration[1], acceleration[2]);

      //fill data to data array
      acceleration[2] = acceleration[2] - GRAVITY_EARTH;
      if (acc_info.cal_config.count > SKIP_COUNT)
        cal_process_sample(&acc_info, acceleration);
      acc_info.cal_config.count++;
    }

    lsm6dso_temp_flag_data_ready_get(ctx, &reg);
    if (reg) {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lsm6dso_temperature_raw_get(ctx, data_raw_temperature.u8bit);
      temperature_degC =
          lsm6dso_from_lsb_to_celsius(data_raw_temperature.i16bit);
      pid_temp_control(&imu_pid_struct, temperature_degC);
    }
    platform_delay(10);
  } while ((acc_info.cal_config.count < SKIP_COUNT * 2)
           && (acc_info.cal_state != CAL_STATE_SUCCESS));

  pid_pwm_control(PWM_DEACTIVATE, 0);

  if (acc_info.cal_state == CAL_STATE_SUCCESS) {
    printf("acc calibration successed!\r\n");
    memcpy(offset, acc_info.offset, sizeof(float) * 3);
    return 0;
  }

  return -1;
}

int32_t lsm6dso_gyro_calibration(stmdev_ctx_t * ctx, float *offset)
{
  uint8_t reg;
  axis3bit16_t data_raw_angular_rate;
  axis1bit16_t data_raw_temperature;
  float temperature_degC;
  float gyroscope[3];
  sensor_info_type gyro_info;

  gyro_info.cal_config.cols = 3;
  gyro_info.cal_config.num_samples = 100;
  gyro_info.cal_config.variance_threshold = 10.0;
  gyro_info.cal_config.check_zero_variance = true;
  gyro_info.cal_config.bias_thresholds[0] = 1.0;
  gyro_info.cal_config.bias_thresholds[1] = 1.0;
  gyro_info.cal_config.bias_thresholds[2] = 1.0;
  gyro_info.cal_config.count = 0;
  gyro_info.cal_dynm_data.sample_count = 0;
  gyro_info.cal_state = CAL_STATE_START;

  lsm6dso_reinit_device(ctx);
  lsm6dso_block_data_update_set(ctx, PROPERTY_ENABLE);

  PID_Init(&imu_pid_struct, PID_TARGET_TEMP);
  pid_pwm_control(PWM_ACTIVATE, 0);

  platform_delay(20);

  do {
    //printf("reading gyro data\r\n");
    lsm6dso_gy_flag_data_ready_get(ctx, &reg);
    if (reg) {
      /* Read angular rate field data */
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
      lsm6dso_angular_rate_raw_get(ctx, data_raw_angular_rate.u8bit);
      gyroscope[0] =
          lsm6dso_from_fs1000_to_mdps(data_raw_angular_rate.i16bit[0]) /
          1000.0f * DPS2RPS;
      gyroscope[1] =
          lsm6dso_from_fs1000_to_mdps(data_raw_angular_rate.i16bit[1]) /
          1000.0f * DPS2RPS;
      gyroscope[2] =
          lsm6dso_from_fs1000_to_mdps(data_raw_angular_rate.i16bit[2]) /
          1000.0f * DPS2RPS;

      printf("%d, {%f, %f, %f}\r\n", gyro_info.cal_config.count,
             gyroscope[0], gyroscope[1], gyroscope[2]);
      //apply axis transfer first
      ConvertCoordinate(gyroscope, g_imu_axis_order, g_imu_axis_sign);

      //fill data to data array
      printf("%d, {%f, %f, %f}\r\n", gyro_info.cal_config.count,
             gyroscope[0], gyroscope[1], gyroscope[2]);
      if (gyro_info.cal_config.count > SKIP_COUNT_GYRO)
        cal_process_sample(&gyro_info, gyroscope);
      gyro_info.cal_config.count++;

      lsm6dso_temp_flag_data_ready_get(ctx, &reg);
      if (reg) {
        /* Read temperature data */
        memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
        lsm6dso_temperature_raw_get(ctx, data_raw_temperature.u8bit);
        temperature_degC =
            lsm6dso_from_lsb_to_celsius(data_raw_temperature.i16bit);
        pid_temp_control(&imu_pid_struct, temperature_degC);
      }
    }
    platform_delay(10);
  } while ((gyro_info.cal_config.count < SKIP_COUNT * 2)
           && (gyro_info.cal_state != CAL_STATE_SUCCESS));

  pid_pwm_control(PWM_DEACTIVATE, 0);

  if (gyro_info.cal_state == CAL_STATE_SUCCESS) {
    printf("gyro calibration successed!\r\n");
    memcpy(offset, gyro_info.offset, sizeof(float) * 3);
    return 0;
  }

  return -1;
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

#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include "stdio.h"

typedef struct {
  uint8_t shape_cali_data[316];
  uint8_t grid_cali_data[600];
  uint8_t cali_times;
} tof_cali_data;

typedef struct calibrationData {
  float acc_bias[3];
  uint32_t acc_cali_ver;
  float gyro_bias[3];
  uint32_t gyr_cali_ver;
  float lux_scale;
  uint32_t light_cali_ver;
  tof_cali_data tof_cali_t;
  uint32_t tof_cali_ver;
} calibrationData;

#endif

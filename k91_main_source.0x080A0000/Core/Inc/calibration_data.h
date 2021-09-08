#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include "stdio.h"

#define DEFAULT_VERSION 0xFFFFFFFF

typedef struct {
  uint8_t shape_cali_data[316];
  uint8_t grid_cali_data[600];
  uint32_t max_cross_talk;
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

int update_calibration_data_to_flash(calibrationData * cali_data);
#endif

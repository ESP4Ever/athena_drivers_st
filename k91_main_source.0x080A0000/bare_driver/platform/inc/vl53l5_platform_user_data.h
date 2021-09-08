#ifndef _VL53L5_PLATFORM_USER_DATA_H_
#define _VL53L5_PLATFORM_USER_DATA_H_

#include "vl53l5_device.h"
#include "vl53l5_types.h"

/**
 * @file   vl53l5_platform_user_data.h
 *
 * @brief  All end user OS/platform/application porting
 */

/** @brief  Contains the current state and internal values of the API
 */

#ifdef __cplusplus
extern "C" {
#endif

  enum vl53l5_comms_type {
    VL53L5_I2C = 0,
    VL53L5_SPI
  };

  struct vl53l5_range_data_t {
    struct vl53l5_range_results_t core;
  };

#ifdef VL53L5_CALIBRATION_DECODE_ON
  struct vl53l5_calibration_data_t {
    struct vl53l5_calibration_dev_t core;
  };
#endif

  struct vl53l5_dev_handle_t {
    uint8_t bus_address;
    uint8_t comms_type;         /* TODO: remove comms type from delivery */
    uint32_t comms_speed_khz;   /* TODO: remove comms type from delivery */

    /* Per-device GPIO options */
    //uint8_t gpio_lpn; /* TODO: change to gpio_low_power (check match with datasheet) */
    uint8_t gpio_lpn;           /* TODO: change to gpio_low_power (check match with datasheet) */
    uint8_t gpio_gpio1;
    uint8_t gpio_gpio2;
    uint8_t gpio_comms_select;
    void * spi_handle;
    uint64_t (*timer_handle)(void);
    void * tof_cs_gpio_port;
    uint16_t* tof_cs_pin;
    /* Set to true to override deletion of trace log files */
    bool keep_all_tracefiles;

    struct vl53l5_dev_handle_internal_t host_dev;
  };

#ifdef __cplusplus
}
#endif
#endif

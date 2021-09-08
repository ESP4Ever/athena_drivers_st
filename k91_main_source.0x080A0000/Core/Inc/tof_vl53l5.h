/*
 * tof_vl53l5.h
 *
 *  Created on: Dec 22, 2020
 *      Author: sangweilin@xiaomi.com
 */

#ifndef INC_TOF_VL53L5_H_
#define INC_TOF_VL53L5_H_

#include "vl53l5_platform_algo_limits.h"
#include "vl53l5_api_core.h"
#include "vl53l5_api_ranging.h"
#include "vl53l5_api_range_decode.h"

#include "vl53l5_platform_init.h"
#include "vl53l5_platform.h"

#include "vl53l5_fw_data.h"
#include "vl53l5_customer_config_data.h"

typedef int32_t(*stmdev_write_ptr) (void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t(*stmdev_read_ptr) (void *, uint8_t, uint8_t *, uint16_t);

typedef struct {
        /** Component mandatory fields **/
  stmdev_write_ptr write_reg;
  stmdev_read_ptr read_reg;
        /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

#define INITIAL_PAGE_ID 0x7FFF
#define TOF_DEVICE_ID   0x0000
#define TOF_REVISION_ID 0x0001
#define TOF_DEVICE_ID_VALUE 0xf0
#define TOF_REVISION_ID_VALUE 0x2

int32_t send_device_config(struct vl53l5_dev_handle_t *pdev);
int32_t ranging_loop(struct vl53l5_dev_handle_t *pdev);

#endif /* INC_TOF_VL53L5_H_ */

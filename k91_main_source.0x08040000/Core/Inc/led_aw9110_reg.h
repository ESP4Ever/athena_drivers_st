/*
 * led_reg.h
 *
 *  Created on: Dec 1, 2020
 *      Author: sangweilin@xiaomi.com
 */

#ifndef INC_LED_AW9110_REG_H_
#define INC_LED_AW9110_REG_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>

typedef int32_t(*stmdev_write_ptr) (void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t(*stmdev_read_ptr) (void *, uint8_t, uint8_t *, uint16_t);

typedef struct {
        /** Component mandatory fields **/
  stmdev_write_ptr write_reg;
  stmdev_read_ptr read_reg;
        /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5

#define REG_INPUT_P0        0x00
#define REG_INPUT_P1        0x01
#define REG_OUTPUT_P0       0x02
#define REG_OUTPUT_P1       0x03
#define REG_CONFIG_P0       0x04
#define REG_CONFIG_P1       0x05
#define REG_INT_P0          0x06
#define REG_INT_P1          0x07
#define REG_ID              0x10
#define REG_CTRL            0x11
#define REG_WORK_MODE_P0    0x12
#define REG_WORK_MODE_P1    0x13
#define REG_EN_BREATH       0x14
#define REG_FADE_TIME       0x15
#define REG_FULL_TIME       0x16
#define REG_DLY0_BREATH     0x17
#define REG_DLY1_BREATH     0x18
#define REG_DLY2_BREATH     0x19
#define REG_DLY3_BREATH     0x1a
#define REG_DLY4_BREATH     0x1b
#define REG_DLY5_BREATH     0x1c
#define REG_DIM00           0x20
#define REG_DIM01           0x21
#define REG_DIM02           0x22
#define REG_DIM03           0x23
#define REG_DIM04           0x24
#define REG_DIM05           0x25
#define REG_DIM06           0x26
#define REG_DIM07           0x27
#define REG_DIM08           0x28
#define REG_DIM09           0x29
#define REG_SWRST           0x7F

/* aw9110 register read/write access*/
#define REG_NONE_ACCESS                 0
#define REG_RD_ACCESS                   1 << 0
#define REG_WR_ACCESS                   1 << 1
#define AW9110_REG_MAX                  0xFF

const unsigned char aw9110_reg_access[AW9110_REG_MAX] = {
  [REG_INPUT_P0] = REG_RD_ACCESS,
  [REG_INPUT_P1] = REG_RD_ACCESS,
  [REG_OUTPUT_P0] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_OUTPUT_P1] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_CONFIG_P0] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_CONFIG_P1] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_INT_P0] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_INT_P1] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_ID] = REG_RD_ACCESS,
  [REG_CTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_WORK_MODE_P0] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_WORK_MODE_P1] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_EN_BREATH] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_FADE_TIME] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_FULL_TIME] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_DLY0_BREATH] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_DLY1_BREATH] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_DLY2_BREATH] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_DLY3_BREATH] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_DLY4_BREATH] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_DLY5_BREATH] = REG_RD_ACCESS | REG_WR_ACCESS,
  [REG_DIM00] = REG_WR_ACCESS,
  [REG_DIM01] = REG_WR_ACCESS,
  [REG_DIM02] = REG_WR_ACCESS,
  [REG_DIM03] = REG_WR_ACCESS,
  [REG_DIM04] = REG_WR_ACCESS,
  [REG_DIM05] = REG_WR_ACCESS,
  [REG_DIM06] = REG_WR_ACCESS,
  [REG_DIM07] = REG_WR_ACCESS,
  [REG_DIM08] = REG_WR_ACCESS,
  [REG_DIM09] = REG_WR_ACCESS,
  [REG_SWRST] = REG_WR_ACCESS,
};

#define max_brightness 255
#define MAX_I2C_BUFFER_SIZE 65536
#define HEAD_LED_I2C_COMUNICATE_ADD (0xb0)
#define REAR_LED_I2C_COMUNICATE_ADD (0xb6)
#define AW9110_ID 0x23

enum led_brightness {
  LED_OFF = 0,
  LED_ON = 1,
  LED_HALF = 127,
  LED_FULL = 255,
};

struct aw9110B {
  int imax;                     //:3;
  int rise_time;                //:2;
  int on_time;                  //:2;
  int fall_time;                //:2;
  int off_time;                 //:2;
};

#endif /* INC_LED_AW9110_REG_H_ */

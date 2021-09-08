/*
 * bf2092_reg.h
 *
 *  Created on: Dec 1, 2020
 *      Author: sangweilin@xiaomi.com
 */

#ifndef INC_BF2092_REG_H_
#define INC_BF2092_REG_H_

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

#define BF2092_I2C_COMUNICATE_ADD (0xdc)

#define BF2092_REG_PART_ID_0 (0xfc)
#define BF2092_REG_PART_ID_1 (0xfd)

#define BF2092_REG_PART_ID_0_VALUE (0x20)
#define BF2092_REG_PART_ID_1_VALUE (0x92)

#define IMAGE_Q_REG (0x57)
#define LIGHT_Q_REG (0xa1)

#define DATA_READY_REG (0x53)

#define DATA_X_REG (0x54)
#define DATA_Y_REG (0x55)

#define RESET_REG (0xf2)

#define ENABLE_REG (0xe7)

/* INITIAL Reg and value
0x07,0x00
0xf1,0x03
0xe0,0xA5
0xe2,0x44
0xe3,0x24
0xe4,0x09
0xe5,0x7c
0xe6,0x7e
0xe7,0xc0
0x18,0x12
0x66,0x23
0x6c,0x24
0x6d,0x25
0xb3,0x06
0xd3,0x00
0xd4,0xe4
0x5c,0x08
0x01,0x40
0x2a,0xb3
0xa6,0x02
0xa8,0xd6
0xa9,0xd6
0x05,0xE4
0xB9,0xFF
*/
#define COM7 (0x07)
#define ISPBYPS (0xf1)
#define REG_CTR0 (0xe0)
#define REG_CTR2 (0xe2)
#define REG_CTR3 (0xe3)
#define REG_CTR4 (0xe4)
#define REG_CTR5 (0xe5)
#define REG_CTR6 (0xe6)
#define REG_CTR7 (0xe7)
#define DIN_SET1 (0x18)
#define COUTER3 (0x66)
#define COUTER9 (0x6c)
#define COUTER10 (0x6d)
#define INT_TIM_TH (0xb3)
#define BPS_PARAH (0xd3)
#define BPS_PARAL (0xd4)
#define UART_REG (0x5c)
#define SC_CNTL1 (0x01)
#define MODE_CNTL (0x2a)
#define INT_MAX_I2C (0xa6)
#define INT_STEP_50 (0xa8)
#define INT_STEP_60 (0xa9)
#define DM_ROWL_AF_PIX (0x05)
#define GLB_MAX3 (0xb9)
#define SPEED_PLL (0xe1)
#endif /* INC_BF2092_REG_H_ */

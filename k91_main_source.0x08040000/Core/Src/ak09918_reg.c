/******************************************************************************
 *
 * COPYRIGHT 2017 ASAHI KASEI MICRODEVICES CORPORATION ("AKM")
 * All Rights Reserved.
 *
 * This software is licensed to you under the Apache License, Version 2.0
 * (http://www.apache.org/licenses/LICENSE-2.0) except for using, copying,
 * modifying, merging, publishing and/or distributing in combination with
 * AKM's Proprietary Software defined below. 
 *
 * "Proprietary Software" means the software and its related documentations
 * which AKM will provide only to those who have entered into the commercial
 * license agreement with AKM separately. If you wish to use, copy, modify,
 * merge, publish and/or distribute this software in combination with AKM's
 * Proprietary Software, you need to request AKM to enter into such agreement
 * and grant commercial license to you.
 *
 ******************************************************************************/
#include "ak09918_reg.h"

/******************************************************************************/
/***** AKH public APIs ********************************************************/

/**
 * @brief  Read generic device register
 *
 * @param  ctx   read / write interface definitions(ptr)
 * @param  reg   register to read
 * @param  data  pointer to buffer that store the data read(ptr)
 * @param  len   number of consecutive register to read
 * @retval          interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ak09918_read_reg(stmdev_ctx_t * ctx, uint8_t reg, uint8_t * data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
 * @brief  Write generic device register
 *
 * @param  ctx   read / write interface definitions(ptr)
 * @param  reg   register to write
 * @param  data  pointer to data to write in register reg(ptr)
 * @param  len   number of consecutive register to write
 * @retval          interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ak09918_write_reg(stmdev_ctx_t * ctx, uint8_t reg, uint8_t * data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}

/******************************************************************************/
/***** AKS public APIs ********************************************************/

int32_t ak099xx_set_mode(stmdev_ctx_t * ctx, const uint8_t mode)
{
  uint8_t i2cData;
  int32_t fret;

  i2cData = mode;
  printf("ak099xx_set_mode: 0x%x\r\n", mode);
  fret = ak09918_write_reg(ctx, AK099XX_REG_CNTL2, &i2cData, 1);
  if (fret != AKM_SUCCESS) {
    return fret;
  }

  return AKM_SUCCESS;
}

int32_t ak099xx_get_WhoAmI(stmdev_ctx_t * ctx, uint16_t * WhoAmI)
{
  uint8_t i2cData[4];
  int32_t fret;

  /* Read WIA */
  fret = ak09918_read_reg(ctx, AK099XX_REG_WIA1, i2cData, 4);

  if (fret != AKM_SUCCESS) {
    return fret;
  }

  /* Store device id (actually, it is company id.) */
  *WhoAmI = (((uint16_t) i2cData[1] << 8) | i2cData[0]);
  return AKM_SUCCESS;
}

int32_t ak099xx_soft_reset(stmdev_ctx_t * ctx)
{
  uint8_t i2cData;
  int32_t fret;

  /* Soft Reset */
  i2cData = AK099XX_SOFT_RESET;
  fret = ak09918_write_reg(ctx, AK099XX_REG_CNTL3, &i2cData, 1);

  if (fret != AKM_SUCCESS) {
    return fret;
  }

  return AKM_SUCCESS;
}

int32_t ak099xx_check_rdy(stmdev_ctx_t * ctx)
{
  uint8_t i2cData;
  int32_t fret;

  /* Check DRDY bit of ST1 register */
  fret = ak09918_read_reg(ctx, AK099XX_REG_ST1, &i2cData, 1);
  //printf("ak099xx_check_rdy reg: 0x%x\r\n", i2cData);
  if (fret != AKM_SUCCESS) {
    //printf("ak099xx_check_rdy error: %d\r\n", fret);
    return 0;
  }

  /* AK09911/09912/09913 has only one data.
   * So, return is 0 or 1. */
  if (i2cData & 0x01) {
    //printf("ak099xx_check_rdy success\r\n");
    return 1;
  } else {
    //printf("ak099xx_check_rdy failed\r\n");
    return 0;
  }
}

int32_t ak099xx_get_data(stmdev_ctx_t * ctx, struct AKM_SENSOR_DATA * data,
                         uint8_t * num)
{
  uint8_t i2cData[AK099XX_BDATA_SIZE];
  int16_t tmp;
  int32_t fret;
  uint8_t i;

  /* check arg */
  if (*num < 1) {
    return AKM_ERR_INVALID_ARG;
  }

  /* Read data */
  fret = ak09918_read_reg(ctx, AK099XX_REG_ST1, i2cData, AK099XX_BDATA_SIZE);

  if (fret != AKM_SUCCESS) {
    printf("AK099XX_REG_ST1 read error!\r\n");
    return fret;
  }

  for (i = 0; i < 3; i++) {
    /* convert to int16 data */
    tmp = MAKE_S16(i2cData[i * 2 + 2], i2cData[i * 2 + 1]);
    /* multiply ASA and convert to micro tesla in Q16 */
    data->u.v[i] = tmp * SENS_0150_Q16;

  }

  //printf("mag data: { 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x } \r\n", i2cData[1], i2cData[2], i2cData[3], i2cData[4], i2cData[5], i2cData[6]);
  //data->timestamp = 

  data->status[0] = i2cData[0];
  data->status[1] = i2cData[AK099XX_BDATA_SIZE - 1];
  *num = 1;
  return AKM_SUCCESS;
}

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

#include <stdio.h>
#include "ak09918_reg.h"

#define TLIMIT_NO_RST        0x101
#define TLIMIT_NO_RST_READ   0x102

#define TLIMIT_NO_RST_WIA1   0x103
#define TLIMIT_LO_RST_WIA1   0x48
#define TLIMIT_HI_RST_WIA1   0x48
#define TLIMIT_NO_RST_WIA2   0x104
#define TLIMIT_LO_RST_WIA2   0x0c
#define TLIMIT_HI_RST_WIA2   0x0c

#define TLIMIT_NO_SNG_CNTL2  0x201
#define TLIMIT_NO_SNG_WAIT   0x202

#define TLIMIT_NO_SNG_ST1    0x203
#define TLIMIT_LO_SNG_ST1    1
#define TLIMIT_HI_SNG_ST1    1

#define TLIMIT_NO_SNG_HX     0x204
#define TLIMIT_LO_SNG_HX     -32751
#define TLIMIT_HI_SNG_HX     32751

#define TLIMIT_NO_SNG_HY     0x206
#define TLIMIT_LO_SNG_HY     -32751
#define TLIMIT_HI_SNG_HY     32751

#define TLIMIT_NO_SNG_HZ     0x208
#define TLIMIT_LO_SNG_HZ     -32751
#define TLIMIT_HI_SNG_HZ     32751

/* Only MASKED bit of ST2 register is evaluated */
#define TLIMIT_NO_SNG_ST2    0x20A
#define TLIMIT_LO_SNG_ST2    0
#define TLIMIT_HI_SNG_ST2    0
#define TLIMIT_ST2_MASK      0x08

#define TLIMIT_NO_SLF_CNTL2  0x20B
#define TLIMIT_NO_SLF_WAIT   0x20C

#define TLIMIT_NO_SLF_ST1    0x20D
#define TLIMIT_LO_SLF_ST1    1
#define TLIMIT_HI_SLF_ST1    1

#define TLIMIT_NO_SLF_RVHX   0x20E
#define TLIMIT_LO_SLF_RVHX   -200
#define TLIMIT_HI_SLF_RVHX   200

#define TLIMIT_NO_SLF_RVHY   0x210
#define TLIMIT_LO_SLF_RVHY   -200
#define TLIMIT_HI_SLF_RVHY   200

#define TLIMIT_NO_SLF_RVHZ   0x212
#define TLIMIT_LO_SLF_RVHZ   -1000
#define TLIMIT_HI_SLF_RVHZ   -200

#define TLIMIT_NO_SLF_ST2    0x214
#define TLIMIT_LO_SLF_ST2    0
#define TLIMIT_HI_SLF_ST2    0

int16_t aks_fst_test_data(uint16_t testno, int16_t testdata, int16_t lolimit,
                          int16_t hilimit)
{
  if ((lolimit <= testdata) && (testdata <= hilimit)) {
    return AKM_SUCCESS;
  } else {
    printf("aks_fst_test_data failed %u %d! \r\n", testno, testdata);
    return AKM_ERROR;
  }
}

#define AKM_CHECK_IN_THRESHOLD(no, data, lo, hi) \
    if (aks_fst_test_data((no), (data), (lo), (hi)) != AKM_SUCCESS) \
    { goto SELFTEST_FAIL; }

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

/*
 * \result upper_16bit test number
 * \result lower_16bit test result data.
 */
int16_t ak09918_self_test(stmdev_ctx_t * ctx)
{
  int16_t fret;
  uint8_t i2cData[AK099XX_BDATA_SIZE];
  int16_t xval_i16, yval_i16, zval_i16;

  /* initialize arg */
        /**********************************************************************
	 * Step 1
	 **********************************************************************/

  /* Soft Reset */
  fret = ak099xx_soft_reset(ctx);

  if (AKM_SUCCESS != fret) {
    printf("ak099xx_soft_reset failed result = %d\r\n", fret);  //(TLIMIT_NO_RST)
    goto SELFTEST_FAIL;
  }

  /* Wait over 1000 us */
  platform_delay(1);

  /* Read values. */
  fret = ak09918_read_reg(ctx, AK099XX_REG_WIA1, i2cData, 2);

  if (AKM_SUCCESS != fret) {
    printf("ak09918_read_reg AK099XX_REG_WIA1 failed result = %d\r\n", fret);   //TLIMIT_NO_RST_READ
    goto SELFTEST_FAIL;
  }

  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_RST_WIA1, i2cData[0], TLIMIT_LO_RST_WIA1,
                         TLIMIT_HI_RST_WIA1);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_RST_WIA2, i2cData[1], TLIMIT_LO_RST_WIA2,
                         TLIMIT_HI_RST_WIA2);

        /**********************************************************************
	 * Step 2
	 **********************************************************************/

  /* Set to SNG measurement pattern. */
  fret = ak099xx_set_mode(ctx, AK099XX_MODE_SNG_MEASURE);

  if (AKM_SUCCESS != fret) {
    printf("ak09918_set_mode AK099XX_MODE_SNG_MEASURE failed result = %d\r\n", fret);   //TLIMIT_NO_SNG_CNTL2
    goto SELFTEST_FAIL;
  }

  /* Wait for single measurement. */
  platform_delay(10);

  /*
   * Get measurement data from AK09918
   * ST1 + (HXL/H) + (HYL/H) + (HZL/H) + TMPS + ST2 = 9bytes */
  fret = ak09918_read_reg(ctx, AK099XX_REG_ST1, i2cData, AK099XX_BDATA_SIZE);

  if (AKM_SUCCESS != fret) {
    printf("ak09918_read_reg AK099XX_REG_ST1 failed result = %d\r\n", fret);    //TLIMIT_NO_SNG_WAIT
    goto SELFTEST_FAIL;
  }

  /* Convert to 16-bit integer value. */
  xval_i16 = (int16_t) (((uint16_t) i2cData[1]) | ((uint16_t) i2cData[2] << 8));
  yval_i16 = (int16_t) (((uint16_t) i2cData[3]) | ((uint16_t) i2cData[4] << 8));
  zval_i16 = (int16_t) (((uint16_t) i2cData[5]) | ((uint16_t) i2cData[6] << 8));

  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SNG_ST1, i2cData[0], TLIMIT_LO_SNG_ST1,
                         TLIMIT_HI_SNG_ST1);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SNG_HX, xval_i16, TLIMIT_LO_SNG_HX,
                         TLIMIT_HI_SNG_HX);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SNG_HY, yval_i16, TLIMIT_LO_SNG_HY,
                         TLIMIT_HI_SNG_HY);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SNG_HZ, zval_i16, TLIMIT_LO_SNG_HZ,
                         TLIMIT_HI_SNG_HZ);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SNG_ST2, (i2cData[8] & TLIMIT_ST2_MASK),
                         TLIMIT_LO_SNG_ST2, TLIMIT_HI_SNG_ST2);

  /* Set to self-test mode. */
  fret = ak099xx_set_mode(ctx, AK099XX_MODE_SELF_TEST);

  if (AKM_SUCCESS != fret) {
    printf("ak09918_set_mode AK099XX_MODE_SELF_TEST failed result = %d\r\n", fret);     //TLIMIT_NO_SNG_CNTL2
    goto SELFTEST_FAIL;
  }

  /* Wait for self-test measurement. */
  /* Maximum time for measurement is 8.2 ms */
  /* Refer to datasheet p.6 */
  platform_delay(9);

  /*
   * Get measurement data from AK09918
   * ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TMPS + ST2 = 9bytes */
  fret = ak09918_read_reg(ctx, AK099XX_REG_ST1, i2cData, AK099XX_BDATA_SIZE);

  if (AKM_SUCCESS != fret) {
    printf("ak09918_read_reg AK099XX_REG_ST1 failed result = %d\r\n", fret);    //TLIMIT_NO_SLF_WAIT
    goto SELFTEST_FAIL;
  }

  /* Convert to 16-bit integer value. */
  xval_i16 = (int16_t) (((uint16_t) i2cData[1]) | ((uint16_t) i2cData[2] << 8));
  yval_i16 = (int16_t) (((uint16_t) i2cData[3]) | ((uint16_t) i2cData[4] << 8));
  zval_i16 = (int16_t) (((uint16_t) i2cData[5]) | ((uint16_t) i2cData[6] << 8));

  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SLF_ST1, i2cData[0], TLIMIT_LO_SLF_ST1,
                         TLIMIT_HI_SLF_ST1);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SLF_RVHX, xval_i16, TLIMIT_LO_SLF_RVHX,
                         TLIMIT_HI_SLF_RVHX);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SLF_RVHY, yval_i16, TLIMIT_LO_SLF_RVHY,
                         TLIMIT_HI_SLF_RVHY);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SLF_RVHZ, zval_i16, TLIMIT_LO_SLF_RVHZ,
                         TLIMIT_HI_SLF_RVHZ);
  AKM_CHECK_IN_THRESHOLD(TLIMIT_NO_SLF_ST2, (i2cData[8] & TLIMIT_ST2_MASK),
                         TLIMIT_LO_SLF_ST2, TLIMIT_HI_SLF_ST2);

  printf("ak09918_self_test success\r\n");
  return AKM_SUCCESS;

SELFTEST_FAIL:
  printf("ak09918_self_test failed\r\n");
  return AKM_ERROR;
}

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
#ifndef INCLUDE_AK099XX_REGISTER_H
#define INCLUDE_AK099XX_REGISTER_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

enum AKM_DEVICES {
  /* Empty sensors. These are not phisical sensor */
  AKM_DEVICE_NONE = -1,
  AKM_DEVICE_TEST = 0,

  /* List of supported sensors.
   * List is grouped by a sensor module. */

  /* aks_mag_ak8963 */
  AKM_MAGNETOMETER_AK8963,

  /* aks_mag_ak099xx */
  AKM_MAGNETOMETER_AK09911,
  AKM_MAGNETOMETER_AK09912,
  AKM_MAGNETOMETER_AK09913,
  AKM_MAGNETOMETER_AK09915,
  AKM_MAGNETOMETER_AK09915D,
  AKM_MAGNETOMETER_AK09916C,
  AKM_MAGNETOMETER_AK09916D,
  AKM_MAGNETOMETER_AK09917D,
  AKM_MAGNETOMETER_AK09918,

  /* aks_mag_ak0994x */
  AKM_MAGNETOMETER_AK09940,
};

/*! This value represents success of operation. */
#define AKM_SUCCESS                 (0)
/*! This value represents some error happend while operation execution. */
#define AKM_ERROR                   (-1)
/*! This value represents argument of fuction is out of range or invalid. */
#define AKM_ERR_INVALID_ARG         (-2)
/*! This value represents the operation is not supported on this platform. */
#define AKM_ERR_NOT_SUPPORT         (-3)
/*! This value represents some I/O error happend while operation execution. */
#define AKM_ERR_IO                  (-4)
/*! This value represents device is busy or other process blocks the operation
 * execution. */
#define AKM_ERR_BUSY                (-5)
/*! This value represents the operation could not be done within the specified
 * duration time. */
#define AKM_ERR_TIMEOUT             (-6)
/*! This value represents that the calculation is not yet done. */
#define AKM_ERR_NOT_YET_CALCULATED  (-7)

struct AKM_SENSOR_DATA {
  union {
    struct {
      int32_t x;
      int32_t y;
      int32_t z;
    } s;
    struct {
      int32_t i1;
      int32_t i2;
      int32_t i3;
      int32_t i4;
      int32_t t;
    } r;
    int32_t v[5];
  } u;
  /*!
   * This field holds time stamp for the data.
   * The unit of timestamp depends is nano seconds when AKM_TIMESTAMP_NANOSECOND
   * is defined. Otherwise, the unit of timestamp is micro seconds.
   * Ideally, this value is calculated from system tick, but fixed interval
   * value is acceptable.\n */
  uint64_t timestamp;

  /*!
   * This field is used to store device status register value.
   * In case of AKM compass, \c ST1 and \c ST2 value is put to
   * \c status[0] and \c status[1] respectively.*/
  int16_t status[2];
};

void AKS_ConvertCoordinate(int32_t vec[3], const uint8_t axis_order[3],
                           const uint8_t axis_sign[3]);

typedef int32_t(*stmdev_write_ptr) (void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t(*stmdev_read_ptr) (void *, uint8_t, uint8_t *, uint16_t);

typedef struct {
        /** Component mandatory fields **/
  stmdev_write_ptr write_reg;
  stmdev_read_ptr read_reg;
        /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

//register defines
#define AK099XX_REG_WIA1                 0x00
#define AK099XX_REG_WIA2                 0x01
#define AK099XX_REG_INFO1                0x02
#define AK099XX_REG_INFO2                0x03
#define AK099XX_REG_ST1                  0x10
#define AK099XX_REG_HXL                  0x11
#define AK099XX_REG_HXH                  0x12
#define AK099XX_REG_HYL                  0x13
#define AK099XX_REG_HYH                  0x14
#define AK099XX_REG_HZL                  0x15
#define AK099XX_REG_HZH                  0x16
#define AK099XX_REG_TMPS                 0x17
#define AK099XX_REG_ST2                  0x18
#define AK099XX_REG_CNTL1                0x30
#define AK099XX_REG_CNTL2                0x31
#define AK099XX_REG_CNTL3                0x32

#define AK099XX_FUSE_ASAX                0x60
#define AK099XX_FUSE_ASAY                0x61
#define AK099XX_FUSE_ASAZ                0x62

#define AK099XX_BDATA_SIZE               9

#define AK099XX_MODE_SNG_MEASURE         0x01
#define AK099XX_MODE_CONT_MEASURE_MODE1  0x02
#define AK099XX_MODE_CONT_MEASURE_MODE2  0x04
#define AK099XX_MODE_CONT_MEASURE_MODE3  0x06
#define AK099XX_MODE_CONT_MEASURE_MODE4  0x08
#define AK099XX_MODE_CONT_MEASURE_MODE5  0x0A
#define AK099XX_MODE_CONT_MEASURE_MODE6  0x0C
#define AK099XX_MODE_SELF_TEST           0x10
#define AK099XX_MODE_FUSE_ACCESS         0x1F
#define AK099XX_MODE_POWER_DOWN          0x00

#define AK099XX_SOFT_RESET               0x01

#define AK09911_WIA_VAL                  0x548
#define AK09912_WIA_VAL                  0x448
#define AK09913_WIA_VAL                  0x848
#define AK09915_WIA_VAL                  0x1048
#define AK09916C_WIA_VAL                 0x948
#define AK09916D_WIA_VAL                 0xB48
#define AK09917D_WIA_VAL                 0xD48
#define AK09918_WIA_VAL                  0xC48
#define AK09915D_INFO_VAL                0x02

/*** Device Slave addresses. **************************************************/
/* Slave address is defined as 8-bit address. */
/* It means that R/W bit 0 is added at LSB. */

/* AKM 3D Mag devices. */
#define MAGNETOMETER_SLAVE_ADDR  0x18

#define AK099XX_NSF_VAL  0x40
#define AK099XX_SET_LOWNOISE(cntl2)  ((0x40) | (cntl2))

#define MAKE_S16(U8H, U8L) \
    (int16_t)(((uint16_t)(U8H) << 8) | (uint16_t)(U8L))

#define SENS_0600_Q16  ((int32_t)(39322))       /* 0.6  in Q16 format */
#define SENS_0150_Q16  ((int32_t)(9830))        /* 0.15 in Q16 format */

int32_t ak099xx_get_WhoAmI(stmdev_ctx_t * ctx, uint16_t * WhoAmI);
int32_t ak099xx_set_mode(stmdev_ctx_t * ctx, const uint8_t mode);
int32_t ak099xx_soft_reset(stmdev_ctx_t * ctx);
int32_t ak099xx_check_rdy(stmdev_ctx_t * ctx);
int32_t ak099xx_get_data(stmdev_ctx_t * ctx, struct AKM_SENSOR_DATA *data,
                         uint8_t * num);
int16_t ak09918_self_test(stmdev_ctx_t * ctx);
#endif /* INCLUDE_AK099XX_REGISTER_H */

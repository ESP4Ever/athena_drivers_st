#ifndef _SENSORS_DEVICE_H_
#define _SENSORS_DEVICE_H_

#include "sensor.h"

#include "main.h"

#include <stdbool.h>

//todo add sensor related data struct header files

typedef struct sensor_ops {
  int (*init) (void *para1, void *para2);
  int (*init_complete) (void *para);
  int (*activate) (bool activate);
  int (*publish_sensor_data) (void *para);
  int (*config) (uint8_t config_type, void *para);
  int (*publish_config_resp) (void *para);
} sensor_ops;

typedef struct sensor_device {
  /* this sensor's type. */
  int type;

  /* sensor enable status */
  bool activate;

  /* sensor enable count */
  int8_t activate_count;

  /* sensor current ODR in ns */
  uint16_t sample_rate;

  /* store sensor config data such as bias etc. */
  float config_data[3];

  /* sensor init status */
  bool init_completed;

  /* sensor interrupt mode */
  bool use_interrupt;

  /* axis number */
  uint8_t axis_num;
  /* pointer to sensor related functions */
  sensor_ops *sensor_op_ptr;
} sensor_device;

#define SPEED_ENABLE_BITMASK (0x01 << SENSOR_TYPE_SPEED_VECTOR)
#define ACC_ENABLE_BITMASK (0x01 << SENSOR_TYPE_ACCELEROMETER)
#define MAG_ENABLE_BITMASK (0x01 << SENSOR_TYPE_MAGNETIC_FIELD)
#define GYRO_ENABLE_BITMASK (0x01 << SENSOR_TYPE_GYROSCOPE)
#define LIGHT_ENABLE_BITMASK (0x01 << SENSOR_TYPE_LIGHT)
#define LED_HEAD_ENABLE_BITMASK (0x01 << SENSOR_TYPE_LED_HEAD)
#define LEAD_REAR_ENABLE_BITMASK (0x01 << SENSOR_TYPE_LEAD_REAR)
#define PROXIMITY_HEAD_ENABLE_BITMASK (0x01 << SENSOR_TYPE_PROXIMITY_HEAD)
#define PROXIMITY_BOT_ENABLE_BITMASK (0x01 << SENSOR_TYPE_PROXIMITY_BOT)
#define PROXIMITY_REAR_ENABLE_BITMASK (0x01 << SENSOR_TYPE_PROXIMITY_REAR)
#define ROTV_ENABLE_BITMASK (0x01 << SENSOR_TYPE_ROTATION_VECTOR)
#define SPEEDV_ENABLE_BITMASK (0x01 << SENSOR_TYPE_SPEED_VECTOR)

#define SENSOR_ENABLE_NULL 0x0000;
#define SENSOR_ENABLE_ALL 0xFFFF;

#define SENSOR_SUCCESS  0
#define SENSOR_FAILED  -1

#define DEFAULE_SAMPLE_RATE_MS 100

extern sensor_device *sensors[SENSOR_TYPE_MAX];
extern uint16_t sensorEnabledBitMask;
extern uint16_t sensorAvaiableBitMask;

void sensor_register(platform_prams * parms);
void init_registered_sensors();

int sensor_config(config_event_t config_event);
int sensor_timer_handler(void);
int sensor_irq_handler(interrupt_event_t intr_event);

bool sensor_avaiable_check(uint8_t sensor_type);
uint8_t sensor_get_index_length(uint8_t sensor_type);

int sensor_self_test(config_event_t config_event);
int sensor_calibrate(config_event_t config_event);
int sensor_data_handler(sensors_event_t * sensor_data);
bool sensor_mainboard_check();

uint64_t sensor_get_timestamp();

int sensor_op_timer_enable(TIM_HandleTypeDef * htim);
int sensor_op_timer_disable(TIM_HandleTypeDef * htim);

#define ARRAY_SIZE(x) (sizeof((x)) / (sizeof(((x)[0]))))

#endif // _SENSORS_DEVICE_H_

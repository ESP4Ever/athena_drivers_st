// This file is used for sensor on STM32

#ifndef _HARDWARE_SENSORS_BASE_CONSTANTS_H_
#define _HARDWARE_SENSORS_BASE_CONSTANTS_H_

enum {
  SENSOR_TYPE_ACCELEROMETER = 0,
  SENSOR_TYPE_MAGNETIC_FIELD = 1,
  SENSOR_TYPE_GYROSCOPE = 2,
  SENSOR_TYPE_LIGHT = 3,
  SENSOR_TYPE_LED_HEAD = 4,
  SENSOR_TYPE_PROXIMITY_HEAD = 5,
  SENSOR_TYPE_PROXIMITY_BOT = 6,
  SENSOR_TYPE_PROXIMITY_REAR = 7,
  SENSOR_TYPE_LIGHT_SPEED = 8,
  SENSOR_TYPE_LED_REAR = 9,
  //make sure virtual sensor index starts after all hardware sensors
  //we need to check if hw sensor present to decide if this virtual sensor should probe
  SENSOR_TYPE_ROTATION_VECTOR = 13,
  SENSOR_TYPE_SPEED_VECTOR = 14,
  SENSOR_TYPE_MAX = 15,
};

enum {
  SENSOR_STATUS_NO_CONTACT = -1 /* -1 */ ,
  SENSOR_STATUS_UNRELIABLE = 0,
  SENSOR_STATUS_ACCURACY_LOW = 1,
  SENSOR_STATUS_ACCURACY_MEDIUM = 2,
  SENSOR_STATUS_ACCURACY_HIGH = 3,
};

typedef enum {
  SENSOR_CONFIG_MESSAGE = 0,
  SENSOR_DATA_MESSAGE = 1,
  SENSOR_TIMESTAMP_MESSAGE = 2,
  SENSOR_CONFIG_RESP_MESSAGE = 3,
  SENSOR_OTA_MESSAGE = 4,
  SENSOR_VERSION_MSG = 5,
  SENSOR_TIME_SYNC_MSG = 6,
  SENSOR_TIMER_EVENT = 7,
  SENSOR_INTERRUPT_EVENT = 8,
  SENSOR_INIT_COMPLETE_EVENT = 9,
  SENSOR_ACTIVATE_COMPLETE_EVENT = 10,
  SENSOR_DEACTIVATE_COMPLETE_EVENT = 11,
} sensor_message_event_type;

typedef enum {
  SENSOR_DEACTIVATE = 0,
  SENSOR_ACTIVATE = 1,
  SENSOR_CONFIG_SELFTEST = 2,
  SENSOR_CONFIG_CALIBRATION = 3,
  SENSOR_CONFIG_BIAS = 4,
  SENSOR_CONFIG_META = 5,
  SENSOR_CONFIG_TIMEOUT = 6,
  SENSOR_CONFIG_DATA = 7,
  SENSOR_CALIBRATION_RESULT = 8,
} sensor_config_event_type;

/**
 * Values returned by the accelerometer in various locations in the universe.
 * all values are in SI units (m/s^2)
 */
#define GRAVITY_SUN                      (275.0f)
#define GRAVITY_EARTH              (9.80665f)

/** Maximum magnetic field on Earth's surface */
#define MAGNETIC_FIELD_EARTH_MAX        (60.0f)

/** Minimum magnetic field on Earth's surface */
#define MAGNETIC_FIELD_EARTH_MIN        (30.0f)

#define MAX_AUTO_DETECT_DEV_NUM 2
/**
 * sensor event data
 */
typedef struct sensors_vec_t {
  float v[3];
  unsigned int status;
} sensors_vec_t;

/**
 * uncalibrated accelerometer, gyroscope and magnetometer event data
 */
typedef struct uncalibrated_event_t {
  float uncalib[3];
  float bias[3];
} uncalibrated_event_t;

/**
 * Union of the various types of sensor data
 * that can be returned.
 */
typedef struct sensors_event_t {
  /* sensor type */
  unsigned int sensor_type;

  /* sensor data accuracy */
  unsigned int accuracy;

  /* time is in nanosecond */
  unsigned long long timestamp;

  union {
    union {
      float data[16];

      /* acceleration values are in meter per second per second (m/s^2) */
      sensors_vec_t acceleration;

      /* magnetic vector values are in micro-Tesla (uT) */
      sensors_vec_t magnetic;

      /* orientation values are in degrees */
      sensors_vec_t orientation;

      /* gyroscope values are in rad/s */
      sensors_vec_t gyro;

      /* temperature is in degrees centigrade (Celsius) */
      float temperature;

      /* distance in centimeters */
      float distance;

      /* light in SI lux units */
      float light;

      /* pressure in hectopascal (hPa) */
      float pressure;

      /* relative humidity in percent */
      float relative_humidity;

      /* uncalibrated gyroscope values are in rad/s */
      uncalibrated_event_t uncalibrated_gyro;

      /* uncalibrated magnetometer values are in micro-Teslas */
      uncalibrated_event_t uncalibrated_magnetic;

      /* uncalibrated accelerometer values are in  meter per second per second (m/s^2) */
      uncalibrated_event_t uncalibrated_accelerometer;
    } vec;

  } sensor_data_t;
} sensors_event_t;

typedef struct sensor_data_converter {
  sensors_event_t sensor_data[SENSOR_TYPE_MAX];
  uint16_t sensor_data_bitmask[SENSOR_TYPE_MAX];
} sensor_data_converter;

#endif // _HARDWARE_SENSORS_BASE_CONSTANTS_H_

/*
 * speed_vector.c
 *
 *  Created on: Jan 2, 2021
 *      Author: sangweilin@xiaomi.com
 */

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "sensor_device.h"

static sensor_device **sensor_ptr = NULL;
static platform_prams *speed_v_init_parms;
static bool speed_v_activated = false;
static uint8_t sensor_activate_count;
static float raw_proximity_data = 0;
static uint64_t last_op_timestamp = 0;
static uint64_t current_op_timestamp = 0;
static float raw_optical_data[2] = { 0 };

int speedv_init(void *para, void *para2)
{
  speed_v_init_parms = (platform_prams *) para;
  sensor_ptr = (sensor_device **) para2;

  if ((sensor_ptr[SENSOR_TYPE_PROXIMITY_BOT] != NULL)
      && (sensor_ptr[SENSOR_TYPE_LIGHT_SPEED] != NULL)) {
    if ((sensor_ptr[SENSOR_TYPE_PROXIMITY_BOT]->init_completed)
        && (sensor_ptr[SENSOR_TYPE_LIGHT_SPEED]->init_completed)) {
      printf("all depending sensor enabled\r\n");
      return SENSOR_SUCCESS;
    } else {
      printf("depending sensor missing, init will exit with fail!\r\n");
    }
  }

  return SENSOR_FAILED;
}

int speedv_init_complete(void *para)
{
  return SENSOR_SUCCESS;
}

int speedv_enable()
{
  sensor_message_event_t sensor_msg_event;
  osStatus_t res;
  if (sensor_activate_count == 0) {
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type = SENSOR_ACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_LIGHT_SPEED;
    osMessageQueuePut(speed_v_init_parms->SensorMessageQHandle,
                      &sensor_msg_event, 0, 0);
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type = SENSOR_ACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_PROXIMITY_BOT;
    osMessageQueuePut(speed_v_init_parms->SensorMessageQHandle,
                      &sensor_msg_event, 0, 0);
    speed_v_activated = true;
  }
  sensor_activate_count++;
}

int speedv_disable()
{
  sensor_message_event_t sensor_msg_event;
  osStatus_t res;
  sensor_activate_count--;
  if (sensor_activate_count == 0) {
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type =
        SENSOR_DEACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_LIGHT_SPEED;
    osMessageQueuePut(speed_v_init_parms->SensorMessageQHandle,
                      &sensor_msg_event, 0, 0);
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type =
        SENSOR_DEACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_PROXIMITY_BOT;
    osMessageQueuePut(speed_v_init_parms->SensorMessageQHandle,
                      &sensor_msg_event, 0, 0);
    speed_v_activated = false;
  }
}

int speedv_activate(bool activate)
{
  if (activate) {
    speedv_enable();
  } else {
    speedv_disable();
  }
  return SENSOR_SUCCESS;
}

int speedv_publish_sensor_data(void *para)
{
  /* Read samples in polling mode (no int) */
  sensors_event_t sensor_data = { 0 };
  uint64_t difftime = 0;
  if (speed_v_activated) {
    //send related messages to SensorDataQ
    sensor_data.sensor_type = SENSOR_TYPE_SPEED_VECTOR;
    sensor_data.accuracy = 3;
    sensor_data.timestamp = sensor_get_timestamp();
    if (last_op_timestamp == 0)
      last_op_timestamp = current_op_timestamp;
    else {
      difftime = current_op_timestamp - last_op_timestamp;
      if (difftime > 0) {
        sensor_data.sensor_data_t.vec.data[0] =
        raw_optical_data[0] * raw_proximity_data / 1000;
        sensor_data.sensor_data_t.vec.data[1] =
        raw_optical_data[1] * raw_proximity_data / 1000;
        osMessageQueuePut(speed_v_init_parms->SensorDataQHandle, &sensor_data, 0, 0);
        last_op_timestamp = current_op_timestamp;
      }
    }
  }
  return SENSOR_SUCCESS;
}

int speedv_config(uint8_t config_type, void *para)
{
  sensors_event_t *sensor_data;
  sensor_data = (sensors_event_t *) para;
  if (config_type == SENSOR_CONFIG_DATA) {
    if (sensor_data->sensor_type == SENSOR_TYPE_LIGHT_SPEED) {
      raw_optical_data[0] = sensor_data->sensor_data_t.vec.data[0];
      raw_optical_data[1] = sensor_data->sensor_data_t.vec.data[1];
      current_op_timestamp = sensor_data->timestamp;
    } else if (sensor_data->sensor_type == SENSOR_TYPE_PROXIMITY_BOT) {
      raw_proximity_data = sensor_data->sensor_data_t.vec.data[0];
    }
  }
  return SENSOR_SUCCESS;
}

int speedv_publish_config_resp(void *para)
{
  return SENSOR_SUCCESS;
}

#include "sensor_device.h"

#include "sensor_driver.h"

sensor_device *sensors[SENSOR_TYPE_MAX] = { 0 };

uint16_t sensorEnabledBitMask = 0x0000;
uint16_t sensorAvaiableBitMask = 0x0000;

//pointer to related plafrom resource handles
static platform_prams *platform_init_parms = NULL;
static uint16_t platform_timer_enable_count = 0;
TIM_HandleTypeDef *op_timer;

static uint8_t timeout_sensor_type[] = {
  SENSOR_TYPE_PROXIMITY_HEAD,
  SENSOR_TYPE_PROXIMITY_REAR,
  SENSOR_TYPE_LED_HEAD,
};

/* acc_gyro sensor related code starts from here */
//fill the real hardware sensor related callback functions here
sensor_ops gyro_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {.init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
  {
   .init = lsm6dso_gyro_init,
   .init_complete = lsm6dso_init_complete,
   .activate = lsm6dso_gyro_activate,
   .publish_sensor_data = lsm6dso_publish_sensor_data,
   .config = lsm6dso_gyro_config,
   .publish_config_resp = lsm6dso_publish_config_resp,
   },
};

sensor_device gyro_sensor = {
  .type = SENSOR_TYPE_GYROSCOPE,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 4,
  .sensor_op_ptr = &gyro_ops[0],
};

sensor_ops acc_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = lsm6dso_acc_init,
   .init_complete = lsm6dso_init_complete,
   .activate = lsm6dso_acc_activate,
   .publish_sensor_data = lsm6dso_publish_sensor_data,
   .config = lsm6dso_acc_config,
   .publish_config_resp = lsm6dso_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device acc_sensor = {
  .type = SENSOR_TYPE_ACCELEROMETER,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 4,
  .sensor_op_ptr = &acc_ops[0],
};

//mag sensor starts from here
sensor_ops mag_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = ak09918_mag_init,
   .init_complete = ak09918_mag_init_complete,
   .activate = ak09918_mag_activate,
   .publish_sensor_data = ak09918_mag_publish_sensor_data,
   .config = ak09918_mag_config,
   .publish_config_resp = ak09918_mag_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device mag_sensor = {
  .type = SENSOR_TYPE_MAGNETIC_FIELD,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 3,
  .sensor_op_ptr = &mag_ops[0],
};

/* light sensor related code starts from here */
//fill the real hardware sensor related callback functions here
sensor_ops light_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = bu27030_light_init,
   .init_complete = bu27030_init_complete,
   .activate = bu27030_light_activate,
   .publish_sensor_data = bu27030_publish_sensor_data,
   .config = bu27030_config,
   .publish_config_resp = bu27030_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device light_sensor = {
  .type = SENSOR_TYPE_LIGHT,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 4,
  .sensor_op_ptr = &light_ops[0],
};

/* optical flow sensor related code starts from here */
//fill the real hardware sensor related callback functions here
sensor_ops optical_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = bf2092_optical_init,
   .init_complete = bf2092_init_complete,
   .activate = bf2092_optical_activate,
   .publish_sensor_data = bf2092_publish_sensor_data,
   .config = bf2092_config,
   .publish_config_resp = bf2092_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device optical_sensor = {
  .type = SENSOR_TYPE_LIGHT_SPEED,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 2,
  .sensor_op_ptr = &optical_ops[0],
};

/* led sensor related code starts from here */

sensor_ops headled_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = headled_init,
   .init_complete = headled_init_complete,
   .activate = headled_activate,
   .publish_sensor_data = NULL,
   .config = headled_config,
   .publish_config_resp = headled_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_ops led_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = led_init,
   .init_complete = led_init_complete,
   .activate = led_activate,
   .publish_sensor_data = NULL,
   .config = led_config,
   .publish_config_resp = led_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device head_led_sensor = {
  .type = SENSOR_TYPE_LED_HEAD,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 1,
  .sensor_op_ptr = &headled_ops[0],
};

sensor_device rear_led_sensor = {
  .type = SENSOR_TYPE_LED_REAR,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 1,
  .sensor_op_ptr = &led_ops[0],
};

/* proximity HEAD */
sensor_ops prox_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = prox_init,
   .init_complete = prox_init_complete,
   .activate = prox_activate,
   .publish_sensor_data = prox_publish_sensor_data,
   .config = prox_config,
   .publish_config_resp = prox_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

/* proximity REAR */
sensor_ops rear_prox_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device head_prox_sensor = {
  .type = SENSOR_TYPE_PROXIMITY_HEAD,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = true,
  .axis_num = 2,
  .sensor_op_ptr = &prox_ops[0],
};

sensor_device rear_prox_sensor = {
  .type = SENSOR_TYPE_PROXIMITY_REAR,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = true,
  .axis_num = 2,
  .sensor_op_ptr = &rear_prox_ops[0],
};

/* proximity BOT */
sensor_ops tof_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = tof_init,
   .init_complete = tof_init_complete,
   .activate = tof_activate,
   .publish_sensor_data = tof_publish_sensor_data,
   .config = tof_config,
   .publish_config_resp = tof_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device tof_sensor = {
  .type = SENSOR_TYPE_PROXIMITY_BOT,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 2,
  .sensor_op_ptr = &tof_ops[0],
};

/* rotation vector*/
sensor_ops rotv_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = rotv_init,
   .init_complete = rotv_init_complete,
   .activate = rotv_activate,
   .publish_sensor_data = rotv_publish_sensor_data,
   .config = rotv_config,
   .publish_config_resp = rotv_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device rotv_sensor = {
  .type = SENSOR_TYPE_ROTATION_VECTOR,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 4,
  .sensor_op_ptr = &rotv_ops[0],
};

/* rotation vector*/
sensor_ops speedv_ops[MAX_AUTO_DETECT_DEV_NUM] = {
  {
   .init = speedv_init,
   .init_complete = speedv_init_complete,
   .activate = speedv_activate,
   .publish_sensor_data = speedv_publish_sensor_data,
   .config = speedv_config,
   .publish_config_resp = speedv_publish_config_resp,
   },
  {
   .init = NULL,
   .init_complete = NULL,
   .activate = NULL,
   .publish_sensor_data = NULL,
   .config = NULL,
   .publish_config_resp = NULL,
   },
};

sensor_device speedv_sensor = {
  .type = SENSOR_TYPE_SPEED_VECTOR,
  .activate = false,
  .activate_count = 0,
  .sample_rate = 0xFFFF,
  .init_completed = false,
  .use_interrupt = false,
  .axis_num = 2,
  .sensor_op_ptr = &speedv_ops[0],
};

/* ======================================================================================================= */
bool sensor_avaiable_check(uint8_t sensor_type)
{
  if (sensor_type == SENSOR_TYPE_MAX) {
    return true;
  } else if (sensor_type < SENSOR_TYPE_MAX) {
    if (sensors[sensor_type] != NULL)
      return sensors[sensor_type]->init_completed;
    else
      return false;
  } else {
    return false;
  }
}

bool sensor_mainboard_check()
{
  if ((sensors[SENSOR_TYPE_ACCELEROMETER] != NULL)
      && (sensors[SENSOR_TYPE_MAGNETIC_FIELD] != NULL)
      && (sensors[SENSOR_TYPE_GYROSCOPE] != NULL)) {
    if ((sensors[SENSOR_TYPE_ACCELEROMETER]->init_completed)
        && (sensors[SENSOR_TYPE_MAGNETIC_FIELD]->init_completed)
        && (sensors[SENSOR_TYPE_GYROSCOPE]->init_completed)) {
      printf("this is the sensor mainboard enabled\r\n");
      return true;
    } else {
      printf("imu or mag missing, this is a slave board!\r\n");
    }
  }

  return false;
}

uint8_t sensor_get_index_length(uint8_t sensor_type)
{
  if (sensor_type < SENSOR_TYPE_MAX && sensors[sensor_type] != NULL) {
    return sensors[sensor_type]->axis_num;
  } else {
    return 0xFF;
  }
}

/* common register code starts from here */
void sensor_register(platform_prams * parms)
{
  //fill the sensors[SENSOR_TYPE_MAX] data struct for this chip
  platform_init_parms = parms;

  op_timer = &platform_init_parms->OperateTimerHandle;

  sensors[SENSOR_TYPE_ACCELEROMETER] = &acc_sensor;
  sensors[SENSOR_TYPE_GYROSCOPE] = &gyro_sensor;
  sensors[SENSOR_TYPE_MAGNETIC_FIELD] = &mag_sensor;
  sensors[SENSOR_TYPE_LIGHT] = &light_sensor;
  sensors[SENSOR_TYPE_LED_HEAD] = &head_led_sensor;
  sensors[SENSOR_TYPE_LED_REAR] = &rear_led_sensor;
  sensors[SENSOR_TYPE_PROXIMITY_HEAD] = &head_prox_sensor;
  sensors[SENSOR_TYPE_PROXIMITY_REAR] = &rear_prox_sensor;
  sensors[SENSOR_TYPE_LIGHT_SPEED] = &optical_sensor;
  sensors[SENSOR_TYPE_ROTATION_VECTOR] = &rotv_sensor;
  sensors[SENSOR_TYPE_SPEED_VECTOR] = &speedv_sensor;

  sensors[SENSOR_TYPE_PROXIMITY_BOT] = &tof_sensor;
  //register_acc_gyro(/* pointer to bus, pointer to sensor manager Q*/);
  //register_mag(/* pointer to bus, pointer to sensor manager Q*/);
  //register_als(/* pointer to bus, pointer to sensor manager Q*/);
}

void init_registered_sensors()
{
  int res = SENSOR_SUCCESS;
  sensor_message_event_t sensor_msg_event;

  if (platform_init_parms != NULL) {
    for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
      if (sensors[i] != NULL) {
        if (platform_init_parms->boardID == REAR_BOARD && (i == 4 || i == 5)) { // REAR BOARD skip led and proximity on HEAD
          continue;
        }
        if (platform_init_parms->boardID == HEAD_BOARD && (i == 9 || i == 7)) { // HEAD BOARD skip led and proximity on REAR
          continue;
        }
        if (sensors[i]->init_completed) {
          continue;
        }

        for (int j = 0; j < MAX_AUTO_DETECT_DEV_NUM; j++) {
          if (sensors[i]->sensor_op_ptr[j].init != NULL) {
            res =
                sensors[i]->sensor_op_ptr[j].init(platform_init_parms, sensors);
            if (res == SENSOR_SUCCESS) {
              res =
                  sensors[i]->sensor_op_ptr[j].
                  init_complete(platform_init_parms);
              sensorAvaiableBitMask = sensorAvaiableBitMask | (1 << i);
              //always set the avaiable sensor_op_ptr to index 0 of array so that it is sample for smgr to use
              sensors[i]->sensor_op_ptr[0] = sensors[i]->sensor_op_ptr[j];
              sensors[i]->init_completed = true;
              break;
            }
          }
        }
      }
    }
    sensor_msg_event.message_event_type = SENSOR_INIT_COMPLETE_EVENT;
    //notifiy smgr that init is completed
    osMessageQueuePut(platform_init_parms->SensorMessageQHandle,
                      &sensor_msg_event, 0, 0);
  } else {
    //sensor_register is not called successfully
  }
  return;
}

int sensor_op_timer_enable(TIM_HandleTypeDef * htim)
{
  int res = SENSOR_SUCCESS;

  if (htim == op_timer) {
    if (platform_timer_enable_count == 0) {
      HAL_TIM_Base_Start_IT(htim);
    }
    platform_timer_enable_count++;
  } else {
    res = SENSOR_FAILED;
  }

  return res;
}

int sensor_op_timer_disable(TIM_HandleTypeDef * htim)
{
  int res = SENSOR_SUCCESS;

  if (htim == op_timer) {
    if (platform_timer_enable_count == 0) {
      return SENSOR_FAILED;
    }
    platform_timer_enable_count--;
    if (platform_timer_enable_count == 0) {
      HAL_TIM_Base_Stop_IT(htim);
    }
  } else {
    res = SENSOR_FAILED;
  }
  return res;
}

int sensor_config(config_event_t config_event)
{
  int res = SENSOR_SUCCESS;
  sensor_message_event_t sensor_msg_event;

  printf("config: %d, %d \r\n", config_event.config_type,
         config_event.sensor_type);

  switch (config_event.config_type) {
  case SENSOR_LED_MODE_CONFIG:
    if (config_event.sensor_type == SENSOR_TYPE_LED_HEAD
        || config_event.sensor_type == SENSOR_TYPE_LED_REAR) {
      if (platform_init_parms != NULL) {
        if (sensors[config_event.sensor_type] != NULL) {
          if (sensors[config_event.sensor_type]->init_completed) {
            res =
                sensors[config_event.sensor_type]->sensor_op_ptr[0].
                config(config_event.config_type,
                       config_event.cfg_data.config_data_u8);
          }
        }
      }
      return SENSOR_SUCCESS;
    }
    break;
  case SENSOR_ACTIVATE:
    if (config_event.sensor_type == SENSOR_TYPE_MAX) {
      //default setting to run all sensor when start
      if (platform_init_parms != NULL) {
        for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
          if (sensors[i] != NULL) {
            if (!sensors[i]->init_completed) {
              continue;
            }
            if (sensors[i]->sensor_op_ptr[0].activate != NULL) {
              if (sensors[i]->activate_count == 0) {
                res = sensors[i]->sensor_op_ptr[0].activate(true);
                if (res == SENSOR_SUCCESS) {
                  sensors[i]->activate_count++;
                  sensorEnabledBitMask = sensorEnabledBitMask | (1 << i);
                  sensors[i]->activate = true;
                  sensors[i]->sample_rate = DEFAULE_SAMPLE_RATE_MS;
                } else {
                  //continue enable flow
                  continue;
                }
              } else {
                sensors[i]->activate_count++;
                printf("this sensor %d is already enable %d!\r\n", i,
                       sensors[i]->activate_count);
              }
            } else {
              //assert here
            }
          }
        }
        printf("starting timer!!! \r\n");
        sensor_msg_event.message_event_type = SENSOR_ACTIVATE_COMPLETE_EVENT;
        //notifiy smgr that activate is completed
        osMessageQueuePut(platform_init_parms->SensorMessageQHandle,
                          &sensor_msg_event, 0, 0);
      } else {
        //register sensor is not called successfully
        printf("platform_init_parms == NULL!\r\n");
        //assert here
      }
    } else {
      if (!sensor_avaiable_check(config_event.sensor_type)) {
        return SENSOR_FAILED;
      }

      if (platform_init_parms == NULL) {
        return SENSOR_FAILED;
      }

      bool is_first_user = false;
      if (sensors[config_event.sensor_type]->activate_count == 0) {
        if (!sensors[config_event.sensor_type]->activate) {
          if (sensors[config_event.sensor_type]->sensor_op_ptr[0].activate
              != NULL) {
            res =
                sensors[config_event.sensor_type]->sensor_op_ptr[0].
                activate(true);
            if (res == SENSOR_SUCCESS) {
              if (sensorEnabledBitMask == 0x0000) {
                is_first_user = true;
              }
              sensors[config_event.sensor_type]->activate_count++;
              sensorEnabledBitMask =
                  sensorEnabledBitMask | (1 << config_event.sensor_type);
              sensors[config_event.sensor_type]->activate = true;
              sensors[config_event.sensor_type]->sample_rate =
                  DEFAULE_SAMPLE_RATE_MS;
              if (is_first_user) {
                printf("first user starting timer!!! \r\n");
                sensor_msg_event.message_event_type =
                    SENSOR_ACTIVATE_COMPLETE_EVENT;
                //notifiy smgr that activate is completed
                osMessageQueuePut(platform_init_parms->SensorMessageQHandle,
                                  &sensor_msg_event, 0, 0);
              }
            } else {
              //error print here
              printf("sensor %d enable failed \r\n", config_event.sensor_type);
              break;
            }
          } else {
            //assert here
          }
        } else {
          //assert here since sensors should not activated when activate count is 0
        }
      } else {
        sensors[config_event.sensor_type]->activate_count++;
        printf("this sensor %d is already enable %d!\r\n",
               config_event.sensor_type,
               sensors[config_event.sensor_type]->activate_count);
      }
    }
    break;
  case SENSOR_DEACTIVATE:
    if (config_event.sensor_type == SENSOR_TYPE_MAX) {
      //default setting to run all sensor when receive special sensor type
      if (platform_init_parms != NULL) {
        for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
          if (sensors[i] != NULL) {
            if (!sensors[i]->init_completed) {
              continue;
            }
            if (sensors[i]->sensor_op_ptr[0].activate != NULL) {
              sensors[i]->activate_count--;
              if (sensors[i]->activate_count < 0) {
                printf("sensor %d is already disabled!\r\n", i);
                sensors[i]->activate_count = 0;
                continue;
              }
              if (sensors[i]->activate_count == 0) {
                res = sensors[i]->sensor_op_ptr[0].activate(false);
                if (res == SENSOR_SUCCESS) {
                  sensorEnabledBitMask = sensorEnabledBitMask & (~(1 << i));
                  sensors[i]->activate = false;
                  sensors[i]->sample_rate = 0xFFFF;
                } else {
                  printf("disable sensor %d failed\r\n", i);
                  continue;
                }
              }
            } else {
              //assert here
            }
          }
        }
        sensor_msg_event.message_event_type = SENSOR_DEACTIVATE_COMPLETE_EVENT;
        //notifiy smgr that deactivate is completed
        osMessageQueuePut(platform_init_parms->SensorMessageQHandle,
                          &sensor_msg_event, 0, 0);
      } else {
        //register sensor is not called successfully
      }
    } else {
      if (!sensor_avaiable_check(config_event.sensor_type)) {
        return SENSOR_FAILED;
      }

      if (platform_init_parms == NULL) {
        return SENSOR_FAILED;
      }

      sensors[config_event.sensor_type]->activate_count--;
      if (sensors[config_event.sensor_type]->activate_count < 0) {
        printf("sensor %d is already disabled!\r\n", config_event.sensor_type);
        sensors[config_event.sensor_type]->activate_count = 0;
        return SENSOR_SUCCESS;
      }

      if (sensors[config_event.sensor_type]->activate_count == 0) {
        if (sensors[config_event.sensor_type]->activate) {
          if (sensors[config_event.sensor_type]->sensor_op_ptr[0].activate
              != NULL) {
            res =
                sensors[config_event.sensor_type]->sensor_op_ptr[0].
                activate(false);
            if (res == SENSOR_SUCCESS) {
              sensorEnabledBitMask =
                  sensorEnabledBitMask & (~(1 << config_event.sensor_type));
              sensors[config_event.sensor_type]->activate = false;
              sensors[config_event.sensor_type]->sample_rate = 0xFFFF;
            } else {
              //print error log here
              printf("sensor %d disable failed\r\n", config_event.sensor_type);
              break;
            }
          } else {
            //assert here
          }
          if (sensorEnabledBitMask == 0x0000) {
            if (platform_init_parms != NULL) {
              sensor_msg_event.message_event_type =
                  SENSOR_DEACTIVATE_COMPLETE_EVENT;
              //notifiy smgr that deactivate is completed
              osMessageQueuePut(platform_init_parms->SensorMessageQHandle,
                                &sensor_msg_event, 0, 0);
            } else {
              //assert
            }
          }
        } else {
          //assert here since sensors should not deactivated when activate count is 0
        }
      }
    }
    break;
  case SENSOR_CONFIG_SELFTEST:
  case SENSOR_CONFIG_CALIBRATION:
    if (config_event.sensor_type == SENSOR_TYPE_MAX) {
      //do not support All type calibration
      printf("selftest is not supporting SENSOR_TYPE_MAX \r\n");
    } else {
      if (!sensor_avaiable_check(config_event.sensor_type)) {
        return SENSOR_FAILED;
      }

      if (platform_init_parms == NULL) {
        return SENSOR_FAILED;
      }
      printf("sensor config cali/selftest\r\n");
      if (sensors[config_event.sensor_type]->activate) {
        printf
            ("selftest/cali is not supporting when sensor is enabled! disable this sensor first \r\n");
      } else {
        if (sensors[config_event.sensor_type]->sensor_op_ptr[0].config != NULL) {
          res =
              sensors[config_event.sensor_type]->sensor_op_ptr[0].
              config(config_event.config_type, platform_init_parms);
          if (platform_init_parms->use_uart_mode == 1) {
            uint8_t uart_sensor_data[UART_TX_CODE_LENGTH] = { 0 };
            memcpy(&uart_sensor_data[0], res, sizeof(int));
            osMessageQueuePut(platform_init_parms->UartReportQHandle,
                              uart_sensor_data, 0, 0);
          } else {
            sensor_msg_event.message_event_type = SENSOR_CONFIG_RESP_MESSAGE;
            sensor_msg_event.message_event_t.resp_event.sensor_type =
                config_event.sensor_type;
            sensor_msg_event.message_event_t.resp_event.config_type =
                config_event.config_type;
            sensor_msg_event.message_event_t.resp_event.cfg_data.resp_data[0] =
                res;
            osMessageQueuePut(platform_init_parms->SensorMessageQHandle,
                              &sensor_msg_event, 0, 0);
          }
        } else {
          //assert here if config is null
          printf("config ptr is NULL!\r\n");
          break;
        }
      }
    }
    break;
  case SENSOR_CONFIG_TIMEOUT:
    if (platform_init_parms != NULL) {
      for (int i = 0; i < sizeof(timeout_sensor_type) / sizeof(uint8_t);
           i = i + 1) {
        if (sensors[timeout_sensor_type[i]] != NULL) {
          if (sensors[timeout_sensor_type[i]]->activate) {
            //printf("@@@SENSOR_CONFIG_TIMEOUT!\r\n");
            res =
                sensors[timeout_sensor_type[i]]->sensor_op_ptr[0].
                config(config_event.config_type, platform_init_parms);
            if (res != SENSOR_SUCCESS) {
              printf("config failed for SENSOR_CONFIG_TIMEOUT\r\n");
            }
          }
        }
      }
    } else {
      //assert here should be occured!
    }
    break;
  case SENSOR_CONFIG_BIAS:
  case SENSOR_CALIBRATION_RESULT:
    if (platform_init_parms != NULL) {
      if (!sensor_avaiable_check(config_event.sensor_type)) {
        return SENSOR_FAILED;
      }
      if (sensors[config_event.sensor_type] != NULL) {
        //printf("@@@SENSOR_CONFIG_BIAS!\r\n");
        res =
            sensors[config_event.sensor_type]->sensor_op_ptr[0].
            config(config_event.config_type, platform_init_parms);
        if (res != SENSOR_SUCCESS) {
          printf("config failed for SENSOR_CONFIG_BIAS\r\n");
        }
      }
    } else {
      //assert here should be occured!
    }
    break;
  default:
    break;
  }

  return res;
}

int sensor_timer_handler(void)
{
  int res = SENSOR_SUCCESS;
  if (platform_init_parms != NULL) {
    for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
      if (sensors[i] != NULL) {
        if (!sensors[i]->init_completed) {
          continue;
        }
        if (!sensors[i]->activate) {
          continue;
        }
        if (sensors[i]->use_interrupt) {
          continue;
        }

        if (sensors[i]->sensor_op_ptr[0].publish_sensor_data != NULL) {
          res =
              sensors[i]->sensor_op_ptr[0].
              publish_sensor_data(platform_init_parms);
          if (res == SENSOR_SUCCESS) {
            //do nothing
          } else {
            continue;
          }
        } else {
          //assert here
        }
      }
    }
  } else {
    //register sensor is not called successfully
  }
  return res;
}

int sensor_irq_handler(interrupt_event_t intr_event)
{
  int res = SENSOR_SUCCESS;
  if (platform_init_parms != NULL) {
    for (int i = SENSOR_TYPE_PROXIMITY_HEAD; i <= SENSOR_TYPE_PROXIMITY_REAR;
         i = i + 2) {
      if (sensors[i] != NULL) {
        if (!sensors[i]->init_completed) {
          continue;
        }
        res =
            sensors[i]->sensor_op_ptr[0].publish_sensor_data(&intr_event.
                                                             interrupt_num);
        if (res != SENSOR_SUCCESS) {
          printf("publish_sensor_data failed\r\n");
        }
      }
    }
  } else {
    //assert here should be occured!
  }
  return 0;
}

int sensor_data_handler(sensors_event_t * sensor_data)
{
  if (sensor_avaiable_check(SENSOR_TYPE_ROTATION_VECTOR)
      && ((sensorEnabledBitMask >> SENSOR_TYPE_ROTATION_VECTOR) & 1)
      && (sensor_data->sensor_type == SENSOR_TYPE_ACCELEROMETER
          || sensor_data->sensor_type == SENSOR_TYPE_GYROSCOPE
          // || sensor_data->sensor_type == SENSOR_TYPE_MAGNETIC_FIELD
      ))
    sensors[SENSOR_TYPE_ROTATION_VECTOR]->sensor_op_ptr[0].
        config(SENSOR_CONFIG_DATA, sensor_data);
  else if (sensor_avaiable_check(SENSOR_TYPE_SPEED_VECTOR)
           && ((sensorEnabledBitMask >> SENSOR_TYPE_SPEED_VECTOR) & 1)
           && (sensor_data->sensor_type == SENSOR_TYPE_LIGHT_SPEED
               || sensor_data->sensor_type == SENSOR_TYPE_PROXIMITY_BOT))
    sensors[SENSOR_TYPE_SPEED_VECTOR]->sensor_op_ptr[0].
        config(SENSOR_CONFIG_DATA, sensor_data);
  return 0;
}

#define MAX_U32_TIMESTAMP 0xFFFFFFFF

uint64_t sensor_get_timestamp()
{
  static uint32_t lastu32timestamp = 0;
  static uint32_t currentu32timestamp = 0;
  static uint64_t sensor_u64timestamp = 0;
  static uint64_t last_sensor_u64timestamp = 0;
  static uint8_t u32overflowcount = 0;
  static uint8_t u64overflowcount = 0;

  currentu32timestamp = osKernelGetTickCount();

  if (currentu32timestamp < lastu32timestamp) {
    u32overflowcount++;
    sensor_u64timestamp = MAX_U32_TIMESTAMP * u32overflowcount;
    last_sensor_u64timestamp = MAX_U32_TIMESTAMP * u32overflowcount;
  }

  if (u32overflowcount) {
    sensor_u64timestamp = last_sensor_u64timestamp + currentu32timestamp;
  } else {
    sensor_u64timestamp = currentu32timestamp;
  }

  lastu32timestamp = currentu32timestamp;
  last_sensor_u64timestamp = sensor_u64timestamp;
  return sensor_u64timestamp;
}

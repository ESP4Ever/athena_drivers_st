// This file is used for can communication on STM32 can bus

#ifndef _CAN_PROTO_H_
#define _CAN_PROTO_H_

#include "main.h"

//TxHeader.StdId: (EVENT_TYPE 1byte) << 8 | (SENSOR_TYPE 1byte) << 4 | (COMMAND_CODE 1byte) 
//TxHeader.ExtId = 0x01;

#define SENSOR_TYPE_BIT_SHIFT                 4
#define SENSOR_TYPE_BIT_MASK                  0x0F << SENSOR_TYPE_BIT_SHIFT
#define SENSOR_EVENT_MESSAGE_BIT_SHIFT        8
#define SENSOR_EVENT_MESSAGE_BIT_MASK         0x0F << SENSOR_EVENT_MESSAGE_BIT_SHIFT
#define OTA_EXTEND_MESSAGE_BIT_SHIFT          12
#define OTA_EXTEND_MESSAGE_BIT_MASK           0xFFFF << OTA_EXTEND_MESSAGE_BIT_SHIFT
#define SENSOR_COMMAND_BIT_MASK               0x0F

#define CHIP_ID_BIT_SHIFT                 4
#define CHIP_ID_BIT_MASK                  0x0F << CHIP_ID_BIT_SHIFT
#define CANCOM_MODE_BIT_MASK              0x0F

//examples
#define CAN_ID_SENSOR_ENABLE_ALL          0x0F1 //SENSOR_CONFIG_MESSAGE      | SENSOR_TYPE_MAX            | SENSOR_ACTIVATE
#define CAN_ID_SENSOR_DISABLE_ALL         0x0F0 //SENSOR_CONFIG_MESSAGE      | SENSOR_TYPE_MAX            | SENSOR_DEACTIVATE
#define CAN_ID_SENSOR_SELFTEST_ACC        0x002 //SENSOR_CONFIG_MESSAGE      | SENSOR_TYPE_ACCELEROMETER  | SENSOR_CONFIG_SELFTEST
#define CAN_ID_SENSOR_ACC_DATA            0x103 //SENSOR_DATA_MESSAGE        | SENSOR_TYPE_ACCELEROMETER  | DATA_AXIS_NUM
#define CAN_ID_SENSOR_MAG_DATA            0x113 //SENSOR_DATA_MESSAGE        | SENSOR_TYPE_MAGNETIC_FIELD | DATA_AXIS_NUM
#define CAN_ID_SENSOR_GYRO_DATA           0x123 //SENSOR_DATA_MESSAGE        | SENSOR_TYPE_GYROSCOPE      | DATA_AXIS_NUM
#define CAN_ID_SENSOR_GYRO_TINESTAMP      0x223 //SENSOR_TIMESTAMP_MESSAGE   | SENSOR_TYPE_GYROSCOPE      | DATA_AXIS_NUM

#define CAN_START_OTA                     0x430 //SENSOR_OTA_MESSAGE         | STM32 INDEX                | canComMode

typedef struct can_message_event_t {
  union {
    CAN_TxHeaderTypeDef TxHeader;
    CAN_RxHeaderTypeDef RxHeader;
  } CanHeader;
  union {
    uint8_t RxData[8];
    uint8_t TxData[8];
  } CanData;
} can_message_event_t;

enum canComMode {
  CANCOM_OTA_START = 0,         //OTA-启动
  CANCOM_OTA_INFO = 1,          //OTA-升级文件描述
  CANCOM_OTA_ING = 2,           //OTA-升级中
  CANCOM_OTA_END = 3,           //OTA-升级完成
  CANCOM_OTA_BOOTUP = 4,        //OTA 结束后开始启动
  CANCOM_OTA_SETBOOT_SECTOR = 5,        //OTA 强制设置启动分区
  CANCOM_OTA_MODE_ENTER = 6,

  CANCOM_OTA_START_ACK = 7,
  CANCOM_OTA_INFO_ACK = 8,
  CANCOM_OTA_ING_ACK = 9,
  CANCOM_OTA_END_ACK = 10,
  CANCOM_OTA_SETBOOT_SECTOR_ACK = 11,
  CANCOM_OTA_MODE_ENTER_ACK = 12,
  CANCOM_OTA_BOOTUP_ACK = 13,
  CANCOM_OTA_ERROR_ACK = 14,

  CANCOM_MODE_TOTAL = 0xF,
};

typedef enum { CAN_OTA_ERROR = 0, CAN_OTA_SUCCESS = !CAN_OTA_ERROR } ErrStatus;

#endif // _CAN_PROTO_H_

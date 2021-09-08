/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "stdio.h"
#include "calibration_data.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

  typedef struct platform_prams {
    uint8_t boardID;
    I2C_HandleTypeDef i2c_handle;
    UART_HandleTypeDef uart;
    SPI_HandleTypeDef spi_handle;
    osMessageQueueId_t SensorMessageQHandle;
    osMessageQueueId_t SensorDataQHandle;
    osMessageQueueId_t UartReportQHandle;
    osTimerId_t PollingTimerHandle;
    TIM_HandleTypeDef OperateTimerHandle;
    TIM_HandleTypeDef PWMTimerHandle;
    calibrationData *board_calidata;
    uint8_t use_uart_mode;
    GPIO_TypeDef* tof_cs_gpio_port;
    uint16_t tof_cs_pin;
  } platform_prams;

#define UART_RX_CODE_LENGTH (4)
#define UART_TX_CODE_LENGTH (29)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

  void HAL_TIM_MspPostInit(TIM_HandleTypeDef * htim);

/* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define REAR_PROG0_PIN_Pin GPIO_PIN_0
#define REAR_PROG0_PIN_GPIO_Port GPIOC
#define HEAD_PROG0_PIN_Pin GPIO_PIN_1
#define HEAD_PROG0_PIN_GPIO_Port GPIOC
#define HEAD_AND_REAR_ULTRALSONIC_RESET_N_PIN_Pin GPIO_PIN_2
#define HEAD_AND_REAR_ULTRALSONIC_RESET_N_PIN_GPIO_Port GPIOC
#define HEAD_AND_REAR_ULTRASONIC_INT_Pin GPIO_PIN_3
#define HEAD_AND_REAR_ULTRASONIC_INT_GPIO_Port GPIOC
#define HEAD_AND_REAR_ULTRASONIC_INT_EXTI_IRQn EXTI3_IRQn
#define TOF_CS_Pin GPIO_PIN_4
#define TOF_CS_GPIO_Port GPIOA
#define GUANGLIU_EN_Pin GPIO_PIN_4
#define GUANGLIU_EN_GPIO_Port GPIOC
#define TOF_EN_Pin GPIO_PIN_5
#define TOF_EN_GPIO_Port GPIOC
#define HEAD_AND_REAR_LED_DRIVER_EN_Pin GPIO_PIN_12
#define HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port GPIOB
#define TOF_GPIO1_Pin GPIO_PIN_6
#define TOF_GPIO1_GPIO_Port GPIOC
#define IMU_INT_Pin GPIO_PIN_7
#define IMU_INT_GPIO_Port GPIOC
#define TOF_GPIO2_Pin GPIO_PIN_9
#define TOF_GPIO2_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
#define HEAD_BOARD 0x3
#define BOT_BOARD 0x2
#define REAR_BOARD 0x0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif
#endif                          /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

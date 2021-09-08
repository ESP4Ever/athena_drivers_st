/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/

#include "can_proto.h"
#include "sensor_base.h"
#include "flash_interface.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#define USE_P1 1

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TASK_STACK_SIZE_IN_WORD 256
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Definitions for Can1Broadcast */
osThreadId_t OtaTaskHandle;
const osThreadAttr_t OtaTask_attributes = {
  .name = "OtaTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 2
};

/* Definitions for Can1Broadcast */
osThreadId_t Can1BroadcastHandle;
const osThreadAttr_t Can1Broadcast_attributes = {
  .name = "Can1Broadcast",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 4
};

/* Definitions for Can1Receive */
osThreadId_t Can1ReceiveHandle;
const osThreadAttr_t Can1Receive_attributes = {
  .name = "Can1Receive",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 4
};

/* Definitions for Can1BroadcastQ */
osMessageQueueId_t Can1BroadcastQHandle;
const osMessageQueueAttr_t Can1BroadcastQ_attributes = {
  .name = "Can1BroadcastQ"
};

/* Definitions for Can1ReceiveQ */
osMessageQueueId_t Can1ReceiveQHandle;
const osMessageQueueAttr_t Can1ReceiveQ_attributes = {
  .name = "Can1ReceiveQ"
};

/* Definitions for Can2Broadcast */
osThreadId_t Can2BroadcastHandle;
const osThreadAttr_t Can2Broadcast_attributes = {
  .name = "Can2Broadcast",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 4     //1024
};

/* Definitions for Can2Receive */
osThreadId_t Can2ReceiveHandle;
const osThreadAttr_t Can2Receive_attributes = {
  .name = "Can2Receive",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 4     //1024
};

/* Definitions for Can2BroadcastQ */
osMessageQueueId_t Can2BroadcastQHandle;
const osMessageQueueAttr_t Can2BroadcastQ_attributes = {
  .name = "Can2BroadcastQ"
};

/* Definitions for Can2ReceiveQ */
osMessageQueueId_t Can2ReceiveQHandle;
const osMessageQueueAttr_t Can2ReceiveQ_attributes = {
  .name = "Can2ReceiveQ"
};

/* Definitions for Can2ReceiveQ */
osMessageQueueId_t BootStartQHandle;
const osMessageQueueAttr_t BootStartQQ_attributes = {
  .name = "BootStartQ"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
void StartCan1BroadcastTask(void *argument);
void StartCan1ReceiveTask(void *argument);
void StartCan2BroadcastTask(void *argument);
void StartCan2ReceiveTask(void *argument);
void StartOTATask(void *argument);
void start_boot(uint32_t boot_info);

uint8_t boardID = 0xFF;

void Get_BoardID()
{
  boardID =
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 1) | HAL_GPIO_ReadPin(GPIOB,
                                                                    GPIO_PIN_1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_FilterTypeDef sCan1FilterConfig;
CAN_FilterTypeDef sCan2FilterConfig;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  if (flashOtaInfo.boot_mode != RAM_MAGIC_WORD) {
    start_boot(flashOtaInfo.using_app_id);
  }

  /* USER CODE END 1 */

  /* MCU Configuration-------------------------------------------------------- */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  Get_BoardID();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* Create the queue(s) */
  /* creation of Can1BroadcastQ */
  Can1BroadcastQHandle = osMessageQueueNew(16, sizeof(can_message_event_t),
                                           &Can1BroadcastQ_attributes);

  /* creation of Can1ReceiveQ */
  Can1ReceiveQHandle = osMessageQueueNew(16, sizeof(can_message_event_t),
                                         &Can1ReceiveQ_attributes);

  /* creation of Can2BroadcastQ */
  Can2BroadcastQHandle = osMessageQueueNew(16, sizeof(can_message_event_t),
                                           &Can2BroadcastQ_attributes);

  /* creation of Can2ReceiveQ */
  Can2ReceiveQHandle = osMessageQueueNew(16, sizeof(can_message_event_t),
                                         &Can2ReceiveQ_attributes);

  BootStartQHandle = osMessageQueueNew(8, sizeof(uint8_t), &BootStartQHandle);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* Create the thread(s) */
  /* creation of Can1Broadcast */
  Can1BroadcastHandle = osThreadNew(StartCan1BroadcastTask, NULL,
                                    &Can1Broadcast_attributes);

  /* creation of Can1Receive */
  Can1ReceiveHandle = osThreadNew(StartCan1ReceiveTask, NULL,
                                  &Can1Receive_attributes);

  /* creation of Can2Broadcast */
  Can2BroadcastHandle = osThreadNew(StartCan2BroadcastTask, NULL,
                                    &Can2Broadcast_attributes);

  /* creation of Can2Receive */
  Can2ReceiveHandle = osThreadNew(StartCan2ReceiveTask, NULL,
                                  &Can2Receive_attributes);

  OtaTaskHandle = osThreadNew(StartOTATask, NULL, &OtaTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /*##-2- Configure the CAN1 Filter ########################################### */
  sCan1FilterConfig.FilterBank = 0;
  sCan1FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sCan1FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sCan1FilterConfig.FilterIdHigh = 0x0000;
  sCan1FilterConfig.FilterIdLow = 0x0000;
  sCan1FilterConfig.FilterMaskIdHigh = 0x0000;
  sCan1FilterConfig.FilterMaskIdLow = 0x0000;
  sCan1FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sCan1FilterConfig.FilterActivation = ENABLE;
  sCan1FilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sCan1FilterConfig) != HAL_OK) {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ########################################### */
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    /* Start Error */
    Error_Handler();
  }

  /*##-4- Activate CAN RX notification ####################################### */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    /* Notification Error */
    Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  /*##-2- Configure the CAN1 Filter ########################################### */
  sCan2FilterConfig.FilterBank = 14;
  sCan2FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sCan2FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sCan2FilterConfig.FilterIdHigh = 0x0000;
  sCan2FilterConfig.FilterIdLow = 0x0000;
  sCan2FilterConfig.FilterMaskIdHigh = 0x0000;
  sCan2FilterConfig.FilterMaskIdLow = 0x0000;
  sCan2FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sCan2FilterConfig.FilterActivation = ENABLE;
  sCan2FilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &sCan2FilterConfig) != HAL_OK) {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ########################################### */
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    /* Start Error */
    Error_Handler();
  }

  /*##-4- Activate CAN RX notification ####################################### */
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    /* Notification Error */
    Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    GPIO_PIN_0 | PROG0_PIN_Pin | RESET_N_PIN_Pin | GPIO_PIN_7,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PROG0_PIN_Pin RESET_N_PIN_Pin PC7 */
  GPIO_InitStruct.Pin =
      GPIO_PIN_0 | PROG0_PIN_Pin | RESET_N_PIN_Pin | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 IO0_EXTI_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | IO0_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart2, (uint8_t *) & ch, 1, 0xFFFF);
  return ch;
}

// can related callback function when enable interrupt mode
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  HAL_StatusTypeDef HAL_RetStaus;
  can_message_event_t can_event_t;
  osStatus_t res;

  if (hcan == &hcan1) {
    HAL_RetStaus = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                        &can_event_t.CanHeader.RxHeader,
                                        can_event_t.CanData.RxData);
    if (HAL_OK == HAL_RetStaus) {
      if(can_event_t.CanHeader.RxHeader.IDE == CAN_ID_EXT)
        res = osMessageQueuePut(Can1ReceiveQHandle, &can_event_t, 0, 0);
      //CAN1 frame was successfully received.
    } else {
    }
  }

  if (hcan == &hcan2) {
    HAL_RetStaus = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                        &can_event_t.CanHeader.RxHeader,
                                        can_event_t.CanData.RxData);
    if (HAL_OK == HAL_RetStaus) {
      if(can_event_t.CanHeader.RxHeader.IDE == CAN_ID_EXT)
#if USE_P1
        res = osMessageQueuePut(Can2ReceiveQHandle, &can_event_t, 0, 0);
#else
        res = osMessageQueuePut(Can1ReceiveQHandle, &can_event_t, 0, 0);
#endif
      //CAN2 frame was successfully received.
    } else {
    }
  }
}

void start_boot(uint32_t boot_info)
{
  //1 stands for app2 while 0 stands for app1
  if (boot_info == 1) {
    /* Test if user code is programmed starting from address "USER_CODE2_START_ADDR" */
    if (((*(__IO uint32_t *) USER_CODE2_START_ADDR) & 0x2FFD0000) == 0x20000000) {
      /* Jump to user application */
      JumpAddress = *(__IO uint32_t *) (USER_CODE2_START_ADDR + 4);
      JumpToApplication = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t *) USER_CODE2_START_ADDR);
    }
  } else {
    /* Test if user code is programmed starting from address "USER_CODE1_START_ADDR" */
    if (((*(__IO uint32_t *) USER_CODE1_START_ADDR) & 0x2FFD0000) == 0x20000000) {
      /* Jump to user application */
      JumpAddress = *(__IO uint32_t *) (USER_CODE1_START_ADDR + 4);
      JumpToApplication = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t *) USER_CODE1_START_ADDR);
    }
  }

  /* reset all peripheral */
  __HAL_RCC_PWR_FORCE_RESET();
  __HAL_RCC_PWR_RELEASE_RESET();
  HAL_RCC_DeInit();

  JumpToApplication();
}

void can1_ota_ack(enum canComMode mode, ErrStatus status,
                  can_message_event_t * can_event_t)
{
  can_event_t->CanHeader.TxHeader.ExtId =
      (SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT) | (boardID <<
                                                                CHIP_ID_BIT_SHIFT)
      | mode;

  can_event_t->CanData.TxData[0] = status;

#if USE_P1
  osMessageQueuePut(Can1BroadcastQHandle, can_event_t, 0, 0);
#else
  osMessageQueuePut(Can2BroadcastQHandle, can_event_t, 0, 0);
#endif
}

void can1_ota_start_boot(void)
{
  can_message_event_t can_event_t;

  can_event_t.CanHeader.TxHeader.ExtId =
      (SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT) | (boardID <<
                                                                CHIP_ID_BIT_SHIFT)
      | CANCOM_OTA_BOOTUP;

  osMessageQueuePut(Can1ReceiveQHandle, &can_event_t, 0, 0);
}

uint32_t flash_get_ota_sector(void)
{
  return flashOtaInfo.updating_app_id;
}

uint32_t flash_get_boot_mode(void)
{
  return flashOtaInfo.boot_mode;
}

/**
 * @brief  Function implementing the Can1Broadcast thread.
 * @param  argument: Not used
 * @retval None
 */
void StartOTATask(void *argument)
{
  osStatus_t res;
  uint8_t boot_info;
  /* Infinite loop */
  for (;;) {
    res = osMessageQueueGet(BootStartQHandle, &boot_info, 0, portMAX_DELAY);
    __NVIC_SystemReset();
    osDelay(1);
  }
}

/**
 * @brief  Function implementing the Can1Broadcast thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan1BroadcastTask(void *argument)
{
  uint32_t TxMailbox;
  osStatus_t res;
  can_message_event_t can_event_t;

  /* Infinite loop */
  for (;;) {
    res =
        osMessageQueueGet(Can1BroadcastQHandle, &can_event_t, 0, portMAX_DELAY);
    printf("CAN1 BC MSG: (0x%x) \r\n", can_event_t.CanHeader.TxHeader.ExtId);
    /*Configure Transmission process */
    can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
    can_event_t.CanHeader.TxHeader.IDE = CAN_ID_EXT;
    can_event_t.CanHeader.TxHeader.DLC = 8;
    can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
    /* CAN send test. */
    if (HAL_CAN_AddTxMessage(&hcan1, &can_event_t.CanHeader.TxHeader,
                             can_event_t.CanData.TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      Error_Handler();
    }
    osDelay(1);
  }
}

/**
 * @brief  Function implementing the Can1Receive thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan1ReceiveTask(void *argument)
{
  /* Infinite loop */
  can_message_event_t can_event_t;
  can_message_event_t can_ack_event_t;
  osStatus_t res;
  sensor_message_event_type event_type;
  uint8_t chip_id;
  uint8_t cancom_mode;
  uint32_t boot_info;

  uint32_t valBuf[2], binSize = 0, binPackNum = 0, can_recv_count = 0;
  uint32_t address = 0, ack_err_address = 0;
  uint32_t ret = 0;
  uint32_t boot_sector = 0;
  uint32_t current_ota_sector = 0;

  static uint32_t count = 0;

  for (;;) {
    //printf("waiting for can1 message \r\n");
    res = osMessageQueueGet(Can1ReceiveQHandle, &can_event_t, 0, portMAX_DELAY);
    //printf("rev can1 msg StdId: 0x%x\r\n", can_event_t.CanHeader.RxHeader.StdId);
    printf("rev can1 msg ExtId: 0x%x\r\n",
           can_event_t.CanHeader.RxHeader.ExtId);

    event_type =
        (can_event_t.CanHeader.RxHeader.
         ExtId & SENSOR_EVENT_MESSAGE_BIT_MASK) >>
        SENSOR_EVENT_MESSAGE_BIT_SHIFT;
    chip_id =
        (can_event_t.CanHeader.RxHeader.
         ExtId & CHIP_ID_BIT_MASK) >> CHIP_ID_BIT_SHIFT;
    cancom_mode = can_event_t.CanHeader.RxHeader.ExtId & CANCOM_MODE_BIT_MASK;
    can_recv_count =
        (can_event_t.CanHeader.RxHeader.
         ExtId & OTA_EXTEND_MESSAGE_BIT_MASK) >> OTA_EXTEND_MESSAGE_BIT_SHIFT;

    if (chip_id != boardID) {
      //foward this can message to can2 canmessage send
      res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
    } else {
      switch (cancom_mode) {
      case CANCOM_OTA_INFO:
        memcpy(&binSize, &(can_event_t.CanData.RxData[0]), sizeof(uint32_t));
        memcpy(&binPackNum, &(can_event_t.CanData.RxData[4]), sizeof(uint32_t));
        count = 0;
        if (flash_get_ota_sector() == 0) {
          address = USER_CODE1_START_ADDR;
          current_ota_sector = 0;
        } else {
          address = USER_CODE2_START_ADDR;
          current_ota_sector = 1;
        }
        //erase flash here before write operation
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                               FLASH_FLAG_PGSERR);
        flash_erase_by_sector(address, 3);
        can1_ota_ack(CANCOM_OTA_INFO_ACK, CAN_OTA_SUCCESS, &can_ack_event_t);
        break;
      case CANCOM_OTA_ING:
        if (can_recv_count == count) {
          if (count == 20442) {
            uint32_t add_temp = address + count * 8;
            printf("debug, address + count*8: 0x%x", add_temp);
          }
          memcpy(valBuf, can_event_t.CanData.RxData, 8);

          if (__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                                    FLASH_FLAG_PGSERR)) != RESET) {

            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                                   FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                                   FLASH_FLAG_PGSERR);
            ack_err_address = 0xFFFFFFFF;
            memcpy(&can_ack_event_t.CanData.TxData[1], &ack_err_address,
                   sizeof(uint32_t));
            can1_ota_ack(CANCOM_OTA_ING_ACK, CAN_OTA_ERROR, &can_ack_event_t);
            break;
          }

          ret = flash_program_by_word(address + count * 8, valBuf[0]);
          if (ret != HAL_OK) {
            ack_err_address = address + count * 8;
            memcpy(&can_ack_event_t.CanData.TxData[1], &ack_err_address,
                   sizeof(uint32_t));
            can1_ota_ack(CANCOM_OTA_ING_ACK, CAN_OTA_ERROR, &can_ack_event_t);
            break;
          }
          ret = flash_program_by_word(address + count * 8 + 4, valBuf[1]);
          if (ret != HAL_OK) {
            ack_err_address = address + count * 8 + 4;
            memcpy(&can_ack_event_t.CanData.TxData[1], &ack_err_address,
                   sizeof(uint32_t));
            can1_ota_ack(CANCOM_OTA_ING_ACK, CAN_OTA_ERROR, &can_ack_event_t);
            break;
          }
          can1_ota_ack(CANCOM_OTA_ING_ACK, CAN_OTA_SUCCESS, &can_ack_event_t);
          count++;
        } else {
          //force set count to can_recv_count
          count = can_recv_count;
          memcpy(&can_ack_event_t.CanData.TxData[1], &count, sizeof(uint32_t));
          can1_ota_ack(CANCOM_OTA_ING_ACK, CAN_OTA_ERROR, &can_ack_event_t);
        }
        break;
      case CANCOM_OTA_END:
        memcpy(&binPackNum, &(can_event_t.CanData.RxData[0]), sizeof(uint32_t));
        if ((count == binPackNum) && (binPackNum > 0)) {
          can1_ota_ack(CANCOM_OTA_END_ACK, CAN_OTA_SUCCESS, &can_ack_event_t);
          //upadte boot up start address stored in FLASH_OTAINFO_ADD
          flash_set_bootup_sector(current_ota_sector);
          flash_clear_boot_magic_number();

          ret = flash_lock();
          if (ret == HAL_OK) {
            can1_ota_ack(CANCOM_OTA_END_ACK, CAN_OTA_SUCCESS, &can_ack_event_t);
          } else {
            can1_ota_ack(CANCOM_OTA_END_ACK, CAN_OTA_ERROR, &can_ack_event_t);
          }
          osDelay(1);
          can1_ota_start_boot();
          break;
        } else {
          memcpy(can_event_t.CanData.TxData, &count, 2);
          can1_ota_ack(CANCOM_OTA_END_ACK, CAN_OTA_ERROR, &can_ack_event_t);
          osDelay(1);
          flash_clear_boot_magic_number();
          ret = flash_lock();
          boot_info = flashOtaInfo.using_app_id;
          osMessageQueuePut(BootStartQHandle, &boot_info, 0, 0);
          break;
        }
      case CANCOM_OTA_START:
        ret = flash_unlock();
        if (ret == HAL_OK) {
          memcpy(&can_ack_event_t.CanData.TxData[1],
                 &flashOtaInfo.updating_app_id, sizeof(uint32_t));
          //different update address need a different image APP, tell the master which is updating
          can1_ota_ack(CANCOM_OTA_START_ACK, CAN_OTA_SUCCESS, &can_ack_event_t);
        } else {
          can1_ota_ack(CANCOM_OTA_START_ACK, CAN_OTA_ERROR, &can_ack_event_t);
        }
        break;
      case CANCOM_OTA_MODE_ENTER:
        can1_ota_ack(CANCOM_OTA_MODE_ENTER_ACK, CAN_OTA_SUCCESS,
                     &can_ack_event_t);
        break;
      case CANCOM_OTA_BOOTUP:
        boot_info = flashOtaInfo.using_app_id;
        flash_unlock();
        flash_clear_boot_magic_number();
        flash_lock();
        can1_ota_ack(CANCOM_OTA_BOOTUP_ACK, CAN_OTA_SUCCESS, &can_ack_event_t);
        osDelay(10);
        osMessageQueuePut(BootStartQHandle, &boot_info, 0, 0);
        break;
      case CANCOM_OTA_SETBOOT_SECTOR:
        boot_sector = can_event_t.CanData.RxData[0];
        ret = flash_unlock();
        if (ret == HAL_OK) {
          //can1_ota_ack(CANCOM_OTA_SETBOOT_SECTOR_ACK, CAN_OTA_SUCCESS, &can_ack_event_t);
        } else {
          can1_ota_ack(CANCOM_OTA_SETBOOT_SECTOR_ACK, CAN_OTA_ERROR,
                       &can_ack_event_t);
        }
        osDelay(1);
        flash_set_bootup_sector(boot_sector);
        ret = flash_lock();
        if (ret == HAL_OK) {
          can1_ota_ack(CANCOM_OTA_SETBOOT_SECTOR_ACK, CAN_OTA_SUCCESS,
                       &can_ack_event_t);
        } else {
          can1_ota_ack(CANCOM_OTA_SETBOOT_SECTOR_ACK, CAN_OTA_ERROR,
                       &can_ack_event_t);
          break;
        }
        break;
      default:
        break;
      }
    }
    //parse can1 message and foward to tasks, RxData is only sizeof uint8_t * 8
    osDelay(1);
  }
}

/**
 * @brief  Function implementing the Can2Broadcast thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan2BroadcastTask(void *argument)
{
  uint32_t TxMailbox;
  osStatus_t res;
  can_message_event_t can_event_t;

  /* Infinite loop */
  for (;;) {
    //for master stm32 can2 is used to broad cast config message to slave stm32 can1
    res =
        osMessageQueueGet(Can2BroadcastQHandle, &can_event_t, 0, portMAX_DELAY);
    printf("CAN2 BC MSG: (0x%x) res: %d \r\n",
           (uint32_t) can_event_t.CanHeader.TxHeader.ExtId, res);
    /*Configure Transmission process */
    can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
    can_event_t.CanHeader.TxHeader.IDE = CAN_ID_EXT;
    can_event_t.CanHeader.TxHeader.DLC = 8;
    can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
    /* CAN send test. */
    if (HAL_CAN_AddTxMessage(&hcan2, &can_event_t.CanHeader.TxHeader,
                             can_event_t.CanData.TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      Error_Handler();
    }

    osDelay(1);
  }

}

/**
 * @brief  Function implementing the Can2Receive thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan2ReceiveTask(void *argument)
{
  /* Infinite loop */
  can_message_event_t can_event_t;
  osStatus_t res;
  sensor_message_event_type event_type;
  uint8_t chip_id;
  uint8_t cancom_mode;

  for (;;) {
    //for master stm32 can2 will receive data from slave stm32
    //for slave stm32 can2 is disabled
    res = osMessageQueueGet(Can2ReceiveQHandle, &can_event_t, 0, portMAX_DELAY);
    printf("rev can2 msg StdId: 0x%x, ExtId: 0x%x\r\n",
           can_event_t.CanHeader.RxHeader.StdId,
           can_event_t.CanHeader.RxHeader.ExtId);

    event_type =
        (can_event_t.CanHeader.RxHeader.
         ExtId & SENSOR_EVENT_MESSAGE_BIT_MASK) >>
        SENSOR_EVENT_MESSAGE_BIT_SHIFT;
    chip_id =
        (can_event_t.CanHeader.RxHeader.
         ExtId & CHIP_ID_BIT_MASK) >> CHIP_ID_BIT_SHIFT;
    cancom_mode = can_event_t.CanHeader.RxHeader.ExtId & CANCOM_MODE_BIT_MASK;

    if (chip_id != boardID) {
      //foward this can message to can1 can message send
      res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
    }

    osDelay(1);
  }
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

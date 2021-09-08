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
/* USER CODE BEGIN Includes */
#include "sensor_device.h"
#include "can_proto.h"
#include "flash_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USE_P1 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TASK_STACK_SIZE_IN_WORD 1024
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 4
};

/* Definitions for DataTimer */
osTimerId_t DataTimerHandle;
const osTimerAttr_t DataTimer_attributes = {
  .name = "DataTimer"
};

/* USER CODE BEGIN PV */

platform_prams platformInitPrams;

/* Definitions for SensorManager */
//SensorManager Process is used for handling
//Hardware Sensor Operations of accessing bus to read data
osThreadId_t SensorManagerHandle;
const osThreadAttr_t SensorManager_attributes = {
  .name = "SensorManager",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 4 * 4
};

/* Definitions for SensorDataProc */
//SensorDataProc is waiting SensorDataQ Events to Process
//Can2Receiver or SensorMAnagerProcess may send events to SensroDataQ
osThreadId_t SensorDataProcHandle;
const osThreadAttr_t SensorDataProc_attributes = {
  .name = "SensorDataProc",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 4 * 4
};

/* Definitions for SensorDataProcQ */
osMessageQueueId_t SensorDataQHandle;
const osMessageQueueAttr_t SensorDataQ_attributes = {
  .name = "SensorDataQ"
};

/* Definitions for SensorMsgQHandle */
osMessageQueueId_t SensorMsgQHandle;
const osMessageQueueAttr_t SensorMsgQ_attributes = {
  .name = "SensorMsgQ"
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

/* Definitions for Uart2Task */
osThreadId_t Uart2TaskHandle;
const osThreadAttr_t Uart2Task_attributes = {
  .name = "Uart2Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TASK_STACK_SIZE_IN_WORD * 2     //1024
};

/* Definitions for Uart2TaskQ */
osMessageQueueId_t Uart2MsgQHandle;
const osMessageQueueAttr_t Uart2MsgQ_attributes = {
  .name = "Uart2TaskQ"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
void StartInitTask(void *argument);
void DataTimerCallback(void *argument);

/* USER CODE BEGIN PFP */
void StartCan1BroadcastTask(void *argument);
void StartCan1ReceiveTask(void *argument);
void StartCan2BroadcastTask(void *argument);
void StartCan2ReceiveTask(void *argument);

void StartUart2Task(void *argument);

void StartSensorManagerTask(void *argument);
void StartSensorDataProcesser(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_FilterTypeDef sCan1FilterConfig;
CAN_FilterTypeDef sCan2FilterConfig;

uint8_t boardID = 0xFF;
uint16_t CAN1_ENABLED_SENSORBIT = 0x0;
uint16_t CAN1_GET_CALI_SENSORBIT = 0x0;
static bool board_init_complete = false;
static uint32_t software_version = 0x21080214;
static bool uart_log_print_enable = true;

static uint8_t uart_msg_code[UART_TX_CODE_LENGTH] = { 0 };

// uart protocal define
/*
 * 					帧头1	帧头2	数据长度			类型(ID)		数据				校验字chkSum
 *			字节数		1		1		1				1			n					2
 *	master->slaver	0XAA	0X55	0~255【1+n】		0X00~0XFF	见协议		【数据长度~数据】所有按字节相加，取低16位，并按[L8,H8]排列
 *	slaver->master	0X5A	0XA5	0~255【1+n】		0X00~0XFF	见协议		【数据长度~数据】所有按字节相加，取低16位，并按[L8,H8]排列
 *
 *	BYTE0
 *	帧计数
 *	0~255循环
 *
 *	BYTE1~4
 *	时间戳
 *	"u32
 *	单位us"
 *
 *	BYTE5~6
 *	accRange
 *	"u16类型
 *	加速度计量程（单位mg）
 *	4000对应的是±4000mg"
 *
 *	BYTE7~8
 *	acc_x
 *	"S16类型
 *	加速度计X轴
 *	float取值：
 *	acc_x*accRange/32768"
 *
 *	BYTE9~10
 *	acc_y
 *	"S16类型
 *	加速度计Y轴
 *	float取值:
 *	acc_y*accRange/32768"
 *
 *	BYTE11~12
 *	acc_z
 *	"S16类型
 *	加速度计Z轴
 *	float取值:
 *	acc_z*accRange/32768"
 *
 *	BYTE13~14
 *	gyroRange
 *	"u16类型
 *	陀螺仪量程
 *	单位 °/s
 *	500对应的是±500°/s"
 *
 *	BYTE15~16
 *	gyro_x
 *	"S16类型
 *	陀螺仪X轴
 *	float取值:
 *	gyro_x*gyroRange/32768"
 *
 *	BYTE17~18
 *	gyro_y
 *	"S16类型
 *	陀螺仪Y轴
 *	float取值:
 *	gyro_y*gyroRange/32768"
 *
 *	BYTE19~20
 *	gyro_z
 *	"S16类型
 *	陀螺仪Z轴
 *	float取值:
 *	gyro_z*gyroRange/32768"
 *
 *	BYTE21~22
 *	temperature
 *	"S16类型 温度
 *	单位：0.1℃"
 * */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
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

  /* Create the timer(s) */
  /* creation of DataTimer */
  DataTimerHandle =
      osTimerNew(DataTimerCallback, osTimerPeriodic, NULL,
                 &DataTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* Create the queue(s) */
  /* creation of Can1BroadcastQ */
  Can1BroadcastQHandle = osMessageQueueNew(32, sizeof(can_message_event_t),
                                           &Can1BroadcastQ_attributes);

  /* creation of Can1ReceiveQ */
  Can1ReceiveQHandle = osMessageQueueNew(32, sizeof(can_message_event_t),
                                         &Can1ReceiveQ_attributes);

  /* creation of Can2BroadcastQ */
  Can2BroadcastQHandle = osMessageQueueNew(32, sizeof(can_message_event_t),
                                           &Can2BroadcastQ_attributes);

  /* creation of Can2ReceiveQ */
  Can2ReceiveQHandle = osMessageQueueNew(32, sizeof(can_message_event_t),
                                         &Can2ReceiveQ_attributes);

  /* creation of Uart2TaskQ */
  Uart2MsgQHandle = osMessageQueueNew(32, sizeof(uint8_t) * UART_TX_CODE_LENGTH,
                                      &Uart2MsgQ_attributes);

  /* creation of SensorDataQ */
  SensorDataQHandle = osMessageQueueNew(64, sizeof(sensors_event_t),
                                        &SensorDataQ_attributes);

  /* creation of SensorMessageQ */
  SensorMsgQHandle = osMessageQueueNew(32, sizeof(sensor_message_event_t),
                                       &SensorMsgQ_attributes);
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

  /* creation of Uart2Task */
  Uart2TaskHandle = osThreadNew(StartUart2Task, NULL, &Uart2Task_attributes);

  /* creation of SensorManager */
  //sensor manager thread need to handle different config messages such as selftest/calibration/timer
  SensorManagerHandle = osThreadNew(StartSensorManagerTask, NULL,
                                    &SensorManager_attributes);

  /* creation of SensorDataProc */
  SensorDataProcHandle = osThreadNew(StartSensorDataProcesser, NULL,
                                     &SensorDataProc_attributes);

  //init sensor related parameters
  platformInitPrams.boardID = boardID;
  platformInitPrams.i2c_handle = hi2c2;
  platformInitPrams.uart = huart2;
  platformInitPrams.spi_handle = hspi1;
  platformInitPrams.SensorMessageQHandle = SensorMsgQHandle;
  platformInitPrams.SensorDataQHandle = SensorDataQHandle;
  platformInitPrams.UartReportQHandle = Uart2MsgQHandle;
  platformInitPrams.PollingTimerHandle = DataTimerHandle;
  platformInitPrams.OperateTimerHandle = htim3;
  platformInitPrams.PWMTimerHandle = htim2;
  platformInitPrams.use_uart_mode = false;
  platformInitPrams.tof_cs_gpio_port = TOF_CS_GPIO_Port;
  platformInitPrams.tof_cs_pin = TOF_CS_Pin;
  platformInitPrams.board_calidata =
      (calibrationData *) calloc(1, sizeof(calibrationData));
  if (platformInitPrams.board_calidata == NULL) {
    /* Notification Error */
    Error_Handler();
  }

  memcpy(platformInitPrams.board_calidata, &flashCaliInfo,
         sizeof(calibrationData));

  sensor_register(&platformInitPrams);

  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

  //osTimerStart(DataTimerHandle, 5);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
      != HAL_OK) {
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
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)
      != HAL_OK) {
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
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };
  TIM_OC_InitTypeDef sConfigOC = { 0 };
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 179;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.RepetitionCounter = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 6000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim2, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };
  TIM_OC_InitTypeDef sConfigOC = { 0 };

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_UART_Receive_DMA(&huart2, uart_msg_code, UART_RX_CODE_LENGTH);

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

  /*Configure GPIO pins : REAR_PROG0_PIN_Pin HEAD_PROG0_PIN_Pin HEAD_AND_REAR_ULTRALSONIC_RESET_N_PIN_Pin GUANGLIU_EN_Pin
     TOF_EN_Pin IMU_INT_Pin TOF_GPIO2_Pin */
  GPIO_InitStruct.Pin =
      REAR_PROG0_PIN_Pin | HEAD_PROG0_PIN_Pin |
      HEAD_AND_REAR_ULTRALSONIC_RESET_N_PIN_Pin | GUANGLIU_EN_Pin | TOF_EN_Pin |
      IMU_INT_Pin | TOF_GPIO2_Pin | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : HEAD_AND_REAR_ULTRASONIC_INT_Pin */
  GPIO_InitStruct.Pin = HEAD_AND_REAR_ULTRASONIC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HEAD_AND_REAR_ULTRASONIC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_CS_Pin */
  GPIO_InitStruct.Pin = TOF_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HEAD_AND_REAR_LED_DRIVER_EN_Pin */
  GPIO_InitStruct.Pin = HEAD_AND_REAR_LED_DRIVER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_GPIO1_Pin */
  GPIO_InitStruct.Pin = TOF_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    REAR_PROG0_PIN_Pin | HEAD_PROG0_PIN_Pin |
                    HEAD_AND_REAR_ULTRALSONIC_RESET_N_PIN_Pin | GUANGLIU_EN_Pin
                    | TOF_EN_Pin | IMU_INT_Pin | TOF_GPIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOF_CS_GPIO_Port, TOF_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEAD_AND_REAR_LED_DRIVER_EN_GPIO_Port,
                    HEAD_AND_REAR_LED_DRIVER_EN_Pin, GPIO_PIN_RESET);

  /*ultralsound intterupt level shift enable */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

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
  if (uart_log_print_enable)
    HAL_UART_Transmit(&huart2, (uint8_t *) & ch, 1, 0xFFFF);
  //HAL_UART_Transmit_DMA(&huart2, (uint8_t*) &ch, 1);
  return ch;
}

void Get_BoardID()
{
  boardID =
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 1) | HAL_GPIO_ReadPin(GPIOB,
                                                                    GPIO_PIN_1);
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
      res = osMessageQueuePut(Can1ReceiveQHandle, &can_event_t, 0, 0);
      //CAN1 frame was successfully received.
      if (res != osOK) {
      }
    } else {

    }
  }

  if (hcan == &hcan2) {
    HAL_RetStaus = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                        &can_event_t.CanHeader.RxHeader,
                                        can_event_t.CanData.RxData);
    if (HAL_OK == HAL_RetStaus) {
#if USE_P1
      res = osMessageQueuePut(Can2ReceiveQHandle, &can_event_t, 0, 0);
      if (res != osOK) {
      }
#else
      res = osMessageQueuePut(Can1ReceiveQHandle, &can_event_t, 0, 0);
      if (res != osOK) {
      }
#endif
      //CAN2 frame was successfully received.
    } else {

    }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  if (hcan == &hcan1) {
    printf("can1 errorcode: 0x%x \r\n", hcan->ErrorCode);
  }

  if (hcan == &hcan2) {
    printf("can2 errorcode: 0x%x \r\n", hcan->ErrorCode);
  }
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_ErrorCallback could be implemented in the user file
   */
}

// uart related callback function when enable interrupt mode
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  HAL_StatusTypeDef HAL_RetStaus;
  osStatus_t res;

  if (huart == &huart2) {
    res = osMessageQueuePut(Uart2MsgQHandle, uart_msg_code, 0, 0);
    if (res != osOK) {
      printf("res:%d", res);
    }

  }
}

/**
 * @brief  Function implementing the Can1Broadcast thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan1BroadcastTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t TxMailbox;
  osStatus_t res;
  can_message_event_t can_event_t;

  /* Infinite loop */
  for (;;) {
    res =
        osMessageQueueGet(Can1BroadcastQHandle, &can_event_t, 0, portMAX_DELAY);
    //printf("CAN1 BC MSG: (0x%x) \r\n",
    //         can_event_t.CanHeader.TxHeader.StdId);
    /*Configure Transmission process */
    can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
    can_event_t.CanHeader.TxHeader.IDE = CAN_ID_STD;
    can_event_t.CanHeader.TxHeader.DLC = 8;
    can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
    /* CAN send test. */
    if (HAL_CAN_AddTxMessage(&hcan1, &can_event_t.CanHeader.TxHeader,
                             can_event_t.CanData.TxData,
                             &TxMailbox) != HAL_OK) {
      printf("can1 errorcode: 0x%x \r\n", hcan1.ErrorCode);
      /* Transmission request Error */
      Error_Handler();
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Function implementing the Can1Receive thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan1ReceiveTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  can_message_event_t can_event_t;
  sensor_message_event_t sensor_msg_event;
  osStatus_t res;
  sensor_message_event_type event_type;
  uint8_t sensor_type;
  int8_t command_code;
  for (;;) {
    //printf("waiting for can1 message \r\n");
    res = osMessageQueueGet(Can1ReceiveQHandle, &can_event_t, 0, portMAX_DELAY);
    printf("rev can1 msg StdId: 0x%x\r\n",
           can_event_t.CanHeader.RxHeader.StdId);
    event_type =
        (can_event_t.CanHeader.RxHeader.
         StdId & SENSOR_EVENT_MESSAGE_BIT_MASK) >>
        SENSOR_EVENT_MESSAGE_BIT_SHIFT;
    sensor_type =
        (can_event_t.CanHeader.RxHeader.
         StdId & SENSOR_TYPE_BIT_MASK) >> SENSOR_TYPE_BIT_SHIFT;
    command_code =
        can_event_t.CanHeader.RxHeader.StdId & SENSOR_COMMAND_BIT_MASK;
    //printf("can code: {%d, %d, %d}\r\n", event_type, sensor_type, command_code);
    if ((event_type == SENSOR_DEBUG_CONFIG_MSG)
        && (sensor_type == SENSOR_TYPE_LED_HEAD
            || sensor_type == SENSOR_TYPE_LED_REAR)) {
      if (sensor_avaiable_check(sensor_type)) {
        sensor_msg_event.message_event_type = event_type;
        sensor_msg_event.message_event_t.config_event.config_type =
            SENSOR_LED_MODE_CONFIG;
        sensor_msg_event.message_event_t.config_event.cfg_data.
            config_data_u8[0] = can_event_t.CanData.RxData[0];
        sensor_msg_event.message_event_t.config_event.sensor_type = sensor_type;
        res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
      } else
        res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);

    } else if (event_type == SENSOR_CONFIG_MESSAGE) {
      //check if supporting this kind of sensor
      if (command_code == SENSOR_ACTIVATE)
        CAN1_ENABLED_SENSORBIT |= (1 << sensor_type);
      else if (command_code == SENSOR_DEACTIVATE)
        CAN1_ENABLED_SENSORBIT &= ~(1 << sensor_type);
      if (command_code == SENSOR_CALIBRATION_RESULT)
        CAN1_GET_CALI_SENSORBIT |= (1 << sensor_type);

      if (sensor_avaiable_check(sensor_type)) {
        switch (command_code) {
        case SENSOR_ACTIVATE:
          //receive sensor enable message and forward it to smgr
          sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
          sensor_msg_event.message_event_t.config_event.config_type =
              SENSOR_ACTIVATE;
          sensor_msg_event.message_event_t.config_event.sensor_type =
              sensor_type;
          res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
          break;
        case SENSOR_DEACTIVATE:
          sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
          sensor_msg_event.message_event_t.config_event.config_type =
              SENSOR_DEACTIVATE;
          sensor_msg_event.message_event_t.config_event.sensor_type =
              sensor_type;
          res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
          break;
        case SENSOR_CONFIG_SELFTEST:
          sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
          sensor_msg_event.message_event_t.config_event.config_type =
              SENSOR_CONFIG_SELFTEST;
          sensor_msg_event.message_event_t.config_event.sensor_type =
              sensor_type;
          res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
          break;
        case SENSOR_CONFIG_CALIBRATION:
          sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
          sensor_msg_event.message_event_t.config_event.config_type =
              SENSOR_CONFIG_CALIBRATION;
          sensor_msg_event.message_event_t.config_event.sensor_type =
              sensor_type;
          res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
          break;
        case SENSOR_CALIBRATION_RESULT:
          sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
          sensor_msg_event.message_event_t.config_event.config_type =
              SENSOR_CALIBRATION_RESULT;
          sensor_msg_event.message_event_t.config_event.sensor_type =
              sensor_type;
          res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
          break;
        default:
          break;
        }
        if (sensor_type == SENSOR_TYPE_MAX) {
          //foward sensor command to can2 if this sensor is a all sensor enable message
          res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
        }
      } else {
        //foward sensor command to can2 if this sensor is not on this chip
        res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
      }
    } else if (event_type == SENSOR_DATA_MESSAGE
               || event_type == SENSOR_TIMESTAMP_MESSAGE) {
      //can1 receive should not handle data or timestamp msg to avoid can msg storm on can net work between slave stm32s
      printf("Can1ReceiveQ data or timstamp msg event_type = %d\r\n",
             event_type);
    } else if (event_type == SENSOR_OTA_MESSAGE) {
      sensor_message_event_type event_type;
      uint8_t chip_id;
      uint8_t cancom_mode;

      event_type =
          (can_event_t.CanHeader.RxHeader.
           StdId & SENSOR_EVENT_MESSAGE_BIT_MASK) >>
          SENSOR_EVENT_MESSAGE_BIT_SHIFT;
      chip_id =
          (can_event_t.CanHeader.RxHeader.
           StdId & CHIP_ID_BIT_MASK) >> CHIP_ID_BIT_SHIFT;
      cancom_mode = can_event_t.CanHeader.RxHeader.StdId & CANCOM_MODE_BIT_MASK;
      if (chip_id == boardID) {
        if ((cancom_mode == CANCOM_OTA_MODE_ENTER)) {
          //first update flash
          flash_unlock();
          flash_update_boot_magic_number();
          flash_lock();
          can_event_t.CanHeader.TxHeader.StdId =
              (SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
              | (boardID << SENSOR_TYPE_BIT_SHIFT)
              | CANCOM_OTA_MODE_ENTER_ACK;
#if USE_P1
          res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
#else
          res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
#endif
          //than reset
          osDelay(5);
          __NVIC_SystemReset();
        }
      } else {
        //foward this message to can2 to tell sub stm32
        res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
      }
    } else if (event_type == SENSOR_VERSION_MSG) {
      sensor_message_event_type event_type;
      uint8_t chip_id;
      uint8_t cancom_mode;

      event_type =
          (can_event_t.CanHeader.RxHeader.
           StdId & SENSOR_EVENT_MESSAGE_BIT_MASK) >>
          SENSOR_EVENT_MESSAGE_BIT_SHIFT;
      chip_id =
          (can_event_t.CanHeader.RxHeader.
           StdId & CHIP_ID_BIT_MASK) >> CHIP_ID_BIT_SHIFT;
      cancom_mode = can_event_t.CanHeader.RxHeader.StdId & CANCOM_MODE_BIT_MASK;
      if (chip_id == boardID) {
        can_event_t.CanHeader.TxHeader.StdId =
            (SENSOR_VERSION_MSG << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
            | (boardID << SENSOR_TYPE_BIT_SHIFT)
            | 0xf;
        memcpy(&can_event_t.CanData.TxData[0], &software_version,
               sizeof(uint32_t));
        res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
      } else {
        //foward this message to can2 to tell sub stm32
        res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
      }
    } else if (event_type == SENSOR_TIME_SYNC_MSG) {
      sensor_message_event_type event_type;
      uint8_t chip_id;

      event_type =
          (can_event_t.CanHeader.RxHeader.
           StdId & SENSOR_EVENT_MESSAGE_BIT_MASK) >>
          SENSOR_EVENT_MESSAGE_BIT_SHIFT;
      chip_id =
          (can_event_t.CanHeader.RxHeader.
           StdId & CHIP_ID_BIT_MASK) >> CHIP_ID_BIT_SHIFT;

      if (chip_id == boardID) {
        can_event_t.CanHeader.TxHeader.StdId =
            (SENSOR_TIME_SYNC_MSG << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
            | (boardID << SENSOR_TYPE_BIT_SHIFT)
            | 0xf;
        uint64_t current_time = sensor_get_timestamp();
        memcpy(&can_event_t.CanData.TxData[0], &current_time, sizeof(uint64_t));
        res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
      } else {
        //foward this message to can2 to tell sub stm32
        res = osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
      }
    } else {
      printf("Can1ReceiveQ unsupported msg event_type = %d \r\n", event_type);
    }
    //parse can1 message and foward to tasks, RxData is only sizeof uint8_t * 8
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Function implementing the Can2Broadcast thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan2BroadcastTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t TxMailbox;
  osStatus_t res;
  can_message_event_t can_event_t;

  /* Infinite loop */
  for (;;) {
    //for master stm32 can2 is used to broad cast config message to slave stm32 can1
    res =
        osMessageQueueGet(Can2BroadcastQHandle, &can_event_t, 0, portMAX_DELAY);
    //printf("CAN2 BC MSG: (0x%x) \r\n", can_event_t.CanHeader.TxHeader.StdId);
    /*Configure Transmission process */
    can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
    can_event_t.CanHeader.TxHeader.IDE = CAN_ID_STD;
    can_event_t.CanHeader.TxHeader.DLC = 8;
    can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
    /* CAN send test. */
    if (HAL_CAN_AddTxMessage(&hcan2, &can_event_t.CanHeader.TxHeader,
                             can_event_t.CanData.TxData,
                             &TxMailbox) != HAL_OK) {
      printf("can2 errorcode: 0x%x \r\n", hcan2.ErrorCode);
      /* Transmission request Error */
      Error_Handler();
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Function implementing the Can2Receive thread.
 * @param  argument: Not used
 * @retval None
 */
void StartCan2ReceiveTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  can_message_event_t can_event_t;
  osStatus_t res;
  sensor_message_event_t sensor_msg_event;
  sensor_message_event_type event_type;
  sensors_event_t sensor_data[SENSOR_TYPE_MAX];
  uint8_t sensor_type;
  int8_t command_code;
  uint8_t data_index;
  uint8_t data_size[SENSOR_TYPE_MAX] = { 0 };
  uint8_t data_bitmask[SENSOR_TYPE_MAX] = { 0 };

  for (;;) {
    //for master stm32 can2 will receive data from slave stm32
    //for slave stm32 can2 is disabled
    res = osMessageQueueGet(Can2ReceiveQHandle, &can_event_t, 0, portMAX_DELAY);
    //printf("rev can2 msg StdId: 0x%x, ExtId: 0x%x\r\n", can_event_t.CanHeader.RxHeader.StdId, can_event_t.CanHeader.RxHeader.ExtId);
    event_type =
        (can_event_t.CanHeader.RxHeader.
         StdId & SENSOR_EVENT_MESSAGE_BIT_MASK) >>
        SENSOR_EVENT_MESSAGE_BIT_SHIFT;
    sensor_type =
        (can_event_t.CanHeader.RxHeader.
         StdId & SENSOR_TYPE_BIT_MASK) >> SENSOR_TYPE_BIT_SHIFT;
    command_code =
        can_event_t.CanHeader.RxHeader.StdId & SENSOR_COMMAND_BIT_MASK;
    //printf("can code: {%d, %d, %d}\r\n", event_type, sensor_type, command_code);
    //if can2 receive data message transfer it to sensordataQ
    //parse can2 message and foward to tasks, RxData is only sizeof uint8_t * 8
    if (event_type == SENSOR_CONFIG_MESSAGE) {
      //check if supporting this kind of sensor
      printf("Can2ReceiveQ config msg \r\n");
    } else if (event_type == SENSOR_DATA_MESSAGE) {
      //printf("Can2ReceiveQ data msg \r\n");
      //forward can2 data message to SensorDataQ
      //when receiving data message, command code is axis_num + 1, parse sensordata here
      data_size[sensor_type] = command_code;
      sensor_data[sensor_type].sensor_type = sensor_type;
      data_index = can_event_t.CanData.RxData[0];
      if (!(data_bitmask[sensor_type] & (1 << data_index))) {
        memcpy(&sensor_data[sensor_type].sensor_data_t.vec.data[data_index],
               &can_event_t.CanData.RxData[1], sizeof(float));
        data_bitmask[sensor_type] =
            data_bitmask[sensor_type] | (1 << data_index);
      } else {
        //drop data since received data index is not under expect
        printf("unexpected data index arrived\r\n");
      }

    } else if (event_type == SENSOR_TIMESTAMP_MESSAGE) {
      if (data_bitmask[sensor_type] ==
          ((uint8_t) pow(2, data_size[sensor_type]) - 1)) {
        memcpy(&sensor_data[sensor_type].timestamp,
               &can_event_t.CanData.RxData[0], sizeof(uint32_t));
        printf("transfer data: { %f, %f, %f, %f }\r\n",
               sensor_data[sensor_type].sensor_data_t.vec.data[0],
               sensor_data[sensor_type].sensor_data_t.vec.data[1],
               sensor_data[sensor_type].sensor_data_t.vec.data[2],
               sensor_data[sensor_type].sensor_data_t.vec.data[3]);
        osMessageQueuePut(SensorDataQHandle, &sensor_data[sensor_type], 0, 0);
        data_bitmask[sensor_type] = 0x00;
      } else {
        printf("error event index for sensor data!\r\n");
        data_bitmask[sensor_type] = 0x00;
      }
    } else if (event_type == SENSOR_CONFIG_RESP_MESSAGE) {
      //broadcast this message to AP through can1 broadcast task
      res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
    } else if (event_type == SENSOR_OTA_MESSAGE) {
      //foward this message to can1 to tell master
      res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
    } else if (event_type == SENSOR_VERSION_MSG) {
      //foward this message to can1 to tell master
      res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
    } else if (event_type == SENSOR_TIME_SYNC_MSG) {
      //foward this message to can1 to tell master
      res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
    } else {
      printf("Can2ReceiveQ unsupported msg \r\n");
    }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Function implementing the Uart2Task thread.
 * @param  argument: Not used
 * @retval None
 */
void StartUart2Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  HAL_StatusTypeDef HAL_RetStaus;
  osStatus_t res;
  sensor_message_event_t sensor_msg_event;
  uint8_t uart_msg_q[UART_TX_CODE_LENGTH];

  for (;;) {
    res = osMessageQueueGet(Uart2MsgQHandle, uart_msg_q, 0, portMAX_DELAY);
    printf("uart_rx_q: %s, size: %u \r\n", uart_msg_q, huart2.RxXferSize);
    if (uart_msg_q[0] == 0xAA && uart_msg_q[1] == 0x55) {
      //printf("uart2 rx size: %u \r\n", huart2.RxXferSize);
      if (sensor_avaiable_check(uart_msg_q[2])) {
        switch (uart_msg_q[3]) {
        case SENSOR_ACTIVATE:
          break;
        case SENSOR_DEACTIVATE:
          break;
        case SENSOR_CONFIG_SELFTEST:
          break;
        default:
          //do nothing
          break;
        }
        sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
        sensor_msg_event.message_event_t.config_event.config_type =
            uart_msg_q[3];
        sensor_msg_event.message_event_t.config_event.sensor_type =
            uart_msg_q[2];
        res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
      }
    } else if (uart_msg_q[0] == 0x5A && uart_msg_q[1] == 0xA5) {
      HAL_RetStaus =
          HAL_UART_Transmit(&huart2, uart_msg_q, UART_TX_CODE_LENGTH, 0xFFFF);
    } else if (uart_msg_q[0] == 0xA5 && uart_msg_q[1] == 0x5A) {
      switch (uart_msg_q[2]) {
      case 0:
        uart_log_print_enable = false;
        platformInitPrams.use_uart_mode = 1;
        //HAL_UART_Receive_DMA(&huart2, uart_msg_code, UART_RX_CODE_LENGTH);
        break;
      default:
        uart_log_print_enable = true;
        platformInitPrams.use_uart_mode = 0;
        break;
      }
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorManagerTask */
/**
 * @brief  Function implementing the SensorManager thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorManagerTask */
void StartSensorManagerTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  sensor_message_event_t sensor_msg_event;
  sensors_event_t sensor_data;
  can_message_event_t can_event_t;
  osStatus_t res;
  uint8_t TxData[8];
  int ret;

  /* Infinite loop */
  for (;;) {
    //printf("waiting for sensor_msg_event \r\n");
    res =
        osMessageQueueGet(SensorMsgQHandle, &sensor_msg_event, 0,
                          portMAX_DELAY);
    if (res) {
      /* osMessageQueueGet Error */
      Error_Handler();
    }
    //printf("smgr event: %d \r\n", sensor_msg_event.message_event_type);

    switch (sensor_msg_event.message_event_type) {
    case SENSOR_DEBUG_CONFIG_MSG:
    case SENSOR_CONFIG_MESSAGE:
      //check sensor type and decide to enable timer
      if (!board_init_complete) {
        //if init is not completed re-add this message to SensorMsgQ
        res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
        break;
      }
      ret = sensor_config(sensor_msg_event.message_event_t.config_event);
      break;
    case SENSOR_TIMER_EVENT:
      if (sensor_msg_event.message_event_t.timer_event.timer_num == 1) {
        //os common timer for data polling
        //printf("timeout timer event! %u\r\n", osKernelGetTickCount());
        ret = sensor_timer_handler();
      } else {
        //special timer for ultrasound gpio operations
        config_event_t sensor_cfg_event = { 0 };
        sensor_cfg_event.config_type = SENSOR_CONFIG_TIMEOUT;
        //printf("timeout config event! %u\r\n", osKernelGetTickCount());
        ret = sensor_config(sensor_cfg_event);
      }
      break;
    case SENSOR_INTERRUPT_EVENT:
      if (!board_init_complete) {
        //if init is not completed re-add this message to SensorMsgQ
        res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
        break;
      }
      //printf("interrupt event %d!\r\n",
      //       sensor_msg_event.message_event_t.interrupt_event.interrupt_num);
      printf("interrupt event! %u\r\n", osKernelGetTickCount());
      ret =
          sensor_irq_handler(sensor_msg_event.message_event_t.interrupt_event);
      break;
    case SENSOR_DATA_MESSAGE:
      //data message may come from can2 on other stm32 chip
      break;
    case SENSOR_ACTIVATE_COMPLETE_EVENT:
      printf("$$$activate done!\r\n");
      osTimerStart(DataTimerHandle, DEFAULE_SAMPLE_RATE_MS);
      break;
    case SENSOR_DEACTIVATE_COMPLETE_EVENT:
      printf("###deactivate done!\r\n");
      osTimerStop(DataTimerHandle);
      break;
    case SENSOR_INIT_COMPLETE_EVENT:
      printf
          ("@@@init done! sizeof sensor_message_event_t: %d, sensors_event_t: %d \r\n",
           sizeof(sensor_message_event_t), sizeof(sensors_event_t));
      //sensor_mainboard_check();
      board_init_complete = true;
      break;
    case SENSOR_CONFIG_RESP_MESSAGE:
      printf("***config resp msg!\r\n");
      if (sensor_msg_event.message_event_t.resp_event.config_type ==
          SENSOR_CONFIG_CALIBRATION) {
        if (sensor_msg_event.message_event_t.resp_event.cfg_data.resp_data[0] ==
            0) {
          //update flash to store calibration data
          update_calibration_data_to_flash(platformInitPrams.board_calidata);
          can_event_t.CanData.TxData[0] = 1;
        } else {
          can_event_t.CanData.TxData[0] = 0;
        }
      } else if (sensor_msg_event.message_event_t.resp_event.config_type ==
                 SENSOR_CONFIG_SELFTEST) {
        if (sensor_msg_event.message_event_t.resp_event.cfg_data.resp_data[0] ==
            0) {
          can_event_t.CanData.TxData[0] = 1;
        } else {
          can_event_t.CanData.TxData[0] = 0;
        }
      }
      //return to can1 when receive cfg resp message stands for calibration success
      can_event_t.CanHeader.TxHeader.StdId =
          (SENSOR_CONFIG_RESP_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
          | (sensor_msg_event.message_event_t.resp_event.
             sensor_type << SENSOR_TYPE_BIT_SHIFT)
          | sensor_msg_event.message_event_t.resp_event.config_type;
      can_event_t.CanHeader.TxHeader.ExtId = 0x0;
      can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
      can_event_t.CanHeader.TxHeader.IDE = CAN_ID_STD;
      can_event_t.CanHeader.TxHeader.DLC = 8;
      can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
      osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
      break;
    case SENSOR_BUS_ERROR_EVENT:
      can_event_t.CanHeader.TxHeader.StdId =
        (SENSOR_DEBUG_CONFIG_MSG << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
        | (sensor_msg_event.message_event_t.resp_event.
           sensor_type << SENSOR_TYPE_BIT_SHIFT)
        | sensor_msg_event.message_event_t.resp_event.config_type;
      can_event_t.CanHeader.TxHeader.ExtId = 0x0;
      can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
      can_event_t.CanHeader.TxHeader.IDE = CAN_ID_STD;
      can_event_t.CanHeader.TxHeader.DLC = 8;
      can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
      osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
      break;
    default:
      break;
    }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorDataProcesser */
/**
 * @brief Function implementing the SensorDataProc thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorDataProcesser */
void StartSensorDataProcesser(void *argument)
{
  /* USER CODE BEGIN StartSensorDataProcesser */
  sensors_event_t sensor_data;
  osStatus_t res;
  float data[4];
  can_message_event_t can_event_t;
  uint8_t data_size;
  int ret;
  // we use sensor data proc to update all sensor results to can1 broadcast task
  // sensor data proc is also used for fusion algo calc
  /* Infinite loop */
  for (;;) {
    //printf("waiting for sensor data \r\n");
    res = osMessageQueueGet(SensorDataQHandle, &sensor_data, 0, portMAX_DELAY);
    if (!board_init_complete) {
      //if init is not completed re-add this message to SensorMsgQ
      res = osMessageQueuePut(SensorDataQHandle, &sensor_data, 0, 0);
      continue;
    }
    ret = sensor_data_handler(&sensor_data);
    if (((CAN1_ENABLED_SENSORBIT >> sensor_data.sensor_type) & 1)
        || ((CAN1_ENABLED_SENSORBIT >> SENSOR_TYPE_MAX) & 1)
        || (sensor_data.accuracy == 0xFF)
        || ((CAN1_GET_CALI_SENSORBIT >> sensor_data.sensor_type) & 1)) {
      data[0] = sensor_data.sensor_data_t.vec.data[0];
      data[1] = sensor_data.sensor_data_t.vec.data[1];
      data[2] = sensor_data.sensor_data_t.vec.data[2];
      data[3] = sensor_data.sensor_data_t.vec.data[3];
      //printf("DataQ %d { %f, %f, %f, %f} \r\n", sensor_data.sensor_type, data[0], data[1], data[2], data[3]);
      //printf("u8: %d, float: %d, uint64_t: %d \r\n", sizeof(uint8_t), sizeof(float), sizeof(uint64_t));
      data_size = sensor_get_index_length(sensor_data.sensor_type);
      if (data_size != 0xFF) {
        for (int i = 0; i < data_size; i++) {
          can_event_t.CanHeader.TxHeader.StdId =
              (SENSOR_DATA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
              | (sensor_data.sensor_type << SENSOR_TYPE_BIT_SHIFT)
              | data_size;
          can_event_t.CanHeader.TxHeader.ExtId = 0x0;
          can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
          can_event_t.CanHeader.TxHeader.IDE = CAN_ID_STD;
          can_event_t.CanHeader.TxHeader.DLC = 8;
          can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
          can_event_t.CanData.TxData[0] = i;
          memcpy(&can_event_t.CanData.TxData[1], &data[i], sizeof(float));
#if USE_P1
          osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
#else
          osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
#endif
        }
        //add timestamp message to list-tail: Todo, check if this kind of method can be used
        can_event_t.CanHeader.TxHeader.StdId =
            (SENSOR_TIMESTAMP_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
            | (sensor_data.sensor_type << SENSOR_TYPE_BIT_SHIFT)
            | data_size;
        can_event_t.CanHeader.TxHeader.ExtId = 0x0;
        can_event_t.CanHeader.TxHeader.RTR = CAN_RTR_DATA;
        can_event_t.CanHeader.TxHeader.IDE = CAN_ID_STD;
        can_event_t.CanHeader.TxHeader.DLC = 8;
        can_event_t.CanHeader.TxHeader.TransmitGlobalTime = DISABLE;
        memcpy(&can_event_t.CanData.TxData[0], &sensor_data.timestamp, sizeof(uint64_t));       //add timestamp here to userd
#if USE_P1
        osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);
#else
        osMessageQueuePut(Can2BroadcastQHandle, &can_event_t, 0, 0);
#endif
      }
      if ((CAN1_GET_CALI_SENSORBIT >> sensor_data.sensor_type) & 1) {
        CAN1_GET_CALI_SENSORBIT &= ~(1 << sensor_data.sensor_type);
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartSensorDataProcesser */
}

/* USER CODE BEGIN Header_StartInitTask */
/**
 * @brief  Function implementing the InitTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInitTask */
void StartInitTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  sensor_message_event_t sensor_msg_event;
  can_message_event_t can_event_t;
  osStatus_t res;

  init_registered_sensors();

  //update related sensor calibration data
  sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
  sensor_msg_event.message_event_t.config_event.config_type =
      SENSOR_CONFIG_BIAS;
  sensor_msg_event.message_event_t.config_event.sensor_type =
      SENSOR_TYPE_ACCELEROMETER;
  res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);

  sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
  sensor_msg_event.message_event_t.config_event.config_type =
      SENSOR_CONFIG_BIAS;
  sensor_msg_event.message_event_t.config_event.sensor_type =
      SENSOR_TYPE_GYROSCOPE;
  res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);

  sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
  sensor_msg_event.message_event_t.config_event.config_type =
      SENSOR_CONFIG_BIAS;
  sensor_msg_event.message_event_t.config_event.sensor_type = SENSOR_TYPE_LIGHT;
  res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);

  sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
  sensor_msg_event.message_event_t.config_event.config_type =
      SENSOR_CONFIG_BIAS;
  sensor_msg_event.message_event_t.config_event.sensor_type =
      SENSOR_TYPE_PROXIMITY_BOT;
  res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);

  can_event_t.CanHeader.TxHeader.StdId =
      (SENSOR_TIME_SYNC_MSG << SENSOR_EVENT_MESSAGE_BIT_SHIFT)
      | (boardID << SENSOR_TYPE_BIT_SHIFT)
      | 0xf;
  uint64_t current_time = sensor_get_timestamp();
  memcpy(&can_event_t.CanData.TxData[0], &current_time, sizeof(uint64_t));
  res = osMessageQueuePut(Can1BroadcastQHandle, &can_event_t, 0, 0);

  for (;;) {
    osThreadSuspend(InitTaskHandle);
  }
  /* USER CODE END 5 */
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  sensor_message_event_t sensor_msg_event;
  sensor_msg_event.message_event_type = SENSOR_INTERRUPT_EVENT;
  sensor_msg_event.message_event_t.interrupt_event.interrupt_num = gpio_pin;
  osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
}

/* USER CODE END 4 */

/* DataTimerCallback function */
void DataTimerCallback(void *argument)
{
  /* USER CODE BEGIN DataTimerCallback */
  sensor_message_event_t sensor_msg_event;
  sensor_msg_event.message_event_type = SENSOR_TIMER_EVENT;
  sensor_msg_event.message_event_t.timer_event.timer_num = 1;
  osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
  /* USER CODE END DataTimerCallback */
}

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
  if (htim->Instance == TIM3) {
    //clear update flag
    sensor_message_event_t sensor_msg_event;
    osStatus_t res;
    sensor_msg_event.message_event_type = SENSOR_TIMER_EVENT;
    sensor_msg_event.message_event_t.timer_event.timer_num = 3;
    res = osMessageQueuePut(SensorMsgQHandle, &sensor_msg_event, 0, 0);
  }
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

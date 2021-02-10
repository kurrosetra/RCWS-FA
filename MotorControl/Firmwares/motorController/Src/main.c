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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rwsCanID.h"
#include "stm_hal_serial.h"
#include "Ingenia_CanServoDriver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	LIM_SWITCH_AZIMUTH_ZERO = 0,
	LIM_SWITCH_ELEVATION_UP,
	LIM_SWITCH_ELEVATION_DOWN
} LimitSwitchBit_e;

typedef union
{
	float f;
	uint32_t u32;
	int32_t i32;
	uint8_t b[4];
} Union_u;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG					1

#if DEBUG==1
#	define DEBUG_MTR_POSVELO	0
#	define DEBUG_INGENIA_DRIVER	0
#	if DEBUG_INGENIA_DRIVER==1
#		define MTR_ID			0x20
#	endif	//if DEBUG_INGENIA_DRIVER==1
#endif	//if DEBUG==1

#define MTR_AZ_ENABLE			1
#define MTR_EL_ENABLE			1
#define MTR_COCK_ENABLE			0

#define MTR_AZ_ID				0x20
#define MTR_EL_ID				0x21
#define MTR_COCK_ID				0x22

#ifdef RING_BUFFER_SIZE
#define UART_BUFSIZE			RING_BUFFER_SIZE
#else
#define UART_BUFSIZE			256
#endif

#ifndef CAN_DATA_MAX
#define CAN_DATA_MAX		8
#endif

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define MOTOR_MAX_SPEED			116500UL	//1750RPM
#define MOTOR_MAX_ACCEL			2500000UL

#define TPDO4_EVENT_TIMER_100HZ			10
#define TPDO4_EVENT_TIMER_125HZ			8
#define TPDO4_EVENT_TIMER_200HZ			5
#define TPDO4_EVENT_TIMER				TPDO4_EVENT_TIMER_100HZ

#define MOTOR_UPDATE_TIMEOUT			10	//100HZ

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* TODO Global Variables*/
uint16_t bufLen = 0;
char buf[UART_BUFSIZE];

#if DEBUG
Ring_Buffer_t tx6Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx6Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx6Buffer, &tx6Buffer, &huart6 };
#endif	//if DEBUG

#if DEBUG==1
char vt100_home[10];
#define DEBUG_LINE_MAX		25
char vt100_lineX[DEBUG_LINE_MAX][16];
#endif	//if DEBUG==1

CAN_TxHeaderTypeDef can2TxHeader;
uint32_t can2TxMailBox;
uint8_t can2TxBuffer[8];
CAN_RxHeaderTypeDef can2RxHeader;
uint8_t can2RxBuffer[8];

CAN_TxHeaderTypeDef *busTxHeader;
uint32_t *busTxMailBox;
uint8_t *busTxBuffer;
CAN_RxHeaderTypeDef *busRxHeader;
uint8_t *busRxBuffer;

#if DEBUG_INGENIA_DRIVER==1
Servo_t mtr;
Servo_t dummy;
#endif	//if DEBUG_INGENIA_DRIVER==1

#if MTR_AZ_ENABLE==1
Servo_t mtrAzi;
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
Servo_t mtrEle;
#endif	//if MTR_EL_ENABLE==1

#if MTR_COCK_ENABLE==1
servo_t mtrCock;
#endif	//if MTR_COCK_ENABLE==1

volatile uint8_t limitSwitch = 0;

uint32_t motorCommandTimeout = 0;
uint32_t stabCommandTimeout = 0;
uint32_t buttonCommandTimeout = 0;
int32_t panelCommand[2];
int32_t stabCommand[2];
volatile bool cockStart = false;

typedef enum
{
	SERVO_PAN,
	SERVO_TILT
} EServoName;

typedef enum
{
	DIR_UP_RIGHT = false,
	DIR_DOWN_LEFT = true
} EServoDirection;

typedef struct
{
	EServoName name;
	volatile bool enable;
	Servo_t *servo;
	uint32_t prevSpeed;
	bool direction;
	float relPosition;
	float velo;
} TServoState;
TServoState pan;
TServoState tilt;

#define CAN_BUFSIZE							8
const uint32_t COMMAND_RECEIVE_TIMEOUT = 500;

typedef struct
{
	uint32_t id;
	volatile bool state;
	uint8_t data[8];
	uint8_t size;
	uint8_t online;
} TCanRecvBuffer;

typedef struct
{
	uint32_t id;
	uint8_t data[8];
	uint8_t size;
} TCanSendBuffer;

TCanRecvBuffer canRecvPanel = { CAN_ID_RWS_PNL_MTR, false, { 0 }, 7, 0 };
TCanRecvBuffer canRecvButton = { CAN_ID_RWS_BUTTON, false, { 0 }, 3, 0 };
TCanRecvBuffer canRecvStabMode = { CAN_ID_RWS_PNL_STAB_MODE, false, { 0 }, 1, 0 };
TCanRecvBuffer canRecvStabCommand = { CAN_ID_RWS_STAB_PNL, false, { 0 }, 8, 0 };

TCanSendBuffer canSendMotor = { CAN_ID_RWS_MOTOR, { 0 }, 6 };
TCanSendBuffer canSendMotorAngle = { CAN_ID_RWS_MTR_STAB_ANGLE, { 0 }, 8 };
TCanSendBuffer canSendMotorSpeed = { CAN_ID_RWS_MTR_STAB_SPD, { 0 }, 8 };

#if DEBUG_MTR_POSVELO==1
volatile uint32_t countTPDO4 = 0;
#endif	//if DEBUG_MTR_POSVELO==1
volatile uint16_t countAngleVelo[2] = { 0, 0 };
volatile uint32_t cStabMode = 0;
volatile uint32_t cStabModule = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
#if DEBUG_INGENIA_DRIVER==1
static void displayCanBuffer(CAN_Data_t *data);
static void debugMotor();
#endif	//if DEBUG_INGENIA_DRIVER==1

static HAL_StatusTypeDef motorInit();
static void motorStop(TServoState *motor);
static void motorSpeed(TServoState *motor, int32_t spd);
static void motorHandler();
static void motorParamEventTime(Servo_t *servo, uint16_t rate);

static HAL_StatusTypeDef busInit();
static void busCommandParsing();
static void stabCommandParsing();
static void busHandler(CAN_HandleTypeDef *bus);
static void busSendingHandler();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if DEBUG==1
	sprintf(vt100_home, "\x1b[2J\x1b[H");
	unsigned char debugLine = 0;
	for ( debugLine = 0; debugLine < DEBUG_LINE_MAX; debugLine++ )
		sprintf(vt100_lineX[debugLine], "\x1b[%d;0H\x1b[2K", debugLine);
#endif	//if DEBUG==1
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_IWDG_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_IWDG_Refresh(&hiwdg);

#if DEBUG==1
	serial_init(&debug);

	bufLen = sprintf(buf, "%sIngenia CAN library\r\n", vt100_home);
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

#if DEBUG==1
	bufLen = sprintf(buf, "motor init...\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

	if (motorInit() != HAL_OK) {
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		while (1)
			;
	}
#if DEBUG==1
	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);

	bufLen = sprintf(buf, "bus init...");
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

	if (busInit() != HAL_OK) {
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		while (1)
			;
	}

#if DEBUG==1
	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

#if DEBUG_INGENIA_DRIVER==1
	debugMotor();
#endif	//if DEBUG_INGENIA_DRIVER==1

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if DEBUG_MTR_POSVELO==1
	uint32_t countTpdoTimer = 0;
	Union_u p, v;
#endif	//if DEBUG_INGENIA_DRIVER==1

	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/

		if (HAL_GetTick() >= buttonCommandTimeout) {
			HAL_GPIO_WritePin(TRIGGER_ENABLE_GPIO_Port, TRIGGER_ENABLE_Pin, GPIO_PIN_RESET);
			canRecvButton.online = false;
		}

		if (canRecvButton.state) {
			canRecvButton.state = false;
			buttonCommandTimeout = HAL_GetTick() + COMMAND_RECEIVE_TIMEOUT;
			canRecvButton.online = true;

			HAL_GPIO_WritePin(TRIGGER_ENABLE_GPIO_Port, TRIGGER_ENABLE_Pin,
					bitRead(canRecvButton.data[0], 0));

			cockStart = bitRead(canRecvButton.data[0], 1);
		}

		if (bitRead(canRecvStabMode.data[0], 0)) {
			if (HAL_GetTick() >= stabCommandTimeout && canRecvStabCommand.online) {
				canRecvStabCommand.online = false;
				pan.enable = false;
				tilt.enable = false;
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
			}
		}

		if (HAL_GetTick() >= motorCommandTimeout && canRecvPanel.online) {
			canRecvPanel.online = false;
			if (!bitRead(canRecvStabMode.data[0], 0)) {
				pan.enable = false;
				tilt.enable = false;
			}
			HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
		}

		if (canRecvPanel.state) {
			canRecvPanel.state = false;
			motorCommandTimeout = HAL_GetTick() + COMMAND_RECEIVE_TIMEOUT;
			canRecvPanel.online = true;

			busCommandParsing();
			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		}

		if (bitRead(canRecvStabMode.data[0], 0)) {
			if (canRecvStabCommand.state) {
				canRecvStabCommand.state = false;
				stabCommandTimeout = HAL_GetTick() + COMMAND_RECEIVE_TIMEOUT;
				canRecvStabCommand.online = true;

				stabCommandParsing();
				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
			}
		}

		motorHandler();
		busSendingHandler();

#if DEBUG_MTR_POSVELO==1
		uint8_t _dataRpdo[8];
		if (HAL_GetTick() >= countTpdoTimer) {
			countTpdoTimer = HAL_GetTick() + 1000;

			bufLen = sprintf(buf, "%stpdo4= %d/s", vt100_lineX[11], countTPDO4);
			countTPDO4 = 0;
			serial_write_str(&debug, buf, bufLen);
		}

		if (serial_available(&debug)) {
			char c = serial_read(&debug);

			if (c == 'T') {
				bufLen = sprintf(buf, "%smotor enable\r\n", vt100_lineX[12]);
				serial_write_str(&debug, buf, bufLen);
				_dataRpdo[0] = 0xF;
				_dataRpdo[1] = 0;
				Ingenia_write_rpdo(tilt.servo, COB_RPDO1, _dataRpdo, 2);
			}
			if (c == 't') {
				bufLen = sprintf(buf, "%smotor disable\r\n", vt100_lineX[12]);
				serial_write_str(&debug, buf, bufLen);
				_dataRpdo[0] = 0x7;
				_dataRpdo[1] = 0;
				Ingenia_write_rpdo(tilt.servo, COB_RPDO1, _dataRpdo, 2);
			}
			else if (c == 's') {
				memset(_dataRpdo, 0, 8);
				v.u32 = 0x500;
				p.i32 = 4000;
				for ( int i = 0; i < 4; i++ ) {
					_dataRpdo[i] = v.b[i];
					_dataRpdo[i + 4] = p.b[i];
				}
				Ingenia_write_rpdo(tilt.servo, COB_RPDO2, _dataRpdo, 8);
				bufLen = sprintf(buf, "%sSet new V:0x500 P:4000c", vt100_lineX[12]);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '>') {
				p.i32 = tilt.servo->posActual + 4000;
				Ingenia_setTargetPositionVelocity(tilt.servo, p.i32, 0x500, 1, 0, 0);
				bufLen = sprintf(buf, "%sto %dc w/ 500c/s", vt100_lineX[12], p.i32);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '<') {
				p.i32 = tilt.servo->posActual - 4000;
				Ingenia_setTargetPositionVelocity(tilt.servo, p.i32, 0x500, 1, 0, 0);
				bufLen = sprintf(buf, "%sto %dc w/ 500c/s", vt100_lineX[12], p.i32);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '0') {
				Ingenia_setTargetPositionVelocity(tilt.servo, 0, 0, 1, 0, 1);
				bufLen = sprintf(buf, "%sMotor Stop!", vt100_lineX[12], p.i32);
				serial_write_str(&debug, buf, bufLen);
			}

		}
#endif	//if DEBUG_MTR_POSVELO==1

		/* TODO END LOOP*/
	}	//while(1){
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

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
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_ENABLE_GPIO_Port, TRIGGER_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC8 PC9 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA5 PA6
                           PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM_AZ_Pin LIM_EL_UP_Pin LIM_EL_DOWN_Pin */
  GPIO_InitStruct.Pin = LIM_AZ_Pin|LIM_EL_UP_Pin|LIM_EL_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB14 PB15 PB3 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIGGER_ENABLE_Pin */
  GPIO_InitStruct.Pin = TRIGGER_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TRIGGER_ENABLE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Begin of Functions Declaration*/
#if DEBUG==1
void USART6_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}
#endif	//if DEBUG==1

void EXTI1_IRQHandler(void)
{
	/* limit azimuth zero active */
	SET_BIT(limitSwitch, (1 << LIM_SWITCH_AZIMUTH_ZERO));
}

void EXTI2_IRQHandler(void)
{
	/* limit elevation UP active */
	SET_BIT(limitSwitch, (1 << LIM_SWITCH_ELEVATION_UP));

	/* check elevation's direction */
	/* stop when direction is UP */
}

void EXTI3_IRQHandler(void)
{
	/* limit elevation DOWN active */
	SET_BIT(limitSwitch, (1 << LIM_SWITCH_ELEVATION_DOWN));

	/* check elevation's direction */
	/* stop when direction is UP */
}

static void busSendingHandler()
{
	static uint32_t _stateTimer = 1000;
	static uint32_t _angleTimer = 1000;
	static uint32_t _angularSpeedTimer = 1000;
	static uint32_t sumAngleVeloTimer = 0;

	if (HAL_GetTick() >= sumAngleVeloTimer) {
		sumAngleVeloTimer = HAL_GetTick() + 1000;

		canSendMotor.data[2] = countAngleVelo[0] & 0xFF;
		canSendMotor.data[3] = (countAngleVelo[0] >> 8) & 0xFF;
		canSendMotor.data[4] = countAngleVelo[1] & 0xFF;
		canSendMotor.data[5] = (countAngleVelo[1] >> 8) & 0xFF;

		bufLen = sprintf(buf, "%sSmode:Sval=%d:%d\tS(p:t)=%d:%d\tAV(p:t)=%d:%d", vt100_lineX[15],
				cStabMode, cStabModule, stabCommand[0], stabCommand[1], countAngleVelo[0],
				countAngleVelo[1]);
		serial_write_str(&debug, buf, bufLen);

		countAngleVelo[0] = countAngleVelo[1] = 0;
		cStabMode = cStabModule = 0;
	}

	if (HAL_GetTick() >= _stateTimer) {
		_stateTimer = HAL_GetTick() + 100;

		bitWrite(canSendMotor.data[0], 0, pan.enable);
		bitWrite(canSendMotor.data[0], 1, tilt.enable);
		bitWrite(canSendMotor.data[1], 0,
				HAL_GPIO_ReadPin(TRIGGER_ENABLE_GPIO_Port, TRIGGER_ENABLE_Pin));
		bitWrite(canSendMotor.data[1], 1, cockStart);

		busTxHeader->StdId = canSendMotor.id;
		memcpy(busTxBuffer, canSendMotor.data, canSendMotor.size);
		busTxHeader->DLC = canSendMotor.size;
		if (HAL_CAN_AddTxMessage(&hcan2, busTxHeader, busTxBuffer, busTxMailBox) != HAL_OK)
			_stateTimer = HAL_GetTick() + 10;

#if DEBUG==1
		Union_u u, w;

		u.f = C_TO_DEG_AZ(pan.servo->posActual);
		w.f = C_TO_DEG_AZ(pan.servo->veloActual);
		bufLen = sprintf(buf, "%sPAN: P= %d > %.3f\tV= %d > %.3f", vt100_lineX[9],
				pan.servo->posActual, u.f, pan.servo->veloActual, w.f);
		serial_write_str(&debug, buf, bufLen);

		u.f = C_TO_DEG_EL(tilt.servo->posActual);
		w.f = C_TO_DEG_EL(tilt.servo->veloActual);
		bufLen = sprintf(buf, "%sTILT: P= %d > %.3f\tV= %d > %.3f", vt100_lineX[10],
				tilt.servo->posActual, u.f, tilt.servo->veloActual, w.f);
		serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

	}

	if (HAL_GetTick() >= _angleTimer) {
		_angleTimer = HAL_GetTick() + TPDO4_EVENT_TIMER;

		Union_u pAz, pEl;
#if MTR_AZ_ENABLE==1
		pAz.f = pan.relPosition;
#else
		pAz.f = 0.0f;
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
		pEl.f = tilt.relPosition;
#else
		pEl.f = 0.0f;
#endif	//if MTR_EL_ENABLE==1

		for ( int i = 0; i < 4; i++ ) {
			canSendMotorAngle.data[i] = pAz.b[i];
			canSendMotorAngle.data[i + 4] = pEl.b[i];
		}

		busTxHeader->StdId = canSendMotorAngle.id;
		memcpy(busTxBuffer, canSendMotorAngle.data, canSendMotorAngle.size);
		busTxHeader->DLC = canSendMotorAngle.size;
		if (HAL_CAN_AddTxMessage(&hcan2, busTxHeader, busTxBuffer, busTxMailBox) != HAL_OK)
			_angleTimer = HAL_GetTick() + 1;
	}

	if (HAL_GetTick() >= _angularSpeedTimer) {
		_angularSpeedTimer = HAL_GetTick() + TPDO4_EVENT_TIMER;

		Union_u vAz, vEl;
#if MTR_AZ_ENABLE==1
		vAz.f = pan.velo;
#else
		vAz.f = 0.0f;
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
		vEl.f = tilt.velo;
#else
		vEl.f = 0.0f;
#endif	//if MTR_EL_ENABLE==1

		for ( int i = 0; i < 4; i++ ) {
			canSendMotorSpeed.data[i] = vAz.b[i];
			canSendMotorSpeed.data[i + 4] = vEl.b[i];
		}

		busTxHeader->StdId = canSendMotorSpeed.id;
		memcpy(busTxBuffer, canSendMotorSpeed.data, canSendMotorSpeed.size);
		busTxHeader->DLC = canSendMotorSpeed.size;
		if (HAL_CAN_AddTxMessage(&hcan2, busTxHeader, busTxBuffer, busTxMailBox) != HAL_OK)
			_angleTimer = HAL_GetTick() + 1;
	}
}

static void stabCommandParsing()
{
	Union_u p, t;
	uint8_t dataPT[8];

	pan.enable = bitRead(canRecvPanel.data[0], 0);
	tilt.enable = bitRead(canRecvPanel.data[0], 1);

	memcpy(dataPT, canRecvStabCommand.data, canRecvStabCommand.size);

	/* find angular velocity request from stabilize module */
	for ( int i = 0; i < 4; i++ ) {
		p.b[i] = dataPT[i];
		t.b[i] = dataPT[i + 4];
	}

	/* convert from deg/s to c/s */
	stabCommand[0] = (int32_t) DEG_TO_C_AZ(p.f);
	if (abs(stabCommand[0]) > MOTOR_MAX_SPEED) {
		if (stabCommand[0] >= 0)
			stabCommand[0] = MOTOR_MAX_SPEED;
		else
			stabCommand[0] = 0 - MOTOR_MAX_SPEED;
	}
	stabCommand[1] = (int32_t) DEG_TO_C_EL(t.f);
	if (abs(stabCommand[1]) > MOTOR_MAX_SPEED) {
		if (stabCommand[1] >= 0)
			stabCommand[1] = MOTOR_MAX_SPEED;
		else
			stabCommand[1] = 0 - MOTOR_MAX_SPEED;
	}
}

static void busCommandParsing()
{
	int32_t spd = 0;

	/*
	 * COMMAND BYTE:
	 * BYTE 0:	Trigger & cocking command
	 * BYTE 1:	Reserve
	 * BYTE 2
	 * 	bit 7: motor pan enable
	 * 	bit 6: pan direction
	 * 	bit[4..5]: reserve
	 * 	bit[0..3]: MSB motor pan speed
	 * BYTE [3..4]: LSB motor pan speed
	 * BYTE 5
	 * 	bit 7: motor tilt enable
	 * 	bit 6: tilt direction
	 * 	bit[4..5]: reserve
	 * 	bit[0..3]: MSB motor tilt speed
	 * BYTE [6..7]: LSB motor tilt speed
	 */

	/* PAN motor */
	pan.enable = bitRead(canRecvPanel.data[0], 0);

	if (pan.enable) {
		spd = ((int32_t) canRecvPanel.data[3] << 16) | ((int32_t) canRecvPanel.data[2] << 8)
				| canRecvPanel.data[1];

		if (bitRead(canRecvPanel.data[0], 2))
			spd = 0 - spd;

		panelCommand[0] = spd;
	}
	else
		panelCommand[0] = 0;

	/* TILT motor */
	tilt.enable = bitRead(canRecvPanel.data[0], 1);

	if (tilt.enable) {
		spd = ((int32_t) canRecvPanel.data[6] << 16) | ((int32_t) canRecvPanel.data[5] << 8)
				| canRecvPanel.data[4];

		if (bitRead(canRecvPanel.data[0], 3))
			spd = 0 - spd;

		panelCommand[1] = spd;
	}
	else
		panelCommand[1] = 0;

#if DEBUG==1
	bufLen = sprintf(buf, "%sStab:%d\tP:%d %d %d\tT:%d %d %d", vt100_lineX[7],
			bitRead(canRecvStabMode.data[0], 0), pan.enable, (int) panelCommand[0],
			(int) stabCommand[0], tilt.enable, (int) panelCommand[1], (int) stabCommand[1]);
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

}

static void motorStop(TServoState *motor)
{
	Ingenia_setTargetPositionVelocity(motor->servo, 0, 0, 1, 0, 1);
}

static void motorSpeed(TServoState *motor, int32_t spd)
{
	int _pos = motor->servo->posActual;
	uint32_t _absSpd = abs(spd);

	if (spd == 0)
		motorStop(motor);
	else {
		if (spd < 0)
			_pos -= 50000;
		else
			_pos += 50000;

		Ingenia_setTargetPositionVelocity(motor->servo, _pos, _absSpd, 1, 0, 0);
	}

	motor->prevSpeed = spd;
}

static void motorHandler()
{
	static uint32_t motorUpdateTimer = 0;
	static uint8_t motorUpdateCounter = 0;
	int32_t *panCommand;
	int32_t *tiltCommand;

	if (bitRead(canRecvStabMode.data[0], 0)) {
		panCommand = &stabCommand[0];
		tiltCommand = &stabCommand[1];
	}
	else {
		panCommand = &panelCommand[0];
		tiltCommand = &panelCommand[1];
	}

#if MTR_AZ_ENABLE==1
	static _Bool panEnable = 1;

	if (panEnable != pan.enable) {
		if (pan.enable) {
			/* enable motor */
			Ingenia_enableMotor(pan.servo);
#if DEBUG==1
			bufLen = sprintf(buf, "%span enable!", vt100_lineX[12]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==0
		}
		else {
			/* disable motor */
			Ingenia_disableMotor(pan.servo);
#if DEBUG==1
			bufLen = sprintf(buf, "%span disable!", vt100_lineX[12]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==0
		}
		panEnable = pan.enable;
	}
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
	static _Bool tiltEnable = 1;

	if (tiltEnable != tilt.enable) {
		if (tilt.enable) {
			/* enable motor */
			Ingenia_enableMotor(tilt.servo);
#if DEBUG==1
#if MTR_AZ_ENABLE==1
			bufLen = sprintf(buf, "\ttilt enable!");
#else
			bufLen = sprintf(buf, "%stilt enable!", vt100_lineX[12]);
#endif	//if MTR_AZ_ENABLE==1
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==0
		}
		else {
			/* disable motor */
			Ingenia_disableMotor(tilt.servo);
#if DEBUG==1
#if MTR_AZ_ENABLE==1
			bufLen = sprintf(buf, "\ttilt disable!");
#else
			bufLen = sprintf(buf, "%stilt disable!", vt100_lineX[12]);
#endif	//if MTR_AZ_ENABLE==1
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==0
		}
		tiltEnable = tilt.enable;
	}
#endif	//if MTR_EL_ENABLE==1

	if (HAL_GetTick() >= motorUpdateTimer) {
		motorUpdateTimer = HAL_GetTick() + (MOTOR_UPDATE_TIMEOUT / 2);

#if MTR_AZ_ENABLE==1
		if (panEnable && !bitRead(motorUpdateCounter, 0))
			motorSpeed(&pan, *panCommand);
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
		if (tiltEnable && bitRead(motorUpdateCounter, 0))
			motorSpeed(&tilt, *tiltCommand);
#endif	//if MTR_EL_ENABLE==1

		motorUpdateCounter++;
	}
}

static void motorParamEventTime(Servo_t *servo, uint16_t rate)
{
	/* set TPDO4 event timer */
	Ingenia_write_sdo_u16(servo, 0x1803, 5, rate);
//	/* set TPDO4 inhibit time (max transmission rate to 200Hz */
//	Ingenia_write_sdo_u16(servo, 0x1803, 3, 20);
}

static void motorParamInit(Servo_t *servo)
{

	Ingenia_write_nmt(servo, NMT_START_REMOTE_NODE);

	/* set mode to profile position */
	Ingenia_setModeOfOperation(servo, DRIVE_MODE_PROFILE_POSITION);

	/* set max motor speed */
	Ingenia_write_sdo_u32(servo, 0x6080, 0, MOTOR_MAX_SPEED);
	/* set max velocity profile */
	Ingenia_write_sdo_u32(servo, 0x607F, 0, MOTOR_MAX_SPEED);
	/* set max profile acceleration */
	Ingenia_write_sdo_u32(servo, 0x6083, 0, MOTOR_MAX_ACCEL);
	/* set max profile de-acceleration */
	Ingenia_write_sdo_u32(servo, 0x6084, 0, MOTOR_MAX_ACCEL);
	/* set max profile quick stop de-acceleration */
	Ingenia_write_sdo_u32(servo, 0x6085, 0, 2 * MOTOR_MAX_ACCEL);
}

static HAL_StatusTypeDef motorInit()
{
	HAL_StatusTypeDef ret = HAL_OK;
	Ingenia_begin(&hcan1);

#if DEBUG_INGENIA_DRIVER==1
	Ingenia_init(&mtr, 0x21);
	Ingenia_init(&dummy, 0x22);
	Ingenia_node_remove(&dummy);

	debugMotor();
#endif	//if DEBUG_INGENIA_DRIVER==1

#if MTR_AZ_ENABLE==1
	pan.name = SERVO_PAN;
	pan.enable = false;
	pan.direction = DIR_UP_RIGHT;
	pan.servo = &mtrAzi;
	pan.prevSpeed = 1;

	if (Ingenia_init(pan.servo, MTR_AZ_ID) != HAL_OK)
		return HAL_ERROR;

#if DEBUG==1
	bufLen = sprintf(buf, "enabling pan motor...");
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

	motorParamInit(pan.servo);
	Ingenia_disableMotor(pan.servo);

#if DEBUG==1
	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
	tilt.name = SERVO_TILT;
	tilt.enable = false;
	tilt.direction = DIR_UP_RIGHT;
	tilt.servo = &mtrEle;
	tilt.prevSpeed = 1;

	if (Ingenia_init(tilt.servo, MTR_EL_ID) != HAL_OK)
		return HAL_ERROR;

#if DEBUG==1
	bufLen = sprintf(buf, "enabling tilt motor...");
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

	motorParamInit(tilt.servo);
	Ingenia_disableMotor(tilt.servo);

#if DEBUG==1
	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1
#endif	//if MTR_EL_ENABLE==1

#if MTR_AZ_ENABLE==1
	motorParamEventTime(pan.servo, TPDO4_EVENT_TIMER);
#endif	//if MTR_AZ_ENABLE==1
#if MTR_EL_ENABLE==1
	motorParamEventTime(tilt.servo, TPDO4_EVENT_TIMER);
#endif	//if MTR_EL_ENABLE==1

#if MTR_COCK_ENABLE==1
	Ingenia_init(cock.servo, MTR_COCK_ID);
#endif	//if MTR_COCK_ENABLE==1

	return ret;
}

static HAL_StatusTypeDef busInit()
{
	busTxHeader = &can2TxHeader;
	busTxMailBox = &can2TxMailBox;
	busTxBuffer = can2TxBuffer;
	busRxHeader = &can2RxHeader;
	busRxBuffer = can2RxBuffer;

	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/

	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_PNL_MTR << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_PNL_MTR << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}

	sFilterConfig.FilterBank = 15;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_BUTTON << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_BUTTON << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}

	sFilterConfig.FilterBank = 16;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_PNL_STAB_MODE << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_PNL_STAB_MODE << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}

	sFilterConfig.FilterBank = 17;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_STAB_PNL << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_STAB_PNL << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		/* Start Error */
		return HAL_ERROR;
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
		/* Notification Error */
		return HAL_ERROR;
	}

	/*##-5- Configure Transmission process #####################################*/
	busTxHeader->StdId = 0x1;
	busTxHeader->RTR = CAN_RTR_DATA;
	busTxHeader->IDE = CAN_ID_STD;
	busTxHeader->DLC = CAN_DATA_MAX;
	busTxHeader->TransmitGlobalTime = DISABLE;

	return HAL_OK;
}

static void busHandler(CAN_HandleTypeDef *bus)
{
	uint32_t _id = 0;

	if (HAL_CAN_GetRxMessage(bus, CAN_RX_FIFO1, busRxHeader, busRxBuffer) == HAL_OK) {
		_id = busRxHeader->StdId;
		if (_id == canRecvPanel.id) {
			canRecvPanel.state = true;
			memcpy(canRecvPanel.data, busRxBuffer, canRecvPanel.size);
		}
		else if (_id == canRecvButton.id) {
			canRecvButton.state = true;
			memcpy(canRecvButton.data, busRxBuffer, canRecvButton.size);
		}
		else if (_id == canRecvStabMode.id) {
			canRecvStabMode.state = true;
			memcpy(canRecvStabMode.data, busRxBuffer, canRecvStabMode.size);
			cStabMode++;
		}
		else if (_id == canRecvStabCommand.id) {
			canRecvStabCommand.state = true;
			memcpy(canRecvStabCommand.data, busRxBuffer, canRecvStabCommand.size);
			cStabModule++;
		}
	}
}

/**
 * @brief  Rx Fifo 1 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	busHandler(hcan);
}

#if DEBUG_INGENIA_DRIVER==0
void Ingenia_tpdo_callback(CAN_Buffer_t *buffer)
{
	CAN_Data_t data;
	uint32_t idNode = 0;
	int _pos = 0, _velo = 0;

	if (can_buffer_available(buffer)) {
		can_buffer_read(buffer, &data);
#if DEBUG_MTR_POSVELO==1
		countTPDO4++;
#endif	//if DEBUG_MTR_POSVELO==1

		if ((data.canRxHeader.StdId & COB_TPDO4) == COB_TPDO4) {
			idNode = data.canRxHeader.StdId - COB_TPDO4;
			for ( int i = 0; i < 4; i++ ) {
				_pos |= (int) data.rxData[i] << (8 * i);
				_velo |= (int) data.rxData[i + 4] << (8 * i);
			}
#if MTR_AZ_ENABLE==1
			if (idNode == mtrAzi._u8Node) {
				pan.servo->posActual = _pos;
				pan.servo->veloActual = _velo;

				int32_t _tem = abs(_pos) % C_AZ_FULLSWING;
				if (_pos < 0)
					_tem = C_AZ_FULLSWING - _tem;
				pan.relPosition = C_TO_DEG_AZ(_tem);
				pan.velo = C_TO_DEG_AZ(_velo);

				countAngleVelo[0]++;
			}
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
			if (idNode == mtrEle._u8Node) {
				tilt.servo->posActual = _pos;
				tilt.servo->veloActual = _velo;

				tilt.relPosition = C_TO_DEG_EL(_pos);
				tilt.velo = C_TO_DEG_EL(_velo);

				countAngleVelo[1]++;
			}
#endif	//if MTR_EL_ENABLE==1

		}
	}
}
#endif	//if DEBUG_INGENIA_DRIVER==0

#if DEBUG_INGENIA_DRIVER==1
void Ingenia_tpdo_callback(CAN_Buffer_t *buffer)
{
	CAN_Data_t data;
	int _pos = 0, _velo = 0;

	if (can_buffer_available(buffer)) {
		can_buffer_read(buffer, &data);

		if (data.canRxHeader.StdId == (COB_TPDO4 | mtr._u8Node)) {
			for ( int i = 0; i < 4; i++ )
			_velo |= (int) data.rxData[i + 2] << (8 * i);

			mtr.veloActual = _velo;

			bufLen = sprintf(buf, "v= %d\r\n", mtr.veloActual);
			serial_write_str(&debug, buf, bufLen);
		}
		else if (data.canRxHeader.StdId == (COB_TPDO3 | mtr._u8Node)) {
			for ( int i = 0; i < 4; i++ )
			_pos |= (int) data.rxData[i + 2] << (8 * i);

			mtr.posActual = _pos;

			bufLen = sprintf(buf, "p= %d\t", mtr.posActual);
			serial_write_str(&debug, buf, bufLen);
		}

	}
}

static void displayCanBuffer(CAN_Data_t *data)
{
	bufLen = sprintf(buf, "%d\tid= 0x%03X rtr= %d dlc= %d\r\n", (int) HAL_GetTick(),
			(int) data->canRxHeader.StdId, (int) data->canRxHeader.RTR,
			(int) data->canRxHeader.DLC);
	serial_write_str(&debug, buf, bufLen);
	if (data->canRxHeader.DLC > 0) {
		bufLen = sprintf(buf, "data:");
		serial_write_str(&debug, buf, bufLen);
		for ( int i = 0; i < data->canRxHeader.DLC; i++ ) {
			bufLen = sprintf(buf, "%02X ", data->rxData[i]);
			serial_write_str(&debug, buf, bufLen);
		}
		serial_write_str(&debug, "\r\n", 2);
	}
}

static void debugMotor()
{
	uint8_t _veloDir = 0;
	int32_t _targetVelo = 0;
	int32_t _pos;

	Ingenia_write_nmt(&mtr, NMT_START_REMOTE_NODE);

	while (1) {
		char c;
		_pos = 0;

		HAL_IWDG_Refresh(&hiwdg);

		CAN_Data_t data;
		if (Ingenia_buffer_available(&mtr.buffer.bufEMER, &data) == HAL_OK)
		displayCanBuffer(&data);
		if (Ingenia_buffer_available(&mtr.buffer.bufNMT, &data) == HAL_OK)
		displayCanBuffer(&data);
		if (Ingenia_buffer_available(&mtr.buffer.bufTPDO, &data) == HAL_OK)
		displayCanBuffer(&data);
		if (Ingenia_buffer_available(&mtr.buffer.bufTSDO, &data) == HAL_OK)
		displayCanBuffer(&data);

		if (serial_available(&debug)) {
			c = serial_read(&debug);

			switch (c)
			{
				case 'e':
				bufLen = sprintf(buf, "enabling motor...\r\n");
				serial_write_str(&debug, buf, bufLen);

//				Ingenia_write_nmt(&mtr, NMT_START_REMOTE_NODE);
				Ingenia_enableMotor(&mtr);

				bufLen = sprintf(buf, "done!\r\n");
				serial_write_str(&debug, buf, bufLen);
				break;
				case 'd':
				bufLen = sprintf(buf, "disabling motor...\r\n");
				serial_write_str(&debug, buf, bufLen);

				Ingenia_disableMotor(&mtr);
//				Ingenia_write_nmt(&mtr, NMT_ENTER_PRE_OPS);

				bufLen = sprintf(buf, "done!\r\n");
				serial_write_str(&debug, buf, bufLen);
				break;
				case 'z':
				/* set velocity profile */
				_targetVelo = 2000;
				Ingenia_write_sdo_u32(&mtr, 0x6081, 0, _targetVelo);
				bufLen = sprintf(buf, "set profile velocity= %d\r\n",
						Ingenia_read_reg_sdo(&mtr, 0x6081, 0));
				serial_write_str(&debug, buf, bufLen);
				break;
				case 's':
				Ingenia_setTargetPositionAdv(&mtr, 0, 1, 1, 1);

				bufLen = sprintf(buf, "Stop motor at = %dc\r\n", _pos);
				serial_write_str(&debug, buf, bufLen);
				break;
				case '=':
				_pos = mtr.posActual + 10000;
				/* set position */
				Ingenia_setTargetPositionAdv(&mtr, _pos, 1, 0, 0);
				bufLen = sprintf(buf, "set target Pos= %d\r\n", _pos);
				serial_write_str(&debug, buf, bufLen);
				break;
				case '-':
				_pos = mtr.posActual - 10000;
				/* set position */
				Ingenia_setTargetPositionAdv(&mtr, _pos, 1, 0, 0);
				bufLen = sprintf(buf, "set target Pos= %d\r\n", _pos);
				serial_write_str(&debug, buf, bufLen);
				break;
				case ']':
				_targetVelo += 100;
				Ingenia_write_sdo_u32(&mtr, 0x6081, 0, _targetVelo);
				bufLen = sprintf(buf, "v= %d\r\n", _targetVelo);
				serial_write_str(&debug, buf, bufLen);
				break;
				case '[':
				if (_targetVelo > 100)
				_targetVelo -= 100;
				else
				_targetVelo = 0;
				Ingenia_write_sdo_u32(&mtr, 0x6081, 0, _targetVelo);
				bufLen = sprintf(buf, "v= %d\r\n", _targetVelo);
				serial_write_str(&debug, buf, bufLen);
				break;
				case 'p':
				bufLen = sprintf(buf, "set to profile position\r\n");
				serial_write_str(&debug, buf, bufLen);

				/* set mode to profile position */
				Ingenia_setModeOfOperation(&mtr, DRIVE_MODE_PROFILE_POSITION);
				/* set position */
				Ingenia_setTargetPositionAdv(&mtr, mtr.posActual, 1, 0, 0);
				break;
				case 'q':
				bufLen = sprintf(buf, "pos= %d\r\n", Ingenia_getActualPosition(&mtr));
				serial_write_str(&debug, buf, bufLen);
				break;
				case 'V':
				bufLen = sprintf(buf, "set to profile velocity\r\n");
				serial_write_str(&debug, buf, bufLen);

				/* set mode to profile velocity */
				Ingenia_setModeOfOperation(&mtr, DRIVE_MODE_PROFILE_VELOCITY);
				break;
				case 'v':
				/* set velocity */
				Ingenia_setTargetVelocity(&mtr, _targetVelo);

				if (_veloDir) {
					_targetVelo -= 100;
					if (_targetVelo <= -2500)
					_veloDir = 0;
				}
				else {
					_targetVelo += 100;
					if (_targetVelo >= 2500)
					_veloDir = 1;
				}
				break;
				case 'H':
				bufLen = sprintf(buf, "do homing...\r\n");
				serial_write_str(&debug, buf, bufLen);

				Ingenia_doHoming(&mtr, 35);

				bufLen = sprintf(buf, "done!\r\n");
				serial_write_str(&debug, buf, bufLen);
				break;
			}
		}
	}
}
#endif	//if DEBUG_INGENIA_DRIVER==1

/* USER CODE END 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

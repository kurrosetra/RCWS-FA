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
#include <stdio.h>
#include <string.h>

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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG				1

#if DEBUG==1
#	define DEBUG_INGENIA_DRIVER	1
#endif	//if DEBUG==1

#if DEBUG_INGENIA_DRIVER==0
#	define MTR_AZ_ENABLE		1
#	define MTR_EL_ENABLE		0
#	define MTR_COCK_ENABLE		0
#endif	//if DEBUG_INGENIA_DRIVER==0

#if DEBUG_INGENIA_DRIVER==1
#	define MTR_ID				0x20
#else
#	define MTR_AZ_ID			0x20
#	define MTR_EL_ID			0x21
#	define MTR_COCK_ID			0x22
#endif	//if DEBUG_INGENIA_DRIVER==1

#define UART_BUFSIZE		256

#ifndef CAN_DATA_MAX
#define CAN_DATA_MAX		8
#endif
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

Ring_Buffer_t tx6Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx6Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx6Buffer, &tx6Buffer, &huart6 };

#if DEBUG==1
char vt100_home[10];
#define DEBUG_LINE_MAX		25
char vt100_lineX[DEBUG_LINE_MAX][16];
#endif	//if DEBUG==1

CAN_TxHeaderTypeDef can1TxHeader;
uint32_t can1TxMailBox;
uint8_t can1TxBuffer[8];

CAN_TxHeaderTypeDef can2TxHeader;
uint32_t can2TxMailBox;
uint8_t can2TxBuffer[8];

CAN_TxHeaderTypeDef *busTxHeader;
uint32_t *busTxMailBox;
uint8_t *busTxBuffer;

#if DEBUG==1
Servo_t mtr;
Servo_t dummy;
#endif	//if DEBUG==1

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
#endif	//if DEBUG_INGENIA_DRIVER==1

static HAL_StatusTypeDef busInit();
static void busHandler(CAN_HandleTypeDef *bus);

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
	serial_init(&debug);

#if DEBUG==1
	bufLen = sprintf(buf, "%sIngenia CAN library\r\n", vt100_home);
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

	Ingenia_begin(&hcan1);

#if DEBUG_INGENIA_DRIVER==1
	Ingenia_init(&mtr, 0x21);
	Ingenia_init(&dummy, 0x22);
	Ingenia_node_remove(&dummy);
#endif	//if DEBUG_INGENIA_DRIVER==1

#if MTR_AZ_ENABLE==1
	Ingenia_init(&mtrAzi, MTR_AZ_ID);
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
	Ingenia_init(&mtrEle, MTR_EL_ID);
#endif	//if MTR_EL_ENABLE==1

#if MTR_COCK_ENABLE==1
	Ingenia_init(&mtrCock,MTR_COCK_ID);
#endif	//if MTR_COCK_ENABLE==1
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t millis;
#if DEBUG_INGENIA_DRIVER==1
	uint8_t _veloDir = 0;
	int32_t _targetVelo = 0;
#endif	//if DEBUG_INGENIA_DRIVER==1

	while (1) {
		millis = HAL_GetTick();
		HAL_IWDG_Refresh(&hiwdg);
#if DEBUG_INGENIA_DRIVER==1
		char c;
		int32_t _pos;
#endif	//if DEBUG_INGENIA_DRIVER==1
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

#if DEBUG_INGENIA_DRIVER==1
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
				bufLen = sprintf(buf, "enabling motor in Profile Position Mode...\r\n");
				serial_write_str(&debug, buf, bufLen);

				Ingenia_write_nmt(&mtr, NMT_START_REMOTE_NODE);
				Ingenia_enableMotor(&mtr);

				bufLen = sprintf(buf, "done!\r\n");
				serial_write_str(&debug, buf, bufLen);
				break;
			case 'd':
				bufLen = sprintf(buf, "disabling motor...\r\n");
				serial_write_str(&debug, buf, bufLen);

				Ingenia_disableMotor(&mtr);
				Ingenia_write_nmt(&mtr, NMT_ENTER_PRE_OPS);

				bufLen = sprintf(buf, "done!\r\n");
				serial_write_str(&debug, buf, bufLen);
				break;
			case 'z':
				/* set velocity profile */
				Ingenia_write_sdo_u32(&mtr, 0x6081, 0, 2000);
				bufLen = sprintf(buf, "set profile velocity= %d\r\n",
						Ingenia_read_reg_sdo(&mtr, 0x6081, 0));
				serial_write_str(&debug, buf, bufLen);
				break;
			case 's':
				_pos = mtr.posActual;
				/* set position */
				Ingenia_setTargetPositionAdv(&mtr, _pos, 1, 0, 0);
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
			case 'p':
				bufLen = sprintf(buf, "set to profile position\r\n");
				serial_write_str(&debug, buf, bufLen);

				/* set mode to profile position */
				Ingenia_setModeOfOperation(&mtr, DRIVE_MODE_PROFILE_POSITION);
				/* set position */
				Ingenia_setTargetPositionAdv(&mtr, mtr.posActual, 1, 0, 0);
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
					_targetVelo -= 5000;
					if (_targetVelo <= -25000)
						_veloDir = 0;
				}
				else {
					_targetVelo += 5000;
					if (_targetVelo >= 25000)
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
#endif	//if DEBUG_INGENIA_DRIVER==1
	}	//while(1){
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
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
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
	hcan1.Init.Prescaler = 5;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
	hcan2.Init.Prescaler = 12;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
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
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
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

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 89;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
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
	if (HAL_UART_Init(&huart6) != HAL_OK) {
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
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TRIGGER_ENABLE_GPIO_Port, TRIGGER_ENABLE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
	 PC1 PC2 PC3 PC4
	 PC5 PC8 PC9 PC10
	 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1
			| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9
			| GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA4 PA5 PA6
	 PA8 PA9 PA10 PA11
	 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8
			| GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LIM_AZ_Pin LIM_EL_UP_Pin LIM_EL_DOWN_Pin */
	GPIO_InitStruct.Pin = LIM_AZ_Pin | LIM_EL_UP_Pin | LIM_EL_DOWN_Pin;
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
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_14
			| GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
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
void USART6_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}

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

static HAL_StatusTypeDef busInit()
{
	busTxHeader = &can2TxHeader;
	busTxMailBox = &can2TxMailBox;
	busTxBuffer = can2TxBuffer;

	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 05;
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

#if DEBUG_INGENIA_DRIVER==1
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <math.h>
#include "vector.h"

#include "rwsCanID.h"
#include "stm_hal_serial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union
{
	float f;
	int32_t i32;
	uint32_t u32;
	uint8_t b[4];
} Union_u;

typedef union
{
	int16_t f;
	uint8_t b[2];
} Union_YPR;

typedef struct
{
	uint32_t id;
	volatile bool state;
	uint8_t data[8];
	uint8_t size;
	uint8_t online;
	uint32_t debugCounter;
} TCanRecvBuffer;

typedef struct
{
	uint32_t id;
	uint8_t data[8];
	uint8_t size;
	uint32_t debugCounter;
} TCanSendBuffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFSIZE				1024
#define CAN_BUFSIZE					8

#define CAN_ID_RWS_MTR_STAB_ANGLE		0x301
#define CAN_ID_RWS_MTR_STAB_SPD			0x302
#define CAN_ID_RWS_PNL_STAB_MODE		0x311
#define CAN_ID_RWS_PNL_STAB_MOVE_TARGET	0x313
#define CAN_ID_RWS_PNL_STAB_MOVE_TRACK	0x312

//ADD1602
#define CAN_ID_RWS_STAB_PNL				0x340
#define CAN_ID_RWS_STAB_PNL_TRCK		0x341
#define CAN_ID_RWS_STAB_PNL_YPR			0x342

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define kp1_v3 10.5
#define kp2_v3 18
#define kd1_v3 0.0
#define kd2_v3 0.1
#define ki1_v3 0.01
#define ki2_v3 0.05

#define kp1_v2 10.5
#define kp2_v2 18
#define kd1_v2 0.0
#define kd2_v2 0.1
#define ki1_v2 0.01
#define ki2_v2 0.05

#define kp1_v1 10.5
#define kp2_v1 16
#define kd1_v1 0.0
#define kd2_v1 0.01
#define ki1_v1 0.01
#define ki2_v1 0.01

#define kp1_v 10.5
#define kp2_v 12.5
#define kd1_v 0.0
#define kd2_v 0.01
#define ki1_v 0.01
#define ki2_v 0.01

// Freq Com in Hz
#define FREQ_COM 100
#define LIMIT_SPEED 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* TODO Global Variables*/
uint16_t bufLen = 0;
char buf[UART_BUFSIZE];

/* SERIAL 1 */
Ring_Buffer_t tx1Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx1Buffer = { { 0 }, 0, 0 };
TSerial imu = { &rx1Buffer, &tx1Buffer, &huart1 };

/* SERIAL 2 */
Ring_Buffer_t tx2Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx2Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx2Buffer, &tx2Buffer, &huart2 };

/* CAN BUS */
CAN_TxHeaderTypeDef can1TxHeader;
CAN_RxHeaderTypeDef can1RxHeader;
uint32_t can1TxMailBox;
uint8_t can1TxBuffer[CAN_BUFSIZE];
uint8_t can1RxBuffer[CAN_BUFSIZE];

TCanRecvBuffer canRecvMotorAngle = { CAN_ID_RWS_MTR_STAB_ANGLE, false, { 0 }, 8, 0, 0 };
TCanRecvBuffer canRecvMotorSpeed = { CAN_ID_RWS_MTR_STAB_SPD, false, { 0 }, 8, 0, 0 };
TCanRecvBuffer canRecvPanelMode = { CAN_ID_RWS_PNL_STAB_MODE, false, { 0 }, 1, 0, 0 };
TCanSendBuffer canSendStabilized = { CAN_ID_RWS_STAB_PNL, { 0 }, 8, 0 };

//ADD1602
TCanRecvBuffer canRecvPanelMoveTrack = { CAN_ID_RWS_PNL_STAB_MOVE_TRACK, false, { 0 }, 8, 0, 0 };
TCanRecvBuffer canRecvPanelMoveTarget = { CAN_ID_RWS_PNL_STAB_MOVE_TARGET, false, { 0 }, 8, 0, 0 };
TCanSendBuffer canSendStabilizedTrack = { CAN_ID_RWS_STAB_PNL_TRCK, { 0 }, 8, 0 };
TCanSendBuffer canSendYPR = { CAN_ID_RWS_STAB_PNL_YPR, { 0 }, 6, 0 };

/* Model States */
int countLog = 0;
float x0 = -90 * M_PI / 180;
float x1 = -90 * M_PI / 180;
float x2 = 0;
float theta1 = 90 * M_PI / 180;
float theta2 = 0;
float theta1_d, theta2_d;
float q_acc1, q_acc2;
float e_q1, e_q2;
float q_des_now_1, q_des_now_2;
int first_start = 1;
int countmove = 0;
float speed_limit = LIMIT_SPEED;
float cek_pitch, cek_roll;

float theta1_calc = 0;
float theta2_calc = 0;

float e_q1_last = 90 * M_PI / 180;
float e_q2_last = 0;
float e_qd1, e_qd2;

float Torque1 = 0;
float Torque2 = 0;
float pos_ee[3] = { 0, 0, 0 };
float target_pos[3] = { 0, 0, 0 };
float lengan_pos[3] = { 0, 0, 0 };
float lenganq_pos[3] = { 0, 0, 0 };
float target_pitch = 0;
float dpitch, dyaw;

float backup_val[6] = { kp1_v, kp2_v, kd1_v, kd2_v, ki1_v, ki2_v };
float kp1 = kp1_v, kp2 = kp2_v;
float kd1 = kd1_v, kd2 = kd2_v;
float ki1 = ki1_v, ki2 = ki2_v;
int save_pid_q = 0;
int debugIMUcount = 0;
int i = 0;

float theta1_target = 90 * M_PI / 180;
float theta2_target = 0;
float x0_target = -90 * M_PI / 180;
float x1_target = -90 * M_PI / 180;

float offset0 = 0.0f, offset1 = 0.0f;
int set_2 = 0;

uint32_t period_com = 1000 / FREQ_COM;

/* Move Target */
float move_theta1, move_theta2;
bool update_move;

//ADD1602
float track_theta1, track_theta2;
bool update_move_track;

/* CAN TX Vars */
Union_u sp_send1;
Union_u sp_send2;

/* CAN RX Vars */
bool mode = false;
bool mode_track = false;

/* IMU */
char imuBuf[UART_BUFSIZE];
Union_YPR yaw, pitch, roll;

volatile uint32_t Milliseconds;
volatile uint32_t Microseconds;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void imuHandler();
static void busHandler();
static void kontrolHandler();
static void calc_torques();
static void forward_kinematics();
static void log_data();
static void reset_IMU_integrals();
static float roundOneDec(float);
static void calc_target();
static void gain_schedule();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CAN_Config()
{
	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/

	/* filter can id = CAN_ID_RWS_PNL_STAB_MODE= 0x311 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_PNL_STAB_MODE << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_PNL_STAB_MODE << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_MTR_STAB_ANGLE= 0x301 */
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_MTR_STAB_ANGLE << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_MTR_STAB_ANGLE << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_MTR_STAB_SPD= 0x302 */
	sFilterConfig.FilterBank = 2;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_MTR_STAB_SPD << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_MTR_STAB_SPD << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_PNL_STAB_MOVE_TRACK= 0x312 */
	sFilterConfig.FilterBank = 3;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_PNL_STAB_MOVE_TRACK << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_PNL_STAB_MOVE_TRACK << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_PNL_STAB_MOVE_TRACK= 0x312 */
	sFilterConfig.FilterBank = 4;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_PNL_STAB_MOVE_TARGET << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_PNL_STAB_MOVE_TARGET << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	/*##-5- Configure Transmission process #####################################*/
	can1TxHeader.StdId = 0x123;
	can1TxHeader.ExtId = 0x01;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.DLC = CAN_BUFSIZE;
	can1TxHeader.TransmitGlobalTime = DISABLE;
}

static void ledToggle()
{
	if (__HAL_TIM_GET_COMPARE(&htim2,TIM_CHANNEL_1) == PWM_LED_OFF)
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_LED_ON);
	else
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_LED_OFF);
}

static void ledOn()
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_LED_ON);
}

static void ledOff()
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_LED_OFF);
}

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

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_IWDG_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */
	HAL_IWDG_Refresh(&hiwdg);

	serial_init(&debug);
	bufLen = sprintf(buf, "\r\nStabilizer & Tracker Control Firmware!\r\n");
	serial_write_str(&debug, buf, bufLen);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	ledOn();

//	HAL_TIM_Base_Start_IT(&htim1);
//	__HAL_TIM_SET_COUNTER(&htim1, 0);

	serial_init(&imu);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
//	uint32_t debugTimer = 0;
//	uint32_t pwmLedPeriod = PWM_LED_OFF;
	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		imuHandler();
		busHandler();
		kontrolHandler();
//		if (HAL_GetTick() >= debugTimer) {
//			debugTimer = HAL_GetTick() + 1000;
//
//			if (pwmLedPeriod == PWM_LED_OFF)
//				pwmLedPeriod = PWM_LED_ON;
//			else
//				pwmLedPeriod = PWM_LED_OFF;
//
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmLedPeriod);
//			bufLen = sprintf(buf, "pwm= %d t=%d%03dus\r\n", pwmLedPeriod, Milliseconds,
//					Microseconds);
//			serial_write_str(&debug, buf, bufLen);
//		}

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
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
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
	/** Initializes the CPU, AHB and APB buses clocks
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
	hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
	CAN_Config();
	/* USER CODE END CAN1_Init 2 */

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
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 125;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 89;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 89;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = PWM_LED_OFF;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 230400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	huart2.Init.BaudRate = 921600;
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

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
	 PC1 PC2 PC3 PC4
	 PC5 PC6 PC7 PC8
	 PC9 PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1
			| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
			| GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 PA6
	 PA7 PA8 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7
			| GPIO_PIN_8 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10
	 PB12 PB13 PB14 PB15
	 PB3 PB4 PB5 PB6
	 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_12
			| GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5
			| GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Begin of Functions Declaration*/

//void TIM1_UP_TIM10_IRQHandler(void)
//{
//	/* TIM Update event */
//	if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) {
//		if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET) {
//			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
//
//			Microseconds++;
//			if (Microseconds >= 1000) {
//				Microseconds = 0;
//				Milliseconds++;
//			}
//		}
//	}
//}
/* TODO Begin of Functions Declaration*/
void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(&imu);
}

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}

/**
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t _id = 0;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxHeader, can1RxBuffer) == HAL_OK) {
		_id = can1RxHeader.StdId;
		if (_id == canRecvMotorAngle.id) {
			canRecvMotorAngle.state = true;
			memcpy(canRecvMotorAngle.data, can1RxBuffer, canRecvMotorAngle.size);
			canRecvMotorAngle.debugCounter++;
		}
		else if (_id == canRecvMotorSpeed.id) {
			canRecvMotorSpeed.state = true;
			memcpy(canRecvMotorSpeed.data, can1RxBuffer, canRecvMotorSpeed.size);
			canRecvMotorSpeed.debugCounter++;
		}
		else if (_id == canRecvPanelMode.id) {
			canRecvPanelMode.state = true;
			memcpy(canRecvPanelMode.data, can1RxBuffer, canRecvPanelMode.size);
			canRecvPanelMode.debugCounter++;
		}
		else if (_id == canRecvPanelMoveTarget.id) {
			canRecvPanelMoveTarget.state = true;
			memcpy(canRecvPanelMoveTarget.data, can1RxBuffer, canRecvPanelMoveTarget.size);
			canRecvPanelMoveTarget.debugCounter++;
		}
		else if (_id == canRecvPanelMoveTrack.id) {
			canRecvPanelMoveTrack.state = true;
			memcpy(canRecvPanelMoveTrack.data, can1RxBuffer, canRecvPanelMoveTrack.size);
			canRecvPanelMoveTrack.debugCounter++;
		}
	}
}

static char** str_split(char* a_str, const char a_delim)
{
	char** result = 0;
	size_t count = 0;
	char* tmp = a_str;
	char* last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;

	/* Count how many elements will be extracted. */
	while (*tmp) {
		if (a_delim == *tmp) {
			count++;
			last_comma = tmp;
		}
		tmp++;
	}

	/* Add space for trailing token. */
	count += last_comma < (a_str + strlen(a_str) - 1);

	/* Add space for terminating null string so caller
	 knows where the list of returned strings ends. */
	count++;

	result = malloc(sizeof(char*) * count);

	if (result) {
		size_t idx = 0;
		char* token = strtok(a_str, delim);

		while (token) {
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		*(result + idx) = 0;
	}

	return result;
}

static void busHandler()
{
	bool panelCmdMode = false;
	bool panelCmdModeTrack = false;
	uint8_t recvBuffer[8];
	Union_u x, y;

	if (canRecvPanelMode.state) {
		canRecvPanelMode.state = false;
		panelCmdMode = bitRead(canRecvPanelMode.data[0], 0);
		panelCmdModeTrack = bitRead(canRecvPanelMode.data[0], 1);
		if (mode != panelCmdMode) {
			if (panelCmdMode) {
				/* start stab */
				first_start = 1;
				reset_IMU_integrals();
				ledOn();
			}
			else {
				/* end stab */
				reset_IMU_integrals();
				ledOff();
				update_move = false;
			}

			mode = panelCmdMode;
		}

		//ADD1602
		if (mode_track != panelCmdModeTrack) {
			if (panelCmdModeTrack) {
				/* start track */
				first_start = 1;
				reset_IMU_integrals();
				ledOn();
			}
			else {
				/* end track */
				reset_IMU_integrals();
				ledOff();
				update_move_track = false;
			}
			mode_track = panelCmdModeTrack;
		}
	}

	/* new data actual angle from motor */
	if (canRecvMotorAngle.state) {
		canRecvMotorAngle.state = false;
		memcpy(recvBuffer, canRecvMotorAngle.data, 8);

		for ( int i = 0; i < 4; i++ ) {
			x.b[i] = recvBuffer[i];
			y.b[i] = recvBuffer[4 + i];
		}
		/*
		 * actual angle azimuth = x.f
		 * actual angle elevation = y.f
		 */
		theta1 = wrapAngle(toRad(-x.f + 90));
		theta2 = wrapAngle(toRad(y.f));
	}

	/* new data actual speed from motor */
	if (canRecvMotorSpeed.state) {
		canRecvMotorSpeed.state = false;

		memcpy(recvBuffer, canRecvMotorSpeed.data, 8);

		for ( int i = 0; i < 4; i++ ) {
			x.b[i] = recvBuffer[i];
			y.b[i] = recvBuffer[4 + i];
		}
		/*
		 * actual speed azimuth = x.f
		 * actual speed elevation = y.f
		 */
		theta1_d = toRad(-x.f);
		theta2_d = toRad(y.f);

		if (fabs(-x.f) < 0.061) {
			theta1_d = 0;
		}
		if (fabs(y.f) < 0.061) {
			theta2_d = 0;
		}

	}

	/* new data move target from panel */
	if (canRecvPanelMoveTarget.state) {
		canRecvPanelMoveTarget.state = false;

		memcpy(recvBuffer, canRecvPanelMoveTarget.data, 8);

		for ( int i = 0; i < 4; i++ ) {
			x.b[i] = recvBuffer[i];
			y.b[i] = recvBuffer[4 + i];
		}
		/*
		 * move target speed azimuth = x.f
		 * move target speed elevation = y.f
		 */
		move_theta1 = -x.f * 0.1;  //10 Hz
		move_theta2 = y.f * 0.1;
		update_move = true;
	}

	//ADD1602
	/* new data track from panel */
	if (canRecvPanelMoveTrack.state) {
		canRecvPanelMoveTrack.state = false;

		memcpy(recvBuffer, canRecvPanelMoveTrack.data, 8);

		for ( int i = 0; i < 4; i++ ) {
			x.b[i] = recvBuffer[i];
			y.b[i] = recvBuffer[4 + i];
		}
		/*
		 * move track speed azimuth = x.f
		 * move track speed elevation = y.f
		 */
		track_theta1 = -x.f * 0.1;  //10 Hz
		track_theta2 = y.f * 0.1;
		update_move_track = true;
	}

}

//ADD1602
static float roundOneDec(float val)
{
	float value = (int) (val * 10 + fabs(val) / val * .5);
	return value;
}

static void imuHandler()
{
	char c[2] = { 0, 0 };
	bool dataUpdate = false;
	char *s;
	char **tokens;

	while (serial_available(&imu)) {
		c[0] = serial_read(&imu);
		if (c[0] == '$') {
			memset(imuBuf, 0, UART_BUFSIZE);
		}
		else if (c[0] == '*') {
			dataUpdate = true;
//			strcat(imuBuf, c);
			break;
		}
		strcat(imuBuf, c);
	}

	if (dataUpdate) {
		debugIMUcount += 1;
		/* $VNYPR,+141.058,+002.072,-179.441,... */
//		bufLen = sprintf(buf, "recv msg: \"%s\"\r\n", imuBuf);
//		serial_write_str(&debug, buf, bufLen);
		s = strstr(imuBuf, "$VNYPR,");
		if (s) {
			tokens = str_split(imuBuf, ',');
			if (tokens) {
				for ( int i = 0; *(tokens + i); i++ ) {
					s = *(tokens + i);

					switch (i)
					{
					case 1:
						/* yaw */
						yaw.f = roundOneDec(atof(s));
						break;
					case 2:
						/* pitch */
//						if(!mode){offset0 = atof(s);}
//						x0 = wrapAngle(toRad(atof(s)-90-offset0));
//						if(!mode){offset0 = atof(s);}
						if (fabs(atof(s) - cek_pitch) >= 0.005)
							x0 = wrapAngle(toRad(atof(s) - 90 - offset0));
						cek_pitch = atof(s);
						pitch.f = roundOneDec(atof(s));
						break;
					case 3:
						/* roll */
//						if(!mode){offset1 = atof(s);}
						if (fabs(atof(s) - cek_roll) >= 0.005)
							x1 = wrapAngle(toRad(atof(s) - 90 - offset1));
						cek_roll = atof(s);
						roll.f = roundOneDec(atof(s));
						break;
					}
					free(*(tokens + i));
				}	//for ( int i = 0; *(tokens + i); i++ ) {
				free(tokens);
			}	//if (tokens) {
		} /*if (s) {*/
	} /*if (dataUpdate) {*/
}

static void kontrolHandler()
{
	static uint32_t processTimer = 0;
	static uint32_t imuTimer = 0;

	//ADD1602
	if (HAL_GetTick() >= imuTimer) {
		imuTimer = HAL_GetTick() + 100;  //10Hz

		// Send Data
		for ( int i = 0; i < 2; i++ ) {
			canSendYPR.data[i] = yaw.b[i];
			canSendYPR.data[i + 2] = pitch.b[i];
			canSendYPR.data[i + 4] = roll.b[i];
		}
		can1TxHeader.StdId = canSendYPR.id;
		can1TxHeader.DLC = canSendYPR.size;
		memcpy(can1TxBuffer, canSendYPR.data, canSendYPR.size);

		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) == HAL_OK) {
			canSendYPR.debugCounter++;
		}
		else {
			/* if failed, send again after 1ms */
			imuTimer = HAL_GetTick() + 1;
		}

	}

	if (HAL_GetTick() >= processTimer) {
		processTimer = HAL_GetTick() + period_com;

		if (mode || mode_track) {
			forward_kinematics();  // OK
			calc_torques();  // OK
		}
		else if (!mode) {
			if (!mode_track) {
				sp_send1.f = 0.0;
				sp_send2.f = 0.0;
			}
		}

		// Send Data
//		/* NO STAB */
//		sp_send1.f = 0;
//		sp_send2.f = 0;

		//ADD1602
		if (!mode_track) {  // if track disabled
			for ( int i = 0; i < 4; i++ ) {
				canSendStabilized.data[i] = sp_send1.b[i];
				canSendStabilized.data[i + 4] = sp_send2.b[i];
			}
			can1TxHeader.StdId = canSendStabilized.id;
			can1TxHeader.DLC = canSendStabilized.size;
			memcpy(can1TxBuffer, canSendStabilized.data, canSendStabilized.size);
		}
		else {
			for ( int i = 0; i < 4; i++ ) {
				canSendStabilizedTrack.data[i] = sp_send1.b[i];
				canSendStabilizedTrack.data[i + 4] = sp_send2.b[i];
			}
			can1TxHeader.StdId = canSendStabilizedTrack.id;
			can1TxHeader.DLC = canSendStabilizedTrack.size;
			memcpy(can1TxBuffer, canSendStabilizedTrack.data, canSendStabilizedTrack.size);
		}

		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) == HAL_OK) {
			canSendStabilized.debugCounter++;
			if (speed_limit < LIMIT_SPEED) {
				speed_limit += 1;
				speed_limit = wrapVal(speed_limit, LIMIT_SPEED);
			}

		}
		else {
			/* if failed, send again after 1ms */
			processTimer = HAL_GetTick() + 1;

			/* Smoothing After Data Loss*/
			speed_limit = 3;
		}

		log_data();
	}
}

static void reset_IMU_integrals()
{
	q_acc1 = 0;
	q_acc2 = 0;
	dpitch = 0;
	dyaw = 0;
	sp_send1.f = 0.0;
	sp_send2.f = 0.0;
	e_q1 = 0;
	e_q2 = 0;
	e_qd1 = 0;
	e_qd2 = 0;
}

static float find_pitch(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return atan2f(z2 - z1, sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)));
}

//static float find_dyaw(float x1, float y1, float z1, float x2, float y2, float z2, float xt, float yt, float zt){
//  float a = wrapVal( ((x2-x1)*(xt-x1) + (y2-y1)*(yt-y1))/ (sqrt(pow((x2-x1),2) + pow((y2-y1),2)) * sqrt(pow((xt-x1),2) + pow((yt-y1),2))), 1.0);
//  if(isnan(a)) return 0;
//  else return (sign((x2-x1)*(yt-y1) - (y2-y1) * (xt-x1)) * acos(a));
//}

static void calc_target()
{
	target_pos[0] = 1.0965
			* (-sinf(theta1_target) * sinf(x0_target)
					+ cosf(theta1_target) * cosf(x0_target) * cosf(x1_target)) * cosf(theta2_target)
			+ 0.135 * sinf(theta1_target) * sinf(x0_target)
			+ 1.0965 * sinf(theta2_target) * sinf(x1_target) * cosf(x0_target)
			+ 0.4705 * sinf(x1_target) * cosf(x0_target)
			- 0.135 * cosf(theta1_target) * cosf(x0_target) * cosf(x1_target);
	target_pos[1] = -1.0965 * sinf(theta2_target) * cosf(x1_target)
			+ 1.0965 * sinf(x1_target) * cosf(theta1_target) * cosf(theta2_target)
			- 0.135 * sinf(x1_target) * cosf(theta1_target) - 0.4705 * cosf(x1_target);
	target_pos[2] = 1.0965
			* (sinf(theta1_target) * cosf(x0_target)
					+ sinf(x0_target) * cosf(theta1_target) * cosf(x1_target)) * cosf(theta2_target)
			- 0.135 * sinf(theta1_target) * cosf(x0_target)
			+ 1.0965 * sinf(theta2_target) * sinf(x0_target) * sinf(x1_target)
			+ 0.4705 * sinf(x0_target) * sinf(x1_target)
			- 0.135 * sinf(x0_target) * cosf(theta1_target) * cosf(x1_target);

	lenganq_pos[0] = 0.135 * sinf(theta1_calc) * sinf(x0_target)
			+ 0.4705 * sinf(x1_target) * cosf(x0_target)
			- 0.135 * cosf(theta1_calc) * cosf(x0_target) * cosf(x1_target);
	lenganq_pos[1] = -0.135 * sinf(x1_target) * cosf(theta1_calc) - 0.4705 * cosf(x1_target);
	lenganq_pos[2] = -0.135 * sinf(theta1_calc) * cosf(x0_target)
			+ 0.4705 * sinf(x0_target) * sinf(x1_target)
			- 0.135 * sinf(x0_target) * cosf(theta1_calc) * cosf(x1_target);

	target_pitch = find_pitch(lenganq_pos[0], lenganq_pos[1], lenganq_pos[2], target_pos[0],
			target_pos[1], target_pos[2]);
}

static void forward_kinematics()
{
	//1. Calc Target Pos (Frame I)
	if (first_start == 1) {
		reset_IMU_integrals();
		theta1_target = theta1_calc = theta1;
		theta2_target = theta2_calc = theta2;
		x0_target = x0;
		x1_target = x1;
		calc_target();

		first_start = 0;
	}

	//ADD1602

	if (update_move && !mode_track) {
		Torque1 = toRad(move_theta1 * 10);
		theta2_target += toRad(move_theta2);
		calc_target();
		countmove = 0;
		update_move = false;
	}

	if (update_move_track && mode_track) {
		Torque1 = toRad(track_theta1 * 10);
		theta2_target += toRad(track_theta2);
		calc_target();
		countmove = 0;
		update_move_track = false;
	}

	//2. Calc Pos EE
	pos_ee[0] = 1.0965 * (-sinf(theta1) * sinf(x0) + cosf(theta1) * cosf(x0) * cosf(x1))
			* cosf(theta2) + 0.135 * sinf(theta1) * sinf(x0)
			+ 1.0965 * sinf(theta2) * sinf(x1) * cosf(x0) + 0.4705 * sinf(x1) * cosf(x0)
			- 0.135 * cosf(theta1) * cosf(x0) * cosf(x1);
	pos_ee[1] = -1.0965 * sinf(theta2) * cosf(x1) + 1.0965 * sinf(x1) * cosf(theta1) * cosf(theta2)
			- 0.135 * sinf(x1) * cosf(theta1) - 0.4705 * cosf(x1);
	pos_ee[2] = 1.0965 * (sinf(theta1) * cosf(x0) + sinf(x0) * cosf(theta1) * cosf(x1))
			* cosf(theta2) - 0.135 * sinf(theta1) * cosf(x0)
			+ 1.0965 * sinf(theta2) * sinf(x0) * sinf(x1) + 0.4705 * sinf(x0) * sinf(x1)
			- 0.135 * sinf(x0) * cosf(theta1) * cosf(x1);

	//3. Calc Lengan Pos
	lengan_pos[0] = 0.135 * sinf(theta1) * sinf(x0) + 0.4705 * sinf(x1) * cosf(x0)
			- 0.135 * cosf(theta1) * cosf(x0) * cosf(x1);
	lengan_pos[1] = -0.135 * sinf(x1) * cosf(theta1) - 0.4705 * cosf(x1);
	lengan_pos[2] = -0.135 * sinf(theta1) * cosf(x0) + 0.4705 * sinf(x0) * sinf(x1)
			- 0.135 * sinf(x0) * cosf(theta1) * cosf(x1);

	dpitch = target_pitch
			- find_pitch(lengan_pos[0], lengan_pos[1], lengan_pos[2], pos_ee[0], pos_ee[1],
					pos_ee[2]);
	dyaw = 0;  //find_dyaw(lengan_pos[0], lengan_pos[1], lengan_pos[2], pos_ee[0], pos_ee[1], pos_ee[2], target_pos[0], target_pos[1], target_pos[2]);
//  if(fabs(dpitch)<5E-5) dpitch = 0;
//  if(fabs(dyaw)<5E-5) dyaw = 0;

	if (sqrt(
			pow(target_pos[0] - pos_ee[0], 2) + pow(target_pos[1] - pos_ee[1], 2)
					+ pow(target_pos[2] - pos_ee[2], 2)) < 1E-6)
		dpitch = 0;

	q_des_now_1 = theta1 + dyaw;
	q_des_now_2 = theta2 + dpitch;
}

static void calc_torques()
{
//  e_q1 = wrapVal(q_des_now_1 - theta1, toRad(5));
	e_q2 = wrapVal(q_des_now_2 - theta2, toRad(5));

//  e_qd1 = wrapVal( (e_q1-e_q1_last)/(period_com/1000) , toRad(25));
	e_qd2 = wrapVal((e_q2 - e_q2_last) / (period_com / 1000), toRad(25));
	gain_schedule();
	// Reset Integral Every Time Error Crossed zero
//  if(fabs(e_q1) <= toRad(0.05)) { q_acc1 = 0; }
	if (fabs(e_q2) <= toRad(0.05)) {
		q_acc2 = 0;
	}

	if (countmove >= 10)
		Torque1 = 0;
	else
		countmove++;
//  Torque1 = (kp1 * e_q1 - kd1 * e_qd1 + ki1 * q_acc1);
	Torque2 = (kp2 * e_q2 - kd2 * e_qd2 + ki2 * q_acc2);

	sp_send1.f = wrapVal(-toDeg(Torque1), speed_limit / 2);
	sp_send2.f = wrapVal(toDeg(Torque2), speed_limit);  //90

	//ADD1602
//  if(update_move && !mode_track){
//	  sp_send1.f += move_theta1;
//	  sp_send2.f += move_theta2;
//	  update_move = false;
//  }

//  if(update_move_track && mode_track){
//	  sp_send1.f += track_theta1;
//	  sp_send2.f += track_theta2;
//	  update_move_track = false;
//  }

	if (fabs(sp_send1.f) <= 1E-5) {
		sp_send1.f = 0;
	}
	if (fabs(sp_send2.f) <= 0.001) {
		sp_send2.f = 0;
	}

	if (isnan(sp_send1.f))
		sp_send1.f = 0;
	if (isnan(sp_send2.f))
		sp_send2.f = 0;

	q_acc1 += e_q1;
	q_acc2 += e_q2;
	q_acc1 = wrapVal(q_acc1, 50);
	q_acc2 = wrapVal(q_acc2, 50);
	e_q1_last = e_q1;
	e_q2_last = e_q2;
}

static void log_data()
{
	// mode, sp_send1, sp_send2, dyaw, dpitch, x0, x1, theta1, theta2, theta1_d, theta2_d
//	bufLen = sprintf(buf, "%ld, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f \r\n",
//					 HAL_GetTick(), mode, sp_send1.f, sp_send2.f, toDeg(-dyaw), toDeg(dpitch), toDeg(x0), toDeg(x1), toDeg(theta1), toDeg(theta2), toDeg(theta1_d), toDeg(theta2_d));

	bufLen = sprintf(buf,
			"%ld, %d, %d, %d, %.3f, %.3f, %f, %f, %f, %f, %.4f, %.4f, %.3f, %.3f \r\n",
			HAL_GetTick(), mode, mode_track, set_2, sp_send1.f, sp_send2.f, toDeg(-dyaw),
			toDeg(dpitch), cek_pitch, cek_roll, theta1, theta2, theta1_d, theta2_d);

//	bufLen = sprintf(buf, "%ld, %ld, %d, %d, %.3f, %.3f, %f, %f, %f, %f, %.4f, %.4f \r\n",
//							 HAL_GetTick(), canRecvPanelMoveTrack.debugCounter, mode, mode_track, sp_send1.f, sp_send2.f, toDeg(-dyaw), toDeg(dpitch), x0, x1, theta1, theta2);

	serial_write_str(&debug, buf, bufLen);

//	countLog+=1;
//	bufLen = sprintf(buf, "%ld, %ld, %f \r\n", HAL_GetTick(), canRecvMotorAngle.debugCounter,theta1);
//	serial_write_str(&debug, buf, bufLen);
//	if(countLog==200){
//
//		canRecvMotorAngle.debugCounter = 0;
//		countLog = 0;
//	}

}

static void gain_schedule()
{
	int E2, DE2;  //1 besar, 0 kecil
	E2 = DE2 = 0;
	set_2 = 0;

	if (fabs(e_q2) >= toRad(2.2))
		E2 = 1;
	if (fabs(e_qd2) >= toRad(8))
		DE2 = 1;

	if (E2 && DE2)
		set_2 = 2;
	else if (!E2 && DE2)
		set_2 = 1;
	else if (E2 && !DE2)
		set_2 = 1;
	else if (!(E2))
		set_2 = 0;

	if (set_2 == 0) {  //err kecil, derr kecil
		kp2 = kp2_v;
		kd2 = kd2_v;
		ki2 = ki2_v;
	}
	else if (set_2 == 1) {  //err besar, derr kecil
		kp2 = kp2_v1;
		kd2 = kd2_v1;
		ki2 = ki2_v1;
	}
	else if (set_2 == 2) {  //err kecil, derr besar
		kp2 = kp2_v2;
		kd2 = kd2_v2;
		ki2 = ki2_v2;
	}
	else if (set_2 == 3) {  //err besar, derr besar
		kp2 = kp2_v3;
		kd2 = kd2_v3;
		ki2 = ki2_v3;
	}

}

/* TODO END of Functions Declarations*/

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

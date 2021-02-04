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

#include "rwsCanID.h"
#include "stm_hal_serial.h"
#include "libVisca.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Global Variables*/

#define DEBUG					0
#if DEBUG==1
#	define DEBUG_BUS			1
#	define DEBUG_CAM			1
#	define DEBUG_LRF			1
#endif	//if DEBUG==1

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define UART_BUFSIZE	RING_BUFFER_SIZE

uint16_t bufLen = 0;
char buf[128];

#if DEBUG==1
Ring_Buffer_t tx4Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx4Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx4Buffer, &tx4Buffer, &huart4 };

char vt100_home[10];
#define DEBUG_LINE_MAX		25
char vt100_lineX[DEBUG_LINE_MAX][16];
#endif	//if DEBUG==1

Ring_Buffer_t tx6Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx6Buffer = { { 0 }, 0, 0 };
TSerial sony = { &rx6Buffer, &tx6Buffer, &huart6 };
volatile uint8_t sonyRetState = 0;

Ring_Buffer_t tx3Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx3Buffer = { { 0 }, 0, 0 };
TSerial thermal = { &rx3Buffer, &tx3Buffer, &huart3 };
volatile uint8_t thermalRetState = 0;

Ring_Buffer_t tx2Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx2Buffer = { { 0 }, 0, 0 };
TSerial lrf = { &rx2Buffer, &tx2Buffer, &huart2 };

VISCAInterface_t sonyIface;
VISCACamera_t sonyCamera;

#define SONY_FOCUS_SPEED			1
typedef enum
{
	SONY_FOCUS_SPEED_0 = 0,
	SONY_FOCUS_SPEED_1,
	SONY_FOCUS_SPEED_2,
	SONY_FOCUS_SPEED_3,
	SONY_FOCUS_SPEED_4,
	SONY_FOCUS_SPEED_5,
	SONY_FOCUS_SPEED_6,
	SONY_FOCUS_SPEED_7
} SonyFocusSpeed_e;
uint8_t sonyFocusSpeedByZoom[4] = { SONY_FOCUS_SPEED_3, SONY_FOCUS_SPEED_3, SONY_FOCUS_SPEED_2,
		SONY_FOCUS_SPEED_2 };

typedef enum
{
	ZOOM_IN_bit = 1,
	ZOOM_OUT_bit,
	FOCUS_FAR_bit,
	FOCUS_NEAR_bit,
	BRIGHTNESS_UP_bit,
	BRIGHTNESS_DOWN_bit
} ZoomFocusAperture_bitMasking;

typedef enum
{
	zf_none = 0,
	zf_plus = 1,
	zf_min = 2
} ZoomFocus_value;

typedef enum
{
	CAM_ST_NONE = 0,
	CAM_ST_SONY = 1,
	CAM_ST_THERMAL = 2
} CameraState_e;

typedef enum
{
	BR_ST_NONE = 0,
	BR_ST_UP,
	BR_ST_DOWN
} BrightnessState_e;

typedef struct
{
	uint8_t camState;
	uint8_t zoomState;
	uint8_t focusState;
	uint8_t zoomValue;
	uint8_t focusValue;
	uint8_t brightnessState;
} TCameraData;
TCameraData camData = { CAM_ST_NONE, 0, 0, 0, 0 };

const uint8_t zoomMin[7] = { 0xFF, 0x01, 0, 0x40, 0, 0, 0x41 };
const uint8_t zoomMax[7] = { 0xFF, 0x01, 0, 0x20, 0, 0, 0x21 };
const uint8_t focusMin[7] = { 0xFF, 0x01, 0x01, 0, 0, 0, 0x02 };
const uint8_t focusMax[7] = { 0xFF, 0x01, 0, 0x80, 0, 0, 0x81 };
const uint8_t zfStop[7] = { 0xFF, 0x01, 0, 0, 0, 0, 0x01 };

#define LRF_BUFSIZE		16
typedef enum
{
	LRF_STATE_STANDBY,
	LRF_STATE_POWER_ON,
	LRF_STATE_READY,
	LRF_STATE_1_MEASUREMENT,
	LRF_STATE_DELAY_MEASUREMENT,
	LRF_STATE_CONT_MEASUREMENT,
	LRF_STATE_POWER_OFF
} LRF_StateTypeDef;

typedef enum
{
	LRF_CMD_ENABLE_bit = 0,
	LRF_CMD_START_bit = 1,
	LRF_CMD_START_CONT_bit = 2,
	LRF_CMD_POINTER_bit = 3
} LRF_CommandTypeDef;

typedef struct
{
	uint8_t state;
	uint8_t validCounter;
	uint16_t distance;
	uint8_t buf[LRF_BUFSIZE];
} LRF_HandleTypeDef;
LRF_HandleTypeDef lrfData = { LRF_STATE_STANDBY, 0, 0, { 0 } };

#define CAN_BUFSIZE		8

CAN_TxHeaderTypeDef can1TxHeader;
CAN_RxHeaderTypeDef can1RxHeader;
uint32_t can1TxMailBox;
uint8_t can1TxBuffer[CAN_BUFSIZE];
uint8_t can1RxBuffer[CAN_BUFSIZE];
const uint32_t CAN_RECEIVE_TIMEOUT = 500;

volatile bool canTransmitState = false;

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

TCanRecvBuffer canRecvButton = { CAN_ID_RWS_BUTTON, false, { 0 }, 3, 0 };

TCanSendBuffer canSendOptLrf = { CAN_ID_RWS_OPT_LRF, { 0 }, 3 };
TCanSendBuffer canSendOptCam = { CAN_ID_RWS_OPT_CAM, { 0 }, 1 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void CAN_Config();
static void canHandler();
static void camHandler();
static void lrfHandler();
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
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_UART4_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Initialization*/
	HAL_IWDG_Init(&hiwdg);
	HAL_IWDG_Refresh(&hiwdg);

#if DEBUG==1
	serial_init(&debug);

	bufLen = sprintf(buf, "%sOptronik Firmware\r\n", vt100_home);
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

	sonyIface.port_fd = &sony;
	sonyIface.broadcast = 0;
	sonyCamera.address = 1;

	serial_init(&sony);
	serial_init(&thermal);
	serial_init(&lrf);

	/*##-1- Configure the CAN peripheral #######################################*/
	CAN_Config();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t millis;
	uint32_t ledTimer = 1000;

	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		millis = HAL_GetTick();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/

		canHandler();
		lrfHandler();
		camHandler();

		if (millis >= ledTimer) {
			ledTimer = millis + 500;

			if (canRecvButton.online)
				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
			else
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		}

#if DEBUG_CAM==1
		if (serial_available(&debug)) {
			char c = serial_read(&debug);
			uint16_t shutterValue = 0x6;
			uint8_t mode = 0;

			switch (c)
			{
			case 'g':
				/* get auto exp mode */
				if (VISCA_get_auto_exp_mode(&sonyIface, &sonyCamera, &mode) == VISCA_SUCCESS)
					bufLen = sprintf(buf, "%sauto exp mode: %d\r\n", vt100_lineX[4], mode);
				else
					bufLen = sprintf(buf, "%sfailed to get AE mode\r\n", vt100_lineX[4]);
				serial_write_str(&debug, buf, bufLen);
				break;
			case 'f':
				/* auto exp set to full auto */
				VISCA_set_auto_exp_mode(&sonyIface, &sonyCamera, VISCA_AUTO_EXP_FULL_AUTO);
				bufLen = sprintf(buf, "%sauto exp: FULL AUTO mode\r\n", vt100_lineX[4]);
				serial_write_str(&debug, buf, bufLen);
				break;
			case 's':
				/* auto exp set to shutter priority */
				VISCA_set_auto_exp_mode(&sonyIface, &sonyCamera, VISCA_AUTO_EXP_SHUTTER_PRIORITY);
				VISCA_get_shutter_value(&sonyIface, &sonyCamera, &shutterValue);
				bufLen = sprintf(buf, "%sauto exp: shutterPriority mode\tshutterValue= 0x%02X\r\n",
						vt100_lineX[4], shutterValue);
				serial_write_str(&debug, buf, bufLen);
				break;
			case 'u':
				/* shutter value up */
				VISCA_set_shutter_up(&sonyIface, &sonyCamera);
				VISCA_get_shutter_value(&sonyIface, &sonyCamera, &shutterValue);
				bufLen = sprintf(buf, "%sshutterUP Value= 0x%02X\r\n", vt100_lineX[4],
						shutterValue);
				serial_write_str(&debug, buf, bufLen);
				break;
			case 'd':
				/* shutter value down */
				VISCA_set_shutter_down(&sonyIface, &sonyCamera);
				VISCA_get_shutter_value(&sonyIface, &sonyCamera, &shutterValue);
				bufLen = sprintf(buf, "%sshutterDown Value= 0x%02X\r\n", vt100_lineX[4],
						shutterValue);
				serial_write_str(&debug, buf, bufLen);
				break;
			case 'r':
				/* shutter value reset */
				VISCA_set_shutter_reset(&sonyIface, &sonyCamera);
				VISCA_get_shutter_value(&sonyIface, &sonyCamera, &shutterValue);
				bufLen = sprintf(buf, "%sRESET shutterValue= 0x%02X\r\n", vt100_lineX[4],
						shutterValue);
				serial_write_str(&debug, buf, bufLen);
				break;
			case 'v':
				if (VISCA_get_shutter_value(&sonyIface, &sonyCamera, &shutterValue) == VISCA_SUCCESS)
					bufLen = sprintf(buf, "%sshutterValue= 0x%02X\r\n", vt100_lineX[4],
							shutterValue);
				else
					bufLen = sprintf(buf, "%sfailed to get shutterValue\r\n", vt100_lineX[4]);
				serial_write_str(&debug, buf, bufLen);
				break;
			default:
				break;
			}
		}
#endif	//if DEBUG_CAM==1

		/* TODO END LOOP*/
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
	hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
	hiwdg.Init.Reload = 125;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 921600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

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
	huart2.Init.BaudRate = 19200;
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

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
	huart6.Init.BaudRate = 9600;
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
	HAL_GPIO_WritePin(LRF_EN_GPIO_Port, LRF_EN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CAM_SELECT_Pin | THERMAL_DE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(THERMAL_PWR_EN_GPIO_Port, THERMAL_PWR_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
	 PC1 PC2 PC3 PC5
	 PC8 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1
			| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA5 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LRF_EN_Pin */
	GPIO_InitStruct.Pin = LRF_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LRF_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10
	 PB12 PB13 PB14 PB4
	 PB5 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_12
			| GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : CAM_SELECT_Pin LED_BUILTIN_Pin */
	GPIO_InitStruct.Pin = CAM_SELECT_Pin | LED_BUILTIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RESV_RXD_Pin RESV_TXD_Pin */
	GPIO_InitStruct.Pin = RESV_RXD_Pin | RESV_TXD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : THERMAL_PWR_EN_Pin */
	GPIO_InitStruct.Pin = THERMAL_PWR_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(THERMAL_PWR_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : IMU_RXD_Pin */
	GPIO_InitStruct.Pin = IMU_RXD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
	HAL_GPIO_Init(IMU_RXD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : IMU_TXD_Pin */
	GPIO_InitStruct.Pin = IMU_TXD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
	HAL_GPIO_Init(IMU_TXD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : THERMAL_DE_Pin */
	GPIO_InitStruct.Pin = THERMAL_DE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(THERMAL_DE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Begin of Functions Declaration*/

static void camHandler()
{
	uint32_t millis = HAL_GetTick();
	uint32_t _u32;
	static uint32_t sendTimer = 1000;
#if DEBUG_CAM==1
	const uint8_t startLineDebug = 3;
#endif	//if DEBUG_CAM==1
	static uint8_t firstTimeOnly = 0;

	if (!firstTimeOnly) {
		if (millis >= 10000) {

			if (camData.camState == CAM_ST_SONY && camData.zoomValue == 0) {
				VISCA_set_focus_auto(&sonyIface, &sonyCamera, VISCA_FOCUS_AUTO_ON);
				VISCA_set_zoom_value(&sonyIface, &sonyCamera, 0x2000);

				VISCA_set_auto_exp_mode(&sonyIface, &sonyCamera, VISCA_AUTO_EXP_SHUTTER_PRIORITY);
				VISCA_set_shutter_value(&sonyIface, &sonyCamera, 0x08);
			}

			firstTimeOnly = 1;
		}
	}

	if (thermalRetState == HAL_UART_RETURN_TX_DONE)
		HAL_GPIO_WritePin(THERMAL_DE_GPIO_Port, THERMAL_DE_Pin, GPIO_PIN_RESET);

	/* CAMERA SELECT */
	if (bitRead(canRecvButton.data[1], 0)) {
		if (camData.camState != CAM_ST_THERMAL) {
			camData.camState = CAM_ST_THERMAL;
			camData.zoomState = 0;
			camData.focusState = 0;
			camData.zoomValue = 0;
			camData.focusValue = 0;
			HAL_GPIO_WritePin(THERMAL_PWR_EN_GPIO_Port, THERMAL_PWR_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CAM_SELECT_GPIO_Port, CAM_SELECT_Pin, GPIO_PIN_SET);
#if DEBUG_CAM==1
			bufLen = sprintf(buf, "%sCAM= THERMAL", vt100_lineX[startLineDebug + 0]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1

		}
	}
	else {
		if (camData.camState != CAM_ST_SONY) {
			camData.camState = CAM_ST_SONY;
			camData.zoomState = 0;
			camData.focusState = 0;
			camData.zoomValue = 0;
			camData.focusValue = 0;
			HAL_GPIO_WritePin(THERMAL_PWR_EN_GPIO_Port, THERMAL_PWR_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CAM_SELECT_GPIO_Port, CAM_SELECT_Pin, GPIO_PIN_RESET);

			VISCA_set_focus_auto(&sonyIface, &sonyCamera, VISCA_FOCUS_AUTO_ON);
			VISCA_set_zoom_value(&sonyIface, &sonyCamera, 0x2000);

			VISCA_set_auto_exp_mode(&sonyIface, &sonyCamera, VISCA_AUTO_EXP_SHUTTER_PRIORITY);
			VISCA_set_shutter_value(&sonyIface, &sonyCamera, 0x08);

//			VISCA_set_aperture_value(&sonyIface, &sonyCamera, 15);
//			VISCA_set_cam_stabilizer(&sonyIface, &sonyCamera, VISCA_CAM_STABILIZER_ON);
#if DEBUG_CAM==1
			bufLen = sprintf(buf, "%sCAM= SONY", vt100_lineX[startLineDebug + 0]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
		}
	}

	/* ZOOM */
	if (bitRead(canRecvButton.data[1], ZOOM_IN_bit)) {
		if (camData.zoomState != zf_plus) {
			camData.zoomState = zf_plus;

			/* SONY */
			if (camData.camState == CAM_ST_SONY) {
				if (camData.zoomValue < 3) {
					camData.zoomValue++;

					/* send zoom command to sony */
					if (camData.zoomValue == 1)
						_u32 = 0x4000;
					else if (camData.zoomValue == 2)
						_u32 = 0x6000;
					else if (camData.zoomValue == 3)
						_u32 = 0x7000;

					VISCA_set_focus_auto(&sonyIface, &sonyCamera, VISCA_FOCUS_AUTO_ON);
					VISCA_set_zoom_value(&sonyIface, &sonyCamera, _u32);
#if DEBUG_CAM==1
					bufLen = sprintf(buf, "%szoomValue= %d", vt100_lineX[startLineDebug + 1],
							camData.zoomValue);
					serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
				}
			}
			/* THERMAL */
			else if (camData.camState == CAM_ST_THERMAL) {
				HAL_GPIO_WritePin(THERMAL_DE_GPIO_Port, THERMAL_DE_Pin, GPIO_PIN_SET);
				serial_write_str(&thermal, (char *) zoomMax, 7);
#if DEBUG_CAM==1
				bufLen = sprintf(buf, "%sZOOM IN", vt100_lineX[startLineDebug + 1]);
				serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
			}

		}  //if(camData.zoomState!=zf_plus){
	}	//if(bitRead(canRecvButton.data[1],ZOOM_IN_bit)){
	else if (bitRead(canRecvButton.data[1], ZOOM_OUT_bit)) {
		if (camData.zoomState != zf_min) {
			camData.zoomState = zf_min;

			/* SONY */
			if (camData.camState == CAM_ST_SONY) {
				if (camData.zoomValue > 0) {
					camData.zoomValue--;

					/* send zoom command to sony */
					if (camData.zoomValue == 0)
						_u32 = 0x2000;
					else if (camData.zoomValue == 1)
						_u32 = 0x4000;
					else if (camData.zoomValue == 2)
						_u32 = 0x6000;

					VISCA_set_focus_auto(&sonyIface, &sonyCamera, VISCA_FOCUS_AUTO_ON);
					VISCA_set_zoom_value(&sonyIface, &sonyCamera, _u32);
#if DEBUG_CAM==1
					bufLen = sprintf(buf, "%szoomValue= %d", vt100_lineX[startLineDebug + 1],
							camData.zoomValue);
					serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
				}
			}
			/* THERMAL */
			else if (camData.camState == CAM_ST_THERMAL) {
				HAL_GPIO_WritePin(THERMAL_DE_GPIO_Port, THERMAL_DE_Pin, GPIO_PIN_SET);
				serial_write_str(&thermal, (char *) zoomMin, 7);
#if DEBUG_CAM==1
				bufLen = sprintf(buf, "%sZOOM OUT", vt100_lineX[startLineDebug + 1]);
				serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
			}
		}
	}	//else if(bitRead(canRecvButton.data[1],ZOOM_OUT_bit)){
	else {
		if (camData.zoomState != zf_none) {
			camData.zoomState = zf_none;

			if (camData.camState == CAM_ST_THERMAL) {
				HAL_GPIO_WritePin(THERMAL_DE_GPIO_Port, THERMAL_DE_Pin, GPIO_PIN_SET);
				serial_write_str(&thermal, (char *) zfStop, 7);
#if DEBUG_CAM==1
				bufLen = sprintf(buf, "%sZOOM STOP", vt100_lineX[startLineDebug + 1]);
				serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
			}
		}
	}

	/* FOCUS */
	if (bitRead(canRecvButton.data[1], FOCUS_FAR_bit)) {
		if (camData.focusState != zf_plus) {
			camData.focusState = zf_plus;

			/* SONY */
			if (camData.camState == CAM_ST_SONY) {
				VISCA_set_focus_auto(&sonyIface, &sonyCamera, VISCA_FOCUS_AUTO_OFF);
				VISCA_set_focus_far_speed(&sonyIface, &sonyCamera,
						sonyFocusSpeedByZoom[camData.zoomValue]);
			}
			/* THERMAL */
			else if (camData.camState == CAM_ST_THERMAL) {
				HAL_GPIO_WritePin(THERMAL_DE_GPIO_Port, THERMAL_DE_Pin, GPIO_PIN_SET);
				serial_write_str(&thermal, (char *) focusMax, 7);
			}
#if DEBUG_CAM==1
			bufLen = sprintf(buf, "%sFOCUS FAR", vt100_lineX[startLineDebug + 1]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
		}
	}
	else if (bitRead(canRecvButton.data[1], FOCUS_NEAR_bit)) {
		if (camData.focusState != zf_min) {
			camData.focusState = zf_min;

			/* SONY */
			if (camData.camState == CAM_ST_SONY) {
				VISCA_set_focus_auto(&sonyIface, &sonyCamera, VISCA_FOCUS_AUTO_OFF);
				VISCA_set_focus_near_speed(&sonyIface, &sonyCamera,
						sonyFocusSpeedByZoom[camData.zoomValue]);
			}
			/* THERMAL */
			else if (camData.camState == CAM_ST_THERMAL) {
				HAL_GPIO_WritePin(THERMAL_DE_GPIO_Port, THERMAL_DE_Pin, GPIO_PIN_SET);
				serial_write_str(&thermal, (char *) focusMin, 7);
			}
#if DEBUG_CAM==1
			bufLen = sprintf(buf, "%sFOCUS NEAR", vt100_lineX[startLineDebug + 1]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
		}
	}
	else {
		if (camData.focusState != zf_none) {
			camData.focusState = zf_none;

			/* SONY */
			if (camData.camState == CAM_ST_SONY)
				VISCA_set_focus_stop(&sonyIface, &sonyCamera);
			/* THERMAL */
			else if (camData.camState == CAM_ST_THERMAL) {
				HAL_GPIO_WritePin(THERMAL_DE_GPIO_Port, THERMAL_DE_Pin, GPIO_PIN_SET);
				serial_write_str(&thermal, (char *) zfStop, 7);
			}
#if DEBUG_CAM==1
			bufLen = sprintf(buf, "%sFOCUS STOP", vt100_lineX[startLineDebug + 1]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_CAM==1
		}

	}	// end FOCUS

//	/* APERTURE */
//	if (bitRead(canRecvButton.data[1], APERTURE_UP_bit)) {
//		if (camData.apertureState != AP_ST_UP) {
//			camData.apertureState = AP_ST_UP;
//
//			if (camData.camState == CAM_ST_SONY) {
////				VISCA_set_aperture_value(&sonyIface,&sonyCamera,15);
////				VISCA_set_aperture_up(&sonyIface, &sonyCamera);
//				VISCA_set_bright_up(&sonyIface, &sonyCamera);
//#if DEBUG_CAM==1
//				_i16 = 0;
//				_u16 = 0;
////				VISCA_get_aperture_value(&sonyIface, &sonyCamera, &_i16);
//				VISCA_get_bright_value(&sonyIface, &sonyCamera, &_u16);
//				bufLen = sprintf(buf, "%sAPERTURE UP, val= %d", vt100_lineX[startLineDebug + 1],
//						_u16);
//				serial_write_str(&debug, buf, bufLen);
//#endif	//if DEBUG_CAM==1
//			}
//		}
//	}
//	else if (bitRead(canRecvButton.data[1], APERTURE_DOWN_bit)) {
//		if (camData.apertureState != AP_ST_DOWN) {
//			camData.apertureState = AP_ST_DOWN;
//
//			if (camData.camState == CAM_ST_SONY) {
////				VISCA_set_aperture_value(&sonyIface,&sonyCamera,0);
////				VISCA_set_aperture_down(&sonyIface, &sonyCamera);
//				VISCA_set_bright_down(&sonyIface, &sonyCamera);
//#if DEBUG_CAM==1
//				_i16 = 0;
//				_u16 = 0;
////				VISCA_get_aperture_value(&sonyIface, &sonyCamera, &_i16);
//				VISCA_get_bright_value(&sonyIface, &sonyCamera, &_u16);
//				bufLen = sprintf(buf, "%sAPERTURE DOWN, val= %d", vt100_lineX[startLineDebug + 1],
//						_u16);
//				serial_write_str(&debug, buf, bufLen);
//#endif	//if DEBUG_CAM==1
//			}
//		}
//	}
//	else {
//		camData.apertureState = AP_ST_NONE;
//	}	// end of APERTURE

	/* BRIGHTNESS */
	if (bitRead(canRecvButton.data[1], BRIGHTNESS_UP_bit)) {
		if (camData.brightnessState != BR_ST_UP) {
			camData.brightnessState = BR_ST_UP;

			if (camData.camState == CAM_ST_SONY) {
//				VISCA_set_bright_up(&sonyIface, &sonyCamera);
				VISCA_set_auto_exp_mode(&sonyIface, &sonyCamera, VISCA_AUTO_EXP_SHUTTER_PRIORITY);
//				VISCA_set_shutter_value(&sonyIface, &sonyCamera, 0x08);
				VISCA_set_shutter_up(&sonyIface, &sonyCamera);
			}
		}
	}
	else if (bitRead(canRecvButton.data[1], BRIGHTNESS_DOWN_bit)) {
		if (camData.brightnessState != BR_ST_DOWN) {
			camData.brightnessState = BR_ST_DOWN;

			if (camData.camState == CAM_ST_SONY) {
//				VISCA_set_bright_down(&sonyIface, &sonyCamera);
				VISCA_set_auto_exp_mode(&sonyIface, &sonyCamera, VISCA_AUTO_EXP_SHUTTER_PRIORITY);
//				VISCA_set_shutter_value(&sonyIface, &sonyCamera, 0x08);
				VISCA_set_shutter_down(&sonyIface, &sonyCamera);
			}
		}
	}
	else {
		camData.brightnessState = BR_ST_NONE;
	}	// end of BRIGHTNESS

	if (millis >= sendTimer) {
		sendTimer = millis + 500;

		if (camData.camState == CAM_ST_THERMAL)
			canSendOptCam.data[0] = 1;
		else
			canSendOptCam.data[0] = (camData.zoomValue << 1) & 0b1110;

		can1TxHeader.StdId = CAN_ID_RWS_OPT_CAM;
		memcpy(can1TxBuffer, canSendOptCam.data, canSendOptCam.size);
		can1TxHeader.DLC = canSendOptCam.size;

		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) != HAL_OK)
			;

	}
}

static void lrfPointerOn()
{
	serial_write(&lrf, 0xC5);
	serial_write(&lrf, 0x02);
	serial_write(&lrf, 0x97);
}

static void lrfConfig(uint32_t baud)
{
	uint8_t errorCounter = 0;
	HAL_UART_DeInit(lrf.huart);

	huart2.Instance = USART2;
	huart2.Init.BaudRate = baud;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//	if (HAL_UART_Init(&huart2) != HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
//	}
	while (HAL_UART_Init(&huart2) != HAL_OK) {
		if (++errorCounter >= 10)
			break;
		HAL_Delay(1);
	}

	serial_init(&lrf);
	lrf.TBufferRx->head = lrf.TBufferRx->tail = 0;
}

static void lrfInit(uint8_t step)
{
	memset(lrfData.buf, 0, LRF_BUFSIZE);

	if (step == 0) {
		lrfConfig(19200);
		HAL_GPIO_WritePin(LRF_EN_GPIO_Port, LRF_EN_Pin, GPIO_PIN_SET);
	}
	else if (step == 1) {
		lrfConfig(115200);
		lrfData.state = LRF_STATE_STANDBY;
	}
}

static void lrfHandler()
{
	uint32_t millis = HAL_GetTick();
	static uint32_t sendTimer = 1000;
	static uint32_t lrfTimer = 0;
	static uint8_t bufPointer = 0;
	char c;
#if DEBUG_LRF==1
	uint8_t startDebugLine = 10;
#endif	//if DEBUG_LRF==1
	static uint8_t pointerSetOn = 0;

	switch (lrfData.state)
	{
	case LRF_STATE_STANDBY:
		/* LRF_EN button is ON */
		if (bitRead(canRecvButton.data[2], LRF_CMD_ENABLE_bit)) {
			/* enable LRF */
			lrfInit(0);
			lrfTimer = millis + 500;
			lrfData.state = LRF_STATE_POWER_ON;
		}

		if (bitRead(canRecvButton.data[2], LRF_CMD_POINTER_bit)) {
			if (pointerSetOn == 0) {
				pointerSetOn = 1;
				lrfPointerOn();
#if DEBUG_LRF==1
				bufLen = sprintf(buf, "%sPointer ON", vt100_lineX[startDebugLine + 0]);
				serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_LRF==1
			}
		}
		else
			pointerSetOn = 0;

		break;
	case LRF_STATE_POWER_ON:
		if (lrfTimer > 0 && millis >= lrfTimer) {
			/* TODO check lrf availibility*/

			lrfInit(1);
			lrfTimer = 0;
			lrfData.state = LRF_STATE_READY;
			/* set pointer ON */
			lrfPointerOn();
#if DEBUG_LRF==1
			bufLen = sprintf(buf, "%sPower ON", vt100_lineX[startDebugLine + 0]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_LRF==1
		}
		break;
	case LRF_STATE_READY:
		/* LRF_START button is ON */
		if (bitRead(canRecvButton.data[2], LRF_CMD_START_bit)) {
			/* send 1 measurement command */
			memset(lrfData.buf, 0, LRF_BUFSIZE);
			bufPointer = 0;
			/* clear RX buffer */
			lrf.TBufferRx->head = lrf.TBufferRx->tail = 0;
			serial_write(&lrf, 0x12);
			serial_write(&lrf, 0x42);
			lrfTimer = millis + 5000;
			lrfData.state = LRF_STATE_1_MEASUREMENT;
#if DEBUG_LRF==1
			bufLen = sprintf(buf, "%sStart Measuring", vt100_lineX[startDebugLine + 0]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_LRF==1
		}
		/* LRF_EN button is OFF */
		if (!bitRead(canRecvButton.data[2], LRF_CMD_ENABLE_bit)) {
#if DEBUG_LRF==1
			bufLen = sprintf(buf, "%sPower Off", vt100_lineX[startDebugLine + 0]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_LRF==1
			lrfData.state = LRF_STATE_POWER_OFF;
		}
		break;
	case LRF_STATE_1_MEASUREMENT:
		/* LRF_EN button is OFF */
		if (!bitRead(canRecvButton.data[2], LRF_CMD_ENABLE_bit)) {
			lrfTimer = 0;
			lrfData.state = LRF_STATE_POWER_OFF;
		}

		/* timeout */
		if (lrfTimer > 0 && millis >= lrfTimer) {
			lrfTimer = 0;
			lrfData.state = LRF_STATE_POWER_OFF;
		}

		if (serial_available(&lrf)) {
			c = serial_read(&lrf);

#if DEBUG_LRF==1
			if (c != 0) {
				bufLen = sprintf(buf, "%02X ", (uint8_t) c);
				serial_write_str(&debug, buf, bufLen);
			}
#endif	//if DEBUG_LRF==1

			if (bufPointer == 0) {
				if (c == 0x59)
					lrfData.buf[bufPointer++] = (uint8_t) c;
			}
			else
				lrfData.buf[bufPointer++] = (uint8_t) c;

			if (bufPointer >= 10) {
				if (lrfData.buf[0] == 0x59 && lrfData.buf[1] == 0x12) {
					if ((lrfData.buf[8] & 0x3C) == 0) {
						lrfData.distance = (uint16_t) lrfData.buf[3] << 8 | lrfData.buf[2];
						lrfData.distance /= 2;
#if DEBUG_LRF==1
						bufLen = sprintf(buf, "%s%d\t d= %4dm", vt100_lineX[startDebugLine + 1],
								lrfData.validCounter, lrfData.distance);
						serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_LRF==1
					}
					else {
						lrfData.distance = (lrfData.buf[8] >> 2) & 0xF;
#if DEBUG_LRF==1
						bufLen = sprintf(buf, "%sError, status byte= 0x%X",
								vt100_lineX[startDebugLine + 1], lrfData.buf[8]);
						serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_LRF==1
					}
					lrfData.validCounter++;
				}	//if(lrfData.buf[0]==0x59 && lrfData.buf[1]==0x12){
				lrfTimer = millis + 500;
				lrfData.state = LRF_STATE_DELAY_MEASUREMENT;
			}	//if(bufPointer>=10){
		}
		break;
	case LRF_STATE_DELAY_MEASUREMENT:
		if (lrfTimer > 0 && millis >= lrfTimer) {
			lrfTimer = 0;
			lrfData.state = LRF_STATE_READY;
		}
		break;
	case LRF_STATE_CONT_MEASUREMENT:
		break;
	case LRF_STATE_POWER_OFF:
		/* disable LRF_EN */
		HAL_GPIO_WritePin(LRF_EN_GPIO_Port, LRF_EN_Pin, GPIO_PIN_SET);
		lrfData.state = LRF_STATE_STANDBY;
		break;
	}

	if (millis >= sendTimer) {
		sendTimer = millis + 500;

		canSendOptLrf.data[0] = lrfData.validCounter;
		canSendOptLrf.data[1] = lrfData.distance & 0xFF;
		canSendOptLrf.data[2] = (lrfData.distance >> 8) & 0xFF;

		can1TxHeader.StdId = CAN_ID_RWS_OPT_LRF;
		memcpy(can1TxBuffer, canSendOptLrf.data, canSendOptLrf.size);
		can1TxHeader.DLC = canSendOptLrf.size;

		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) != HAL_OK)
			;
	}
}

static void canHandler()
{
	uint32_t millis = HAL_GetTick();
#if DEBUG_BUS==1
	const uint8_t startLineDebug = 2;
#endif	//if DEBUG_BUS==1
	static uint32_t recvButtonTimer = 0;

	/* Receive Data from Optronik - CAM */
	if (canRecvButton.state) {
		canRecvButton.state = false;
		recvButtonTimer = millis + CAN_RECEIVE_TIMEOUT;
		canRecvButton.online = 1;

#if DEBUG_BUS==1
		bufLen = sprintf(buf, "%sRecv Button: ", vt100_lineX[startLineDebug + 0]);
		serial_write_str(&debug, buf, bufLen);
		for ( int i = 0; i < canRecvButton.size; i++ ) {
			bufLen = sprintf(buf, "0x%02X ", canRecvButton.data[i]);
			serial_write_str(&debug, buf, bufLen);
		}
#endif	//if DEBUG_BUS==1
	}

	if (recvButtonTimer > 0 && millis >= recvButtonTimer) {
		recvButtonTimer = 0;
		canRecvButton.online = 0;

		memset(canRecvButton.data, 0, canRecvButton.size);
#if DEBUG_BUS==1
		bufLen = sprintf(buf, "%sno Data", vt100_lineX[startLineDebug + 0]);
		serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_BUS==1

	}
}

static void CAN_Config()
{
	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_BUTTON << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (CAN_ID_RWS_BUTTON << 5);
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
		/* Notification Error */
		Error_Handler();
	}

	/*##-5- Configure Transmission process #####################################*/
	can1TxHeader.StdId = 0x123;
	can1TxHeader.ExtId = 0x01;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.DLC = CAN_BUFSIZE;
	can1TxHeader.TransmitGlobalTime = DISABLE;

#if USE_CAN2
	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = (0x100 << 5);
	sFilterConfig.FilterIdLow = (0x100 << 5);
	sFilterConfig.FilterMaskIdHigh = (0x700 << 5);
	sFilterConfig.FilterMaskIdLow = (0x700 << 5);
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}

	/*##-5- Configure Transmission process #####################################*/
	can2TxHeader.StdId = 0x21;
	can2TxHeader.ExtId = 0x01;
	can2TxHeader.RTR = CAN_RTR_DATA;
	can2TxHeader.IDE = CAN_ID_STD;
	can2TxHeader.DLC = CAN_BUFSIZE;
	can2TxHeader.TransmitGlobalTime = DISABLE;

#endif	//if USE_CAN2

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
		if (_id == CAN_ID_RWS_BUTTON) {
			canRecvButton.state = true;
			memcpy(canRecvButton.data, can1RxBuffer, canRecvButton.size);
		}
	}
}

#if DEBUG==1
void UART4_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}
#endif	//if DEBUG==1

void USART6_IRQHandler(void)
{
	sonyRetState = USARTx_IRQHandler(&sony);
}

void USART3_IRQHandler(void)
{
	thermalRetState = USARTx_IRQHandler(&thermal);
}

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&lrf);
}

/* TODO End of Functions Declarations*/
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

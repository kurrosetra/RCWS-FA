/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "rwsCanID.h"
#include "stm_hal_serial.h"
#include "libVisca.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Global Variables*/

#define DEBUG					1
#if DEBUG==1
#	define DEBUG_BUS			1
#	define DEBUG_CAM			1
#	define DEBUG_LRF			1
#	define DEBUG_IMU			0
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

Ring_Buffer_t tx5Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx5Buffer = { { 0 }, 0, 0 };
TSerial imu = { &rx5Buffer, &tx5Buffer, &huart5 };

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
uint8_t sonyFocusSpeedByZoom[4] = { SONY_FOCUS_SPEED_4, SONY_FOCUS_SPEED_2, SONY_FOCUS_SPEED_1,
		SONY_FOCUS_SPEED_1 };

typedef enum
{
	ZOOM_IN_bit = 1,
	ZOOM_OUT_bit = 2,
	FOCUS_FAR_bit = 3,
	FOCUS_NEAR_bit = 4
} ZoomFocus_bitMasking;

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

typedef struct
{
	uint8_t camState;
	uint8_t zoomState;
	uint8_t focusState;
	uint8_t zoomValue;
	uint8_t focusValue;
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
	LRF_CMD_PONITER_BIT = 3
} LRF_CommandTypeDef;

typedef struct
{
	uint8_t state;
	uint8_t validCounter;
	uint16_t distance;
	uint8_t buf[LRF_BUFSIZE];
} LRF_HandleTypeDef;
LRF_HandleTypeDef lrfData = { LRF_STATE_STANDBY, 0, 0, { 0 } };

char imuBuf[UART_BUFSIZE];
typedef struct
{
	int16_t yaw;
	float pitch;
	int16_t roll;
	uint32_t timeout;
} TImuData;
TImuData imuData = { 0, 0, 0, 100 };

#define CAN_BUFSIZE							8

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

//TCanRecvBuffer canRecvPanel = { CAN_ID_RWS_PANEL, false, { 0 }, 7, 0};
TCanRecvBuffer canRecvButton = { CAN_ID_RWS_BUTTON, false, { 0 }, 3, 0 };
//TCanRecvBuffer canRecvMotor = { CAN_ID_RWS_MOTOR, false, { 0 }, 2, 0 };
//TCanRecvBuffer canRecvOptLrf = { CAN_ID_RWS_OPT_LRF, false, { 0 }, 3, 0 };
//TCanRecvBuffer canRecvOptImu = { CAN_ID_RWS_OPT_IMU, false, { 0 }, 8, 0 };
//TCanRecvBuffer canRecvOptCam = { CAN_ID_RWS_OPT_CAM, false, { 0 }, 1, 0 };

//TCanSendBuffer canSendMotorCommand = { CAN_ID_RWS_PNL_MTR, { 0 }, 7 };
//TCanSendBuffer canSendButton = { CAN_ID_RWS_BUTTON, { 0 }, 3 };
//TCanSendBuffer canSendMotor = { CAN_ID_RWS_MOTOR, { 0 }, 2 };
TCanSendBuffer canSendOptLrf = { CAN_ID_RWS_OPT_LRF, { 0 }, 3 };
TCanSendBuffer canSendOptImu = { CAN_ID_RWS_OPT_IMU, { 0 }, 8 };
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
static void MX_UART5_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static char** str_split(char* a_str, const char a_delim);

static void CAN_Config();
static void canHandler(uint32_t millis);
static void camHandler(uint32_t millis);
static void lrfHandler(uint32_t millis);
static void imuHandler(uint32_t millis);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_UART5_Init();
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
	serial_init(&imu);
	serial_init(&lrf);

	/*##-1- Configure the CAN peripheral #######################################*/
	CAN_Config();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t millis;
#if DEBUG==1
	uint32_t ledTimer = 1000;
#endif	//if DEBUG==1

	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		millis = HAL_GetTick();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/

		canHandler(millis);
		imuHandler(millis);
		lrfHandler(millis);
		camHandler(millis);

#if DEBUG==1
		if (millis >= ledTimer) {
			ledTimer = millis + 500;

			if (canRecvButton.online)
				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
			else
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		}
#endif	//if DEBUG==1

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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
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
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 45;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
	hiwdg.Init.Reload = 750;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 921600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 PA9   ------> USART1_TX
 PA10   ------> USART1_RX
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

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
	 PC1 PC2 PC3 PC4
	 PC5 PC8 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1
			| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA5 PA7 PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LRF_EN_Pin */
	GPIO_InitStruct.Pin = LRF_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LRF_EN_GPIO_Port, &GPIO_InitStruct);

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

	/*Configure GPIO pin : THERMAL_DE_Pin */
	GPIO_InitStruct.Pin = THERMAL_DE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(THERMAL_DE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Begin of Functions Declaration*/
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

static void camHandler(uint32_t millis)
{
	uint32_t _u32;
	static uint32_t sendTimer = 1000;
#if DEBUG_CAM==1
	const uint8_t startLineDebug = 3;
#endif	//if DEBUG_CAM==1

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
				VISCA_set_focus_far_speed(&sonyIface, &sonyCamera,
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

	}

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
	HAL_UART_DeInit(lrf.huart);

	huart2.Instance = USART2;
	huart2.Init.BaudRate = baud;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
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

static void lrfHandler(uint32_t millis)
{
	static uint32_t sendTimer = 1000;
	static uint32_t lrfTimer = 0;
	static uint8_t bufPointer = 0;
	char c;
#if DEBUG_LRF==1
	uint8_t startDebugLine = 7;
#endif	//if DEBUG_LRF==1

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
		break;
	case LRF_STATE_POWER_ON:
		if (lrfTimer > 0 && millis >= lrfTimer) {
			/* TODO check lrf availibility*/

			lrfInit(1);
			lrfTimer = 0;
			lrfData.state = LRF_STATE_READY;
			/* set pointer ON */
			lrfPointerOn();
		}
		break;
	case LRF_STATE_READY:
		/* LRF_START button is ON */
		if (bitRead(canRecvButton.data[2], LRF_CMD_START_bit)) {
			/* send 1 measurement command */
			memset(lrfData.buf, 0, LRF_BUFSIZE);
			bufPointer = 0;
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

static void imuHandler(uint32_t millis)
{
	static uint32_t imuTimer = 0;
	char c;
	bool dataUpdate = false;
	char *s;
	char **tokens;
	union
	{
		float f;
		uint8_t bytes[4];
	} _pitch;
#if DEBUG_IMU==1
	uint8_t startDebugLine = 5;
#endif	//if DEBUG_IMU==1

	if (serial_available(&imu)) {
		c = serial_read(&imu);
		if (c == '$')
			memset(imuBuf, 0, UART_BUFSIZE);
		else if (c == '*')
			dataUpdate = true;

		strncat(imuBuf, (const char *) &c, 1);
	}

	if (dataUpdate) {
		/* $VNYMR,+141.058,+002.072,-179.441,... */

		s = strstr(imuBuf, "$VNYMR,");
		if (s) {
#if DEBUG_IMU==1
			bufLen = sprintf(buf, "%s%s", vt100_lineX[startDebugLine], s);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_IMU==1
			tokens = str_split(imuBuf, ',');
			if (tokens) {
				for ( int i = 0; *(tokens + i); i++ ) {
					s = *(tokens + i);

					switch (i)
					{
					case 1:
						imuData.yaw = (int16_t) (atof(s) * 100);
						break;
					case 2:
						imuData.pitch = atof(s);
						break;
					case 3:
						imuData.roll = (int16_t) (atof(s) * 100.0f);
						break;
					}
					free(*(tokens + i));
					if (i >= 3)
						break;
				}	//for ( int i = 0; *(tokens + i); i++ ) {
				free(tokens);
			}	//if (tokens) {
		} /*if (s) {*/
#if DEBUG_IMU==1
		bufLen = sprintf(buf, "%sypr= %d %.3f %d", vt100_lineX[startDebugLine + 3], imuData.yaw,
				imuData.pitch, imuData.roll);
		serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_IMU==1
	} /*if (dataUpdate) {*/

	if (millis >= imuTimer) {
		imuTimer = millis + imuData.timeout;

		_pitch.f = imuData.pitch;

		canSendOptImu.data[0] = imuData.yaw & 0xFF;
		canSendOptImu.data[1] = (imuData.yaw >> 8) & 0xFF;

		canSendOptImu.data[2] = _pitch.bytes[0];
		canSendOptImu.data[3] = _pitch.bytes[1];
		canSendOptImu.data[4] = _pitch.bytes[2];
		canSendOptImu.data[5] = _pitch.bytes[3];

		canSendOptImu.data[6] = imuData.roll & 0xFF;
		canSendOptImu.data[7] = (imuData.roll >> 8) & 0xFF;

		can1TxHeader.StdId = CAN_ID_RWS_OPT_IMU;
		memcpy(can1TxBuffer, canSendOptImu.data, canSendOptImu.size);
		can1TxHeader.DLC = canSendOptImu.size;

		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) != HAL_OK)
			;

	}
}

static void canHandler(uint32_t millis)
{
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
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}
//	/* filter can id = CAN_ID_RWS_BUTTON= 0x320 */
//	sFilterConfig.FilterBank = 0;
//	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = (CAN_ID_RWS_BUTTON << 5);
//	sFilterConfig.FilterIdLow = 0;
//	sFilterConfig.FilterMaskIdHigh = (0x7FF << 5);
//	sFilterConfig.FilterMaskIdLow = 0;
//	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//	sFilterConfig.FilterActivation = ENABLE;
//	sFilterConfig.SlaveStartFilterBank = 14;
//
//	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
//		/* filter configuration error */
//		Error_Handler();
//	}

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

void UART5_IRQHandler(void)
{
	USARTx_IRQHandler(&imu);
}

/* TODO End of Functions Declarations*/
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

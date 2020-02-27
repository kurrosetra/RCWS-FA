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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>

#include "rwsCanID.h"
#include "stm_hal_serial.h"
#include "Ingenia_SerialServoDrive.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Global Variables*/
#define DEBUG					1

#define MOTOR_PAN_ENABLE		1
#define MOTOR_TILT_ENABLE		1

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define MOTOR_MAX_SPEED			128000
//#define MOTOR_POS_REV_VAL		2000
#define MOTOR_UPDATE_TIMEOUT	10

#ifdef RING_BUFFER_SIZE
#define UART_BUFSIZE			RING_BUFFER_SIZE
#else
#define UART_BUFSIZE			256
#endif

#if DEBUG
Ring_Buffer_t tx1Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx1Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx1Buffer, &tx1Buffer, &huart1 };
#endif	//if DEBUG

Ring_Buffer_t tx2Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx2Buffer = { { 0 }, 0, 0 };
TSerial serial2 = { &rx2Buffer, &tx2Buffer, &huart2 };

Ring_Buffer_t tx3Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx3Buffer = { { 0 }, 0, 0 };
TSerial serial3 = { &rx3Buffer, &tx3Buffer, &huart3 };

#if DEBUG
char debugStr[UART_BUFSIZE];
char vt100_home[16];
char vt100_line1[16];
char vt100_line2[16];
char vt100_line3[16];
char vt100_line4[16];
char vt100_line5[16];
char vt100_line6[16];
char vt100_line7[16];
#endif	//if DEBUG

TServo azimuth;
TServo elevation;

uint32_t commandTimestamp = 0;
volatile bool trigger = false;
volatile int32_t panelCommand[2];
volatile bool cockStart = false;
int32_t motorCommand[2];

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
	TServo *servo;
	uint32_t prevSpeed;
	uint8_t stopCounter;
	bool direction;
} TServoState;
TServoState pan;
TServoState tilt;

#define CAN_BUFSIZE							8

//CAN_TxHeaderTypeDef can1TxHeader;
//CAN_RxHeaderTypeDef can1RxHeader;
//uint32_t can1TxMailBox;
//uint8_t can1TxBuffer[CAN_BUFSIZE];
//uint8_t can1RxBuffer[CAN_BUFSIZE];
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

TCanRecvBuffer canRecvPanel = { CAN_ID_RWS_PNL_MTR, false, { 0 }, 7, 0 };
TCanRecvBuffer canRecvButton = { CAN_ID_RWS_BUTTON, false, { 0 }, 3, 0 };
//TCanRecvBuffer canRecvMotor = { CAN_ID_RWS_MOTOR, false, { 0 }, 2, 0 };
//TCanRecvBuffer canRecvOptLrf = { CAN_ID_RWS_OPT_LRF, false, { 0 }, 3, 0 };
//TCanRecvBuffer canRecvOptImu = { CAN_ID_RWS_OPT_IMU, false, { 0 }, 8, 0 };
//TCanRecvBuffer canRecvOptCam = { CAN_ID_RWS_OPT_CAM, false, { 0 }, 1, 0 };

//TCanSendBuffer canSendMotorCommand = { CAN_ID_RWS_PNL_MTR, { 0 }, 7 };
//TCanSendBuffer canSendButton = { CAN_ID_RWS_BUTTON, { 0 }, 3 };
TCanSendBuffer canSendMotor = { CAN_ID_RWS_MOTOR, { 0 }, 2 };
//TCanSendBuffer canSendOptLrf = { CAN_ID_RWS_OPT_LRF, { 0 }, 3 };
//TCanSendBuffer canSendOptImu = { CAN_ID_RWS_OPT_IMU, { 0 }, 8 };
//TCanSendBuffer canSendOptCam = { CAN_ID_RWS_OPT_CAM, { 0 }, 1 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* TODO Functions Prototypes*/

static void delayIwdg(uint32_t _delayTime);
static void CAN_Config(void);
static void busCommandParsing();
static void motorInit();
static void motorStop(TServoState *motor);
static void motorSpeed(TServoState *motor, int32_t spd);

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

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART1_UART_Init();
	MX_CAN_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Initialization*/
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
#if DEBUG
	sprintf(vt100_home, "\x1b[2J\x1b[H");
	sprintf(vt100_line1, "\x1b[1;0H\x1b[2K");
	sprintf(vt100_line2, "\x1b[2;0H\x1b[2K");
	sprintf(vt100_line3, "\x1b[3;0H\x1b[2K");
	sprintf(vt100_line4, "\x1b[4;0H\x1b[2K");
	sprintf(vt100_line5, "\x1b[5;0H\x1b[2K");
	sprintf(vt100_line6, "\x1b[6;0H\x1b[2K");
	sprintf(vt100_line7, "\x1b[7;0H\x1b[2K");

	sprintf(debugStr, "%sMotor Controller Test Suite!\r\n", vt100_home);
	serial_write_str(&debug, debugStr, strlen(debugStr));
#endif	//if DEBUG

	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

#if DEBUG
	serial_init(&debug);
#endif	//if DEBUG

	motorInit();

	CAN_Config();

#if DEBUG
	sprintf(debugStr, "%sMotor Controller Firmware!\r\n===================\r\n", vt100_line1);
	serial_write_str(&debug, debugStr, strlen(debugStr));
	sprintf(debugStr, "%sCAN bus Ready!\r\n", vt100_line3);
	serial_write_str(&debug, debugStr, strlen(debugStr));
#endif	//if DEBUG

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t millis = 0;
	char buf[UART_BUFSIZE];
	uint16_t bufLen;
	uint32_t sendTimer = 0;
	uint8_t canCounter = 0;
//	uint32_t motorPanTimer = HAL_GetTick() + 1000;
	uint32_t motorPanTimer = HAL_GetTick() + 1000;
	uint32_t motorTiltTimer = HAL_GetTick() + 1000;
#if DEBUG
	char c;
	int32_t position = 0;
	uint32_t debugTimer = 0;
	uint32_t stampTimer = 0;
	uint32_t deltaTimeStamp = 0;
#endif	//if DEBUG
	uint32_t buttonCommandTimestamp = 0;

	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		millis = HAL_GetTick();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO Begin Loop*/

		if (millis >= buttonCommandTimestamp) {
			HAL_GPIO_WritePin(FIRING_EN_GPIO_Port, FIRING_EN_Pin, GPIO_PIN_RESET);
			buttonCommandTimestamp = 0;
		}

		if (canRecvButton.state) {
			canRecvButton.state = false;

			buttonCommandTimestamp = millis + 500;

			HAL_GPIO_WritePin(FIRING_EN_GPIO_Port, FIRING_EN_Pin,
					bitRead(canRecvButton.data[0], 0));

			cockStart = bitRead(canRecvButton.data[0], 1);
#if DEBUG==1
			bufLen = sprintf(buf, "%sRecvButton= 0x%X", vt100_line5, canRecvButton.data[0]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1

		}

		if (canRecvPanel.state) {
			canRecvPanel.state = false;
			commandTimestamp = millis + 250;
			busCommandParsing();

			/* save from volatile to global */
			motorCommand[0] = panelCommand[0];
			motorCommand[1] = panelCommand[1];

#if DEBUG
			deltaTimeStamp = millis - stampTimer;
			stampTimer = millis;
			bufLen = sprintf(buf, "%s%d\tms%snew command= %d & %d", vt100_line3,
					(int) deltaTimeStamp, vt100_line4, (int) motorCommand[0],
					(int) motorCommand[1]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG
		}

		/* no available bus command */
		if (commandTimestamp > 0) {
			if (millis > commandTimestamp) {
				commandTimestamp = 0;

				pan.enable = false;
				motorCommand[0] = 0;
				tilt.enable = false;
				motorCommand[1] = 0;
			}
		}	//if (commandTimestamp > 0)
		else {
			pan.enable = false;
			motorCommand[0] = 0;
			tilt.enable = false;
			motorCommand[1] = 0;
		}

//		if (millis >= motorPanTimer) {
//			if (abs(motorCommand[0]) > 6400)
//				motorPanTimer = millis + 50;
//			else
//				motorPanTimer = millis + MOTOR_UPDATE_TIMEOUT;
//
//#if MOTOR_PAN_ENABLE
//			/* azimuth command */
//			motorSpeed(&pan, motorCommand[0]);
//#endif	//if MOTOR_PAN_ENABLE
//
//#if MOTOR_TILT_ENABLE
//			/* elevation command */
//			motorSpeed(&tilt, motorCommand[1]);
//#endif	//if MOTOR_TILT_ENABLE
//
//		}
		if (millis >= motorPanTimer) {
			if (abs(motorCommand[0]) > MOTOR_MAX_SPEED / 2)
				motorPanTimer = millis + 100;
			else
				motorPanTimer = millis + MOTOR_UPDATE_TIMEOUT;

#if MOTOR_PAN_ENABLE
			/* azimuth command */
			motorSpeed(&pan, motorCommand[0]);
#endif	//if MOTOR_PAN_ENABLE
		}
		if (millis >= motorTiltTimer) {
			motorTiltTimer = millis + MOTOR_UPDATE_TIMEOUT;

#if MOTOR_TILT_ENABLE
			/* elevation command */
			motorSpeed(&tilt, motorCommand[1]);
#endif	//if MOTOR_TILT_ENABLE
		}

		if (millis >= sendTimer) {
			sendTimer = millis + 100;

			bitWrite(canSendMotor.data[0], 0, pan.enable);
			bitWrite(canSendMotor.data[0], 1, tilt.enable);
			bitWrite(canSendMotor.data[1], 0, HAL_GPIO_ReadPin(FIRING_EN_GPIO_Port,FIRING_EN_Pin));
			bitWrite(canSendMotor.data[1], 1, cockStart);

			memcpy(hcan.pTxMsg->Data, canSendMotor.data, canSendMotor.size);
			hcan.pTxMsg->DLC = canSendMotor.size;
			hcan.pTxMsg->StdId = CAN_ID_RWS_MOTOR;
			HAL_CAN_Transmit_IT(&hcan);

			if (commandTimestamp > 0)
				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		}

	}
	/* TODO End Loop*/
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

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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

/* CAN init function */
static void MX_CAN_Init(void)
{

	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 9;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SJW = CAN_SJW_1TQ;
	hcan.Init.BS1 = CAN_BS1_13TQ;
	hcan.Init.BS2 = CAN_BS2_2TQ;
	hcan.Init.TTCM = DISABLE;
	hcan.Init.ABOM = DISABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.NART = DISABLE;
	hcan.Init.RFLM = DISABLE;
	hcan.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 625;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
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
	huart3.Init.BaudRate = 115200;
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

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(FIRING_EN_GPIO_Port, FIRING_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_BUILTIN_Pin | COCK_POS_Pin | COCK_AWO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(COCK_DIR_GPIO_Port, COCK_DIR_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : PC13 PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA4 PA5 PA6
	 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : FIRING_EN_Pin COCK_DIR_Pin */
	GPIO_InitStruct.Pin = FIRING_EN_Pin | COCK_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LIM_AZ_Pin */
	GPIO_InitStruct.Pin = LIM_AZ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(LIM_AZ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LIM_EL_UP_Pin LIM_EL_DOWN_Pin */
	GPIO_InitStruct.Pin = LIM_EL_UP_Pin | LIM_EL_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB2 PB12 PB3 PB4
	 PB5 PB6 PB7 PB8
	 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5
			| GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_BUILTIN_Pin */
	GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : COCK_POS_Pin COCK_AWO_Pin */
	GPIO_InitStruct.Pin = COCK_POS_Pin | COCK_AWO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Functions Declarations*/
static void delayIwdg(uint32_t _delayTime)
{
	const uint32_t iwdg_reset_time = 100;

	HAL_IWDG_Refresh(&hiwdg);
	if (_delayTime <= iwdg_reset_time)
		HAL_Delay(_delayTime);
	else {
		while (_delayTime > iwdg_reset_time) {
			HAL_Delay(iwdg_reset_time);
			HAL_IWDG_Refresh(&hiwdg);
			_delayTime -= iwdg_reset_time;
		}
		HAL_Delay(_delayTime);
	}
	HAL_IWDG_Refresh(&hiwdg);
}

/**
 * @brief  Configures the CAN.
 * @param  None
 * @retval None
 */
static void CAN_Config(void)
{

	CAN_FilterConfTypeDef sFilterConfig;
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;

	/*##-1- Configure the CAN peripheral #######################################*/
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;

	/*##-2- Configure the CAN Filter ###########################################*/
//	sFilterConfig.FilterNumber = 0;
//	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = 0;
//	sFilterConfig.FilterIdLow = 0;
//	sFilterConfig.FilterMaskIdHigh = 0;
//	sFilterConfig.FilterMaskIdLow = 0;
//	sFilterConfig.FilterFIFOAssignment = 0;
//	sFilterConfig.FilterActivation = ENABLE;
//	sFilterConfig.BankNumber = 14;
//
//	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
//		/* Filter configuration Error */
//		Error_Handler();
//	}
	/* filter can id = CAN_ID_RWS_PNL_MTR= 0x310 */
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = CAN_ID_RWS_PNL_MTR << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_BUTTON= 0x320 */
	sFilterConfig.FilterNumber = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = CAN_ID_RWS_BUTTON << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Configure Transmission process #####################################*/
	hcan.pTxMsg->StdId = CAN_ID_RWS_MOTOR;			// Standard identifier is 0
	hcan.pTxMsg->ExtId = 0x01;			// Setting the extension identifier (29 bits)
	hcan.pTxMsg->RTR = CAN_RTR_DATA;		// Message types for the data frame, 8 bits in a frame
	hcan.pTxMsg->IDE = CAN_ID_STD;		// Using an standard identifier
	hcan.pTxMsg->DLC = 8;

	for ( int i = 0; i < 8; ++i ) {
		hcan.pTxMsg->Data[i] = 0;
	}

	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
		__HAL_CAN_ENABLE_IT(&hcan, (CAN_IT_FF0 | CAN_IT_FMP0));
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	uint32_t _id = hcan->pRxMsg->StdId;

	if (_id == CAN_ID_RWS_PNL_MTR) {
		canRecvPanel.state = true;
		memcpy(canRecvPanel.data, hcan->pRxMsg->Data, canRecvPanel.size);
	}
	else if (_id == CAN_ID_RWS_BUTTON) {
		canRecvButton.state = true;
		memcpy(canRecvButton.data, hcan->pRxMsg->Data, canRecvButton.size);
	}

	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
		__HAL_CAN_ENABLE_IT(hcan, (CAN_IT_FF0 | CAN_IT_FMP0));
}

#if DEBUG
void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}
#endif	//if DEBUG

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&serial2);
}

void USART3_IRQHandler(void)
{
	USARTx_IRQHandler(&serial3);
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

	/* TODO COCKING HANDLER*/
	/* TODO TRIGGER HANDLER*/

	/* PAN motor */
	if (bitRead(canRecvPanel.data[0], 0))
		pan.enable = true;

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
	if (bitRead(canRecvPanel.data[0], 1))
		tilt.enable = true;

	if (tilt.enable) {
		spd = ((int32_t) canRecvPanel.data[6] << 16) | ((int32_t) canRecvPanel.data[5] << 8)
				| canRecvPanel.data[4];

		if (bitRead(canRecvPanel.data[0], 3))
			spd = 0 - spd;

		panelCommand[1] = spd;
	}
	else
		panelCommand[1] = 0;
}

static void motorInit()
{
#if MOTOR_PAN_ENABLE || MOTOR_TILT_ENABLE
	EStateMachineStatus motorStatus = STATUS_NOT_READY_TO_SWITCH_ON;
#endif	//if MOTOR_PAN_ENABLE || MOTOR_TILT_ENABLE

#if MOTOR_PAN_ENABLE
	azimuth.serial = &serial2;
	azimuth._u8Node = 0x20;
	azimuth._isBinaryEnabled = false;
	azimuth._isInitialAngleDeterminationProcessFinished = true;

	pan.name = SERVO_PAN;
	pan.enable = false;
	pan.direction = DIR_UP_RIGHT;
	pan.servo = &azimuth;
	pan.prevSpeed = 1;
	pan.stopCounter = 100;

	serial_init(pan.servo->serial);
#endif	//if MOTOR_PAN_ENABLE

#if MOTOR_TILT_ENABLE
	elevation.serial = &serial3;
	elevation._u8Node = 0x20;
	elevation._isBinaryEnabled = false;
	elevation._isInitialAngleDeterminationProcessFinished = true;

	tilt.name = SERVO_TILT;
	tilt.enable = false;
	tilt.direction = DIR_UP_RIGHT;
	tilt.servo = &elevation;
	tilt.prevSpeed = 1;
	tilt.stopCounter = 100;

	serial_init(tilt.servo->serial);
#endif	//if MOTOR_TILT_ENABLE

#if MOTOR_PAN_ENABLE
	HAL_IWDG_Refresh(&hiwdg);
	motorStatus = STATUS_NOT_READY_TO_SWITCH_ON;
	while (motorStatus != STATUS_OPERATION_ENABLED) {
		Ingenia_setModeOfOperation(pan.servo, DRIVE_MODE_PROFILE_POSITION);
		Ingenia_enableMotor(pan.servo);
		Ingenia_getStateMachineStatus(pan.servo, &motorStatus);
	}
	HAL_IWDG_Refresh(&hiwdg);

	motorSpeed(&pan, 0);
	/* set max motor speed */
	Ingenia_write_i32(pan.servo, 0x6080, 0, MOTOR_MAX_SPEED);
	/* set max velocity profile */
	Ingenia_write_i32(pan.servo, 0x607F, 0, MOTOR_MAX_SPEED);

#if DEBUG
	sprintf(debugStr, "PAN in Profile Position Mode!\r\n");
	serial_write_str(&debug, debugStr, strlen(debugStr));
#endif	//if DEBUG

	HAL_IWDG_Refresh(&hiwdg);
#endif	//if MOTOR_PAN_ENABLE

#if MOTOR_TILT_ENABLE
	HAL_IWDG_Refresh(&hiwdg);
	motorStatus = STATUS_NOT_READY_TO_SWITCH_ON;
	while (motorStatus != STATUS_OPERATION_ENABLED) {
		Ingenia_setModeOfOperation(tilt.servo, DRIVE_MODE_PROFILE_POSITION);
		Ingenia_enableMotor(tilt.servo);
		Ingenia_getStateMachineStatus(tilt.servo, &motorStatus);
	}
	HAL_IWDG_Refresh(&hiwdg);

	motorSpeed(&tilt, 0);
	/* set max motor speed */
	Ingenia_write_i32(pan.servo, 0x6080, 0, MOTOR_MAX_SPEED);
	/* set max velocity profile */
	Ingenia_write_i32(tilt.servo, 0x607F, 0, MOTOR_MAX_SPEED);
	/* set minimum position limit */
//	Ingenia_write_i32(tilt.servo, 0x607D, 1, -12500);
	Ingenia_write_i32(tilt.servo, 0x607D, 1, -50000);
	/* set maximum position limit */
	Ingenia_write_i32(tilt.servo, 0x607D, 2, 50000);

#if DEBUG
	sprintf(debugStr, "TILT in Profile Position Mode!\r\n");
	serial_write_str(&debug, debugStr, strlen(debugStr));
#endif	//if DEBUG

	HAL_IWDG_Refresh(&hiwdg);
#endif	//if MOTOR_TILT_ENABLE

	/* LED DEBUG */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
#if DEBUG && (MOTOR_PAN_ENABLE || MOTOR_TILT_ENABLE)
	sprintf(debugStr, "Motor Controller Ready!\r\n");
	serial_write_str(&debug, debugStr, strlen(debugStr));
#endif	//if DEBUG

}

static void motorStop(TServoState *motor)
{
	int32_t _pos = 0;

	/* halt */
	Ingenia_setTargetPositionAdv(motor->servo, 0, true, true, true);
	if (motor->prevSpeed != 0) {
		Ingenia_disableMotor(motor->servo);
		delayIwdg(1);
		Ingenia_enableMotor(motor->servo);
	}

	/* reset demand position */
	_pos = Ingenia_getActualPosition(motor->servo);
	Ingenia_setTargetPosition(motor->servo, _pos);
}

static void motorSpeed(TServoState *motor, int32_t spd)
{
	uint32_t _spd = abs(spd);
	int32_t _posRelative = 0;
	int32_t _pos = 0;

	if (_spd > 0) {

		/* set maximum speed */
		if (_spd > MOTOR_MAX_SPEED)
			_spd = MOTOR_MAX_SPEED;

		_posRelative = ((_spd / 1000) * MOTOR_UPDATE_TIMEOUT) + 10;
		if (_spd > 6400)
			_posRelative *= 10;
		else if (_spd > 64000)
			_posRelative *= 50;

		/* set speed */
		if (motor->prevSpeed != _spd)
			Ingenia_write_i32(motor->servo, 0x6081, 0, _spd);

		/* update direction & pos*/
		if (spd < 0) {
			_pos -= _posRelative;
			/* if change direction */
			if (motor->direction == DIR_UP_RIGHT)
				motorStop(motor);
		}
		else {
			_pos += _posRelative;
			/* if change direction */
			if (motor->direction == DIR_DOWN_LEFT)
				motorStop(motor);
		}

		/* set position */
		Ingenia_setTargetPositionAdv(motor->servo, _pos, true, true, false);
		motor->stopCounter = 100;
	}	//if (_spd > 0)
	else {
		/* if speed stop */
		if (motor->prevSpeed > 0 || motor->stopCounter++ > 20) {
			motor->stopCounter = 0;
			motorStop(motor);
		}
	}	//if (_spd > 0) .. else ..

	/* update direction */
	if (spd >= 0)
		motor->direction = DIR_UP_RIGHT;
	else
		motor->direction = DIR_DOWN_LEFT;

	/* save current speed */
	motor->prevSpeed = _spd;
}

/* TODO END Function Declarations*/
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

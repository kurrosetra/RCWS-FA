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
#include "stm_hal_serial.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Private Variables*/
#define DEBUG					0
#if DEBUG==1
#	define JOYSTICK_DEBUG		0
#	define BUTTON_DEBUG			0
#	define CMD_DEBUG			0
#endif	//if DEBUG==1

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define UART_BUFSIZE	RING_BUFFER_SIZE

uint16_t bufLen = 0;
char buf[128];

Ring_Buffer_t tx1Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx1Buffer = { { 0 }, 0, 0 };
TSerial panel = { &rx1Buffer, &tx1Buffer, &huart1 };
#define CMD_BUFSIZE			32
char cmdIn[CMD_BUFSIZE];

typedef struct
{
	uint8_t id;
	uint8_t dtab;
	volatile uint16_t azimuth;
	volatile uint16_t elevation;
} Joystick_t;
Joystick_t jRight = { 0, 0, 0, 0 };
Joystick_t jLeft = { 1, 0, 0, 0 };

volatile uint16_t adcValArray[4];
uint32_t buttonState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* TODO Private Function Prototypes*/
static void jsHandler();
static void btnHandler();
static void cmdHandler();

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	/* TODO Initialization*/
	HAL_IWDG_Init(&hiwdg);
	HAL_IWDG_Refresh(&hiwdg);
#if DEBUG==1
	bufLen = sprintf(buf, "Button - Panel RWS firmware!\r\n");
	serial_write_str(&panel, buf, bufLen);
#endif	//if DEBUG==1

	serial_init(&panel);

	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcValArray, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* TODO Private Variables*/
	uint32_t millis = 0;
	uint32_t ledTimer = 1000;
	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		millis = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/
		if (millis >= ledTimer) {
			ledTimer = millis + 50;

#if JOYSTICK_DEBUG==1
			bufLen = sprintf(buf, "jsR:\tdTAB:%d%d%d%d\t%d %d\r\n", bitRead(jRight.dtab, 0),
					bitRead(jRight.dtab, 1), bitRead(jRight.dtab, 2), bitRead(jRight.dtab, 3),
					jRight.azimuth, jRight.elevation);
			serial_write_str(&panel, buf, bufLen);
			bufLen = sprintf(buf, "jsL:\tdTAB:%d%d%d%d\t%d %d\r\n", bitRead(jLeft.dtab, 0),
					bitRead(jLeft.dtab, 1), bitRead(jLeft.dtab, 2), bitRead(jLeft.dtab, 3),
					jLeft.azimuth, jLeft.elevation);
			serial_write_str(&panel, buf, bufLen);
#endif	//if JOYSTICK_DEBUG==1

#if BUTTON_DEBUG==1
			bufLen = sprintf(buf, "buttonState= 0b");
			serial_write_str(&panel, buf, bufLen);
			for ( int i = 0; i < 16; i++ )
			serial_write(&panel, bitRead(buttonState,i) + '0');
			serial_write_str(&panel, "\r\n", 2);
#endif	//if BUTTON_DEBUG==1

#if DEBUG==0
			bufLen = sprintf(buf, "$BTN,%d,%d,%d,%d,%d,%d,%d*\r\n", buttonState, jRight.dtab,
					jRight.azimuth, jRight.elevation, jLeft.dtab, jLeft.azimuth, jLeft.elevation);
			serial_write_str(&panel, buf, bufLen);
#endif	//if DEBUG==0

		}

		jsHandler();
		btnHandler();
		cmdHandler();

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hiwdg.Init.Reload = 312;
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
  htim3.Init.Prescaler = 35;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_SEL_DAY_Pin */
  GPIO_InitStruct.Pin = CAM_SEL_DAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CAM_SEL_DAY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_FOCUS_FAR_Pin TRK_NEXT_Pin */
  GPIO_InitStruct.Pin = CAM_FOCUS_FAR_Pin|TRK_NEXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TRK_ON_Pin FIRE_ACTIVE_Pin TRK_PREV_Pin CAM_FOCUS_NEAR_Pin
                           CAM_SEL_THERMAL_Pin JSR_DEADMAN_Pin JSL_TRIG_Pin CAM_ZOOM_OUT_Pin */
  GPIO_InitStruct.Pin = TRK_ON_Pin|FIRE_ACTIVE_Pin|TRK_PREV_Pin|CAM_FOCUS_NEAR_Pin
                          |CAM_SEL_THERMAL_Pin|JSR_DEADMAN_Pin|JSL_TRIG_Pin|CAM_ZOOM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : JSR_A_Pin JSR_B_Pin JSR_TRIG_Pin JSL_DEADMAN_Pin
                           JSL_A_Pin JSL_B_Pin LRF_MAN_DOWN_Pin LRF_START_Pin
                           MTR_SPD_HIGH_Pin MTR_SPD_LOW_Pin LRF_EN_Pin LRF_MAN_UP_Pin
                           CAM_ZOOM_IN_Pin */
  GPIO_InitStruct.Pin = JSR_A_Pin|JSR_B_Pin|JSR_TRIG_Pin|JSL_DEADMAN_Pin
                          |JSL_A_Pin|JSL_B_Pin|LRF_MAN_DOWN_Pin|LRF_START_Pin
                          |MTR_SPD_HIGH_Pin|MTR_SPD_LOW_Pin|LRF_EN_Pin|LRF_MAN_UP_Pin
                          |CAM_ZOOM_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

}

/* USER CODE BEGIN 4 */
/* TODO Function Declaration*/
static void jsHandler()
{
	bitWrite(jRight.dtab, 0, HAL_GPIO_ReadPin(JSR_DEADMAN_GPIO_Port, JSR_DEADMAN_Pin));
	bitWrite(jRight.dtab, 1, HAL_GPIO_ReadPin(JSR_TRIG_GPIO_Port, JSR_TRIG_Pin));
	bitWrite(jRight.dtab, 2, HAL_GPIO_ReadPin(JSR_A_GPIO_Port,JSR_A_Pin));
	bitWrite(jRight.dtab, 3, HAL_GPIO_ReadPin(JSR_B_GPIO_Port,JSR_B_Pin));

	bitWrite(jLeft.dtab, 0, !HAL_GPIO_ReadPin(JSL_DEADMAN_GPIO_Port, JSL_DEADMAN_Pin));
	bitWrite(jLeft.dtab, 1, !HAL_GPIO_ReadPin(JSL_TRIG_GPIO_Port, JSL_TRIG_Pin));
	bitWrite(jLeft.dtab, 2, !HAL_GPIO_ReadPin(JSL_A_GPIO_Port,JSL_A_Pin));
	bitWrite(jLeft.dtab, 3, !HAL_GPIO_ReadPin(JSL_B_GPIO_Port,JSL_B_Pin));
}

static void btnHandler()
{
	bitWrite(buttonState, 0, !HAL_GPIO_ReadPin(MTR_SPD_LOW_GPIO_Port,MTR_SPD_LOW_Pin));
	bitWrite(buttonState, 1, !HAL_GPIO_ReadPin(MTR_SPD_HIGH_GPIO_Port,MTR_SPD_HIGH_Pin));

	bitWrite(buttonState, 2, !HAL_GPIO_ReadPin(LRF_EN_GPIO_Port,LRF_EN_Pin));
	bitWrite(buttonState, 3, !HAL_GPIO_ReadPin(LRF_START_GPIO_Port,LRF_START_Pin));
	bitWrite(buttonState, 4, !HAL_GPIO_ReadPin(LRF_MAN_UP_GPIO_Port,LRF_MAN_UP_Pin));
	bitWrite(buttonState, 5, !HAL_GPIO_ReadPin(LRF_MAN_DOWN_GPIO_Port,LRF_MAN_DOWN_Pin));

	bitWrite(buttonState, 6, !HAL_GPIO_ReadPin(CAM_ZOOM_IN_GPIO_Port,CAM_ZOOM_IN_Pin));
	bitWrite(buttonState, 7, !HAL_GPIO_ReadPin(CAM_ZOOM_OUT_GPIO_Port,CAM_ZOOM_OUT_Pin));
	bitWrite(buttonState, 8, !HAL_GPIO_ReadPin(CAM_SEL_DAY_GPIO_Port,CAM_SEL_DAY_Pin));
	bitWrite(buttonState, 9, !HAL_GPIO_ReadPin(CAM_SEL_THERMAL_GPIO_Port,CAM_SEL_THERMAL_Pin));
	bitWrite(buttonState, 10, !HAL_GPIO_ReadPin(CAM_FOCUS_FAR_GPIO_Port,CAM_FOCUS_FAR_Pin));
	bitWrite(buttonState, 11, !HAL_GPIO_ReadPin(CAM_FOCUS_NEAR_GPIO_Port,CAM_FOCUS_NEAR_Pin));

	bitWrite(buttonState, 12, !HAL_GPIO_ReadPin(TRK_ON_GPIO_Port,TRK_ON_Pin));
	bitWrite(buttonState, 13, !HAL_GPIO_ReadPin(TRK_NEXT_GPIO_Port,TRK_NEXT_Pin));
	bitWrite(buttonState, 14, !HAL_GPIO_ReadPin(TRK_PREV_GPIO_Port,TRK_PREV_Pin));

	bitWrite(buttonState, 15, !HAL_GPIO_ReadPin(FIRE_ACTIVE_GPIO_Port,FIRE_ACTIVE_Pin));

}

static void cmdHandler()
{
	bool cmdCompleted = false;
	char c;
	uint8_t ledState = 0;

	if (serial_available(&panel)) {
		c = serial_read(&panel);

		if (c == '$')
			memset(cmdIn, 0, CMD_BUFSIZE);
		else if (c == '*')
			cmdCompleted = true;

		strncat(cmdIn, (const char *) &c, 1);
	}

	if (cmdCompleted) {
#if CMD_DEBUG==1
		bufLen = sprintf(buf, "len=%d\t%s\r\n", strlen(cmdIn), cmdIn);
		serial_write_str(&panel, buf, bufLen);
#endif	//if CMD_DEBUG==1

		if (strncmp(cmdIn, "$LED,", 5) == 0 && strlen(cmdIn) == 7) {
			if (cmdIn[5] >= '0' && cmdIn[5] <= '9') {
				ledState = (uint8_t) (cmdIn[5] - '0');

				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, bitRead(ledState, 0));
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, bitRead(ledState, 1));
			}
		}
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}

void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(&panel);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	jRight.azimuth = ((jRight.azimuth * 9) + (adcValArray[0] * 1)) / 10;
	jRight.elevation = ((jRight.elevation * 9) + (adcValArray[1] * 1)) / 10;
	jLeft.azimuth = ((jLeft.azimuth * 9) + (adcValArray[2] * 1)) / 10;
	jLeft.elevation = ((jLeft.elevation * 9) + (adcValArray[3] * 1)) / 10;
}

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

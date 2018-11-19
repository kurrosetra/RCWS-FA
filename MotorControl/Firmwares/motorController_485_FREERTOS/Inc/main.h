/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define TIM_GPIO_PSC 71
#define TIM_GPIO_ON 1000
#define TIM_GPIO_OFF 0

#define MTR_AZ_ZERO_Pin GPIO_PIN_1
#define MTR_AZ_ZERO_GPIO_Port GPIOA
#define MTR_AZ_ZERO_EXTI_IRQn EXTI1_IRQn
#define MTR_AZ_TXD_Pin GPIO_PIN_2
#define MTR_AZ_TXD_GPIO_Port GPIOA
#define MTR_AZ_RXD_Pin GPIO_PIN_3
#define MTR_AZ_RXD_GPIO_Port GPIOA
#define TRIGGER_ENABLE_Pin GPIO_PIN_6
#define TRIGGER_ENABLE_GPIO_Port GPIOA
#define COCK_POS_Pin GPIO_PIN_0
#define COCK_POS_GPIO_Port GPIOB
#define LED_BUILTIN_Pin GPIO_PIN_1
#define LED_BUILTIN_GPIO_Port GPIOB
#define MTR_EL_ZERO_Pin GPIO_PIN_2
#define MTR_EL_ZERO_GPIO_Port GPIOB
#define MTR_EL_ZERO_EXTI_IRQn EXTI2_IRQn
#define MTR_EL_TXD_Pin GPIO_PIN_10
#define MTR_EL_TXD_GPIO_Port GPIOB
#define MTR_EL_RXD_Pin GPIO_PIN_11
#define MTR_EL_RXD_GPIO_Port GPIOB
#define COCK_AWO_Pin GPIO_PIN_15
#define COCK_AWO_GPIO_Port GPIOB
#define COCK_DIR_Pin GPIO_PIN_8
#define COCK_DIR_GPIO_Port GPIOA
#define COCK_PLS_Pin GPIO_PIN_9
#define COCK_PLS_GPIO_Port GPIOA
#define COCK_ALM_Pin GPIO_PIN_10
#define COCK_ALM_GPIO_Port GPIOA
#define BUS_RXD_Pin GPIO_PIN_11
#define BUS_RXD_GPIO_Port GPIOA
#define BUS_TXD_Pin GPIO_PIN_12
#define BUS_TXD_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

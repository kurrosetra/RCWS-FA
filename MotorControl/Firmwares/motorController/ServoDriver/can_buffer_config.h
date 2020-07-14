/*
 * can_buffer_config.h
 *
 *  Created on: Feb 14, 2020
 *      Author: miftakur
 */

#ifndef CAN_BUFFER_CONFIG_H_
#define CAN_BUFFER_CONFIG_H_

/* TODO add define for each stm32fxx here */
#if defined(STM32F103xB) && defined(USE_HAL_DRIVER)
#include "stm32f1xx.h"
#endif

#if defined(STM32F446xx) && defined(USE_HAL_DRIVER)
#include "stm32f4xx.h"
#endif	//if STM32F446xx

#define CAN_RX_BUFFER_SIZE	5
#define CAN_DATA_MAX		8

#endif /* CAN_BUFFER_CONFIG_H_ */

/*
 * config.h
 *
 *  Created on: Nov 24, 2018
 *      Author: miftakur
 */

#ifndef RING_BUFFER_CONFIG_H_
#define RING_BUFFER_CONFIG_H_

/* TODO add define for each stm32fxx here */
#if defined(STM32F103xB) && defined(USE_HAL_DRIVER)
#include "stm32f1xx.h"
#endif

#if defined(STM32F446xx) && defined(USE_HAL_DRIVER)
#include "stm32f4xx.h"
#endif	//if STM32F446xx

#define RING_BUFFER_SIZE	1024

#endif /* RING_BUFFER_CONFIG_H_ */

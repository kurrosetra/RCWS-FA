/*
 * stabControl.h
 *
 *  Created on: Feb 10, 2021
 *      Author: miftakur
 */

#ifndef STABCONTROL_H_
#define STABCONTROL_H_

/* TODO add define for each stm32fxx here */
#if defined(STM32F103xB) && defined(USE_HAL_DRIVER)
#include "stm32f1xx.h"
#endif

#if defined(STM32F446xx) && defined(USE_HAL_DRIVER)
#include "stm32f4xx.h"
#endif	//if STM32F446xx

void stab_calculate(int mode, float pitch, float roll, float q_az, float q_ev, float qd_az, float qd_ev, float *sp1, float *sp2);
void stab_reset();
void stab_updateSetpoint(float dq_az, float dq_ev);


#endif /* STABCONTROL_H_ */

/*
 * can_buffer.h
 *
 *  Created on: Feb 14, 2020
 *      Author: miftakur
 */

#ifndef CAN_BUFFER_H_
#define CAN_BUFFER_H_

#include "can_buffer_config.h"

typedef struct
{
	CAN_RxHeaderTypeDef canRxHeader;
	uint8_t rxData[8];
} CAN_Data_t;

typedef struct
{
	CAN_Data_t buf[CAN_RX_BUFFER_SIZE];
	volatile uint8_t tail;
	volatile uint8_t head;
} CAN_Buffer_t;

uint8_t can_buffer_available(CAN_Buffer_t *buffer);
void can_buffer_write(CAN_Buffer_t *buffer, CAN_Data_t *data);
int8_t can_buffer_peek(CAN_Buffer_t *buffer, CAN_Data_t *data);
int8_t can_buffer_read(CAN_Buffer_t *buffer, CAN_Data_t *data);
void can_buffer_flush(CAN_Buffer_t *buffer);

#endif /* CAN_BUFFER_H_ */

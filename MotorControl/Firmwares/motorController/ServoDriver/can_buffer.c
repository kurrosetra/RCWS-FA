/*
 * can_buffer.c
 *
 *  Created on: Feb 14, 2020
 *      Author: miftakur
 */

#include "can_buffer.h"
#include <string.h>

uint8_t can_buffer_available(CAN_Buffer_t *buffer)
{
	if (buffer->head != buffer->tail)
		return (buffer->head >= buffer->tail) ?
				(buffer->head - buffer->tail) : (buffer->head + CAN_RX_BUFFER_SIZE - buffer->tail);

	return 0;
}

void can_buffer_write(CAN_Buffer_t *buffer, CAN_Data_t *data)
{
	uint8_t head = (buffer->head + 1) % CAN_RX_BUFFER_SIZE;

	if (head != buffer->tail) {
		buffer->buf[buffer->head].canRxHeader = data->canRxHeader;
		memset(buffer->buf[buffer->head].rxData, 0, CAN_DATA_MAX);
		memcpy(buffer->buf[buffer->head].rxData, data->rxData, data->canRxHeader.DLC);

		buffer->head = head;
	}
}

int8_t can_buffer_peek(CAN_Buffer_t *buffer, CAN_Data_t *data)
{
	if (buffer->head != buffer->tail) {
		*data = buffer->buf[buffer->tail];

		return can_buffer_available(buffer);
	}

	return -1;
}

int8_t can_buffer_read(CAN_Buffer_t *buffer, CAN_Data_t *data)
{
	uint8_t ret = can_buffer_peek(buffer, data);
	if (ret > 0)
		buffer->tail = (buffer->tail + 1) % CAN_RX_BUFFER_SIZE;

	return ret;
}

void can_buffer_flush(CAN_Buffer_t *buffer)
{
	buffer->head = buffer->tail = 0;
}


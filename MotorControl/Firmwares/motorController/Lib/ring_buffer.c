/*
 * kurro_buffer.c
 *
 *  Created on: Nov 24, 2018
 *      Author: miftakur
 */

#include <ring_buffer.h>

void ring_buffer_read_str(Ring_Buffer_t *buffer, char *str)
{
	char c;

	while (buffer->head != buffer->tail) {
		c = ring_buffer_read(buffer);
		if (c == -1)
			break;
		else
			*str++ = c;
	}
}

void ring_buffer_write(Ring_Buffer_t *buffer, char c)
{
	uint16_t i = (buffer->head + 1) % RING_BUFFER_SIZE;

	if (i != buffer->tail) {
		buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}

char ring_buffer_read(Ring_Buffer_t *buffer)
{
	if (buffer->head == buffer->tail)
		return -1;
	else {
		char c = buffer->buffer[buffer->tail];
		buffer->tail = (buffer->tail + 1) % RING_BUFFER_SIZE;

		return c;
	}
}

bool ring_buffer_available(Ring_Buffer_t *buffer)
{
	return (buffer->head != buffer->tail);
}

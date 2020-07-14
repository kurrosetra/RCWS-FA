/*
 * kurro_buffer.h
 *
 *  Created on: Nov 24, 2018
 *      Author: miftakur
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <ring_buffer_config.h>
#include <stdbool.h>
#include <stdint-gcc.h>

#ifndef RING_BUFFER_SIZE
#define RING_BUFFER_SIZE	128
#endif

typedef struct
{
	char buffer[RING_BUFFER_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
} Ring_Buffer_t;

void ring_buffer_write(Ring_Buffer_t *buffer, char c);
void ring_buffer_read_str(Ring_Buffer_t *buffer, char *str);
char ring_buffer_read(Ring_Buffer_t *buffer);
bool ring_buffer_available(Ring_Buffer_t *buffer);

#endif /* RING_BUFFER_H_ */

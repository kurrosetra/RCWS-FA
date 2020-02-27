/*
 * stm_hal_serial.h
 *
 *  Created on: Nov 24, 2018
 *      Author: miftakur
 *
 *  ChangeLog:
 *  20190108: 	- add return value from IRQ handler
 *  			- add init function
 */

#ifndef STM_HAL_SERIAL_H_
#define STM_HAL_SERIAL_H_

#include "ring_buffer.h"

typedef enum {
	HAL_UART_RETURN_NONE,
	HAL_UART_RETURN_RX,
	HAL_UART_RETURN_RX_IDLE,
	HAL_UART_RETURN_TX_INGOING,
	HAL_UART_RETURN_TX_FULL,
	HAL_UART_RETURN_TX_DONE
}HAL_UART_ReturnTypeDef;


typedef struct
{
	Ring_Buffer_t *TBufferRx;
	Ring_Buffer_t *TBufferTx;
	UART_HandleTypeDef *huart;
} TSerial;

uint8_t USARTx_IRQHandler(TSerial *serial);

void serial_init(TSerial *serial);
char serial_read(TSerial *serial);
void serial_write(TSerial *serial, char c);
void serial_read_str(TSerial *serial, char *str);
void serial_write_str(TSerial *serial, char *str, uint16_t len);
bool serial_available(TSerial *serial);

#endif /* STM_HAL_SERIAL_H_ */

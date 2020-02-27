/*
 * stm_hal_serial.c
 *
 *  Created on: Nov 24, 2018
 *      Author: miftakur
 */

#include "stm_hal_serial.h"

void serial_start_transmitting(TSerial *serial);

void serial_init(TSerial *serial)
{
	__HAL_UART_ENABLE_IT(serial->huart, UART_IT_RXNE);
	/* TODO add idle line detection*/
//	__HAL_UART_ENABLE_IT(serial->huart, (UART_IT_RXNE|UART_IT_IDLE));
}

uint8_t USARTx_IRQHandler(TSerial *serial)
{
	uint32_t isrflags = READ_REG(serial->huart->Instance->SR);
	uint32_t cr1its = READ_REG(serial->huart->Instance->CR1);
	uint32_t errorflags = 0x00U;
	char c;
	uint8_t ret = HAL_UART_RETURN_NONE;

	/* If no error occurs */
	errorflags = (isrflags & (uint32_t) (USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
	if (errorflags == RESET) {
		/* UART in mode Receiver -------------------------------------------------*/
		if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
			ring_buffer_write(serial->TBufferRx, (char) (serial->huart->Instance->DR & 0xFF));
			ret = HAL_UART_RETURN_RX;
		}

		/* UART in mode Transmitter ------------------------------------------------*/
		if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
			if (serial->TBufferTx->head == serial->TBufferTx->tail) {
				//no more data available
				/* Disable the UART Transmit Complete Interrupt */
				__HAL_UART_DISABLE_IT(serial->huart, UART_IT_TXE);

				/* Enable the UART Transmit Complete Interrupt */
				__HAL_UART_ENABLE_IT(serial->huart, UART_IT_TC);
				ret = HAL_UART_RETURN_TX_FULL;
			}
			else {
				c = ring_buffer_read(serial->TBufferTx);
				serial->huart->Instance->DR = (uint8_t) c;
				ret = HAL_UART_RETURN_TX_INGOING;
			}
		}

		/* UART in mode Transmitter end --------------------------------------------*/
		if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET)) {
			/* Disable the UART Transmit Complete Interrupt */
			__HAL_UART_DISABLE_IT(serial->huart, UART_IT_TC);

			ret = HAL_UART_RETURN_TX_DONE;
		}

	}

	return ret;
}

char serial_read(TSerial *serial)
{
	return ring_buffer_read(serial->TBufferRx);
}

void serial_write(TSerial *serial, char c)
{
	ring_buffer_write(serial->TBufferTx, c);
	serial_start_transmitting(serial);
}

void serial_read_str(TSerial *serial, char *str)
{
	ring_buffer_read_str(serial->TBufferRx, str);
}

void serial_write_str(TSerial *serial, char *str, uint16_t len)
{
	for ( uint16_t i = 0; i < len; i++ )
		ring_buffer_write(serial->TBufferTx, *str++);

	serial_start_transmitting(serial);
//	uint32_t cr1its = READ_REG(serial->huart->Instance->CR1);
//	uint32_t isrflags = READ_REG(serial->huart->Instance->SR);
//	if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) == RESET)) {
//		/* Enable the UART Transmit data register empty Interrupt */
//		__HAL_UART_ENABLE_IT(serial->huart, UART_IT_TXE);
//	}
}

bool serial_available(TSerial *serial)
{
	return (serial->TBufferRx->head != serial->TBufferRx->tail);
}

void serial_start_transmitting(TSerial *serial)
{
	uint32_t cr1its = READ_REG(serial->huart->Instance->CR1);
	uint32_t isrflags = READ_REG(serial->huart->Instance->SR);

	if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) == RESET)) {
		/* Enable the UART Transmit data register empty Interrupt */
		__HAL_UART_ENABLE_IT(serial->huart, UART_IT_TXE);
	}
}

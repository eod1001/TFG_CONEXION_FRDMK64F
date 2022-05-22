/*
 * Comunicacion.h
 *
 *  Created on: 24 mar. 2022
 *      Author: kikee
 */

#ifndef COMUNICACION_H_
#define COMUNICACION_H_


void establecerVelocidad1(uint8_t vel, uart_rtos_handle_t handleUART);
void establecerVelocidad2(uint8_t vel, uart_rtos_handle_t handleUART);
void establecerAceleracion(uint8_t ace, uart_rtos_handle_t handleUART);
void establecerModo(uint8_t modo, uart_rtos_handle_t handleUART);


#endif /* COMUNICACION_H_ */

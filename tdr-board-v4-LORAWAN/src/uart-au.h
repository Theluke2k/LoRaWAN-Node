/*
 * uart.h
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#ifndef UART_AU_H_
#define UART_AU_H_
#include <inttypes.h>

void uart_init();
void uart_deinit();
void uart_write(const char* buffer, uint16_t msg_length);

void DEBUG_RESULT(const char* msg, int res, int err);
void DEBUG_MESSAGE(const char* msg);


#endif /* UART_AU_H_ */

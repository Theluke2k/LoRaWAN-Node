/*
 * uart.c
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#include "uart-au.h"

#include <adi_uart.h>
#include <drivers/general/adi_drivers_general.h>
#include <stdio.h>



// UART0
ADI_ALIGNED_PRAGMA(4) static ADI_UART_HANDLE hUart ADI_ALIGNED_ATTRIBUTE(4);
ADI_ALIGNED_PRAGMA(4) static uint8_t UartDeviceMem[ADI_UART_BIDIR_MEMORY_SIZE] ADI_ALIGNED_ATTRIBUTE(4);



void uart_init()
{
	adi_uart_Open(0u, ADI_UART_DIR_BIDIRECTION, UartDeviceMem, ADI_UART_BIDIR_MEMORY_SIZE, &hUart);
	adi_uart_SetConfiguration(hUart, ADI_UART_NO_PARITY, ADI_UART_ONE_STOPBIT, ADI_UART_WORDLEN_8BITS);
	adi_uart_ConfigBaudRate(hUart, 2, 2, 348, 3); // ~115200
//	adi_uart_RegisterCallback(hUart,UARTCallback,NULL);
}


void uart_deinit()
{
	adi_uart_Close(hUart);
}



void uart_write(const char* buffer, uint16_t msg_length)
{
	bool bTxComplete = false;
	while(bTxComplete == false)
	{
		adi_uart_IsTxComplete(hUart, &bTxComplete);
	}
	adi_uart_SubmitTxBuffer(hUart, (uint8_t*)buffer, msg_length, true);
	uint32_t delay_val = 24000; // 15ms?
	while(--delay_val){};
}


void DEBUG_RESULT(const char* msg, int res, int err)
{
	bool bTxComplete = false;
	if(res != err)
	{
		char buffer[255];
		sprintf(buffer, "ERROR || MSG: %s | RESULT: 0x%08X | EXPECTED: 0x%08X\r\n", msg, res, err);
		while(bTxComplete == false)
		{
			adi_uart_IsTxComplete(hUart, &bTxComplete);
		}
		adi_uart_SubmitTxBuffer(hUart, buffer, strlen(buffer), true);
	    uint32_t delay_val = 32000; // 20ms
	    while(--delay_val){}; //
	}
}

void DEBUG_MESSAGE(const char* msg) // max 112 chars.
{
	bool bTxComplete = false;
	char buffer[255] = {0};
	sprintf(buffer, "DEBUG || MSG: %s\r\n", msg);
	while(bTxComplete == false)
	{
		adi_uart_IsTxComplete(hUart, &bTxComplete);
	}
	adi_uart_SubmitTxBuffer(hUart, buffer, strlen(buffer), true);
    uint32_t delay_val = 32000; // 20ms
    while(--delay_val){}; //
}


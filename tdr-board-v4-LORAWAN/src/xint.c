/*
 * xint.c
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#include "xint.h"
#include <adi_xint.h>
#include "shared.h"
#include "uart-au.h"

static uint8_t xintMemory[ADI_XINT_MEMORY_SIZE];

static void UARTIntCallback(void* pCBParam, uint32_t nEvent,  void* pEventData)
{
	iHibernateExitFlag = 1;
	print_flag = 1;
}

void xint_init()
{
    ADI_XINT_RESULT eXintResult = ADI_XINT_SUCCESS;
	if(ADI_XINT_SUCCESS != (eXintResult = adi_xint_Init(xintMemory, ADI_XINT_MEMORY_SIZE)))
	{
		DEBUG_MESSAGE("adi_xint_Init failed\n");
	}

	if(ADI_XINT_SUCCESS != (eXintResult = adi_xint_RegisterCallback (ADI_XINT_EVENT_INT2, UARTIntCallback, NULL)))
	{
		DEBUG_MESSAGE("adi_xint_RegisterCallback failed\n");
	}
}

void xint_uart_enable()
{
    ADI_XINT_RESULT eXintResult = ADI_XINT_SUCCESS;
	if(ADI_XINT_SUCCESS != (eXintResult = adi_xint_EnableIRQ (ADI_XINT_EVENT_INT2, ADI_XINT_IRQ_FALLING_EDGE)))
	{
		DEBUG_MESSAGE("adi_xint_EnableExIRQ failed\n");
	}
}

void xint_uart_disable()
{
    ADI_XINT_RESULT eXintResult = ADI_XINT_SUCCESS;
	if(ADI_XINT_SUCCESS != (eXintResult = adi_xint_DisableIRQ(ADI_XINT_EVENT_INT2)))
	{
		DEBUG_MESSAGE("adi_xint_DisableExIRQ failed\n");
	}
}

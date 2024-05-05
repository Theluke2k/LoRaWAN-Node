/*
 * flash.c
 *
 *  Created on: 29. jun. 2023
 *      Author: au703540
 */
#include "flash.h"
#include "uart-au.h"
#include <adi_flash.h>



/**
 * LEAVE FOR NOW. Will get back to this at some point.
 */


static ADI_FEE_HANDLE feeDevice;
static uint8_t feeDeviceMemory[ADI_FEE_MEMORY_SIZE];

static void FlashIntCallback(void* pCBParam, uint32_t nEvent,  void* pEventData)
{
	// not sure what to do here for now
}

void flash_init()
{
	ADI_FEE_RESULT eResult;

	if((eResult = adi_fee_Open(FLASH_DEVICE_NUM, feeDeviceMemory, sizeof(feeDeviceMemory), &feeDevice)) != ADI_FEE_SUCCESS)
		DEBUG_RESULT("Failed to open FEE device", eResult, ADI_FEE_SUCCESS);

	if((eResult = adi_fee_RegisterCallback(feeDevice, FlashIntCallback, NULL)) != ADI_FEE_SUCCESS)
		DEBUG_RESULT("Failed to register callback for FEE device", eResult, ADI_FEE_SUCCESS);


}

void flash_deinit()
{
	ADI_FEE_RESULT eResult;

	if((eResult = adi_fee_Close(feeDevice)) != ADI_FEE_SUCCESS)
		DEBUG_RESULT("Failed to close FEE device", eResult, ADI_FEE_SUCCESS);
}

void flash_sleep()
{
	ADI_FEE_RESULT eResult;

	if((eResult = adi_fee_Sleep(feeDevice, true)) != ADI_FEE_SUCCESS)
		DEBUG_RESULT("Failed to put the FEE device to sleep", eResult, ADI_FEE_SUCCESS);
}

void flash_wakeup()
{
	ADI_FEE_RESULT eResult;

	if((eResult = adi_fee_Sleep(feeDevice, false)) != ADI_FEE_SUCCESS)
		DEBUG_RESULT("Failed to wake the FEE device", eResult, ADI_FEE_SUCCESS);
}

void flash_write(uint32_t* address, uint32_t* data, uint32_t num_words)
{
	ADI_FEE_RESULT eResult;

	ADI_FEE_TRANSACTION transaction;

	transaction.nSize = num_words;
	transaction.pWriteAddr = address;
	transaction.pWriteData = data;
	transaction.bUseDma = true;

	uint32_t hw_errors;

	if((eResult = adi_fee_PageErase(feeDevice, *address, (*address + 0xFFFF), &hw_errors)) != ADI_FEE_SUCCESS)
		DEBUG_RESULT("Failed to submit buffer to the FEE device", eResult, ADI_FEE_SUCCESS);

	if((eResult = adi_fee_SubmitBuffer(feeDevice, &transaction)) != ADI_FEE_SUCCESS)
		DEBUG_RESULT("Failed to submit buffer to the FEE device", eResult, ADI_FEE_SUCCESS);

	bool is_complete = false;

	while(!is_complete)
	{
		adi_fee_IsBufferAvailable(feeDevice, &is_complete);
	}
}

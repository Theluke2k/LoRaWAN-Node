/*
 * adc-board.c
 *
 *  Created on: 1. dec. 2024
 *      Author: lselm
 */

#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "adc-board.h"
#include "board-config.h"
#include "gpio-board.h"
#include "delay-board.h"

/**
 * ADC device handle. Aligned ---> Specifies the alignment of data objects in storage, which avoids performance problems with misaligned data.
 */
ADI_ALIGNED_PRAGMA(4)
static ADI_ADC_HANDLE adcDevice ADI_ALIGNED_ATTRIBUTE(4);
/**
 * ADC device memory used by the driver.
 */
ADI_ALIGNED_PRAGMA(4)
static uint8_t DeviceMemory[ADI_ADC_MEMORY_SIZE] ADI_ALIGNED_ATTRIBUTE(4);
/**
 * ADC buffer where the measurements will be stored.
 */
ADI_ALIGNED_PRAGMA(4)
static uint16_t ADC_DataBuffer[ADC_NUM_SAMPLES] ADI_ALIGNED_ATTRIBUTE(4) = {0};


/*!
 * \brief Initializes the ADC object and MCU peripheral
 *
 * \param [IN] obj      ADC object
 * \param [IN] adcInput ADC input pin
 */
void AdcMcuInit( Adc_t *obj, PinNames adcInput ) {
	GpioInit(&obj->AdcInput, adcInput, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, GPIO_MUL1);
}

void AdcMcuInitDriver( bool calibration ) {
	bool bCalibrationDone = false;

	/* Open the ADC device */
	adi_adc_Open(ADC_DEV_NUM, DeviceMemory, sizeof(DeviceMemory), &adcDevice);

	/* Set the delay time */
	adi_adc_SetDelayTime(adcDevice, 10); // 10cycles for TDR is okay

	adi_adc_SetResolution(adcDevice, ADI_ADC_RESOLUTION_12_BIT);

	/* Set ADC reference */
	adi_adc_SetVrefSource(adcDevice, ADI_ADC_VREF_SRC_EXT);

	/* Enable ADC sub system */
	adi_adc_EnableADCSubSystem(adcDevice, true);

	// 10 ms delay
	DelayMsMcu(10);

	/* Start calibration */
	if (calibration) {
		adi_adc_StartCalibration(adcDevice);

		/* Wait until calibration is done */
		while (!bCalibrationDone) {
			adi_adc_IsCalibrationDone(adcDevice, &bCalibrationDone);
		}
	}
}

void AdcMcuDeInitDriver( void ) {
	adi_adc_Close(adcDevice);
}

void AdcMcuPower( bool power ) {
	adi_adc_PowerUp (adcDevice, power);
}

/*!
 * \brief Initializes the ADC internal parameters
 */
void AdcMcuConfig( void ) {

}

/*!
 * \brief Reads the value of the given channel
 *
 * \param [IN] obj     ADC object
 * \param [IN] channel ADC input channel
 */
uint16_t AdcMcuReadChannel( Adc_t *obj, uint32_t channel ) {

}

uint16_t AdcMcuReadChannelCustom( Adc_t *obj, uint32_t channel, uint8_t acquisition_time) {
	uint16_t avg_val = 0;

	// Power on the ADC
	AdcMcuPower(true);

	// Do the measurement
	ADI_ADC_RESULT eResult = ADI_ADC_SUCCESS;
	ADI_ADC_BUFFER Buffer;

	/* Set the acquisition time. (Application need to change it based on the impedance) */
	adi_adc_SetAcquisitionTime(adcDevice, acquisition_time); //10 cycles for TDR is okay, max for therm?


	/* Populate the buffer structure */
	Buffer.nBuffSize = sizeof(ADC_DataBuffer);
	Buffer.nChannels = channel;
	Buffer.nNumConversionPasses = ADC_NUM_SAMPLES;
	Buffer.pDataBuffer = ADC_DataBuffer;

	/* Submit the buffer to the driver */
	adi_adc_SubmitBuffer(adcDevice, &Buffer);

	/* Enable the ADC */
	adi_adc_Enable(adcDevice, true);

	ADI_ADC_BUFFER* pAdcBuffer = &Buffer;
	adi_adc_GetBuffer(adcDevice, &pAdcBuffer);

	/* Disable the ADC */
	adi_adc_Enable(adcDevice, false);

	// Power off the ADC
	AdcMcuPower(false);

	// Compute average of gathered measurements
	for(uint8_t i=0; i<ADC_NUM_SAMPLES;++i)
	{
		avg_val += ADC_DataBuffer[i];
	}
	avg_val /= ADC_NUM_SAMPLES;

	return avg_val;
}


/*
 * adc.c
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#include "adc-au.h"

#include <drivers/general/adi_drivers_general.h>
#include "common.h"

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

/**
 * @brief    ADC init function.
 *
 * Resolution is agreed to be 12b and the delay time between the subsequent measurements in one request is 10 clock cycles. This was seen to be enough
 * for both TDR and thermistors but adjustments may be necessary. The reference is external and set to approx. 3.1V.
 *
 * Based on standard initialization from the example.
 *
 * @note Calibration gets stuck for some reason and I can't explain why so it may be a good idea to investigate it. It worked before.
 *
 * @param[in] calibration True if calibration needed (false for now for the reason in the note).
 */
void adc_init(bool calibration)
{
    ADI_ADC_RESULT  eResult = ADI_ADC_SUCCESS;
    bool bCalibrationDone = false;

    /* Open the ADC device */
    eResult = adi_adc_Open(ADC_DEV_NUM, DeviceMemory, sizeof(DeviceMemory), &adcDevice);
    DEBUG_RESULT("Failed to open ADC device",eResult, ADI_ADC_SUCCESS);

    /* Set the delay time */
    eResult = adi_adc_SetDelayTime ( adcDevice, 10); // 10cycles for TDR is okay
    DEBUG_RESULT("Failed to set the Delay time ", eResult, ADI_ADC_SUCCESS);

	eResult = adi_adc_SetResolution(adcDevice, ADI_ADC_RESOLUTION_12_BIT);
	DEBUG_RESULT("Failed to set resolution", eResult, ADI_ADC_SUCCESS);

    /* Set ADC reference */
    eResult = adi_adc_SetVrefSource (adcDevice, ADI_ADC_VREF_SRC_EXT);
    DEBUG_RESULT("Failed to set ADC reference", eResult, ADI_ADC_SUCCESS);

	/* Enable ADC sub system */
	eResult = adi_adc_EnableADCSubSystem (adcDevice, true);
	DEBUG_RESULT("Failed to enable ADC sub system", eResult, ADI_ADC_SUCCESS);

	uint32_t delay_val = 8000; // 10ms now
	while(--delay_val){};

	/* Start calibration */
	if(calibration)
	{
		eResult = adi_adc_StartCalibration (adcDevice);
		DEBUG_RESULT("Failed to start calibration", eResult, ADI_ADC_SUCCESS);

		/* Wait until calibration is done */
		while (!bCalibrationDone)
		{
			eResult = adi_adc_IsCalibrationDone (adcDevice, &bCalibrationDone);
			DEBUG_RESULT("Failed to get the calibration status", eResult, ADI_ADC_SUCCESS);
		}
	}
}

/**
 * @brief    ADC deinit function.
 *
 * Deinitializes the device, meaning it can't be referenced by the handle defined in this file.
 */
void adc_deinit()
{
	ADI_ADC_RESULT  eResult = ADI_ADC_SUCCESS;
	eResult = adi_adc_Close(adcDevice);
	DEBUG_RESULT("Failed to close ADC", eResult, ADI_ADC_SUCCESS);
}

/**
 * @brief    ADC power up function.
 *
 * This assumes that the adc_init() was called before. It powers up the adc (if EnableADCSubSystem was called before), making it ready for conversions.
 *
 * @note It takes a few hundred us for it to be able to convert.
 *
 */
void power_up_adc()
{
	ADI_ADC_RESULT  eResult = ADI_ADC_SUCCESS;
	/* Power up ADC */
	eResult = adi_adc_PowerUp (adcDevice, true);
	DEBUG_RESULT("Failed to power up ADC", eResult, ADI_ADC_SUCCESS);

	//	usleep (700);
	uint32_t delay_val = 1120; // 700us
	while(--delay_val){};
}

/**
 * @brief    ADC power down function.
 *
 * This assumes that the adc_init() was called before. It powers down the adc (if EnableADCSubSystem and power_up_adc() were called before). This function ALSO TURNS
 * OFF THE SUBSYSTEM, so calling only power_up_adc() is NOT enough to use it afterwards (call adi_adc_EnableADCSubSystem (adcDevice, true) before calling it). Also,
 * this function doesn't need to be called, really, because when the system enters hibernation, the adc is automatically turned off.
 */
void power_down_adc()
{
    ADI_ADC_RESULT  eResult = ADI_ADC_SUCCESS;

	eResult = adi_adc_PowerUp (adcDevice, false);
	DEBUG_RESULT("Failed to power down ADC", eResult, ADI_ADC_SUCCESS);
}

/**
 * @brief    ADC conversion trigger function.
 *
 * @param[in] channels Channels to be converted (should be passed one by one - see note).
 * @param[in] acquisition_time Defines the acquisition time for the conversion.
 *
 * Triggers a set of ADC_NUM_SAMPLES (defined in adc.h) conversions with given acquisition time. Acquisition time should be adjusted to match the impedance of the signal source
 * (will be different for hundreds of tens of thousands of ohms). Works in blocking mode, because there is no need to do it in different way.
 *
 * @note Channels are passed one by one because when you consider the system operation, every sensor have to be set up separately (normally thermistors would be
 * measured simultaneously but because the current mirror is being connected to every one of them separately, it's sequential).
 *
 * Follows standard adc example.
 *
 * The return value is store in the ADC_DataBuffer array and the final value can be obtained outside through get_adc_data_buffer(), which performs a very simple
 * digital filtering (average from the ADC_NUM_SAMPLES number of samples).
 */
void ADC_SampleData(ADI_ADC_CHANNEL channels, uint8_t acquisition_time)
{

    ADI_ADC_RESULT  eResult = ADI_ADC_SUCCESS;
    ADI_ADC_BUFFER Buffer;

    /* Set the acquisition time. (Application need to change it based on the impedance) */
    eResult = adi_adc_SetAcquisitionTime ( adcDevice, acquisition_time); //10 cycles for TDR is okay, max for therm?
    DEBUG_RESULT("Failed to set the acquisition time ", eResult, ADI_ADC_SUCCESS);

    /* Populate the buffer structure */
    Buffer.nBuffSize = sizeof(ADC_DataBuffer);
    Buffer.nChannels = channels;
    Buffer.nNumConversionPasses = ADC_NUM_SAMPLES;
    Buffer.pDataBuffer = ADC_DataBuffer;

    /* Submit the buffer to the driver */
    eResult = adi_adc_SubmitBuffer (adcDevice, &Buffer);
    DEBUG_RESULT("Failed to submit buffer ", eResult, ADI_ADC_SUCCESS);

    /* Enable the ADC */
    eResult = adi_adc_Enable (adcDevice, true);
    DEBUG_RESULT("Failed to enable the ADC for sampling ", eResult, ADI_ADC_SUCCESS);

    ADI_ADC_BUFFER* pAdcBuffer = &Buffer;
    eResult = adi_adc_GetBuffer (adcDevice, &pAdcBuffer);
    DEBUG_RESULT("Failed to Get Buffer ", eResult, ADI_ADC_SUCCESS);

    /* Disable the ADC */
    eResult = adi_adc_Enable (adcDevice, false);
    DEBUG_RESULT("Failed to disable ADC for sampling ", eResult, ADI_ADC_SUCCESS);

}

/**
 * @brief    ADC conversion trigger function.
 *
 * @return Returns the averaged ADC measurements from ADC_NUM_SAMPLES samples.
 */
uint16_t get_adc_data_buffer()
{
	uint16_t local=0;
	for(uint8_t i=0; i<ADC_NUM_SAMPLES;++i)
	{
		local += ADC_DataBuffer[i];
	}
	local /= ADC_NUM_SAMPLES;

	return local;
}

#include "adi_adc.h"

#define ADC_SINGLE_CHANNEL_BUFFER_SIZE 2 // 1 channel, 2 bytes per sample
#define ADC_BUFFER_SIZE 6 // 3 channels, 2 bytes each

// Static ADC handle, shared internally
static ADI_ADC_HANDLE hADC = NULL;

// Memory for ADC driver
static uint8_t adcMemory[ADI_ADC_MEMORY_SIZE];


// Function to initialize and configure the ADC with calibration
ADI_ADC_RESULT ADC_Init() {
    if (hADC) return ADI_ADC_SUCCESS; // Already initialized

    // Open the ADC driver
    ADI_ADC_RESULT result = adi_adc_Open(0, adcMemory, sizeof(adcMemory), &hADC);
    if (result != ADI_ADC_SUCCESS) return result;

    // Power up the ADC
    result = adi_adc_PowerUp(hADC, true);
    if (result != ADI_ADC_SUCCESS) return result;

    // Enable the ADC subsystem
    result = adi_adc_EnableADCSubSystem(hADC, true);
    if (result != ADI_ADC_SUCCESS) return result;

    // Wait for the ADC to be ready
    bool isReady = false;
    while (adi_adc_IsReady(hADC, &isReady) == ADI_ADC_SUCCESS && !isReady);

    // Start ADC calibration
    result = adi_adc_StartCalibration(hADC);
    if (result != ADI_ADC_SUCCESS) return result;

    // Wait for calibration to complete
    bool isCalibrationDone = false;
    while (adi_adc_IsCalibrationDone(hADC, &isCalibrationDone) == ADI_ADC_SUCCESS && !isCalibrationDone);

    // Set reference voltage to 2.5V
    result = adi_adc_SetVrefSource(hADC, ADI_ADC_VREF_SRC_INT_2_50_V);
    if (result != ADI_ADC_SUCCESS) return result;

    // Set resolution to 12 bits
    result = adi_adc_SetResolution(hADC, ADI_ADC_RESOLUTION_12_BIT);
    if (result != ADI_ADC_SUCCESS) return result;

    // Set acquisition time (adjust as needed)
    result = adi_adc_SetAcquisitionTime(hADC, 10);
    if (result != ADI_ADC_SUCCESS) return result;

    return ADI_ADC_SUCCESS;
}

// Sample channels 1, 2, and 3
ADI_ADC_RESULT ADC_SampleChannels(uint16_t* results) {
    if (!hADC) return ADI_ADC_INVALID_DEVICE_HANDLE;

    uint8_t adcBuffer[ADC_BUFFER_SIZE];
    ADI_ADC_BUFFER adcConfig = {
        .nChannels = ADI_ADC_CHANNEL_1 | ADI_ADC_CHANNEL_2 | ADI_ADC_CHANNEL_3,
        .pDataBuffer = adcBuffer,
        .nNumConversionPasses = 1,
        .nBuffSize = ADC_BUFFER_SIZE
    };

    // Submit buffer for sampling
    ADI_ADC_RESULT result = adi_adc_SubmitBuffer(hADC, &adcConfig);
    if (result != ADI_ADC_SUCCESS) return result;

    // Enable ADC
    result = adi_adc_Enable(hADC, true);
    if (result != ADI_ADC_SUCCESS) return result;

    // Poll for completion
    bool isBufferAvailable = false;
    while (adi_adc_IsBufferAvailable(hADC, &isBufferAvailable) == ADI_ADC_SUCCESS && !isBufferAvailable);

    // Get sampled data
    ADI_ADC_BUFFER* completedBuffer;
    result = adi_adc_GetBuffer(hADC, &completedBuffer);
    if (result != ADI_ADC_SUCCESS) return result;

    // Copy results from buffer
    results[0] = ((uint16_t*)adcBuffer)[0]; // Channel 1
    results[1] = ((uint16_t*)adcBuffer)[1]; // Channel 2
    results[2] = ((uint16_t*)adcBuffer)[2]; // Channel 3

    return ADI_ADC_SUCCESS;
}

// Cleanup the ADC
void ADC_Cleanup() {
    if (!hADC) return;

    adi_adc_Enable(hADC, false);
    adi_adc_EnableADCSubSystem(hADC, false);
    adi_adc_PowerUp(hADC, false);
    adi_adc_Close(hADC);

    hADC = NULL;
}

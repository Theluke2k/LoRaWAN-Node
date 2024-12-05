#ifndef ADC_HELPER_H
#define ADC_HELPER_H

#include <stdint.h>
#include "adi_adc.h"  // Include the ADC library

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and configure the ADC.
 *
 * This function initializes the ADC hardware, powers it up,
 * configures its resolution, acquisition time, and sets up the ADC subsystem.
 *
 * @return ADI_ADC_RESULT
 *         - ADI_ADC_SUCCESS on success
 *         - An error code from the ADI_ADC_RESULT enum otherwise.
 */
ADI_ADC_RESULT ADC_Init(void);

/**
 * @brief Sample ADC channels 1, 2, and 3.
 *
 * This function samples the specified channels and returns the results.
 *
 * @param[out] results Pointer to a 3-element array where sampled values for
 *                     channels 1, 2, and 3 will be stored.
 * @return ADI_ADC_RESULT
 *         - ADI_ADC_SUCCESS on success
 *         - An error code from the ADI_ADC_RESULT enum otherwise.
 */
ADI_ADC_RESULT ADC_SampleChannels(uint16_t* results);

/**
 * @brief Cleanup the ADC.
 *
 * This function disables the ADC, powers it down, and releases its resources.
 */
void ADC_Cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_HELPER_H */

/*
 * adc.h
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#ifndef ADC_AU_H_
#define ADC_AU_H_

#include <adi_adc.h>

/**
 * Number of times the line is stimulated to get the TDR readout.
 */
#define ADC_NUM_OF_TDR_STIMULATIONS         5    // 625us per measurement package

/**
 * Number of samples to be taken in the single trigger for any measurement. Allows for simple digital filtering.
 */
#define ADC_NUM_SAMPLES             (10u)

/**
 * ADC ID number.
 */
#define ADC_DEV_NUM                	(0u)

/**
 * Renamed macros for the ADC channels used in the project (more readable).
 */
#define	ADC_THERM_READ_CHANNEL			ADI_ADC_CHANNEL_0
#define ADC_INTEGRATOR1_CHANNEL			ADI_ADC_CHANNEL_1
#define ADC_INTEGRATOR2_CHANNEL			ADI_ADC_CHANNEL_2

/*
 * Approximately 3.1V, but can vary with tolerance defined for the reference voltage generator. Refer to its datasheet.
 */
#define ADC_VCC_REF                     3.100 // V  #5: 3.092V (diff 0.2mV)
/*
 * 2^10 = 4096, so 4095 is the max value for the ADC in this config.
 */
#define ADC_MAX_VAL                     4095.0 // b

void adc_init(bool calibration);
void adc_deinit();
void power_up_adc();
void power_down_adc();
void ADC_SampleData(ADI_ADC_CHANNEL channels, uint8_t acquisition_time);
uint16_t get_adc_data_buffer();

#endif /* ADC_AU_H_ */

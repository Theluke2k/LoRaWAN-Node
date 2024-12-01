/*!
 * \file      adc-board.h
 *
 * \brief     Target board ADC driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __ADC_BOARD_H__
#define __ADC_BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <adi_adc.h>
#include "adc.h"

/**
 * Number of samples to be taken in the single trigger for any measurement. Allows for simple digital filtering.
 */
#define ADC_NUM_SAMPLES             (10u)

/**
 * ADC ID number.
 */
#define ADC_DEV_NUM                	(0u)


void AdcMcuInitDriver( bool calibration );
uint16_t AdcMcuReadChannelCustom( Adc_t *obj, uint32_t channel, uint8_t acquisition_time);
/*!
 * \brief Initializes the ADC object and MCU peripheral
 *
 * \param [IN] obj      ADC object
 * \param [IN] adcInput ADC input pin
 */
void AdcMcuInit( Adc_t *obj, PinNames adcInput );

/*!
 * \brief Initializes the ADC internal parameters
 */
void AdcMcuConfig( void );

/*!
 * \brief Reads the value of the given channel
 *
 * \param [IN] obj     ADC object
 * \param [IN] channel ADC input channel
 */
uint16_t AdcMcuReadChannel( Adc_t *obj, uint32_t channel );

#ifdef __cplusplus
}
#endif

#endif // __ADC_BOARD_H__

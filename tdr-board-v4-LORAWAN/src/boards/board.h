/*!
 * \file      board.h
 *
 * \brief     Target board general functions implementation
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
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "utilities.h"
#include "sx1276Regs-LoRa.h"
#include "sx1276Regs-Fsk.h"
#include "gpio.h"

/*
 * Lucas (25-08-2024):
 * Struct to contain data about the RawLoRa session.
 */
typedef struct RawLoRa_Config
{
	uint32_t		StartIn;
	uint32_t		Duration;
	uint32_t		TxPeriodicity;
    uint8_t        	SpreadingFactor;
    uint32_t		Frequency;
}RawLoRa_Config;

/*!
 * Possible power sources
 */
enum BoardPowerSources
{
    USB_POWER = 0,
    BATTERY_POWER,
};

uint16_t getADCSuperCapMeasurement();

/*!
 * \brief Lucas: Initializes the required systems on the board after hibernate wakeup.
 */
void SystemReinitializerFromHibernate( void );

/*!
 * \brief Lucas: Manages everything that needs to be done before entering hibernate.
*/
void SystemPrepareHibernate( void );

/*!
 * \brief Initializes the mcu.
 */
void BoardInitMcu( void );

/*!
 * \brief Resets the mcu.
 */
void BoardResetMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Gets the current potentiometer level value
 *
 * \retval value  Potentiometer level ( value in percent )
 */
uint8_t BoardGetPotiLevel( void );

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value  battery voltage in volts
 */
uint32_t BoardGetBatteryVoltage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * \brief Get the current MCU temperature in degree celcius * 256
 *
 * \retval temperature * 256
 */
int16_t BoardGetTemperature( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Manages the entry into ARM cortex deep-sleep mode
 */
void BoardLowPowerHandler( void );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source [0: USB_POWER, 1: BATTERY_POWER]
 */
uint8_t GetBoardPowerSource( void );

/*!
 * \brief Get the board version
 *
 * \retval value  Version
 */
Version_t BoardGetVersion( void );

void RawLoRaSend(RawLoRa_Config *config, uint8_t *data, uint8_t packet_length);
uint8_t RawLoRa_RadioConfig(RawLoRa_Config *config);
uint8_t RawLoRa_RadioSend(uint8_t *data, uint8_t packet_length);

#ifdef __cplusplus
}
#endif

#endif // __BOARD_H__

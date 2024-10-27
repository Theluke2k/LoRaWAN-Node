/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 *
 *
 * \Changes: changed to reflect the pins of the AD4050LZ device, not the generic example
 */
#include <sys/platform.h>
#include "adi_initialize.h"
#include <stdint.h>
#include <common.h>
#include <unistd.h>
#include <drivers/general/adi_drivers_general.h>
#include <adi_processor.h>
#include <drivers/pwr/adi_pwr.h>
#include <adi_gpio.h>
#include <drivers/spi/adi_spi.h>

/* These are probably not all necessary */
#include <stdio.h>
#include "utilities.h"
#include "board.h"
#include "gpio.h" //Might have to be changed to be something more specific relating to the board
#include "uart.h"
#include "cli.h"
#include "pinName-board.h"

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      1

/*!
 * Board MCU pins definitions
 */

typedef struct {
    ADI_GPIO_PORT Port;
    ADI_GPIO_DATA Pins;
} PinMap;

//PinMap NSSPin = {ADI_GPIO_PORT2, ADI_GPIO_PIN_8};

//Hmm, it seems that only the first value of the comma-separated expression is read

// SX1276
#define RADIO_RESET                                	P1_6 //old P2_1
#define TCXO_PWR_PIN                                //GPIO( GPIO_PORTA, 9 ) there is not a dedicated pin, might be UART?
#define RF_SWITCH_PIN                               //GPIO( GPIO_PORTA, 13 ) There is not a dedicated RF switch pin, I believe
#define RADIO_MOSI                                  P0_1
#define RADIO_MISO                                  P0_2
#define RADIO_SCLK                                  P0_0
#define RADIO_NSS 								    P0_3
#define RADIO_DIO_0                                 P1_7 //old P1_5
#define RADIO_DIO_1                                 P1_8 //old P1_4
#define RADIO_DIO_2                                 P1_11 //old P1_3
#define RADIO_DIO_3                                 P1_12 //old P1_2
#define RADIO_DIO_4                                 P1_13 //new
#define RADIO_DIO_5                                 P1_14 //new

// EEPROM
#define EEPROM_NSS									P1_10
#define EEPROM_WP									P0_8
#define EEPROM_HOLD									P0_12


//#define LED_1                                       (ADI_GPIO_PORT2, ADI_GPIO_PIN_2) //The yellow LED (why?
#define UART_TX                                     P0__10 // not used
#define UART_RX                                     P0_11  // not used
#define I2C_SDA                                     //GPIO( GPIO_PORTA, 16 ) I2C serial data, don't think it's dedicated
#define I2C_SCL                                     //GPIO( GPIO_PORTA, 17 ) I2C serial clock - can be handled by specific functions

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            NC
#define RADIO_DBG_PIN_RX                            NC

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__

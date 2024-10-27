/*
 * eeprom.h
 *
 *  Created on: 27. okt. 2024
 *      Author: lselm
 */

#ifndef EEPROM_H_
#define EEPROM_H_

/*!
 * EEPROM hardware and global parameters
 */
typedef struct EEPROM_t
{
    Gpio_t        WP;
    Gpio_t        HOLD;
    Spi_t         Spi;
}EEPROM_t;

#endif /* EEPROM_H_ */

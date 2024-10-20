/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
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
#include "utilities.h"
#include "eeprom-board.h"
#include <adi_gpio.h>

/*
 * Lucas (30-03-2024)
 * This driver emulates Eeprom by just using the flash to save the variables.
 * For this to work, it is assumed that the device is never turned off. In
 * the worst case if it turns off, it will just have to rejoin the network again.
 */
#define EMULATED_EEPROM_SIZE			65536

static uint8_t emulatedEepromStorage[EMULATED_EEPROM_SIZE] = {0};

uint16_t s = 0;

LmnStatus_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	adi_gpio_Toggle(ADI_GPIO_PORT0, ADI_GPIO_PIN_14);
	DelayMsMcu(4);
	s += size;
    if(addr + size <= EMULATED_EEPROM_SIZE) {
    	memcpy1(&emulatedEepromStorage[addr], buffer, size);
    	return LMN_STATUS_OK;
    }
    else {
    	printf('damn\n');
    }


    return LMN_STATUS_ERROR;
}

LmnStatus_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	if (addr + size <= EMULATED_EEPROM_SIZE) {
		memcpy1(buffer, &emulatedEepromStorage[addr], size);
		return LMN_STATUS_OK;
	}
	return LMN_STATUS_ERROR;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    while( 1 )
    {
    }
}

LmnStatus_t EepromMcuGetDeviceAddr( void )
{
    while( 1 )
    {
    }
//    return 0;
}

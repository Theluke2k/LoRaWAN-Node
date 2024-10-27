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
#include "gpio.h"
#include "board-config.h"

/*!
 * EEPROM hardware and global parameters
 */
EEPROM_t eeprom;

/*
 * Lucas (30-03-2024)
 * This driver emulates Eeprom by just using the flash to save the variables.
 * For this to work, it is assumed that the device is never turned off. In
 * the worst case if it turns off, it will just have to rejoin the network again.
 */
#define EMULATED_EEPROM_SIZE			65536

static uint8_t emulatedEepromStorage[EMULATED_EEPROM_SIZE] = {0};

uint16_t s = 0;

uint8_t eepDelay = 5;

void EepromIoInit() {
	GpioInit( &eeprom.Spi.Nss, EEPROM_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
	GpioInit( &eeprom.WP, EEPROM_WP, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
	GpioInit( &eeprom.HOLD, EEPROM_HOLD, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void EepromReset() {
	// Variable to store byte received from EEPROM
	uint8_t rx = 0;

	// Disable Write Protect
	GpioWrite( &eeprom.WP, 1 );
	DelayMsMcu(eepDelay);

	// Send a Reset Enable instruction
	GpioWrite( &eeprom.Spi.Nss, 0 );
	rx = SpiInOut( &eeprom.Spi, EEPROM_RSTEN);
	GpioWrite( &eeprom.Spi.Nss, 1 );
	DelayMsMcu(eepDelay);

	// Send a Reset instruction
	GpioWrite( &eeprom.Spi.Nss, 0 );
	rx = SpiInOut( &eeprom.Spi, EEPROM_RESET);
	GpioWrite( &eeprom.Spi.Nss, 1 );
	DelayMsMcu(eepDelay);

	// Enable Write Protect
	GpioWrite( &eeprom.WP, 0 );
	DelayMsMcu(eepDelay);
}
// OLD
//LmnStatus_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
//{
//	DelayMsMcu(4);
//	s += size;
//    if(addr + size <= EMULATED_EEPROM_SIZE) {
//    	memcpy1(&emulatedEepromStorage[addr], buffer, size);
//    	return LMN_STATUS_OK;
//    }
//    else {
//    	printf('damn\n');
//    }
//
//
//    return LMN_STATUS_ERROR;
//}

// OLD
//LmnStatus_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
//{
//	if (addr + size <= EMULATED_EEPROM_SIZE) {
//		memcpy1(buffer, &emulatedEepromStorage[addr], size);
//		return LMN_STATUS_OK;
//	}
//	return LMN_STATUS_ERROR;
//}

// NEW
LmnStatus_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	// Variable to store byte received from EEPROM
	uint8_t rx = 0;

	// Disable Write Protect
	//GpioWrite( &eeprom.WP, 1 );

	// Send a Write Enable instruction
	GpioWrite( &eeprom.Spi.Nss, 0 );
	rx = SpiInOut( &eeprom.Spi, EEPROM_WREN);
	GpioWrite( &eeprom.Spi.Nss, 1 );
	DelayMsMcu(eepDelay);

	// Select slave
	GpioWrite( &eeprom.Spi.Nss, 0 );

	// Send page write instruction
	rx = SpiInOut( &eeprom.Spi, EEPROM_PGWR);

	// Send address
	rx = SpiInOut( &eeprom.Spi, 0);						// Send high byte
	rx = SpiInOut( &eeprom.Spi, (addr >> 8) & 0xFF);	// Send middle byte
	rx = SpiInOut( &eeprom.Spi, addr & 0xFF);			// Send low byte

	// Send data
	for(int i = 0; i < size; i++) {
		rx = SpiInOut( &eeprom.Spi, buffer[i]);
	}

	// De-select slave
	GpioWrite( &eeprom.Spi.Nss, 1 );
	DelayMsMcu(eepDelay);

	// Wait until the Write in Progress bit is cleared
	do {
		// Send Read Status Register instruction
		GpioWrite( &eeprom.Spi.Nss, 0 );
		rx = SpiInOut( &eeprom.Spi, EEPROM_RDSR);
		rx = SpiInOut( &eeprom.Spi, 0xFF);
		GpioWrite( &eeprom.Spi.Nss, 1 );
		DelayMsMcu(eepDelay);

	}while(rx & 0x01); // Check WIP bit

	// Enable Write Protect
	//GpioWrite( &eeprom.WP, 0 );


    return LMN_STATUS_OK;
}

// NEW
LmnStatus_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	// Variable to store byte received from EEPROM
	uint8_t rx = 0;

	// Select slave
	GpioWrite(&eeprom.Spi.Nss, 0);

	// Send page write instruction
	rx = SpiInOut(&eeprom.Spi, EEPROM_READ);

	// Send address
	rx = SpiInOut(&eeprom.Spi, 0);						// Send high byte
	rx = SpiInOut(&eeprom.Spi, (addr >> 8) & 0xFF);		// Send middle byte
	rx = SpiInOut(&eeprom.Spi, addr & 0xFF);			// Send low byte

	// Read data
	for (int i = 0; i < size; i++) {
		buffer[i] = SpiInOut(&eeprom.Spi, 0xFF); // Dummy byte is sent
	}

	// De-select slave
	GpioWrite(&eeprom.Spi.Nss, 1);
	DelayMsMcu(eepDelay);

	return LMN_STATUS_OK;
}


LmnStatus_t EepromMcuReadStatus( uint8_t *buffer, uint16_t size )
{
	// Variable to store byte received from EEPROM
	uint8_t rx = 0;

	// Select slave
	GpioWrite(&eeprom.Spi.Nss, 0);

	// Send read status register instruction
	rx = SpiInOut(&eeprom.Spi, EEPROM_RDSR);

	// Read data
	for (int i = 0; i < size; i++) {
		buffer[i] = SpiInOut(&eeprom.Spi, 0xFF); // Dummy byte is sent
	}

	// De-select slave
	GpioWrite(&eeprom.Spi.Nss, 1);
	DelayMsMcu(eepDelay);

	return LMN_STATUS_OK;
}

LmnStatus_t EepromMcuReadIdentification( uint8_t *buffer, uint16_t size )
{
	// Variable to store byte received from EEPROM
	uint8_t rx = 0;

	// Select slave
	GpioWrite(&eeprom.Spi.Nss, 0);

	// Send read status register instruction
	rx = SpiInOut(&eeprom.Spi, EEPROM_RDID);

	// Send address
	rx = SpiInOut(&eeprom.Spi, 0);						// Send high byte
	rx = SpiInOut(&eeprom.Spi, 0);		// Send middle byte
	rx = SpiInOut(&eeprom.Spi, 0);			// Send low byte

	// Read data
	for (int i = 0; i < size; i++) {
		buffer[i] = SpiInOut(&eeprom.Spi, 0xFF); // Dummy byte is sent
	}

	// De-select slave
	GpioWrite(&eeprom.Spi.Nss, 1);
	//DelayMsMcu(eepDelay);

	return LMN_STATUS_OK;
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

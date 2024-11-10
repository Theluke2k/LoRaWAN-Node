/*!
 * \file      eeprom-board.h
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
#ifndef __EEPROM_BOARD_H__
#define __EEPROM_BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "utilities.h"
#include "eeprom.h"

// Instructions
#define EEPROM_WREN							0x06
#define EEPROM_WRDI							0x04
#define EEPROM_RDSR							0x05
#define EEPROM_WRSR							0x01
#define EEPROM_READ							0x03
#define EEPROM_FREAD							0x0B
#define EEPROM_FDREAD							0x3B
#define EEPROM_FQREAD							0x6B
#define EEPROM_PGWR							0x02
#define EEPROM_PGPR							0x0A
#define EEPROM_PGER							0xDB
#define EEPROM_SCER							0x20
#define EEPROM_BKER							0xD8
#define EEPROM_CHER							0xC7
#define EEPROM_RDID							0x83
#define EEPROM_FRDID							0x8B
#define EEPROM_WRID							0x82
#define EEPROM_DPD								0xB9
#define EEPROM_RDPD							0xAB
#define EEPROM_JEDID							0x9F
#define EEPROM_RDCR							0x15
#define EEPROM_RDVR							0x85
#define EEPROM_WRVR							0x81
#define EEPROM_CLRSF							0x50
#define EEPROM_RDSFDP							0x5A
#define EEPROM_RSTEN							0x66
#define EEPROM_RESET							0x99


#define EMULATED_EEPROM_SIZE			1536
/*
 * Lucas (27/10/24):
 *Add EPROM functionality
 */
/*!
 * EEPROM hardware and global parameters
 */
typedef struct EEPROM_t
{
    Gpio_t        WP;
    Gpio_t        HOLD;
    Spi_t         Spi;
}EEPROM_t;

/*!
 * \brief Initializes the EEPROM I/Os pins interface
 */
void EepromIoInit( void );

/*!
 * \brief De-initializes the EEPROM I/Os pins interface
 */
void EepromIoDeInit();

/*!
 * \brief Resets the EEPROM I/Os pins interface
 */
void EepromReset( void );

/*!
 * \brief Read status register of EEPROM
 */
LmnStatus_t EepromReadStatus( uint8_t *buffer, uint16_t size );

/*!
 * \brief Read Identification register of EEPROM
 */
LmnStatus_t EepromReadIdentification( uint8_t *buffer, uint16_t size );

/*!
 * brief Writes bytes across pages of EEPROM.
 *
 * \param[IN] addr EEPROM address to write to
 * \param[IN] buffer Pointer to the buffer to be written.
 * \param[IN] size Size of the buffer to be written.
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t EepromWriteBufferDirect( uint16_t addr, uint8_t *buffer, uint16_t size );

LmnStatus_t EepromWriteAcrossPage( uint16_t addr, uint8_t *buffer, uint16_t size );

LmnStatus_t EepromReadBufferDirect( uint16_t addr, uint8_t *buffer, uint16_t size );

LmnStatus_t EepromDownloadMirror( uint16_t addr, uint16_t size );

LmnStatus_t EepromUploadMirror( uint16_t addr, uint16_t size );

/*!
 * Writes the given buffer to the EEPROM at the specified address. Has to be within one page.
 *
 * \param[IN] addr EEPROM address to write to
 * \param[IN] buffer Pointer to the buffer to be written.
 * \param[IN] size Size of the buffer to be written.
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size );

/*!
 * Reads the EEPROM at the specified address to the given buffer.
 *
 * \param[IN] addr EEPROM address to read from
 * \param[OUT] buffer Pointer to the buffer to be written with read data.
 * \param[IN] size Size of the buffer to be read.
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size );

/*!
 * Sets the device address.
 *
 * \remark Useful for I2C external EEPROMS
 *
 * \param[IN] addr External EEPROM address
 */
void EepromMcuSetDeviceAddr( uint8_t addr );

/*!
 * Gets the current device address.
 *
 * \remark Useful for I2C external EEPROMS
 *
 * \retval addr External EEPROM address
 */
LmnStatus_t EepromMcuGetDeviceAddr( void );

/*!
 * Radio hardware and global parameters
 */
extern EEPROM_t eeprom;
#ifdef __cplusplus
}
#endif

#endif // __EEPROM_BOARD_H__

/*
 * lora.c
 *
 *  Created on: 25. mar. 2023
 *      Author: au703540
 */

#include "spi-au.h"

#include "sx1276_registers.h"
#include <adi_spi.h>
#include <adi_spi_config.h>
#include <adi_gpio.h>
#include "gpio-au.h"
#include <drivers/general/adi_drivers_general.h>


/*
 * SPI device (ADI driver).
 */
static ADI_SPI_HANDLE spiDevice;

/*
 * SPI device memory (ADI driver).
 */
ADI_ALIGNED_PRAGMA(2)
uint8_t spiDeviceMemory[ADI_SPI_MEMORY_SIZE] ADI_ALIGNED_ATTRIBUTE(2);

/* Transmit data buffer */
ADI_ALIGNED_PRAGMA(2)
static uint8_t spiTxBuffer[SPI_BUFFER_SIZE] ADI_ALIGNED_ATTRIBUTE(2);
/* Receieve data buffer */
ADI_ALIGNED_PRAGMA(2)
static uint8_t spiRxBuffer[SPI_BUFFER_SIZE] ADI_ALIGNED_ATTRIBUTE(2);

/**
 * @brief    SPI init function.
 *
 * Used by the LoRA module SX1276. Set to match its requirements, based on different drivers (Arduino, Semtech and some other for the STMs). Follows the ADI example.
 * Continuous mode selected, which means that bytes are sent without return to 1 when byte is transmitted.
 *
 * It goes like this: 		ADDRESS | DATA | DATA | DATA ...
 * Instead of: 				ADDRESS | DATA | BREAK | DATA | BREAK ....
 *
 * First option is required for SX1276 communication.
 */
ADI_SPI_RESULT spi_init()
{
	ADI_SPI_RESULT eResult;

	 /* Initialize SPI */
	eResult = adi_spi_Open(SPI_DEVICE_NUM, &spiDeviceMemory, ADI_SPI_MEMORY_SIZE, &spiDevice);
//	DEBUG_RESULT("Failed to init SPI driver",eResult,ADI_SPI_SUCCESS);

	/* throttle bitrate to something the controller can reach */
	eResult = adi_spi_SetBitrate(spiDevice, 2500000);

//	DEBUG_RESULT("Failed to set Bitrate",eResult,ADI_SPI_SUCCESS);

	/* Set IRQMODE. In this case we are setting it to the default value  */
	/* This code sequence is just calling out the fact that this API would be required  */
	/* for short bursts (less than the size of the FIFO) in PIO (interrupt) mode        */
	eResult = adi_spi_SetIrqmode(spiDevice, 0u);
//	DEBUG_RESULT("Failed to set Irqmode",eResult,ADI_SPI_SUCCESS);

	/* set the chip select */
	eResult = adi_spi_SetChipSelect(spiDevice, ADI_SPI_CS0);
//	DEBUG_RESULT("Failed to set the chip select",eResult,ADI_SPI_SUCCESS);

	eResult = adi_spi_SetClockPhase(spiDevice, false);

	eResult = adi_spi_SetClockPolarity(spiDevice, false);

	eResult = adi_spi_SetContinuousMode(spiDevice, true);

	return ADI_SPI_SUCCESS;
}

/**
 * @brief    SPI write single byte.
 *
 * Writes a single byte to a device of given address. Works in blocking mode, as there is no need for non-sequential operation of the system. Additional
 * flow control is unused.
 *
 * @param[in] address Address of the slave
 * @param[in] data Byte to send
 */
ADI_SPI_RESULT spi_write_byte(uint8_t address, uint8_t data)
{
	ADI_SPI_RESULT result;
	ADI_SPI_TRANSCEIVER transceiver;

	spiRxBuffer[0] = 0xdd; // some value to initialize it.
	spiRxBuffer[1] = 0xdd; // some value to initialize it. (easier debugging process)

	transceiver.TransmitterBytes = 2;
	transceiver.ReceiverBytes = 2;
	transceiver.pTransmitter = spiTxBuffer;
	transceiver.pReceiver = spiRxBuffer;
	transceiver.nTxIncrement = 1;
	transceiver.nRxIncrement = 1;
	transceiver.bDMA = false;
	transceiver.bRD_CTL = false;

	*spiTxBuffer = address | 0x80;
	*(spiTxBuffer + 1) = data;

    if (ADI_SPI_SUCCESS != (result = adi_spi_MasterReadWrite(spiDevice, &transceiver)))
    {
        return result;
    }
    // DEBUG (added to log SPI data)
    //printf("RX: %d\n", transceiver.pReceiver[0]);
    //printf("RX: %d\n", transceiver.pReceiver[1]);

	return ADI_SPI_SUCCESS;
}

/**
 * @brief    SPI write a stream of bytes.
 *
 * Writes a stream of bytes of given length to given address.
 *
 * @param[in] tx_bytes Bytes to transmit (later one byte will be added for address).
 * @param[in] rx_bytes Bytes to receive. During write operation this needs to be equal to tx_bytes + 1 (additional address) to keep the clock up.
 * @param[in] address Address of the slave
 * @param[in] data Pointer to the array of bytes to send.
 */
ADI_SPI_RESULT spi_write(uint8_t tx_bytes, uint8_t rx_bytes, uint8_t address, uint8_t* data)
{
	ADI_SPI_RESULT result;
	ADI_SPI_TRANSCEIVER transceiver;

	for(uint8_t i=0; i<rx_bytes;++i)
		spiRxBuffer[i] = 0xdd; // some value to initialize it.

	transceiver.TransmitterBytes = tx_bytes + 1;
	transceiver.ReceiverBytes = rx_bytes + 1;
	transceiver.pTransmitter = spiTxBuffer;
	transceiver.pReceiver = spiRxBuffer;
	transceiver.nTxIncrement = 1;
	transceiver.nRxIncrement = 1;
	transceiver.bDMA = false;
	transceiver.bRD_CTL = false;

	spiTxBuffer[0] = address | 0x80;
	for(uint8_t i=1; i<(tx_bytes + 1); ++i)
	{
		spiTxBuffer[i] = data[i - 1];
	}

    if (ADI_SPI_SUCCESS != (result = adi_spi_MasterReadWrite(spiDevice, &transceiver)))
    {
        return result;
    }

	return ADI_SPI_SUCCESS;
}

/**
 * @brief    SPI read a single byte.
 *
 * Reads a single byte from the slave device.
 *
 * @param[in] address Address of the slave
 * @param[in] data Pointer to the variable which will hold the returned byte.
 */
ADI_SPI_RESULT spi_read_byte(uint8_t address, uint8_t* ret)
{
	ADI_SPI_RESULT result;
	ADI_SPI_TRANSCEIVER transceiver;

	for(uint8_t i=0; i<0x2;++i)
		spiRxBuffer[i] = 0xdd; // some value to initialize it.

	transceiver.TransmitterBytes = 2;
	transceiver.ReceiverBytes = 2;
	transceiver.pTransmitter = spiTxBuffer;
	transceiver.pReceiver = spiRxBuffer;
	transceiver.nTxIncrement = 1;
	transceiver.nRxIncrement = 1;
	transceiver.bDMA = false;
	transceiver.bRD_CTL = false;

	*spiTxBuffer = address;
	*(spiTxBuffer + 1) = 0x00; // additional byte for padding (during this byte the response comes on the SPI bus). It keeps the clock going

	if (ADI_SPI_SUCCESS != (result = adi_spi_MasterReadWrite(spiDevice, &transceiver)))
	{
		return result;
	}

	// DEBUG (added to log SPI data)
	//printf("RX: %d\n", transceiver.pReceiver[0]);
	//printf("RX: %d\n", transceiver.pReceiver[1]);

	*ret = spiRxBuffer[1];

	return ADI_SPI_SUCCESS;
}



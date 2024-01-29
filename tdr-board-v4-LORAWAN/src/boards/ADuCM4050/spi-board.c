/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
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
#include "board.h"
#include "gpio.h"
#include "spi-board.h"
#include "board-config.h"
#include "gpio-board.h"

#define SPI_DEVICE_NUM				0U
#define BUFFERSIZE					255

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
static uint8_t overtx[BUFFERSIZE] ADI_ALIGNED_ATTRIBUTE(2);


/* Receieve data buffer a */
ADI_ALIGNED_PRAGMA(2)
static uint8_t overrx[BUFFERSIZE] ADI_ALIGNED_ATTRIBUTE(2);

/*
 * Lucas:
 * Edited to work with ADuCM4050.
 * The code is a bit different from the example of the B-L072Z-LRWAN1 board that we
 * mostly use for inspiration. We do not use the SpiFormat function. Instead we just
 * initialize the Spi as we want from the start in this function. We have taken inspiration
 * from the spi-loopback example provided by ADI.
 * We do not use the SpiFrequency function either. Instead we just set the bitrate from the
 * beginning.
 */
void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
	ADI_SPI_RESULT eResult;


	CRITICAL_SECTION_BEGIN( );


	 /* Initialize SPI */
	eResult = adi_spi_Open(SPI_DEVICE_NUM, &spiDeviceMemory, ADI_SPI_MEMORY_SIZE, &spiDevice);
	DEBUG_RESULT("Failed to init SPI driver",eResult,ADI_SPI_SUCCESS);

	eResult = adi_spi_SetBitrate(spiDevice, 2500000);
	DEBUG_RESULT("Failed to set Bitrate",eResult,ADI_SPI_SUCCESS);

	/* Set IRQMODE. In this case we are setting it to the default value  */
	/* This code sequence is just calling out the fact that this API would be required  */
	/* for short bursts (less than the size of the FIFO) in PIO (interrupt) mode        */
	eResult = adi_spi_SetIrqmode(spiDevice, 0u);
	DEBUG_RESULT("Failed to set Irqmode",eResult,ADI_SPI_SUCCESS);

	/* set the chip select */
	eResult = adi_spi_SetChipSelect(spiDevice, ADI_SPI_CS0);
	DEBUG_RESULT("Failed to set the chip select",eResult,ADI_SPI_SUCCESS);

	eResult = adi_spi_SetClockPhase(spiDevice, false);

	eResult = adi_spi_SetClockPolarity(spiDevice, false);

	eResult = adi_spi_SetContinuousMode(spiDevice, true);

	//Only for debug
//	eResult = adi_spi_SetLoopback(hDevice, true);
//	DEBUG_RESULT("Failed to set internal loopback mode",eResult,ADI_SPI_SUCCESS);

	//Set parameter of the provided object.
	//obj->SpiId = spiId;

	//Initialize spi pins.
	GpioInit(&obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, GPIO_MUL1);
	GpioInit(&obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, GPIO_MUL1);
	GpioInit(&obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, GPIO_MUL1);
	if(nss != NC) {
		GpioInit(&obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, GPIO_MUL1);
	}

	CRITICAL_SECTION_END( );
}

void SpiDeInit( Spi_t *obj )
{
	ADI_SPI_RESULT eResult;

	/* Close the SPI device  */
	eResult = adi_spi_Close(spiDevice);
	DEBUG_RESULT("Failed to uninit SPI driver",eResult,ADI_SPI_SUCCESS);
}

/*
 * Lucas:
 * We dont really need this. It is used to configure the SPI communication in the board
 * but we just configure it how we want from the start in the SpiInit function.
 */
void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{

}

/*
 * Lucas:
 * We dont really need this. It is used to set the bitrate/frequency for the SPI communication
 * but we just configure it how we want from the start in the SpiInit function.
 */
void SpiFrequency( Spi_t *obj, uint32_t hz )
{

}

/*
 * Lucas (22-10-23):
 * Function to read and write data over SPI. We use the built-in
 * adi_spi_MasterReadWrite to read and write a single byte. For every run,
 * the function writes a byte and receives a single byte. The SX1276WriteBuffer
 * and SX1276ReadBufferfunction in sx1276.c controls what data is actually sent.
 * Read the comment there to understand how the SPI communication works!
 *
 * The overtx and overrx are the transmission and receive buffers respectively.
 * Some dummy data is inserted into them to begin with. Afterwards, the byte to
 * transmit is inserted into the first index of the transmission array, overtx.
 * The received data is returned after calling the adi_spi_MasterReadWrite function.
 * The function returns the first element in the receive array, overrx, since we
 * always receive a single byte.
 */
uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
	if(obj == NULL )
	{
	    return 0;
	}

	ADI_SPI_RESULT result;
	ADI_SPI_TRANSCEIVER transceive;

	/* initialize both the RX and TX buffers */
	for (unsigned int i = 0u; i < BUFFERSIZE; i++) {
	    overtx[i] = 0;
	    overrx[i] = (uint8_t)0xdd;
	}

	overtx[0] = (uint8_t)outData;

	/* link transceive data size to the remaining count */
	transceive.TransmitterBytes = 1;

	/* link transceive data size to the remaining count */
	transceive.ReceiverBytes = 1;

	/* initialize data attributes */
	transceive.pTransmitter = overtx;
	transceive.pReceiver = overrx;

	/* auto increment both buffers */
	transceive.nTxIncrement = 1;
	transceive.nRxIncrement = 1;
	transceive.bDMA = false;
	transceive.bRD_CTL = false;

	if (ADI_SPI_SUCCESS != (result = adi_spi_MasterReadWrite(spiDevice, &transceive)))
	{
		printf("Failed: %d\n", result);
	    return result;
	}

	return overrx[0];
}



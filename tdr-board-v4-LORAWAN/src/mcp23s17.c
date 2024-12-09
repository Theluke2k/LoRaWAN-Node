#include "mcp23s17.h"
#include "adi_spi.h"
#include <stdlib.h>

// Local SPI handle managed within the source
static ADI_SPI_HANDLE spiHandle;

// Local array to map device indices to chip select lines
static ADI_SPI_CHIP_SELECT device_cs[] = {
    MCP23S17_CS0, // CS for device 0
    MCP23S17_CS1, // CS for device 1
    MCP23S17_CS2  // CS for device 2
};

// Function to initialize MCP23S17 outputs
static ADI_SPI_RESULT mcp23s17_initialize_outputs(ADI_SPI_HANDLE hSpi, uint8_t cs) {
    uint8_t opcode = (0x40 | ((2 & 0x07) << 1));
    uint8_t txData[4] = {
 		   // Op, addr, dataA, dataB
 		   opcode, 0x00, 0x00, 0x00};

    ADI_SPI_TRANSCEIVER transceiver = {
        .pTransmitter = txData,
        .pReceiver = NULL,
        .TransmitterBytes = 4,
        .ReceiverBytes = 0,
        .nTxIncrement = 1,
        .nRxIncrement = 0,
        .bDMA = false,
        .bRD_CTL = false
    };

    adi_spi_SetChipSelect(hSpi, cs);
    return adi_spi_MasterReadWrite(hSpi, &transceiver);
}

// Function to set MCP23S17 outputs
static ADI_SPI_RESULT mcp23s17_set_outputs(ADI_SPI_HANDLE hSpi, uint8_t cs, uint16_t states) {
    uint8_t opcode = 0x40;
    uint8_t portA = (uint8_t)(states & 0xFF);
    uint8_t portB = (uint8_t)((states >> 8) & 0xFF);

    uint8_t txData[4] = { // !! Change back length to 6.
        //opcode, 0x14, portA, // OLATA register: Set PORTA outputs
        //opcode, 0x15, portB  // OLATB register: Set PORTB outputs
    	opcode, 0x12, portA, portB
    };

    ADI_SPI_TRANSCEIVER transceiver = {
        .pTransmitter = txData,
        .pReceiver = NULL,
        .TransmitterBytes = 4,
        .ReceiverBytes = 0,
        .nTxIncrement = 1,
        .nRxIncrement = 0,
        .bDMA = false,
        .bRD_CTL = false
    };

    adi_spi_SetChipSelect(hSpi, cs);
    return adi_spi_MasterReadWrite(hSpi, &transceiver);
}

static ADI_SPI_RESULT mcp23s17_read(ADI_SPI_HANDLE hSpi, uint8_t cs, uint8_t addr, uint8_t regAddr, uint8_t *value) {
    uint8_t opcode = 0x40 | 0x01; // Set the read bit (bit 0)
    uint8_t txData[3] = {opcode, regAddr, 0x00};
    uint8_t rxData[3] = {0x00, 0x00, 0x00};

    ADI_SPI_TRANSCEIVER transceiver = {
        .pTransmitter = txData,
        .pReceiver = rxData,
        .TransmitterBytes = 3,
        .ReceiverBytes = 3,
        .nTxIncrement = 1,
        .nRxIncrement = 1,
        .bDMA = false,
        .bRD_CTL = false
    };

    adi_spi_SetChipSelect(hSpi, cs);
    if (adi_spi_MasterReadWrite(hSpi, &transceiver) != ADI_SPI_SUCCESS) {
        return ADI_SPI_FAILURE;
    }

    *value = rxData[2]; // The second byte contains the register value
    return ADI_SPI_SUCCESS;
}

int mcp23s17_read_register(uint8_t deviceIndex, uint8_t regAddr, uint8_t *value) {
    if (deviceIndex >= 3 || value == NULL) {
        return -1; // Invalid device index or null pointer
    }

    if (mcp23s17_read(spiHandle, device_cs[deviceIndex], DEVICE_ADDR, regAddr, value) != ADI_SPI_SUCCESS) {
        return -1;
    }

    return 0;
}

int mcp23s17_initialize_all(void) {
    ADI_SPI_RESULT result;

    // Initialize SPI
    result = adi_spi_Open(SPI_BUS_ID, malloc(ADI_SPI_MEMORY_SIZE), ADI_SPI_MEMORY_SIZE, &spiHandle);
    if (result != ADI_SPI_SUCCESS) {
        return -1;
    }

    if (adi_spi_SetIrqmode(spiHandle, 0) != ADI_SPI_SUCCESS) {
        return ADI_SPI_FAILURE;
    }

    // Configure SPI settings
    result = adi_spi_SetMasterMode(spiHandle, true);
    if (result != ADI_SPI_SUCCESS) return -1;

    result = adi_spi_SetBitrate(spiHandle, SPI_BITRATE);
    if (result != ADI_SPI_SUCCESS) return -1;

    result = adi_spi_SetClockPolarity(spiHandle, false); // CPOL=0
    if (result != ADI_SPI_SUCCESS) return -1;

    result = adi_spi_SetClockPhase(spiHandle, false); // CPHA=0
    if (result != ADI_SPI_SUCCESS) return -1;

    adi_spi_SetContinuousMode(spiHandle, true);

    // Initialize all MCP23S17 devices
    for (uint8_t i = 0; i < 3; i++) {
    	if (i < 2) {
    		result = mcp23s17_initialize_outputs(spiHandle, device_cs[i]);
    	}

    }

    return 0;
}

int mcp23s17_set_device_outputs(uint8_t deviceIndex, uint16_t states) {
    if (deviceIndex >= 3) {
        return -1; // Invalid device index
    }

    if (mcp23s17_set_outputs(spiHandle, device_cs[deviceIndex], states) != ADI_SPI_SUCCESS) {
        return -1;
    }

    return 0;
}

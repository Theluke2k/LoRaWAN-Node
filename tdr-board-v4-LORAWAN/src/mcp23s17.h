#ifndef MCP23S17_H
#define MCP23S17_H

#include <stdint.h>

// SPI Configuration
#define DEVICE_ADDR 2                  // MCP23S17 hardware address
#define SPI_BUS_ID 2
#define SPI_BITRATE 2500000            // SPI clock speed in Hz
#define MCP23S17_CS0 ADI_SPI_CS0       // Chip Select for device 0
#define MCP23S17_CS1 ADI_SPI_CS1       // Chip Select for device 1
#define MCP23S17_CS2 ADI_SPI_CS3       // Chip Select for device 2

// Device indices
#define MCP23S17_DEVICE_0 0
#define MCP23S17_DEVICE_1 1
#define MCP23S17_DEVICE_2 2

/**
 * Initializes the SPI interface and all MCP23S17 devices as outputs.
 *
 * @return int indicating success (0) or failure (-1).
 */
int mcp23s17_initialize_all(void);

/**
 * Sets the output states of a specific MCP23S17 device.
 *
 * @param deviceIndex Index of the MCP23S17 device (0, 1, or 2).
 * @param states      16-bit value representing the state of all 16 pins (1=high, 0=low).
 * @return int indicating success (0) or failure (-1).
 */
int mcp23s17_set_device_outputs(uint8_t deviceIndex, uint16_t states);
int mcp23s17_read_register(uint8_t deviceIndex, uint8_t regAddr, uint8_t *value);
void mcp23s16_spi_de_init();
#endif // MCP23S17_H

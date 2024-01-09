/*
 * lora.h
 *
 *  Created on: 25. mar. 2023
 *      Author: au703540
 */

#ifndef SPI_AU_H_
#define SPI_AU_H_

#include <adi_spi.h>

/*
 * SPI device number.
 */
#define SPI_DEVICE_NUM				0U
/*
 * SPI buffer size. Can be kept like this.
 */
#define SPI_BUFFER_SIZE				255


ADI_SPI_RESULT spi_init();
ADI_SPI_RESULT spi_write_byte(uint8_t address, uint8_t data);
ADI_SPI_RESULT spi_write(uint8_t tx_bytes, uint8_t rx_bytes, uint8_t address, uint8_t* data);
ADI_SPI_RESULT spi_read_byte(uint8_t address, uint8_t* ret);


#endif /* SPI_AU_H_ */

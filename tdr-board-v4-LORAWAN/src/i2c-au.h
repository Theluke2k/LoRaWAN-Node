/*
 * i2c.h
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#ifndef I2C_AU_H_
#define I2C_AU_H_

#include <inttypes.h>

/*
 * Address from the datasheet.
 */
#define HONEYWELL_ADDR                  0x27

void i2c_init();
void i2c_write_blank(uint8_t address);
void i2c_read(uint8_t address, uint8_t num_bytes, uint8_t* ret_buffer);


#endif /* I2C_AU_H_ */

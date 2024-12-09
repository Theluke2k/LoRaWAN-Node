/*
 * honeywell.h
 *
 *  Created on: 19. nov. 2024
 *      Author: au748936
 */

#ifndef HONEYWELL_H_
#define HONEYWELL_H_



#include <inttypes.h>
#include <adi_i2c.h>
#include <adi_gpio.h>

/*
 * Address from the datasheet.
 */
void i2c_de_init();
void i2c_init();
void i2c_write_blank(uint8_t address);
void i2c_read(uint8_t address, uint8_t num_bytes, uint8_t* ret_buffer);
double get_honeywell_temp(uint16_t temp);
double get_honeywell_rh(uint16_t rh);
uint8_t honeywell_read(uint16_t vals[2]);
void honeywell_read_and_process(double *rh, double *temp);
void honeywell_read_and_process_raw(uint16_t *raw_rh, uint16_t *raw_temp);

#define HONEYWELL_ADDR                  0x27
#define PWR_HONEYWELL_PORT				ADI_GPIO_PORT2
#define PWR_HONEYWELL_PIN				ADI_GPIO_PIN_3


#endif /* HONEYWELL_H_ */

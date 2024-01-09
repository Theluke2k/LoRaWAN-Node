/*
 * flash.h
 *
 *  Created on: 29. jun. 2023
 *      Author: au703540
 */

#ifndef FLASH_H_
#define FLASH_H_
#include <inttypes.h>

#define FLASH_DEVICE_NUM 					0u

void flash_init();
void flash_deinit();
void flash_write(uint32_t* address, uint32_t* data, uint32_t num_words);


#endif /* FLASH_H_ */

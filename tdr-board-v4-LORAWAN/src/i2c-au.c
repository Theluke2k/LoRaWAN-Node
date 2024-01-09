/*
 * i2c.c
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */
#include "i2c-au.h"

#include <adi_i2c.h>

/*
 * I2C device (ADI driver).
 */
static ADI_I2C_HANDLE i2cDevice;
/*
 * I2C device memory (ADI driver).
 */
static uint8_t i2cMem[ADI_I2C_MEMORY_SIZE];

/*
 * @brief I2C init function.
 *
 * Nothing out of ordinary, sets the speed to 400000Hz, which works well with the HIH6000 temp and humidity sensor.
 *
 * Follows the example code from ADI.
 */
void i2c_init()
{
	adi_i2c_Open(0, &i2cMem, ADI_I2C_MEMORY_SIZE, &i2cDevice);
	adi_i2c_Reset(i2cDevice);
	adi_i2c_SetBitRate(i2cDevice, 400000);
}

/*
 * @brief Writes a blank byte to the I2C device of a given address.
 *
 * @param[in] address Address of the I2C device (usually in hex form in datasheets).
 *
 * This function is exclusively used to trigger the HIH6000 to produce a result. After about 60us the data can be read from the sensor.
 *
 */
void i2c_write_blank(uint8_t address)
{
	ADI_I2C_TRANSACTION xfr;
	ADI_I2C_RESULT result = ADI_I2C_SUCCESS;
    uint32_t hwErrors;

    adi_i2c_SetSlaveAddress(i2cDevice, HONEYWELL_ADDR);

    xfr.pPrologue       = NULL;
    xfr.nPrologueSize   = 0;
    xfr.pData           = NULL;
    xfr.nDataSize       = 0;
    xfr.bReadNotWrite   = false;
    xfr.bRepeatStart    = false;

    result = adi_i2c_ReadWrite(i2cDevice, &xfr, &hwErrors);
    if (result) {
    	//
    }
}

/*
 * @brief Reads a number of bytes from the I2C device.
 *
 * @param[in] address Address of the I2C device (usually in hex form in datasheets).
 *
 *
 */
void i2c_read(uint8_t address, uint8_t num_bytes, uint8_t* ret_buffer)
{
	ADI_I2C_TRANSACTION xfr;
	ADI_I2C_RESULT result = ADI_I2C_SUCCESS;
    uint32_t hwErrors;
    xfr.pPrologue       = NULL;
    xfr.nPrologueSize   = 0;
    xfr.pData           = ret_buffer;
    xfr.nDataSize       = num_bytes;
    xfr.bReadNotWrite   = true;
    xfr.bRepeatStart    = false;

    result = adi_i2c_ReadWrite(i2cDevice, &xfr, &hwErrors);
    if (result) {
        //
    }
}

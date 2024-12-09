/*
 * honeywell.c
 *
 *  Created on: 19. nov. 2024
 *      Author: au748936
 */


#include "honeywell.h"

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

void i2c_de_init(){
	adi_i2c_Close(&i2cDevice);
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

uint8_t honeywell_read(uint16_t* vals)
 {
	uint8_t holder[4] = {0};
    adi_gpio_SetLow(PWR_HONEYWELL_PORT, PWR_HONEYWELL_PIN);

    uint32_t delay_val = 96000; // = 60ms
    while(--delay_val){}; // one value = 625ns of delay

    i2c_write_blank(HONEYWELL_ADDR);

    delay_val = 64000; // = 40ms
    while(--delay_val){}; // one value = 625ns of delay

    i2c_read(HONEYWELL_ADDR, 4, holder);

    adi_gpio_SetHigh(PWR_HONEYWELL_PORT, PWR_HONEYWELL_PIN);

    vals[0] = (((holder[0] & 0b00111111) << 8) | (holder[1]));
    vals[1] = (holder[2] << 6) | (((holder[3] & 0b11111100)) >> 2);
    return 0;
}


double get_honeywell_temp(uint16_t temp)
{
    return ((double)temp*165)/16382 - 40;
}

/*
 * @Brief Returns the RH based on the HIH6000 response.
 *
 * @param[in] temp Return value of HIH6000 related to RH.
 *
 * Formula from the manufacturer.
 */
double get_honeywell_rh(uint16_t rh)
{
    return ((double)rh*100)/16382;
}


#include <stdint.h>

// Function to read from the Honeywell sensor, process data, and return RH and Temp
void honeywell_read_and_process(double *rh, double *temp)
{
    uint8_t holder[4] = {0};
    uint16_t raw_rh, raw_temp;

    // Power up the sensor
    adi_gpio_SetLow(PWR_HONEYWELL_PORT, PWR_HONEYWELL_PIN);

    // Delay for power stabilization (60ms)
    uint32_t delay_val = 96000; // = 60ms
    while (--delay_val) {}; // Each value = 625ns of delay

    // Initiate I2C communication with the Honeywell sensor
    i2c_write_blank(HONEYWELL_ADDR);

    // Delay for data readiness (40ms)
    delay_val = 64000; // = 40ms
    while (--delay_val) {}; // Each value = 625ns of delay

    // Read 4 bytes of data from the Honeywell sensor
    i2c_read(HONEYWELL_ADDR, 4, holder);

    // Power down the sensor
    adi_gpio_SetHigh(PWR_HONEYWELL_PORT, PWR_HONEYWELL_PIN);

    // Parse raw data
    raw_rh = (((holder[0] & 0b00111111) << 8) | (holder[1]));
    raw_temp = (holder[2] << 6) | ((holder[3] & 0b11111100) >> 2);

    // Calculate RH and Temperature based on manufacturer formulas
    *rh = ((double)raw_rh * 100) / 16382;
    *temp = ((double)raw_temp * 165) / 16382 - 40;
}

void honeywell_read_and_process_raw(uint16_t *raw_rh, uint16_t *raw_temp)
{
    uint8_t holder[4] = {0};

    // Power up the sensor
    adi_gpio_SetLow(PWR_HONEYWELL_PORT, PWR_HONEYWELL_PIN);

    // Delay for power stabilization (60ms)
    uint32_t delay_val = 96000; // = 60ms
    while (--delay_val) {}; // Each value = 625ns of delay

    // Initiate I2C communication with the Honeywell sensor
    i2c_write_blank(HONEYWELL_ADDR);

    // Delay for data readiness (40ms)
    delay_val = 64000; // = 40ms
    while (--delay_val) {}; // Each value = 625ns of delay

    // Read 4 bytes of data from the Honeywell sensor
    i2c_read(HONEYWELL_ADDR, 4, holder);

    // Power down the sensor
    adi_gpio_SetHigh(PWR_HONEYWELL_PORT, PWR_HONEYWELL_PIN);

    // Parse raw data
    *raw_rh = (((holder[0] & 0b00111111) << 8) | (holder[1]));
    *raw_temp = (holder[2] << 6) | ((holder[3] & 0b11111100) >> 2);

}

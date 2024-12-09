/*
 * au_sensor_collection.c
 *
 *  Created on: 5. dec. 2024
 *      Author: au748936
 */

#include "gpio-au.h"
#include "adi_initialize.h"
#include "mcp23s17.h"
#include "adc-au.h"
#include "honeywell.h"
#include "uart-au.h"
#include "tdr_interface.h"
#include "spi_temp_sensor.h"
#include "au_sensor_collection.h"
#include "honeywell.h"

void init_sensor_collection(){
	digital_pin_init();
	ADC_Init();
    i2c_init();

    // Pin mux init. Check if this works with Lucas code correctly!
	adi_initComponents();

	adi_gpio_SetLow(SHIFTREG_NRST_PORT, SHIFTREG_NRST_PIN);
	delay_us(100);
	adi_gpio_SetHigh(SHIFTREG_NRST_PORT, SHIFTREG_NRST_PIN);

	mcp23s17_initialize_all();
}

void de_init_sensor_collection(){
	ADC_Cleanup();
	i2c_de_init();
}


void get_all_sensor_data(sensor_data_struct *sensor_data) {

    uint16_t adc_results[3];
    uint16_t thermistor_values[4];
    uint32_t tdr_pulsewidth_raw;
	uint16_t honeywell_humidity;
	uint16_t honeywell_temperature;

	honeywell_read_and_process_raw(&honeywell_humidity, &honeywell_temperature);
	sensor_data->honeywell_humidity    = honeywell_humidity;
	sensor_data->honeywell_temperature = honeywell_temperature;

	// Read temp sensor
    uint8_t temp_sensor_regA, temp_sensor_regB;
	uint8_t temp_sensor_rst = 1;
    uint16_t temp_sensor_output; // 12-bit
	while (temp_sensor_rst) {
		mcp23s17_read_register(MCP23S17_DEVICE_2, 18, &temp_sensor_regA);
		mcp23s17_read_register(MCP23S17_DEVICE_2, 19, &temp_sensor_regB);
        temp_sensor_rst = temp_sensor_regA & 0x01;
	}

    temp_sensor_output = 0;
    temp_sensor_output |= ((temp_sensor_regB >> 0) & 0x01) << 12; // regB[0] -> output[12]
    temp_sensor_output |= ((temp_sensor_regB >> 1) & 0x01) << 11; // regB[1] -> output[11]
    temp_sensor_output |= ((temp_sensor_regB >> 2) & 0x01) << 10; // regB[2] -> output[10]
    temp_sensor_output |= ((temp_sensor_regB >> 3) & 0x01) << 9;  // regB[3] -> output[9]
    temp_sensor_output |= ((temp_sensor_regB >> 4) & 0x01) << 8;  // regB[4] -> output[8]
    temp_sensor_output |= ((temp_sensor_regB >> 5) & 0x01) << 7;  // regB[5] -> output[7]
    temp_sensor_output |= ((temp_sensor_regB >> 6) & 0x01) << 6;  // regB[6] -> output[6]

    temp_sensor_output |= ((temp_sensor_regA >> 6) & 0x01) << 5;  // regA[6] -> output[5]
    temp_sensor_output |= ((temp_sensor_regA >> 5) & 0x01) << 4;  // regA[5] -> output[4]
    temp_sensor_output |= ((temp_sensor_regA >> 4) & 0x01) << 3;  // regA[4] -> output[3]
    temp_sensor_output |= ((temp_sensor_regA >> 3) & 0x01) << 2;  // regA[3] -> output[2]
    temp_sensor_output |= ((temp_sensor_regA >> 2) & 0x01) << 1;  // regA[2] -> output[1]
    temp_sensor_output |= ((temp_sensor_regA >> 1) & 0x01) << 0;  // regA[1] -> output[0]

    sensor_data->bjt_temperature = temp_sensor_output;

    // Power on TDR (active low)
	adi_gpio_SetLow(TDR_PWR_PORT, TDR_PWR_PIN);
	delay_us(100);

	tdr_read_thermistors(thermistor_values);
    sensor_data->tdr_thermistor_0 = thermistor_values[0];
    sensor_data->tdr_thermistor_1 = thermistor_values[1];
    sensor_data->tdr_thermistor_2 = thermistor_values[2];
    sensor_data->tdr_thermistor_3 = thermistor_values[3];

    tdr_pulsewidth_raw = tdr_start_measurement_digital();
    sensor_data->tdr_pulsewidth = tdr_pulsewidth_raw;

    tdr_de_init();

    // Disable TDR power
	adi_gpio_SetHigh(TDR_PWR_PORT, TDR_PWR_PIN);

	// Comment out in case we use the vbat sample from Lucas
	adi_gpio_SetHigh(VBAT_READ_EN_PORT, VBAT_READ_EN_PIN);
	ADC_SampleChannels(adc_results);
	sensor_data->supercap_voltage = adc_results[2];
	adi_gpio_SetLow(VBAT_READ_EN_PORT, VBAT_READ_EN_PIN);

}

void pack_sensor_data(const sensor_data_struct *data, uint8_t *packed_data) {
    uint64_t temp1 = 0, temp2 = 0;

    // Pack the first 64 bits
    temp1 |= (uint64_t)data->honeywell_temperature;              // 16 bits
    temp1 |= (uint64_t)data->honeywell_humidity << 16;           // Next 16 bits
    temp1 |= (uint64_t)(data->bjt_temperature & 0xFFF) << 32;    // 12 bits
    temp1 |= (uint64_t)(data->tdr_thermistor_0 & 0xFFF) << 44;   // 12 bits
    temp1 |= (uint64_t)(data->tdr_thermistor_1 & 0xFF) << 56;    // Lower 8 bits of 12 bits

    // Write first 8 bytes
    packed_data[0] = temp1 & 0xFF;
    packed_data[1] = (temp1 >> 8) & 0xFF;
    packed_data[2] = (temp1 >> 16) & 0xFF;
    packed_data[3] = (temp1 >> 24) & 0xFF;
    packed_data[4] = (temp1 >> 32) & 0xFF;
    packed_data[5] = (temp1 >> 40) & 0xFF;
    packed_data[6] = (temp1 >> 48) & 0xFF;
    packed_data[7] = (temp1 >> 56) & 0xFF;

    // Pack the next 64 bits
    temp2 |= (uint64_t)((data->tdr_thermistor_1 >> 8) & 0xF);    // Upper 4 bits of 12 bits
    temp2 |= (uint64_t)(data->tdr_thermistor_2 & 0xFFF) << 4;    // 12 bits
    temp2 |= (uint64_t)(data->tdr_thermistor_3 & 0xFFF) << 16;   // Next 12 bits
    temp2 |= (uint64_t)(data->tdr_pulsewidth & 0xFFFFF) << 28;   // 20 bits
    temp2 |= (uint64_t)(data->supercap_voltage & 0xFFF) << 48;   // 12 bits

    // Write the next 8 bytes
    packed_data[8] = temp2 & 0xFF;
    packed_data[9] = (temp2 >> 8) & 0xFF;
    packed_data[10] = (temp2 >> 16) & 0xFF;
    packed_data[11] = (temp2 >> 24) & 0xFF;
    packed_data[12] = (temp2 >> 32) & 0xFF;
    packed_data[13] = (temp2 >> 40) & 0xFF;
    packed_data[14] = (temp2 >> 48) & 0xFF;
    packed_data[15] = (temp2 >> 56) & 0xFF;
}

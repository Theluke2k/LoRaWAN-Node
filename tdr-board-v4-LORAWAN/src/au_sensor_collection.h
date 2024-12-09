/*
 * au_sensor_collection.h
 *
 *  Created on: 5. dec. 2024
 *      Author: au748936
 */

#ifndef AU_SENSOR_COLLECTION_H_
#define AU_SENSOR_COLLECTION_H_

#define ADC_CHANNEL_INT_OUT     ADI_ADC_CHANNEL_1
#define ADC_CHANNEL_THERM_READ  ADI_ADC_CHANNEL_2
#define ADC_CHANNEL_VBAT_SAMPLE ADI_ADC_CHANNEL_3

typedef struct {
    uint16_t honeywell_temperature; // 16 bits
    uint16_t honeywell_humidity;    // 16 bits

    uint16_t bjt_temperature;  // 12 bits
    uint16_t tdr_thermistor_0; // 12 bits
    uint16_t tdr_thermistor_1; // 12 bits
    uint16_t tdr_thermistor_2; // 12 bits
    uint16_t tdr_thermistor_3; // 12 bits

    uint32_t tdr_pulsewidth;   // 20 bits
    uint32_t supercap_voltage; // 12 bits
} sensor_data_struct;


void pack_sensor_data(const sensor_data_struct *data, uint8_t *packed_data);

void get_all_sensor_data(sensor_data_struct *sensor_data);

void init_sensor_collection();

void de_init_sensor_collection();


#endif /* AU_SENSOR_COLLECTION_H_ */

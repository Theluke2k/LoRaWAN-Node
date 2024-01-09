/*
 * sensors.h
 *
 *  Created on: 30. maj 2022
 *      Author: au703540
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include <inttypes.h>

/*
 * Thermistor base resistance is 5k. Normally every thermistor should be characterized in special environment (a small climate chamber) but we don't really have
 * access to it as of now.
 */
#define THERMISTOR_BASE_R               5000.0

/*
 * Size of the ratio table for Vishay thermistor.
 */
#define THERMISTORS_RATIO_TABLE_SIZE    206

/*
 * Epsilon for comparisons between the measured value and the ratio table. It can be lowered if necessary but if too restrictive, the result may not be obtained.
 * This value was checked and in works well enough with it.
 */
#define THERMISTORS_RATIO_EPS           2

/*
 * By default the currents for the integrators are equal to 1mA. This can, however, vary from board to board.
 */
#define INT1_CURRENT                    0.001 // A
#define INT2_CURRENT                    0.001 // A

/*
 * Thermistor current can fluctuate from board to board so it should be measured but one can assume 100uA.
 */
#define THERMISTOR_CURRENT              0.0001 // A

/*
 * The capacitance for the ingegrator capacitors is 220pF by design, but can be changed when larger range of TDR than 24m is needed. THE CHANGE NEEDS TO BE REFLECTED
 * HERE.
 */
#define INT_CAPACITANCE                 0.00000000022 // 220pF

/*
 * Enum for the thermistor numbers.
 */
typedef enum
{
    THERMISTOR1,
    THERMISTOR2,
    THERMISTOR3,
    THERMISTOR4
}thermistor;

void select_thermistor(thermistor therm);
uint8_t therm_get_temp_from_ratio(uint16_t ratio);
void therm_read_temp(int16_t* val, thermistor therm);
double get_honeywell_temp(uint16_t temp);
double get_honeywell_rh(uint16_t rh);
uint8_t honeywell_read(uint16_t vals[2]);

#endif /* SENSORS_H_ */

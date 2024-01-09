/*
 * tdr.h
 *
 *  Created on: 1. jun. 2022
 *      Author: au703540
 */

#ifndef TDR_H_
#define TDR_H_
#include <inttypes.h>

/*
 * Enum for different refernce levels. For explanation on the values here, f.e. 0.8*VDD/2, refer to schematics and report.
 */
typedef enum
{
    REF1_0_9_REF2_1_1,
    REF1_0_85_REF2_1_15,
    REF1_0_8_REF2_1_2,
    REF1_0_9_DELAY_REF2_1_25
}compref;


void select_comparator_reference(compref comp_ref);
void line_stimulation(uint16_t integrators_data[2]);
void read_integrators(uint16_t vals[2]);
double fitting_function_int1(uint16_t val);

#endif /* TDR_H_ */

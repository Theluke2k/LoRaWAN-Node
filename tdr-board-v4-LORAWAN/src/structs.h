/*
 * structs.h
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <inttypes.h>

uint16_t packageNumber;

/*
 * TDR data struct definition. Holds information about all of the measurements done in the measurement phase.
 */
struct tdr_data{
    uint16_t int1_integer;
    uint16_t int2_integer;
    int8_t th1_temp; //-127 to 127
    int8_t th2_temp;
    int8_t th3_temp;
    int8_t th4_temp;
    uint8_t honey_rh_integer;
    int8_t honey_temp_integer; //-127 to 127 if we measure above 100 then it all will be dead anyway. The sensor, I mean.
}; //14B

#endif /* STRUCTS_H_ */

/*
 * sensors.c
 *
 *  Created on: 30. maj 2022
 *      Author: au703540
 */

#include "sensors.h"
#include "general_functions.h"
#include <adi_adc.h>
#include <stdlib.h>
#include <stdio.h>

#include "gpio-au.h"
#include "i2c-au.h"
#include "adc-au.h"

/*
 * Array of ratios (R_measured/R_nominal(25))*1000 from the thermistor datasheet. Refer to the datasheet of the thermistor (Vishay
    TFPTL15L5001FL2B).
 */
static const uint16_t THERMISTORS_RATIO_TABLE[THERMISTORS_RATIO_TABLE_SIZE]=
{
    702, 705, 708, 712, 715, 719, 722, 725, 729, 732, 736, 739, 743, 746, 749,
    753, 756, 760, 763, 767, 771, 774, 778, 781, 785, 788, 792, 796, 799, 803,
    806, 810, 814, 817, 821, 825, 828, 832, 836, 839, 843, 847, 851, 854, 858,
    862, 866, 869, 873, 877, 881, 885, 889, 892, 896, 900, 904, 908, 912, 916,
    920, 924, 927, 931, 935, 939, 943, 947, 951, 955, 959, 963, 967, 971, 975,
    980, 984, 988, 992, 996, 1000, 1004, 1008, 1012, 1017, 1021, 1025, 1029, 1033, 1037,
    1042, 1046, 1050, 1054, 1059, 1063, 1067, 1071, 1076, 1080, 1084, 1089, 1093, 1097, 1102,
    1106, 1110, 1115, 1119, 1124, 1128, 1133, 1137, 1141, 1146, 1150, 1155, 1159, 1164, 1168,
    1173, 1177, 1182, 1186, 1191, 1196, 1200, 1205, 1209, 1214, 1219, 1223, 1228, 1232, 1237,
    1242, 1246, 1251, 1256, 1261, 1265, 1270, 1275, 1280, 1284, 1289, 1294, 1299, 1303, 1308,
    1313, 1318, 1323, 1328, 1333, 1337, 1342, 1347, 1352, 1357, 1362, 1367, 1372, 1377, 1382,
    1387, 1392, 1397, 1402, 1407, 1412, 1417, 1422, 1427, 1432, 1437, 1442, 1448, 1453, 1458,
    1463, 1468, 1473, 1478, 1484, 1489, 1494, 1499, 1505, 1510, 1515, 1520, 1526, 1531, 1536,
    1541, 1547, 1552, 1557, 1563, 1568, 1574, 1579, 1584, 1590, 1595
};

/*
 * @Brief Reads the data from the HIH6000 via I2C and stores it in understandable form.
 *
 * This is described in the datasheet/application note regarding I2C protocol usage for this sensor. The values from vals can later be converted using the
 * get_honeywell_temp() and double get_honeywell_rh() functions to actual numerical representations.
 *
 * @param[in,out] vals Returned values from the HIH6000 are stored here.
 */
uint8_t honeywell_read(uint16_t* vals)
 {
	uint8_t holder[4] = {0};
    adi_gpio_SetHigh(MICRO_SENSOR_EN_PORT, MICRO_SENSOR_EN_PIN);

    uint32_t delay_val = 96000; // = 60ms
    while(--delay_val){}; // one value = 625ns of delay

    i2c_write_blank(HONEYWELL_ADDR);

    delay_val = 64000; // = 40ms
    while(--delay_val){}; // one value = 625ns of delay

    i2c_read(HONEYWELL_ADDR, 4, holder);

    adi_gpio_SetLow(MICRO_SENSOR_EN_PORT, MICRO_SENSOR_EN_PIN);

    vals[0] = (((holder[0] & 0b00111111) << 8) | (holder[1]));
    vals[1] = (holder[2] << 6) | (((holder[3] & 0b11111100)) >> 2);
    return 0;
}

/*
 * @Brief Selects 1 of 4 thermistors to be powered by the current source.
 *
 * This selects appropriate setting of the 4x1 switch (refer to schematics).
 *
 * @param[in] therm Thermistor number.
 */
void select_thermistor(thermistor therm)
{
	switch(therm) // easy, I hate it as well
	{
		case THERMISTOR1:
		{
			adi_gpio_SetLow(MICRO_TH_S0_PORT, MICRO_TH_S0_PIN);
			adi_gpio_SetLow(MICRO_TH_S1_PORT, MICRO_TH_S1_PIN);
			break;
		}
		case THERMISTOR2:
		{
			adi_gpio_SetHigh(MICRO_TH_S0_PORT, MICRO_TH_S0_PIN);
			adi_gpio_SetLow(MICRO_TH_S1_PORT, MICRO_TH_S1_PIN);
			break;
		}
		case THERMISTOR3:
		{
			adi_gpio_SetLow(MICRO_TH_S0_PORT, MICRO_TH_S0_PIN);
			adi_gpio_SetHigh(MICRO_TH_S1_PORT, MICRO_TH_S1_PIN);
			break;
		}
		case THERMISTOR4:
		{
			adi_gpio_SetHigh(MICRO_TH_S0_PORT, MICRO_TH_S0_PIN);
			adi_gpio_SetHigh(MICRO_TH_S1_PORT, MICRO_TH_S1_PIN);
			break;
		}
	}
}

/*
 * @Brief Returns the approximation of the temperature for the thermistor in degrees Celsius based on the ratio.
 *
 * Approximation as the ratio table specifies the values for integer values of temperature.
 *
 * @param[in] ratio Ratio from the THERMISTORS_RATIO_TABLE.
 */
uint8_t therm_get_temp_from_ratio(uint16_t ratio)
{
    uint8_t index = 255;
    uint16_t min_val = 65535;
    uint16_t diff = 65535;
    for(uint8_t i=0; i<THERMISTORS_RATIO_TABLE_SIZE; ++i)
    {
        diff = abs(ratio - THERMISTORS_RATIO_TABLE[i]);
        if(diff <= min_val)
        {
            min_val = diff;
            index = i;
        }
        else if(diff > min_val)
            return -55 + index;
    }
    return 0;
}


/*
 * @Brief Performs the temperature readout from the selected thermistor.
 *
 * @param[in, out] val Pointer to the variable which will hold the thermistor measurement result.
 * @param[in] therm Thermistor number.
 *
 * @note val is int16_t because the return values are in the range of -55 to 150 which is larger than range of int8_t (even if only slightly...).
 */
void therm_read_temp(int16_t* val, thermistor therm)
{
    select_thermistor(therm);

    adi_gpio_SetLow(MICRO_TH_EN_PORT, MICRO_TH_EN_PIN);
    ADC_SampleData(ADC_THERM_READ_CHANNEL, 255);
    ADC_SampleData(ADC_THERM_READ_CHANNEL, 255);
    uint16_t buffer = get_adc_data_buffer();

    uint16_t ratio = (uint16_t)((((double)buffer*ADC_VCC_REF)/ADC_MAX_VAL)/THERMISTOR_CURRENT/THERMISTOR_BASE_R*1000);
    *val = therm_get_temp_from_ratio(ratio);

    adi_gpio_SetHigh(MICRO_TH_EN_PORT, MICRO_TH_EN_PIN);
}

/*
 * @Brief Returns the temperature based on the HIH6000 response.
 *
 * @param[in] temp Return value of HIH6000 related to temperature.
 *
 * Formula from the manufacturer.
 */
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

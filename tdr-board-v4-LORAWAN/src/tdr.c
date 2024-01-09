/*
 * tdr.c
 *
 *  Created on: 1. jun. 2022
 *      Author: au703540
 */

#include "tdr.h"

#include "adc-au.h"
#include "general_functions.h"
#include "gpio-au.h"
#include "tdr-board-v4_config.h"

/*
 * @brief Selects the comparator reference level for the TDR.
 *
 * Those levels are used to start and stop the integration on the integrating capacitors. Those values steer the actual discrete switch (refer to the Schematics).
 *
 * @param[in] comp_ref Comparator reference ID.
 */
void select_comparator_reference(compref comp_ref)
{
	switch(comp_ref) // easy, I hate it as well
	{
		case REF1_0_9_REF2_1_1:
		{
			adi_gpio_SetLow(MICRO_REF_S0_PORT, MICRO_REF_S0_PIN);
			adi_gpio_SetLow(MICRO_REF_S1_PORT, MICRO_REF_S1_PIN);
			break;
		}
		case REF1_0_85_REF2_1_15:
		{
			adi_gpio_SetHigh(MICRO_REF_S0_PORT, MICRO_REF_S0_PIN);
			adi_gpio_SetLow(MICRO_REF_S1_PORT, MICRO_REF_S1_PIN);
			break;
		}
		case REF1_0_8_REF2_1_2:
		{
			adi_gpio_SetLow(MICRO_REF_S0_PORT, MICRO_REF_S0_PIN);
			adi_gpio_SetHigh(MICRO_REF_S1_PORT, MICRO_REF_S1_PIN);
			break;
		}
		case REF1_0_9_DELAY_REF2_1_25:
		{
			adi_gpio_SetHigh(MICRO_REF_S0_PORT, MICRO_REF_S0_PIN);
			adi_gpio_SetHigh(MICRO_REF_S1_PORT, MICRO_REF_S1_PIN);
			break;
		}
	}
}

/*
 * @brief Reads the voltage levels of the integrators.
 *
 * @param[in,out] vals Pointer to the array that will hold the ADC values related to the voltage levels on the capacitors.
 */
void read_integrators(uint16_t vals[2])
{
	ADC_SampleData(ADC_INTEGRATOR1_CHANNEL, 10);
	uint16_t buffer = get_adc_data_buffer();
    vals[0] = buffer;

	ADC_SampleData(ADC_INTEGRATOR2_CHANNEL, 10);
	buffer = get_adc_data_buffer();
    vals[1] = buffer;
}

/*
 * @brief Stimulates the line by sending a short pulse wave via transmission line.
 *
 * Precise delays needed to be achieved so the NOPs were used. They are dependent on how long it takes for the comparators and other parts of the measurement
 * system for TDR to turn on and be operational. The lengths of the delay were selected to work with EXACTLY those devices which are used on the boards. HOWEVER,
 * from board to board there may be fluctioations on how long it takes to turn on certain parts of the TDR system, ergo it would be the best to have this delay
 * variable, adjusted to each and every board. Values proposed here SHOULD work for all of the five v4 boards.
 *
 * To understand the stimulation procedure, refer to the TDR 1st report.
 *
 * @param[in,out] integrators_data Pointer to the array that will hold the ADC values related to the voltage levels on the capacitors.
 */
void line_stimulation(uint16_t integrators_data[2])
{
	adi_gpio_SetHigh(MICRO_RST_INT_PORT, MICRO_RST_INT_PIN); // 2.125us
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    adi_gpio_SetHigh(MICRO_COMP_ON_PORT, MICRO_COMP_ON_PIN); // 2.125us
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    //
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    adi_gpio_Toggle(MICRO_STM_START_PORT, MICRO_STM_START_PIN | MICRO_RST_INT_PIN); // RST_HIGH -> STM HIGH = 6us

    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    read_integrators(integrators_data);

    adi_gpio_SetLow(MICRO_COMP_ON_PORT, MICRO_COMP_ON_PIN);
    adi_gpio_SetLow(MICRO_STM_START_PORT, MICRO_STM_START_PIN); // STM start -> STM start low = 537.3us

    __NOP();
    __NOP();
}

/*
 * Normally the output of the integrators would be converted to meters but because there were not enough measurement points, this was very inaccurate. We switched
 * to the pure ADC values because at this stage (the board) we don't really care, we can just convert it ourselves.
 */
double fitting_function_int1(uint16_t val) //for INT1, INT2 needs to be derived accurately
{
    return TDR_LINEAR_A*((double)val) + TDR_LINEAR_B;
}

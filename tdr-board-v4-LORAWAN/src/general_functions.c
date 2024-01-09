/*
 * general_functions.c
 *
 *  Created on: 27. maj 2022
 *      Author: au703540
 */

#include <adi_pwr.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "general_functions.h"

#include "adc-au.h"
#include "gpio-au.h"
#include "i2c-au.h"
#include "tdr.h"
#include "rtc.h"
#include "sensors.h"
#include "xint.h"
#include "sx1276.h"
#include "shared.h"
#include "spi-au.h"
#include "tdr-board-v4_config.h"
#include "uart-au.h"


/**
 * @brief    Initializes the core components of the system.
 *
 * Nothing extraordinary here, the external sources for both main and RTC clocks are selected (16MHz and 32.768kHz, respectively).
 * All other modules should be initialized in this function. This function should run in the very beginning of the application. I'd suggest
 * *against* changing the clocks during the operation.
 */


void init_system()
{
    adi_pwr_Init();
	adi_pwr_SetHPBuckLoadMode(ADI_PWR_HPBUCK_LD_MODE_LOW);

	ADI_CLOCK_SOURCE_STATUS clock_status = ADI_CLOCK_SOURCE_ENABLED_NOT_STABLE;
	adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_HFXTAL, true);
	while(clock_status != ADI_CLOCK_SOURCE_ENABLED_STABLE)
	{
		adi_pwr_GetClockStatus(ADI_CLOCK_SOURCE_HFXTAL, &clock_status);
	};


	adi_pwr_SetRootClockMux(ADI_CLOCK_MUX_ROOT_HFXTAL);

	adi_pwr_SetClockDivider(ADI_CLOCK_HCLK,1);
	adi_pwr_SetClockDivider(ADI_CLOCK_PCLK,1);


	adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_LFXTAL, true);

	clock_status = ADI_CLOCK_SOURCE_ENABLED_NOT_STABLE;
	while(clock_status != ADI_CLOCK_SOURCE_ENABLED_STABLE)
	{
		adi_pwr_GetClockStatus(ADI_CLOCK_SOURCE_LFXTAL, &clock_status);
	};

	adi_pwr_SetLFClockMux(ADI_CLOCK_MUX_LFCLK_LFXTAL);

	adi_pwr_UpdateCoreClock();

/*
 * Following if we know the offset of the RTC oscillator, we can trim it if necessary
 * If this is to be used the macro ADI_RTC_CALIBRATE has to be set
 */
// #define ADI_RTC_CALIBRATE
//	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_SetTrim(hDevice0,ADI_RTC_TRIM_INTERVAL_14,ADI_RTC_TRIM_1,ADI_RTC_TRIM_SUB)))
//	{
//		DEBUG_RESULT("\n Failed to set the device %04d", eResult, ADI_RTC_SUCCESS);
//		return(eResult);
//	}
//	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_EnableTrim(hDevice0, true)))
//	{
//		DEBUG_RESULT("\n Failed to enable the trim %04d", eResult, ADI_RTC_SUCCESS);
//		return(eResult);
//	}

    uart_init();

    xint_init();

	gpio_init();

	adc_init(false); // TODO: calibration stuck. Check later.

    i2c_init();

    rtc_Init();

    spi_init();
	lora_initialize();

}

/**
 * @brief    Changes the state of the board components to default (turn off).
 *
 * This is called in every measuring phase to set the initial state for the store function.
 *
 * @sa run_and_store_measurements
 */
void init_store()
{
		adi_gpio_SetLow(MICRO_APWR_EN_PORT, MICRO_APWR_EN_PIN);

//		adi_gpio_SetLow(MICRO_IMP_S2_PORT, MICRO_IMP_S2_PIN);
//		adi_gpio_SetLow(MICRO_IMP_S1_PORT, MICRO_IMP_S1_PIN);
//		adi_gpio_SetLow(MICRO_IMP_S0_PORT, MICRO_IMP_S0_PIN); //Legacy.


		adi_gpio_SetHigh(MICRO_TH_EN_PORT, MICRO_TH_EN_PIN);
		adi_gpio_SetLow(MICRO_TH_S0_PORT, MICRO_TH_S0_PIN);
		adi_gpio_SetLow(MICRO_TH_S1_PORT, MICRO_TH_S1_PIN);
		adi_gpio_SetLow(MICRO_SENSOR_EN_PORT, MICRO_SENSOR_EN_PIN);
		adi_gpio_SetLow(MICRO_RST_INT_PORT, MICRO_RST_INT_PIN);
		adi_gpio_SetLow(MICRO_COMP_ON_PORT, MICRO_COMP_ON_PIN);
		adi_gpio_SetLow(MICRO_STM_START_PORT, MICRO_STM_START_PIN);

		adi_gpio_SetLow(MICRO_REF_S1_PORT, MICRO_REF_S1_PIN);
		adi_gpio_SetLow(MICRO_REF_S0_PORT, MICRO_REF_S0_PIN);

		adi_gpio_SetLow(MICRO_INTEGRATOR_TEST_PORT, MICRO_INTEGRATOR_TEST_PIN); //not really needed but it can be like this



#if BOARD_NUM == 1
    select_comparator_reference(REF1_0_8_REF2_1_2);
#elif BOARD_NUM == 2
    select_comparator_reference(REF1_0_8_REF2_1_2);
#elif BOARD_NUM == 3
    select_comparator_reference(REF1_0_8_REF2_1_2);
#elif BOARD_NUM == 4
    select_comparator_reference(REF1_0_8_REF2_1_2);
#else
    select_comparator_reference(REF1_0_8_REF2_1_2);
#endif

    uart_init();
    xint_init();
    gpio_init();
    adc_init(false);
    i2c_init();
    rtc_Init();
    spi_init();
    lora_initialize();

}

/**
 * @brief    Prints the TDR data memory (array) to UART.
 *
 * @param[in] tdr_data The array of structs holding the "memory" of the system (sensor data).
 *
 * This SHOULD BE viewed as a debug function. For now it can be used to spit out the measurement result to be saved to SD Card or to be viewed in the terminal via
 * UART-USB converter.
 *
 * @sa struct tdr_data
 */
void print_tdr_data_to_uart(struct tdr_data tdr_data[])
{
	char buffer[128]={0}; // should be enough.
	uint16_t msg_length;
	for(uint16_t i=0;i<TDR_MEMORY_SIZE;++i)
		{
			msg_length = sprintf(buffer, "TDR %d: INT1: %u.%02u INT2: %u.%02u || THERM 1: %d | 2: %d | 3: %d | 4: %d|| HONEY: RH::%u.%u TEMP::%u.%u\r\n",
				i,
				tdr_data[i].int1_integer,
				tdr_data[i].int1_decimal,
				tdr_data[i].int2_integer,
				tdr_data[i].int2_decimal,
				tdr_data[i].th1_temp,
				tdr_data[i].th2_temp,
				tdr_data[i].th3_temp,
				tdr_data[i].th4_temp,
				tdr_data[i].honey_rh_integer,
				tdr_data[i].honey_rh_decimal,
				tdr_data[i].honey_temp_integer,
				tdr_data[i].honey_temp_decimal
				);
				uart_write(buffer, msg_length);
		}
}

/**
 * @brief    Function to enter the sleep state.
 *
 * In this mode most of the functionality is off, which sometimes requires reinitializing the modules (most notable example is UART, which means that after waking up
 * uart_init() function should be called). iHibernateExitFlag is a shared variable found in shared.h, which is cleared before entering hibernation and is modified by
 * specific interrupts (from RTC (alarm) or from external interrupts, wired to the UART RX). This modification is picked up by the uC and it wakes up the system.
 *
 * @sa run_and_store_measurements
 */
uint8_t enter_hibernation()
{
	ADI_PWR_RESULT eResult;
	if((eResult = adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_HIBERNATE, &iHibernateExitFlag, 0)) != ADI_PWR_SUCCESS)
	{
		DEBUG_RESULT("Error during entering hibernation", eResult, ADI_PWR_SUCCESS);
		return 1;
	}
	return 0;
}

/**
 * @brief    Sends the data using LoRA.
 *
 * The proposed protocol is very simple. This function should also be used only when pure LoRA communication is used (no gateway and no LoRAWAN). Introduced
 * mainly as a temporary workaround before AAU guys figure out the LoRAWAN and how to talk to the gateway on the drone.
 *
 * @param[in] tdr_data The array of structs holding the "memory" of the system (sensor data).
 * @param[in] num_msg Number of message being sent (one can check if the receiver is getting all the data).
 *
 * @sa run_and_store_measurements
 */
void send_data_package(struct tdr_data integrator_vals_memory[], uint32_t num_msg)
{
	char buffer[256]={0};

	int packageCount = 0;

	uint8_t packet_length = sprintf(buffer, "[%d]{$%3u.%02u/%3u.%02u$%2d/%2d/%2d/%2d$%2u.%2u/%2u.%2u} -> Package %d",
					packageCount,
					integrator_vals_memory[0].int1_integer,
					integrator_vals_memory[0].int1_decimal,
					integrator_vals_memory[0].int2_integer,
					integrator_vals_memory[0].int2_decimal,
					integrator_vals_memory[0].th1_temp,
					integrator_vals_memory[0].th2_temp,
					integrator_vals_memory[0].th3_temp,
					integrator_vals_memory[0].th4_temp,
					integrator_vals_memory[0].honey_rh_integer,
					integrator_vals_memory[0].honey_rh_decimal,
					integrator_vals_memory[0].honey_temp_integer,
					integrator_vals_memory[0].honey_temp_decimal,
					packageNumber
					);

    while (lora_waitACK("Roger !", 10) && packageCount <= num_msg) {
        lora_send_frame(buffer, packet_length);
        packageCount++;
    }
}

/**
 * @brief    Function with the entire measurement logic.
 *
 * In this function every measurement occurs, starting from the TDR, then thermistors and ending with Honeywell (temperature + humidity). Values got from the
 * specific sensors is convered to be stored in struct tdr_part. Before the measurements (TDR + thermistors), the APWR power rail has to be turned on. To reduce
 * the power consumption, it should be turned off after those two sets of measurements are done.
 *
 *
 * @note Some delays in this function may seem random but they depend on the datasheet of the uC, digital part of the TDR measurement setup or other requirements.
 * Sometimes they can be swapped for different values, but please, do explain the changes if you want to introduce them.
 *
 * @param[in] tdr_data The array of structs holding the "memory" of the system (sensor data).
 * @param[in] index Number of the record in tdr_data memory.
 *
 * @sa run_and_store_measurements
 */
void run_and_store_measurements(struct tdr_data* tdr_data, uint16_t* index)
{
    uint16_t pin_status;
    uint32_t delay_val;
    adi_gpio_GetData(MICRO_APWR_EN_PORT, MICRO_APWR_EN_PIN, &pin_status);
    if(!pin_status)
    {
        adi_gpio_SetHigh(MICRO_APWR_EN_PORT, MICRO_APWR_EN_PIN);
        uint32_t delay_val = 960; // = 600us
        while(--delay_val){}; // one value = 625ns of delay
    }

	int16_t thermistor_reading = 0;
	uint16_t honeywell_data[2] = {0};

	uint16_t integrators_data[2] = {0};
	uint16_t integrators_avg[2] = {0};

	double holder = 0;
	power_up_adc(); // APWR -> RST = 71.2ms!!!
	line_stimulation(integrators_data); //discarding one measurement

	delay_val = 16; // around 10us
	while(--delay_val){};

	for(uint8_t i=0; i<ADC_NUM_OF_TDR_STIMULATIONS; ++i) // 5 stimulations, 10 samples each, 50 measurements.
	{
		line_stimulation(integrators_data);
		integrators_avg[0] += integrators_data[0];
		integrators_avg[1] += integrators_data[1];
		delay_val = 16; // around 10us
		while(--delay_val){};
	}

	integrators_avg[0] /= ADC_NUM_OF_TDR_STIMULATIONS;
	integrators_avg[1] /= ADC_NUM_OF_TDR_STIMULATIONS;


	holder = fitting_function_int1(integrators_avg[0]);
	tdr_data[*index].int1_integer = (uint16_t)holder;
	tdr_data[*index].int1_decimal = (uint8_t)((holder - (uint32_t)holder)*100);

	holder = fitting_function_int1(integrators_avg[1]);
	tdr_data[*index].int2_integer = (uint16_t)holder;
	tdr_data[*index].int2_decimal = (uint8_t)((holder - (uint32_t)holder)*100);

	therm_read_temp(&thermistor_reading, THERMISTOR1);
	tdr_data[*index].th1_temp = (int8_t)thermistor_reading;

	therm_read_temp(&thermistor_reading, THERMISTOR2);
	tdr_data[*index].th2_temp = (int8_t)thermistor_reading;

	therm_read_temp(&thermistor_reading, THERMISTOR3);
	tdr_data[*index].th3_temp = (int8_t)thermistor_reading;

	therm_read_temp(&thermistor_reading, THERMISTOR4);
	tdr_data[*index].th4_temp = (int8_t)thermistor_reading;


	adi_gpio_SetLow(MICRO_APWR_EN_PORT, MICRO_APWR_EN_PIN);

	honeywell_read(honeywell_data);


	holder = get_honeywell_rh(honeywell_data[0]);
	tdr_data[*index].honey_rh_integer = (uint8_t)holder;
	tdr_data[*index].honey_rh_decimal = (uint8_t)((holder - (uint32_t)holder)*100);

	holder = get_honeywell_temp(honeywell_data[1]);
	tdr_data[*index].honey_temp_integer = (uint8_t)holder;
	tdr_data[*index].honey_temp_decimal = (uint8_t)((holder - (uint32_t)holder)*100);


	if(++(*index) == TDR_MEMORY_SIZE)
		*index = 0;


	select_comparator_reference(REF1_0_9_REF2_1_1);
}

void delay(uint32_t time) {
    uint32_t clockFreq = getClkFreq(ADI_CLOCK_HCLK);
    uint32_t delay_val = clockFreq / 10000;
    delay_val *= time;
    while(--delay_val) {}
}

uint32_t getClkFreq(const ADI_CLOCK_ID eClockId) {
    uint32_t clockFreq;
    adi_pwr_GetClockFrequency(eClockId, &clockFreq);
    return clockFreq;
}

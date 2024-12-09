#include <stdint.h>
#include "tdr_interface.h"
#include "uart-au.h"
#include "adc-au.h"

void set_tdr_register_bits(const TDR_RegisterBits *bits) {
    uint16_t DATA_P0 = 0;
    uint16_t DATA_P1 = 0;

    // Construct the 16-bit value for P0
    DATA_P0 |= (bits->REF_SEL_2 & 0x1) << 0;
    DATA_P0 |= (bits->REF_SEL_1 & 0x1) << 1;
    DATA_P0 |= (bits->REF_SEL_0 & 0x1) << 2;
    DATA_P0 |= (bits->OPEN_DETECT_EN & 0x1) << 3;
    DATA_P0 |= (bits->CHOP_SW_DIRECTED & 0x1) << 4;
    DATA_P0 |= (bits->RESET_IN & 0x1) << 5;
    DATA_P0 |= (bits->BG_EN & 0x1) << 6;
    DATA_P0 |= (bits->CRYSTAL_REF_EN & 0x1) << 7;
    // P0[7] is fixed at 0
    DATA_P0 |= (bits->F_SEL0 & 0x1) << 8;
    DATA_P0 |= (bits->F_SEL1 & 0x1) << 9;
    DATA_P0 |= (bits->VCO_DOUBLE_CUR & 0x1) << 10;
    DATA_P0 |= (bits->CTR_PWR_ON & 0x1) << 11;
    DATA_P0 |= (bits->READ_SRIO_EN & 0x1) << 12;
    DATA_P0 |= (bits->DATA_IN_SRIO & 0x1) << 13;
    DATA_P0 |= (bits->CLK_SRIO_EN & 0x1) << 14;
    // P0[15] is fixed at 0

    // Construct the 16-bit value for P1
    DATA_P1 |= (bits->I_CAL_EN & 0x1) << 0;
    DATA_P1 |= (bits->THERMISTOR_EN & 0x1) << 1;
    DATA_P1 |= (bits->INTEGRATOR_EN & 0x1) << 2;
    DATA_P1 |= (bits->DAC_CUR_SEL0 & 0x1) << 3;
    DATA_P1 |= (bits->DAC_CUR_SEL1 & 0x1) << 4;
    DATA_P1 |= (bits->DAC_CUR_SEL2 & 0x1) << 5;
    DATA_P1 |= (bits->THERMISTOR_SEL1 & 0x1) << 6;
    DATA_P1 |= (bits->THERMISTOR_SEL0 & 0x1) << 7;
    DATA_P1 |= (bits->TEST_EN & 0x1) << 8;
    DATA_P1 |= (bits->DRV_D0 & 0x1) << 9;
    DATA_P1 |= (bits->DRV_D1 & 0x1) << 10;
    DATA_P1 |= (bits->DRV_D2 & 0x1) << 11;
    DATA_P1 |= (bits->DRV_D3 & 0x1) << 12;
    DATA_P1 |= (bits->DRV_D4 & 0x1) << 13;
    DATA_P1 |= (bits->COMP_EN & 0x1) << 14;
    // P1[15] is fixed at 0

    // Write the constructed values to the respective registers
    mcp23s17_set_device_outputs(MCP23S17_DEVICE_0, DATA_P0);
    mcp23s17_set_device_outputs(MCP23S17_DEVICE_1, DATA_P1);
}

void tdr_start_test_measurement_digital(uint32_t test_pulsewidth_us) {
    char uart_msg[255] = {0};

    uint16_t srio_data_pin_value;
    uint32_t srio_data;
    float srio_data_converted;

	adi_gpio_SetLow(DRV_IN_PORT, DRV_IN_PIN);
	adi_gpio_SetLow(PLL_ON_PORT, PLL_ON_PIN);
	adi_gpio_SetLow(TEST_STOP_PORT, TEST_STOP_PIN);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

    TDR_RegisterBits tdr_bits = {0}; // Initialize all bits to 0

	tdr_bits.REF_SEL_2 = 1;
	tdr_bits.REF_SEL_1 = 1;
	tdr_bits.REF_SEL_0 = 1;
	tdr_bits.OPEN_DETECT_EN = 0;
	tdr_bits.CHOP_SW_DIRECTED = 1;
	tdr_bits.F_SEL0 = 1;
	tdr_bits.F_SEL1 = 0;
	tdr_bits.VCO_DOUBLE_CUR = 0;
	tdr_bits.BG_EN = 1;
	tdr_bits.I_CAL_EN = 0;
	tdr_bits.THERMISTOR_EN = 0;
	tdr_bits.INTEGRATOR_EN = 0;
	tdr_bits.DAC_CUR_SEL0 = 0;
	tdr_bits.DAC_CUR_SEL1 = 0;
	tdr_bits.DAC_CUR_SEL2 = 0;
	tdr_bits.DRV_D0 = 0;
	tdr_bits.DRV_D1 = 0;
	tdr_bits.DRV_D2 = 0;
	tdr_bits.DRV_D3 = 0;
	tdr_bits.DRV_D4 = 0;
	tdr_bits.THERMISTOR_SEL1 = 0;
	tdr_bits.THERMISTOR_SEL0 = 0;
	tdr_bits.COMP_EN = 1;

	tdr_bits.CRYSTAL_REF_EN = 1;
	tdr_bits.CTR_PWR_ON = 1;
	tdr_bits.TEST_EN = 1;
	tdr_bits.RESET_IN = 0;
	tdr_bits.CLK_SRIO_EN = 0;
	tdr_bits.READ_SRIO_EN = 0;
	tdr_bits.DATA_IN_SRIO = 0;
	set_tdr_register_bits(&tdr_bits);

	// Start sequence
	adi_gpio_SetHigh(PLL_ON_PORT, PLL_ON_PIN);
	tdr_bits.RESET_IN = 1;
	set_tdr_register_bits(&tdr_bits);
	delay_us(100);
	tdr_bits.RESET_IN = 0;
	set_tdr_register_bits(&tdr_bits);
	delay_us(100);

	adi_gpio_SetHigh(DRV_IN_PORT, DRV_IN_PIN);
	delay_us(test_pulsewidth_us);
	adi_gpio_SetHigh(TEST_STOP_PORT, TEST_STOP_PIN);

	// Reading sequence
	tdr_bits.CLK_SRIO_EN = 1;
	set_tdr_register_bits(&tdr_bits);
	adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
	delay_us(100);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);
	delay_us(100);

	tdr_bits.READ_SRIO_EN = 1;
	set_tdr_register_bits(&tdr_bits);
	//adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
	//delay_us(100);
	//adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);
	//delay_us(100);

	srio_data = 0;
	for (int i = 19; i >= 0 ; i--) {
		adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
		delay_us(100);
		adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

		// Sample data at posedge and populate srio_data bit by bit.
		adi_gpio_GetData(DATA_OUT_SRIO_PORT, DATA_OUT_SRIO_PIN, &srio_data_pin_value);
		srio_data_pin_value = srio_data_pin_value >> 10;
	    srio_data |= (srio_data_pin_value << i);
		delay_us(100);
	}
	srio_data_converted = srio_data/(1.28*1000);

	adi_gpio_SetLow(DRV_IN_PORT, DRV_IN_PIN);
	adi_gpio_SetLow(PLL_ON_PORT, PLL_ON_PIN);
	adi_gpio_SetLow(TEST_STOP_PORT, TEST_STOP_PIN);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);


	//sprintf(uart_msg, "measured pulsewidth: %.2f, applied pulsewidth: %d", srio_data_converted, test_pulsewidth_us);
	//uart_write(uart_msg);

	//delay_us(10000);

}

void tdr_de_init(){
	adi_gpio_SetLow(SHIFTREG_NRST_PORT, SHIFTREG_NRST_PIN);
	delay_us(10);
	adi_gpio_SetHigh(SHIFTREG_NRST_PORT, SHIFTREG_NRST_PIN);
}

uint32_t tdr_start_measurement_digital() {
    char uart_msg[255] = {0};

    uint16_t srio_data_pin_value;
    uint32_t srio_data;
    float srio_data_converted;


	adi_gpio_SetLow(DRV_IN_PORT, DRV_IN_PIN);
	adi_gpio_SetLow(PLL_ON_PORT, PLL_ON_PIN);
	adi_gpio_SetLow(TEST_STOP_PORT, TEST_STOP_PIN);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

    TDR_RegisterBits tdr_bits = {0}; // Initialize all bits to 0

	tdr_bits.REF_SEL_2 = 0;
	tdr_bits.REF_SEL_1 = 0;
	tdr_bits.REF_SEL_0 = 0;
	tdr_bits.OPEN_DETECT_EN = 0;
	tdr_bits.CHOP_SW_DIRECTED = 0;
	tdr_bits.F_SEL0 = 1;
	tdr_bits.F_SEL1 = 0;
	tdr_bits.VCO_DOUBLE_CUR = 0;
	tdr_bits.BG_EN = 1;
	tdr_bits.I_CAL_EN = 0;
	tdr_bits.THERMISTOR_EN = 0;
	tdr_bits.INTEGRATOR_EN = 0;
	tdr_bits.DAC_CUR_SEL0 = 0;
	tdr_bits.DAC_CUR_SEL1 = 0;
	tdr_bits.DAC_CUR_SEL2 = 0;
	tdr_bits.DRV_D0 = 1;
	tdr_bits.DRV_D1 = 1;
	tdr_bits.DRV_D2 = 1;
	tdr_bits.DRV_D3 = 1;
	tdr_bits.DRV_D4 = 1;
	tdr_bits.THERMISTOR_SEL1 = 0;
	tdr_bits.THERMISTOR_SEL0 = 0;
	tdr_bits.COMP_EN = 1;

	tdr_bits.CRYSTAL_REF_EN = 1;
	tdr_bits.CTR_PWR_ON = 1;
	tdr_bits.TEST_EN  = 0;
	tdr_bits.RESET_IN = 0;
	tdr_bits.CLK_SRIO_EN = 0;
	tdr_bits.READ_SRIO_EN = 0;
	tdr_bits.DATA_IN_SRIO = 0;
	set_tdr_register_bits(&tdr_bits);

	// Start sequence
	adi_gpio_SetHigh(PLL_ON_PORT, PLL_ON_PIN);
	tdr_bits.RESET_IN = 1;
	set_tdr_register_bits(&tdr_bits);
	delay_us(100);
	tdr_bits.RESET_IN = 0;
	set_tdr_register_bits(&tdr_bits);
	delay_us(100);

	adi_gpio_SetHigh(DRV_IN_PORT, DRV_IN_PIN);
	//delay_us(test_pulsewidth_us);
	//adi_gpio_SetHigh(TEST_STOP_PORT, TEST_STOP_PIN);
	delay_us(100);

	// Reading sequence
	tdr_bits.CLK_SRIO_EN = 1;
	set_tdr_register_bits(&tdr_bits);
	adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
	delay_us(100);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);
	delay_us(100);

	tdr_bits.READ_SRIO_EN = 1;
	set_tdr_register_bits(&tdr_bits);
	//adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
	//delay_us(100);
	//adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);
	//delay_us(100);

	srio_data = 0;
	for (int i = 19; i >= 0 ; i--) {
		adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
		delay_us(100);
		adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

		// Sample data at posedge and populate srio_data bit by bit.
		adi_gpio_GetData(DATA_OUT_SRIO_PORT, DATA_OUT_SRIO_PIN, &srio_data_pin_value);
		srio_data_pin_value = srio_data_pin_value >> 10;
	    srio_data |= (srio_data_pin_value << i);
		delay_us(100);
	}
	//srio_data_converted = srio_data/(1.28*1000);

	adi_gpio_SetLow(DRV_IN_PORT, DRV_IN_PIN);
	adi_gpio_SetLow(PLL_ON_PORT, PLL_ON_PIN);
	adi_gpio_SetLow(TEST_STOP_PORT, TEST_STOP_PIN);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

	/*
	sprintf(uart_msg, "measured pulsewidth: %.5f", srio_data_converted);
	uart_write(uart_msg);

	sprintf(uart_msg, "raw pulsewidth: %d", srio_data);
	uart_write(uart_msg);
	 */

	return srio_data;

	//delay_us(10000);

}

void tdr_read_thermistors(uint16_t* thermistor_values){
    char uart_msg[255] = {0};
    uint16_t adc_results[3];

    uint16_t srio_data_pin_value;
    uint32_t srio_data;
    float srio_data_converted;

	adi_gpio_SetLow(DRV_IN_PORT, DRV_IN_PIN);
	//adi_gpio_SetLow(PLL_ON_PORT, PLL_ON_PIN);
	adi_gpio_SetLow(TEST_STOP_PORT, TEST_STOP_PIN);
	//adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

    TDR_RegisterBits tdr_bits = {0}; // Initialize all bits to 0

	tdr_bits.REF_SEL_2 = 0;
	tdr_bits.REF_SEL_1 = 0;
	tdr_bits.REF_SEL_0 = 1;
	tdr_bits.OPEN_DETECT_EN = 0;
	tdr_bits.CHOP_SW_DIRECTED = 0;
	tdr_bits.F_SEL0 = 0;
	tdr_bits.F_SEL1 = 0;
	tdr_bits.VCO_DOUBLE_CUR = 0;
	tdr_bits.BG_EN = 1;
	tdr_bits.I_CAL_EN = 1;
	tdr_bits.THERMISTOR_EN = 1;
	tdr_bits.INTEGRATOR_EN = 0;
	tdr_bits.DAC_CUR_SEL0 = 1;
	tdr_bits.DAC_CUR_SEL1 = 1;
	tdr_bits.DAC_CUR_SEL2 = 1;
	tdr_bits.DRV_D0 = 1;
	tdr_bits.DRV_D1 = 1;
	tdr_bits.DRV_D2 = 1;
	tdr_bits.DRV_D3 = 1;
	tdr_bits.DRV_D4 = 1;
	tdr_bits.THERMISTOR_SEL1 = 0;
	tdr_bits.THERMISTOR_SEL0 = 0;
	tdr_bits.COMP_EN = 1;
	tdr_bits.CRYSTAL_REF_EN = 1;
	tdr_bits.CTR_PWR_ON = 0;
	tdr_bits.TEST_EN = 0;
	tdr_bits.RESET_IN = 0;
	tdr_bits.CLK_SRIO_EN = 0;
	tdr_bits.READ_SRIO_EN = 0;
	tdr_bits.DATA_IN_SRIO = 0;
	set_tdr_register_bits(&tdr_bits);
	delay_us(10);

	// Read from all 4 thermistors via mux
	ADC_SampleChannels(adc_results);
	thermistor_values[0] = adc_results[1];

	tdr_bits.THERMISTOR_SEL1 = 0;
	tdr_bits.THERMISTOR_SEL0 = 1;
	set_tdr_register_bits(&tdr_bits);
	ADC_SampleChannels(adc_results);
	thermistor_values[1] = adc_results[1];

	tdr_bits.THERMISTOR_SEL1 = 1;
	tdr_bits.THERMISTOR_SEL0 = 0;
	set_tdr_register_bits(&tdr_bits);
	ADC_SampleChannels(adc_results);
	thermistor_values[2] = adc_results[1];

	tdr_bits.THERMISTOR_SEL1 = 1;
	tdr_bits.THERMISTOR_SEL0 = 1;
	set_tdr_register_bits(&tdr_bits);
	ADC_SampleChannels(adc_results);
	thermistor_values[3] = adc_results[1];

}

void tdr_start_measurement_analog(uint32_t test_pulsewidth_us) {
    char uart_msg[255] = {0};

    uint16_t srio_data_pin_value;
    uint32_t srio_data;
    float srio_data_converted;


	adi_gpio_SetLow(DRV_IN_PORT, DRV_IN_PIN);
	//adi_gpio_SetLow(PLL_ON_PORT, PLL_ON_PIN);
	adi_gpio_SetLow(TEST_STOP_PORT, TEST_STOP_PIN);
	//adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

    TDR_RegisterBits tdr_bits = {0}; // Initialize all bits to 0

	tdr_bits.REF_SEL_2 = 0;
	tdr_bits.REF_SEL_1 = 0;
	tdr_bits.REF_SEL_0 = 1;
	tdr_bits.OPEN_DETECT_EN = 0;
	tdr_bits.CHOP_SW_DIRECTED = 0;
	tdr_bits.F_SEL0 = 0;
	tdr_bits.F_SEL1 = 0;
	tdr_bits.VCO_DOUBLE_CUR = 0;
	tdr_bits.BG_EN = 1;
	tdr_bits.I_CAL_EN = 1;
	tdr_bits.THERMISTOR_EN = 0;
	tdr_bits.INTEGRATOR_EN = 1;
	tdr_bits.DAC_CUR_SEL0 = 0;
	tdr_bits.DAC_CUR_SEL1 = 0;
	tdr_bits.DAC_CUR_SEL2 = 0;
	tdr_bits.DRV_D0 = 1;
	tdr_bits.DRV_D1 = 1;
	tdr_bits.DRV_D2 = 1;
	tdr_bits.DRV_D3 = 1;
	tdr_bits.DRV_D4 = 1;
	tdr_bits.THERMISTOR_SEL1 = 0;
	tdr_bits.THERMISTOR_SEL0 = 0;
	tdr_bits.COMP_EN = 1;

	tdr_bits.CRYSTAL_REF_EN = 1;
	tdr_bits.CTR_PWR_ON = 0;
	tdr_bits.TEST_EN = 1;
	tdr_bits.RESET_IN = 0;
	tdr_bits.CLK_SRIO_EN = 0;
	tdr_bits.READ_SRIO_EN = 0;
	tdr_bits.DATA_IN_SRIO = 0;
	set_tdr_register_bits(&tdr_bits);

	// Start sequence

	//adi_gpio_SetHigh(PLL_ON_PORT, PLL_ON_PIN);
	tdr_bits.RESET_IN = 1;
	set_tdr_register_bits(&tdr_bits);
	delay_us(10);
	tdr_bits.RESET_IN = 0;
	set_tdr_register_bits(&tdr_bits);
	delay_us(10);


	adi_gpio_SetHigh(DRV_IN_PORT, DRV_IN_PIN);
	delay_us(test_pulsewidth_us);
	adi_gpio_SetHigh(TEST_STOP_PORT, TEST_STOP_PIN);

	/*
	// Reading sequence
	tdr_bits.CLK_SRIO_EN = 1;
	set_tdr_register_bits(&tdr_bits);
	adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
	delay_us(100);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);
	delay_us(100);

	tdr_bits.READ_SRIO_EN = 1;
	set_tdr_register_bits(&tdr_bits);
	//adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
	//delay_us(100);
	//adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);
	//delay_us(100);

	srio_data = 0;
	for (int i = 19; i >= 0 ; i--) {
		adi_gpio_SetLow(CLK_SRIO_PORT, CLK_SRIO_PIN);
		delay_us(100);
		adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);

		// Sample data at posedge and populate srio_data bit by bit.
		adi_gpio_GetData(DATA_OUT_SRIO_PORT, DATA_OUT_SRIO_PIN, &srio_data_pin_value);
		srio_data_pin_value = srio_data_pin_value >> 10;
	    srio_data |= (srio_data_pin_value << i);
		delay_us(100);
	}
	srio_data_converted = srio_data/(1.28*1000) - 12.83;

	adi_gpio_SetLow(DRV_IN_PORT, DRV_IN_PIN);
	adi_gpio_SetLow(PLL_ON_PORT, PLL_ON_PIN);
	adi_gpio_SetLow(TEST_STOP_PORT, TEST_STOP_PIN);
	adi_gpio_SetHigh(CLK_SRIO_PORT, CLK_SRIO_PIN);
	*/

	//sprintf(uart_msg, "measured pulsewidth: %.2f, applied pulsewidth: %d", srio_data_converted, test_pulsewidth_us);
	//uart_write(uart_msg);

	delay_us(10000);

}

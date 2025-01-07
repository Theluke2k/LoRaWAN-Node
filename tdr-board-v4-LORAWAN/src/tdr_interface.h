/*
 * tdr_interface.h
 *
 *  Created on: 27. nov. 2024
 *      Author: au748936
 */

#ifndef TDR_INTERFACE_H_
#define TDR_INTERFACE_H_

#include "gpio-au.h"
#include "general_functions.h"
#include "mcp23s17.h"

typedef struct {
    uint8_t REF_SEL_2;
    uint8_t REF_SEL_1;
    uint8_t REF_SEL_0;
    uint8_t OPEN_DETECT_EN;
    uint8_t CHOP_SW_DIRECTED;
    uint8_t F_SEL0;
    uint8_t F_SEL1;
    uint8_t VCO_DOUBLE_CUR;
    uint8_t BG_EN;
    uint8_t I_CAL_EN;
    uint8_t THERMISTOR_EN;
    uint8_t INTEGRATOR_EN;
    uint8_t DAC_CUR_SEL0;
    uint8_t DAC_CUR_SEL1;
    uint8_t DAC_CUR_SEL2;
    uint8_t DRV_D0;
    uint8_t DRV_D1;
    uint8_t DRV_D2;
    uint8_t DRV_D3;
    uint8_t DRV_D4;
    uint8_t THERMISTOR_SEL1;
    uint8_t THERMISTOR_SEL0;
    uint8_t COMP_EN;
    uint8_t CTR_PWR_ON;
    uint8_t TEST_EN;
    uint8_t RESET_IN;
    uint8_t CLK_SRIO_EN;
    uint8_t READ_SRIO_EN;
    uint8_t DATA_IN_SRIO;
    uint8_t CRYSTAL_REF_EN;
} TDR_RegisterBits;

void set_tdr_register_bits(const TDR_RegisterBits *bits);

void tdr_de_init();
uint32_t tdr_start_measurement_digital();
uint32_t tdr_start_test_measurement_digital(uint32_t test_pulsewidth_us);

void tdr_start_measurement_analog(uint32_t test_pulsewidth_us);
void tdr_read_thermistors(uint16_t* thermistor_values);

#endif /* TDR_INTERFACE_H_ */

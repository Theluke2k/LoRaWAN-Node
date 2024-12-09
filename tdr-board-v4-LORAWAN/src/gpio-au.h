/*
 * pin_configuration.h
 *
 *  Created on: 27. maj 2022
 *      Author: au703540
 */

#ifndef GPIO_AU_H_
#define GPIO_AU_H_

#include <adi_gpio.h>


#define UART_WAKEUP_PORT					ADI_GPIO_PORT0
#define UART_WAKEUP_PIN						ADI_GPIO_PIN_13

#define LORA_RST_PORT						ADI_GPIO_PORT2
#define LORA_RST_PIN						ADI_GPIO_PIN_1

#define LORA_DIO0_PORT						ADI_GPIO_PORT1
#define LORA_DIO0_PIN						ADI_GPIO_PIN_5

#define THERM_READ_PORT					    ADI_GPIO_PORT2
#define THERM_READ_PIN						ADI_GPIO_PIN_5

#define VBAT_SAMPLE_PORT		            ADI_GPIO_PORT2
#define VBAT_SAMPLE_PIN						ADI_GPIO_PIN_6

#define DEBUG_PIN_PORT		                ADI_GPIO_PORT0
#define DEBUG_PIN_PIN						ADI_GPIO_PIN_14

#define TDR_PWR_PORT		                ADI_GPIO_PORT0
#define TDR_PWR_PIN						    ADI_GPIO_PIN_13

#define VBAT_READ_EN_PORT		            ADI_GPIO_PORT2
#define VBAT_READ_EN_PIN					ADI_GPIO_PIN_1

#define SHIFTREG_NRST_PORT					ADI_GPIO_PORT2
#define SHIFTREG_NRST_PIN					ADI_GPIO_PIN_11

#define PWR_HONEYWELL_PORT					ADI_GPIO_PORT2
#define PWR_HONEYWELL_PIN					ADI_GPIO_PIN_3


////////////////////// TDR STUFF

#define TEST_STOP_PORT						DEBUG_PIN_PORT
#define TEST_STOP_PIN						DEBUG_PIN_PIN

#define DRV_IN_PORT							ADI_GPIO_PORT2
#define DRV_IN_PIN							ADI_GPIO_PIN_2

#define PLL_ON_PORT							ADI_GPIO_PORT2
#define PLL_ON_PIN							ADI_GPIO_PIN_8

#define CLK_SRIO_PORT						ADI_GPIO_PORT2
#define CLK_SRIO_PIN						ADI_GPIO_PIN_9

#define DATA_OUT_SRIO_PORT					ADI_GPIO_PORT2
#define DATA_OUT_SRIO_PIN					ADI_GPIO_PIN_10

// For debug only. Remove later.
#define DRV_IN_PORT_TEST	  ADI_GPIO_PORT0
#define DRV_IN_PIN_TEST	  	  ADI_GPIO_PIN_0

#define PLL_ON_PORT_TEST	  ADI_GPIO_PORT0
#define PLL_ON_PIN_TEST	  	  ADI_GPIO_PIN_1

#define CLK_SRIO_PORT_TEST	  ADI_GPIO_PORT0
#define CLK_SRIO_PIN_TEST	  ADI_GPIO_PIN_14

#define TEST_STOP_PORT_TEST ADI_GPIO_PORT0
#define TEST_STOP_PIN_TEST  ADI_GPIO_PIN_3

void analog_pin_init();
void digital_pin_init();
void i2c_pin_init();
void gpio_init();

#endif /* GPIO_AU_H_ */

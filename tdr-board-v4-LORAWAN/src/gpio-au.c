/*
 * pin_configuration.c
 *
 *  Created on: 30. maj 2022
 *      Author: au703540
 */
#include "gpio-au.h"

#include <sys/platform.h>
#include "uart-au.h"
#include "sx1276-board.h"
#include "gpio.h"
#include "board-config.h"
#include "gpio-board.h"

/*
 * Renamed macros for the MUXes for GPIO pins. In ADuCM4050 this is how you select the special function of the GPIO pin (f.e. pin X may be a simple GPIO,
 * clock for I2C or MOSI for SPI - you can select this with pinMuxing). This is nicely described in the reference manual of the uC.
 */
// ADC
#define ADC0_IN_ADC0_VIN0_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<6))
#define ADC0_IN_ADC0_VIN1_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define ADC0_IN_ADC0_VIN2_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<10))
//#define ADC0_IN_ADC0_VIN3_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<12))
//#define ADC0_IN_ADC0_VIN4_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<14))
//#define ADC0_IN_ADC0_VIN5_PORTP2_MUX  ((uint32_t) ((uint32_t) 1<<16))
//#define ADC0_IN_ADC0_VIN6_PORTP2_MUX  ((uint32_t) ((uint32_t) 1<<18))
//#define ADC0_IN_ADC0_VIN7_PORTP2_MUX  ((uint32_t) ((uint32_t) 1<<20))

// I2C
#define I2C0_SCL0_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define I2C0_SDA0_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<10))

// SPI
#define SPI0_CLK_PORTP0_MUX   	((uint16_t) ((uint16_t) 1<<0))
#define SPI0_MOSI_PORTP0_MUX  	((uint16_t) ((uint16_t) 1<<2))
#define SPI0_MISO_PORTP0_MUX  	((uint16_t) ((uint16_t) 1<<4))
#define SPI0_CS_PORT0_MUX		((uint16_t) ((uint16_t) 1<<6))

// UART0
#define UART0_TX_PORTP0_MUX  ((uint32_t) ((uint32_t) 0x1 << 20))
#define UART0_RX_PORTP0_MUX  ((uint32_t) ((uint32_t) 0x1 << 22))

// UART1
#define UART1_TX_PORTP2_MUX  ((uint32_t) ((uint32_t) 0x2 << 30))
#define UART1_RX_PORTP3_MUX  ((uint32_t) ((uint32_t) 0x2 << 0))

/**
 * GPIO device memory used by the driver.
 */
static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];

/**
 * @brief    Analog pin mux setup.
 *
 * Sets the mux values for all of the analog pins used in the project.
 */
void analog_pin_init()
{
	*pREG_GPIO2_CFG |= ADC0_IN_ADC0_VIN0_PORTP2_MUX | ADC0_IN_ADC0_VIN1_PORTP2_MUX
	 | ADC0_IN_ADC0_VIN2_PORTP2_MUX;
}

/**
 * @brief    Digital pin mux and initial values setup.
 *
 * Sets the mux values for all of the digital pins used in the project and sets the initial values so that most of the board is turned off.
 */
void digital_pin_init()
{
	ADI_GPIO_RESULT error_status = ADI_GPIO_SUCCESS;

	// XINT
//	if(ADI_GPIO_SUCCESS != (error_status = adi_gpio_InputEnable(UART_WAKEUP_PORT, UART_WAKEUP_PIN, true)))
//	{
//		DEBUG_MESSAGE("adi_gpio_InputEnable failed\n");
//	}

	// UART0
	//*((volatile uint32_t *)REG_GPIO0_CFG) |= UART0_TX_PORTP0_MUX;
	//*((volatile uint32_t *)REG_GPIO0_CFG) |= UART0_RX_PORTP0_MUX;

	// UART1
    //*((volatile uint32_t *)REG_GPIO1_CFG) |= UART1_TX_PORTP2_MUX;
	//*((volatile uint32_t *) REG_GPIO2_CFG) |= UART1_RX_PORTP3_MUX;

	// DEBUG start
    // LORA/SPI
    //*((volatile uint32_t *)REG_GPIO0_CFG) |= SPI0_CLK_PORTP0_MUX | SPI0_MOSI_PORTP0_MUX | SPI0_MISO_PORTP0_MUX | SPI0_CS_PORT0_MUX;
	/*
    if(ADI_GPIO_SUCCESS != (error_status = adi_gpio_OutputEnable(LORA_RST_PORT, LORA_RST_PIN, true)))
	{
		DEBUG_MESSAGE("adi_gpio_OutputEnable failed\n");
	}
	if(ADI_GPIO_SUCCESS != (error_status = adi_gpio_InputEnable(LORA_DIO0_PORT, LORA_DIO0_PIN, true)))
	{
		DEBUG_MESSAGE("adi_gpio_InputEnable failed\n");
	}
*/
	//adi_gpio_SetHigh(LORA_RST_PORT, LORA_RST_PIN);
	//DEBUG end

	// Radio and EEPROM power pin
	if (ADI_GPIO_SUCCESS != (error_status = adi_gpio_OutputEnable(LORA_PWR_PORT, LORA_PWR_PIN, true))) {
		DEBUG_MESSAGE("adi_gpio_OutputEnable failed\n");
	}

	/*
	 * Lucas (28-07-2024):
	 * DEBUG PINS
	 */
	if (ADI_GPIO_SUCCESS != (error_status = adi_gpio_OutputEnable(ADI_GPIO_PORT0, ADI_GPIO_PIN_14, true))) {
		DEBUG_MESSAGE("adi_gpio_OutputEnable failed\n");
	}

	/*
	 * Lucas (20-10-2024):
	 * EEPROM PINS
	 */
	if (ADI_GPIO_SUCCESS != (error_status = adi_gpio_OutputEnable(EEPROM_WP_PORT, EEPROM_WP_PIN, true))) {
		DEBUG_MESSAGE("adi_gpio_OutputEnable failed\n");
	}

	if (ADI_GPIO_SUCCESS != (error_status = adi_gpio_OutputEnable(EEPROM_HOLD_PORT, EEPROM_HOLD_PIN, true))) {
		DEBUG_MESSAGE("adi_gpio_OutputEnable failed\n");
	}

	// INITIAL PINS STATES
	adi_gpio_SetLow(EEPROM_WP_PORT, EEPROM_WP_PIN);					// Enable write protection. Remember to disable when writing to EEPROM
	adi_gpio_SetHigh(EEPROM_HOLD_PORT, EEPROM_HOLD_PORT);			// Disable Hold. If enabled, the communication with the EEPROM is paused.
	adi_gpio_SetHigh(EEPROM_CS_PORT, EEPROM_CS_PORT);				// De-select the chip. Remember to select before communication.
	adi_gpio_SetLow(LORA_PWR_PORT, LORA_PWR_PIN);					// Allow power to the radio
}

/*
 * Lucas (28-07-2024):
 * Function to initialize digital pins for measurement mode.
 */
void DigitalPinsEnable() {
	//Initial states
	adi_gpio_SetLow(MICRO_APWR_EN_PORT, MICRO_APWR_EN_PIN);
	adi_gpio_SetHigh(MICRO_TH_EN_PORT, MICRO_TH_EN_PIN);
	adi_gpio_SetLow(MICRO_TH_S0_PORT, MICRO_TH_S0_PIN);
	adi_gpio_SetLow(MICRO_TH_S1_PORT, MICRO_TH_S1_PIN);
	adi_gpio_SetLow(MICRO_SENSOR_EN_PORT, MICRO_SENSOR_EN_PIN);
	adi_gpio_SetLow(MICRO_RST_INT_PORT, MICRO_RST_INT_PIN);
	adi_gpio_SetLow(MICRO_COMP_ON_PORT, MICRO_COMP_ON_PIN);
	adi_gpio_SetLow(MICRO_STM_START_PORT, MICRO_STM_START_PIN);
	adi_gpio_SetLow(MICRO_REF_S1_PORT, MICRO_REF_S1_PIN);
	adi_gpio_SetLow(MICRO_REF_S0_PORT, MICRO_REF_S0_PORT);
	adi_gpio_SetLow(MICRO_INTEGRATOR_TEST_PORT, MICRO_INTEGRATOR_TEST_PIN);
}

/*
 * Lucas (28-07-2024):
 * Function to disable any digital pin used for measurements.
 */
void DigitalPinsDisable() {
	// Disable everything
	adi_gpio_SetLow(MICRO_APWR_EN_PORT, MICRO_APWR_EN_PIN);
	adi_gpio_SetLow(MICRO_TH_EN_PORT, MICRO_TH_EN_PIN);
	adi_gpio_SetLow(MICRO_TH_S0_PORT, MICRO_TH_S0_PIN);
	adi_gpio_SetLow(MICRO_TH_S1_PORT, MICRO_TH_S1_PIN);
	adi_gpio_SetLow(MICRO_SENSOR_EN_PORT, MICRO_SENSOR_EN_PIN);
	adi_gpio_SetLow(MICRO_RST_INT_PORT, MICRO_RST_INT_PIN);
	adi_gpio_SetLow(MICRO_COMP_ON_PORT, MICRO_COMP_ON_PIN);
	adi_gpio_SetLow(MICRO_STM_START_PORT, MICRO_STM_START_PIN);
	adi_gpio_SetLow(MICRO_REF_S1_PORT, MICRO_REF_S1_PIN);
	adi_gpio_SetLow(MICRO_REF_S0_PORT, MICRO_REF_S0_PORT);
	adi_gpio_SetLow(MICRO_INTEGRATOR_TEST_PORT, MICRO_INTEGRATOR_TEST_PIN);
}

/**
 * @brief    I2C pin mux setup.
 *
 * Sets the mux values for all of the I2C pins used in the project.
 */
void i2c_pin_init()
{
	*((volatile uint32_t *)REG_GPIO0_CFG) |= I2C0_SCL0_PORTP0_MUX | I2C0_SDA0_PORTP0_MUX;
}

/**
 * @brief    GPIO device init function.
 *
 * Initializes the GPIO device from the ADI driver. Includes the pin initialization functions.
 */
void gpio_init()
{
	ADI_GPIO_RESULT gpioStatus = ADI_GPIO_SUCCESS;
	gpioStatus = adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);
	DEBUG_RESULT("GPIO init failed", gpioStatus, ADI_GPIO_SUCCESS);

	digital_pin_init();
    //analog_pin_init();
    //i2c_pin_init();
}

void InitGPIODriver() {
	ADI_GPIO_RESULT gpioStatus = ADI_GPIO_SUCCESS;
	gpioStatus = adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);
	DEBUG_RESULT("GPIO init failed", gpioStatus, ADI_GPIO_SUCCESS);
}

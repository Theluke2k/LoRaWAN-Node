/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */

#include <sys/platform.h>
#include "adi_initialize.h"
#include <stdint.h>
#include <common.h>
#include <unistd.h>
#include <drivers/general/adi_drivers_general.h>
#include <adi_processor.h>
#include <drivers/pwr/adi_pwr.h>
#include <adi_gpio.h>
#include <drivers/spi/adi_spi.h>

/* Below: new includes. May be shaky? NB: ..//is for going to the parent directory*/
#include <stdio.h>
#include "utilities.h"
//Might have to be changed to be something more specific relating to the board
#include "gpio-board.h"
#include "gpio.h"
#include "sx1276.h"

ADI_GPIO_PORT port0 = ADI_GPIO_PORT0;
ADI_GPIO_PORT port1 = ADI_GPIO_PORT1;
ADI_GPIO_PORT port2 = ADI_GPIO_PORT2;
ADI_GPIO_PORT port3 = ADI_GPIO_PORT3;

/*
 * Lucas (12-11-23):
 * To know which callbacks to use when different pins trigger an interrupt,
 * we use an array which contains pointers to the corresponding callback functions.
 */
typedef void (*FunctionPointer)( void* context );

FunctionPointer array[51];

/*
 * Lucas:
 * This function initializes an arbitrary gpio pin.
 *
 * The function is written with inspiration from the gpio-board.c file for the
 * B-L072Z board.
 *
 * It first sets the pullup/pulldown of the pin, the adi_gpio_PullUpEnable
 * only enables whatever pull-up/down is set on the pin. I think you have
 * to use the port configuration to set if it should be a pullup or pulldown.
 * Also it checks the mode of the pin and set it to that mode. If the mode
 * is PIN_ALTERNATE_FCT, we use GPIO multiplexing to set the mode of the pin.
 * This is used to set the SPI pins.
 *
 * (IMPORTANT!)
 * To choose the gpio multiplexing mode of a given pin, the "mode" parameter of the GpioMcuInit function must be
 * set to PIN_ALTERNATE_FCT, and the "value" parameter must be set to either:
 * - GPIO_MUL0
 * - GPIO_MUL1
 * - GPIO_MUL2
 * - GPIO_MUL3
 *
*/
void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
	//printf("GPIO mcu init\n");
	/*
	 * Lucas:
	 * Conditionals check which port the pin belong to. The pins are listed as an
	 * enum in gpio.h and defined in pinName-board.h. ADI_GPIO_PORT0 basically just
	 * maps to the integer 0 and ADI_GPIO_PORT1 maps to 1 and so on. This number
	 * is then assigned to the Gpio_t object that was passed to the function.
	 */
	if(pin < 16) {
		obj->port = &port0;
	}
	else if(pin < 32) {
		obj->port = &port1;
	}
	else if(pin < 48) {
		obj->port = &port2;
	}
	else if(pin < 52) {
		obj->port = &port3;
	}

	//printf("obj->port: %d\n", *(ADI_GPIO_PORT*)obj->port);

	//Set pin
	obj->pin = pin;

	//printf("obj->pin: %d\n", obj->pin);
	/*
	 * Lucas:
	 * Sets the pin index of the port register. The obj->pin & 0x0F masks the
	 * lowest four bits of obj->pin so that the value always wraps around the interval
	 * 0 to 15. So for example if the pin is 17, the obj->pin & 0x0F gives
	 * 0010001 & 0X0F = 0010001 & 00001111 = 0000001 = 1. So smart! In this way
	 * the pinIndex value always points to the correct register in the port.
	 * The register values for each register is found in adi_gpio.h.
	 *
	 * REMEMBER: after the obj->pin is masked in the line ( obj->pin & 0x0F ), we take the
	 * number 1 and left-shift it by the amount calculated in the mask. So pinIndex doesn't
	 * map f.eks. 21 to (21-16=5), but instead it would output (0000000000100000). So it
	 * makes a 16 bit number and puts a 1 in the place that corresponds to the pin.
	 */
	obj->pinIndex = ( 0x01 << ( obj->pin & 0x0F ) );

	/*
	* Lucas:
	* Initialize the pin type: PIN_NO_PULL, PIN_PULL_UP or PIN_PULL_DOWN.
	* PullDown is not available from the built in functions, but only from
	* registers. I have not found a function yet that uses pull-up/down though.
	*/
	ADI_GPIO_RESULT error_status = ADI_GPIO_SUCCESS;

	obj->pull = type;

	if(type == PIN_PULL_UP) {
		if(ADI_GPIO_SUCCESS != (error_status = adi_gpio_PullUpEnable(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)), true)))
		{
			DEBUG_MESSAGE("adi_gpio_InputEnable failed\n");
		}
	}

	/*
	 * Lucas:
	 * Initialize the pin based on input, output or GPIO multiplexing.
	 */
	if(mode == PIN_INPUT) {
		if(ADI_GPIO_SUCCESS != (error_status = adi_gpio_InputEnable(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)), true)))
		{
			DEBUG_MESSAGE("adi_gpio_InputEnable failed\n");
		}
	}
	else if(mode == PIN_OUTPUT) {
		if(ADI_GPIO_SUCCESS != (error_status = adi_gpio_OutputEnable(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)), true)))
		{
			DEBUG_MESSAGE("adi_gpio_OutputEnable failed\n");
		}
		//If the pin was set successfully, we set the inital value.
		else if(ADI_GPIO_SUCCESS == error_status) {
			if(value == 0) {
				adi_gpio_SetLow(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)));
			}
			else if(value == 1) {
				adi_gpio_SetHigh(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)));
			}
		}
	}
	/*
	 * Lucas:
	 * The last case here is if the pinMode is alternate. This is when we want
	 * to configure the GPIO multiplexing in the system, such as setting up the SPI
	 * related pins or I2C.
	 *
	 * THE CODE CAN BE OPTIMIZED BY LOOKING AT THE PATTERN OF THE ADRESSES OF THE CONFIGURATION REGISTERS.
	 */
	else if (mode == PIN_ALTERNATE_FCT)
	{
		if(pin < 16) {
			switch (value) {
			case GPIO_MUL0:
				break;
			case GPIO_MUL1:
				*((volatile uint32_t *) REG_GPIO0_CFG) |= ((uint16_t) ((uint16_t) (0x01 << ((obj->pin & 0x0F) * 2))));
				break;
			case GPIO_MUL2:
				*((volatile uint32_t *) REG_GPIO0_CFG) |= ((uint16_t) ((uint16_t) (0x02 << ((obj->pin & 0x0F) * 2))));
				break;
			case GPIO_MUL3:
				*((volatile uint32_t *) REG_GPIO0_CFG) |= ((uint16_t) ((uint16_t) (0x03 << ((obj->pin & 0x0F) * 2))));
				break;
			}
		}
		else if(pin < 32) {
			switch (value) {
			case GPIO_MUL0:
				break;
			case GPIO_MUL1:
				*((volatile uint32_t *) REG_GPIO1_CFG) |= ((uint16_t) ((uint16_t) (0x01 << ((obj->pin & 0x0F) * 2))));
				break;
			case GPIO_MUL2:
				*((volatile uint32_t *) REG_GPIO1_CFG) |= ((uint16_t) ((uint16_t) (0x02 << ((obj->pin & 0x0F) * 2))));
				break;
			case GPIO_MUL3:
				*((volatile uint32_t *) REG_GPIO1_CFG) |= ((uint16_t) ((uint16_t) (0x03 << ((obj->pin & 0x0F) * 2))));
				break;
			}
		}
		else if(pin < 48) {
			switch (value) {
			case GPIO_MUL0:
				break;
			case GPIO_MUL1:
				*((volatile uint32_t *) REG_GPIO2_CFG) |= ((uint16_t) ((uint16_t) (0x01 << ((obj->pin & 0x0F) * 2))));
				break;
			case GPIO_MUL2:
				*((volatile uint32_t *) REG_GPIO2_CFG) |= ((uint16_t) ((uint16_t) (0x02 << ((obj->pin & 0x0F) * 2))));
				break;
			case GPIO_MUL3:
				*((volatile uint32_t *) REG_GPIO2_CFG) |= ((uint16_t) ((uint16_t) (0x03 << ((obj->pin & 0x0F) * 2))));
				break;
			}
		}
	}


	/*
	 * Lucas:
	 * The example for B-L072Z creates some other functions for other PinModes
	 * and configurations, but i don't think we need them for now.
	 */
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
	obj->Context = context;
}

// DEBUG START
#define GPIOCallback_MAX 100
uint32_t GPIOcallbackCounter = 0;
void (*ExecutedGPIOCallbacks[GPIOCallback_MAX])(void* context);
// DEBUG END


/*
 * Lucas (12-11-23):
 * This function is called when an interrupt occurs on interrupt line A.
 * First it checks which pin triggered the interrupt. Next it finds out
 * where the interrupt came from and then calls the corresponding callback function.
 */
void AllPinsCallback(void* pCBParam, uint32_t Port, void* PinIntData) //uint32_t Pins
{
	//adi_gpio_Toggle(ADI_GPIO_PORT2, ADI_GPIO_PIN_0);
	/* Lucas (12-11-23):
	 * Loop for checking the interrupt status of all ports.
	 */
	// Find the pin which triggered the interrupt.
	uint32_t triggerPin = 0;
	for (int i = 0; i <= 15; i++) {
		if (*(uint32_t*) PinIntData & (1 << i)) {
			triggerPin = i;
		}
	}

	// Check which port the interrupt came from. And call the corresponding callback function.
	if (Port == (uint32_t) ADI_GPIO_PORT0) {
		if (array[0 + triggerPin] != NULL) {
			if(GPIOcallbackCounter < GPIOCallback_MAX) {
				ExecutedGPIOCallbacks[GPIOcallbackCounter++] = array[0 + triggerPin];
			}
			array[0 + triggerPin](NULL);
		}
	}
	else if (Port == (uint32_t) ADI_GPIO_PORT1) {
		if (array[16 + triggerPin] != NULL) {
			if(GPIOcallbackCounter < GPIOCallback_MAX) {
				ExecutedGPIOCallbacks[GPIOcallbackCounter++] = array[16 + triggerPin];
			}
			array[16 + triggerPin](NULL);
		}
	}
	else if (Port == (uint32_t) ADI_GPIO_PORT2) {
		if (array[32 + triggerPin] != NULL) {
			if(GPIOcallbackCounter < GPIOCallback_MAX) {
				ExecutedGPIOCallbacks[GPIOcallbackCounter++] = array[32 + triggerPin];
			}
			array[32 + triggerPin](NULL);
		}
	}
}
void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
 {
	obj->IrqHandler = irqHandler;
	//printf("Pin: %d\n", obj->pin);
	array[obj->pin] = irqHandler;

	if (irqMode == IRQ_RISING_EDGE)
	{
		//adi_gpio_SetGroupInterruptPolarity(*(ADI_GPIO_PORT*) obj->port, (ADI_GPIO_DATA) (obj->pinIndex));
	}
	//Sets pins, use pinsToEnable when testing the real program
	//adi_gpio_SetGroupInterruptPins(*(ADI_GPIO_PORT*) obj->port, SYS_GPIO_INTA_IRQn, (ADI_GPIO_DATA) (obj->pinIndex));

	/* Below: functions for handling the callbacks. Taken from Semtech */
	adi_gpio_SetGroupInterruptPolarity(ADI_GPIO_PORT1, ADI_GPIO_PIN_5 | ADI_GPIO_PIN_4 |ADI_GPIO_PIN_3 | ADI_GPIO_PIN_2);
	adi_gpio_SetGroupInterruptPins(ADI_GPIO_PORT1, ADI_GPIO_INTA_IRQ, ADI_GPIO_PIN_5 | ADI_GPIO_PIN_4 |ADI_GPIO_PIN_3 | ADI_GPIO_PIN_2);
	adi_gpio_RegisterCallback(SYS_GPIO_INTA_IRQn, AllPinsCallback, (void*) SYS_GPIO_INTA_IRQn);
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
	/*
	 * Lucas (12-11-23):
	 * This function is not used anywhere in loramac.
	*/
}

/*
 * Lucas:
 * Writes a value to a given gpio pin. The port is converted to the ADI_GPIO_PORT type
 * so it works with the SetHigh and SetLow functions.
 */
void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
	ADI_GPIO_RESULT eResult = ADI_GPIO_SUCCESS;

	if(value) {
		eResult = adi_gpio_SetHigh(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)));
		DEBUG_RESULT("Failed to set GPIO pin high",eResult,ADI_GPIO_SUCCESS);
	}
	else {
		eResult = adi_gpio_SetLow(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)));
		DEBUG_RESULT("Failed to set GPIO pin low",eResult,ADI_GPIO_SUCCESS);
	}
}

/*
 * Lucas:
 * Toggles a given GPIO pin
 */
void GpioMcuToggle( Gpio_t *obj )
{
	ADI_GPIO_RESULT eResult = ADI_GPIO_SUCCESS;

	eResult = adi_gpio_Toggle(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)));
	DEBUG_RESULT("Failed to toggle GPIO pin",eResult,ADI_GPIO_SUCCESS);
}

/*
 * Lucas:
 * Reads the logical level on an input pin.
 */
uint32_t GpioMcuRead( Gpio_t *obj )
{
	ADI_GPIO_RESULT eResult = ADI_GPIO_SUCCESS;

	uint16_t result = 0;

	eResult = adi_gpio_GetData(*(ADI_GPIO_PORT*)obj->port, ((ADI_GPIO_DATA)(obj->pinIndex)), &result);
	DEBUG_RESULT("Failed to read GPIO value",eResult,ADI_GPIO_SUCCESS);

	return result;
}

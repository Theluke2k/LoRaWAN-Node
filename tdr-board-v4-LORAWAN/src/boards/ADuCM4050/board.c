/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
#include "board-config.h"
#include "utilities.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "rtc-board.h"
#include "gps.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#include "board.h"
#include <sys/platform.h>
#include <drivers/general/adi_drivers_general.h>
#include "LoRaMac.h"
#include "gpio-board.h"

/*!
 * Unique Devices IDs register set ( STM32L0xxx )
 */
#define         ID1                                 ( 0x40002020 )
#define         ID2                                 ( 0x40002024 )
#define         ID3                                 ( 0x40002040 )

/**
 * GPIO device memory used by the driver.
 */
static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag used to indicate if board is powered from the USB
 */
static bool UsbIsConnected = true;

/*
 * Lucas:
 * This should work for ADI
 */
void BoardCriticalSectionBegin( uint32_t *mask )
{
	*mask  = __get_PRIMASK();
	__disable_irq();
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
	__set_PRIMASK( *mask );
}

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void )
{
/*Initializing peripherals is handled by Crosscore*/
}




/*!
 * \brief Initializes the mcu.
 */
void BoardInitMcu( void )
{
	if( McuInitialized == false )
	{
		/*
		 * Lucas (23/03/2024):
		 * Removed for merge
		 */
		SystemClockConfig();

		/*
		 * Lucas:
		 * External interrupts are not used in this our application.
		 */
		//xint_init();

		UsbIsConnected = true;

		/*
		 * Lucas:
		 * Initialize Gpio pins
		 */
		ADI_GPIO_RESULT gpioStatus = ADI_GPIO_SUCCESS;
		gpioStatus = adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);
		DEBUG_RESULT("GPIO init failed", gpioStatus, ADI_GPIO_SUCCESS);

		RtcInit();
	}
	else {
		/*
		 * Lucas (10-11-23):
		 * System clock reconfig here? The examples have it. Don't know if we need it.
		 */
	}
	SpiInit( &SX1276.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
	SX1276IoInit( );
	/*
	 * Lucas (10-11-23):
	 * The examples have the following code. I dont think that the
	 * SX1276IoDbgInot does anything useful but i have read that the
	 * SX1276IoTcxoInit controls the power that is input to the
	 * SX1276 radio. I think it is used to provide a stable clock
	 * signal to the radio which is important for frequency, timing etc.
	 * Ask Per if we need it.
	 */
    if( McuInitialized == false )
    {
        McuInitialized = true;
        //SX1276IoDbgInit( );
        //SX1276IoTcxoInit( );
    }
}

/*!
 * \brief Resets the mcu.
 */
void BoardResetMcu( void )
{
	/*
	 * Lucas (11-11-23):
	 * Maybe we need this at some point. There is a whole adi file
	 *called reset_ADuCM4050
	 */
}

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void )
{
	SpiDeInit( &SX1276.Spi );
	SX1276IoDeInit( );
}

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void )
{
	/*
	 * Lucas (11-11-23):
	 * This function is only needed for ABP (Activation By Personalization)
	 * The seed needs to be generated from some MCU specific values. ^ is XOR.
	*/
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint16_t BoardBatteryMeasureVoltage( void )
{
    return 0;
}

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value  battery voltage in volts
 */
uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

static void BoardUnusedIoInit( void )
{
	/*
	 * Lucas (11-11-23):
	 * The following are from the example. I don't think we need them.
	 */
    //HAL_DBGMCU_EnableDBGSleepMode( );
    //HAL_DBGMCU_EnableDBGStopMode( );
    //HAL_DBGMCU_EnableDBGStandbyMode( );
}

void SystemClockConfig( void ) {
	adi_pwr_Init();
	adi_pwr_SetHPBuckLoadMode(ADI_PWR_HPBUCK_LD_MODE_LOW);

	ADI_CLOCK_SOURCE_STATUS clock_status = ADI_CLOCK_SOURCE_ENABLED_NOT_STABLE;
	adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_HFXTAL, true);
	while (clock_status != ADI_CLOCK_SOURCE_ENABLED_STABLE) {
		adi_pwr_GetClockStatus(ADI_CLOCK_SOURCE_HFXTAL, &clock_status);
	};

	adi_pwr_SetRootClockMux(ADI_CLOCK_MUX_ROOT_HFXTAL);

	adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1);
	adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1);

	adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_LFXTAL, true);

	clock_status = ADI_CLOCK_SOURCE_ENABLED_NOT_STABLE;
	while (clock_status != ADI_CLOCK_SOURCE_ENABLED_STABLE) {
		adi_pwr_GetClockStatus(ADI_CLOCK_SOURCE_LFXTAL, &clock_status);
	};

	adi_pwr_SetLFClockMux(ADI_CLOCK_MUX_LFCLK_LFXTAL);

	adi_pwr_UpdateCoreClock();
}

void SystemClockReConfig( void )
{
    /*
     * Lucas (11-11-23):
     * I don't know if we need this function.
     */
}

void SysTick_Handler( void )
{
	/*
	 * Lucas (11-11-23):
	 * We probably do not need this. The code underneath is from
	 * the example.
	*/
    //HAL_IncTick( );
    //HAL_SYSTICK_IRQHandler( );
}

/*!
 * \brief Get the board power source
 *
 * \retval value  power source [0: USB_POWER, 1: BATTERY_POWER]
 */
uint8_t GetBoardPowerSource( void )
{
    if( UsbIsConnected == false )
    {
        return BATTERY_POWER;
    }
    else
    {
        return USB_POWER;
    }
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exists the function when waking up
  */
void LpmEnterStopMode( void)
{
	/*
	 * Lucas (11-11-23):
	 * This function should put the processor into shutdown mode (lowest power consumption).
	 * There must be some built-in functions for this from ADI somewhere.
	 * The code underneath is from the example for inspiration.
	 */
	/*
    CRITICAL_SECTION_BEGIN( );

    BoardDeInitMcu( );

    // Disable the Power Voltage Detector
    HAL_PWR_DisablePVD( );

    // Clear wake up flag
    SET_BIT( PWR->CR, PWR_CR_CWUF );

    // Enable Ultra low power mode
    HAL_PWREx_EnableUltraLowPower( );

    // Enable the fast wake up from Ultra low power mode
    HAL_PWREx_EnableFastWakeUp( );

    CRITICAL_SECTION_END( );

    // Enter Stop Mode
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
	*/
}

/*!
 * \brief Exists Low Power Stop Mode
 */
void LpmExitStopMode( void )
{
	/*
	 * Lucas (11-11-23):
	 * Exits shutdown mode på initializing the MCU again.
	 * The code underneath is from the example. We should wait
	 * activating this function until the LpmEnterStopMode function
	 * is written.
	 */
	/*
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );

    // Initilizes the peripherals
    BoardInitMcu( );

    CRITICAL_SECTION_END( );
    */
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode( void) // brug AU hibernation function.
{
	/*
	 * Lucas (11-11-23):
	 * Enter sleep mode. (not lowest power)
	 */
    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void BoardLowPowerHandler( void )
{
	/*
	 * Lucas (11-11-23):
	 * Basically just enter low power mode. But we need to disable
	 * interrupts first so that they are not pending and wake the MCU
	 * up at once after.
	 */
	/*
    __disable_irq( );

//    If an interrupt has occurred after __disable_irq( ), it is kept pending
//    and cortex will not enter low power anyway


    LpmEnterLowPower( );

    __enable_irq( );
    */
}

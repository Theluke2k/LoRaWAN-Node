/*! ****************************************************************************
 * @file    adi_wdt_config.h
 * @brief   WDT device driver configuration
 * -----------------------------------------------------------------------------
 * Copyright (c) 2016-2019 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * For license terms and conditions, please refer to the '*EZDFP Click Thru SLA'
 * file in the License subdirectory of this pack.
 *
 ******************************************************************************/
#ifndef ADI_WDT_CONFIG_H
#define ADI_WDT_CONFIG_H


/** @addtogroup WDT_Driver_Config Static Configuration
 *  @ingroup WDT_Driver
 *  @{
 */


/************* WDT Static Configuration ***************/

/*! WDT Timer Reload Value\n
    Value used to reload the WDT count register after count expires.\n
   0-65535 - WDT reload value (default is 0x0100).
*/
#define ADI_WDT_LOAD_VALUE                                  (0x1000u)

/*! WDT Timer Mode\n
   Selects WDT operating mode.\n
   0 -  WDT operates in free-running mode.\n
   1 -  WDT operates in periodic mode (default).
*/
#define ADI_WDT_CONTROL_TIMER_MODE                               (1u)

/*! WDT Clock Prescaler\n
   Controls WDT clock prescale.\n
   0 -  WDT operates at (source clock)/1.\n
   1 -  WDT operates at (source clock)/16.\n
   2 -  WDT operates at (source clock)/256 (default).\n
*/
#define ADI_WDT_CONTROL_CLOCK_PRESCALER                          (2u)

/*! WDT Timeout Mode\n
   Controls WDT timeout behaviour.\n
   0 -  WDT issues RESET on timeout (default).\n
   1 -  WDT issues INTERRUPT on timeout.
*/
#define ADI_WDT_CONTROL_TIMEOUT_MODE                             (0u)

/*! WDT Power Mode Disable\n
   Controls WDT countdown in hibernate or halted mode.\n
   0 -  WDT continues to count down when core is halted or in hibernate.\n
   1 -  WDT pauses count down when core is halted or in hibernate (default).\n
*/
#define ADI_WDT_CONTROL_POWER_MODE                               (1u)

/************** Macro Validation *****************************/

#if ( ADI_WDT_LOAD_VALUE  > 65535u )
#error "Invalid configuration"
#endif

#if ( ADI_WDT_CONTROL_TIMER_MODE > 1u )
#error "Invalid configuration"
#endif

#if ( ADI_WDT_CONTROL_CLOCK_PRESCALER > 2u )
#error "Invalid configuration"
#endif

#if ( ADI_WDT_CONTROL_TIMEOUT_MODE > 1u )
#error "Invalid configuration"
#endif

#if ( ADI_WDT_CONTROL_POWER_MODE > 1u )
#error "Invalid configuration"
#endif

/**
 *  @}
 */

#endif /* ADI_WDT_CONFIG_H */

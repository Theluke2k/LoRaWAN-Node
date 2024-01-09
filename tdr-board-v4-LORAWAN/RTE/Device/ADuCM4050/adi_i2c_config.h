/*! ****************************************************************************
 * @file:    adi_i2c_config.h
 * @brief:   Configuration options for I2C driver.
 *           This is specific to the I2C driver and will be included by the driver.
 *           It is not required for the application to include this header file.
 * -----------------------------------------------------------------------------
 * Copyright (c) 2016-2019 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * For license terms and conditions, please refer to the '*EZDFP Click Thru SLA'
 * file in the License subdirectory of this pack.
 *
 ******************************************************************************/
#ifndef ADI_I2C_CONFIG_H
#define ADI_I2C_CONFIG_H
#include <adi_global_config.h>

/** @addtogroup I2C_Driver_Config Static Configuration
 *  @ingroup I2C_Driver
 *  @{
 */

/************* I2C Driver configurations ***************/

/*! Master control register TX FIFO decrement control bit.\n
    1 - Decrement master TX FIFO status when a byte has been fully serialized.\n
    0 - Decrement master TX FIFO status when a byte is unloaded from the TX FIFO,
        but not yet serialized on the bus. */
#define ADI_I2C_CFG_MCTL_MXMITDEC               (0)

/*! Master control register STOP condition interrupt enable.\n
    1 - Enable completion interrupt when a STOP condition is detected.\n
    0 - Disable completion interrupt when a STOP condition is detected. */
#define ADI_I2C_CFG_MCTL_IENCMP                 (1)

/*! Master control register NACK (NotACKnowledge) interrupt enable.\n
    1 - Enable NACK interrupt when an acknowledge is not received.\n
    0 - Disable NACK interrupt when an acknowledge is not received. */
#define ADI_I2C_CFG_MCTL_IENACK                 (1)

/*! Master control register ALOST (Arbitration LOST) interrupt enable.\n
    1 - Enable ALOST interrupt when bus arbitration is lost.\n
    0 - Disable ALOST interrupt when bus arbitration is lost. */
#define ADI_I2C_CFG_MCTL_IENALOST               (1)

/*! Master control register clock stretch enable.\n
    1 - Enable clock stretch by slave device.\n
    0 - Disable clock stretch by slave device. */
#define ADI_I2C_CFG_MCTL_STRETCHSCL             (0)

/*! Master control register internal loopback enable.\n
    1 - Enable internal looping of SCL and SDA outputs onto their corresponding inputs.\n
    0 - Disable internal looping of SCL and SDA outputs onto their corresponding inputs. */
#define ADI_I2C_CFG_MCTL_LOOPBACK               (0)

/*! Master control register start condition back-off disable.\n
    1 - Enables controller to compete for bus ownership even if another device is driving a START condition.\n
    0 - Disables controller to compete for bus ownership even if another device is driving a START condition. */
#define ADI_I2C_CFG_MCTL_COMPLETE               (0)

/*! Master control register device enable.\n
    1 - Enable controller as a Master device.\n
    0 - Disables controller as a Master device. */
#define ADI_I2C_CFG_MCTL_MASEN                  (0)

/*! Master control register busclear enable.\n
    1-Enable busclear bit for generating extra nine clock cycles when I2C bus hangs
    0-Disable busclear bit. */
#define ADI_I2C_CFG_MCTL_BUSCLR                 (1)

/*! Master control register stopbusclear enable.\n
    1-Enable stopbusclear bit for stopping the master to generate extra clock cycles when SDA release
    0-Disable stopbusclear bit. */
#define ADI_I2C_CFG_MCTL_STOPBUSCLR             (1)


/*!
 * Standard Clock divider Clock-HI settings.
 * Assuming a 26 MHz core clock, the following settings
 * will be useful:                                           \n
 *     - For STANDARD (100 kHz) rate, use: HI= 25, LO= 31.   \n
 *     - For FAST (400 kHz) rate, use:     HI=123, LO=129.   \n
 *                                                           \n
 * @note The clock high setting varies with pull-up loading,
 * board layout, slew-rate, etc., so exact settings are somewhat
 * empirical.  The clock high counter does not start until
 * a logic high transition is sensed on the clock line, so
 * variability in this logic transaction will alter the
 * effective clock rate.  This results from the internal
 * clock-stretch hardware feature supporting a slave slow device
 * that may hold off the master by holding the clock line low.
 *
 * @sa ADI_I2C_CFG_DIV_LOW
 */
#define ADI_I2C_CFG_DIV_HIGH                    (25)

/*! Standard Clock divider Clock-LO setting
 *
 * @sa ADI_I2C_CFG_DIV_HIGH
 */
#define ADI_I2C_CFG_DIV_LOW                     (31)

/*! Shared control reset START/STOP detect circuit.\n
    1 - Reset the SCL and SDA synchronizers, START/STOP detect logic, and LINEBUSY detect logic.\n
    0 - Do nothing. */
#define ADI_I2C_CFG_SHCTL_RST                   (0)

/*! Timing control filter disable.\n
    1 - Disable digital input clock filter.\n
    0 - Enable digital input clock filter (1 PCLK). */
#define ADI_I2C_CFG_TCTL_FILTEROFF              (0)

/*! Timing control data input hold time requirement to recognize START/STOP condition (5-bit max).\n
    Value - Minimum data input hold time count in units of PCLK period.  (Value = Thd/PCLK-period) */
#define ADI_I2C_CFG_TCTL_THDATIN                (1)

/*! Master automatic stretch mode duration (4-bit), e.g., (in binary):\n
    - 0b0000 - No SCL clock stretching.\n
    - 0b0001 - Timeout after hold SCL LOW 2^1 = 2 bit-times.\n
    - 0b0010 - Timeout after hold SCL LOW 2^2 = 4 bit-times.\n
    - ...\n
    - 0b1110 - Timeout after hold SCL LOW 2^14 = 16,384 bit-times.\n
    - 0b1111 - Hold SCL LOW with no timeout.\n
\n
    Where "bit-time" is computed by CLKDIV values and incoming UCLK (see HRM). */
#define ADI_I2C_CFG_ASTRETCH_MST                (0)

/*! Unformatted, 7-bit max width I2C "7-bit Addressing" slave device address value (unshifted and excluding R/W direction bit).\n
    For example, the value:\n
        0x50 - Is the "raw" (unencoded) slave address for the "Aardvark Activity Board" ATMEL AT24C02 I2C slave EEPROM device.\n
            It is encoded (upshifted by one and ORed with R/W direction bit) on the I2C bus as:\n
                - 0xA0 for write operations, or\n
                - 0xA1 for read operations */
#define ADI_I2C_CFG_SLAVE_ADDRESS              (0x50)


/***********************************\
|* Check for overflowing values... *|
\***********************************/

#if (ADI_I2C_CFG_MCTL_MXMITDEC >> 1)
#error "Decrement TX FIFO status config value too wide"
#endif

#if (ADI_I2C_CFG_MCTL_IENCMP >> 1)
#error "Transaction complete (STOP) interrupt enable config value too wide"
#endif

#if (ADI_I2C_CFG_MCTL_IENACK >> 1)
#error "NACK interrupt enable config value too wide"
#endif

#if (ADI_I2C_CFG_MCTL_IENALOST >> 1)
#error "ALOST interrupt enable config value too wide"
#endif

#if (ADI_I2C_CFG_MCTL_STRETCHSCL >> 1)
#error "Clock stretch enable config value too wide"
#endif

#if (ADI_I2C_CFG_MCTL_LOOPBACK >> 1)
#error "Loopback enable config value too wide"
#endif

#if (ADI_I2C_CFG_MCTL_COMPLETE >> 1)
#error "Start back-off disable config value too wide"
#endif

#if (ADI_I2C_CFG_MCTL_MASEN >> 1)
#error "Master device enable config value too wide"
#endif

#if (ADI_I2C_CFG_DIV_HIGH >> 8)
#error "Clock HIGH time config value too wide"
#endif

#if (ADI_I2C_CFG_DIV_LOW >> 8)
#error "Clock LOW time config value too wide"
#endif

#if (ADI_I2C_CFG_SHCTL_RST >> 1)
#error "Shared control reset config value too wide"
#endif

#if (ADI_I2C_CFG_TCTL_FILTEROFF >> 1)
#error "Timing control filter-off config value too wide"
#endif

#if (ADI_I2C_CFG_TCTL_THDATIN >> 5)
#error "Timing control filter-off config value too wide"
#endif

#if (ADI_I2C_CFG_ASTRETCH_MST >> 4)
#error "Master clock stretch config value too wide"
#endif

#if (ADI_I2C_CFG_SLAVE_ADDRESS >> 7)
#error "Slave address config value too wide"
#endif

/*! @} */

#endif /* ADI_I2C_CONFIG_H */

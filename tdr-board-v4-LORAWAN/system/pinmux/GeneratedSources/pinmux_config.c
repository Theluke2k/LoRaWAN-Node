/*
 **
 ** Source file generated on december 9, 2024 at 12:08:25.	
 **
 ** Copyright (C) 2011-2024 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in 
 ** the Pin Multiplexing configuration editor. Changes to the Pin Multiplexing
 ** configuration should be made by changing the appropriate options rather
 ** than editing this file.
 **
 ** Selected Peripherals
 ** --------------------
 ** SPI2 (CLK, MOSI, MISO, CS_0, CS_1, CS_3)
 ** I2C0 (SCL, SDA)
 ** UART0 (Tx, Rx)
 ** UART1 (Tx, Rx)
 ** ADC0_IN (ADC0_VIN1, ADC0_VIN2, ADC0_VIN3)
 **
 ** GPIO (unavailable)
 ** ------------------
 ** P0_04, P0_05, P0_09, P0_10, P0_11, P1_02, P1_03, P1_04, P1_05, P1_15, P2_00, P2_04,
 ** P2_05, P2_06, P2_07
 */

#include <sys/platform.h>
#include <stdint.h>

#define SPI2_CLK_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI2_MOSI_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<6))
#define SPI2_MISO_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define SPI2_CS_0_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<10))
#define SPI2_CS_1_PORTP0_MUX  ((uint32_t) ((uint32_t) 2<<18))
#define SPI2_CS_3_PORTP2_MUX  ((uint16_t) ((uint16_t) 2<<14))
#define I2C0_SCL_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define I2C0_SDA_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<10))
#define UART0_TX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<20))
#define UART0_RX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<22))
#define UART1_TX_PORTP1_MUX  ((uint32_t) ((uint32_t) 2<<30))
#define UART1_RX_PORTP2_MUX  ((uint16_t) ((uint16_t) 2<<0))
#define ADC0_IN_ADC0_VIN1_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define ADC0_IN_ADC0_VIN2_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<10))
#define ADC0_IN_ADC0_VIN3_PORTP2_MUX  ((uint16_t) ((uint16_t) 1<<12))

int32_t adi_initpinmux(void);

/*
 * Initialize the Port Control MUX Registers
 */
int32_t adi_initpinmux(void) {
    /* PORTx_MUX registers */
    *pREG_GPIO0_CFG = SPI2_CS_1_PORTP0_MUX | I2C0_SCL_PORTP0_MUX
     | I2C0_SDA_PORTP0_MUX | UART0_TX_PORTP0_MUX | UART0_RX_PORTP0_MUX;
    *pREG_GPIO1_CFG = SPI2_CLK_PORTP1_MUX | SPI2_MOSI_PORTP1_MUX
     | SPI2_MISO_PORTP1_MUX | SPI2_CS_0_PORTP1_MUX | UART1_TX_PORTP1_MUX;
    *pREG_GPIO2_CFG = SPI2_CS_3_PORTP2_MUX | UART1_RX_PORTP2_MUX
     | ADC0_IN_ADC0_VIN1_PORTP2_MUX | ADC0_IN_ADC0_VIN2_PORTP2_MUX | ADC0_IN_ADC0_VIN3_PORTP2_MUX;

    return 0;
}


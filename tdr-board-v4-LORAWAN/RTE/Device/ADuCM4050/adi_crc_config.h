/*! ****************************************************************************
 * @file:    adi_crc_config.h
 * @brief:   Configuration options for CRC driver.
 *           This is specific to the CRC driver and will be included by the driver.
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
#ifndef ADI_CRC_CONFIG_H
#define ADI_CRC_CONFIG_H

#include <adi_global_config.h>

/** @defgroup CRC_Driver_Cfg Static Configuration
 *  @ingroup CRC_Driver
 */

/** @addtogroup CRC_Driver_Cfg Static Configuration
*  @{
*/

/************* CRC Driver configurations ***************/
/*!
   Enable DMA support in the driver code.\n
   1 -  To have the DMA support code in the driver.\n
   0 -  To eliminate the DMA support. Operates in core mode.\n
*/
#define ADI_CRC_CFG_ENABLE_DMA_SUPPORT         0

/*!
   Enable Byte mirroring option\n
   1 -  To enable byte mirroring \n
   0 -  To disable the byte mirroring.
*/
#define ADI_CFG_CRC_ENABLE_BYTE_MIRRORING      0
/*!
   Enable Bit mirroring option\n
   1 -  To enable bit mirroring \n
   0 -  To disable the bit mirroring.
*/
#define ADI_CFG_CRC_ENABLE_BIT_MIRRORING       0

/*!
   To specify the seed value for CRC computation
*/

#define ADI_CFG_CRC_SEED_VALUE                 (0xFFFFFFFFu)

/*!
   To specify the polynomial to be used for CRC computation
*/
#define ADI_CFG_CRC_POLYNOMIAL                 (0x04C11DB7u)

/*!
   To specify the Software DMA channel to be used for the CRC computation
   0 -> DMA channel SIP0, ..., 7 -> DMA channel SIP7
*/
#define ADI_CFG_CRC_SOFTWARE_DMA_CHANNEL_ID     7

#endif /* ADI_CRC_CONFIG_H */
/*! @} */

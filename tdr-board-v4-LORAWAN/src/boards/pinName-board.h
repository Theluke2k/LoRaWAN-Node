/*!
 * \file      pinName-board.h
 *
 * \brief     Target board GPIO pins definitions
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
 */
#ifndef __PIN_NAME_BOARD_H__
#define __PIN_NAME_BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Lucas:
 * ADuCM4050 pin names. I don't know if the ADuCM4050 has more pins
 */
#define MCU_PINS \
    P0_0 = 0, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, P0_9, P0_10, P0_11, P0_12, P0_13, P0_14, P0_15, \
    P1_0, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7, P1_8, P1_9, P1_10, P1_11, P1_12, P1_13, P1_14, P1_15,     \
    P2_0, P2_1, P2_2, P2_3, P2_4, P2_5, P2_6, P2_7, P2_8, P2_9, P2_10, P2_11, P2_12, P2_13, P2_14, P2_15,     \
    P3_0, P3_1, P3_2, P3_3

#ifdef __cplusplus
}
#endif

#endif // __PIN_NAME_BOARD_H__

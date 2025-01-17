/*!
 * \file      se-identity.h
 *
 * \brief     Secure Element identity and keys
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
 *              (C)2020 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 */
#ifndef __SOFT_SE_IDENTITY_H__
#define __SOFT_SE_IDENTITY_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 ******************************************************************************
 ********************************** WARNING ***********************************
 ******************************************************************************
  The secure-element implementation supports both 1.0.x and 1.1.x LoRaWAN
  versions of the specification.
  Thus it has been decided to use the 1.1.x keys and EUI name definitions.
  The below table shows the names equivalence between versions:
               +---------------------+-------------------------+
               |       1.0.x         |          1.1.x          |
               +=====================+=========================+
               | LORAWAN_DEVICE_EUI  | LORAWAN_DEVICE_EUI      |
               +---------------------+-------------------------+
               | LORAWAN_APP_EUI     | LORAWAN_JOIN_EUI        |
               +---------------------+-------------------------+
               | LORAWAN_GEN_APP_KEY | LORAWAN_APP_KEY         |
               +---------------------+-------------------------+
               | LORAWAN_APP_KEY     | LORAWAN_NWK_KEY         |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_F_NWK_S_INT_KEY |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_S_NWK_S_INT_KEY |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_NWK_S_ENC_KEY   |
               +---------------------+-------------------------+
               | LORAWAN_APP_S_KEY   | LORAWAN_APP_S_KEY       |
               +---------------------+-------------------------+
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************
 */

/*!
 * When set to 1 DevEui is LORAWAN_DEVICE_EUI
 * When set to 0 DevEui is automatically set with a value provided by MCU platform
 */
#define STATIC_DEVICE_EUI                                  1

/*!
 * end-device IEEE EUI (big endian)
 */
#define LORAWAN_DEVICE_EUI                                 { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06 }

/*!
 * App/Join server IEEE EUI (big endian)
 */
#define LORAWAN_JOIN_EUI                                   { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/*!
 * Secure-element pin
 */
#define SECURE_ELEMENT_PIN                                 { 0x00, 0x00, 0x00, 0x00 }

/*!
 * When set to 1 DevAddr is LORAWAN_DEVICE_ADDRESS
 * When set to 0 DevAddr is automatically set with a value provided by a pseudo
 *      random generator seeded with a value provided by the MCU platform
 */
#define STATIC_DEVICE_ADDRESS                              1

/*!
 * Device address on the network (big endian)
 */
#define LORAWAN_DEVICE_ADDRESS                             ( uint32_t )0x12345678

/*
 * 1.1 OTAA
 * DevEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
 * JoinEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 * AppKey: 0x6F, 0x74, 0x33, 0x1C, 0x62, 0xD6, 0x6E, 0x99, 0x29, 0x86, 0x1A, 0x34, 0x87, 0x53, 0x7D, 0xC0
 * NwkSKey:
 * AppSKey:
 */

/*
 * 1.0 OTAA
 * DevEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02
 * JoinEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 * AppKey: 0x49, 0x0B, 0x1C, 0x4C, 0xC2, 0x51, 0x3C, 0x83, 0x4B, 0xBF, 0x94, 0x4D, 0x3B, 0x8E, 0xDD, 0x68
 * NwkSKey: 0xDE, 0xBD, 0x05, 0x9F, 0xCD, 0x67, 0x65, 0x32, 0x50, 0x1E, 0xAA, 0x32, 0xEC, 0x09, 0x94, 0xD6
 * AppSKey: 0xF3, 0xBB, 0x5A, 0xEF, 0x0A, 0x8D, 0x74, 0xAD, 0x87, 0xF6, 0x74, 0xA0, 0xB5, 0x1E, 0x9A, 0x07
 *
 */

/*
 * 1.0 OTAA 2
 * DevEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04
 * JoinEUI: 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 * AppKey:  0x90, 0x46, 0x41, 0x01, 0x14, 0x87, 0xBC, 0xDE, 0x57, 0x4F, 0xD7, 0x5C, 0x03, 0xB3, 0x03, 0x77
 * NwkSKey: 0x96, 0x5E, 0x63, 0xE7, 0xC9, 0x82, 0x22, 0x27, 0x72, 0xE8, 0x93, 0x4D, 0x15, 0x6E, 0x3C, 0x62
 * AppSKey: 0x69, 0x35, 0x9B, 0xED, 0xF0, 0xE6, 0x1D, 0xD9, 0x8D, 0xC6, 0x2C, 0xD2, 0x4A, 0x8C, 0xF0, 0x2A
 *
 */

/*
 * 1.0 ABP
 * DevEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03
 * DevAddr: 0x12345678
 * AppKey: 0xCA, 0x19, 0x22, 0xC8, 0xFB, 0xCB, 0xF7, 0x18, 0x80, 0x61, 0x37, 0x1A, 0x8F, 0x36, 0x3A, 0xBA
 * NwkSKey: 0xDE, 0xBD, 0x05, 0x9F, 0xCD, 0x67, 0x65, 0x32, 0x50, 0x1E, 0xAA, 0x32, 0xEC, 0x09, 0x94, 0xD6
 * AppSKey:
 *
 */

/*
 * 1.0 OTAA 3 "NewTestDevice"
 * DevEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05
 * JoinEUI: 0x12, 0x12, 0x12, 0x12, 0x13, 0x14, 0x14, 0x14
 * AppKey:  0x90, 0x46, 0x41, 0x01, 0x14, 0x87, 0xBC, 0xDE, 0x57, 0x4F, 0xD7, 0x5C, 0x03, 0xB3, 0x03, 0x77
 * NwkSKey: 0xF3, 0xE1, 0x85, 0xCD, 0x9E, 0x4F, 0x4F, 0x37, 0xE5, 0x2A, 0x1E, 0x57, 0x63, 0x78, 0xC2, 0xF9
 * AppSKey: 0xD7, 0x78, 0xEC, 0xF0, 0xD7, 0x61, 0xD1, 0xCD, 0xDD, 0x0D, 0x62, 0x5B, 0x76, 0x23, 0x35, 0x03
 *
 */

/*
 * 1.0 OTAA 3 "NewTestDevice2"
 * DevEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06
 * JoinEUI: 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 * AppKey:  0x90, 0x46, 0x41, 0x01, 0x14, 0x87, 0xBC, 0xDE, 0x57, 0x4F, 0xD7, 0x5C, 0x03, 0xB3, 0x03, 0x77
 * NwkSKey: 0x98, 0x62, 0x4F, 0x70, 0x74, 0xAB, 0x23, 0x98, 0xF3, 0xD3, 0x6B, 0xEB, 0x38, 0x35, 0xB5, 0x4B
 * AppSKey: 0x30, 0x73, 0x24, 0x63, 0x06, 0xB9, 0x75, 0x42, 0x00, 0x98, 0xC2, 0xB5, 0x90, 0x22, 0xFD, 0xE2
 *
 */

#define SOFT_SE_KEY_LIST                                                                                            \
    {                                                                                                               \
        {                                                                                                  \
            /*!                                                                                                     \
             * Application root key                                                                                 \
             * WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_GEN_APP_KEY                                        \
             */                                                                                                     \
            .KeyID    = APP_KEY,                                                                                    \
            .KeyValue = { 0x90, 0x46, 0x41, 0x01, 0x14, 0x87, 0xBC, 0xDE, 0x57, 0x4F, 0xD7, 0x5C, 0x03, 0xB3, 0x03, \
                          0x77 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Network root key                                                                                     \
             * WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_APP_KEY                                            \
             */                                                                                                     \
            .KeyID    = NWK_KEY,                                                                                    \
            .KeyValue = { 0x90, 0x46, 0x41, 0x01, 0x14, 0x87, 0xBC, 0xDE, 0x57, 0x4F, 0xD7, 0x5C, 0x03, 0xB3, 0x03, \
                          0x77 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Join session integrity key (Dynamically updated)                                                     \
             * WARNING: NOT USED FOR 1.0.x DEVICES                                                                  \
             */                                                                                                     \
            .KeyID    = J_S_INT_KEY,                                                                                \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Join session encryption key (Dynamically updated)                                                    \
             * WARNING: NOT USED FOR 1.0.x DEVICES                                                                  \
             */                                                                                                     \
            .KeyID    = J_S_ENC_KEY,                                                                                \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Forwarding Network session integrity key                                                             \
             * WARNING: NWK_S_KEY FOR 1.0.x DEVICES                                                                 \
             */                                                                                                     \
            .KeyID    = F_NWK_S_INT_KEY,                                                                            \
            .KeyValue = { 0xF3, 0xE1, 0x85, 0xCD, 0x9E, 0x4F, 0x4F, 0x37, 0xE5, 0x2A, 0x1E, 0x57, 0x63, 0x78, 0xC2, \
            		      0xF9 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Serving Network session integrity key                                                                \
             * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY                \
             */                                                                                                     \
            .KeyID    = S_NWK_S_INT_KEY,                                                                            \
            .KeyValue = { 0x96, 0x5E, 0x63, 0xE7, 0xC9, 0x82, 0x22, 0x27, 0x72, 0xE8, 0x93, 0x4D, 0x15, 0x6E, 0x3C, \
                          0x62 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Network session encryption key                                                                       \
             * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY                \
             */                                                                                                     \
            .KeyID    = NWK_S_ENC_KEY,                                                                              \
            .KeyValue = { 0x96, 0x5E, 0x63, 0xE7, 0xC9, 0x82, 0x22, 0x27, 0x72, 0xE8, 0x93, 0x4D, 0x15, 0x6E, 0x3C, \
                          0x62 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Application session key                                                                              \
             */                                                                                                     \
            .KeyID    = APP_S_KEY,                                                                                  \
            .KeyValue = { 0xD7, 0x78, 0xEC, 0xF0, 0xD7, 0x61, 0xD1, 0xCD, 0xDD, 0x0D, 0x62, 0x5B, 0x76, 0x23, 0x35, \
            		      0x03 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast root key (Dynamically updated)                                                             \
             */                                                                                                     \
            .KeyID    = MC_ROOT_KEY,                                                                                \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast key encryption key (Dynamically updated)                                                   \
             */                                                                                                     \
            .KeyID    = MC_KE_KEY,                                                                                  \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_0,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_0,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_0,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #1 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_1,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #1 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_1,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #1 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_1,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #2 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_2,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #2 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_2,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #2 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_2,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #3 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_3,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #3 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_3,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #3 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_3,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * All zeros key. (ClassB usage)(constant)                                                              \
             */                                                                                                     \
            .KeyID    = SLOT_RAND_ZERO_KEY,                                                                         \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
    },

#ifdef __cplusplus
}
#endif

#endif  //  __SOFT_SE_IDENTITY_H__

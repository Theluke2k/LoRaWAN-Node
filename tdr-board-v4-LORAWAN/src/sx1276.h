/*
 * lora.h
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#ifndef SX1276_H_
#define SX1276_H_

#include "inttypes.h"
/*
 *
 * LORA (SX1276) SPECIFIC
 *
 */

/*
 * Maximum length of the SX1276 packet that can be processed (TX/RX).
 */
#define SX1276_MAX_PACKET	256

/*
 * Frequency bands compatibile with SX1276. We use 868MHz in Europe.
 */
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

/*
 * Some useful macros for steering the SX1276 module.
 */
#define SX1276_LORA_MODE				(0x01 << 7)
#define SX1276_FSK_MODE					0x00
#define SX1276_SLEEP					0x00
#define SX1276_STANDBY					0x01
#define SX1276_TX						0x03
#define SX1276_RX_CONT					0x05
#define SX1276_RX_SINGLE				0x06

#define SX1276_PACONFIG_BOOST				(0x01 << 7)
#define SX1276_PACONFIG_MAXPOWER_DEFAULT	(0x04 << 4)
#define SX1276_PACONFIG_OUTPUTPOWER_MAX		0x0F

#define SX1276_PADAC_HIGHPOWER				0x87

#define SX1276_LNA_GAIN_MAX					(0x01 << 5)
#define SX1276_LNA_BOOST_HF					0x03

#define SX1276_MODEM3_AGCAUTOON				(0x01 << 2)

#define LORA_SYNC_WORD						0x34

#define SX1276_IRQFLAGSMASK_TXDONE			(0x01 << 3)
#define SX1276_IRQFLAGS_TXDONE				(0x01 << 3)

#define SX1276_DIOMAPPING1_TXDONE			(0x01 << 6)


void lora_reset();
uint8_t lora_initialize();
uint8_t lora_send_frame(uint8_t* data, uint8_t packet_length);

/* RX mode */
uint16_t lora_parse_packet();
int8_t lora_available();
int8_t lora_read();
int8_t lora_waitACK(char* ackMsg, uint16_t timeOutDelay);

uint8_t fsk_initialize();
void fsk_send_frame(uint8_t address, uint8_t* data, uint8_t packet_length, uint8_t request_ack, uint8_t send_ack);

#endif /* SX1276_H_ */

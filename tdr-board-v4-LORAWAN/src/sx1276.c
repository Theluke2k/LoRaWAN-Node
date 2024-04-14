/*
 * lora.c
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#include "sx1276.h"
#include "spi-au.h"
#include "sx1276_registers.h"
#include "gpio-au.h"
#include "shared.h"


uint16_t _packetIndex;


/*
 * @brief Performs a reset of the SX1276.
 */
void lora_reset() {
    adi_gpio_SetLow(LORA_RST_PORT, LORA_RST_PIN);
    delay(10);
    adi_gpio_SetHigh(LORA_RST_PORT, LORA_RST_PIN);
    delay(10);
}

/*
 * Commands seen underneath are based on many drivers and SX1276 documentation.
 */

/*
 * @brief SX1276 init function for LoRA communication.
 *
 * Set up for 868MHz, default SF and BW. Also, max power is selected for transmit (20dBm) and for the receive.
 */
uint8_t lora_initialize() {
    lora_reset();

    uint8_t loRaVersion;

    /* Recover the LoRa module version */
    spi_read_byte(REG_VERSION, &loRaVersion);
    if (loRaVersion != SX1276_ID)
        return 1;

    /* Set the module in sleep mode */
    spi_write_byte(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);

//    /* Set the frequency to 868 MHz */
//    spi_write_byte(REG_FRF_MSB, 0xD9);
//    spi_write_byte(REG_FRF_MID, 0x00);
//    spi_write_byte(REG_FRF_LSB, 0x00);
//
//    /* Set base addresses */
//    spi_write_byte(REG_FIFO_TX_BASE_ADDR, 0x00);
//    spi_write_byte(REG_FIFO_RX_BASE_ADDR, 0x00);
//
//    /* Set LNA boost */
//    uint8_t lnaReg;
//    spi_read_byte(REG_LNA, &lnaReg);
//    spi_write_byte(REG_LNA, lnaReg | 0x03);
//
//    /* Set the output power at +20 dBm */
//    spi_write_byte(REG_PA_DAC, 0x87);
//    uint8_t paConfig;
//    spi_read_byte(REG_PA_CONFIG, &paConfig);
//    spi_write_byte(REG_PA_CONFIG, paConfig | PA_BOOST);
//
//    /* Set OCP */
//    uint8_t ocpTrim = (140 + 30) / 10; /* Here 140 is the current limit in mA */
//    spi_write_byte(REG_OCP, 0x20 | (0x1F & ocpTrim));
//
//    /* Set the module in LoRa and standby mode */
//    spi_write_byte(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    return 0;
}


/*
 * @brief Sends a frame via LoRA.
 *
 * Broadcasts a frame with 4-byte header and data provided by the user. A very simple communication with pure LoRA, not LoRAWAN.
 *
 * @param[in] address "Address" of the LoRA node, can be used to easily distinguish the node which sent the data.
 * @param[in] data Pointer to the data to be sent (bytes)
 * @param[in] packet_length Packet length (data only)
 */
uint8_t lora_send_frame(uint8_t* data, uint8_t packet_length) {
    uint8_t currentLength;

    /* Set the module in LoRa and standby mode */
    spi_write_byte(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    /* Reset FIFO address and payload length */
    spi_write_byte(REG_FIFO_ADDR_PTR, 0x00);
    spi_write_byte(REG_PAYLOAD_LENGTH, 0x00);

    /* Check packet length */
    spi_read_byte(REG_PAYLOAD_LENGTH, &currentLength);
    if ((currentLength + packet_length) > MAX_PKT_LENGTH) {
        packet_length = MAX_PKT_LENGTH - currentLength;
    }

    /* Update length */
    spi_write_byte(REG_PAYLOAD_LENGTH, currentLength + packet_length);

    /* Write data */
    for (size_t i = 0; i < packet_length; i++) {
        spi_write_byte(REG_FIFO, data[i]);
    }

    /* Put module in TX mode */
    spi_write_byte(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    /* Wait for TX done */
    uint8_t txDone = 0;

    while (txDone & IRQ_TX_DONE_MASK) {
        spi_read_byte(REG_IRQ_FLAGS, &txDone);
    }

	/* Clear IRQ's */
	spi_write_byte(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

	/* Set the module in sleep mode */
	spi_write_byte(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
	/* WARNING !!!
	 * If you put the LoRa module in sleep mode, you need to call the initialize function before use it again */

    return 0;
}

/* Check if a packet was received */
uint16_t lora_parse_packet() {
    uint8_t packetLength = 0;
    uint8_t irqFlags;
    uint8_t regOpMode;
    spi_read_byte(REG_OP_MODE, &regOpMode);
    spi_read_byte(REG_IRQ_FLAGS, &irqFlags);

    /* Clear IRQ's */
    spi_write_byte(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        /* Received a packet */
        _packetIndex = 0;

        /* Read packet length */
        spi_read_byte(REG_RX_NB_BYTES, &packetLength);

        /* Set FIFO address to current RX address */
        uint8_t fifoRxAddr;
        spi_read_byte(REG_FIFO_RX_CURRENT_ADDR, &fifoRxAddr);
        spi_write_byte(REG_FIFO_ADDR_PTR, 0x00);

        /* Set the module in LoRa and standby mode */
        spi_write_byte(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    } else if (regOpMode != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        /* Not currently in RX mode */

        /* Reset FIFO address */
        spi_write_byte(REG_FIFO_ADDR_PTR, 0x00);

        /* Put it in single RX mode */
        spi_write_byte(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

/* Check if there is available bytes in the FIFO */
int8_t lora_available() {
    uint8_t byte;
    spi_read_byte(REG_RX_NB_BYTES, &byte);
    uint16_t nbBytes = byte - _packetIndex;
    return nbBytes;
}

/* Read the next byte into the FIFO, and increment the FIFO pointer */
int8_t lora_read() {
    if (!lora_available()) {
        return -1;
    }

    _packetIndex++;
    uint8_t regFifo;
    spi_read_byte(REG_FIFO, &regFifo);

    return regFifo;
}

/* Check if an acknowledge was received */
int8_t lora_waitACK(char* ackMsg, uint16_t timeOutDelay) {
    char rxBuffer[128] = {0};
    uint8_t packetSize = 0;

    uint32_t clockFreq = getClkFreq(ADI_CLOCK_HCLK);
    uint32_t delay_val = clockFreq / 100000;
    delay_val *= timeOutDelay;

    while (--delay_val && !packetSize) {
        packetSize = lora_parse_packet();
    }

    if (packetSize) {
        uint8_t i = 0;
        while (lora_available()) {
            rxBuffer[i] = (char)lora_read();
            i++;
        }
    }

    if (!strcmp(rxBuffer, ackMsg)) return 0;
    else return 1;
}

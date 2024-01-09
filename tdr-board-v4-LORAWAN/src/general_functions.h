/*****************************************************************************
 * tdr-board-v4.h
 *****************************************************************************/

#ifndef __TDR_BOARD_V4_H__
#define __TDR_BOARD_V4_H__
#include "structs.h"
#include "tdr-board-v4_config.h"
#include <inttypes.h>
#include <adi_pwr.h>
/* Add your custom header content here */

void init_system();
void init_store();

void run_and_store_measurements(struct tdr_data integrator_vals_memory[], uint16_t* index);
void print_tdr_data_to_uart(struct tdr_data integrator_vals_memory[]);
uint8_t enter_hibernation();

void send_data_package(struct tdr_data integrator_vals_memory[], uint32_t num_msg);

void delay(uint32_t time);
uint32_t getClkFreq(const ADI_CLOCK_ID eClockId);

#endif /* __TDR_BOARD_V4_H__ */

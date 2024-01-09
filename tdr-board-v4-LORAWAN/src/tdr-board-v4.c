/*****************************************************************************
 * tdr-board-v4.c
 *****************************************************************************/

#include "tdr-board-v4_config.h"

#include "general_functions.h"
#include "rtc.h"
#include "structs.h"
#include "shared.h"
#include "uart-au.h"
#include "xint.h"


// LED current = 0.1755mA + 0.3424mA = 0.5179mA
// BOARD sleep = 0.72mA


//volatile uint32_t iHibernateExitFlag = 0;
//volatile uint8_t print_flag = 0;

/*
 * The system performs measurements every SLEEP_TIME (rtc.h) seconds and prints the measurement via UART - this is used to work with the external logging solution,
 * for the reasons stated by me during the technical meeting. Basically, anything with uart and an SD card will do just fine, or something on computer (terminal or something\
 * smarter, like datalogger). Also, during the sleep one can plug in the uart converter and send a byte - that would also result in tdr data being sent back.
 */

//int main(int argc, char *argv[])
//{
//    init_system();
//	struct tdr_data tdr_data[TDR_MEMORY_SIZE];
//	uint16_t index = 0;
//	uint16_t package_counter = 5;
//
//	packageNumber = 0;
//
//	while(1)
//	{
// 		if(print_flag)
//		{
// 			xint_uart_disable();
//			rtc_enable(false);
//			uart_init();
//		    uint32_t delay_val = 1600; // 20ms
//		    while(--delay_val){};
//			print_flag = 0;
//			print_tdr_data_to_uart(tdr_data);
//			uart_deinit();
//			iHibernateExitFlag = 0;
//			rtc_enable(true);
//			xint_uart_enable();
//			enter_hibernation();
//		}
//		else
//		{
// 			xint_uart_disable();
//			init_store();
//			run_and_store_measurements(tdr_data, &index);
//			uart_init();
//		    uint32_t delay_val = 1600; // 20ms
//		    while(--delay_val){};
//			print_tdr_data_to_uart(tdr_data);
//			uart_deinit();
//			send_data_package(tdr_data, package_counter);
//			packageNumber++;
//			iHibernateExitFlag = 0;
//			rtc_UpdateAlarm();
//			xint_uart_enable();
//			enter_hibernation();
//		}
//	}
//	return 0;
//}


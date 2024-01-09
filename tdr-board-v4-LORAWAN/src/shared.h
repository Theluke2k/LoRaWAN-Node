/*
 * shared.h
 *
 *  Created on: 26. jun. 2023
 *      Author: au703540
 */

#ifndef SHARED_H_
#define SHARED_H_

/*
 * Those variables are shared in different parts of the project and are usually modified in the interrupts, ergo volatile. Defined in tdr-board-v4.c.
 */

/*
 * Variable used to escape the hibernation mode when a proper interrupt occurs.
 */
extern volatile uint32_t iHibernateExitFlag;
/*
 * If set the system will transmit the information about tdr_data over UART.
 */
extern volatile uint8_t print_flag;

#endif /* SHARED_H_ */

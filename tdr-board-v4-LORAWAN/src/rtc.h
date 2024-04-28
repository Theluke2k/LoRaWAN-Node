/*
 * rtc.h
 *
 *  Created on: 1. jun. 2022
 *      Author: au703540
 */

#ifndef RTC_H_
#define RTC_H_
#include <inttypes.h>
#include <adi_rtc.h>
#include <adi_pwr.h>

/*
 * RTC-0 or RTC-1 can be selected.
 */
#define RTC_DEVICE_NUM    			0

/* leap-year compute macro (ignores leap-seconds) */
#define LEAP_YEAR(x) (((0==x%4)&&(0!=x%100))||(0==x%400))

/*
 * Amount of time the uC should sleep (in seconds).
 */
#define SLEEP_TIME						10

/* If the RTC needs to be calibrated -> uncomment the next line and add a calibration function to the init. */
//#define ADI_RTC_CALIBRATE


/* Trim interval */
#define ADI_RTC_TRIM_INTERVAL    	ADI_RTC_TRIM_INTERVAL_14
/* Trim operation +/- */
#define ADI_RTC_TRIM_DIRECTION   	ADI_RTC_TRIM_SUB
/* Trim  value */
#define ADI_RTC_TRIM_VALUE       	ADI_RTC_TRIM_1


uint32_t BuildSeconds(void);
ADI_RTC_RESULT rtc_Init (void);
ADI_RTC_RESULT rtc_Calibrate (void);
ADI_RTC_RESULT rtc_UpdateAlarm (void);
void rtc_enable(bool flag);
void set_iHibernateExitFlag(uint32_t x);
uint32_t get_iHibernateExitFlag();

#endif /* RTC_H_ */

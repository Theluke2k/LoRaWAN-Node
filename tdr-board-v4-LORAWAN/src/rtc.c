/*
 * rtc.c
 *
 *  Created on: 1. jun. 2022
 *      Author: au703540
 */
#include "rtc.h"

#include "tdr-board-v4_config.h"
#include "shared.h"

#include <adi_rtc.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include "uart-au.h"

/*
 * RTC memory (ADI driver).
 */
static uint8_t aRtcDevMem1[ADI_RTC_MEMORY_SIZE];

/* Device handle for RTC device-0 which act as wake up timer*/
static ADI_RTC_HANDLE hDevice1;

/*
 * @brief Callback function for the RTC.
 *
 * Sets the variable being used to wake up from the hibernation. Don't need to care about the parameters for this one. This var is available via shared.h
 * as an extern, defined in the main source file.
 */
static void rtc1Callback(void* hWut, uint32_t nEvent, void* pArg)
{
	iHibernateExitFlag = 1;
}

/*
 * @brief Enabling/Disabling the RTC.
 *
 * If disabled, the RTC will pause but preserve the counter register. So if resumed the counting will start from where it left off.
 *
 * @param[in] flag True -> ON, False -> OFF.
 */
void rtc_enable(bool flag)
{
	adi_rtc_Enable(hDevice1, flag);
}

/*
 * Lucas: MOVED TO "rtc-board.c".
 * @brief Function to obtain the data and time when the program is built.
 *
 * This is used to set up the initial value for the RTC register.
 *
 * @return Seconds from 01/01/1970 to the most recent build time.
 */
//uint32_t BuildSeconds(void)
//{
//
//    char timestamp[] = __DATE__ " " __TIME__;
//    int month_days [] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
//    uint32_t days, month = 1u, date, year, hours, minutes, seconds;
//    char Month[4];
//
//    /* parse the build timestamp */
//    sscanf(timestamp, "%s %d %d %d:%d:%d", Month, (int *)&date,(int *)&year, (int *)&hours, (int *)&minutes, (int *)&seconds);
//
//    /* parse ASCII month to a value */
//    if     ( !strncmp(Month, "Jan", 3 )) month = 1;
//    else if( !strncmp(Month, "Feb", 3 )) month = 2;
//    else if( !strncmp(Month, "Mar", 3 )) month = 3;
//    else if( !strncmp(Month, "Apr", 3 )) month = 4;
//    else if( !strncmp(Month, "May", 3 )) month = 5;
//    else if( !strncmp(Month, "Jun", 3 )) month = 6;
//    else if( !strncmp(Month, "Jul", 3 )) month = 7;
//    else if( !strncmp(Month, "Aug", 3 )) month = 8;
//    else if( !strncmp(Month, "Sep", 3 )) month = 9;
//    else if( !strncmp(Month, "Oct", 3 )) month = 10;
//    else if( !strncmp(Month, "Nov", 3 )) month = 11;
//    else if( !strncmp(Month, "Dec", 3 )) month = 12;
//
//    /* count days from prior years */
//    days=0;
//    for (int y=1970; y<year; y++) {
//        days += 365;
//        if (LEAP_YEAR(y))
//            days += 1;
//    }
//
//    /* add days for current year */
//    for (int m=1; m<month; m++)
//        days += month_days[m-1];
//
//    /* adjust if current year is a leap year */
//    if ( (LEAP_YEAR(year) && ( (month > 2) || ((month == 2) && (date == 29)) ) ) )
//        days += 1;
//
//    /* add days this month (not including current day) */
//    days += date-1;
//
//    return (days*24*60*60 + hours*60*60 + minutes*60 + seconds);
//}

/*
 * @brief RTC init function.
 *
 * RTC-0 is selected for this application, because it can wake up the uC in deep hibernation mode. RTC-1 has more options but we don't really need them and there
 * are some quirks with using it so it was not chosen. RTC initialized in such a way that the counter register changes every 1s for easier usage (setting the alarm is
 * just adding number of seconds to the value from the register).
 *
 * The trim is set to default values (refer to the reference manual). Also, the alarm is enabled.
 */
ADI_RTC_RESULT rtc_Init (void) {

    uint32_t buildTime = BuildSeconds();
    ADI_RTC_RESULT eResult;

    /* Use both static configuration and dynamic configuration for illustrative purpsoes */
    do
    {
        eResult = adi_rtc_Open(RTC_DEVICE_NUM, aRtcDevMem1, ADI_RTC_MEMORY_SIZE, &hDevice1);
        DEBUG_RESULT("\n Failed to open the device %04d",eResult,ADI_RTC_SUCCESS);

        eResult = adi_rtc_EnableInterrupts(hDevice1, ADI_RTC_ALARM_INT, true);
        DEBUG_RESULT("\n Failed to enable interrupts",eResult,ADI_RTC_SUCCESS);
        /*
        //Why this?
        eResult = adi_rtc_SetPreScale(hDevice1, 15);
        DEBUG_RESULT("\n Failed to set prescale to 15 (32768 ticks per 1 change of rtc reg)",eResult,ADI_RTC_SUCCESS);
	    */
        eResult = adi_rtc_RegisterCallback(hDevice1, rtc1Callback, hDevice1);
        DEBUG_RESULT("\n Failed to register callback",eResult,ADI_RTC_SUCCESS);

//        eResult = adi_rtc_SetCount(hDevice1, buildTime);
//        DEBUG_RESULT("Failed to set the count", eResult, ADI_RTC_SUCCESS);

        eResult = adi_rtc_SetTrim(hDevice1, ADI_RTC_TRIM_INTERVAL, ADI_RTC_TRIM_VALUE, ADI_RTC_TRIM_DIRECTION);
        DEBUG_RESULT("Failed to set the trim value",eResult,ADI_RTC_SUCCESS);

    /* force a reset to the latest build timestamp */
        DEBUG_MESSAGE("Resetting clock");
        eResult = adi_rtc_SetCount(hDevice1, buildTime);
        DEBUG_RESULT("Failed to set count",eResult,ADI_RTC_SUCCESS);

//        DEBUG_MESSAGE("New time is:");
//        rtc_ReportTime();

        eResult = adi_rtc_Enable(hDevice1, true);
        DEBUG_RESULT("Failed to enable the device",eResult,ADI_RTC_SUCCESS);

        if(ADI_RTC_SUCCESS != (eResult = adi_rtc_EnableAlarm(hDevice1, true)))
    	{
            DEBUG_RESULT("adi_RTC_EnableAlarm failed",eResult,ADI_RTC_SUCCESS);
    	}

    } while(0);

    return(eResult);
}


/*
 * @brief RTC calibration function.
 *
 * This is not used, yet can be useful. Nicely described below.
 */
ADI_RTC_RESULT rtc_Calibrate (void)
{
    ADI_RTC_RESULT eResult= ADI_RTC_FAILURE;
#ifdef ADI_RTC_CALIBRATE

    /*

    Compute the LF crystal trim values to compensate the RTC.  This can
    come from a static measure (a frequency counter), a real-time drift measure
    based external reference.

    Commercial crystals typically run between 20-100 ppm.  As an exercise, we
    demonstrate trimming a particular crystal and board configuration in which
    we measure an untrimmed error of about +58.6ppm (0.00586%).  This corresponds
    to a raw clock about 35.5 seconds/week fast (30 minutes/year).

    Available Trim Corrections:
    	X axis: trim interval (seconds)
    	Y axis: trim value (seconds)
    	Table content: trim correction (ppm)
      Value     16384    32768    65536   131072 (Interval)
        0        0.00     0.00     0.00     0.00
        1       61.04    30.52    15.26     7.63
        2      122.07    61.04    30.52    15.26
        3      183.11    91.55    45.78    22.89
        4      244.14   122.07    61.04    30.52
        5      305.18   152.59    76.29    38.15
        6      366.21   183.11    91.55    45.78
        7      427.25   213.62   106.81    53.41

    Referencing the trim table, we see the closest matching ppm correction for
    our example is 61.04.  In case there are different combinations yielding
    the same desired correction, we prefer the shortest trim interval (and
    smallest trim value) so as to minimize instantaneous drift.

    So we choose a trim interval of 2^14 seconds with a negative trim value of 1
    second, subtracting 1 second every 4.5 hours to "slow" the fast crystal down
    to a more reasonable rate.  This particular trim leaves a residual error of
    negative 2.44ppm (0.000244%), making the trimmed clock a tad slow (less than
    1.5 seconds/week or about 1.3 minutes/year), but much better than the
    untrimmed accuracy of 30 minutes/year.

    */

    ADI_PWR_RESULT ePowResult;
    if( ADI_PWR_SUCCESS != (ePowResult = adi_pwr_SetLFClockMux(ADI_CLOCK_MUX_LFCLK_LFXTAL)))   /*  select LFXTL clock */
    {
        DEBUG_RESULT("\n Failed to set the LF Clock Mux", ePowResult, ADI_PWR_SUCCESS);
        return(ADI_RTC_FAILURE);
    }


    if(ADI_RTC_SUCCESS != (eResult = adi_rtc_SetTrim(hDevice1,ADI_RTC_TRIM_INTERVAL_14,ADI_RTC_TRIM_1,ADI_RTC_TRIM_SUB)))
    {
        DEBUG_RESULT("\n Failed to set the device %04d", eResult, ADI_RTC_SUCCESS);
        return(eResult);
    }
    if(ADI_RTC_SUCCESS != (eResult = adi_rtc_EnableTrim(hDevice1, true)))
    {
        DEBUG_RESULT("\n Failed to enable the trim %04d", eResult, ADI_RTC_SUCCESS);
        return(eResult);
    }

#else
    printf("Use \"ADI_RTC_CALIBRATE\" preprocessor macro for RTC calibration.");
#endif
    return(eResult);
}

/*
 * @brief RTC alarm setup.
 *
 * As there is only one alarm being set, to ring in some fixed interval from one measurement phase to the other, the SLEEP_TIME macro is defined in rtc.h. This should
 * be called before entering hibernation to ensure the bloody thing will wake up when needed.
 */
ADI_RTC_RESULT rtc_UpdateAlarm (void) {
    ADI_RTC_RESULT eResult;
    uint32_t rtcCount;

    if(ADI_RTC_SUCCESS != (eResult = adi_rtc_GetCount(hDevice1,&rtcCount)))
    {
        DEBUG_RESULT("\n Failed to get RTC Count %04d",eResult,ADI_RTC_SUCCESS);
        return(eResult);
    }
    if(ADI_RTC_SUCCESS != (eResult = adi_rtc_SetAlarm(hDevice1, rtcCount + SLEEP_TIME)))
    {
        DEBUG_RESULT("\n Failed to set RTC Alarm %04d",eResult,ADI_RTC_SUCCESS);
        return(eResult);
    }

//    char buffer[128];
//    DEBUG_MESSAGE("UPDATE >>>>>");
//	adi_rtc_GetCount(hDevice1, &rtcCount);
//	sprintf(buffer, "RAW RTC time: %lu", rtcCount);
//	DEBUG_MESSAGE(buffer);
//	adi_rtc_GetAlarm(hDevice1, &rtcCount);
//	sprintf(buffer, "RAW ALARM time: %lu", rtcCount);
//	DEBUG_MESSAGE(buffer);
//    DEBUG_MESSAGE("UPDATE <<<<<");

    return(eResult);
}



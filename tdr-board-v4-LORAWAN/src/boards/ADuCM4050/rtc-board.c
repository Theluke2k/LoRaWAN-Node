/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#include "board-config.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "rtc-board.h"
#include "lpm-board.h"
#include <adi_rtc.h>

#define RTC_DEBUG_ENABLE                            1
#define RTC_DEBUG_DISABLE                           0

#define RTC_DEBUG_GPIO_STATE                        RTC_DEBUG_DISABLE
#define RTC_DEBUG_PRINTF_STATE                      RTC_DEBUG_DISABLE


/*
 * Lucas:
 * MCU wake-up time (in ticks) calculated from the worst case hibernation wake-up time.
 * Page 31 in ADuCM4050 datasheet top of the page.
 */
#define MIN_ALARM_DELAY                             25

/*
 * Lucas:
 * RTC-0 or RTC-1 can be selected. Only RTC0 is available in shutdown mode.
 */
#define RTC_DEVICE_NUM    			1

/*
 * AU guys:
 * Leap-year compute macro (ignores leap-seconds)
 */
#define LEAP_YEAR(x) (((0==x%4)&&(0!=x%100))||(0==x%400))



/*
 * Lucas:
 * - Trim interval
 * - Trim operation
 * - Trim value
 * Found from the RTC example.
 */
#define ADI_RTC_TRIM_INTERVAL    	ADI_RTC_TRIM_INTERVAL_14
#define ADI_RTC_TRIM_DIRECTION   	ADI_RTC_TRIM_SUB
#define ADI_RTC_TRIM_VALUE       	ADI_RTC_TRIM_1

#define PRESCALAR					0
#define RTC_COUNTER_FREQ			(uint32_t)(1 << (15-PRESCALAR))



//RTC memory (ADI driver).
static uint8_t rtc0Mem[ADI_RTC_MEMORY_SIZE]; // For RTC0 (if used)
static uint8_t rtc1Mem[ADI_RTC_MEMORY_SIZE];// For RTC1

//Device handle for RTC device-0
static ADI_RTC_HANDLE hDevRtc0 = NULL;
static ADI_RTC_HANDLE hDevRtc1 = NULL;

//Alarm callback
static void           rtc1Callback (void *pCBParam, uint32_t Event, void *EventArg);

/*
 * Variable used to track the last time an alarm was set.
 */
static uint32_t RtcTimerContext = 0;

/*
 * Lucas:
 * Callback function for the alarm that is set.
 */
static void rtc1Callback(void *pCBParam, uint32_t Event, void *EventArg)
{
	//adi_gpio_Toggle(ADI_GPIO_PORT1, ADI_GPIO_PIN_15); // DEBUG
	TimerIrqHandler();
}

/*
 * Lucas:
 * Built in ADI function, from the RTC example.
 *
 * @brief Function to obtain the data and time when the program is built.
 *
 * This is used to set up the initial value for the RTC register.
 *
 * @return Seconds from 01/01/1970 to the most recent build time.
 */
uint32_t BuildSeconds(void)
{

    char timestamp[] = __DATE__ " " __TIME__;
    int month_days [] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint32_t days, month = 1u, date, year, hours, minutes, seconds;
    char Month[4];

    /* parse the build timestamp */
    sscanf(timestamp, "%s %d %d %d:%d:%d", Month, (int *)&date,(int *)&year, (int *)&hours, (int *)&minutes, (int *)&seconds);

    /* parse ASCII month to a value */
    if     ( !strncmp(Month, "Jan", 3 )) month = 1;
    else if( !strncmp(Month, "Feb", 3 )) month = 2;
    else if( !strncmp(Month, "Mar", 3 )) month = 3;
    else if( !strncmp(Month, "Apr", 3 )) month = 4;
    else if( !strncmp(Month, "May", 3 )) month = 5;
    else if( !strncmp(Month, "Jun", 3 )) month = 6;
    else if( !strncmp(Month, "Jul", 3 )) month = 7;
    else if( !strncmp(Month, "Aug", 3 )) month = 8;
    else if( !strncmp(Month, "Sep", 3 )) month = 9;
    else if( !strncmp(Month, "Oct", 3 )) month = 10;
    else if( !strncmp(Month, "Nov", 3 )) month = 11;
    else if( !strncmp(Month, "Dec", 3 )) month = 12;

    /* count days from prior years */
    days=0;
    for (int y=1970; y<year; y++) {
        days += 365;
        if (LEAP_YEAR(y))
            days += 1;
    }

    /* add days for current year */
    for (int m=1; m<month; m++)
        days += month_days[m-1];

    /* adjust if current year is a leap year */
    if ( (LEAP_YEAR(year) && ( (month > 2) || ((month == 2) && (date == 29)) ) ) )
        days += 1;

    /* add days this month (not including current day) */
    days += date-1;

    return (days*24*60*60 + hours*60*60 + minutes*60 + seconds);
}

/*  standard ctime (time.h) constructs */
static void rtc_ReportTime(void) {

    char buffer[128];


    time_t rawtime;

    /* get the RTC count through the "time" CRTL function */
    time(&rawtime);


    /* print raw count */
    sprintf (buffer, "    Raw time: %d", (int)rawtime);
    DEBUG_MESSAGE(buffer);

    /* convert to UTC string and print that too */
    sprintf (buffer, "    UTC time: %s", ctime(&rawtime));
    DEBUG_MESSAGE(buffer);

}


/*
 * Lucas:
 * Updated to use the ADI built in function.
 *
 * We use RTC with a prescale of 0, thus every tick corresponds to 30,52 microseconds.
 *
 * When an alarm executes, it triggers an interrupt that runs the callback function
 * "rtc1Callback". This function calls the LoRaMac timer interrupt handler so we don't need to do
 * anything else with that matter.
 *
 * The commented code in the bottom is a test program for the counter and alarm callback function.
 */
void RtcInit( void )
{
	uint32_t buildTime = BuildSeconds();
	ADI_RTC_RESULT eResult;
	uint32_t count;

	eResult = adi_rtc_Open(RTC_DEVICE_NUM, rtc1Mem, ADI_RTC_MEMORY_SIZE, &hDevRtc1);
	DEBUG_RESULT("Failed to open the device",eResult,ADI_RTC_SUCCESS);

	eResult = adi_rtc_RegisterCallback(hDevRtc1, rtc1Callback, hDevRtc1);
	DEBUG_RESULT("Failed to register callback %04d",eResult,ADI_RTC_SUCCESS);

	//eResult = adi_rtc_SetCount(hDevRtc1, buildTime);
	//DEBUG_RESULT("Failed to set the count", eResult, ADI_RTC_SUCCESS);

	//VIRKER KUN FOR RTC1
    eResult = adi_rtc_SetPreScale(hDevRtc1, PRESCALAR);
    DEBUG_RESULT("\n Failed to set prescale to 0",eResult,ADI_RTC_SUCCESS);

    eResult = adi_rtc_SetTrim(hDevRtc1, ADI_RTC_TRIM_INTERVAL_6, ADI_RTC_TRIM_4, ADI_RTC_TRIM_ADD);
	DEBUG_RESULT("Failed to set Trim value  ", eResult, ADI_RTC_SUCCESS);

	/*
	 * Lucas:
	 * DONT ENABLE TRIM
	 * This makes the timer 2 seconds too fast per 30 seconds! Trust me i tested it...
	 */
	//eResult = adi_rtc_EnableTrim(hDevRtc1, true);
	//DEBUG_RESULT("\n Failed to enable Trim ", eResult, ADI_RTC_SUCCESS);

	/* force a reset to the latest build timestamp */
	//DEBUG_MESSAGE("Resetting clock to latest build time...");
	//eResult = adi_rtc_SetCount(hDevRtc1, buildTime);
	//DEBUG_RESULT("Failed to set count",eResult,ADI_RTC_SUCCESS);

	DEBUG_MESSAGE("New time is:");
	rtc_ReportTime();

	eResult = adi_rtc_Enable(hDevRtc1, true);
	DEBUG_RESULT("Failed to enable the device", eResult, ADI_RTC_SUCCESS);



/*
	//Program to test timer.
	RtcSetTimerContext( );
	RtcSetAlarm( 327680/2 );
	while(iFlag == 0) {

	}
	printf("Timer executed interrupt!\n");
*/
	/*
	 * Lucas:
	 * PROGRAM TO TEST COUNTER
	 */
	/*
	eResult = adi_rtc_EnableAlarm(hDevRtc1, false);
	DEBUG_RESULT("adi_RTC_EnableAlarm failed",eResult,ADI_RTC_SUCCESS);

	eResult = adi_rtc_EnableInterrupts(hDevRtc1, ADI_RTC_ALARM_INT, false);
	DEBUG_RESULT("adi_RTC_EnableInterrupts failed",eResult,ADI_RTC_SUCCESS);

	if (ADI_RTC_SUCCESS != (eResult = adi_rtc_GetCount(hDevRtc1, &count)))
	{
		DEBUG_RESULT("\n Failed to get RTC Count %04d", eResult, ADI_RTC_SUCCESS);
	}
	printf("Count: %d\n", count);

	uint32_t rtcCounts[20];
	uint32_t loopTimes[20];
	uint32_t oldCount = count;
	uint32_t loopTime = 0;
	for(int i = 0; i < 10; i++) {
		while(count == oldCount) {
			if (ADI_RTC_SUCCESS != (eResult = adi_rtc_GetCount(hDevRtc1, &count)))
			{
				DEBUG_RESULT("\n Failed to get RTC Count %04d", eResult, ADI_RTC_SUCCESS);
			}
			loopTime++;
		}
		rtcCounts[i] = count;
		loopTimes[i] = loopTime;
		loopTime = 0;
		oldCount = count;
	}

	for (int i = 0; i < 10; i++) {
		printf("Count: %d\n", rtcCounts[i]);
		printf("LoopTime: %d\n", loopTimes[i]);
	}*/

	/*
	 * Lucas:
	 * PROGRAM TO TEST ALARM
	 */

	/*
	uint32_t counter;

	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_GetCount(hDevRtc1,&counter)))
	{
	    DEBUG_RESULT("\n Failed to get RTC Count %04d",eResult,ADI_RTC_SUCCESS);
	}
	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_SetAlarm(hDevRtc1, counter + 163840)))
	{
	    DEBUG_RESULT("\n Failed to set RTC Alarm %04d",eResult,ADI_RTC_SUCCESS);
	}



	iHibernateExitFlag = 0;
	if (ADI_RTC_SUCCESS != (eResult = adi_rtc_GetCount(hDevRtc1, &count)))
	{
		DEBUG_RESULT("\n Failed to get RTC Count %04d", eResult, ADI_RTC_SUCCESS);
	}
	printf("Count: %d\n", count);
	if (adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_HIBERNATE, &iHibernateExitFlag, 0))
	{
	    DEBUG_MESSAGE("System Entering to Low Power Mode failed");
	}
	printf("Timer Callback Called!\n");
	if (ADI_RTC_SUCCESS != (eResult = adi_rtc_GetCount(hDevRtc1, &count)))
	{
		DEBUG_RESULT("\n Failed to get RTC Count %04d", eResult, ADI_RTC_SUCCESS);
	}
	printf("Count: %d\n", count);

	while(testerInt != 1) {
		printf("%d\n", testerInt);
	}
	*/
}

/*
 * Lucas:
 * This function saves the current tick of the counter register so it can
 * be used as reference e.g. if we want to get the elapsed time of the timer. OLD
 *//*
uint32_t RtcSetTimerContext( void )
{
	ADI_RTC_RESULT eResult;
	eResult = adi_rtc_GetCount(hDevRtc1, &RtcTimerContext);
	DEBUG_RESULT("\n Failed to get RTC Count %04d",eResult,ADI_RTC_SUCCESS);
	//printf("RtcSetTimerContext returned: %d\n", RtcTimerContext);
	return ( uint32_t )RtcTimerContext;
}*/

/*
 * Lucas:
 * This function saves the current tick of the counter register so it can
 * be used as reference e.g. if we want to get the elapsed time of the timer.
 */
uint32_t RtcSetTimerContext( void )
{
	RtcTimerContext = RtcGetTimerValue();
	return ( uint32_t )RtcTimerContext;
}

/*
 * Lucas:
 * Just return the timer context value. This is just a getter.
 */
uint32_t RtcGetTimerContext( void )
{
	return ( uint32_t )RtcTimerContext;
}

/*
 * Lucas:
 * Returns the worst case wake-up time delay of the ADuCM4050 when it wakes up from
 * hibernation mode.
 */
uint32_t RtcGetMinimumTimeout( void )
{
	return( MIN_ALARM_DELAY );
}

/*
 * Lucas:
 * Our RTC runs at 32,768 kHz so each tick is 1/f = 30,518 us.
 * Milliseconds is 1000 times smaller than seconds so the scale between ticks and
 * milliseconds becomes 30,518 us * 1000 = 0.030518. (1 tick = 0.030518 ms).
 *
 * Be aware that the equation will round down to nearest integer, so in the worst
 * case we can lose 1 tick which is not a lot. We will get a maximum deviation of
 * 30,52 us.
 */
uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
	//printf("RtcMs2Tick returned: %d\n", (int) ((milliseconds * RTC_COUNTER_FREQ) / 1000.0f));
	return (uint32_t) (milliseconds * (RTC_COUNTER_FREQ / 1000.0f));
}

TimerTime_t RtcTick2Ms( uint32_t tick )
{
	//printf("RtcTick2Ms returned: %d\n", (int) ((tick * 1000.0f) / RTC_COUNTER_FREQ));
	return (uint32_t) (tick * (1000.0f / RTC_COUNTER_FREQ));
}

/*
 * Lucas:
 * This creates a delay using the RTC. __NOP( ) is "no operation" in assembly. The
 * processor will be idle while waiting.
 * (will interrupts work?)
 */
void RtcDelayMs( uint32_t delay )
{
	uint32_t delayTicks = 0;
	uint32_t refTicks = RtcGetTimerValue( );

	delayTicks = RtcMs2Tick( delay );

	// Wait delay ms
	while( ( ( RtcGetTimerValue( ) - refTicks ) ) < delayTicks )
	{
	    __NOP( );
	}
}

/*
 * Lucas: (UNFINISHED)
 * I currently don't know what to do with the Lpm functions. I think the reason for the logic
 * is to disable low power mode if a timer is below the wake up time. This ensures that the
 * mcu does not delay the execution of the timer because it went into low power mode right
 * before an alarm execution.
 *
 * We should check out what the Lpm functions do but right now we just hope it wont enter
 * lpm before an alarm executes or just live with the consequences.
 */
void RtcSetAlarm( uint32_t timeout )
{
	// We don't go in Low Power mode for timeout below MIN_ALARM_DELAY
	if( ( int64_t )MIN_ALARM_DELAY < ( int64_t )( timeout - RtcGetTimerElapsedTime( ) ) )
	{
	    //LpmSetStopMode( LPM_RTC_ID, LPM_ENABLE );
	}
	else
	{
	    //LpmSetStopMode( LPM_RTC_ID, LPM_DISABLE );
	}
	RtcStartAlarm( timeout );
}

/*
 * Lucas:
 * Stops the alarm.
 * (Will this also stop interrupts? Maybe interrupts need to be initialized after this?)
 */
void RtcStopAlarm( void )
{
	ADI_RTC_RESULT eResult;

	eResult = adi_rtc_EnableAlarm(hDevRtc1, false);
	DEBUG_RESULT("\n Failed to disable the device",eResult,ADI_RTC_SUCCESS);
}

/*
 * Lucas:
 * Starts the timer for a time specified by timeout (in ticks). OLD
 */
void RtcStartAlarm( uint32_t timeout )
{
	//printf("RtcStartAlarm called with timout: %d\n", (int) timeout);
	//printf("Real value of alarm is: %d\n", (int) (RtcTimerContext + timeout));
	ADI_RTC_RESULT eResult;

	RtcStopAlarm( );

	eResult = adi_rtc_SetAlarm(hDevRtc1, RtcTimerContext + timeout);
	DEBUG_RESULT("\n Failed to set alarm",eResult,ADI_RTC_SUCCESS);

	eResult = adi_rtc_EnableAlarm(hDevRtc1,true);
	DEBUG_RESULT("Failed to enable alarm",eResult,ADI_RTC_SUCCESS);

	eResult = adi_rtc_EnableInterrupts(hDevRtc1, ADI_RTC_ALARM_INT, true);
	DEBUG_RESULT("Failed to enable interrupts",eResult,ADI_RTC_SUCCESS);

}

/*
 * Lucas:
 * Starts the timer for a time specified by timeout (in ticks).
 *//*
void RtcStartAlarm( uint32_t timeout )
{
	//printf("RtcStartAlarm called with timout: %d\n", (int) timeout);
	//printf("Real value of alarm is: %d\n", (int) (RtcTimerContext + timeout));
	ADI_RTC_RESULT eResult;

	RtcStopAlarm( );

	eResult = adi_rtc_SetAlarmRegs(hDevRtc1, (uint16_t) ((RtcTimerContext + timeout) >> 15), 0, (uint16_t) ((RtcTimerContext + timeout) & 0x7FFF));
	DEBUG_RESULT("\n Failed to set alarm",eResult,ADI_RTC_SUCCESS);

	eResult = adi_rtc_EnableAlarm(hDevRtc1,true);
	DEBUG_RESULT("Failed to enable alarm",eResult,ADI_RTC_SUCCESS);

	eResult = adi_rtc_EnableInterrupts(hDevRtc1, ADI_RTC_ALARM_INT, true);
	DEBUG_RESULT("Failed to enable interrupts",eResult,ADI_RTC_SUCCESS);

}*/

/*
 * Lucas:
 * Gets the current ticks of the RTC counter. OLD
 */
uint32_t RtcGetTimerValue( void )
{
	ADI_RTC_RESULT eResult;
	uint32_t count = 0;

	eResult = adi_rtc_GetCount(hDevRtc1,&count);
	DEBUG_RESULT("\n Failed to get RTC Count %04d",eResult,ADI_RTC_SUCCESS);

	return count;
}

/*
 * Lucas:
 * Gets the current ticks of the RTC counter.
 *//*
uint32_t RtcGetTimerValue( void )
{
	ADI_RTC_RESULT eResult;
	uint32_t pnCount = 0;
	uint32_t pfCount = 0;

	eResult = adi_rtc_GetCountRegs(hDevRtc1, &pnCount, &pfCount);
	DEBUG_RESULT("\n Failed to get RTC Count %04d",eResult,ADI_RTC_SUCCESS);

	uint64_t totalCount = (pnCount << 15) | (pfCount);

	return (uint32_t) totalCount;
}*/

/*
 * Lucas:
 * Gets the elapsed time of the current timer. RtcTimerContext is the time of
 * start of the ongoing timer and RtcGetTimerValue() is the current time (in ticks).
 * Subtracting these gets the elapsed time.
 */
uint32_t RtcGetTimerElapsedTime( void )
{
	//printf("RtcGetTimerElapsedTime returned: %d\n", (int) (RtcGetTimerValue() - RtcTimerContext));
	return( (uint32_t) (RtcGetTimerValue() - RtcTimerContext));
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
	return 0;
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{

}

void RtcBkupRead( uint32_t* data0, uint32_t* data1 )
{

}

void RtcProcess( void )
{
	// Not used on this platform.
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    return 0;
}


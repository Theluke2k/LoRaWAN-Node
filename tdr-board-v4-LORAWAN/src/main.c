/*
 * main.c
 *
 *  Created on: 29. aug. 2023
 *      Author: lselm
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "firmwareVersion.h"
#include "apps/LoRaMac/common/githubVersion.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "RegionCommon.h"

#include "cli.h"
#include "Commissioning.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
//#include "CayenneLpp.h"
#include "LmHandlerMsgDisplay.h"

#include "radio.h"

#include "tdr-board-v4_config.h"
#include "general_functions.h"
#include "rtc.h"
#include "structs.h"
#include "shared.h"
#include "uart-au.h"
#include "xint.h"
#include "spi-au.h"
#include <adi_gpio.h>
#include "spi.h"

// DEBUG
uint8_t tester = 0; //

#define ACTIVE_REGION LORAMAC_REGION_EU868

//LoRaMac
#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#ifndef LORAWAN_DEFAULT_CLASS
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
#endif

/*!
 * Defines the application data transmission duty cycle. 10s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                           	1000 // Currently Unused

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        0

/*!
 * LoRaWAN Adaptive Data Rate (Skal den være til eller fra?)
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0 // DEBUG (default: DR0)

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_CONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            100

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are reserved.
 */
#define LORAWAN_APP_PORT                            2

/*!
 *
 */
typedef enum {
	LORAMAC_HANDLER_TX_ON_TIMER, LORAMAC_HANDLER_TX_ON_EVENT,
} LmHandlerTxEvents_t;

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!preap
 * User application data structure
 */
static LmHandlerAppData_t AppData = { .Buffer = AppDataBuffer, .BufferSize = 0,
		.Port = 0, };


/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;
static TimerEvent_t SleepTimer;

static void OnMacProcessNotify(void);
static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size);
static void OnNetworkParametersChange(CommissioningParams_t* params);
static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq,
		TimerTime_t nextTxIn);
static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq,
		TimerTime_t nextTxIn);
static void OnJoinRequest(LmHandlerJoinParams_t* params);
static void OnTxData(LmHandlerTxParams_t* params);
static void OnRxData(LmHandlerAppData_t* appData, LmHandlerRxParams_t* params);
static void OnClassChange(DeviceClass_t deviceClass);
static void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t* params);
#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection);
#else
static void OnSysTimeUpdate( void );
#endif
static void PrepareTxFrame(void);
static void StartTxProcess(LmHandlerTxEvents_t txEvent);
static void UplinkProcess(void);

static void OnTxPeriodicityChanged(uint32_t periodicity);
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent(void* context);
static void OnSleepTimerEvent(void* context);

//Function pointers for loramac callbacks.
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = NULL,
    .GetRandomSeed = BoardGetRandomSeed,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams =
{
    .Region = ACTIVE_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = AppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};
static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = FIRMWARE_VERSION,
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};
/*!
 * Indicates if LoRaMacProcess call is pending.
 *
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

/*
 * Lucas (25-02-2024)
 * TDR data struct definition. Holds information about all of the measurements done in the measurement phase.
 */
struct tdr_data tdr_data[1];
volatile uint32_t iHibernateExitFlag = 0;
volatile uint8_t print_flag = 0;
uint8_t desiredUplinks = 0;
uint8_t uplinksSent = 0;
uint8_t initialized = 0;
uint32_t sleepTime = 10000;
int32_t sleepTimeOffset = 0;

// Logical Flags
uint8_t enableSleepFlag = 0;
uint8_t isJoiningFlag = 0;
uint8_t hasHibernated = 0;




// Function definitions
int32_t getSleepTimeOffset(uint32_t random_value, int32_t MIN, int32_t MAX);

/*
 * Lucas (22-10-23):
 * Main program.
 */
int main(void) {
 	uint16_t index = 0;

 	// Initializes the system clock and needed drivers.
 	init_system();

	// Create timer to wake up processor during join accept.
	TimerInit( &SleepTimer, OnSleepTimerEvent );

	while (1) {
		// Reinitialize system that were closed during hibernation
		/*
		if(hasHibernated) {
			reinit_system();
			hasHibernated = 0;
		}
		*/

		/*
		 * Lucas (30-03-2024):
		 * AU runs their measurements. The data is stored in tdr_data.
		 * The stack uses this struct as data source when transmitting data.
		 */
		// Initialize measure mode
		//InitMeasureMode();

		// Run measurements
  		init_store();
		run_and_store_measurements(tdr_data, &index);

		// Deinitialize measure mode
		//DeInitMeasureMode();


		// Specify the amount of desired uplinks before going to sleep.
		desiredUplinks = 1;

		/*
		 * Lucas (30-03-2024):
		 * Run the LoRaMac stack.
		 */
		// Initlialize board (sets up pins as LoRaMac wants it)
		BoardInitMcu();

		// Set interrup priorities. SPI must have highest prioriy!
		NVIC_SetPriority(SYS_GPIO_INTA_IRQn, 2);
		NVIC_SetPriority(SPI0_EVT_IRQn, 1);
		NVIC_SetPriority(RTC1_EVT_IRQn, 2);
		NVIC_SetPriority(RTC0_EVT_IRQn, 2);

		// Initialize transmission perhiodicity variable
		TxPeriodicity = APP_TX_DUTYCYCLE
				+ randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);

		const Version_t appVersion = { .Value = FIRMWARE_VERSION };
		const Version_t gitHubVersion = { .Value = GITHUB_VERSION };

		DisplayAppInfo("periodic-uplink-lpp", &appVersion, &gitHubVersion);

		if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams) != LORAMAC_HANDLER_SUCCESS) {
			printf("LoRaMac wasn't properly initialized\n");
			// Fatal error, endless loop.
			while (1) {
			}
		}

		if(!initialized) { // If the board is on first startup, run the following code:
			// Set system maximum tolerated rx error in milliseconds
			LmHandlerSetSystemMaxRxError(20);

			// The LoRa-Alliance Compliance protocol package should always be initialized and activated.
			LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams);

			// The join process can be made here but it does not need to run. The state machine handles it.
			LmHandlerJoin(); // DEBUG: should be deleted when eeprom is implemented

			// Mark the program as initiated.
			initialized = 1;
		}

		// Reset number of uplinks for this power cycle.
		uplinksSent = 0;

		do  {
			// Processes the LoRaMac events
			LmHandlerProcess();

			// Try to send uplink
			if (uplinksSent < desiredUplinks) {
				PrepareTxFrame();
			}

			CRITICAL_SECTION_BEGIN( );

			/*
			 * Lucas (28-04-2024):
			 * The following first checks if there is a MAC process pending. If there is
			 * the flag is cleared and the board does not send another frame or goes to sleep,
			 * allowing the state machine to run one more time to processes the MAC layer. If
			 * There is no MAC process pending, it checks if the LmHandler is busy. If it is not busy
			 * then we check if we have sent all of our frames. If we have not, we send the next frame.
			 * If we have sent all frames, the loop breaks and the board goes to sleep, until the next
			 * power cycle.
			 */
			if (IsMacProcessPending == 1) {
				IsMacProcessPending = 0;
			}
			else if ((LmHandlerIsBusy() == false))
			{
				if (uplinksSent >= desiredUplinks) {
					CRITICAL_SECTION_END( );
					break;
				}
			}
			else if(enableSleepFlag) {
				// End critical section to enable interrupts
				CRITICAL_SECTION_END( );

				// Set radio to sleep
				Radio.Write(0x01, 0x00);

				// Set sleep timer depending on if we are joining or not
				if (isJoiningFlag) {
					TimerSetValue(&SleepTimer, 4800);
				} else {
					TimerSetValue(&SleepTimer, 800);
				}

				// Clear flags
				enableSleepFlag = 0;
				isJoiningFlag = 0;
				iHibernateExitFlag = 0;

				// Start sleep timer
				TimerStart(&SleepTimer);

				// Enter hibernation mode
				enter_hibernation();

				// Reinitializez required systems after hibernate wakeup.
				//SystemReinitializerFromHibernate();
			}
			CRITICAL_SECTION_END( );

		} while (1);

		// Generate a random uint32_t using Radio.Random(). Map value to min -3000 and max 3000
		sleepTimeOffset = getSleepTimeOffset(Radio.Random(), -3000, 3000);

		// Deinitialize Loramac
		LmHandlerDeInit();

		// Set radio to sleep
		Radio.Write(0x01, 0x00);

		// Reset Sleep Flag
		iHibernateExitFlag = 0;

		// Calculate time offset of +- 3000 ms to avoid packet collisions
		TimerSetValue( &SleepTimer, sleepTime + sleepTimeOffset);

		// De-initialize system we don't need while hibernating
		//deinit_system();

		// Set Wakeup Alarm
		TimerStart(&SleepTimer);

		// Enter Hibernate Mode
		enter_hibernation();

		// Set flag to reinitialize systems
		//hasHibernated = 1;
	}

	return 0;
}

/*
 * Lucas:
 * The function maps a randomly generated uint32_t to an integer between -3000 and +3000.
 */
int32_t getSleepTimeOffset(uint32_t random_value, int32_t MIN, int32_t MAX) {
	// Normalize random value
	float norm = (float)random_value / (float)UINT32_MAX;
	int32_t offset = (int32_t)((norm * (MAX - MIN)) + MIN);
	return offset;
}

/*
 * Lucas:
 * Function to handle the CLI interface functions.
 */
bool CLIHandler(LmHandlerAppData_t* appData) {
	// Define expected CLI functions:
	if(appData->Buffer == NULL || appData->BufferSize == 0) {
		return false;
	}

	// Create new buffer to add null terminator
	char receiveBuffer[appData->BufferSize + 1]; // Add extra space for null terminator
	memcpy(receiveBuffer, appData->Buffer, appData->BufferSize); // Copy original array
	receiveBuffer[appData->BufferSize] = '\0'; // Add null terminator

	// Check the different commands
	if(strcmp(receiveBuffer, "123") == 0) {
		// Execute handler for command
		printf("ping\n");
		return true;
	}
	else if(strncmp(receiveBuffer, "Sleep", 5) == 0) {
			int time = 0;
			if(sscanf(receiveBuffer, "Sleep %d", &time) == 1) {
				// Execute handler for command
				printf("deep sleep: %d\n", time);
				sleepTime = time;
				return true;
			}
		}
	else if(strncmp(receiveBuffer, "{\"config\":{\"adr\":\"", 18) == 0) {
		bool setADR = 0;
		if(sscanf(receiveBuffer, "{\"config\":{\"adr\":\"%d", &setADR) == 1) {
			if(LmHandlerParams.AdrEnable != setADR) {
				// Execute handler for command
				LmHandlerParams.AdrEnable = setADR;

				if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams) != LORAMAC_HANDLER_SUCCESS) {
					printf("LoRaMac wasn't properly initialized\n");
					// Fatal error, endless loop.
					while (1) {}
				}
			}
			printf("setadr: %d\n", setADR);
			return true;
		}
	}

	else {
		printf("unknown command\n");
	}

	return false;
}

static void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
    DisplayNvmDataChange( state, size );
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
    DisplayNetworkParametersUpdate( params );
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
    DisplayMacMcpsRequestUpdate( status, mcpsReq, nextTxIn );
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
    DisplayMacMlmeRequestUpdate( status, mlmeReq, nextTxIn );
}

static void OnJoinRequest( LmHandlerJoinParams_t* params )
{
    DisplayJoinRequestUpdate( params );
    if( params->Status == LORAMAC_HANDLER_ERROR )
    {
        LmHandlerJoin( );
    }
    else
    {
        LmHandlerRequestClass( LORAWAN_DEFAULT_CLASS );
    }
}

static void OnTxData( LmHandlerTxParams_t* params )
{
    DisplayTxUpdate( params );
}

static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params )
{
    DisplayRxUpdate( appData, params );

    switch( appData->Port )
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case LORAWAN_APP_PORT:
        {
        	CLIHandler(appData);
        	//AppLedStateOn = appData->Buffer[0] & 0x01;
            //GpioWrite( &Led4, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 1 : 0 );
        }
        break;
    default:
        break;
    }


}

static void OnClassChange( DeviceClass_t deviceClass )
{
    DisplayClassUpdate( deviceClass );

    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0,
    };
    LmHandlerSend( &appData, LORAMAC_HANDLER_UNCONFIRMED_MSG );
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{
    switch( params->State )
    {
        case LORAMAC_HANDLER_BEACON_RX:
        {
            //TimerStart( &LedBeaconTimer );
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            //TimerStop( &LedBeaconTimer );
            break;
        }
        default:
        {
            break;
        }
    }

    DisplayBeaconUpdate( params );
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{

}
#else
static void OnSysTimeUpdate( void )
{

}
#endif

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame( void )
{
    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    // DEBUG start (fill some test data to send)
    /*
    tdr_data[0].int1_integer = 1;
	tdr_data[0].int1_decimal = 2;
	tdr_data[0].int2_integer = 3;
	tdr_data[0].int2_decimal = 4;
	tdr_data[0].th1_temp = 5;
	tdr_data[0].th2_temp = 6;
	tdr_data[0].th3_temp = 7;
	tdr_data[0].th4_temp = 8;
	tdr_data[0].honey_rh_integer = 9;
	tdr_data[0].honey_rh_decimal = 10;
	tdr_data[0].honey_temp_integer = 11;
	tdr_data[0].honey_temp_decimal = 12;
	*/
    // DEBUG end


    // Specify the port on which to send
    AppData.Port = LORAWAN_APP_PORT;

    uint8_t packet_length = sizeof(tdr_data);

    // Copy the contents of the tdr_data variable into the appdata buffer
    memcpy1(AppData.Buffer, tdr_data, packet_length);

    // The size of the buffer should always be equal to the maximum size
    AppData.BufferSize = packet_length;

    LmHandlerErrorStatus_t t = LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed );

    if(t == LORAMAC_HANDLER_SUCCESS) {
    	uplinksSent++;
    }
}

static void StartTxProcess( LmHandlerTxEvents_t txEvent )
{
    switch( txEvent )
    {
    default:
        // Intentional fall through
    case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit( &TxTimer, OnTxTimerEvent );
            TimerSetValue( &TxTimer, TxPeriodicity );
            OnTxTimerEvent( NULL );
        }
        break;
    case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
        break;
    }
}

static void UplinkProcess( void )
{
    uint8_t isPending = 0;
    CRITICAL_SECTION_BEGIN( );
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    CRITICAL_SECTION_END( );

    if( isPending == 1 )
    {
        PrepareTxFrame( );
    }
}

static void OnTxPeriodicityChanged( uint32_t periodicity )
{
    TxPeriodicity = periodicity;

    if( TxPeriodicity == 0 )
    { // Revert to application default periodicity
        TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
    }

    // Update timer periodicity
    TimerStop( &TxTimer );
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity )
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}
uint32_t x = 0;
/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
	// DEBUG start
	if( LmHandlerJoinStatus( ) == LORAMAC_HANDLER_SET ) {
		tester = 1;
		x++;
	}
	PAJ("OnTxTimerEvent\n");
	// DEBUG end
	TimerStop(&TxTimer);

	IsTxFramePending = 1;

	// Schedule next transmission
	TimerSetValue(&TxTimer, TxPeriodicity);
	TimerStart(&TxTimer);
}

static void OnSleepTimerEvent( void* context )
{
	iHibernateExitFlag = 1;
	//adi_gpio_Toggle(ADI_GPIO_PORT2, ADI_GPIO_PIN_0);
}






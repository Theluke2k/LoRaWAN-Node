/*
 * main.c
 *
 *  Created on: 29. aug. 2023
 *      Author: lselm
 */
#include <stdio.h>
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
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

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
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_OFF

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size (Er 242 rigtigt?)
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

/*!
 * LoRaWAN application port (Hvad skal denne sættes til?)
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
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE] = {1};

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData = { .Buffer = AppDataBuffer, .BufferSize = 0,
		.Port = 0, };


/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

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
 * Lucas (22-10-23):
 * Main program.
 */
int main(void) {
	printf("Program Start...\n");
	BoardInitMcu( );
	NVIC_SetPriority(SYS_GPIO_INTA_IRQn, 1);
	NVIC_SetPriority(SPI0_EVT_IRQn, 0);
	NVIC_SetPriority(RTC1_EVT_IRQn, 1);
	//NVIC_SetPriority(SPI0_EVT_IRQn, 1);
	//Hejsa


	// Initialize transmission perhiodicity variable
	TxPeriodicity = APP_TX_DUTYCYCLE
			+ randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);

	const Version_t appVersion = { .Value = FIRMWARE_VERSION };
	const Version_t gitHubVersion = { .Value = GITHUB_VERSION };

	//DisplayAppInfo("periodic-uplink-lpp", &appVersion, &gitHubVersion);

	if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams) != LORAMAC_HANDLER_SUCCESS) {
		printf("LoRaMac wasn't properly initialized\n");
		// Fatal error, endless loop.
		while (1)
		{
		}
	}
	//printf("1...\n");

	// Set system maximum tolerated rx error in milliseconds
	LmHandlerSetSystemMaxRxError( 20 );

	// The LoRa-Alliance Compliance protocol package should always be
	// initialized and activated.
	LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );

	//printf("Joining...\n");
	LmHandlerJoin( );

	StartTxProcess( LORAMAC_HANDLER_TX_ON_TIMER );
/*
	while(1) {
		//volatile int *register_ptr = (volatile int *)0x40020070;
		//printf("INT STATUS: %x\n", *register_ptr);
	}
*/

	//printf("Uplink Process Starting...\n");
	while (1) {
		// Processes the LoRaMac events
		//printf("LmHandlerProcess...\n");
		LmHandlerProcess();

		//printf("UplinkProcess...\n");
		// Process application uplinks management
		UplinkProcess();

		//printf("Critical begin...\n");
		CRITICAL_SECTION_BEGIN( );
		if (IsMacProcessPending == 1) {
			// Clear flag and prevent MCU to go into low power modes.
			IsMacProcessPending = 0;
			//printf("ProcessPending...\n");
		}
		else
		{
			//printf("BoardLowPower...\n");
			// The MCU wakes up through events
			BoardLowPowerHandler();
		}
		//printf("Critical end...\n");
		CRITICAL_SECTION_END( );
		//printf("end\n");
	}

	return 0;
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
            //AppLedStateOn = appData->Buffer[0] & 0x01;
            //GpioWrite( &Led4, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 1 : 0 );
        }
        break;
    default:
        break;
    }

    // Switch LED 2 ON for each received downlink
    //GpioWrite( &Led3, 1 );
    //TimerStart( &Led3Timer );
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

    //uint8_t channel = 0;

    AppData.Port = LORAWAN_APP_PORT;
    //AppDataBuffer
    /*
     * Lucas (17-11-23):
     * We do not need anything related to Cayenne. We must define our
     * own structure of what we want to send.
     *
     * Use the AppDataBuffer to specify what to send!
    */
    /*
    CayenneLppReset( );
    CayenneLppAddDigitalInput( channel++, AppLedStateOn );
    CayenneLppAddAnalogInput( channel++, BoardGetBatteryLevel( ) * 100 / 254 );

    CayenneLppCopy( AppData.Buffer );
    AppData.BufferSize = CayenneLppGetSize( );
	*/

    memcpy1(AppData.Buffer, AppDataBuffer, LORAWAN_APP_DATA_BUFFER_MAX_SIZE);
    AppData.BufferSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE;

    if( LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed ) == LORAMAC_HANDLER_SUCCESS )
    {
        // Switch LED 1 ON
        //GpioWrite( &Led1, 1 );
        //TimerStart( &Led1Timer );
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

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
	printf("OnTxTimerEvent\n");

    TimerStop( &TxTimer );

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}




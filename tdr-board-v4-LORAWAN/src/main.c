
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
#include "sx1276-board.h"
#include "eeprom-board.h"
#include "au_sensor_collection.h"

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
#define APP_TX_DUTYCYCLE                           	1000000 // Currently Unused

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
static TimerEvent_t RawLoRaStartInTimer;
static TimerEvent_t RawLoRaDurationTimer;
static TimerEvent_t RawLoRaPeriodicityTimer;
static TimerEvent_t UplinkPeriodicityTimer;

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
static void OnRawLoRaStartInEvent(void* context);
static void OnRawLoRaDurationEvent(void* context);
static void OnRawLoRaPeriodicityEvent(void* context);
static void OnUplinkPeriodicityEvent(void* context);

/*!
 * Custom Functions
 */
static uint8_t CLIHandler2(LmHandlerAppData_t* appData);
float SCVoltageAfterTX(float V_cap_i, float C, float t_tx, float V_sup, float i_tx);
bool IsVoltageSufficient(float V_cap_i, float t_tx, float V_sup, float i_sup);
void UpdateSuperCapVariables(uint16_t *ADC_meas, float *V_SC_m_local, float *V_SC_local, bool *TXreadyFlag_local, float t_tx_local, float V_sup, float i_sup);
void LoRaWANModeSleep();


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
uint8_t packed_data[16] = {0};
volatile uint32_t iHibernateExitFlag = 0;
volatile uint8_t print_flag = 0;
uint8_t desiredUplinks = 0;
uint8_t uplinksSent = 0;
uint8_t initialized = 0;
uint32_t uplinkPeriodicity = 10000;
int32_t sleepTime = 0;
const uint32_t maxInitializationTime = 1500; // Maximum time it takes between wakeup and next uplink.
const uint32_t maxInitializationTimeRawLoRa = 300;
int32_t sleepTimeOffset = 0;
uint32_t lastUplinkTime = 0;
uint32_t lastRawTX = 0;
uint32_t uplinkTimeDiff = 0;
const int32_t minSleepMs = 300;
const uint32_t chargingTime = 5000;

// For supercap calculations (SC = Super Capacitor)
const float C_SC = 1.0;			// Supercap capacitance
const float V_sup = 3.3;		// Radio supply voltage
const float V_SC_min = 3.8;	// Minimum supercap voltage for operation
const float i_tx = 0.12;		// Current draw during radio transmission
const float R102 = 150.0;
const float R104 = 100.0;
float t_tx = 3;				// Radio transmission time
float V_SC;				// Voltage over the supercapacitor
float V_SC_e;				// Voltage over the supercapacitor after hypothetical transmission
float V_SC_m;			// Measured voltage after voltage division
uint16_t ADC_SC;
bool TXreadyFlag = false;

// Logical Flags
uint8_t enableSleepFlag = 0;
uint8_t isJoiningFlag = 1; // Initialize as 1
uint8_t hasHibernated = 0;
uint8_t testModeInitialized = 0;
uint8_t rawLoRaEnabled = 0;

// Function definitions
int32_t getSleepTimeOffset(uint32_t random_value, int32_t MIN, int32_t MAX);

// Raw LoRa configuration defaults
static RawLoRa_Config RawLoRaConfig =
{
		.StartIn = 5000,
		.Duration = 10000,
		.TxPeriodicity = 5000,
		.SpreadingFactor = 12,
		.Frequency = 868000000,
};

/*
 * Lucas (22-10-23):
 * Main program.
 */
int main(void) {
	uint16_t index = 0;

 	// Initializes the system clock and needed drivers.
 	init_system();

	// Set interrup priorities. SPI must have highest prioriy!
	NVIC_SetPriority(SYS_GPIO_INTA_IRQn, 2);
	NVIC_SetPriority(SPI0_EVT_IRQn, 1);
	NVIC_SetPriority(RTC1_EVT_IRQn, 2);
	NVIC_SetPriority(RTC0_EVT_IRQn, 2);

 	// Create timers
 	TimerInit( &SleepTimer, OnSleepTimerEvent );
 	TimerInit( &RawLoRaStartInTimer, OnRawLoRaStartInEvent );
 	TimerInit( &RawLoRaDurationTimer, OnRawLoRaDurationEvent );
 	TimerInit( &RawLoRaPeriodicityTimer, OnRawLoRaPeriodicityEvent );
 	TimerInit( &UplinkPeriodicityTimer, OnUplinkPeriodicityEvent );

	while (1) {
		// Initialize board with AAU settings
		BoardInitMcu();

		/*
		 * If the flag is set, the board executes the raw lora session
		 * until the flag is disabled unset.
		 */
		if (rawLoRaEnabled) {
			// Start LoRa session
			while (rawLoRaEnabled) {
				RawLoRaSession();
			}
			// Reset settings to to be sure
			BoardInitMcu();
		}

		// Read supercap voltage and check if we have enough power to send.
		UpdateSuperCapVariables(&ADC_SC, &V_SC_m, &V_SC, &TXreadyFlag, t_tx,V_sup, i_tx);

		// Sleep until we have enough power
		while (!TXreadyFlag) {
			LoRaWANModeSleep();

			UpdateSuperCapVariables(&ADC_SC, &V_SC_m, &V_SC, &TXreadyFlag, t_tx,V_sup, i_tx);
		}

		/*
		 * AU runs their measurements. The data is stored in tdr_data.
		 * The stack uses this struct as data source when transmitting data.
		 */

		// Init data struct
		init_sensor_collection();
		sensor_data_struct sensor_data; // Different struct from Viktors code!
    	get_all_sensor_data(&sensor_data);

        // Pack the bits efficiently
        pack_sensor_data(&sensor_data, packed_data);

    	de_init_sensor_collection();

    	// Reinitialize board
    	BoardInitMcu();

		// Specify the amount of desired uplinks before going to sleep.
		desiredUplinks = 1;

		/*
		 * Run the LoRaMac stack.
		 */
		// Initlialize board (sets up pins as LoRaMac wants it)


		// Download mirror from EEPROM
		uint32_t ST = RtcGetTimerValue();
		EepromDownloadMirror(0, EMULATED_EEPROM_SIZE);
		uint32_t ET = RtcTick2Ms(RtcGetTimerValue() - ST);

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
			LmHandlerJoin(); // DELETE THIS IF WE WANT TO NOT HAVING THE BOARD JOIN ON EACH STARTUP?

			// Mark the program as initiated.
			initialized = 1;
		}

		// Reset number of uplinks for this power cycle.
		uplinksSent = 0;


		// MAIN STATE MACHINE IN ACTIVE MODE
		do  {

			// Processes the LoRaMac events
			LmHandlerProcess();

			// Try to send uplink
			if (uplinksSent < desiredUplinks) {
				LmHandlerErrorStatus_t deviceTimeReq = LmHandlerDeviceTimeReq();
				if (LmHandlerIsBusy() == false && uplinksSent == 0) {
					// Read supercap voltage and check if we have enough power to send.
					UpdateSuperCapVariables(&ADC_SC, &V_SC_m, &V_SC,
							&TXreadyFlag, t_tx, V_sup, i_tx);
					if (!TXreadyFlag) {
						// If there is not enough energy to send an uplink. Go directly to sleep
						break;
					}
				}
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

				// Prepare for hibernation
				//SystemPrepareHibernate();

				// Start sleep timer
				TimerStart(&SleepTimer);

				// Enter hibernation mode
				iHibernateExitFlag = 0;
				enter_hibernation();
				iHibernateExitFlag = 0;

				// Reinitializez required systems after hibernate wakeup.
				SystemReinitializerFromHibernate();
			}
			CRITICAL_SECTION_END( );

		} while (1);

		// If Raw LoRa is ready to be enabled, dont go to sleep. Just start the loop over
		if(rawLoRaEnabled) {
			continue;
		}

		// Generate a random uint32_t using Radio.Random(). Map value to min -3000 and max 3000
		sleepTimeOffset = getSleepTimeOffset(Radio.Random(), 500-uplinkPeriodicity, 5000); // NOT TESTED (default: -3000, 3000)

		// Deinitialize Loramac
		LmHandlerDeInit();

		// Set radio to sleep
		//Radio.Write(0x01, 0x00);

		// Calculate time offset of +- 3000 ms to avoid packet collisions
		//TimerSetValue( &SleepTimer, sleepTime + 0); // DEBUG (default + sleepTimeOffset)

		// Upload the EEPROM mirror
		ST = RtcGetTimerValue();
		EepromUploadMirror(0, EMULATED_EEPROM_SIZE);
		ET = RtcTick2Ms(RtcGetTimerValue() - ST);

		LoRaWANModeSleep();
	}
	return 0;
}

void LoRaWANModeSleep() {
	if (TXreadyFlag) {
		// Compute sleepTime
		sleepTime = uplinkPeriodicity - (TimerGetCurrentTime() - lastUplinkTime)
				- maxInitializationTime;
	} else {
		// Sleep for X seconds to charge
		sleepTime = chargingTime;
	}

	// Start timer and enter hibernation is sleepTime is larger than 300 ms.
	if (sleepTime > minSleepMs) {
		// De-initialize system we don't need while hibernating
		SystemPrepareHibernate();

		// Set timers
		TimerSetValue(&UplinkPeriodicityTimer, sleepTime);
		TimerStart(&UplinkPeriodicityTimer);

		// Enter hibernation
		iHibernateExitFlag = 0;
		enter_hibernation();
		iHibernateExitFlag = 0;

		// Re-initialize system after hibernate
		SystemReinitializerFromHibernate();
	}
}

float SCVoltageAfterTX(float V_cap_i, float C, float t_tx, float V_sup, float i_tx) {
	return sqrt(V_cap_i*V_cap_i - (2*i_tx*V_sup*t_tx) / C);
}

bool IsVoltageSufficient(float V_cap_i, float t_tx, float V_sup, float i_sup) {
	// Compute voltage after transmission
	float V_cap_e = SCVoltageAfterTX(V_cap_i, C_SC, t_tx, V_sup, i_sup);

	// Check if the end voltage is over the required operating voltage
	if( V_cap_e >= V_SC_min ) {
		return true;
	}
	else {
		return false;
	}
}

void UpdateSuperCapVariables(uint16_t *ADC_meas, float *V_SC_m_local, float *V_SC_local, bool *TXreadyFlag_local, float t_tx_local, float V_sup, float i_sup) {
	// Initialize local variables
	float ADC = 0;
	float V_measured = 0;
	float V_real = 0;
	bool readyFlag = 0;

	// Get the ADC measurement
	ADC = getADCSuperCapMeasurement(); // Get measured voltage

	// Convert ADC value to voltage
	V_measured = ADC * (2.5 / 4096.0);

	// Compute real voltage over the supercap
	V_real = V_measured * 2.5;

	// Save in global varaibles
	*ADC_meas = ADC;
	*V_SC_m_local = V_measured;
	*V_SC_local = V_real;

	// Get voltage over supercap after transmission
	readyFlag = IsVoltageSufficient(V_real, t_tx_local, V_sup, i_sup);

	// Save in global
	*TXreadyFlag_local = readyFlag;

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
 * Lucas (23-08-2024):
 * Code that executes during raw LoRa session.
 */
void RawLoRaSession() {
	// Read supercap voltage and check if we have enough power to send.
	UpdateSuperCapVariables(&ADC_SC, &V_SC_m, &V_SC, &TXreadyFlag, t_tx, V_sup, i_tx);

	if (TXreadyFlag) {
		// Get data to send?
		//??????????????????

		// For dummy data, send the same as in LoRaWAN
		uint8_t packet_length = sizeof(tdr_data);
		memcpy1(AppData.Buffer, tdr_data, packet_length);
		AppData.BufferSize = packet_length;

		// Configure the radio
		RawLoRa_RadioConfig(&RawLoRaConfig);

		// Wait until it is time to send
		uint32_t currentTime = TimerGetCurrentTime();
		while (currentTime - lastRawTX < RawLoRaConfig.TxPeriodicity) {
			currentTime = TimerGetCurrentTime();
		}

		// Send the data packet
		RawLoRa_RadioSend(AppData.Buffer, AppData.BufferSize);
		lastRawTX = currentTime;
	}

	if (TXreadyFlag) {
		// Compute sleepTime
		sleepTime = uplinkPeriodicity - (TimerGetCurrentTime() - lastUplinkTime)- maxInitializationTime;
		}
	else {
		// Sleep for X seconds to charge
		sleepTime = chargingTime;
	}

	// Compute sleep time
	sleepTime = RawLoRaConfig.TxPeriodicity - (TimerGetCurrentTime() - lastRawTX) - maxInitializationTimeRawLoRa;

	// Start timer and enter hibernation is sleepTime is larger than 300 ms.
	if(sleepTime > minSleepMs) {
		// Prepeare board to hibernate
		SystemPrepareHibernate();

		// Set timers for session duration and TX periodicity.
		TimerSetValue(&RawLoRaPeriodicityTimer,	sleepTime);
		TimerStart(&RawLoRaPeriodicityTimer);

		// Hibernate
		iHibernateExitFlag = 0;
		enter_hibernation();
		iHibernateExitFlag = 0;

		// Reinitializez required systems after hibernate wakeup.
		SystemReinitializerFromHibernate();
	}
}


static uint8_t CLIHandler2(LmHandlerAppData_t* appData) { // BEFORE: char *command_string
	// Define expected CLI functions:
	if (appData->Buffer == NULL || appData->BufferSize == 0) {
		return false;
	}

	// Save appdata buffer in new buffer that can be modified
	char command_string[appData->BufferSize]; // Add extra space for null terminator
	memcpy(command_string, appData->Buffer, appData->BufferSize); // Copy original array


	// Number of commands received.
	uint8_t num_of_commands = 0;

	// Split the command string into individual commands separated by ;
	char *command = strtok(command_string, ";");

	// Loop through all of the commands in the command string
	while (command != NULL) {
		/* PROCESS COMMAND */
		// Ping
		if (strncmp(command, "Pi=", 3) == 0) {
			// Execute handler for command
			printf("ping\n");
			num_of_commands++;
		}
		// LoRaWAN uplink periodicity
		else if (strncmp(command, "UpPe=", 5) == 0) {
			int time = 0;
			if (sscanf(command, "UpPe=%d", &time) == 1) {
				// Execute handler for command
				// Check validity
				if(time >= maxInitializationTime) {
					uplinkPeriodicity = time;
				}

				num_of_commands++;
			}
		}
		else if (strncmp(command, "SeAd=", 5) == 0) {
			int setADR = 0;
			if (sscanf(command, "SeAd=%d", &setADR) == 1) {
				// Execute handler for command

				// Check validity
				if (setADR == 1 || setADR == 0) {
					if (LmHandlerParams.AdrEnable != setADR) {
						// Update ADR parameter
						LmHandlerParams.AdrEnable = setADR;

						// Reinitialize LmHandler
						if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams) != LORAMAC_HANDLER_SUCCESS) {
							printf("LoRaMac wasn't properly initialized\n");
							// Fatal error, endless loop.
							while (1) {
							}
						}
					}
				}
				num_of_commands++;
			}
		}
		else if (strncmp(command, "SeDa=", 5) == 0) {
			int set_datarate = 0;
			if (sscanf(command, "SeDa=%d", &set_datarate) == 1) {
				// Execute handler for command

				// Check validity
				if (set_datarate >= 0 && set_datarate <= 5) {
					// Update ADR parameter
					LmHandlerParams.TxDatarate = set_datarate;

					// Reinitialize LmHandler
					if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams)!= LORAMAC_HANDLER_SUCCESS) {
						printf("LoRaMac wasn't properly initialized\n");
						// Fatal error, endless loop.
						while (1) {
						}
					}

				}
				num_of_commands++;
			}
		}
		/*
		 * New RawLora Commands.
		 */
		else if (strncmp(command, "RaLoStIn=", 9) == 0) {
			uint32_t startTime = 0;
			if (sscanf(command, "RaLoStIn=%d", &startTime) == 1) {
				// Execute handler for command

				// Check validity
				if (startTime < 131072000) {
					RawLoRaConfig.StartIn = startTime; // set flag

					// Set and start timer to start LoRa in amount of timer
					TimerSetValue(&RawLoRaStartInTimer, RawLoRaConfig.StartIn);
					TimerStart(&RawLoRaStartInTimer);
				}
				num_of_commands++;
			}
		} else if (strncmp(command, "RaLoDu=", 7) == 0) {
			uint32_t duration = 0;
			if (sscanf(command, "RaLoDu=%d", &duration) == 1) {
				// Execute handler for command

				// Check validity
				if (duration < 131072000 && duration > 5000) {
					RawLoRaConfig.Duration = duration; // set flag
				}
				num_of_commands++;
			}
		} else if (strncmp(command, "RaLoSF=", 7) == 0) {
			uint32_t SF = 0;
			if (sscanf(command, "RaLoSF=%d", &SF)	== 1) {
				// Execute handler for command

				// Check validity
				if (SF < 13 && SF > 6) {
					RawLoRaConfig.SpreadingFactor = SF;
				}
				num_of_commands++;
			}
		} else if (strncmp(command, "RaLoPe=", 7) == 0) {
			uint32_t period = 0;
			if (sscanf(command, "RaLoPe=%d", &period) == 1) {
				// Execute handler for command

				// Check validity
				if (period < 131072000 && period >= 1000) {
					RawLoRaConfig.TxPeriodicity = period; // set flag
				}
				num_of_commands++;
			}
		} else if (strncmp(command, "RaLoFr=:", 7) == 0) {
			uint32_t freq = 0;
			if (sscanf(command, "RaLoFr=%d", &freq) == 1) {
				// Execute handler for command

				// Check validity TODO: update this check!
				if (freq >= 0) {
					RawLoRaConfig.Frequency = freq; // set flag
				}
				num_of_commands++;
			}
		}

		// Move to the next command in the command string
		command = strtok(NULL, ";");
	}
	return num_of_commands;
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
        	CLIHandler2(appData);
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

//    // DEBUG start (fill some test data to send)
//    tdr_data[0].int1_integer = 1;
//	//tdr_data[0].int1_decimal = 2;
//	tdr_data[0].int2_integer = 3;
//	//tdr_data[0].int2_decimal = 4;
//	tdr_data[0].th1_temp = 5;
//	tdr_data[0].th2_temp = 6;
//	tdr_data[0].th3_temp = 7;
//	tdr_data[0].th4_temp = 8;
//	tdr_data[0].honey_rh_integer = 9;
//	//tdr_data[0].honey_rh_decimal = 10;
//	tdr_data[0].honey_temp_integer = 11;
//	//tdr_data[0].honey_temp_decimal = 12;
//
//    // DEBUG end

//    // Specify the port on which to send
//    AppData.Port = LORAWAN_APP_PORT;
//
//    uint8_t packet_length = sizeof(tdr_data);
//
//    // Copy the contents of the tdr_data variable into the appdata buffer
//    memcpy1(AppData.Buffer, tdr_data, packet_length);
//
//    // The size of the buffer should always be equal to the maximum size
//    AppData.BufferSize = packet_length;

    // Specify the port on which to send
    AppData.Port = LORAWAN_APP_PORT;

    uint8_t packet_length = sizeof(packed_data);

    // Copy the contents of the tdr_data variable into the appdata buffer
    memcpy1(AppData.Buffer, packed_data, packet_length);

    // The size of the buffer should always be equal to the maximum size
    AppData.BufferSize = packet_length;

    uint32_t currentTime = TimerGetCurrentTime();
    while(currentTime - lastUplinkTime < uplinkPeriodicity) {
    	currentTime = TimerGetCurrentTime();
    }

    LmHandlerErrorStatus_t t = LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed );

    if(t == LORAMAC_HANDLER_SUCCESS) {
    	uplinksSent++;
    	lastUplinkTime = currentTime;
    }
    else {
    	TimerStop( &UplinkPeriodicityTimer );
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
}

static void OnUplinkPeriodicityEvent( void* context )
{
	iHibernateExitFlag = 1;
}

static void OnRawLoRaStartInEvent( void* context )
{
	// Get the processor out of sleep if it is sleeping
	iHibernateExitFlag = 1;

	// Stop Sleeptimers
	TimerStop( &SleepTimer );
	TimerStop( &UplinkPeriodicityTimer );

	// Start timer for duration of session.
	TimerSetValue( &RawLoRaDurationTimer, RawLoRaConfig.Duration );
	TimerStart( &RawLoRaDurationTimer );

	// Set enable flag for RawLoRa session.
	rawLoRaEnabled = 1;
}

static void OnRawLoRaDurationEvent( void* context )
{
	// Wake up from sleep. Periodicity timer wont do it.
	iHibernateExitFlag = 1;

	// Stop timers
	TimerStop( &RawLoRaPeriodicityTimer );

	// Clear flag for RawLoRa session.
	rawLoRaEnabled = 0;
}

static void OnRawLoRaPeriodicityEvent( void* context )
{
	iHibernateExitFlag = 1;
}





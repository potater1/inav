#include "io/gps.h"
#include "io/serial.h"

#define GSM_TEST_SETTINGS
#define GSM_AT_COMMAND_MAX_SIZE 256
#define GSM_RESPONSE_BUFFER_SIZE 256
#define GSM_CYCLE_MS 5000 								// wait between gsm command cycles
#define GSM_AT_COMMAND_DELAY_MS 3000
#define GSM_AT_COMMAND_DELAY_MIN_MS 500

#define GSM_SMS_COMMAND_RTH 		"RTH"
#define GSM_SMS_COMMAND_ABORT_RTH 	"-RTH"

typedef enum  {
    GSM_STATE_INIT = 0,
    GSM_STATE_INIT2,
    GSM_STATE_INIT_SET_SMS_MODE,
    GSM_STATE_INIT_SET_SMS_RECEIVE_MODE,
    GSM_STATE_INIT_SET_CLIP,
    GSM_STATE_INIT_ENTER_PIN,
    GSM_STATE_DIAL,
    GSM_STATE_DIAL_HANGUP,
    GSM_STATE_READ_SMS,
    GSM_STATE_DELETE_SMS,
    GSM_STATE_SEND_SMS,
    GSM_STATE_SEND_SMS_ENTER_MESSAGE,
    GSM_STATE_CHECK_SIGNAL,
    GSM_STATE_CHECK_SIGNAL2

} gsmTelemetryState_e;

typedef enum  {
    GSM_AT_OK = 0,
    GSM_AT_ERROR,
    GSM_AT_WAITING_FOR_RESPONSE
} gsmATCommandState_e;


void readGsmResponse(void);
void readGsmResponseData(void);
void handleGsmTelemetry();
void freeGsmTelemetryPort(void);
void initGsmTelemetry(void);
void checkGsmTelemetryState(void);
void configureGsmTelemetryPort(void);
void sendATCommand(const char* command);
void sendSMS(void);
void readSMS(void);
void readOriginatingNumber(uint8_t*);
bool isGroundStationNumberDefined();




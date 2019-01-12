#include "build/build_config.h"

#include "common/axis.h"
#include "common/streambuf.h"
#include "common/utils.h"
#include "common/printf.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/fc_init.h"

#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "drivers/serial_uart.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/diagnostics.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/pitotmeter.h"

#include "telemetry/gsm.h"
#include "telemetry/telemetry.h"

#include "drivers/light_led.h"

#include "common/string_light.h"

#include <stdlib.h>
#include <string.h>

#include "build/debug.h"


//#define GSM_TEST_SETTINGS

#ifdef GSM_TEST_SETTINGS
void cliSerial(char *cmdline);
#endif

static serialPort_t *gsmPort;
static serialPortConfig_t *portConfig;
static bool gsmEnabled = false;

static uint8_t atCommand[GSM_AT_COMMAND_MAX_SIZE];
static int gsmTelemetryState = GSM_STATE_INIT;
static timeUs_t gsmNextTime = 0;
static uint8_t gsmResponse[GSM_RESPONSE_BUFFER_SIZE + 1];
static int atCommandStatus = GSM_AT_OK;
static uint8_t* gsmResponseValue = NULL;
static bool gsmWaitAfterResponse = false;
static bool readingSMS = false;

extern gpsLocation_t        GPS_home;
extern uint16_t             GPS_distanceToHome;
extern int16_t              GPS_directionToHome;
extern gpsSolutionData_t    gpsSol;

int gsmRssi;


bool isGroundStationNumberDefined() {
    return telemetryConfigMutable()->gsmGroundStationNumber[0] != '\0';
}

void readGsmResponse()
{
    DEBUG_TRACE_SYNC("%s", gsmResponse);
    if (readingSMS) {
        readingSMS = false;
        readSMS();
    }
    if (gsmResponse[0] == 'O' && gsmResponse[1] == 'K') {
        atCommandStatus = GSM_AT_OK;
        if (!gsmWaitAfterResponse) {
            gsmNextTime = millis() + GSM_AT_COMMAND_DELAY_MIN_MS;
        }
        DEBUG_TRACE_SYNC(">>>OK");
        return;
    } else if (gsmResponse[0] == 'E' && gsmResponse[1] == 'R') {
        atCommandStatus = GSM_AT_ERROR;
        if (!gsmWaitAfterResponse) {
            gsmNextTime = millis() + GSM_AT_COMMAND_DELAY_MIN_MS;
        }
        DEBUG_TRACE_SYNC(">>>ERR");
        return;
    } else if (gsmResponse[0] == 'R' && gsmResponse[1] == 'I') {        
        DEBUG_TRACE_SYNC(">>>RING");
        if (isGroundStationNumberDefined()) {
            gsmTelemetryState = GSM_STATE_SEND_SMS;
        }
    } else if (gsmResponse[0] == '+') {
        int i;
        for (i = 0; i < GSM_RESPONSE_BUFFER_SIZE && gsmResponse[i] != ':'; i++);
        if (gsmResponse[i] == ':') {
            gsmResponseValue = &gsmResponse[i+2];
            readGsmResponseData();
        }
        return;
    }
}

void readGsmResponseData()
{
    if (gsmResponse[1] == 'C' && gsmResponse[2] == 'S') {
        // +CSQ: 26,0
        gsmRssi = atoi((char*)gsmResponseValue);
        DEBUG_TRACE_SYNC(">>>RSSI:%d", gsmRssi);
    } else if (gsmResponse[1] == 'C' && gsmResponse[2] == 'L') {
        // +CLIP: "3581234567"
        readOriginatingNumber(&gsmResponse[8]);
    } else if (gsmResponse[1] == 'C' && gsmResponse[2] == 'M') {
        // +CMT: <oa>,[<alpha>],<scts>[,<tooa>,<fo>,<pid>,<dcs>,<sca>,<tosca>,<length>]<CR><LF><data>
        readOriginatingNumber(&gsmResponse[7]);
        readingSMS = true; // next gsmResponse line will be SMS content
    }
}

void readOriginatingNumber(uint8_t* rv)
{
    int i;
    uint8_t* gsn = telemetryConfigMutable()->gsmGroundStationNumber;
    if (gsn[0] != '\0')
        return;
    for (i = 0; i < 15 && rv[i] != '\"'; i++)
         gsn[i] = rv[i];
    gsn[i] = '\0';
    DEBUG_TRACE_SYNC(">>>NUM:%s",(char*)gsn);
}

void readSMS()
{
    DEBUG_TRACE_SYNC(">>>SMS:\"%s\"", gsmResponse);
    if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_TELEMETRY) == 0) {
        gsmTelemetryState = GSM_STATE_SEND_SMS;
    } else if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_RTH) == 0) {
        DEBUG_TRACE_SYNC(">>>SMS: FORCED RTH");
        activateForcedRTH();
    } else if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_ABORT_RTH) == 0) {
        DEBUG_TRACE_SYNC(">>>SMS: ABORT FORCED RTH");
        abortForcedRTH();
    }
}

void handleGsmTelemetry()
{
    static int ri = 0;
    uint32_t now = millis();

    if (!gsmEnabled)
        return;
    if (!gsmPort)
        return;

    uint8_t c;
    for (; serialRxBytesWaiting(gsmPort); ri++) {
        c = serialRead(gsmPort);
        gsmResponse[ri] = c;
        if (c == '\n' || ri == GSM_RESPONSE_BUFFER_SIZE) {
            // response line expected to end in \r\n, remove them
            gsmResponse[ri] = '\0';
            if (ri > 0)
                gsmResponse[--ri] = '\0';
            readGsmResponse();
            ri = 0;
            return;
        }
    }

    if (now < gsmNextTime)
        return;

//    DEBUG_TRACE_SYNC("gs num: #%s#",telemetryConfigMutable()->gsmGroundStationNumber);

    gsmNextTime = now + GSM_AT_COMMAND_DELAY_MS;       // by default, if OK or ERROR not received, wait this long
    gsmWaitAfterResponse = false;   // by default, if OK or ERROR received, go to next state immediately.
    switch (gsmTelemetryState) {
        case GSM_STATE_INIT:
        DEBUG_TRACE_SYNC("GSM INIT");
        sendATCommand("AT\n");
        gsmTelemetryState = GSM_STATE_INIT2;
        break;
        case GSM_STATE_INIT2:
        sendATCommand("AT+CPBS=\"SM\"\n");
        gsmTelemetryState = GSM_STATE_INIT_ENTER_PIN;
        break;
        case GSM_STATE_INIT_ENTER_PIN:
        sendATCommand("AT+CPIN=0000\n");
        gsmTelemetryState = GSM_STATE_INIT_SET_SMS_MODE;
        break;
        case GSM_STATE_INIT_SET_SMS_MODE:
        sendATCommand("AT+CMGF=1\n");
        gsmTelemetryState = GSM_STATE_INIT_SET_SMS_RECEIVE_MODE;
        break;
        case GSM_STATE_INIT_SET_SMS_RECEIVE_MODE:
        sendATCommand("AT+CNMI=3,2\n");
        gsmTelemetryState = GSM_STATE_INIT_SET_CLIP;
        break;
        case GSM_STATE_INIT_SET_CLIP:
        sendATCommand("AT+CLIP=1\n");
        gsmTelemetryState = GSM_STATE_CHECK_SIGNAL; // SMS read and delete not needed with CNMI=3,2
        break;
/*
        case GSM_STATE_READ_SMS:
        sendATCommand("AT+CMGL=\"ALL\"\n");         // list all, REC UNREAD lists unread
        gsmNextTime = now + 5000;
        gsmTelemetryState = GSM_STATE_DELETE_SMS;
        break;
        case GSM_STATE_DELETE_SMS:
        sendATCommand("AT+CMGD=1,1\n"); // ,1 : delete all read messages, max response time 5 s (delete 1 msg)
        gsmNextTime = now + 5000;
        gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        break;
*/
        case GSM_STATE_SEND_SMS:
        sendATCommand("AT+CMGS=\"");
        sendATCommand((char*)telemetryConfigMutable()->gsmGroundStationNumber);
        sendATCommand("\"\r");
        gsmTelemetryState = GSM_STATE_SEND_SMS_ENTER_MESSAGE;
        gsmNextTime = now + 100;
        break;
        case GSM_STATE_SEND_SMS_ENTER_MESSAGE:
        sendSMS();
        gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        break;
        case GSM_STATE_CHECK_SIGNAL:
        sendATCommand("AT+CSQ\n");
        gsmNextTime = now + GSM_CYCLE_MS;
        gsmWaitAfterResponse = true;
        gsmTelemetryState = GSM_STATE_INIT;
        break;
/*
        case GSM_STATE_DIAL:
        sendATCommand("ATD+");
        sendATCommand((char*)gsmGroundStationNumber);
        sendATCommand(";\n");
        gsmNextTime = now + GSM_DIAL_WAIT_MS;
        gsmWaitAfterResponse = true;
        gsmTelemetryState = GSM_STATE_DIAL_HANGUP;
        gsmDoTestDial = false;
        break;
        case GSM_STATE_DIAL_HANGUP:
        sendATCommand("ATH\n");
        gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        break;
*/
    }
}


void sendATCommand(const char* command)
{
    atCommandStatus = GSM_AT_WAITING_FOR_RESPONSE;
    int len = strlen((char*)command);
    if (len >GSM_AT_COMMAND_MAX_SIZE)
        len = GSM_AT_COMMAND_MAX_SIZE;
    serialWriteBuf(gsmPort, (const uint8_t*) command, len);
}

void sendSMS()
{
    int32_t lat = 0, lon = 0, alt = 0, gs = 0;
    int vbat = 0;

    if (sensors(SENSOR_GPS)) {
        lat = gpsSol.llh.lat;
        lon = gpsSol.llh.lon;
        gs = gpsSol.groundSpeed / 100;
    }
#if defined(USE_NAV)
    alt = getEstimatedActualPosition(Z); // cm
#else
    alt = sensors(SENSOR_GPS) ? gpsSol.llh.alt : 0; // cm
#endif
//    lat = 651231237;    lon = 243213216;
    vbat = getBatteryVoltage() * 10;    //vbat converted to mv
    int len;
    int avgSpeed = (int)round(10 * calculateAverageSpeed());
    int32_t E7 = 10000000;
    // \x1a sends msg, \x1b cancels
    len = tfp_sprintf((char*)atCommand, "VBAT:%d ALT:%ld DIST:%d SPEED:%ld TDIST:%d AVGSPD:%d.%d SATS:%d GSM:%d MODE:%d google.com/maps/@%ld.%07ld,%ld.%07ld,500m\x1a",
        vbat, alt / 100, GPS_distanceToHome, gs, getTotalTravelDistance(), avgSpeed / 10, avgSpeed % 10, gpsSol.numSat, gsmRssi,
        lat / E7, lat % E7, lon / E7, lon % E7, getFlightModeForTelemetry());
    serialWriteBuf(gsmPort, atCommand, len);
    atCommandStatus = GSM_AT_WAITING_FOR_RESPONSE;
}

void freeGsmTelemetryPort(void)
{
    closeSerialPort(gsmPort);
    gsmPort = NULL;
    gsmEnabled = false;
}

void initGsmTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_GSM);
#ifdef GSM_TEST_SETTINGS
    if (!portConfig) {
        featureSet(FEATURE_DEBUG_TRACE);
        featureSet(FEATURE_TELEMETRY);
        cliSerial("0 2 115200 9600 9600 115200"); // 0=USART1 GPS
        cliSerial("2 131072 115200 38400 9600 115200"); // 2=USART3
        cliSerial("5 32768 115200 115200 115200 115200"); // 5=USART6
        writeEEPROM();
        fcReboot(false);
    }
#endif
}

void checkGsmTelemetryState(void)
{
    if (gsmEnabled) {
        return;
    }
    configureGsmTelemetryPort();
}


void configureGsmTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    gsmPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_GSM, NULL, NULL, baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

    if (!gsmPort) {
        return;
    }
    gsmEnabled = true;
}

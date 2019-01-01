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

#include <stdlib.h>
#include <string.h>

#include "build/debug.h"

//#define GSM_TEST_SETTINGS

#ifdef GSM_TEST_SETTINGS
void cliSerial(char *cmdline);
extern serialPort_t * tracePort;
#endif

static serialPort_t *gsmPort;
static serialPortConfig_t *portConfig;
static bool gsmEnabled = false;

static uint8_t atCommand[GSM_AT_COMMAND_MAX_SIZE];
static int gsmTelemetryState = GSM_STATE_INIT;
static timeUs_t gsmTimeStamp = 0;
static timeUs_t gsmNextTime = 0;
static uint8_t gsmResponse[GSM_RESPONSE_BUFFER_SIZE + 1];
static uint8_t gsmGroundStationNumber[] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
static int atCommandStatus = GSM_AT_OK;
static uint8_t* gsmResponseValue = NULL;
static bool gsmWaitAfterResponse = false;
static bool gsmDoTestDial = false;

extern gpsLocation_t        GPS_home;
extern uint16_t             GPS_distanceToHome;
extern int16_t              GPS_directionToHome;
extern gpsSolutionData_t gpsSol;

int gsmRssi;


void readGsmResponse()
{
    DEBUG_TRACE_SYNC("%s", gsmResponse);
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
        gsmTelemetryState = GSM_STATE_SEND_SMS;
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
        if (gsmGroundStationNumber[0] != '\0')
            return;
        int i;
        uint8_t* rv = &gsmResponseValue[1];
        for (i = 0; i < 16 && rv[i] != '\"'; i++)
            gsmGroundStationNumber[i] = rv[i];
        gsmGroundStationNumber[i] = '\0';
        DEBUG_TRACE_SYNC(">>>NUM:%s",(char*)gsmGroundStationNumber);
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
    while (serialRxBytesWaiting(gsmPort)) {
        c = serialRead(gsmPort);
        gsmResponse[ri++] = c;
        if (c == '\n' || ri == GSM_RESPONSE_BUFFER_SIZE) {
            gsmResponse[ri] = '\0';
            readGsmResponse();
            ri = 0;
        }
    }

    if (now < gsmNextTime)
        return;

    gsmTimeStamp = now;
    gsmNextTime = now + 3000;       // by default, if OK or ERROR not received, wait this long
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
        gsmTelemetryState = GSM_STATE_INIT_SET_CLIP;
        break;
        case GSM_STATE_INIT_SET_CLIP:
        sendATCommand("AT+CLIP=1\n");
        if (gsmDoTestDial && now > 10000) {
            gsmTelemetryState = GSM_STATE_DIAL;
        } else {
            gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        }
        break;
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
        case GSM_STATE_SEND_SMS:
        sendATCommand("AT+CMGS=\"");
        sendATCommand((char*)gsmGroundStationNumber);
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
        case GSM_STATE_CHECK_SIGNAL2:
        if (atCommandStatus == GSM_AT_OK)
            gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        else
            gsmTelemetryState = GSM_STATE_INIT;
        break;
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
    // \x1a sends msg, \x1b cancels
    len = tfp_sprintf((char*)atCommand, "VBAT:%d ALT:%ld DIST:%d SPEED:%ld SATS:%d GSM:%d google.com/maps/@%ld.%07ld,%ld.%07ld,500m\x1a",
        vbat, alt / 100, GPS_distanceToHome, gs, gpsSol.numSat, gsmRssi, lat / 10000000, lat % 10000000, lon / 10000000, lon % 10000000);
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
    if (gsmGroundStationNumber[0] != '\0') {
        gsmDoTestDial = true;
    }
}

#include "build/build_config.h"

#include "common/axis.h"
#include "common/streambuf.h"
#include "common/utils.h"
#include "common/printf.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/fc_init.h"

#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/serial.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/diagnostics.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/gsm.h"
#include "telemetry/telemetry.h"

#include "common/string_light.h"
#include "common/typeconversion.h"
#include "build/debug.h"

#include <string.h>


#ifdef GSM_TEST_SETTINGS
void cliSerial(char *cmdline);
#endif

static serialPort_t *gsmPort;
static serialPortConfig_t *portConfig;
static bool gsmEnabled = false;

static uint8_t atCommand[GSM_AT_COMMAND_MAX_SIZE];
static int gsmTelemetryState = GSM_STATE_INIT;
static timeUs_t gsm_t_stateChange = 0;
static uint8_t gsmResponse[GSM_RESPONSE_BUFFER_SIZE + 1];
static int atCommandStatus = GSM_AT_OK;
static bool gsmWaitAfterResponse = false;
static bool readingSMS = false;
static bool failsafeMessageSent = false;

int gsmRssi;
uint32_t t_accEventDetected = 0;
uint32_t t_accEventMessageSent = 0;
uint8_t accEvent = ACC_EVENT_NONE;
char* accEventDescriptions[] = { "", "HIT! ", "DROP ", "HIT " };
char* modeDescriptions[] = { "MAN", "ACR", "ANG", "HOR", "ALH", "POS", "RTH", "WP", "LAU", "FS" };

extern gpsLocation_t        GPS_home;
extern uint16_t             GPS_distanceToHome;
extern int16_t              GPS_directionToHome;
extern gpsSolutionData_t    gpsSol;
extern navigationPosControl_t  posControl;


bool isGroundStationNumberDefined() {
    return telemetryConfig()->gsmGroundStationNumber[0] != '\0';
}

bool checkGroundStationNumber(uint8_t* rv) {
    const uint8_t* gsn = telemetryConfig()->gsmGroundStationNumber;
    if (strlen((char*)gsn) == 0) return false;
    if (gsn[0] == '+')
        gsn++;
    if (rv[0] == '+')
        rv++;
    for (int i = 0; i < 16 && rv[i] != '\"'; i++)
        if (gsn[i] != rv[i]) return false;
    return true;
}

void requestSendSMS()
{
    if (gsmTelemetryState == GSM_STATE_SEND_SMS_ENTER_MESSAGE)
        return; // sending right now, don't reissue AT command
    gsmTelemetryState = GSM_STATE_SEND_SMS;
    if (atCommandStatus != GSM_AT_WAITING_FOR_RESPONSE)
        gsm_t_stateChange = 0; // send immediately
}

void readGsmResponse()
{
#ifdef GSM_TEST_SETTINGS
    DEBUG_TRACE_SYNC("%s", gsmResponse);
#endif
    if (readingSMS) {
        readSMS();
        readingSMS = false;
        return;
    }

    uint8_t* resp = gsmResponse;
    uint32_t responseCode = 0;
    if (gsmResponse[0] == '+') {
        resp++;
    }
    responseCode = *resp++;
    responseCode <<= 8; responseCode |= *resp++;
    responseCode <<= 8; responseCode |= *resp++;
    responseCode <<= 8; responseCode |= *resp++;

#ifdef GSM_TEST_SETTINGS
    DEBUG_TRACE_SYNC("***RC: %d", responseCode);
#endif
    if (responseCode == GSM_RESPONSE_CODE_OK) {
        // OK
        atCommandStatus = GSM_AT_OK;
        if (!gsmWaitAfterResponse) {
            gsm_t_stateChange = millis() + GSM_AT_COMMAND_DELAY_MIN_MS;
        }
#ifdef GSM_TEST_SETTINGS
        DEBUG_TRACE_SYNC(">>>OK");
#endif
        return;
    } else if (responseCode == GSM_RESPONSE_CODE_ERROR) {
        // ERROR
        atCommandStatus = GSM_AT_ERROR;
        if (!gsmWaitAfterResponse) {
            gsm_t_stateChange = millis() + GSM_AT_COMMAND_DELAY_MIN_MS;
        }
#ifdef GSM_TEST_SETTINGS
        DEBUG_TRACE_SYNC(">>>ERR");
#endif
        return;
    } else if (responseCode == GSM_RESPONSE_CODE_RING) {
        // RING
#ifdef GSM_TEST_SETTINGS
        DEBUG_TRACE_SYNC(">>>RING");
#endif
    } else if (responseCode == GSM_RESPONSE_CODE_CSQ) {
        // +CSQ: 26,0
        gsmRssi = fastA2I((char*)&gsmResponse[6]);
#ifdef GSM_TEST_SETTINGS
        DEBUG_TRACE_SYNC(">>>RSSI:%d", gsmRssi);
#endif
    } else if (responseCode == GSM_RESPONSE_CODE_CLIP) {
        // we always get this after a RING when a call is incoming
        // +CLIP: "+3581234567"
        readOriginatingNumber(&gsmResponse[8]);
        if (checkGroundStationNumber(&gsmResponse[8])) {
            requestSendSMS();
        }
    } else if (responseCode == GSM_RESPONSE_CODE_CMT) {
        // +CMT: <oa>,[<alpha>],<scts>[,<tooa>,<fo>,<pid>,<dcs>,<sca>,<tosca>,<length>]<CR><LF><data>
        // +CMT: "+3581234567","","19/02/12,14:57:24+08"
        readOriginatingNumber(&gsmResponse[7]);
        if (checkGroundStationNumber(&gsmResponse[7])) {
            readingSMS = true; // next gsmResponse line will be SMS content
        } else {
            // skip SMS content
            while (serialRxBytesWaiting(gsmPort)) {
                if (serialRead(gsmPort) == '\n') return;
            }
        }
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
#ifdef GSM_TEST_SETTINGS
    DEBUG_TRACE_SYNC(">>>NUM:%s",(char*)gsn);
#endif
}

void readSMS()
{
#ifdef GSM_TEST_SETTINGS
    DEBUG_TRACE_SYNC(">>>SMS:\"%s\"", gsmResponse);
#endif
    if (sl_strcasecmp((char*)gsmResponse, GSM_SMS_COMMAND_TRANSMISSION) == 0) {
        telemetryConfigMutable()->gsmTransmissionInterval *= -1;
        return;
    } else if (sl_strcasecmp((char*)gsmResponse, GSM_SMS_COMMAND_RTH) == 0) {
        if (!posControl.flags.forcedRTHActivated) {
            activateForcedRTH();
#ifdef GSM_TEST_SETTINGS
        DEBUG_TRACE_SYNC(">>>SMS: FORCED RTH");
#endif
        } else {
            abortForcedRTH();
#ifdef GSM_TEST_SETTINGS
        DEBUG_TRACE_SYNC(">>>SMS: ABORT FORCED RTH");
#endif
        }
    }
    requestSendSMS();
}

void detectAccEvents()
{
    uint32_t now = millis();
//    float acceleration = sqrtf(vectorNormSquared(acc.accADCf));
    uint32_t accSq = sq(imuMeasuredAccelBF.x) + sq(imuMeasuredAccelBF.y) + sq(imuMeasuredAccelBF.z);

    if (telemetryConfig()->accEventThresholdHigh > 0 && accSq > sq(telemetryConfig()->accEventThresholdHigh))
        accEvent = ACC_EVENT_HIGH;
    else if (accSq < sq(telemetryConfig()->accEventThresholdLow))
        accEvent = ACC_EVENT_LOW;
    else if (telemetryConfig()->accEventThresholdNegX > 0 && imuMeasuredAccelBF.x < -telemetryConfig()->accEventThresholdNegX)
        accEvent = ACC_EVENT_NEG_X;
    else
        return;

#ifdef GSM_TEST_SETTINGS
    DEBUG_TRACE_SYNC("ACC EVENT %d A=%f", accEvent, sqrtf(accSq));
#endif
    t_accEventDetected = now;
    if (now - t_accEventMessageSent > 5000) {
        requestSendSMS();
        t_accEventMessageSent = now;
    }
}

void detectFailsafe()
{
    if (!failsafeMessageSent && FLIGHT_MODE(FAILSAFE_MODE) && ARMING_FLAG(ARMED)) {
        failsafeMessageSent = true;        
        requestSendSMS();
    }
    if (!ARMING_FLAG(ARMED))
        failsafeMessageSent = false;
}

void transmit()
{
    static uint32_t t_nextMessage = 0;

    uint32_t now = millis();

    if (!ARMING_FLAG(ARMED) || telemetryConfig()->gsmTransmissionInterval < GSM_MIN_TRANSMISSION_INTERVAL) {
        t_nextMessage = 0;
    } else if (now > t_nextMessage) {
        requestSendSMS();
        t_nextMessage = now + 1000 * telemetryConfig()->gsmTransmissionInterval;
    }
}

void handleGsmTelemetry()
{
    static uint16_t gsmResponseIndex = 0;
    uint32_t now = millis();

    if (!gsmEnabled)
        return;
    if (!gsmPort)
        return;

    while (serialRxBytesWaiting(gsmPort) > 0) {
        uint8_t c = serialRead(gsmPort);
        if (c == '\n' || gsmResponseIndex == GSM_RESPONSE_BUFFER_SIZE) {
            gsmResponse[gsmResponseIndex] = '\0';
            if (gsmResponseIndex > 0) gsmResponseIndex--;
            if (gsmResponse[gsmResponseIndex] == '\r') gsmResponse[gsmResponseIndex] = '\0';
            gsmResponseIndex = 0; //data ok
            readGsmResponse();
            break;
        } else {
            gsmResponse[gsmResponseIndex] = c;
            gsmResponseIndex++;
        }
    }

    detectAccEvents();
    detectFailsafe();
    transmit();

    if (now < gsm_t_stateChange)
        return;

    gsm_t_stateChange = now + GSM_AT_COMMAND_DELAY_MS;       // by default, if OK or ERROR not received, wait this long
    gsmWaitAfterResponse = false;   // by default, if OK or ERROR received, go to next state immediately.
    switch (gsmTelemetryState) {
        case GSM_STATE_INIT:
#ifdef GSM_TEST_SETTINGS
        DEBUG_TRACE_SYNC("GSM INIT");
#endif
        sendATCommand("AT\r");
        gsmTelemetryState = GSM_STATE_INIT2;
        break;
        case GSM_STATE_INIT2:
        sendATCommand("ATE0\r");
        gsmTelemetryState = GSM_STATE_INIT_ENTER_PIN;
        break;
        case GSM_STATE_INIT_ENTER_PIN:
        sendATCommand("AT+CPIN=" GSM_PIN "\r");
        gsmTelemetryState = GSM_STATE_SET_MODES;
        break;
        case GSM_STATE_SET_MODES:
        sendATCommand("AT+CMGF=1;+CNMI=3,2;+CLIP=1;+CSQ\r");
        gsmTelemetryState = GSM_STATE_INIT;
        gsm_t_stateChange = now + GSM_CYCLE_MS;
        break;
        case GSM_STATE_SEND_SMS:
        sendATCommand("AT+CMGS=\"");
        sendATCommand((char*)telemetryConfig()->gsmGroundStationNumber);
        sendATCommand("\"\r");
        gsmTelemetryState = GSM_STATE_SEND_SMS_ENTER_MESSAGE;
        gsm_t_stateChange = now + 100;
        break;
        case GSM_STATE_SEND_SMS_ENTER_MESSAGE:
        sendSMS();
        gsmTelemetryState = GSM_STATE_INIT;
        gsm_t_stateChange = now + GSM_CYCLE_MS;
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
    int vbat = getBatteryVoltage();
    int16_t amps = isAmperageConfigured() ? getAmperage() / 10 : 0; // 1 = 100 milliamps
    int avgSpeed = (int)round(10 * calculateAverageSpeed());
    uint32_t now = millis();

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
    int len;
    int32_t E7 = 10000000;
    // \x1a sends msg, \x1b cancels
    len = tfp_sprintf((char*)atCommand, "%s%d.%02dV %d.%dA ALT:%ld SPD:%ld/%d.%d DIST:%d/%d SAT:%d GSM:%d %s google.com/maps/place/%ld.%07ld,%ld.%07ld,500m\x1a",
        (now - t_accEventDetected) < 5000 ? accEventDescriptions[accEvent] : "",
        vbat / 100, vbat % 100,
        amps / 10, amps % 10,
        alt / 100,
        gs, avgSpeed / 10, avgSpeed % 10,
        GPS_distanceToHome, getTotalTravelDistance() / 100,
        gpsSol.numSat, gsmRssi,
        posControl.flags.forcedRTHActivated ? "RTH" : modeDescriptions[getFlightModeForTelemetry()],
        lat / E7, lat % E7, lon / E7, lon % E7);
    serialWriteBuf(gsmPort, atCommand, len);
//    DEBUG_TRACE_SYNC("%s", atCommand);
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
    gsmPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_GSM, NULL, NULL,
        baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

    if (!gsmPort) {
        return;
    }
    gsmEnabled = true;
}

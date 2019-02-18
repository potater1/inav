#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_SIM)

#include <string.h>

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

#include "common/string_light.h"
#include "common/typeconversion.h"
#include "build/debug.h"

#include "telemetry/sim.h"
#include "telemetry/telemetry.h"


static serialPort_t *simPort;
static serialPortConfig_t *portConfig;
static bool simEnabled = false;

static uint8_t atCommand[SIM_AT_COMMAND_MAX_SIZE];
static int simTelemetryState = SIM_STATE_INIT;
static timeUs_t sim_t_stateChange = 0;
static uint8_t simResponse[SIM_RESPONSE_BUFFER_SIZE + 1];
static int atCommandStatus = SIM_AT_OK;
static bool simWaitAfterResponse = false;
static bool readingSMS = false;
static bool failsafeMessageSent = false;

int simRssi;
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
    return telemetryConfig()->simGroundStationNumber[0] != '\0';
}

bool checkGroundStationNumber(uint8_t* rv) {
    const uint8_t* gsn = telemetryConfig()->simGroundStationNumber;
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
    if (simTelemetryState == SIM_STATE_SEND_SMS_ENTER_MESSAGE)
        return; // sending right now, don't reissue AT command
    simTelemetryState = SIM_STATE_SEND_SMS;
    if (atCommandStatus != SIM_AT_WAITING_FOR_RESPONSE)
        sim_t_stateChange = 0; // send immediately
}

void readSimResponse()
{
    if (readingSMS) {
        readSMS();
        readingSMS = false;
        return;
    }

    uint8_t* resp = simResponse;
    uint32_t responseCode = 0;
    if (simResponse[0] == '+') {
        resp++;
    }
    responseCode = *resp++;
    responseCode <<= 8; responseCode |= *resp++;
    responseCode <<= 8; responseCode |= *resp++;
    responseCode <<= 8; responseCode |= *resp++;

    if (responseCode == SIM_RESPONSE_CODE_OK) {
        // OK
        atCommandStatus = SIM_AT_OK;
        if (!simWaitAfterResponse) {
            sim_t_stateChange = millis() + SIM_AT_COMMAND_DELAY_MIN_MS;
        }
        return;
    } else if (responseCode == SIM_RESPONSE_CODE_ERROR) {
        // ERROR
        atCommandStatus = SIM_AT_ERROR;
        if (!simWaitAfterResponse) {
            sim_t_stateChange = millis() + SIM_AT_COMMAND_DELAY_MIN_MS;
        }
        return;
    } else if (responseCode == SIM_RESPONSE_CODE_RING) {
        // RING
    } else if (responseCode == SIM_RESPONSE_CODE_CSQ) {
        // +CSQ: 26,0
        simRssi = fastA2I((char*)&simResponse[6]);
    } else if (responseCode == SIM_RESPONSE_CODE_CLIP) {
        // we always get this after a RING when a call is incoming
        // +CLIP: "+3581234567"
        readOriginatingNumber(&simResponse[8]);
        if (checkGroundStationNumber(&simResponse[8])) {
            requestSendSMS();
        }
    } else if (responseCode == SIM_RESPONSE_CODE_CMT) {
        // +CMT: <oa>,[<alpha>],<scts>[,<tooa>,<fo>,<pid>,<dcs>,<sca>,<tosca>,<length>]<CR><LF><data>
        // +CMT: "+3581234567","","19/02/12,14:57:24+08"
        readOriginatingNumber(&simResponse[7]);
        if (checkGroundStationNumber(&simResponse[7])) {
            readingSMS = true; // next simResponse line will be SMS content
        } else {
            // skip SMS content
            while (serialRxBytesWaiting(simPort)) {
                if (serialRead(simPort) == '\n') return;
            }
        }
    }
}

void readOriginatingNumber(uint8_t* rv)
{
    int i;
    uint8_t* gsn = telemetryConfigMutable()->simGroundStationNumber;
    if (gsn[0] != '\0')
        return;
    for (i = 0; i < 15 && rv[i] != '\"'; i++)
         gsn[i] = rv[i];
    gsn[i] = '\0';
}

void readSMS()
{
    if (sl_strcasecmp((char*)simResponse, SIM_SMS_COMMAND_TRANSMISSION) == 0) {
        telemetryConfigMutable()->simTransmissionInterval *= -1;
        return;
    } else if (sl_strcasecmp((char*)simResponse, SIM_SMS_COMMAND_RTH) == 0) {
        if (!posControl.flags.forcedRTHActivated) {
            activateForcedRTH();
        } else {
            abortForcedRTH();
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

    if (!ARMING_FLAG(ARMED) || telemetryConfig()->simTransmissionInterval < SIM_MIN_TRANSMISSION_INTERVAL) {
        t_nextMessage = 0;
    } else if (now > t_nextMessage) {
        requestSendSMS();
        t_nextMessage = now + 1000 * telemetryConfig()->simTransmissionInterval;
    }
}

void handleSimTelemetry()
{
    static uint16_t simResponseIndex = 0;
    uint32_t now = millis();

    if (!simEnabled)
        return;
    if (!simPort)
        return;

    while (serialRxBytesWaiting(simPort) > 0) {
        uint8_t c = serialRead(simPort);
        if (c == '\n' || simResponseIndex == SIM_RESPONSE_BUFFER_SIZE) {
            simResponse[simResponseIndex] = '\0';
            if (simResponseIndex > 0) simResponseIndex--;
            if (simResponse[simResponseIndex] == '\r') simResponse[simResponseIndex] = '\0';
            simResponseIndex = 0; //data ok
            readSimResponse();
            break;
        } else {
            simResponse[simResponseIndex] = c;
            simResponseIndex++;
        }
    }

    detectAccEvents();
    detectFailsafe();
    transmit();

    if (now < sim_t_stateChange)
        return;

    sim_t_stateChange = now + SIM_AT_COMMAND_DELAY_MS;       // by default, if OK or ERROR not received, wait this long
    simWaitAfterResponse = false;   // by default, if OK or ERROR received, go to next state immediately.
    switch (simTelemetryState) {
        case SIM_STATE_INIT:
        sendATCommand("AT\r");
        simTelemetryState = SIM_STATE_INIT2;
        break;
        case SIM_STATE_INIT2:
        sendATCommand("ATE0\r");
        simTelemetryState = SIM_STATE_INIT_ENTER_PIN;
        break;
        case SIM_STATE_INIT_ENTER_PIN:
        sendATCommand("AT+CPIN=" SIM_PIN "\r");
        simTelemetryState = SIM_STATE_SET_MODES;
        break;
        case SIM_STATE_SET_MODES:
        sendATCommand("AT+CMGF=1;+CNMI=3,2;+CLIP=1;+CSQ\r");
        simTelemetryState = SIM_STATE_INIT;
        sim_t_stateChange = now + SIM_CYCLE_MS;
        simWaitAfterResponse = true;
        break;
        case SIM_STATE_SEND_SMS:
        sendATCommand("AT+CMGS=\"");
        sendATCommand((char*)telemetryConfig()->simGroundStationNumber);
        sendATCommand("\"\r");
        simTelemetryState = SIM_STATE_SEND_SMS_ENTER_MESSAGE;
        sim_t_stateChange = now + 100;
        break;
        case SIM_STATE_SEND_SMS_ENTER_MESSAGE:
        sendSMS();
        simTelemetryState = SIM_STATE_INIT;
        sim_t_stateChange = now + SIM_CYCLE_MS;
        simWaitAfterResponse = true;
        break;
    }
}


void sendATCommand(const char* command)
{
    atCommandStatus = SIM_AT_WAITING_FOR_RESPONSE;
    int len = strlen((char*)command);
    if (len >SIM_AT_COMMAND_MAX_SIZE)
        len = SIM_AT_COMMAND_MAX_SIZE;
    serialWriteBuf(simPort, (const uint8_t*) command, len);
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
    len = tfp_sprintf((char*)atCommand, "%s%d.%02dV %d.%dA ALT:%ld SPD:%ld/%d.%d DIS:%d/%d SAT:%d SIG:%d %s google.com/maps/place/%ld.%07ld,%ld.%07ld,500m\x1a",
        (now - t_accEventDetected) < 5000 ? accEventDescriptions[accEvent] : "",
        vbat / 100, vbat % 100,
        amps / 10, amps % 10,
        alt / 100,
        gs, avgSpeed / 10, avgSpeed % 10,
        GPS_distanceToHome, getTotalTravelDistance() / 100,
        gpsSol.numSat, simRssi,
        posControl.flags.forcedRTHActivated ? "RTH" : modeDescriptions[getFlightModeForTelemetry()],
        lat / E7, lat % E7, lon / E7, lon % E7);
    serialWriteBuf(simPort, atCommand, len);
    atCommandStatus = SIM_AT_WAITING_FOR_RESPONSE;
}

void freeSimTelemetryPort(void)
{
    closeSerialPort(simPort);
    simPort = NULL;
    simEnabled = false;
}

void initSimTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SIM);
}

void checkSimTelemetryState(void)
{
    if (simEnabled) {
        return;
    }
    configureSimTelemetryPort();
}

void configureSimTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    simPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SIM, NULL, NULL,
        baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

    if (!simPort) {
        return;
    }
    simEnabled = true;
    sim_t_stateChange = millis() + SIM_STARTUP_DELAY_MS;
}

#endif

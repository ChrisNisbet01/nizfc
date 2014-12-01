#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <receiver.h>
#include <uart.h>
#include "serial_msp.h"

extern uint32_t IMUDelta;
#define cycleTime IMUDelta
#define serialPort_t	serial_port_st

static const char MSPHeader[3] = {'$', 'M', '<'};



/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * Clients SHOULD operate in READ-ONLY mode and SHOULD present a warning to the user to state
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

#define MSP_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1 // increment when major changes are made
#define API_VERSION_MINOR                   0 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR

#define API_VERSION_LENGTH                  2

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
static const char flightControllerIdentifier[] = "NIZF"; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

const char boardIdentifier[] = "SDF3";
#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

// These are baseflight specific flags but they are useless now since MW 2.3 uses the upper 4 bits for the navigation version.
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)

// MW 2.3 stores NAVI_VERSION in the top 4 bits of the capability mask.
#define CAP_NAVI_VERSION_BIT_4_MSB  ((uint32_t)1 << 31)
#define CAP_NAVI_VERSION_BIT_3      ((uint32_t)1 << 30)
#define CAP_NAVI_VERSION_BIT_2      ((uint32_t)1 << 29)
#define CAP_NAVI_VERSION_BIT_1_LSB  ((uint32_t)1 << 28)

#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FLAPS                   ((uint32_t)1 << 3)
#define CAP_NAVCAP                  ((uint32_t)1 << 4)
#define CAP_EXTAUX                  ((uint32_t)1 << 5)

/**
 * Returns MSP protocol version
 * API version
 * Flight Controller Identifier
 * Flight Controller build version (major, minor, patchlevel)
 * Board Identifier
 * Board Hardware Revision
 * Build Date - "MMM DD YYYY" MMM = Jan/Feb/...
 * Build Time - "HH:MM:SS"
 * SCM reference length
 * SCM reference (git revision, svn commit id)
 * Additional FC information length
 * Additional FC information (as decided by the FC, for FC specific tools to use as required)
 **/
#define MSP_API_VERSION                 1    //out message

//
// MSP commands for Cleanflight original features
//
#define MSP_CHANNEL_FORWARDING          32    //out message         Returns channel forwarding settings
#define MSP_SET_CHANNEL_FORWARDING      33    //in message          Channel forwarding settings

#define MSP_MODE_RANGES                 34    //out message         Returns all mode ranges
#define MSP_SET_MODE_RANGE              35    //in message          Sets a single mode range

#define MSP_FEATURE                     36
#define MSP_SET_FEATURE                 37

#define MSP_BOARD_ALIGNMENT             38
#define MSP_SET_BOARD_ALIGNMENT         39

#define MSP_CURRENT_METER_CONFIG        40
#define MSP_SET_CURRENT_METER_CONFIG    41

#define MSP_MIXER                       42
#define MSP_SET_MIXER                   43

#define MSP_RX_CONFIG                   44
#define MSP_SET_RX_CONFIG               45

#define MSP_LED_COLORS                  46
#define MSP_SET_LED_COLORS              47

#define MSP_LED_STRIP_CONFIG            48
#define MSP_SET_LED_STRIP_CONFIG        49

#define MSP_RSSI_CONFIG                 50
#define MSP_SET_RSSI_CONFIG             51

//
// Baseflight MSP commands (if enabled they exist in Cleanflight)
//
#define MSP_RX_MAP                      64 //out message get channel map (also returns number of channels total)
#define MSP_SET_RX_MAP                  65 //in message set rx map, numchannels to set comes from MSP_RX_MAP

// DO NOT IMPLEMENT "MSP_CONFIG" and MSP_SET_CONFIG in Cleanflight, isolated commands already exist
//#define MSP_CONFIG                    66 //out message baseflight-specific settings that aren't covered elsewhere
//#define MSP_SET_CONFIG                67 //in message baseflight-specific settings save

#define MSP_REBOOT                      68 //in message reboot settings

// DEPRECATED - Use MSP_API_VERSION instead
//#define MSP_BUILD_INFO                  69 //out message build date as well as some space for future expansion

//
// Multwii original MSP commands
//

// DEPRECATED - See MSP_API_VERSION and MSP_MIXER
#define MSP_IDENT                100    //out message         multitype + multiwii version + protocol version + capability variable


#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan and more
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         altitude, variometer
#define MSP_ANALOG               110    //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113    //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114    //out message         powermeter trig
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119    //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120    //out message         Servo settings
#define MSP_NAV_STATUS           121    //out message         Returns navigation status
#define MSP_NAV_CONFIG           122    //out message         Returns navigation parameters

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203    //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_SET_WP               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210    //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211    //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212    //in message          Servo settings
#define MSP_SET_MOTOR            214    //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215    //in message          Sets nav config parameters - write to the eeprom

// #define MSP_BIND                 240    //in message          no param

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUGMSG             253    //out message         debug string buffer
#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

// Additional commands that are not compatible with MultiWii
#define MSP_UID                  160    //out message         Unique device ID
#define MSP_ACC_TRIM             240    //out message         get acc angle trim values
#define MSP_SET_ACC_TRIM         239    //in message          set acc angle trim values
#define MSP_GPSSVINFO            164    //out message         get Signal Strength (only U-Blox)

#define INBUF_SIZE 64

typedef struct box_e {
    const uint8_t boxId;         // see boxId_e
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      //
} box_t;


typedef enum {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
} mspState_e;

typedef enum {
    UNUSED_PORT = 0,
    FOR_GENERAL_MSP,
    FOR_TELEMETRY
} mspPortUsage_e;

typedef struct mspPort_s {
    serial_port_st * serialPort;
    uint8_t offset;
    uint8_t dataSize;
    uint8_t checksum;
    uint8_t indRX;
    uint8_t inBuf[INBUF_SIZE];

    uint8_t outBuf[INBUF_SIZE];
    uint_fast8_t outIdx;

    mspState_e c_state;
    uint8_t cmdMSP;
    mspPortUsage_e mspPortUsage;
} mspPort_t;

static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

static mspPort_t *currentPort;
#define mspSerialPort currentPort->serialPort

extern float RollAngle, PitchAngle, Heading;

void serialWrite( serial_port_st *port, uint8_t ch )
{
	if ( port->methods->writeBulkBlockingWithTimeout == NULL )
	{
		/* write out each char */
		port->methods->writeCharBlockingWithTimeout( port->serialCtx, ch, 50 );
	}
	else
	{
		if ( currentPort->outIdx < INBUF_SIZE )
		{
			currentPort->outBuf[currentPort->outIdx++] = ch;
		}
		if ( currentPort->outIdx == INBUF_SIZE )
		{
			port->methods->writeBulkBlockingWithTimeout( port->serialCtx, currentPort->outBuf, currentPort->outIdx, 50 );
			currentPort->outIdx = 0;
		}
	}
}

void serialFlush( serial_port_st *port )
{
	if ( port->methods->writeBulkBlockingWithTimeout != NULL && currentPort->outIdx > 0 )
	{
		port->methods->writeBulkBlockingWithTimeout( port->serialCtx, currentPort->outBuf, currentPort->outIdx, 50 );
		currentPort->outIdx = 0;
	}
}

void serialize32(uint32_t a)
{
    static uint8_t t;
    t = a;
    serialWrite(mspSerialPort, t);
    currentPort->checksum ^= t;
    t = a >> 8;
    serialWrite(mspSerialPort, t);
    currentPort->checksum ^= t;
    t = a >> 16;
    serialWrite(mspSerialPort, t);
    currentPort->checksum ^= t;
    t = a >> 24;
    serialWrite(mspSerialPort, t);
    currentPort->checksum ^= t;
}

void serialize16(int16_t a)
{
    static uint8_t t;
    t = a;
    serialWrite(mspSerialPort, t);
    currentPort->checksum ^= t;
    t = a >> 8 & 0xff;
    serialWrite(mspSerialPort, t);
    currentPort->checksum ^= t;
}

void serialize8(uint8_t a)
{
    serialWrite(mspSerialPort, a);
    currentPort->checksum ^= a;
}

uint8_t read8(void)
{
    return currentPort->inBuf[currentPort->indRX++] & 0xff;
}

uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}

uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}

void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    currentPort->checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(currentPort->cmdMSP);
}

void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void tailSerialReply(void)
{
    serialize8(currentPort->checksum);
    serialFlush(mspSerialPort);
}

void s_struct(uint8_t *cb, uint8_t siz)
{
    headSerialReply(siz);
    while (siz--)
        serialize8(*cb++);
}

void serializeNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(*c);
}

static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t * serialPort, mspPortUsage_e usage)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->serialPort = serialPort;
    mspPortToReset->mspPortUsage = usage;
}

// This rate is chosen since softserial supports it.

void mspInit(serialPort_t *serialPort)
{

    memset(mspPorts, 0, sizeof(mspPorts));

	mspPorts[0].serialPort = serialPort;

}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

extern float accelerometerValues[3];
extern float gyroValues[3];
extern float magnetometerValues[3];
extern float filteredAccelerometerValues[3];
extern float filteredGyroValues[3];
extern float filteredMagnetometerValues[3];

#define BUILD_DATE_LENGTH 11
char* buildDate = __DATE__;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
char* buildTime = __TIME__;  // "HH:MM:SS"

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
char* shortGitRevision = "1234567";

#define API_VERSION_MAJOR                   1 // increment when major changes are made
#define API_VERSION_MINOR                   0 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR

#define API_VERSION_LENGTH                  2
#define FC_VERSION_MAJOR            1  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            0  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed
#define MW_VERSION              230
#define U_ID_0 (*(uint32_t*)0x1FFFF7AC)
#define U_ID_1 (*(uint32_t*)0x1FFFF7B0)
#define U_ID_2 (*(uint32_t*)0x1FFFF7B4)

static bool processOutCommand(uint8_t cmdMSP)
{
    uint32_t i, tmp, junk;

    switch (cmdMSP) {
    case MSP_API_VERSION:
        // the components of this command are in an order such that future changes could be made to it without breaking clients.
        // i.e. most important first.
        headSerialReply(
            1 + // protocol version length
            API_VERSION_LENGTH +
            FLIGHT_CONTROLLER_IDENTIFIER_LENGTH +
            FLIGHT_CONTROLLER_VERSION_LENGTH +
            BOARD_IDENTIFIER_LENGTH +
            BOARD_HARDWARE_REVISION_LENGTH +
            BUILD_DATE_LENGTH +
            BUILD_TIME_LENGTH +
            1 + // scm reference length
            GIT_SHORT_REVISION_LENGTH +
            1 // additional FC specific length
            // no addition FC specific data yet.
        );
        serialize8(MSP_PROTOCOL_VERSION);

        serialize8(API_VERSION_MAJOR);
        serialize8(API_VERSION_MINOR);

        for (i = 0; i < FLIGHT_CONTROLLER_IDENTIFIER_LENGTH; i++) {
            serialize8(flightControllerIdentifier[i]);
        }

        serialize8(FC_VERSION_MAJOR);
        serialize8(FC_VERSION_MINOR);
        serialize8(FC_VERSION_PATCH_LEVEL);

        for (i = 0; i < BOARD_IDENTIFIER_LENGTH; i++) {
            serialize8(boardIdentifier[i]);
        }
        serialize16(0); // No other build targets currently have hardware revision detection.

        for (i = 0; i < BUILD_DATE_LENGTH; i++) {
            serialize8(buildDate[i]);
        }
        for (i = 0; i < BUILD_TIME_LENGTH; i++) {
            serialize8(buildTime[i]);
        }

        serialize8(GIT_SHORT_REVISION_LENGTH);
        for (i = 0; i < GIT_SHORT_REVISION_LENGTH; i++) {
            serialize8(shortGitRevision[i]);
        }
        serialize8(0); // No flight controller specific information to follow.
        break;
    case MSP_IDENT:
        headSerialReply(7);
        serialize8(MW_VERSION);
        serialize8(3); // type of multicopter
        serialize8(MSP_PROTOCOL_VERSION);
        serialize32(CAP_DYNBALANCE); // "capability"
        break;

    case MSP_STATUS:
        headSerialReply(11);
        serialize16(cycleTime);
        serialize16(0);
        serialize16(5);	/* bit 0 == gyro + accel, bit 1 = baro, bit 2 == magnetometer, bit 3 = GPS, bit 4 = sonar */
        serialize32(0);
        serialize8(0);
        break;
    case MSP_RAW_IMU:
        headSerialReply(18);
        // Retarded hack until multiwiidorks start using real units for sensor data
        for (i = 0; i < 3; i++)
	        serialize16(lrintf(filteredAccelerometerValues[i]*512.0f));
        for (i = 0; i < 3; i++)
            serialize16(lrintf(gyroValues[i]*16.4f/4.0f));
        for (i = 0; i < 3; i++)
            serialize16(lrintf(filteredMagnetometerValues[i]*1090.0f));
        break;
    case MSP_MOTOR:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
	        serialize16(getMotorValue(i));
        break;
    case MSP_RC:
        headSerialReply(2 * 8);
        for (i = 0; i < 8; i++)
            serialize16(readReceiverChannel(i));
        break;
    case MSP_RC_TUNING:
        headSerialReply(7);
        serialize8(0);
        serialize8(0);
        serialize8(0);
        serialize8(0);
        serialize8(0);
        serialize8(0);
        serialize8(0);
        break;
    case MSP_SERVO:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
	        serialize16(0);
        break;
    case MSP_ATTITUDE:
        headSerialReply(6);
        serialize16(lrintf(RollAngle*10.0f));
        serialize16(lrintf(PitchAngle*10.0f));
        serialize16(lrintf(Heading));
        break;
    case MSP_ANALOG:
        headSerialReply(7);
        serialize8(100);
        serialize16(0); // milliamphours drawn from battery
        serialize16(700);
        serialize16(0); // send amperage in 0.01 A steps
        break;
    case MSP_BOXNAMES:
        headSerialReply(0);
        break;
    case MSP_BOXIDS:
        headSerialReply(0);
        break;
    case MSP_MOTOR_PINS:
        headSerialReply(8);
        for (i = 0; i < 8; i++)
            serialize8(i + 1);
        break;
    case MSP_DEBUG:
        headSerialReply(8);
        for (i = 0; i < 4; i++)
            serialize16(0);      // 4 variables are here for general monitoring purpose
        break;
    case MSP_ACC_TRIM:
        headSerialReply(4);
        serialize16(0);
        serialize16(0);
        break;
    case MSP_UID:
        headSerialReply(12);
        serialize32(U_ID_0);
        serialize32(U_ID_1);
        serialize32(U_ID_2);
        break;
    case MSP_MISC:
        headSerialReply(2 * 6 + 4 + 2 + 4);
        serialize16(0); // intPowerTrigger1 (aka useless trash)
        serialize16(1000);	/* min trottle */
        serialize16(2000);	/* max throttle */
        serialize16(1000);	/* min command */
        serialize16(1000);	/* failsafe throttle */
        serialize16(0); // plog useless shit
        serialize32(0); // plog useless shit
        serialize16(0); // magnetic declination
        serialize8(10);
        serialize8(35);
        serialize8(42);
        serialize8(0);
        break;
    default:
        return false;
    }
    return true;
}

static bool processInCommand(void)
{
    uint32_t i;

    switch (currentPort->cmdMSP) {
    case MSP_SET_MOTOR:
        for (i = 0; i < 8; i++) // FIXME should this use MAX_MOTORS or MAX_SUPPORTED_MOTORS instead of 8
            setMotorDisarmedValue( i, read16() );
        break;
    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return false;
    }
    headSerialReply(0);
    return true;
}

static bool mspProcessPort(uint8_t c)
{
	bool receivingMSP = true;

	/* if the state is idle at the end of the function, receivingMSP will be set to false */
    {
        if (currentPort->c_state == IDLE) {
            currentPort->c_state = (c == MSPHeader[0]) ? HEADER_START : IDLE;
        } else if (currentPort->c_state == HEADER_START) {
            currentPort->c_state = (c == MSPHeader[1]) ? HEADER_M : IDLE;
        } else if (currentPort->c_state == HEADER_M) {
            currentPort->c_state = (c == MSPHeader[2]) ? HEADER_ARROW : IDLE;
        } else if (currentPort->c_state == HEADER_ARROW) {
            if (c > INBUF_SIZE)
            {       // now we are expecting the payload size
                currentPort->c_state = IDLE;
            }
            else
            {
	            currentPort->dataSize = c;
	            currentPort->offset = 0;
	            currentPort->checksum = 0;
	            currentPort->indRX = 0;
	            currentPort->checksum ^= c;
	            currentPort->c_state = HEADER_SIZE;      // the command is to follow
            }
        } else if (currentPort->c_state == HEADER_SIZE) {
            currentPort->cmdMSP = c;
            currentPort->checksum ^= c;
            currentPort->c_state = HEADER_CMD;
        } else if (currentPort->c_state == HEADER_CMD && currentPort->offset < currentPort->dataSize) {
            currentPort->checksum ^= c;
            currentPort->inBuf[currentPort->offset++] = c;
        } else if (currentPort->c_state == HEADER_CMD && currentPort->offset >= currentPort->dataSize) {
            if (currentPort->checksum == c) {        // compare calculated and transferred checksum
                // we got a valid packet, evaluate it
		        if (!(processOutCommand(currentPort->cmdMSP) || processInCommand()))
		        {
    				printf("\r\nunsupported code: %d", currentPort->cmdMSP );
                    headSerialError(0);
                }
                tailSerialReply();
            }

            currentPort->c_state = IDLE;
        }
    }

    if (currentPort->c_state == IDLE )
    	receivingMSP = false;

    return receivingMSP;
}

void setCurrentPort(mspPort_t *port)
{
    currentPort = port;
    mspSerialPort = currentPort->serialPort;
}

bool mspProcess(serial_port_st *port, uint8_t c)
{
	mspPorts[0].serialPort = port;
    setCurrentPort(&mspPorts[0]);

    return mspProcessPort(c);
}


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <coos.h>
#include <utils.h>
#include <pid.h>
#include <roll_pitch_configuration.h>
#include <receiver.h>
#include <receiver_handler.h>
#include <outputs.h>
#include <stm32f3_discovery.h>

typedef struct motorMixRatios_st
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixRatios_st;

typedef struct craftType_st
{
	const char              * name;
    const motorMixRatios_st * motorRatios;
    uint_fast8_t 			nbMotors;
} craftType_st;


extern float RollAngFiltered, PitchAngFiltered, Heading;
extern float filteredGyroValues[3];

static pid_st rollAnglePID;
static pid_st pitchAnglePID;
static pid_st yawRatePID;

static const motorMixRatios_st HQuadMixRatios[] =
{
    { .throttle = 1.0f, .roll = -1.0f, .pitch = 1.0f,  .yaw = -1.0f }, // right rear
    { .throttle = 1.0f, .roll = -1.0f, .pitch = -1.0f, .yaw = 1.0f  }, // right front
    { .throttle = 1.0f, .roll = 1.0f,  .pitch = 1.0f,  .yaw = 1.0f  }, // left rear
    { .throttle = 1.0f, .roll = 1.0f,  .pitch = -1.0f, .yaw = -1.0f }, // left front
};

static const craftType_st crafts[] =
{
	{
	.name = "hquad",
	.motorRatios = HQuadMixRatios,
	.nbMotors = ARRAY_SIZE(HQuadMixRatios)
	}
};

static uint_fast16_t motorValues[6];	// TODO: defined limit
static uint_fast16_t disarmedMotorValues[6];	// TODO: defined limit


void initMotorControl( void )
{
	unsigned int index;

	// TODO: separate config for roll and pitch
	// TODO: runtime update of PID settings.
	initPID( &rollAnglePID,
				roll_configuration[0].pidRange,
				roll_configuration[0].kP,
				roll_configuration[0].kI,
				roll_configuration[0].kD,
				roll_configuration[0].integralLimit,
				roll_configuration[0].dLimit );

	initPID( &pitchAnglePID,
				pitch_configuration[0].pidRange,
				pitch_configuration[0].kP,
				pitch_configuration[0].kI,
				pitch_configuration[0].kD,
				pitch_configuration[0].integralLimit,
				pitch_configuration[0].dLimit );

	initPID( &yawRatePID,
				yaw_configuration[0].pidRange,
				yaw_configuration[0].kP,
				yaw_configuration[0].kI,
				yaw_configuration[0].kD,
				yaw_configuration[0].integralLimit,
				yaw_configuration[0].dLimit );

	for (index = 0; index < ARRAY_SIZE(disarmedMotorValues); index++ )
		disarmedMotorValues[index] = 1000;
}

static bool craftIsArmed = false;
void armCraft( void )
{
	printf("\narmed!");
	craftIsArmed = true;
	STM_EVAL_LEDOn(LED10);
}

void disarmCraft( void )
{
	craftIsArmed = false;
	printf("\ndisarmed");
	STM_EVAL_LEDOff(LED10);
}

static bool isCraftArmed( void )
{
	uint_fast16_t throttleChannel = readReceiverChannel(0);
	uint_fast16_t rollChannel = readReceiverChannel(3);
	static U64 lastChange;
	U64 now;
	bool canChange = false;

	now = CoGetOSTime();
	if ( throttleChannel > 750 && throttleChannel < 1050 && rollChannel > 750 && rollChannel < 1050 )
	{
		if ((now - lastChange) > CFG_SYSTICK_FREQ/2 )
		{
			canChange = true;
			lastChange = now;
		}
	}
	else
		lastChange = now;

	if ( canChange == true )
	{
		if ( craftIsArmed == false )
			armCraft();
		else
			disarmCraft();
	}

	return craftIsArmed;
}

void updatePIDControlLoops( void )
{
	static U64 last_time;
	U64 now;
	uint32_t delta_time;
	float dT;

	// TODO: better time handling
	now = CoGetOSTime();
	delta_time = now - last_time;
	last_time = now;
	dT = (float)delta_time/CFG_SYSTICK_FREQ;

	/* only if armed, and only if throttle is above minimum */
	if ( isCraftArmed() == true && getThrottleSetpoint() > 1050 )
	{
		if ( delta_time > 0 )
		{
			updatePID( &rollAnglePID, RollAngFiltered, getRollAngleSetpoint(), dT );
			updatePID( &pitchAnglePID, PitchAngFiltered, getPitchAngleSetpoint(), dT );
			updatePID( &yawRatePID, filteredGyroValues[2], getYawRateSetpoint(), dT );
		}
	}
	else
	{
		/* reset the PIDs to prevent windup etc */
		resetPID( &rollAnglePID );
		resetPID( &pitchAnglePID );
	}
}

uint16_t getMotorValue( uint_fast8_t motorIndex )
{
	craftType_st const * craft;
	uint16_t motorValue;

	craft = &crafts[0];	// TODO configurable;
	if ( motorIndex < craft->nbMotors)
		motorValue = motorValues[motorIndex];
	else
		motorValue = 0;

	return motorValue;
}

void setMotorDisarmed( uint_fast8_t motorIndex, uint_fast16_t value )
{
	if (motorIndex < ARRAY_SIZE(disarmedMotorValues))
		disarmedMotorValues[motorIndex] = limit( value, 1000, 2000 );
}

float getRollPIDOutput( void )
{
	return rollAnglePID.outputValue;
}

float getPitchPIDOutput( void )
{
	return pitchAnglePID.outputValue;
}

void updateMotorOutputs( void )
{
	unsigned int motorIndex;
	uint_fast16_t maxMotorValue = 0;
	craftType_st const * craft;

	craft = &crafts[0];	// TODO configurable;
	uint_fast16_t tempMotorValues[craft->nbMotors];

	if ( isCraftArmed() )
	{
		motorMixRatios_st const * mixer;

		mixer = craft->motorRatios;
		for ( motorIndex=0; motorIndex < craft->nbMotors; motorIndex++ )
		{
			tempMotorValues[motorIndex] = lrintf(getThrottleSetpoint() * mixer[motorIndex].throttle);
			/* only add in PID control values once throttle is above 0 */
			if ( getThrottleSetpoint() > 1050 )
			{
				tempMotorValues[motorIndex] += lrintf(pitchAnglePID.outputValue * mixer[motorIndex].pitch
								+ rollAnglePID.outputValue * mixer[motorIndex].roll
								+ yawRatePID.outputValue * mixer[motorIndex].yaw);
			}
			if ( maxMotorValue < tempMotorValues[motorIndex] )
				maxMotorValue = tempMotorValues[motorIndex];
		}
	}

	for ( motorIndex=0; motorIndex < craft->nbMotors; motorIndex++ )
	{
		if ( isCraftArmed() )
		{
			if ( maxMotorValue > 2000 )
				tempMotorValues[motorIndex] -= (maxMotorValue - 2000);
		}
		else
		{
			tempMotorValues[motorIndex] = disarmedMotorValues[motorIndex];	// TODO: configurable value
		}
		/* store so that the output value can be displayed */
		motorValues[motorIndex] = limit(tempMotorValues[motorIndex], 1000, 2000);

		/* update the output to the motor esc */
		setMotorOutput( motorIndex, motorValues[motorIndex] );
	}
}


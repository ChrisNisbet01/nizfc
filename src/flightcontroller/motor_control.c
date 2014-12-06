#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <coos.h>
#include <utils.h>
#include <pid.h>
#include <angle_mode_configuration.h>
#include <rate_mode_configuration.h>
#include <yaw_configuration.h>
#include <receiver.h>
#include <receiver_handler.h>
#include <outputs.h>
#include <failsafe.h>
#include <craft_types.h>
#include <aux_configuration.h>
#include <leds.h>

typedef struct motorMixRatios_st
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixRatios_st;

typedef struct craftType_st
{
	uint8_t                 craftType;
    const motorMixRatios_st * motorRatios;
    uint_fast8_t 			nbMotors;
} craftType_st;

#define THROTTLE_POSITION_TO_ENABLE_CONTROL_LOOPS	1050	// TODO: configurable

extern float RollAngle, PitchAngle, Heading;
extern float filteredGyroValues[3];

static pid_st rollAnglePID;
static pid_st pitchAnglePID;
static pid_st rollRatePID;
static pid_st pitchRatePID;
static pid_st yawRatePID;

static const motorMixRatios_st HQuadMixRatios[] =
{
    { .throttle = 1.0f, .roll = -1.0f, .pitch = 1.0f,  .yaw = 1.0f  }, // right rear
    { .throttle = 1.0f, .roll = -1.0f, .pitch = -1.0f, .yaw = -1.0f }, // right front
    { .throttle = 1.0f, .roll = 1.0f,  .pitch = 1.0f,  .yaw = -1.0f }, // left rear
    { .throttle = 1.0f, .roll = 1.0f,  .pitch = -1.0f, .yaw = 1.0f  }, // left front
};

static const craftType_st crafts[] =
{
	{
	.craftType = (uint8_t)craft_type_quadh,
	.motorRatios = HQuadMixRatios,
	.nbMotors = ARRAY_SIZE(HQuadMixRatios)
	}
};

static uint_fast16_t motorValues[6];	// TODO: defined limit
static uint_fast16_t disarmedMotorValues[6];	// TODO: defined limit
static craftType_st const * currentCraft;

static void assignCraftType( craft_type_t craftType )
{
	unsigned int craftIndex;

	/* forget the old craft assignment */
	currentCraft = NULL;

	for (craftIndex = 0; craftIndex < ARRAY_SIZE(crafts); craftIndex++ )
	{
		if ( crafts[craftIndex].craftType == craftType )
		{
			currentCraft = &crafts[craftIndex];
			break;
		}
	}
}

void initMotorControl( craft_type_t craftType )
{
	unsigned int index;

	// TODO: runtime update of PID settings.
	initPID( &rollAnglePID,
				&angle_mode_configuration[0].roll_pidRange,
				&angle_mode_configuration[0].roll_kP,
				&angle_mode_configuration[0].roll_kI,
				&angle_mode_configuration[0].roll_kD,
				&angle_mode_configuration[0].roll_integralLimit,
				&angle_mode_configuration[0].roll_dLimit );

	initPID( &pitchAnglePID,
				&angle_mode_configuration[0].pitch_pidRange,
				&angle_mode_configuration[0].pitch_kP,
				&angle_mode_configuration[0].pitch_kI,
				&angle_mode_configuration[0].pitch_kD,
				&angle_mode_configuration[0].pitch_integralLimit,
				&angle_mode_configuration[0].pitch_dLimit );

	initPID( &rollRatePID,
				&rate_mode_configuration[0].roll_pidRange,
				&rate_mode_configuration[0].roll_kP,
				&rate_mode_configuration[0].roll_kI,
				&rate_mode_configuration[0].roll_kD,
				&rate_mode_configuration[0].roll_integralLimit,
				&rate_mode_configuration[0].roll_dLimit );

	initPID( &pitchRatePID,
				&rate_mode_configuration[0].pitch_pidRange,
				&rate_mode_configuration[0].pitch_kP,
				&rate_mode_configuration[0].pitch_kI,
				&rate_mode_configuration[0].pitch_kD,
				&rate_mode_configuration[0].pitch_integralLimit,
				&rate_mode_configuration[0].pitch_dLimit );

	initPID( &yawRatePID,
				&yaw_configuration[0].pidRange,
				&yaw_configuration[0].kP,
				&yaw_configuration[0].kI,
				&yaw_configuration[0].kD,
				&yaw_configuration[0].integralLimit,
				&yaw_configuration[0].dLimit );

	for (index = 0; index < ARRAY_SIZE(disarmedMotorValues); index++ )
		disarmedMotorValues[index] = 1000;	// TODO: configurable

	assignCraftType( craftType );
	if ( currentCraft != NULL )
	{
		openOutputs(currentCraft->nbMotors);
	}
}

void updatePIDControlLoops( void )
{
	static uint32_t last_time;
	uint32_t now;
	uint32_t delta_time;
	float dT;

	// TODO: better time handling
	now = micros();
	delta_time = now - last_time;
	last_time = now;
	dT = (float)delta_time/1000000.0f;

	/* only if armed, and only if throttle is above minimum */
	if ( isCraftArmed() == true && getThrottleSetpoint() > THROTTLE_POSITION_TO_ENABLE_CONTROL_LOOPS )
	{
		if ( delta_time > 0 )
		{
			updatePID( &pitchAnglePID, PitchAngle, getPitchAngleSetpoint(), dT );
			updatePID( &rollAnglePID, RollAngle, getRollAngleSetpoint(), dT );
			updatePID( &pitchRatePID, -filteredGyroValues[0], getPitchRateSetpoint(), dT );
			updatePID( &rollRatePID, filteredGyroValues[1], getRollRateSetpoint(), dT );
			updatePID( &yawRatePID, filteredGyroValues[2], getYawRateSetpoint(), dT );
		}
	}
	else
	{
		/* reset the PIDs to prevent windup etc */
		resetPID( &rollAnglePID );
		resetPID( &pitchAnglePID );
		resetPID( &rollRatePID );
		resetPID( &pitchRatePID );
		resetPID( &yawRatePID );
	}
}

uint16_t getMotorValue( uint_fast8_t motorIndex )
{
	uint16_t motorValue;

	if ( currentCraft != NULL && motorIndex < currentCraft->nbMotors)
		motorValue = motorValues[motorIndex];
	else
		motorValue = 0;

	return motorValue;
}

void setMotorDisarmedValue( uint_fast8_t motorIndex, uint_fast16_t value )
{
	if (motorIndex < ARRAY_SIZE(disarmedMotorValues))
		disarmedMotorValues[motorIndex] = limit( value, 1000, 2000 );
}

float getRollAnglePIDOutput( void )
{
	return rollAnglePID.outputValue;
}

float getPitchAnglePIDOutput( void )
{
	return pitchAnglePID.outputValue;
}

float getRollRatePIDOutput( void )
{
	return rollRatePID.outputValue;
}

float getPitchRatePIDOutput( void )
{
	return pitchRatePID.outputValue;
}

void updateMotorOutputs( void )
{
	unsigned int motorIndex;
	uint_fast16_t maxMotorValue = 0;
	craftType_st const * craft;

	craft = currentCraft;
	if ( craft != NULL )
	{
		uint_fast16_t tempMotorValues[craft->nbMotors];

		if ( isCraftArmed() && hasFailsafeTriggered() == false )
		{
			motorMixRatios_st const * mixer;

			mixer = craft->motorRatios;
			for ( motorIndex=0; motorIndex < craft->nbMotors; motorIndex++ )
			{
				tempMotorValues[motorIndex] = lrintf(getThrottleSetpoint() * mixer[motorIndex].throttle);
				/* only add in PID control values once throttle is above 0 */
				if ( getThrottleSetpoint() > THROTTLE_POSITION_TO_ENABLE_CONTROL_LOOPS )
				{
					/* If not in rate mode, default to angle mode. Angle mode overrides rate mode */
					// TODO: reset PID state on change of mode
					if (isFunctionEnabled(aux_function_angle_mode) == true || isFunctionEnabled(aux_function_rate_mode) == false)
					{
					setLED(ANGLE_MODE_LED, led_state_on);
					tempMotorValues[motorIndex] += lrintf(
									pitchAnglePID.outputValue * mixer[motorIndex].pitch
									+ rollAnglePID.outputValue * mixer[motorIndex].roll);
					}
					else	/* rate mode */
					{
					setLED(ANGLE_MODE_LED, led_state_off);
					tempMotorValues[motorIndex] += lrintf(
									pitchRatePID.outputValue * mixer[motorIndex].pitch
									+ rollRatePID.outputValue * mixer[motorIndex].roll);
					}
					tempMotorValues[motorIndex] += lrintf(yawRatePID.outputValue * mixer[motorIndex].yaw);

				}
				if ( maxMotorValue < tempMotorValues[motorIndex] )
					maxMotorValue = tempMotorValues[motorIndex];
			}
		}

		for ( motorIndex=0; motorIndex < craft->nbMotors; motorIndex++ )
		{
			if ( hasFailsafeTriggered() == true )
			{
				tempMotorValues[motorIndex] = getFailsafeMotorSpeed();
			}
			else if ( isCraftArmed() )
			{
				if ( maxMotorValue > 2000 )
					tempMotorValues[motorIndex] -= (maxMotorValue - 2000);
			}
			else
			{
				tempMotorValues[motorIndex] = disarmedMotorValues[motorIndex];
			}
			/* store so that the output value can be displayed */
			motorValues[motorIndex] = limit(tempMotorValues[motorIndex], 1000, 2000);	// TODO: configurable limits

			/* update the output to the motor esc */
			setMotorOutput( motorIndex, motorValues[motorIndex] );
		}
	}
}


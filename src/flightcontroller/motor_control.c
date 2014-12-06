#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <coos.h>
#include <utils.h>
#include <pid_control.h>
#include <receiver.h>
#include <receiver_handler.h>
#include <outputs.h>
#include <failsafe.h>
#include <craft_types.h>
#include <aux_configuration.h>

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

	for (index = 0; index < ARRAY_SIZE(disarmedMotorValues); index++ )
		disarmedMotorValues[index] = 1000;	// TODO: configurable

	assignCraftType( craftType );
	if ( currentCraft != NULL )
	{
		openOutputs(currentCraft->nbMotors);
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
					switch ( getCurrentFlightMode() )
					{
						case angle_flight_mode:
							tempMotorValues[motorIndex] += lrintf(getPitchAnglePIDOutput() * mixer[motorIndex].pitch
																	+ getRollAnglePIDOutput() * mixer[motorIndex].roll);
							break;
						case rate_flight_mode:
							tempMotorValues[motorIndex] += lrintf(getPitchRatePIDOutput() * mixer[motorIndex].pitch
																	+ getRollRatePIDOutput() * mixer[motorIndex].roll);
							break;
						default:	/* ??? */
							break;
					}
					tempMotorValues[motorIndex] += lrintf(getYawRatePIDOutput() * mixer[motorIndex].yaw);

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


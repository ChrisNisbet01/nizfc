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
#include <board_configuration.h>

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

#define MAX_MOTORS	4

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

static uint_fast16_t motorOutputs[MAX_MOTORS];			// TODO: defined limit
static uint_fast16_t disarmedMotorOutputs[MAX_MOTORS];	// TODO: defined limit
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

	for (index = 0; index < ARRAY_SIZE(disarmedMotorOutputs); index++ )
		disarmedMotorOutputs[index] = board_configuration[0].minMotorOutput;

	assignCraftType( craftType );
	if ( currentCraft != NULL )
	{
		openOutputs(currentCraft->nbMotors);
	}
}

uint16_t getMotorOutput( uint_fast8_t motorIndex )
{
	uint16_t motorOutput;

	if ( currentCraft != NULL && motorIndex < currentCraft->nbMotors && motorIndex < MAX_MOTORS)
		motorOutput = motorOutputs[motorIndex];
	else
		motorOutput = 0;

	return motorOutput;
}

void setMotorDisarmedValue( uint_fast8_t motorIndex, uint_fast16_t value )
{
	if (motorIndex < ARRAY_SIZE(disarmedMotorOutputs))
		disarmedMotorOutputs[motorIndex] = limit( value, board_configuration[0].minMotorOutput, board_configuration[0].maxMotorOutput );
}

void updateMotorOutputs( void )
{
	unsigned int motorIndex;
	uint_fast16_t maxMotorOutput = 0;
	craftType_st const * craft;

	craft = currentCraft;
	if ( craft != NULL )
	{
		uint_fast16_t tempMotorOutputs[craft->nbMotors];

		if ( isCraftArmed() && hasFailsafeTriggered() == false )
		{
			motorMixRatios_st const * mixer;

			mixer = craft->motorRatios;
			for ( motorIndex=0; motorIndex < craft->nbMotors; motorIndex++ )
			{
				tempMotorOutputs[motorIndex] = lrintf(getThrottleSetpoint() * mixer[motorIndex].throttle);
				/* only add in PID control values once throttle is above 0 */
				if ( getThrottleSetpoint() > THROTTLE_POSITION_TO_ENABLE_CONTROL_LOOPS )
				{
					switch ( getCurrentFlightMode() )
					{
						case angle_flight_mode:
							tempMotorOutputs[motorIndex] += lrintf(getPitchAnglePIDOutput() * mixer[motorIndex].pitch
																	+ getRollAnglePIDOutput() * mixer[motorIndex].roll);
							break;
						case rate_flight_mode:
							tempMotorOutputs[motorIndex] += lrintf(getPitchRatePIDOutput() * mixer[motorIndex].pitch
																	+ getRollRatePIDOutput() * mixer[motorIndex].roll);
							break;
						default:	/* ??? */
							break;
					}
					tempMotorOutputs[motorIndex] += lrintf(getYawRatePIDOutput() * mixer[motorIndex].yaw);

				}
				if ( maxMotorOutput < tempMotorOutputs[motorIndex] )
					maxMotorOutput = tempMotorOutputs[motorIndex];
			}
		}

		for ( motorIndex=0; motorIndex < craft->nbMotors; motorIndex++ )
		{
			if ( hasFailsafeTriggered() == true )
			{
				tempMotorOutputs[motorIndex] = getFailsafeMotorSpeed();
			}
			else if ( isCraftArmed() )
			{
				if ( maxMotorOutput > board_configuration[0].maxMotorOutput )
					tempMotorOutputs[motorIndex] -= (maxMotorOutput - board_configuration[0].maxMotorOutput);
			}
			else
			{
				tempMotorOutputs[motorIndex] = disarmedMotorOutputs[motorIndex];
			}
			/* store so that the output value can be displayed */
			motorOutputs[motorIndex] = limit(tempMotorOutputs[motorIndex],
											board_configuration[0].minMotorOutput,
											board_configuration[0].maxMotorOutput);

			/* update the output to the motor esc */
			setMotorOutput( motorIndex, motorOutputs[motorIndex] );
		}
	}
}


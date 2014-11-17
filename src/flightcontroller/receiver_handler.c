#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f30x_tim.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f3_discovery.h>
#include <coos.h>
#include <utils.h>
#include <receiver.h>
#include <receiver_configuration.h>

typedef struct setpoint_st
{
	float throttle;
	float roll_angle;
	float pitch_angle;
	float yaw_rate;

} setpoint_st;

static setpoint_st setpoints;

static void determineThrottleSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	setpoints.throttle = scale(temp, 1000, 2000, 1000, 2000 );
}

static void determineRollAngleSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	setpoints.roll_angle = scale(temp, 1000, 2000, -50.0f, 50.0f );
}

static void determinePitchAngleSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	// TODO: expo?
	temp = limit( channel, 1000, 2000 );

	setpoints.pitch_angle = scale(temp, 1000, 2000, -50.0f, 50.0f );
}

static void determineYawRateSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	setpoints.yaw_rate = scale(temp, 1000, 2000, 0.0f, 0.0f );
}


void processStickPositions( void )
{
	/* from the various input values, determine things like arming, angle setpoints etc. */
	// TODO: receiver channel mappings. For now, we have TAER
	// TODO: read all channel values in one go.
	determineThrottleSetpoint(readReceiverChannel(0));
	determineRollAngleSetpoint(readReceiverChannel(1));
	determinePitchAngleSetpoint(readReceiverChannel(2));
	determineYawRateSetpoint(readReceiverChannel(3));
	// TODO: Arming, flight mode etc
}

float getThrottleSetpoint( void )
{
	return setpoints.throttle;
}

float getRollAngleSetpoint( void )
{
	return setpoints.roll_angle;
}

float getPitchAngleSetpoint( void )
{
	return setpoints.pitch_angle;
}


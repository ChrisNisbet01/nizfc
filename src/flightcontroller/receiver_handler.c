#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <stm32f30x_tim.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f3_discovery.h>
#include <coos.h>
#include <utils.h>
#include <receiver.h>
#include <receiver_configuration.h>
#include <receiver_handler.h>
#include <roll_pitch_configuration.h>
#include <failsafe.h>

typedef struct setpoint_st
{
	float throttle;
	float roll_angle;
	float pitch_angle;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;

} setpoint_st;

/* TODO: configurable */
#define STICK_MIN_VALID_VALUE	750
#define STICK_LOW_VALUE			1050
#define STICK_MID_LOW_VALUE		1450
#define STICK_MID_HIGH_VALUE	1550
#define STICK_HIGH_VALUE		1950
#define STICK_MAX_VALID_VALUE	2250

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

	setpoints.roll_angle = scale(temp, 1000, 2000, -roll_configuration[0].maxStick, roll_configuration[0].maxStick );
}

static void determinePitchAngleSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	// TODO: expo?
	temp = limit( channel, 1000, 2000 );

	setpoints.pitch_angle = scale(temp, 1000, 2000, -pitch_configuration[0].maxStick, pitch_configuration[0].maxStick );
}

static void determineRollRateSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	setpoints.roll_rate = scale(temp, 1000, 2000, -roll_configuration[1].maxStick, roll_configuration[1].maxStick );
}

static void determinePitchRateSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	// TODO: expo?
	temp = limit( channel, 1000, 2000 );

	setpoints.pitch_rate = scale(temp, 1000, 2000, -pitch_configuration[1].maxStick, pitch_configuration[1].maxStick );
}


static void determineYawRateSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	setpoints.yaw_rate = scale(temp, 1000, 2000, -yaw_configuration[0].maxStick, yaw_configuration[0].maxStick );
}

static void determineArmingState( void )
{
	uint_fast16_t throttleChannel = readReceiverChannel(0);
	uint_fast16_t yawChannel = readReceiverChannel(3);
	static U32 timerStartTime;
	static bool timerRunning = false;
	U32 now;

	now = *CoGetOSTime2();
	if ( throttleChannel > STICK_MIN_VALID_VALUE && throttleChannel < STICK_LOW_VALUE )
	{
		if ( yawChannel > STICK_MIN_VALID_VALUE && yawChannel < STICK_LOW_VALUE )
		{
			/* disarm request */
			if ( timerRunning == false )
			{
				timerRunning = true;
				timerStartTime = now;
			}
			if ((long)(now - timerStartTime) > CFG_SYSTICK_FREQ/2 )
			{
				disarmCraft();
			}
		}
		else if ( yawChannel < STICK_MAX_VALID_VALUE && yawChannel < STICK_HIGH_VALUE )
		{
			if ( timerRunning == false )
			{
				timerRunning = true;
				timerStartTime = now;
			}
			if ((now - timerStartTime) > CFG_SYSTICK_FREQ/2 )
			{
				armCraft();
			}
		}
		else
			timerRunning = false;
	}
	else
		timerRunning = false;
}

void processStickPositions( void )
{
	/* from the various input values, determine things like arming, angle setpoints etc. */
	// TODO: receiver channel mappings. For now, we have TAER
	// TODO: read all channel values in one go.
	determineThrottleSetpoint(readReceiverChannel(0));
	determineRollAngleSetpoint(readReceiverChannel(1));
	determinePitchAngleSetpoint(readReceiverChannel(2));
	determineRollRateSetpoint(readReceiverChannel(1));
	determinePitchRateSetpoint(readReceiverChannel(2));
	determineYawRateSetpoint(readReceiverChannel(3));
	determineArmingState();
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

float getRollRateSetpoint( void )
{
	return setpoints.roll_rate;
}

float getPitchRateSetpoint( void )
{
	return setpoints.pitch_rate;
}

float getYawRateSetpoint( void )
{
	return setpoints.yaw_rate;
}

static bool craftIsArmed = false;
void armCraft( void )
{
	if ( hasFailsafeTriggered() == false )
	{
		printf("\narmed!");
		craftIsArmed = true;
		STM_EVAL_LEDOn(LED10);
	}
}

void disarmCraft( void )
{
	craftIsArmed = false;
	printf("\ndisarmed");
	STM_EVAL_LEDOff(LED10);
}

bool isCraftArmed( void )
{
	return craftIsArmed;
}



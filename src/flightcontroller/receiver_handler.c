#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <coos.h>
#include <utils.h>
#include <receiver.h>
#include <receiver_configuration.h>
#include <receiver_handler.h>
#include <angle_mode_configuration.h>
#include <rate_mode_configuration.h>
#include <yaw_configuration.h>
#include <failsafe.h>
#include <leds.h>

typedef enum aux_switch_position_t
{
	aux_switch_low,
	aux_switch_middle,
	aux_switch_high,
	aux_switch_invalid
} aux_switch_position_t;

typedef struct receiver_state_st
{
	float throttle_position;
	float roll_angle_setpoint;
	float pitch_angle_setpoint;
	float roll_rate_setpoint;
	float pitch_rate_setpoint;
	float yaw_rate_setpoint;

} receiver_state_st;

/* TODO: configurable */
#define STICK_MIN_VALID_VALUE	750
#define STICK_LOW_VALUE			1050
#define STICK_MID_LOW_VALUE		1450
#define STICK_MID_HIGH_VALUE	1550
#define STICK_HIGH_VALUE		1950
#define STICK_MAX_VALID_VALUE	2250

static receiver_state_st receiver_state;

static void determineThrottleSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	receiver_state.throttle_position = scale(temp, 1000, 2000, 1000, 2000 );
}

static void determineRollAngleSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	receiver_state.roll_angle_setpoint = scale(temp, 1000, 2000, -angle_mode_configuration[0].roll_maxAngle, angle_mode_configuration[0].roll_maxAngle );
}

static void determinePitchAngleSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	// TODO: expo?
	temp = limit( channel, 1000, 2000 );

	receiver_state.pitch_angle_setpoint = scale(temp, 1000, 2000, -angle_mode_configuration[0].pitch_maxAngle, angle_mode_configuration[0].pitch_maxAngle );
}

static void determineRollRateSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	receiver_state.roll_rate_setpoint = scale(temp, 1000, 2000, -rate_mode_configuration[0].roll_maxRate, rate_mode_configuration[0].roll_maxRate );
}

static void determinePitchRateSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	// TODO: expo?
	temp = limit( channel, 1000, 2000 );

	receiver_state.pitch_rate_setpoint = scale(temp, 1000, 2000, -rate_mode_configuration[0].pitch_maxRate, rate_mode_configuration[0].pitch_maxRate );
}


static void determineYawRateSetpoint( uint_fast16_t channel )
{
	int temp;
	/* just return a value in the range 1000-2000 */
	// TODO: scale between configured low/high limits
	temp = limit( channel, 1000, 2000 );

	receiver_state.yaw_rate_setpoint = scale(temp, 1000, 2000, yaw_configuration[0].maxRate, -yaw_configuration[0].maxRate );
}

static void determineArmingState( void )
{
	uint_fast16_t throttleChannel = readReceiverChannel(0);
	uint_fast16_t yawChannel = readReceiverChannel(3);
	static U32 timerStartTime;
	static bool timerRunning = false;
	U32 now;

	now = CoGetOSTime32();
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
			if ((long)(now - timerStartTime) > MSTOTICKS(500) )
			{
				disarmCraft();
			}
		}
		else if ( yawChannel < STICK_MAX_VALID_VALUE && yawChannel > STICK_HIGH_VALUE )
		{
			if ( timerRunning == false )
			{
				timerRunning = true;
				timerStartTime = now;
			}
			if ((now - timerStartTime) > MSTOTICKS(500) )
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


void processReceiverSignals( void )
{
	unsigned int aux_switch_channel, aux_switch_index;

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
}

float getThrottleSetpoint( void )
{
	return receiver_state.throttle_position;
}

float getRollAngleSetpoint( void )
{
	return receiver_state.roll_angle_setpoint;
}

float getPitchAngleSetpoint( void )
{
	return receiver_state.pitch_angle_setpoint;
}

float getRollRateSetpoint( void )
{
	return receiver_state.roll_rate_setpoint;
}

float getPitchRateSetpoint( void )
{
	return receiver_state.pitch_rate_setpoint;
}

float getYawRateSetpoint( void )
{
	return receiver_state.yaw_rate_setpoint;
}

static bool craftIsArmed = false;
void armCraft( void )
{
	if ( hasFailsafeTriggered() == false && craftIsArmed == false )
	{
		printf("\narmed!");
		craftIsArmed = true;
		setLED(ARMED_LED, led_state_on);
		enableFailsafe();
	}
}

void disarmCraft( void )
{
	if ( craftIsArmed == true )
	{
		craftIsArmed = false;
		printf("\ndisarmed");
		setLED(ARMED_LED, led_state_off);
		disableFailsafe();
	}
}

bool isCraftArmed( void )
{
	return craftIsArmed;
}



#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <pid_control.h>
#include <pid.h>
#include <hirestimer.h>
#include <receiver_handler.h>
#include <angle_mode_configuration.h>
#include <rate_mode_configuration.h>
#include <yaw_configuration.h>
#include <imu.h>	/* for roll angle etc. Fix this up */

static pid_st rollAnglePID;
static pid_st pitchAnglePID;
static pid_st rollRatePID;
static pid_st pitchRatePID;
static pid_st yawRatePID;

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

float getYawRatePIDOutput( void )
{
	return yawRatePID.outputValue;
}

void resetAngleModePID( void )
{
	resetPID( &rollAnglePID );
	resetPID( &pitchAnglePID );
}

void resetRateModePID( void )
{
	resetPID( &rollRatePID );
	resetPID( &pitchRatePID );
}

void updatePIDControlLoops( void )
{
	static uint32_t last_time;
	uint32_t now;
	uint32_t delta_time;
	float dT;

	now = micros();
	delta_time = now - last_time;
	last_time = now;
	dT = (float)delta_time/1000000.0f;

	/* only if armed, and only if throttle is above minimum */
	if ( isCraftArmed() == true && getThrottleSetpoint() > THROTTLE_POSITION_TO_ENABLE_CONTROL_LOOPS )
	{
		if ( delta_time > 0 )
		{
			// TODO: only update PIDs when appropriate for current flight mode
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

void initPIDControl( void )
{
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
}


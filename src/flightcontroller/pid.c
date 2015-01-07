#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <pid.h>
#include <stdio.h>
#include <utils.h>

void resetPID( pid_st *pid )
{
	pid->integralOffset = 0.0f;
	pid->last_pv = 0.0f;
	pid->nbUpdates = 0;
}

void initPID( pid_st *pid, float *maximumRange, float *kP, float *kI, float *kD, float *intergralLimit, float *dTermLimit )
{
	resetPID( pid );
	pid->maximumRange = maximumRange;
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->integralLimit = intergralLimit;
	pid->dTermLimit = dTermLimit;
	pid->outputValue = 0.0f;
}

void updatePID( pid_st *pid, float pv, float setpoint, float dt )
{
	float error = setpoint - pv;
	float pTerm;
	float iTerm;
	float dTerm;

	pTerm = error * *pid->kP;
	/* limit P term to maximum range? */

	/* I term */
	if ( *pid->kI != 0.0f )
	{
		pid->integralOffset += error * dt * *pid->kI;
		/* ensure that the iterm stays within limits (if specified) */
		if ( *pid->integralLimit != 0.0f )
			pid->integralOffset = limitFloat( pid->integralOffset, -(*pid->integralLimit), *pid->integralLimit );

		iTerm = pid->integralOffset;
	}
	else
		iTerm = 0.0f;

	/* D term. Based upon changes in pv, not changes in error. */
	// TODO: based upon PID mode selection
	if ( *pid->kD != 0.0f )
	{
		if ( pid->nbUpdates > 0 )
		{
			float delta;

			delta = pv - pid->last_pv;
			delta /= dt;	/* account for changes in update frequency */
			// TODO: filter
			dTerm = delta * *pid->kD;
			if ( *pid->dTermLimit != 0.0f )
				dTerm = limitFloat( dTerm, -(*pid->dTermLimit), *pid->dTermLimit );
		}
		else
		{
			pid->nbUpdates++;
			dTerm = 0.0f;
		}
		pid->last_pv = pv;
	}
	else
		dTerm = 0.0f;

	pid->outputValue = pTerm + iTerm - dTerm;
	if ( *pid->maximumRange != 0.0f )
		pid->outputValue = limitFloat( pid->outputValue, -(*pid->maximumRange), *pid->maximumRange );
}

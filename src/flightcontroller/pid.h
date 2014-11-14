#ifndef __PID_H__
#define __PID_H__

typedef struct pid_st
{
	float kP;
	float kI;
	float kD;
	float maximumRange;
	float integralLimit;	/* +- */
	float dTermLimit;		/* +- */	/* limit D term effect */

	float mode;				// TODO: type of PID controller.

	uint_fast8_t	nbUpdates;
	float integralOffset;	/* calculated integral */
	float last_pv;			/* remember pv from pervious update */

	// TODO: D filtering

	float outputValue;
} pid_st;

void initPID( pid_st *pid, float maximumRange, float kP, float kI, float kD, float intergralLimit, float dTermLimit );
void updatePID( pid_st *pid, float pv, float setpoint, float dt );
void resetPID( pid_st *pid );	/* reset I term, D error values etc */

#endif

#ifndef __RATE_MODE_CONFIGURATION_H__
#define __RATE_MODE_CONFIGURATION_H__

#define NB_RATE_MODE_CONFIGURATIONS	1

typedef struct rate_mode_configuration_st
{
	float roll_kP;
	float roll_kI;
	float roll_kD;
	float roll_integralLimit;
	float roll_dLimit;
	float roll_pidRange;
	float roll_maxRate;	/* maximum rate setpoint */
	float pitch_kP;
	float pitch_kI;
	float pitch_kD;
	float pitch_integralLimit;
	float pitch_dLimit;
	float pitch_pidRange;
	float pitch_maxRate;	/* maximum rate setpoint */
} rate_mode_configuration_st;


extern rate_mode_configuration_st rate_mode_configuration[NB_RATE_MODE_CONFIGURATIONS];

#endif /* __RATE_MODE_CONFIGURATION_H__ */


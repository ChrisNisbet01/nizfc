#ifndef __ROLL_PITCH_CONFIGURATION_H__
#define __ROLL_PITCH_CONFIGURATION_H__

#define NB_ROLL_PITCH_CONFIGURATIONS	1

typedef struct roll_pitch_configuration_st
{
	float roll_lpf_factor;
	float pitch_lpf_factor;
	float kP;
	float kI;
	float kD;
	float integralLimit;
	float dLimit;
	float pidRange;
} roll_pitch_configuration_st;


extern roll_pitch_configuration_st roll_pitch_configuration[NB_ROLL_PITCH_CONFIGURATIONS];

#endif /* __ROLL_PITCH_CONFIGURATION_H__ */


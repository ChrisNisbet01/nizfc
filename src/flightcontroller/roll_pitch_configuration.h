#ifndef __ROLL_PITCH_CONFIGURATION_H__
#define __ROLL_PITCH_CONFIGURATION_H__

#define NB_ROLL_CONFIGURATIONS	1
#define NB_PITCH_CONFIGURATIONS	1

typedef struct roll_pitch_configuration_st
{
	float lpf_factor;
	float kP;
	float kI;
	float kD;
	float integralLimit;
	float dLimit;
	float pidRange;
} roll_pitch_configuration_st;


extern roll_pitch_configuration_st roll_configuration[NB_ROLL_CONFIGURATIONS];
extern roll_pitch_configuration_st pitch_configuration[NB_PITCH_CONFIGURATIONS];

#endif /* __ROLL_PITCH_CONFIGURATION_H__ */


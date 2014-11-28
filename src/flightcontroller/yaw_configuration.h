#ifndef __YAW_CONFIGURATION_H__
#define __YAW_CONFIGURATION_H__

#define NB_YAW_CONFIGURATIONS	1

typedef struct yaw_configuration_st
{
	float lpf_factor;
	float kP;
	float kI;
	float kD;
	float integralLimit;
	float dLimit;
	float pidRange;
	float maxRate;
} yaw_configuration_st;


extern yaw_configuration_st yaw_configuration[NB_YAW_CONFIGURATIONS];

#endif /* __YAW_CONFIGURATION_H__ */


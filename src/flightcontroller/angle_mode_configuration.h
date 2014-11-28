#ifndef __ANGLE_MODE_CONFIGURATION_H__
#define __ANGLE_MODE_CONFIGURATION_H__

#define NB_ANGLE_MODE_CONFIGURATIONS	1

typedef struct angle_mode_configuration_st
{
	float roll_kP;
	float roll_kI;
	float roll_kD;
	float roll_integralLimit;
	float roll_dLimit;
	float roll_pidRange;
	float roll_maxAngle;	/* maximum angle setpoint */
	float pitch_kP;
	float pitch_kI;
	float pitch_kD;
	float pitch_integralLimit;
	float pitch_dLimit;
	float pitch_pidRange;
	float pitch_maxAngle;	/* maximum angle setpoint */
} angle_mode_configuration_st;


extern angle_mode_configuration_st angle_mode_configuration[NB_ANGLE_MODE_CONFIGURATIONS];

#endif /* __ANGLE_MODE_CONFIGURATION_H__ */


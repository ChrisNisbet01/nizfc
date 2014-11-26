#ifndef __PWM_INPUTS_H__
#define __PWM_INPUTS_H__

/*
	Hardware agnostic ID values for the PWM input pins.
*/
typedef enum pwm_input_id_t
{
	pwm_input_1,
	pwm_input_2,
	pwm_input_3,
	pwm_input_4,
	pwm_input_5,
	pwm_input_6,
	pwm_input_7,
	pwm_input_8
} pwm_input_id_t;

typedef enum pwm_mode_t
{
	pwm_mode,
	ppm_mode
} pwm_mode_t;

#endif /* __PWM_INPUTS_H__ */

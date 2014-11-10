#ifndef __RECEIVER_CONFIGURATION_H__
#define __RECEIVER_CONFIGURATION_H__

#define NB_RECEIVER_CONFIGURATIONS	1u

typedef enum receiver_mode_type
{
	receiver_mode_pwm = 0,
	receiver_mode_ppm = 1
} receiver_mode_type;

typedef struct receiver_configuration_st
{
	int8_t mode;	/* note that as this is mapped to an enum data type, its size must be 8 bits */
}receiver_configuration_st;


extern receiver_configuration_st receiver_configuration[NB_RECEIVER_CONFIGURATIONS];

#endif /* __RECEIVER_CONFIGURATION_H__ */


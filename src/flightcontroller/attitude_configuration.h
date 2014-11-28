#ifndef __ATTITUDE_CONFIGURATION_H__
#define __ATTITUDE_CONFIGURATION_H__

#define NB_ATTITUDE_CONFIGURATIONS	1

typedef struct attitude_configuration_st
{
	uint16_t roll_lpf;
	uint16_t pitch_lpf;
	uint16_t heading_lpf;
} attitude_configuration_st;


extern attitude_configuration_st attitude_configuration[NB_ATTITUDE_CONFIGURATIONS];

#endif /* __ATTITUDE_CONFIGURATION_H__ */


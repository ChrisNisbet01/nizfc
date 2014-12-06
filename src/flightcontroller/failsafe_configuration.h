#ifndef __FAILSAFE_CONFIGURATION_H__
#define __FAILSAFE_CONFIGURATION_H__

#define NB_FAILSAFE_CONFIGURATIONS	1u

typedef struct failsafe_configuration_st
{
	bool	 enabled;				/* failsafe function enabled */
	uint8_t  nbRequiredChannels;	/* number of channels monitored (sequential, starting at channel 1 */
	uint16_t maxQuietTimeMs;		/* maximum time without all rrequired triggers before triggering failsafe */
	uint16_t motorOutput;			/* motors set to this value if failsafe triggers (still constrained by board minMotor) */
} failsafe_configuration_st;


extern failsafe_configuration_st failsafe_configuration[NB_FAILSAFE_CONFIGURATIONS];

#endif /* __FAILSAFE_CONFIGURATION_H__ */


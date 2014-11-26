#ifndef __BOARD_CONFIGURATION_H__
#define __BOARD_CONFIGURATION_H__

#define NB_BOARD_CONFIGURATIONS	1u

typedef struct board_configuration_st
{
	uint8_t			craftType;
	float  			boardOrientation[3];
	uint32_t 		debug;			/* debug flags */
	uint32_t		updateTime;		/* time between updates of the IMU, motor outputs etc. */
}board_configuration_st;


extern board_configuration_st board_configuration[NB_BOARD_CONFIGURATIONS];

#endif /* __BOARD_CONFIGURATION_H__ */


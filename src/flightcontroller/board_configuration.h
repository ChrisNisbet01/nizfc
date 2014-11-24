#ifndef __BOARD_CONFIGURATION_H__
#define __BOARD_CONFIGURATION_H__

#define NB_BOARD_CONFIGURATIONS	1u

typedef enum board_type_type
{
	board_type_quadx = 0
} board_type_type;

typedef struct board_configuration_st
{
	uint8_t			boardType;

	float  			boardOrientation[3];

	uint32_t 		debug;			/* debug flags */
}board_configuration_st;


extern board_configuration_st board_configuration[NB_BOARD_CONFIGURATIONS];

#endif /* __BOARD_CONFIGURATION_H__ */


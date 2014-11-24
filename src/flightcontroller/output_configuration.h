#ifndef __OUTPUT_CONFIGURATION_H__
#define __OUTPUT_CONFIGURATION_H__

#define NB_OUTPUT_CONFIGURATIONS	1u

typedef struct output_configuration_st
{
	uint16_t pwmrate;
}output_configuration_st;


extern output_configuration_st output_configuration[NB_OUTPUT_CONFIGURATIONS];

#endif /* __OUTPUT_CONFIGURATION_H__ */


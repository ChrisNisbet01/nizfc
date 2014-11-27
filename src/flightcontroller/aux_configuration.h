#ifndef __AUX_CONFIGURATION_H__
#define __AUX_CONFIGURATION_H__

typedef enum aux_function_mappings_t
{
	aux_function_angle_mode = 0,
	aux_function_rate_mode = 1,
	aux_function_max
} aux_function_mappings_t;

bool isFunctionEnabled( aux_function_mappings_t function );
void updateFunctionEnables( void );

#endif /* __AUX_CONFIGURATION_H__ */


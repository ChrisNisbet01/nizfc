#ifndef __AUX_CONFIGURATION_H__
#define __AUX_CONFIGURATION_H__

typedef enum flight_mode_t
{
	invalid_flight_mode,
	angle_flight_mode,
	rate_flight_mode
} flight_mode_t;

typedef enum aux_function_mappings_t
{
	aux_function_angle_mode = 0,
	aux_function_rate_mode = 1,
	aux_function_max
} aux_function_mappings_t;

bool isFunctionEnabled( aux_function_mappings_t function );
void updateFunctionEnables( void );
flight_mode_t getCurrentFlightMode( void );

#endif /* __AUX_CONFIGURATION_H__ */


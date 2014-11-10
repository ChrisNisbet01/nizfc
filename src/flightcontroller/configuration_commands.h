#ifndef __CONFIGURATION_COMMANDS__
#define __CONFIGURATION_COMMANDS__

typedef struct show_config_data_st
{
	run_command_data_st *run_command_data;
	configuration_id_t configuration_id;
	unsigned int instance;
	unsigned int parameter_id;
	config_data_types_t data_type;
	void const * pcfg;				/* pointer to the data available after the current header */
} show_config_data_st;

int saveParameterValues( run_command_data_st const * command_context,
					configuration_id_t configuration_id,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					config_data_point_st const * data_points,
					unsigned int const nb_data_points
					);

int printSavedParameters( void * pv,
					command_st const *commands,
					unsigned int nb_commands,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					config_data_point_st const * data_points,
					unsigned int const nb_data_points,
					char const * const * parameter_name_mappings,
					unsigned int const nb_parameter_name_mappings
					);

#endif /* __CONFIGURATION_COMMANDS__ */

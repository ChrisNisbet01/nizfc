#ifndef __CONFIGURATION_COMMANDS__
#define __CONFIGURATION_COMMANDS__

int saveParameterValues( run_command_data_st const * command_context,
					configuration_id_t configuration_id,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs
					);

int printParametersHandler( void * pv,
					command_st const *commands,
					unsigned int nb_commands,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs,
					char const * const * parameter_name_mappings
					);

int loadParametersHandler( void *pv,
					command_st const *commands,
					unsigned int nb_commands,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs
					);

#endif /* __CONFIGURATION_COMMANDS__ */

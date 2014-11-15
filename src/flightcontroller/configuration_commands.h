#ifndef __CONFIGURATION_COMMANDS__
#define __CONFIGURATION_COMMANDS__

int saveParameterValues( run_command_data_st const * command_context,
					command_st const *commands,
					unsigned int nb_commands
					);

int printParametersHandler( void * pv,
					command_st const *commands,
					unsigned int nb_commands,
					char const * const * parameter_name_mappings
					);

int loadParametersHandler( void *pv,
					command_st const *commands,
					unsigned int nb_commands );

#endif /* __CONFIGURATION_COMMANDS__ */

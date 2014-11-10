#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>
#include <config_structure.h>
#include <polling.h>
#include <cmds.h>
#include <cli.h>
#include <configuration.h>
#include <configuration_commands.h>

static int saveCommand( run_command_data_st *pcommand );
static int showCommand( run_command_data_st *pcommand );

static const command_st config_commands[] =
{
	{ .group_id = configuration_id_save, .name = "save",    .handler = saveCommand	},
	{ .group_id = configuration_id_show, .name = "show",    .handler = showCommand	}
};

typedef enum printConfig_t
{
	print_saved,		/* print anything in the saved config */
	print_current,	/* print anythnig that differs from default */
	print_unsaved,	/* print anything that differs from what we have saved. */
} printConfig_t;

typedef struct show_config_data_st
{
	run_command_data_st *run_command_data;
	printConfig_t		whatToPrint;
	configuration_id_t configuration_id;
	unsigned int instance;
	unsigned int parameter_id;
	config_data_types_t data_type;
	void const * pcfg;				/* pointer to the data available after the current header */
} show_config_data_st;


static void const * findSavedParameter( configuration_id_t const configuration_id, unsigned int const instance, unsigned int const parameter_id )
{
	unsigned int configuration_data_size;
	void const * pcfg = getConfigurationData( &configuration_data_size );
	void const *psaved = NULL;

	if ( pcfg != NULL )
	{
		while( configuration_data_size >= sizeof(uint32_t) )
		{
			configuration_id_t saved_configuration_id;
			unsigned int saved_instance;
			unsigned int saved_parameter_id;
			config_data_types_t saved_data_type;
			int saved_data_length;

			saved_configuration_id = GET_CONFIG_FIELD( *(uint32_t *)pcfg, GROUP );
			saved_instance = GET_CONFIG_FIELD( *(uint32_t *)pcfg, INSTANCE );
			saved_parameter_id = GET_CONFIG_FIELD( *(uint32_t *)pcfg, PARAMETER_ID );
			saved_data_type = GET_CONFIG_FIELD( *(uint32_t *)pcfg, PARAMETER_TYPE );

			if ( saved_configuration_id == configuration_id_reserved )	/* indicates end of config */
				break;

			if ( (saved_data_length = getLengthOfData( saved_data_type, (char *)pcfg + sizeof(uint32_t) )) < 0
				|| (unsigned)saved_data_length > configuration_data_size - sizeof(uint32_t) )
			{
				goto done;
			}

			if (saved_configuration_id == configuration_id && saved_instance == instance && saved_parameter_id == parameter_id )
			{
				psaved = pcfg;
				goto done;	/* found it */
			}

			/* move to next item */
			pcfg = (char *)pcfg + sizeof(uint32_t) + saved_data_length;
			configuration_data_size -= sizeof(uint32_t) + saved_data_length;

		}
	}

done:
	return psaved;
}

static int printUnsavedConfig( run_command_data_st *pcommand )
{
	int result = poll_result_error;
	show_config_data_st show_config_data;

	memset( &show_config_data, 0, sizeof show_config_data );
	show_config_data.run_command_data = pcommand;
	show_config_data.whatToPrint = print_unsaved;

	result = pollCodeGroups( poll_id_show_configuration, &show_config_data, false );

	return result;
}

static int printCurrentConfig( run_command_data_st *pcommand )
{
	int result = poll_result_error;
	show_config_data_st show_config_data;

	memset( &show_config_data, 0, sizeof show_config_data );
	show_config_data.run_command_data = pcommand;
	show_config_data.whatToPrint = print_current;

	result = pollCodeGroups( poll_id_show_configuration, &show_config_data, false );

	return result;
}

static int printSavedConfig( run_command_data_st *pcommand )
{
	int result = poll_result_error;

	unsigned int configuration_data_size;
	void const * pcfg = getConfigurationData( &configuration_data_size );

	if ( pcfg != NULL )
	{
		show_config_data_st show_config_data;

		show_config_data.run_command_data = pcommand;
		show_config_data.whatToPrint = print_saved;
		show_config_data.pcfg = pcfg;
		while( configuration_data_size >= sizeof(uint32_t) )
		{
			int data_length;

			show_config_data.configuration_id = GET_CONFIG_FIELD( *(uint32_t *)pcfg, GROUP );
			show_config_data.instance = GET_CONFIG_FIELD( *(uint32_t *)pcfg, INSTANCE );
			show_config_data.parameter_id = GET_CONFIG_FIELD( *(uint32_t *)pcfg, PARAMETER_ID );
			show_config_data.data_type = GET_CONFIG_FIELD( *(uint32_t *)pcfg, PARAMETER_TYPE );

			if ( show_config_data.configuration_id == configuration_id_reserved ) /* indicates end of config */
				break;
			show_config_data.pcfg = (char *)show_config_data.pcfg + sizeof(uint32_t);
			configuration_data_size -= sizeof(uint32_t);

			if ( (data_length = getLengthOfData( show_config_data.data_type, pcfg )) < 0 || (unsigned)data_length > configuration_data_size )
			{
				cliPrintf( pcommand->cliCtx, "\nError processing configuration item" );
				cliPrintf( pcommand->cliCtx, "\ng:%d i:%d p:%d",
							show_config_data.configuration_id,
							show_config_data.instance,
							show_config_data.parameter_id );
				break;
			}

			if ( pollCodeGroups( poll_id_show_configuration, &show_config_data, false ) != poll_result_ok )
			{
				cliPrintf( pcommand->cliCtx, "\nUnprocessed configuration item" );
				cliPrintf( pcommand->cliCtx, "\ng:%d i:%d p:%d",
							show_config_data.configuration_id,
							show_config_data.instance,
							show_config_data.parameter_id );
			}

			/* move to next item */
			show_config_data.pcfg = (char *)show_config_data.pcfg + data_length;
			configuration_data_size -= data_length;

		}

		result = poll_result_ok;
	}
	else
		cliPrintf( pcommand->cliCtx, "\nSaved configuration is invalid" );

	return result;
}

static int showCommand( run_command_data_st *pcommand )
{
	int result = poll_result_error;
	int argc = pcommand->argc;
	char * * argv = pcommand->argv;

	if ( argc > 1 )
	{
		if ( strncasecmp( argv[1], "saved", strlen(argv[1]) ) == 0 )
			result = printSavedConfig( pcommand );
		else if ( strncasecmp( argv[1], "current", strlen(argv[1]) ) == 0 )
			result = printCurrentConfig( pcommand );
		else if ( strncasecmp( argv[1], "unsaved", strlen(argv[1]) ) == 0 )
			result = printUnsavedConfig( pcommand );
	}

	if ( result == poll_result_error )
	{
		cliPrintf( pcommand->cliCtx, "\nFormat: %s ?                                - show help for this command", argv[0] );
		cliPrintf( pcommand->cliCtx, "\nFormat: %s <saved|current|unsaved>          - show configuration", argv[0] );
	}
	return result;
}

/*
	saveCommand:
	A command to save all configuration.
	A save_config poll is made, which should result in all supporintg code calling the save_config
	function for all of their configuration data.
*/
static int saveCommand( run_command_data_st *pcommand )
{
	int result = -1;

	if ( initConfigurationSave() == false )
	{
		cliPrintf( pcommand->cliCtx, "\nError initialising config save");
		goto done;
	}

	pollCodeGroups( poll_id_save_configuration, pcommand, 1 );

	if ( completeConfigurationSave() == false )
	{
		cliPrintf( pcommand->cliCtx, "\nError completing config save");
		goto done;
	}

	cliPrintf( pcommand->cliCtx, "Configuration size: %d bytes", getConfigurationSize() );

	result = 0;

done:

	return result;
}

static bool writeParameterToFlash( configuration_id_t configuration_id,
								unsigned int instance,
								void const * pcfg,
								parameterConfig_st const * parameterConfig )
{
	bool savedParameter = false;
	int data_length;
	uint32_t data_header = 0;

	if ( (data_length = getLengthOfData( parameterConfig->data_type, (char *)pcfg + parameterConfig->offsetToData )) < 0 )
		goto done;

	data_header |= MAKE_CONFIG_FIELD_VALUE(configuration_id, GROUP);
	data_header |= MAKE_CONFIG_FIELD_VALUE(instance, INSTANCE);
	data_header |= MAKE_CONFIG_FIELD_VALUE(parameterConfig->parameter_id, PARAMETER_ID);
	data_header |= MAKE_CONFIG_FIELD_VALUE(parameterConfig->data_type, PARAMETER_TYPE);

	if ( saveConfigurationData( &data_header, sizeof(data_header) ) == false )
		goto done;

	if ( saveConfigurationData( (char *)pcfg + parameterConfig->offsetToData, data_length ) == false )
		goto done;

	savedParameter = true;

done:

	return savedParameter;
}

int saveParameterValues( run_command_data_st const * command_context,
					configuration_id_t configuration_id,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs
					)
{
	/*
		For each data point in each configuration, we write out the current value,
		but only if it differs from the default value.
	*/
	UNUSED(command_context);
	int result = poll_result_error;
	unsigned int configuration_index, parameterConfig_index;

	for ( configuration_index = 0; configuration_index < nb_configurations; configuration_index++ )
	{
		unsigned int offset_to_configuration_data = (configuration_index*configuration_size);

		for ( parameterConfig_index = 0; parameterConfig_index < nbParameterConfigs; parameterConfig_index++ )
		{
			if (currentParameterValueMatchesDefaultValue( (char *)pcfg + offset_to_configuration_data + parameterConfigs[parameterConfig_index].offsetToData,
															default_configuration,
															&parameterConfigs[parameterConfig_index] ) == false )
			{
				if (writeParameterToFlash( configuration_id,
									configuration_index,
									(char *)pcfg + offset_to_configuration_data,
									&parameterConfigs[parameterConfig_index] ) == false)
				{
					goto done;
				}
			}
		}
	}

	result = poll_result_ok;

done:

	return result;
}

static int printSavedParameters( void *pv,
					command_st const *commands,
					unsigned int nb_commands,
					unsigned int const nb_configurations,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs,
					char const * const * parameter_name_mappings
					)
{
	show_config_data_st * show_config_data = pv;
	run_command_data_st *run_command_data = show_config_data->run_command_data;
	int result = poll_result_error;
	command_st const *command;

	if (  (command = findCommandFromID( commands,
										nb_commands,
										show_config_data->configuration_id )) != NULL )
	{
		if ( show_config_data->instance < nb_configurations )
		{
			parameterConfig_st const * parameterConfig;

			if ( (parameterConfig=findDataPointFromParameterID( parameterConfigs,
															nbParameterConfigs,
															show_config_data->parameter_id )) != NULL )
			{
				/* check that the data types match */
				if ( parameterConfig->data_type == show_config_data->data_type )
				{
					cliPrintf( run_command_data->cliCtx,
								"\n%s %d %s ",
								command->name,
								show_config_data->instance,
								parameter_name_mappings[show_config_data->parameter_id]);
					printParameterValue( show_config_data->pcfg,
													parameterConfig,
													run_command_data->cliCtx );
				}
				result = poll_result_ok;
			}
		}
	}

	return result;
}

static int printCurrentParameters( void *pv,
					command_st const *commands,
					unsigned int nb_commands,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs,
					char const * const * parameter_name_mappings
					)
{
	show_config_data_st * show_config_data = pv;
	run_command_data_st *run_command_data = show_config_data->run_command_data;
	int result = poll_result_ok;
	unsigned int command_index;

	for (command_index = 0; command_index < nb_commands; command_index++)
	{
		unsigned int instance;

		for (instance = 0; instance < nb_configurations; instance++)
		{
			unsigned int parameterConfig_instance;
			unsigned int offset_to_configuration_data = (instance*configuration_size);

			for (parameterConfig_instance = 0; parameterConfig_instance < nbParameterConfigs; parameterConfig_instance++)
			{
				void *parameter = (char *)pcfg + offset_to_configuration_data + parameterConfigs[parameterConfig_instance].offsetToData;

				if (currentParameterValueMatchesDefaultValue(parameter, default_configuration, &parameterConfigs[parameterConfig_instance] ) == false)
				{
					cliPrintf( run_command_data->cliCtx,
								"\n%s %d %s ",
								commands[command_index].name,
								instance,
								parameter_name_mappings[parameterConfigs[parameterConfig_instance].parameter_id]);
					printParameterValue( parameter,
											&parameterConfigs[parameterConfig_instance],
											run_command_data->cliCtx );
				}
			}
		}
	}

	return result;
}

static int printUnsavedParameters( void *pv,
					command_st const *commands,
					unsigned int nb_commands,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs,
					char const * const * parameter_name_mappings
					)
{
	show_config_data_st * show_config_data = pv;
	run_command_data_st *run_command_data = show_config_data->run_command_data;
	int result = poll_result_ok;
	unsigned int command_index;

	for (command_index = 0; command_index < nb_commands; command_index++)
	{
		unsigned int instance;

		for (instance = 0; instance < nb_configurations; instance++)
		{
			unsigned int parameterConfig_instance;
			unsigned int offset_to_configuration_data = (instance*configuration_size);

			for (parameterConfig_instance = 0; parameterConfig_instance < nbParameterConfigs; parameterConfig_instance++)
			{
				void const * parameter = (char *)pcfg + offset_to_configuration_data + parameterConfigs[parameterConfig_instance].offsetToData;
				void const * psaved = findSavedParameter( commands[command_index].group_id, instance, parameterConfigs[parameterConfig_instance].parameter_id );
				bool printParameter = false;
				/*
					if in saved config and value doesn't match, or if non-default and not in saved config
				*/
				if ( psaved == NULL )
				{
					if (currentParameterValueMatchesDefaultValue(parameter, default_configuration, &parameterConfigs[parameterConfig_instance] ) == false)
						printParameter = true;
				}
				else if (savedParameterValueMatchesCurrentValue( psaved, parameter, &parameterConfigs[parameterConfig_instance] ) == false)
				{
					printParameter = true;
				}
				if ( printParameter == true )
				{
					cliPrintf( run_command_data->cliCtx,
								"\n%s %d %s ",
								commands[command_index].name,
								instance,
								parameter_name_mappings[parameterConfigs[parameterConfig_instance].parameter_id]);
					printParameterValue( parameter,
											&parameterConfigs[parameterConfig_instance],
											run_command_data->cliCtx );
				}
			}
		}
	}

	return result;
}


int printParametersHandler( void *pv,
					command_st const *commands,
					unsigned int nb_commands,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs,
					char const * const * parameter_name_mappings)
{
	show_config_data_st * show_config_data = pv;
	int result = poll_result_error;

	switch (show_config_data->whatToPrint)
	{
		case print_saved:
			result = printSavedParameters( pv,
											commands,
											nb_commands,
											nb_configurations,
											parameterConfigs,
											nbParameterConfigs,
											parameter_name_mappings );
			break;
		case print_current:	/* anything that isn't default */
			result = printCurrentParameters( pv,
												commands,
												nb_commands,
												pcfg,
												nb_configurations,
												configuration_size,
												default_configuration,
												parameterConfigs,
												nbParameterConfigs,
												parameter_name_mappings );
			break;
		case print_unsaved:	/* unsaved changes */
			result = printUnsavedParameters( pv,
												commands,
												nb_commands,
												pcfg,
												nb_configurations,
												configuration_size,
												default_configuration,
												parameterConfigs,
												nbParameterConfigs,
												parameter_name_mappings );
		default:
			break;
	}

	return result;
}

typedef struct load_config_data_st
{
	configuration_id_t configuration_id;
	unsigned int instance;
	unsigned int parameter_id;
	config_data_types_t data_type;
	void const * pcfg;				/* pointer to the data available after the current header */
} load_config_data_st;

int loadParametersHandler( void *pv,
					command_st const *commands,
					unsigned int nb_commands,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					parameterConfig_st const * parameterConfigs,
					unsigned int const nbParameterConfigs
					)
{
	load_config_data_st * load_config_data = pv;
	int result = poll_result_error;
	command_st const *command;

	if (  (command = findCommandFromID( commands,
										nb_commands,
										load_config_data->configuration_id )) != NULL )
	{
		if ( load_config_data->instance < nb_configurations )
		{
			parameterConfig_st const * parameterConfig;

			if ( (parameterConfig=findDataPointFromParameterID( parameterConfigs,
															nbParameterConfigs,
															load_config_data->parameter_id )) != NULL )
			{
				unsigned int offset_to_configuration_data = (load_config_data->instance*configuration_size);
				void * const config_instance = (char *)pcfg + offset_to_configuration_data;

				assignSavedParameterValue( load_config_data->pcfg,
											load_config_data->data_type,
											config_instance + parameterConfig->offsetToData,
											parameterConfig );
				result = poll_result_ok;
			}
		}
	}

	return result;
}

void loadSavedConfiguration( void )
{
	unsigned int configuration_data_size;
	void const * pcfg = getConfigurationData( &configuration_data_size );

	if ( pcfg != NULL )
	{
		load_config_data_st load_config_data;

		load_config_data.pcfg = pcfg;
		while( configuration_data_size >= sizeof(uint32_t) )
		{
			int data_length;

			load_config_data.configuration_id = GET_CONFIG_FIELD( *(uint32_t *)pcfg, GROUP );
			load_config_data.instance = GET_CONFIG_FIELD( *(uint32_t *)pcfg, INSTANCE );
			load_config_data.parameter_id = GET_CONFIG_FIELD( *(uint32_t *)pcfg, PARAMETER_ID );
			load_config_data.data_type = GET_CONFIG_FIELD( *(uint32_t *)pcfg, PARAMETER_TYPE );

			if ( load_config_data.configuration_id == configuration_id_reserved ) /* indicates end of config */
				break;

			load_config_data.pcfg = (char *)load_config_data.pcfg + sizeof(uint32_t);
			configuration_data_size -= sizeof(uint32_t);

			if ( (data_length = getLengthOfData( load_config_data.data_type, pcfg )) < 0 || (unsigned)data_length > configuration_data_size )
			{
				break;
			}

			if ( pollCodeGroups( poll_id_load_configuration, &load_config_data, false ) != poll_result_ok )
			{
				/* unprocessed saved value. */
			}

			/* move to next item */
			load_config_data.pcfg = (char *)load_config_data.pcfg + data_length;
			configuration_data_size -= data_length;

		}
	}
}

void initialiseCodeGroups( void )
{
	pollCodeGroups( poll_id_initialise, NULL, 1 );
}


int configPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_run_command:
		{
			result = runCommandHandler( config_commands, ARRAY_SIZE(config_commands), pv );
			break;
		}
		default:
			break;
	}

	return result;
}



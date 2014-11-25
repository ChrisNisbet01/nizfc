#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <utils.h>
#include <config_structure.h>
#include <polling.h>
#include <cmds.h>
#include <cli.h>
#include <configuration.h>
#include <configuration_commands.h>
#include <stm32f30x_misc.h>
#include <motor_control.h>

static int saveCommand( non_config_run_command_data_st *pcommand );
static int showCommand( non_config_run_command_data_st *pcommand );
static int helpCommand( non_config_run_command_data_st *pcommand );
static int rebootCommand( non_config_run_command_data_st *pcommand );
static int armCommand( non_config_run_command_data_st *pcommand );
static int disarmCommand( non_config_run_command_data_st *pcommand );
static int debugCommand( non_config_run_command_data_st *pcommand );

static const non_config_command_st config_commands[] =
{
	{ .name = "save",    .handler = saveCommand	},
	{ .name = "show",    .handler = showCommand	},
	{ .name = "?",       .handler = helpCommand	},
	{ .name = "help",    .handler = helpCommand	},
	{ .name = "reboot",  .handler = rebootCommand	},
	{ .name = "arm",     .handler = armCommand	},
	{ .name = "disarm",  .handler = disarmCommand	},
	{ .name = "debug",   .handler = debugCommand	}
};

typedef enum printConfig_t
{
	print_saved,		/* print anything in the saved config */
	print_current,		/* print anything that differs from default */
	print_all,			/* print all parameters */
	print_unsaved,		/* print anything that differs from what we have saved. */
} printConfig_t;

typedef struct show_config_data_st
{
	non_config_run_command_data_st *run_command_data;
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

static int printUnsavedConfig( non_config_run_command_data_st *pcommand )
{
	int result;
	show_config_data_st show_config_data;

	memset( &show_config_data, 0, sizeof show_config_data );
	show_config_data.run_command_data = pcommand;
	show_config_data.whatToPrint = print_unsaved;

	result = pollCodeGroups( poll_id_show_configuration, &show_config_data, true );

	return result;
}

static int printCurrentConfig( non_config_run_command_data_st *pcommand, printConfig_t whatToPrint )
{
	int result;
	show_config_data_st show_config_data;

	memset( &show_config_data, 0, sizeof show_config_data );
	show_config_data.run_command_data = pcommand;
	show_config_data.whatToPrint = whatToPrint;

	result = pollCodeGroups( poll_id_show_configuration, &show_config_data, true );

	return result;
}

static int printSavedConfig( non_config_run_command_data_st *pcommand )
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

			show_config_data.configuration_id = GET_CONFIG_FIELD( *(uint32_t *)show_config_data.pcfg, GROUP );
			show_config_data.instance = GET_CONFIG_FIELD( *(uint32_t *)show_config_data.pcfg, INSTANCE );
			show_config_data.parameter_id = GET_CONFIG_FIELD( *(uint32_t *)show_config_data.pcfg, PARAMETER_ID );
			show_config_data.data_type = GET_CONFIG_FIELD( *(uint32_t *)show_config_data.pcfg, PARAMETER_TYPE );

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

			/* poll the item arount to everybody */
			(void)pollCodeGroups( poll_id_show_configuration, &show_config_data, true );

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

void systemReset(void)
{
	NVIC_SystemReset();
}

static int rebootCommand( non_config_run_command_data_st *pcommand )
{
	UNUSED(pcommand);

	systemReset();

	return poll_result_ok;
}

static int armCommand( non_config_run_command_data_st *pcommand )
{
	UNUSED(pcommand);

	armCraft();

	return poll_result_ok;
}

static int disarmCommand( non_config_run_command_data_st *pcommand )
{
	UNUSED(pcommand);

	disarmCraft();

	return poll_result_ok;
}

static int debugCommand( non_config_run_command_data_st *pcommand )
{
	bool show_usage = true;

	if ( pcommand->argc >= 1 )
	{
		unsigned int port;

		if ( strncasecmp( pcommand->argv[1], "off", strlen( pcommand->argv[1] ) ) == 0 )
			show_usage = setDebugPort( -1 ) == false;	/* XXX use invalid port number */
		else if ( strtoint( pcommand->argv[1], &port ) )
			show_usage = setDebugPort( port ) == false;
	}

	if ( show_usage == true )
	{
		cliPrintf( pcommand->cliCtx, "Format: %s off|0|1", pcommand->command->name );
	}

	return poll_result_ok;
}

static int helpCommand( non_config_run_command_data_st *pcommand )
{
	help_command_data_st help_command;

	help_command.argc = pcommand->argc;
	help_command.argv = pcommand->argv;
	help_command.cliCtx = pcommand->cliCtx;

	pollCodeGroups( poll_id_identify, &help_command, true );
	pollCodeGroups( poll_id_identify_non_config, &help_command, true );

	return poll_result_ok;
}

static int showCommand( non_config_run_command_data_st *pcommand )
{
	int result = poll_result_error;
	int argc = pcommand->argc;
	char * * argv = pcommand->argv;

	if ( argc > 1 )
	{
		if ( strncasecmp( argv[1], "saved", strlen(argv[1]) ) == 0 )
			result = printSavedConfig( pcommand );
		else if ( strncasecmp( argv[1], "current", strlen(argv[1]) ) == 0 )
			result = printCurrentConfig( pcommand, print_current );
		else if ( strncasecmp( argv[1], "all", strlen(argv[1]) ) == 0 )
			result = printCurrentConfig( pcommand, print_all );
		else if ( strncasecmp( argv[1], "unsaved", strlen(argv[1]) ) == 0 )
			result = printUnsavedConfig( pcommand );
	}

	if ( result == poll_result_error )
	{
		cliPrintf( pcommand->cliCtx, "\nFormat: %s ?                                - show help for this command", pcommand->command->name );
		cliPrintf( pcommand->cliCtx, "\nFormat: %s <saved|current|unsaved|all>      - show configuration", pcommand->command->name );
	}
	return result;
}

/*
	saveCommand:
	A command to save all configuration.
	A save_config poll is made, which should result in all supporintg code calling the save_config
	function for all of their configuration data.
*/
static int saveCommand( non_config_run_command_data_st *pcommand )
{
	int result = -1;

	if ( initConfigurationSave() == false )
	{
		cliPrintf( pcommand->cliCtx, "\nError initialising config save");
		goto done;
	}

	pollCodeGroups( poll_id_save_configuration, pcommand, true );

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
					command_st const *commands,
					unsigned int nb_commands
					)
{
	/*
		For each data point in each configuration, we write out the current value,
		but only if it differs from the default value.
	*/
	UNUSED(command_context);
	int result = poll_result_error;
	unsigned int configuration_index, parameterConfig_index;
	unsigned int command_index;

	for (command_index = 0; command_index < nb_commands; command_index++ )
	{
		command_st const *command = &commands[command_index];

		for ( configuration_index = 0; configuration_index < command->nb_configuration_instance; configuration_index++ )
		{
			unsigned int offset_to_configuration_data = (configuration_index * command->configuration_size);

			for ( parameterConfig_index = 0; parameterConfig_index < command->nbParameterConfigs; parameterConfig_index++ )
			{
				if (currentParameterValueMatchesDefaultValue( (char *)command->configuration + offset_to_configuration_data + command->parameterConfigs[parameterConfig_index].offsetToData,
																command->default_configuration,
																&command->parameterConfigs[parameterConfig_index] ) == false )
				{
					if (writeParameterToFlash( command->group_id,
										configuration_index,
										(char *)command->configuration + offset_to_configuration_data,
										&command->parameterConfigs[parameterConfig_index] ) == false)
					{
						goto done;
					}
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
					char const * const * parameter_name_mappings
					)
{
	show_config_data_st * show_config_data = pv;
	non_config_run_command_data_st *run_command_data = show_config_data->run_command_data;
	int result = poll_result_error;
	command_st const *command;

	if (  (command = findCommandFromID( commands,
										nb_commands,
										show_config_data->configuration_id )) != NULL )
	{
		if ( show_config_data->instance < command->nb_configuration_instance )
		{
			parameterConfig_st const * parameterConfig;

			if ( (parameterConfig=findDataPointFromParameterID( command->parameterConfigs,
															command->nbParameterConfigs,
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
					char const * const * parameter_name_mappings,
					printConfig_t whatToPrint
					)
{
	show_config_data_st * show_config_data = pv;
	non_config_run_command_data_st *run_command_data = show_config_data->run_command_data;
	int result = poll_result_ok;
	unsigned int command_index;

	for (command_index = 0; command_index < nb_commands; command_index++)
	{
		command_st const *command = &commands[command_index];
		unsigned int instance;

		for (instance = 0; instance < command->nb_configuration_instance; instance++)
		{
			unsigned int parameterConfig_instance;
			unsigned int offset_to_configuration_data = (instance * command->configuration_size);

			for (parameterConfig_instance = 0; parameterConfig_instance < command->nbParameterConfigs; parameterConfig_instance++)
			{
				void *parameter = (char *)command->configuration + offset_to_configuration_data + command->parameterConfigs[parameterConfig_instance].offsetToData;

				if ( whatToPrint == print_all
					|| currentParameterValueMatchesDefaultValue(parameter, command->default_configuration, &command->parameterConfigs[parameterConfig_instance] ) == false)
				{
					cliPrintf( run_command_data->cliCtx,
								"\n%s %d %s ",
								commands[command_index].name,
								instance,
								parameter_name_mappings[command->parameterConfigs[parameterConfig_instance].parameter_id]);
					printParameterValue( parameter,
											&command->parameterConfigs[parameterConfig_instance],
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
					char const * const * parameter_name_mappings
					)
{
	show_config_data_st * show_config_data = pv;
	non_config_run_command_data_st *run_command_data = show_config_data->run_command_data;
	int result = poll_result_ok;
	unsigned int command_index;

	for (command_index = 0; command_index < nb_commands; command_index++)
	{
		command_st const *command = &commands[command_index];
		unsigned int instance;

		for (instance = 0; instance < command->nb_configuration_instance; instance++)
		{
			unsigned int parameterConfig_instance;
			unsigned int offset_to_configuration_data = (instance * command->configuration_size);

			for (parameterConfig_instance = 0; parameterConfig_instance < command->nbParameterConfigs; parameterConfig_instance++)
			{
				void const * parameter = (char *)command->configuration + offset_to_configuration_data + command->parameterConfigs[parameterConfig_instance].offsetToData;
				void const * psaved = findSavedParameter( commands[command_index].group_id, instance, command->parameterConfigs[parameterConfig_instance].parameter_id );
				bool printParameter = false;
				/*
					if in saved config and value doesn't match, or if non-default and not in saved config
				*/
				if ( psaved == NULL )
				{
					if (currentParameterValueMatchesDefaultValue(parameter, command->default_configuration, &command->parameterConfigs[parameterConfig_instance] ) == false)
						printParameter = true;
				}
				else if (savedParameterValueMatchesCurrentValue( psaved, parameter, &command->parameterConfigs[parameterConfig_instance] ) == false)
				{
					printParameter = true;
				}
				if ( printParameter == true )
				{
					cliPrintf( run_command_data->cliCtx,
								"\n%s %d %s ",
								commands[command_index].name,
								instance,
								parameter_name_mappings[command->parameterConfigs[parameterConfig_instance].parameter_id]);
					printParameterValue( parameter,
											&command->parameterConfigs[parameterConfig_instance],
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
											parameter_name_mappings );
			break;
		case print_current:
		case print_all:
			result = printCurrentParameters( pv,
												commands,
												nb_commands,
												parameter_name_mappings,
												show_config_data->whatToPrint );
			break;
		case print_unsaved:	/* unsaved changes */
			result = printUnsavedParameters( pv,
												commands,
												nb_commands,
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
					unsigned int nb_commands )
{
	load_config_data_st * load_config_data = pv;
	int result = poll_result_error;
	command_st const *command;

	if (  (command = findCommandFromID( commands,
										nb_commands,
										load_config_data->configuration_id )) != NULL )
	{
		if ( load_config_data->instance < command->nb_configuration_instance )
		{
			parameterConfig_st const * parameterConfig;

			if ( (parameterConfig=findDataPointFromParameterID( command->parameterConfigs,
															command->nbParameterConfigs,
															load_config_data->parameter_id )) != NULL )
			{
				unsigned int offset_to_configuration_data = (load_config_data->instance * command->configuration_size);
				void * const config_instance = (char *)command->configuration + offset_to_configuration_data;

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
			uint32_t hdr = *(uint32_t *)load_config_data.pcfg;

			load_config_data.configuration_id = GET_CONFIG_FIELD( hdr, GROUP );
			load_config_data.instance = GET_CONFIG_FIELD( hdr, INSTANCE );
			load_config_data.parameter_id = GET_CONFIG_FIELD( hdr, PARAMETER_ID );
			load_config_data.data_type = GET_CONFIG_FIELD( hdr, PARAMETER_TYPE );

			//printf("\nid %d inst %d id %d type %d", load_config_data.configuration_id, load_config_data.instance, load_config_data.parameter_id, load_config_data.data_type );
			if ( load_config_data.configuration_id == configuration_id_reserved ) /* indicates end of config */
				break;

			load_config_data.pcfg = (char *)load_config_data.pcfg + sizeof(uint32_t);
			configuration_data_size -= sizeof(uint32_t);

			if ( (data_length = getLengthOfData( load_config_data.data_type, pcfg )) < 0 || (unsigned)data_length > configuration_data_size )
			{
				break;
			}

			if ( pollCodeGroups( poll_id_load_configuration, &load_config_data, true ) != poll_result_ok )
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
	pollCodeGroups( poll_id_initialise, NULL, true );
}


int configPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify_non_config:
		{
			idNonConfigCommandHandler( config_commands, ARRAY_SIZE(config_commands), pv );
			result = poll_result_ok;
			break;
		}
		case poll_id_run_non_config_command:
		{
			result = runNonConfigCommandHandler( config_commands, ARRAY_SIZE(config_commands), pv );
			break;
		}
		default:
			break;
	}

	return result;
}



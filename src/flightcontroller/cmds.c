#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <utils.h>
#include <configuration.h>
#include <cmds.h>
#include <configuration_commands.h>
#include <polling.h>
#include <cli.h>

static command_st const * findCommand( command_st const * commands, uint32_t nb_commands, char const * name )
{
	uint32_t index;

	for (index = 0; index < nb_commands; index++)
	{
		if ( strncasecmp(name, commands[index].name, strlen(name) ) == 0 )
			return &commands[index];
	}

	return NULL;
}

static non_config_command_st const * findNonConfigCommand( non_config_command_st const * commands, uint32_t nb_commands, char const * name )
{
	uint32_t index;

	for (index = 0; index < nb_commands; index++)
	{
		if ( strncasecmp(name, commands[index].name, strlen(name) ) == 0 )
			return &commands[index];
	}

	return NULL;
}

static void printParameterName( parameterConfig_st const * parameterConfig, ParameterNameLookup ParameterNameLookupCB, void *cliCtx )
{
	cliPrintf( cliCtx, "%20s", ParameterNameLookupCB(parameterConfig->parameter_id));
}

static void printParameterNames( parameterConfig_st const * parameterConfigs, unsigned int nbParameterConfigs, ParameterNameLookup ParameterNameLookupCB, void *cliCtx )
{
	unsigned int index;

	cliPrintf( cliCtx, "\r\nparameters are:" );

	for (index=0; index < nbParameterConfigs; index++ )
	{
		cliPrintf( cliCtx, "\n" );
		printParameterName( &parameterConfigs[index], ParameterNameLookupCB, cliCtx );
	}
}

static char const *lookupEnumValue( int8_t value, enum_mapping_st const * mappings, uint_fast8_t nb_mappings )
{
	uint_fast8_t index;

	for (index=0; index < nb_mappings; index++)
	{
		if (value == mappings[index].value)
			return mappings[index].name;
	}

	/* not found in mappings */
	return NULL;
}

static bool lookupEnumValueByName( char const * const name, enum_mapping_st const const * const mappings, uint_fast8_t const nb_mappings, uint8_t * const enum_value )
{
	uint_fast8_t index;

	for (index=0; index < nb_mappings; index++)
	{
		if (strncasecmp(name, mappings[index].name, strlen(name)) == 0)
		{
			*enum_value = mappings[index].value;
			return true;
		}
	}

	/* not found in mappings */
	return false;
}


void printParameterValue( void const * pconfig_data,
											parameterConfig_st const * parameterConfig,
											void *cliCtx )
{

	switch( parameterConfig->data_type )
	{
		case config_data_type_boolean:
		{
			int8_t value = *(int8_t *)pconfig_data;

			cliPrintf(cliCtx, "%s", (value != 0) ? "on" : "off");
			break;
		}
		case config_data_type_int8:
		{
			int value = *(int8_t *)pconfig_data;

			cliPrintf(cliCtx, "%d", value);
			break;
		}
		case config_data_type_int16:
		{
			int value = *(int16_t *)pconfig_data;

			cliPrintf(cliCtx, "%d", value);
			break;
		}
		case config_data_type_int32:
		{
			int value = *(int32_t *)pconfig_data;

			cliPrintf(cliCtx, "%d", value);
			break;
		}
		case config_data_type_uint8:
		{
			unsigned int value = *(int8_t *)pconfig_data;

			cliPrintf(cliCtx, "%u", value);
			break;
		}
		case config_data_type_uint16:
		{
			uint16_t value = *(int16_t *)pconfig_data;

			cliPrintf(cliCtx, "%u", value);
			break;
		}
		case config_data_type_uint32:
		{
			uint16_t value = *(int32_t *)pconfig_data;

			cliPrintf(cliCtx, "%u", value);
			break;
		}
		case config_data_type_float:
		{
			cliPrintf(cliCtx, "%g", pconfig_data);
			break;
		}
		case config_data_type_string:
		{
			char * value = (char *)pconfig_data;

			cliPrintf(cliCtx, "%s", value);
			break;
		}
		case config_data_type_enum:
		{
			int8_t value = *(int8_t *)pconfig_data;
			char const * mapping = lookupEnumValue( value,
												parameterConfig->type_specific.enum_data.mappings,
												parameterConfig->type_specific.enum_data.num_mappings );

			cliPrintf(cliCtx, "%s", (mapping != NULL) ? mapping : "???" );
			break;
		}
	}
}

/*
	A table of all string considered to be 'true', or 'on'.
	Everything else is considered to be 'false'.
*/
static const char * const trueValues[] =
{
	"yes",
	"on",
	"1",
	"true"
};

static bool isValueTrue( char const * const value )
{
	uint32_t index;

	for (index=0; index < ARRAY_SIZE(trueValues); index++)
	{
		if (strcasecmp( value, trueValues[index] ) == 0)
			return true;
	}

	return false;
}

bool assignSavedParameterValue( void const * const saved_data,
								config_data_types_t saved_data_type,
								void * const parameter,
								parameterConfig_st const * const parameterConfig )
{
	bool assigned = true;
	/*
		To allow for the case where the saved type doesn't match the current type, we attempt to convert between the types.
	*/
	uint32_t savedValue = 0;
	float savedFloatValue = 0.0f;

	switch ( saved_data_type )
	{
		case config_data_type_boolean:
		case config_data_type_int8:
		case config_data_type_enum:
			savedValue = (uint32_t)*(int8_t *)saved_data;
			break;
		case config_data_type_int16:
			savedValue = (uint32_t)*(int16_t *)saved_data;
			break;
		case config_data_type_int32:
			savedValue = (uint32_t)*(int32_t *)saved_data;
			break;
		case config_data_type_uint8:
			savedValue = (uint32_t)*(uint8_t *)saved_data;
			break;
		case config_data_type_uint16:
			savedValue = (uint32_t)*(uint16_t *)saved_data;
			break;
		case config_data_type_uint32:
			savedValue = (uint32_t)*(uint32_t *)saved_data;
			break;
		case config_data_type_float:
		{
			union {
				uint8_t buf[4];
				float f;
			} u;
			u.buf[0] = *((uint8_t *)saved_data + 0);
			u.buf[1] = *((uint8_t *)saved_data + 1);
			u.buf[2] = *((uint8_t *)saved_data + 2);
			u.buf[3] = *((uint8_t *)saved_data + 3);
			savedFloatValue = u.f;

			break;
		}
		default:
			break;
	}
	switch( parameterConfig->data_type )
	{
		case config_data_type_boolean:
		case config_data_type_int8:
		case config_data_type_enum:
			if ( saved_data_type == config_data_type_float )
				*(int8_t *)parameter = lrintf(savedFloatValue);
			else if ( saved_data_type != config_data_type_string )
				*(int8_t *)parameter = savedValue;
			else	/* string */
			{
			}
			break;
		case config_data_type_int16:
			if ( saved_data_type == config_data_type_float )
				*(int16_t *)parameter = lrintf(savedFloatValue);
			else if ( saved_data_type != config_data_type_string )
				*(int16_t *)parameter = savedValue;
			else	/* string */
			{
			}
			break;
		case config_data_type_int32:
			if ( saved_data_type == config_data_type_float )
				*(int32_t *)parameter = lrintf(savedFloatValue);
			else if ( saved_data_type != config_data_type_string )
				*(int32_t *)parameter = savedValue;
			else	/* string */
			{
			}
			break;
		case config_data_type_uint8:
			if ( saved_data_type == config_data_type_float )
				*(uint8_t *)parameter = lrintf(savedFloatValue);
			else if ( saved_data_type != config_data_type_string )
				*(uint8_t *)parameter = savedValue;
			else	/* string */
			{
			}
			break;
		case config_data_type_uint16:
			if ( saved_data_type == config_data_type_float )
				*(uint16_t *)parameter = lrintf(savedFloatValue);
			else if ( saved_data_type != config_data_type_string )
				*(uint16_t *)parameter = savedValue;
			else	/* string */
			{
			}
			break;
		case config_data_type_uint32:
			if ( saved_data_type == config_data_type_float )
				*(uint32_t *)parameter = lrintf(savedFloatValue);
			else if ( saved_data_type != config_data_type_string )
				*(uint32_t *)parameter = savedValue;
			else	/* string */
			{
			}
			break;
		case config_data_type_float:
			if ( saved_data_type == config_data_type_float )
			{
				memcpy( parameter, &savedFloatValue, sizeof(float ) );
				//*(float *)parameter = savedFloatValue;
			}
			else if ( saved_data_type != config_data_type_string )
				*(float *)parameter = (float)savedValue;
			else	/* string */
			{
			}
			break;
		case config_data_type_string:
			if (saved_data_type == config_data_type_string)
				strlcpy( (char *)parameter, saved_data, parameterConfig->type_specific.max_string_length );
			else	/* string */
			{
				/* do itoa() or something? */
			}
			break;
	}

	return assigned;
}

static bool assignParameterValue( void * const pdata,
												void const * const pdefault_configuration,
												parameterConfig_st const * const parameterConfig,
												char const * const value )
{
	bool wrote_parameter_value = false;
	void *pconfig_data = pdata;
	static const uint32_t zeroOther = 0;
	static const float zeroFloat = 0.0f;

	if ( strcmp( value, "!" ) == 0 )	/* write default value to current value */
	{
		void const * pdefault_data;
		float const *default_float;

		if (pdefault_configuration == NULL)
		{
			pdefault_data = &zeroOther;
			default_float = &zeroFloat;
		}
		else
		{
			pdefault_data = (char const *)pdefault_configuration + parameterConfig->offsetToData;
			default_float = (float *)((char const *)pdefault_configuration + parameterConfig->offsetToData);
		}

		switch( parameterConfig->data_type )
		{
			case config_data_type_boolean:
			case config_data_type_int8:
			case config_data_type_uint8:
			case config_data_type_enum:
				*(int8_t *)pconfig_data = *(int8_t *)pdefault_data;
				wrote_parameter_value = true;
				break;
			case config_data_type_int16:
			case config_data_type_uint16:
				*(int16_t *)pconfig_data = *(int16_t *)pdefault_data;
				wrote_parameter_value = true;
				break;
			case config_data_type_int32:
			case config_data_type_uint32:
				*(int32_t *)pconfig_data = *(int32_t *)pdefault_data;
				wrote_parameter_value = true;
				break;
			case config_data_type_float:
				*(float *)pconfig_data = *default_float;
				wrote_parameter_value = true;
				break;
			case config_data_type_string:
				strlcpy( (char *)pconfig_data, (char *)pdefault_data, parameterConfig->type_specific.max_string_length );
				wrote_parameter_value = true;
				break;
		}
	}
	else
	{
		long val = strtol(value, NULL, 0);

		switch( parameterConfig->data_type )
		{
			case config_data_type_boolean:
			{
				int is_true = isValueTrue( value );

				*(int8_t *)pconfig_data = is_true;
				wrote_parameter_value = true;
				break;
			}
			case config_data_type_int8:
				*(int8_t *)pconfig_data = val;
				wrote_parameter_value = true;
				break;
			case config_data_type_int16:
				*(int16_t *)pconfig_data = val;
				wrote_parameter_value = true;
				break;
			case config_data_type_int32:
				*(int32_t *)pconfig_data = val;
				wrote_parameter_value = true;
				break;
			case config_data_type_uint8:
				*(uint8_t *)pconfig_data = val;
				wrote_parameter_value = true;
				break;
			case config_data_type_uint16:
				*(uint16_t *)pconfig_data = val;
				wrote_parameter_value = true;
				break;
			case config_data_type_uint32:
				*(uint32_t *)pconfig_data = strtoul(value, NULL, 0);
				wrote_parameter_value = true;
				break;
			case config_data_type_float:
				*(float *)pconfig_data = strtof(value, NULL);
				wrote_parameter_value = true;
				break;
			case config_data_type_string:
				strlcpy( (char *)pconfig_data, value, parameterConfig->type_specific.max_string_length );
				wrote_parameter_value = true;
				break;
			case config_data_type_enum:
			{
				uint8_t enum_value;
				bool found_mapping = lookupEnumValueByName( value,
														parameterConfig->type_specific.enum_data.mappings,
														parameterConfig->type_specific.enum_data.num_mappings,
														&enum_value );

				if ( found_mapping == false )
				{
					/* see if we can find a matching enum by value */
					if (lookupEnumValue( val,
									parameterConfig->type_specific.enum_data.mappings,
									parameterConfig->type_specific.enum_data.num_mappings ) != NULL)
					{
						enum_value = val;
						found_mapping = true;
					}
				}
				if ( found_mapping == true )
				{
					*(uint8_t *)pconfig_data = enum_value;
					wrote_parameter_value = true;
				}
				break;
			}
		}
	}

	return wrote_parameter_value;
}


static parameterConfig_st const * lookupParameterByName( parameterConfig_st const * const parameterConfigs,
																uint32_t nbParameterConfigs,
																ParameterNameLookup ParameterNameLookupCB,
																char const * parameter_name)
{
	unsigned int index;

	for (index = 0; index < nbParameterConfigs; index++)
	{
		if (strncasecmp( parameter_name, ParameterNameLookupCB(parameterConfigs[index].parameter_id), strlen(parameter_name) ) == 0 )
		{
			return &parameterConfigs[index];
		}
	}

	return NULL;
}

static bool lookupParameterPrintValue( void *pdata,
						parameterConfig_st const * const parameterConfigs,
						uint8_t nbParameterConfigs,
						ParameterNameLookup ParameterNameLookupCB,
						char const * parameter_name,
						void *cliCtx
						)
{
	parameterConfig_st const *parameterConfig;
	bool printed_parameter = false;

	parameterConfig = lookupParameterByName( parameterConfigs, nbParameterConfigs, ParameterNameLookupCB, parameter_name );
	if (parameterConfig != NULL)
	{
		printParameterValue( (char *)pdata + parameterConfig->offsetToData, parameterConfig, cliCtx );
		printed_parameter = true;
	}

	return printed_parameter;
}

static bool lookupParameterAssignValue( void *pcfg,
							void const * pdefault_configuration,
							parameterConfig_st const * const parameterConfigs,
							uint8_t nbParameterConfigs,
							ParameterNameLookup ParameterNameLookupCB,
							char * parameter_name,
							char *parameter_value)
{
	parameterConfig_st const *parameterConfig;
	bool wrote_parameter = false;

	parameterConfig = lookupParameterByName( parameterConfigs, nbParameterConfigs, ParameterNameLookupCB, parameter_name );
	if (parameterConfig != NULL)
	{
		wrote_parameter = assignParameterValue( (char *)pcfg + parameterConfig->offsetToData, pdefault_configuration, parameterConfig, parameter_value );
	}

	return wrote_parameter;
}

bool savedParameterValueMatchesCurrentValue( void const *psaved,
												void const * pCurrentValue,
												parameterConfig_st const * parameterConfig )
{
	bool areSame = false;
	config_data_types_t data_type;
	void const *savedValue;

	data_type = GET_CONFIG_FIELD( *(uint32_t *)psaved, PARAMETER_TYPE );
	savedValue = (char *)psaved + sizeof(uint32_t);

	if (data_type != parameterConfig->data_type)
	{
		goto done;
	}

	switch (data_type)
	{
		case config_data_type_boolean:
		case config_data_type_int8:
		case config_data_type_uint8:
		case config_data_type_enum:
			areSame = *(int8_t *)pCurrentValue == *(int8_t *)savedValue;
			break;
		case config_data_type_int16:
		case config_data_type_uint16:
			areSame = *(int16_t *)pCurrentValue == *(int16_t *)savedValue;
			break;
		case config_data_type_int32:
		case config_data_type_uint32:
			areSame = *(int32_t *)pCurrentValue == *(int32_t *)savedValue;
			break;
		case config_data_type_float:
			/* casting to floats and comparing results in a crash */
			areSame = memcmp( pCurrentValue, savedValue, sizeof(float) ) == 0;
			break;
		case config_data_type_string:
			areSame = strcasecmp( (char *)pCurrentValue, (char *)savedValue ) == 0;
			break;
	}

done:

	return areSame;
}

bool currentParameterValueMatchesDefaultValue( void const * pconfig_data,
												void const * pdefault_data,
												parameterConfig_st const * parameterConfig )
{
	bool areSame = false;

	static const uint32_t zeroBuffer[2] = {0};
	static float floatZero = 0.0f;
	void const *default_other;
	float const *default_float;

	/*
		pconfig_data points to the actual parameter.
		pdefault data points to the start of the configuration sturcture, so needs the dta point offset added to it.
	*/

	if (pdefault_data == NULL)
	{
		default_other = zeroBuffer;
		default_float = &floatZero;
	}
	else
	{
		default_other = (char *)pdefault_data + parameterConfig->offsetToData;
		default_float = (float *)((char *)pdefault_data + parameterConfig->offsetToData);
	}

	switch (parameterConfig->data_type)
	{
		case config_data_type_boolean:
		case config_data_type_int8:
		case config_data_type_uint8:
		case config_data_type_enum:
			areSame = *(int8_t *)pconfig_data == *(int8_t *)default_other;
			break;
		case config_data_type_int16:
		case config_data_type_uint16:
			areSame = *(int16_t *)pconfig_data == *(int16_t *)default_other;
			break;
		case config_data_type_int32:
		case config_data_type_uint32:
			areSame = *(int32_t *)pconfig_data == *(int32_t *)default_other;
			break;
		case config_data_type_float:
			areSame = *(float *)pconfig_data == *default_float;
			break;
		case config_data_type_string:
			areSame = strcasecmp( (char *)pconfig_data, (char *)default_other ) == 0;
			break;
	}

	return areSame;
}


/*
	handleStandardCommand:
	handle a standard CLI command of the form:
		<name> ?									print parameter names
		<name> <instance> ?							print all parameter values for this instance
		<name> <instance> <parameter> ?				print a single parameter value
		<name> <instance> <parameter> <value|!>		set a parameter value
*/
int handleStandardCommand( run_command_data_st const * command_context )
{
	void * const cliCtx = command_context->cliCtx;
	command_st const * command = command_context->command;
	int const argc = command_context->argc;
	char * * const argv = command_context->argv;
	int result = poll_result_error;

	if ( argc == 2 && strcmp( argv[1], "?" ) == 0 )
	{
		/* display all parameter names */
		printParameterNames( command->parameterConfigs, command->nbParameterConfigs, command->ParameterNameLookupCB, cliCtx );
		result = poll_result_ok;
	}
	else if (argc == 3 && strcmp( argv[2], "?" ) == 0 )
	{
		/* print all parameters for this group/instance */
		unsigned int instance;

		if ( strtoint( argv[1], &instance ) && instance < command->nb_configuration_instance )
		{
			unsigned int index;

			for (index=0; index < command->nbParameterConfigs; index++)
			{
				char const * parameter_name = command->ParameterNameLookupCB(command->parameterConfigs[index].parameter_id);
				unsigned int offset_to_correct_configuration_data = (instance * command->configuration_size);

				cliPrintf( cliCtx, "\n%20s: ", parameter_name );
				(void)lookupParameterPrintValue( (char *)command->configuration + offset_to_correct_configuration_data,
											command->parameterConfigs,
											command->nbParameterConfigs,
											command->ParameterNameLookupCB,
											parameter_name,
											cliCtx );
			}
			result = poll_result_ok;
		}
	}
	else if ( argc > 3 )
	{
		unsigned int instance;

		if ( strtoint( argv[1], &instance ) )
		{
			if ( instance < command->nb_configuration_instance )
			{
				unsigned int offset_to_configuration_data = (instance * command->configuration_size);

				if ( strcmp( argv[3], "?" ) == 0 )
				{

					if ( lookupParameterPrintValue( (char *)command->configuration + offset_to_configuration_data,
												command->parameterConfigs,
												command->nbParameterConfigs,
												command->ParameterNameLookupCB,
												argv[2],
												cliCtx ) == true )
					{
						result = poll_result_ok;
					}
				}
				else
				{
					/* write the new value */
					if ( lookupParameterAssignValue( (char *)command->configuration + offset_to_configuration_data,
												command->default_configuration,
												command->parameterConfigs,
												command->nbParameterConfigs,
												command->ParameterNameLookupCB,
												argv[2],
												argv[3] ) == true )
					{
						result = poll_result_ok;
					}
				}
			}
		}
	}

	if ( result == poll_result_error )
	{
		cliPrintf( cliCtx, "\nFormat: %s ?                               - print all parameter names", command->name );
		cliPrintf( cliCtx, "\n        %s <0 -> %d> ?                      - print all parameter values", command->name, command->nb_configuration_instance-1 );
		cliPrintf( cliCtx, "\n        %s <0 -> %d> <parameter> ?          - print a single parameter value", command->name, command->nb_configuration_instance-1 );
		cliPrintf( cliCtx, "\n        %s <0 -> %d> <parameter> <value|!>  - set a parameter value (!) = default)", command->name, command->nb_configuration_instance-1 );
	}

	return result;
}

int getLengthOfData( config_data_types_t data_type, void const * pcfg )
{
	int data_length = -1;

	switch( data_type )
	{
		case config_data_type_boolean:
		case config_data_type_int8:
		case config_data_type_uint8:
		case config_data_type_enum:
			data_length = sizeof(int8_t);
			break;
		case config_data_type_int16:
		case config_data_type_uint16:
			data_length = sizeof(int16_t);
			break;
		case config_data_type_int32:
		case config_data_type_uint32:
			data_length = sizeof(int32_t);
			break;
		case config_data_type_float:
			data_length = sizeof(float);
			break;
		case config_data_type_string:
			data_length = strlen( (char *)pcfg ) + 1;	/* include NUL terminator */
			break;
		default:
			break;
	}

	return data_length;
}

command_st const *findCommandFromID( command_st const *commands,
										unsigned int nb_commands,
										configuration_id_t command_id )
{
	unsigned int command_index;

	for ( command_index = 0; command_index < nb_commands; command_index++ )
	{
		if ( commands[command_index].group_id == command_id )
			return &commands[command_index];
	}

	return NULL;
}

parameterConfig_st const * findDataPointFromParameterID( parameterConfig_st const * parameterConfigs,
																	unsigned int const nbParameterConfigs,
																	unsigned int parameterID )
{
	unsigned int parameterConfig_index;

	/* find the data point with the matching parameter ID */
	for ( parameterConfig_index = 0; parameterConfig_index < nbParameterConfigs; parameterConfig_index++ )
	{
		if ( parameterConfigs[parameterConfig_index].parameter_id == parameterID )
		{
			return &parameterConfigs[parameterConfig_index];
		}
	}

	return NULL;
}

/* called from the CLI */
int runCommand( int argc, char **argv, void *cliCtx )
{
	run_command_data_st command_data;
	int result;

	command_data.argc = argc;
	command_data.argv = argv;
	command_data.cliCtx = cliCtx;
	command_data.handled = false;
	result = pollCodeGroups( poll_id_run_command, &command_data, false );

	/* Not a config command. Maybe it's a non-config command */
	if (command_data.handled == false)
	{
		non_config_run_command_data_st non_config_command_data;

		non_config_command_data.argc = argc;
		non_config_command_data.argv = argv;
		non_config_command_data.cliCtx = cliCtx;
		result = pollCodeGroups( poll_id_run_non_config_command, &non_config_command_data, false );
	}

	return result;
}

/* called from the code responsible for the various commands from within the poll_id_run_command pollCodeGroups() call. */
int runCommandHandler( command_st const * commands, uint32_t nb_commands, void *pv )
{
	command_st const * pcmd;
	int result;
	run_command_data_st *pcmd_data = pv;

	if ( (pcmd=findCommand(commands, nb_commands, pcmd_data->argv[0])) != NULL )
	{
		pcmd_data->command = pcmd;
		result = pcmd->handler( pcmd_data );
		pcmd_data->handled = true;
	}
	else
	{
		result = poll_result_error;
	}

	return result;
}

int runNonConfigCommandHandler( non_config_command_st const * commands, uint32_t nb_commands, void *pv )
{
	non_config_command_st const * pcmd;
	int result;
	non_config_run_command_data_st *pcmd_data = pv;

	if ( (pcmd=findNonConfigCommand(commands, nb_commands, pcmd_data->argv[0])) != NULL )
	{
		pcmd_data->command = pcmd;
		result = pcmd->handler( pcmd_data );
	}
	else
	{
		result = poll_result_error;
	}

	return result;
}


int idCommandHandler( command_st const * commands, uint32_t nb_commands, void *pv )
{
	help_command_data_st *pcmd_data = pv;
	unsigned int command_index;

	for ( command_index = 0; command_index < nb_commands; command_index++ )
	{
		cliPrintf( pcmd_data->cliCtx, "\n%s", commands[command_index].name );
	}

	return poll_result_ok;
}

int idNonConfigCommandHandler( non_config_command_st const * commands, uint32_t nb_commands, void *pv )
{
	help_command_data_st *pcmd_data = pv;
	unsigned int command_index;

	for ( command_index = 0; command_index < nb_commands; command_index++ )
	{
		cliPrintf( pcmd_data->cliCtx, "\n%s", commands[command_index].name );
	}

	return poll_result_ok;
}


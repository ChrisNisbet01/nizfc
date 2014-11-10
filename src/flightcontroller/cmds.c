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

static void printParameterName( parameterConfig_st const * data_point, ParameterNameLookup ParameterNameLookupCB, void *cliCtx )
{
	cliPrintf( cliCtx, "%20s", ParameterNameLookupCB(data_point->parameter_id));
}

static void printParameterNames( parameterConfig_st const * data_points, unsigned int nb_data_points, ParameterNameLookup ParameterNameLookupCB, void *cliCtx )
{
	unsigned int index;

	cliPrintf( cliCtx, "\r\nparameters are:" );

	for (index=0; index < nb_data_points; index++ )
	{
		cliPrintf( cliCtx, "\n" );
		printParameterName( &data_points[index], ParameterNameLookupCB, cliCtx );
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
			float value = *(float *)pconfig_data;

			// TODO: printf floating point
			cliPrintf(cliCtx, "%f", value);
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
												parameterConfig->type_specific.enum_data.enum_mappings,
												parameterConfig->type_specific.enum_data.num_enum_mappings );

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
			savedFloatValue = *(float *)saved_data;
			break;
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
				*(float *)parameter = savedFloatValue;
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
			pdefault_data = (char const *)pdefault_configuration + parameterConfig->offset_to_data_point;
			default_float = (float *)((char const *)pdefault_configuration + parameterConfig->offset_to_data_point);
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
														parameterConfig->type_specific.enum_data.enum_mappings,
														parameterConfig->type_specific.enum_data.num_enum_mappings,
														&enum_value );

				if ( found_mapping == false )
				{
					/* see if we can find a matching enum by value */
					if (lookupEnumValue( val,
									parameterConfig->type_specific.enum_data.enum_mappings,
									parameterConfig->type_specific.enum_data.num_enum_mappings ) != NULL)
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


static parameterConfig_st const * lookupParameterByName( parameterConfig_st const * const data_points,
																uint32_t nb_data_points,
																ParameterNameLookup ParameterNameLookupCB,
																char const * parameter_name)
{
	unsigned int index;

	for (index = 0; index < nb_data_points; index++)
	{
		if (strncasecmp( parameter_name, ParameterNameLookupCB(data_points[index].parameter_id), strlen(parameter_name) ) == 0 )
		{
			return &data_points[index];
		}
	}

	return NULL;
}

static bool lookupParameterPrintValue( void *pdata,
						parameterConfig_st const * const data_points,
						uint8_t nb_data_points,
						ParameterNameLookup ParameterNameLookupCB,
						char const * parameter_name,
						void *cliCtx
						)
{
	parameterConfig_st const *data_point;
	bool printed_parameter = false;

	data_point = lookupParameterByName( data_points, nb_data_points, ParameterNameLookupCB, parameter_name );
	if (data_point != NULL)
	{
		printParameterValue( (char *)pdata + data_point->offset_to_data_point, data_point, cliCtx );
		printed_parameter = true;
	}

	return printed_parameter;
}

static bool lookupParameterAssignValue( void *pcfg,
							void const * pdefault_configuration,
							parameterConfig_st const * const data_points,
							uint8_t nb_data_points,
							ParameterNameLookup ParameterNameLookupCB,
							char * parameter_name,
							char *parameter_value)
{
	parameterConfig_st const *data_point;
	bool wrote_parameter = false;

	data_point = lookupParameterByName( data_points, nb_data_points, ParameterNameLookupCB, parameter_name );
	if (data_point != NULL)
	{
		wrote_parameter = assignParameterValue( (char *)pcfg + data_point->offset_to_data_point, pdefault_configuration, data_point, parameter_value );
	}

	return wrote_parameter;
}

bool savedParameterValueMatchesCurrentValue( void const *psaved,
												void const * pCurrentValue,
												parameterConfig_st const * data_point )
{
	bool areSame = false;
	config_data_types_t data_type;
	void const *savedValue;

	data_type = GET_CONFIG_FIELD( *(uint32_t *)psaved, PARAMETER_TYPE );
	savedValue = (char *)psaved + sizeof(uint32_t);

	if (data_type != data_point->data_type)
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
			areSame = *(float *)pCurrentValue == *(float *)savedValue;
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
												parameterConfig_st const * data_point )
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
		default_other = (char *)pdefault_data + data_point->offset_to_data_point;
		default_float = (float *)((char *)pdefault_data + data_point->offset_to_data_point);
	}

	switch (data_point->data_type)
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
int handleStandardCommand( run_command_data_st const * command_context,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					parameterConfig_st const * data_points,
					unsigned int const nb_data_points,
					ParameterNameLookup ParameterNameLookupCB
					)
{
	void * const cliCtx = command_context->cliCtx;
	int const argc = command_context->argc;
	char * * const argv = command_context->argv;
	int result = poll_result_error;

	if ( argc == 2 && strcmp( argv[1], "?" ) == 0 )
	{
		/* display all parameter names */
		printParameterNames( data_points, nb_data_points, ParameterNameLookupCB, cliCtx );
		result = poll_result_ok;
	}
	else if (argc == 3 && strcmp( argv[2], "?" ) == 0 )
	{
		/* print all parameters for this group/instance */
		unsigned int instance;

		if ( strtoint( argv[1], &instance ) )
		{
			unsigned int index;

			for (index=0; index < nb_data_points; index++)
			{
				char const * parameter_name = ParameterNameLookupCB(data_points[index].parameter_id);
				unsigned int offset_to_correct_configuration_data = (instance*configuration_size);

				cliPrintf( cliCtx, "\n%20s: ", parameter_name );
				(void)lookupParameterPrintValue( (char *)pcfg + offset_to_correct_configuration_data,
											data_points,
											nb_data_points,
											ParameterNameLookupCB,
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
			if ( instance < nb_configurations )
			{
				unsigned int offset_to_configuration_data = (instance*configuration_size);

				if ( strcmp( argv[3], "?" ) == 0 )
				{

					if ( lookupParameterPrintValue( (char *)pcfg + offset_to_configuration_data,
												data_points,
												nb_data_points,
												ParameterNameLookupCB,
												argv[2],
												cliCtx ) == true )
					{
						result = poll_result_ok;
					}
				}
				else
				{
					/* write the new value */
					if ( lookupParameterAssignValue( (char *)pcfg + offset_to_configuration_data,
												default_configuration,
												data_points,
												nb_data_points,
												ParameterNameLookupCB,
												argv[2],
												argv[3] ) == true )
					{
						result = poll_result_ok;
					}
				}
			}
		}
	}

	if ( result == -1 )
	{
		cliPrintf( cliCtx, "\nFormat: %s ?                                - print all parameter names", argv[0] );
		cliPrintf( cliCtx, "\n        %s <0 -> %d> ?                      - print all parameter values", argv[0], nb_configurations-1 );
		cliPrintf( cliCtx, "\n        %s <0 -> %d> <parameter> ?          - print a single parameter value", argv[0], nb_configurations-1 );
		cliPrintf( cliCtx, "\n        %s <0 -> %d> <parameter> <value|!>  - set a paremeter value (!) = default)", argv[0], nb_configurations-1 );
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

parameterConfig_st const * findDataPointFromParameterID( parameterConfig_st const * data_points,
																	unsigned int const nb_data_points,
																	unsigned int parameterID )
{
	unsigned int data_point_index;

	/* find the data point with the matching parameter ID */
	for ( data_point_index = 0; data_point_index < nb_data_points; data_point_index++ )
	{
		if ( data_points[data_point_index].parameter_id == parameterID )
		{
			return &data_points[data_point_index];
		}
	}

	return NULL;
}

/* called from the CLI */
int runCommand( int argc, char **argv, void *cliCtx )
{
	run_command_data_st command_data;

	command_data.argc = argc;
	command_data.argv = argv;
	command_data.cliCtx = cliCtx;

	return pollCodeGroups( poll_id_run_command, &command_data, 0 );
}

/* called from the code repsonsible for the various commands from within the poll_id_run_command pollCodeGroups() call. */
int runCommandHandler( command_st const * commands, uint32_t nb_commands, void *pv )
{
	command_st const * pcmd;
	int result;
	run_command_data_st *pcmd_data = pv;

	if ( (pcmd=findCommand(commands, nb_commands, pcmd_data->argv[0])) != NULL )
		result = pcmd->handler( pcmd_data );
	else
		result = poll_result_error;

	return result;
}



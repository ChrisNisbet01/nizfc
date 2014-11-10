#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>
#include <config_structure.h>
#include <cmds.h>
#include <cli.h>

static command_st const * findCommand( command_st const * commands, uint32_t nb_commands, char const * name )
{
	uint32_t index;

	for (index = 0; index < nb_commands; index++)
	{
		if ( strcasecmp(name, commands[index].name) == 0 )
			return &commands[index];
	}

	return NULL;
}

static void print_parameter_name( config_data_point_st const * data_point, ParameterNameLookup ParameterNameLookupCB, void *cliCtx )
{
	cliPrintf( cliCtx, "%20s", ParameterNameLookupCB(data_point->parameter_id));
}

static void print_parameter_names( config_data_point_st const * data_points, unsigned int nb_data_points, ParameterNameLookup ParameterNameLookupCB, void *cliCtx )
{
	unsigned int index;

	cliPrintf( cliCtx, "\r\nparameters are:" );

	for (index=0; index < nb_data_points; index++ )
	{
		cliPrintf( cliCtx, "\n" );
		print_parameter_name( &data_points[index], ParameterNameLookupCB, cliCtx );
	}
}

static char const *enumLookup( int8_t value, enum_mapping_st const * mappings, uint_fast8_t nb_mappings )
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

static bool enumLookupByName( char const * const name, enum_mapping_st const const * const mappings, uint_fast8_t const nb_mappings, uint8_t * const enum_value )
{
	uint_fast8_t index;

	for (index=0; index < nb_mappings; index++)
	{
		if (strcasecmp(name, mappings[index].name) == 0)
		{
			*enum_value = mappings[index].value;
			return true;
		}
	}

	/* not found in mappings */
	return false;
}


void print_configuration_data_point( void *pconfig_data,
											config_data_point_st const * pconfig,
											void *printfpv )
{

	switch( pconfig->data_type )
	{
		case config_data_type_boolean:
		{
			int8_t value = *(int8_t *)pconfig_data;

			cliPrintf(printfpv, "%s", (value != 0) ? "on" : "off");
			break;
		}
		case config_data_type_int8:
		{
			int value = *(int8_t *)pconfig_data;

			cliPrintf(printfpv, "%d", value);
			break;
		}
		case config_data_type_int16:
		{
			int value = *(int16_t *)pconfig_data;

			cliPrintf(printfpv, "%d", value);
			break;
		}
		case config_data_type_int32:
		{
			int value = *(int32_t *)pconfig_data;

			cliPrintf(printfpv, "%d", value);
			break;
		}
		case config_data_type_uint8:
		{
			unsigned int value = *(int8_t *)pconfig_data;

			cliPrintf(printfpv, "%u", value);
			break;
		}
		case config_data_type_uint16:
		{
			uint16_t value = *(int16_t *)pconfig_data;

			cliPrintf(printfpv, "%u", value);
			break;
		}
		case config_data_type_uint32:
		{
			uint16_t value = *(int32_t *)pconfig_data;

			cliPrintf(printfpv, "%u", value);
			break;
		}
		case config_data_type_float:
		{
			float value = *(float *)pconfig_data;

			// TODO: printf floating point
			cliPrintf(printfpv, "%f", value);
			break;
		}
		case config_data_type_string:
		{
			char * value = (char *)pconfig_data;

			cliPrintf(printfpv, "%s", value);
			break;
		}
		case config_data_type_enum:
		{
			int8_t value = *(int8_t *)pconfig_data;
			char const * mapping = enumLookup( value,
												pconfig->type_specific.enum_data.enum_mappings,
												pconfig->type_specific.enum_data.num_enum_mappings );

			cliPrintf(printfpv, "%s", (mapping != NULL) ? mapping : "???" );
			break;
		}
	}
}

/*
	A table of all string considered to be 'true', or 'on'.
	Everything else is considered to be 'false'.
*/
static const char * const true_values[] =
{
	"yes",
	"on",
	"1",
	"true"
};

static bool isValueTrue( char const * const value )
{
	uint32_t index;

	for (index=0; index < ARRAY_SIZE(true_values); index++)
	{
		if (strcasecmp( value, true_values[index] ) == 0)
			return true;
	}

	return false;
}

static bool assign_configuration_data_point( void * const pdata,
												void const * const pdefault_configuration,
												config_data_point_st const * const pconfig,
												char const * const value )
{
	bool wrote_parameter_value = false;
	void *pconfig_data = (char *)pdata + pconfig->offset_to_data_point;
	static uint32_t const zero = 0;

	if ( strcmp( value, "!" ) == 0 )	/* write default value to current value */
	{
		void const * pdefault_data = (char const *)pdefault_configuration + pconfig->offset_to_data_point;

		if (pdefault_data == NULL)
			pdefault_data = &zero;

		switch( pconfig->data_type )
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
				*(float *)pconfig_data = *(float *)pdefault_data;
				wrote_parameter_value = true;
				break;
			case config_data_type_string:
				strlcpy( (char *)pconfig_data, (char *)pdefault_data, pconfig->type_specific.max_string_length );
				wrote_parameter_value = true;
				break;
		}
	}
	else
	{
		long val = strtol(value, NULL, 0);

		switch( pconfig->data_type )
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
				strlcpy( (char *)pconfig_data, value, pconfig->type_specific.max_string_length );
				wrote_parameter_value = true;
				break;
			case config_data_type_enum:
			{
				uint8_t enum_value;
				bool found_mapping = enumLookupByName( value,
														pconfig->type_specific.enum_data.enum_mappings,
														pconfig->type_specific.enum_data.num_enum_mappings,
														&enum_value );

				if ( found_mapping == false )
				{
					/* see if we can find a matching enum by value */
					if (enumLookup( val,
									pconfig->type_specific.enum_data.enum_mappings,
									pconfig->type_specific.enum_data.num_enum_mappings ) != NULL)
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


static config_data_point_st const * config_data_point_lookup( config_data_point_st const * const data_points,
																uint32_t nb_data_points,
																ParameterNameLookup ParameterNameLookupCB,
																char const * parameter_name)
{
	unsigned int index;

	for (index = 0; index < nb_data_points; index++)
	{
		if (strcasecmp( ParameterNameLookupCB(data_points[index].parameter_id), parameter_name ) == 0 )
		{
			return &data_points[index];
		}
	}

	return NULL;
}

static bool print_config_value( void *pdata,
						config_data_point_st const * const data_points,
						uint8_t nb_data_points,
						ParameterNameLookup ParameterNameLookupCB,
						char const * parameter_name,
						void *printfpv
						)
{
	config_data_point_st const *data_point;
	bool printed_parameter = false;

	data_point = config_data_point_lookup( data_points, nb_data_points, ParameterNameLookupCB, parameter_name );
	if (data_point != NULL)
	{
		print_configuration_data_point( (char *)pdata + data_point->offset_to_data_point, data_point, printfpv );
		printed_parameter = true;
	}

	return printed_parameter;
}

static bool assign_config_value( void *pcfg,
							void const * pdefault_configuration,
							config_data_point_st const * const data_points,
							uint8_t nb_data_points,
							ParameterNameLookup ParameterNameLookupCB,
							char * parameter_name,
							char *parameter_value)
{
	config_data_point_st const *data_point;
	bool wrote_parameter = false;

	data_point = config_data_point_lookup( data_points, nb_data_points, ParameterNameLookupCB, parameter_name );
	if (data_point != NULL)
	{
		wrote_parameter = assign_configuration_data_point( pcfg, pdefault_configuration, data_point, parameter_value );
	}

	return wrote_parameter;
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
					config_data_point_st const * data_points,
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
		print_parameter_names( data_points, nb_data_points, ParameterNameLookupCB, cliCtx );
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
				(void)print_config_value( (char *)pcfg + offset_to_correct_configuration_data,
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
				if ( strcmp( argv[3], "?" ) == 0 )
				{
					unsigned int offset_to_correct_configuration_data = (instance*configuration_size);

					if ( print_config_value( (char *)pcfg + offset_to_correct_configuration_data,
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
					if ( assign_config_value( (char *)pcfg + (instance*configuration_size),
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

/* called from the CLI */
int runCommand( int argc, char **argv, void *cliCtx )
{
	run_command_data_st command_data;

	command_data.argc = argc;
	command_data.argv = argv;
	command_data.cliCtx = cliCtx;

	return poll_groups( poll_id_run_command, &command_data, 0 );
}

/* called from the code repsonsible for the various commands from within the poll_id_run_command poll_groups() call. */
int runCommandHandler( command_st const * commands, uint32_t nb_commands, void *pv )
{
	command_st const * pcmd;
	int result;
	run_command_data_st *pcmd_data = pv;

	if ( (pcmd=findCommand(commands, nb_commands, pcmd_data->argv[0])) != NULL )
		result = pcmd->handler( pcmd_data );
	else
		result = -1;

	return result;
}



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
	unsigned int index;

	for (index = 0; index < nb_commands; index++)
	{
		if ( strcasecmp(commands[index].name, name) == 0 )
			return &commands[index];
	}

	return NULL;
}

static char const * command_name_lookup( command_st *commands, uint32_t nb_commands, char *partial_name, uint32_t *previous_index )
{
	uint_fast32_t index;

	for (index=(*previous_index+1); index < nb_commands; index++)
	{
		if( strncasecmp( partial_name, commands[index].name, strlen( partial_name ) ) == 0 )
		{
			*previous_index = index;
			return commands[index].name;
		}
	}

	return NULL;
}

/* called from the CLI */
int runCommand( int argc, char **argv, void *pv )
{
	run_command_data_st command_data;

	command_data.argc = argc;
	command_data.argv = argv;
	command_data.pctx = pv;

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

static void print_parameter_names( config_data_point_st const * data_points, unsigned int nb_data_points, void *pv )
{
	unsigned int index;

	cliPrintf( pv, "parameters are:\n" );

	for (index=0; index < nb_data_points; index++ )
	{
		cliPrintf( pv, "    %20s", data_points[index].name );
	}
}

/*
	handleStandardCommand:
	handle a standard CLI command of the form:
		<name> ?
		<name> <instance> <parameter> ?
		<name> <instance> <parameter> <value|!>
*/
int handleStandardCommand( run_command_data_st const * command_context,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					config_data_point_st const * data_points,
					unsigned int const nb_data_points
					)
{
	void * const pctx = command_context->pctx;
	int const argc = command_context->argc;
	char * * const argv = command_context->argv;
	int result = -1;

	if ( argc == 2 && strcmp( argv[1], "?" ) == 0 )
	{
		/* display all parameter names */
		print_parameter_names( data_points, nb_data_points, pctx );
		result = 0;
	}
#if 0
	else if ( argc == 3 && && strcmp( argv[1], "?" ) == 0 )
	{
		/* print all parameter values for a given instance */
		unsigned int index;
		unsigned int instance = atoi( argv[1] );

		if ( instance < nb_configurations )
		{
			cliPrintf( pctx, "%s %d parameter values:\n", argv[0], instance );
			for (index=0; index < nb_data_points; index++ )
			{
				cliPrintf( pctx, "%20s: ", data_points[index]->name );
				print_configuration_data_point( (char *)pcfg + (instance*configuration_size), data_points[index], pctx );
				cliPrintf( pctx, "\n", data_points[index]->name );
			}
		}
	}
#endif
	else if ( argc > 3 )
	{
		unsigned int instance = atoi( argv[1] );

		if ( instance < nb_configurations )
		{
			if ( strcmp( argv[3], "?" ) == 0 )
			{
				if ( print_config_value( (char *)pcfg + (instance*configuration_size),
											data_points,
											nb_data_points,
											argv[2],
											pctx ) == true )
				{
					result = 0;
				}
			}
			else
			{
				/* write the new value */
				if ( assign_config_value( (char *)pcfg + (instance*configuration_size),
											default_configuration,
											data_points,
											nb_data_points,
											argv[2],
											argv[3] ) == true )
				{
					result = 0;
				}
			}
		}
	}

	if ( result == -1 )
	{
		cliPrintf( pctx, "Format: %s <0 -> %d> <parameter> <value|!>\n", argv[0], nb_configurations-1 );
	}

	return result;
}

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>
#include <config_structure.h>
#include <cmds.h>

typedef struct run_command_data_st
{
	void *pctx;
	int argc;
	char **argv;
} run_command_data_st;

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

	return poll_groups( poll_id_run_command, &pv, 0 );
}

/* called from the code repsonsible for the various commands from within the poll_id_run_command poll_groups() call. */
int runCommandHandler( command_st const * commands, uint32_t nb_commands, void *pv )
{
	command_st const * pcmd;
	int result;
	run_command_data_st *pcmd_data = pv;

	if ( (pcmd=findCommand(commands, nb_commands, pcmd_data->argv[0])) != NULL )
		result = pcmd->handler( pcmd_data->argc, pcmd_data->argv, pcmd_data->pctx );
	else
		result = -1;

	return result;
}


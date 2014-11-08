#ifndef __CMDS_H__
#define __CMDS_H__

#include <config_structure.h>

typedef struct command_st
{
	char const * name;		   /* Name of command */
	int (* handler)(int argc, char *argv[], void *p);
} command_st;

typedef struct run_command_data_st
{
	void *pctx;
	int argc;
	char **argv;
} run_command_data_st;

int runCommand( int argc, char **argv, void *pv );
int runCommandHandler( command_st const * const commands, uint32_t nb_commands, void *pv );
int handleCommand( run_command_data_st const * command_context,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					config_data_point_st const * data_points,
					unsigned int const nb_data_points
					);

#endif /* __CMDS_H__ */

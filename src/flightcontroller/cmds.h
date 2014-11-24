#ifndef __CMDS_H__
#define __CMDS_H__

#include <config_structure.h>

typedef struct run_command_data_st run_command_data_st;
typedef struct non_config_run_command_data_st non_config_run_command_data_st;
typedef char const * (*ParameterNameLookup)( unsigned int parameterID );

typedef struct command_st
{
	configuration_id_t	group_id;
	char 				const * name;
	int 				(* handler)(run_command_data_st *p);
	void				* configuration;
	unsigned int		nb_configuration_instance;
	unsigned int		configuration_size;
	void				const * default_configuration;
	parameterConfig_st  const * parameterConfigs;
	unsigned int		nbParameterConfigs;
	ParameterNameLookup ParameterNameLookupCB;
} command_st;

typedef struct non_config_command_st
{
	char 				const * name;
	int 				(* handler)(non_config_run_command_data_st *p);
} non_config_command_st;

struct run_command_data_st
{
	void *cliCtx;
	int argc;
	char **argv;
	command_st const * command;
	bool handled;
};

struct non_config_run_command_data_st
{
	void *cliCtx;
	int argc;
	char **argv;
	non_config_command_st const * command;
};

typedef struct help_command_data_st
{
	void *cliCtx;
	int argc;
	char **argv;
} help_command_data_st;

int getLengthOfData( config_data_types_t data_type, void const * pcfg );

int runCommand( int argc, char **argv, void *cliCtx );
int runCommandHandler( command_st const * const commands, uint32_t nb_commands, void *pv );
int runNonConfigCommandHandler( non_config_command_st const * commands, uint32_t nb_commands, void *pv );
int idCommandHandler( command_st const * commands, uint32_t nb_commands, void *pv );
int idNonConfigCommandHandler( non_config_command_st const * commands, uint32_t nb_commands, void *pv );

int handleStandardCommand( run_command_data_st const * command_context );
bool savedParameterValueMatchesCurrentValue( void const *psaved,
												void const * pCurrentValue,
												parameterConfig_st const * parameterConfig );

bool currentParameterValueMatchesDefaultValue( void const * pconfig_data,
														void const * pdefault_data,
														parameterConfig_st const * parameterConfig );

command_st const *findCommandFromID( command_st const *commands,
										unsigned int nb_commands,
										configuration_id_t command_id );

parameterConfig_st const * findDataPointFromParameterID( parameterConfig_st const * parameterConfigs,
																	unsigned int const nbParameterConfigs,
																	unsigned int parameterID );

void printParameterValue( void const * pconfig_data,
											parameterConfig_st const * parameterConfig,
											void *printfpv );

bool assignSavedParameterValue( void const * const saved_data, config_data_types_t saved_data_type, void * const parameter, parameterConfig_st const * const parameterConfig );

#endif /* __CMDS_H__ */

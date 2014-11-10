#ifndef __CMDS_H__
#define __CMDS_H__

#include <config_structure.h>

typedef struct run_command_data_st
{
	void *cliCtx;
	int argc;
	char **argv;
} run_command_data_st;

typedef struct command_st
{
	configuration_id_t	group_id;
	char 				const * name;
	int 				(* handler)(run_command_data_st *p);
} command_st;

typedef char const * (*ParameterNameLookup)( unsigned int parameterID );

int getLengthOfData( config_data_types_t data_type, void const * pcfg );

int runCommand( int argc, char **argv, void *cliCtx );
int runCommandHandler( command_st const * const commands, uint32_t nb_commands, void *pv );
int handleStandardCommand( run_command_data_st const * command_context,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					config_data_point_st const * data_points,
					unsigned int const nb_data_points,
					ParameterNameLookup ParameterNameLookupCB
					);
bool savedParameterValueMatchesCurrentValue( void const *psaved,
												void const * pCurrentValue,
												config_data_point_st const * data_point );

bool currentParameterValueMatchesDefaultValue( void const * pconfig_data,
														void const * pdefault_data,
														config_data_point_st const * data_point );

command_st const *findCommandFromID( command_st const *commands,
										unsigned int nb_commands,
										configuration_id_t command_id );

config_data_point_st const * findDataPointFromParameterID( config_data_point_st const * data_points,
																	unsigned int const nb_data_points,
																	unsigned int parameterID );

void printParameterValue( void const * pconfig_data,
											config_data_point_st const * pconfig,
											void *printfpv );

#endif /* __CMDS_H__ */

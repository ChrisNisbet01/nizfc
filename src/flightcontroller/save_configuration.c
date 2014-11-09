#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>
#include <config_structure.h>
#include <cmds.h>
#include <cli.h>
#include <configuration.h>

static int save_command( run_command_data_st *pcommand );

static const command_st config_commands[] =
{
	{ configuration_id_save,     .handler = save_command	}
};

/*
	save_command:
	A command to save all configuration.
	A save_config poll is made, which should result in all supporintg code calling the save_config
	function for all of their configuration data.
*/
static int save_command( run_command_data_st *pcommand )
{
	int result = -1;

	if ( initConfigurationSave() == false )
	{
		cliPrintf( pcommand->cliCtx, "\nError initialising config save");
		goto done;
	}

	poll_groups( poll_id_save_configuration, pcommand, 1 );

	if ( completeConfigurationSave() == false )
	{
		cliPrintf( pcommand->cliCtx, "\nError completing config save");
		goto done;
	}

	result = 0;

done:

	return result;
}

int config_poll_handler( poll_id_t poll_id, void *pv )
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

static bool current_value_matches_default_value( void const * pconfig_data,
														void const * pdefault_data,
														config_data_point_st const * data_point )
{
	bool areSame = false;

	switch (data_point->data_type)
	{
		case config_data_type_boolean:
		case config_data_type_int8:
		case config_data_type_uint8:
		case config_data_type_enum:
			areSame = *(int8_t *)pconfig_data == *(int8_t *)pdefault_data;
			break;
		case config_data_type_int16:
		case config_data_type_uint16:
			areSame = *(int16_t *)pconfig_data == *(int16_t *)pdefault_data;
			break;
		case config_data_type_int32:
		case config_data_type_uint32:
			areSame = *(int32_t *)pconfig_data == *(int32_t *)pdefault_data;
			break;
		case config_data_type_float:
			areSame = *(float *)pconfig_data == *(float *)pdefault_data;
			break;
		case config_data_type_string:
			areSame = strcasecmp( (char *)pconfig_data, (char *)pdefault_data ) == 0;
			break;
	}

	return areSame;
}

int save_configuration( run_command_data_st const * command_context,
					configuration_id_t configuration_id,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					config_data_point_st const * data_points,
					unsigned int const nb_data_points
					)
{
	/*
		For each data point in each configuration, we write out the current value,
		but only if it differs from the default value.
	*/
	unsigned int configuration_index, data_point_index;

	for ( configuration_index = 0; configuration_index < nb_configurations; configuration_index++ )
	{
		unsigned int offset_to_correct_configuration_data = (configuration_index*configuration_size);

		for ( data_point_index = 0; data_point_index < nb_data_points; data_point_index++ )
		{
			if (current_value_matches_default_value( (char *)pcfg + offset_to_correct_configuration_data,
															default_configuration,
															&data_points[data_point_index] ) == false )
			{
				cliPrintf( command_context->cliCtx, "need to save" );
#if 0
				save_data_point( configuration_id,
									(char *)pcfg + offset_to_correct_configuration_data,
									default_configuration,
									&data_point[data_point_index] );
#endif
			}
			else
			{
				cliPrintf( command_context->cliCtx, "no need to save" );
			}
		}
	}

	return 0;
}

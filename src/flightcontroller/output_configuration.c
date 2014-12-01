#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f30x_tim.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <coos.h>
#include <utils.h>
#include <polling.h>
#include <cmds.h>
#include <configuration_commands.h>
#include <cli.h>
#include <output_configuration.h>


static int output_command( run_command_data_st *pcommand );
static char const * outputParameterNameLookup( unsigned int parameterID );

output_configuration_st output_configuration[NB_OUTPUT_CONFIGURATIONS];

static const output_configuration_st default_output_configuration =
{
	.pwmrate = 400 /* Hz */
};

typedef enum output_parameter_id_t
{
	output_parameter_id_mode = 0
} output_parameter_id_t;


static char const * const output_parameter_name_mappings[] =
{
	[output_parameter_id_mode] = "pwmrate"
};

static const parameterConfig_st output_config_parameterConfigs[] =
{
	{
	.parameter_id = output_parameter_id_mode,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(output_configuration_st, pwmrate),
	}
};

static const command_st output_commands[] =
{
	{
		.group_id = configuration_id_output,
		.name = "output",
		.handler = output_command,
		.configuration = output_configuration,
		.nb_configuration_instance = ARRAY_SIZE(output_configuration),
		.configuration_size = sizeof(output_configuration[0]),
		.default_configuration = &default_output_configuration,
		.parameterConfigs = output_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(output_config_parameterConfigs),
		.ParameterNameLookupCB = outputParameterNameLookup
	}
};

static char const * outputParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(output_parameter_name_mappings) )
		return output_parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int output_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initOutputConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(output_configuration); index++ )
		memcpy( &output_configuration[index], &default_output_configuration, sizeof output_configuration[index] );
}

int outputPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( output_commands, ARRAY_SIZE(output_commands), pv );
			break;
		case poll_id_initialise:
			initOutputConfiguration();
			break;
		case poll_id_run_command:
			result = runCommandHandler( output_commands, ARRAY_SIZE(output_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			result = saveParameterValues( pv,
								output_commands,
								ARRAY_SIZE(output_commands)
								);
			break;
		}
		case poll_id_show_configuration:
			result = printParametersHandler( pv,
								output_commands,
								ARRAY_SIZE(output_commands),
								output_parameter_name_mappings
								);
			break;
		case poll_id_load_configuration:
			result = loadParametersHandler( pv,
								output_commands,
								ARRAY_SIZE(output_commands)
								);
			break;
		default:
			break;
	}

	return result;
}



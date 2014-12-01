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
#include <attitude_configuration.h>


static int attitude_command( run_command_data_st *pcommand );
static char const * attitudeParameterNameLookup( unsigned int parameterID );

attitude_configuration_st attitude_configuration[NB_ATTITUDE_CONFIGURATIONS];

// TODO: decent defaults
static const attitude_configuration_st default_attitude_configuration =
{
	.roll_lpf = 200,
	.pitch_lpf = 200,
	.heading_lpf = 100
};

typedef enum attitude_parameter_id_t
{
	parameter_id_roll_lpf = 0,
	parameter_id_pitch_lpf = 1,
	parameter_id_heading_lpf = 2
} attitude_parameter_id_t;

static char const * const parameter_name_mappings[] =
{
	[parameter_id_roll_lpf] = "rlpf",
	[parameter_id_pitch_lpf] = "plpf",
	[parameter_id_heading_lpf] = "hlpf"
};

static const parameterConfig_st attitude_config_parameterConfigs[] =
{
	{
	.parameter_id = parameter_id_roll_lpf,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(attitude_configuration_st, roll_lpf),
	},
	{
	.parameter_id = parameter_id_pitch_lpf,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(attitude_configuration_st, pitch_lpf),
	},
	{
	.parameter_id = parameter_id_heading_lpf,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(attitude_configuration_st, heading_lpf),
	},
};

static const command_st attitude_commands[] =
{
	{
		.group_id = configuration_id_attitude,
		.name = "attitude",
		.handler = attitude_command,
		.configuration = attitude_configuration,
		.nb_configuration_instance = ARRAY_SIZE(attitude_configuration),
		.configuration_size = sizeof(attitude_configuration[0]),
		.default_configuration = &default_attitude_configuration,
		.parameterConfigs = attitude_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(attitude_config_parameterConfigs),
		.ParameterNameLookupCB = attitudeParameterNameLookup
	}
};

static char const * attitudeParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(parameter_name_mappings) )
		return parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int attitude_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initAttitudeConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(attitude_configuration); index++ )
		memcpy( &attitude_configuration[index], &default_attitude_configuration, sizeof attitude_configuration[index] );
}

int attitudePollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( attitude_commands, ARRAY_SIZE(attitude_commands), pv );
			break;
		case poll_id_initialise:
			initAttitudeConfiguration();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( attitude_commands, ARRAY_SIZE(attitude_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			(void)saveParameterValues( pv,
								attitude_commands,
								ARRAY_SIZE(attitude_commands)
								);
			result = poll_result_ok;
			break;
		}
		case poll_id_show_configuration:
			(void)printParametersHandler( pv,
								attitude_commands,
								ARRAY_SIZE(attitude_commands),
								parameter_name_mappings
								);
			result = poll_result_ok;
			break;
		case poll_id_load_configuration:
			(void)loadParametersHandler( pv,
								attitude_commands,
								ARRAY_SIZE(attitude_commands)
								);
			result = poll_result_ok;
			break;
		default:
			break;
	}

	return result;
}



#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <coos.h>
#include <utils.h>
#include <polling.h>
#include <cmds.h>
#include <configuration_commands.h>
#include <cli.h>
#include <receiver.h>
#include <receiver_configuration.h>


static int receiver_command( run_command_data_st *pcommand );
static char const * receiverParameterNameLookup( unsigned int parameterID );

static const enum_mapping_st receiver_mode_mappings[] =
{
	{
		.name = "ppm",
		.value = (int8_t)receiver_mode_ppm
	},
	{
		.name = "pwm",
		.value = (int8_t)receiver_mode_pwm
	}
};

receiver_configuration_st receiver_configuration[NB_RECEIVER_CONFIGURATIONS];
static const receiver_configuration_st default_receiver_configuration =
{
	.mode = (int8_t)receiver_mode_pwm
};

typedef enum receiver_parameter_id_t
{
	receiver_parameter_id_mode = 0
} receiver_parameter_id_t;


static char const * const receiver_parameter_name_mappings[] =
{
	[receiver_parameter_id_mode] = "mode"
};

static const parameterConfig_st receiver_config_parameterConfigs[] =
{
	{
	.parameter_id = receiver_parameter_id_mode,
	.data_type = config_data_type_enum,
	.offsetToData = offsetof(receiver_configuration_st, mode),
	.type_specific.enum_data.mappings = receiver_mode_mappings,
	.type_specific.enum_data.num_mappings = ARRAY_SIZE(receiver_mode_mappings)
	}
};

static const command_st receiver_commands[] =
{
	{
		.group_id = configuration_id_receiver,
		.name = "rx",
		.handler = receiver_command,
		.configuration = receiver_configuration,
		.nb_configuration_instance = ARRAY_SIZE(receiver_configuration),
		.configuration_size = sizeof(receiver_configuration[0]),
		.default_configuration = &default_receiver_configuration,
		.parameterConfigs = receiver_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(receiver_config_parameterConfigs),
		.ParameterNameLookupCB = receiverParameterNameLookup
	}
};

static char const * receiverParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(receiver_parameter_name_mappings) )
		return receiver_parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int receiver_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initReceiverConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(receiver_configuration); index++ )
		memcpy( &receiver_configuration[index], &default_receiver_configuration, sizeof receiver_configuration[index] );
}

int receiverPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( receiver_commands, ARRAY_SIZE(receiver_commands), pv );
			break;
		case poll_id_initialise:
			initReceiverConfiguration();
			initReceiver();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( receiver_commands, ARRAY_SIZE(receiver_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			result = saveParameterValues( pv,
								receiver_commands,
								ARRAY_SIZE(receiver_commands)
								);
			break;
		}
		case poll_id_show_configuration:
			result = printParametersHandler( pv,
								receiver_commands,
								ARRAY_SIZE(receiver_commands),
								receiver_parameter_name_mappings
								);
			break;
		case poll_id_load_configuration:
			result = loadParametersHandler( pv,
								receiver_commands,
								ARRAY_SIZE(receiver_commands)
								);
			break;
		default:
			break;
	}

	return result;
}



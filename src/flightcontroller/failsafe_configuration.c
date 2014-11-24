#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <utils.h>
#include <polling.h>
#include <cmds.h>
#include <configuration_commands.h>
#include <failsafe_configuration.h>
#include <failsafe.h>

static int failsafe_command( run_command_data_st *pcommand );
static char const * failsafeParameterNameLookup( unsigned int parameterID );

failsafe_configuration_st failsafe_configuration[NB_FAILSAFE_CONFIGURATIONS];

static const failsafe_configuration_st default_failsafe_configuration =
{
	.enabled = false,
	.nbRequiredChannels = 4,
	.maxQuietTimeMs = 1000,
	.motorOutput = 1000
};

typedef enum failsafe_parameter_id_t
{
	failsafe_parameter_id_enabled = 0,
	failsafe_parameter_id_nbRequiredChannels = 1,
	failsafe_parameter_id_maxQuietTimems = 2,
	failsafe_parameter_id_motorOutput = 3
} failsafe_parameter_id_t;


static char const * const failsafe_parameter_name_mappings[] =
{
	[failsafe_parameter_id_enabled] = "enabled",
	[failsafe_parameter_id_nbRequiredChannels] = "nbchannels",
	[failsafe_parameter_id_maxQuietTimems] = "maxquiet",
	[failsafe_parameter_id_motorOutput] = "motor"
};

static const parameterConfig_st failsafe_config_parameterConfigs[] =
{
	{
	.parameter_id = failsafe_parameter_id_enabled,
	.data_type = config_data_type_boolean,
	.offsetToData = offsetof(failsafe_configuration_st, enabled)
	},
	{
	.parameter_id = failsafe_parameter_id_nbRequiredChannels,
	.data_type = config_data_type_uint8,
	.offsetToData = offsetof(failsafe_configuration_st, nbRequiredChannels)
	},
	{
	.parameter_id = failsafe_parameter_id_maxQuietTimems,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(failsafe_configuration_st, maxQuietTimeMs)
	},
	{
	.parameter_id = failsafe_parameter_id_motorOutput,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(failsafe_configuration_st, motorOutput)
	}
};

static const command_st failsafe_commands[] =
{
	{
		.group_id = configuration_id_failsafe,
		.name = "failsafe",
		.handler = failsafe_command,
		.configuration = failsafe_configuration,
		.nb_configuration_instance = ARRAY_SIZE(failsafe_configuration),
		.configuration_size = sizeof(failsafe_configuration[0]),
		.default_configuration = &default_failsafe_configuration,
		.parameterConfigs = failsafe_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(failsafe_config_parameterConfigs),
		.ParameterNameLookupCB = failsafeParameterNameLookup
	}
};

static char const * failsafeParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(failsafe_parameter_name_mappings) )
		return failsafe_parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int failsafe_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initFailsafeConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(failsafe_configuration); index++ )
		memcpy( &failsafe_configuration[index], &default_failsafe_configuration, sizeof failsafe_configuration[index] );
}

int failsafePollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( failsafe_commands, ARRAY_SIZE(failsafe_commands), pv );
			break;
		case poll_id_initialise:
			initFailsafeConfiguration();
			break;
		case poll_id_run_command:
			result = runCommandHandler( failsafe_commands, ARRAY_SIZE(failsafe_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			result = saveParameterValues( pv,
								failsafe_commands,
								ARRAY_SIZE(failsafe_commands)
								);
			break;
		}
		case poll_id_show_configuration:
			result = printParametersHandler( pv,
								failsafe_commands,
								ARRAY_SIZE(failsafe_commands),
								failsafe_parameter_name_mappings
								);
			break;
		case poll_id_load_configuration:
			result = loadParametersHandler( pv,
								failsafe_commands,
								ARRAY_SIZE(failsafe_commands)
								);
			break;
		default:
			break;
	}

	return result;
}



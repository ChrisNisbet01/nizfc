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
#include <yaw_configuration.h>


static int yaw_command( run_command_data_st *pcommand );
static char const * yawParameterNameLookup( unsigned int parameterID );

yaw_configuration_st yaw_configuration[NB_YAW_CONFIGURATIONS];

// TODO: decent defaults
static const yaw_configuration_st default_yaw_configuration =
{
	.kP = 2.0f,
	.kI = 0.5f,
	.kD = 0.0f,
	.integralLimit = 100.0f,
	.dLimit = 0.0f,
	.pidRange = 250.0f,
	.maxRate = 100.0f
};

typedef enum yaw_parameter_id_t
{
	parameter_id_kP = 0,
	parameter_id_kI = 1,
	parameter_id_kD = 2,
	parameter_id_integralLimit = 3,
	parameter_id_dLimit = 4,
	parameter_id_pidRange = 5,
	parameter_id_maxRate = 6
} yaw_parameter_id_t;

static char const * const parameter_name_mappings[] =
{
	[parameter_id_kP] = "kp",
	[parameter_id_kI] = "ki",
	[parameter_id_kD] = "kd",
	[parameter_id_integralLimit] = "ilimit",
	[parameter_id_dLimit] = "dlimit",
	[parameter_id_pidRange] = "pidrange",
	[parameter_id_maxRate] = "maxrate"
};

static const parameterConfig_st yaw_config_parameterConfigs[] =
{
	{
	.parameter_id = parameter_id_kP,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(yaw_configuration_st, kP),
	},
	{
	.parameter_id = parameter_id_kI,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(yaw_configuration_st, kI),
	},
	{
	.parameter_id = parameter_id_kD,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(yaw_configuration_st, kD),
	},
	{
	.parameter_id = parameter_id_integralLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(yaw_configuration_st, integralLimit),
	},
	{
	.parameter_id = parameter_id_dLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(yaw_configuration_st, dLimit),
	},
	{
	.parameter_id = parameter_id_pidRange,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(yaw_configuration_st, pidRange),
	},
	{
	.parameter_id = parameter_id_maxRate,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(yaw_configuration_st, maxRate),
	}
};

static const command_st yaw_commands[] =
{
	{
		.group_id = configuration_id_yaw,
		.name = "yaw",
		.handler = yaw_command,
		.configuration = yaw_configuration,
		.nb_configuration_instance = ARRAY_SIZE(yaw_configuration),
		.configuration_size = sizeof(yaw_configuration[0]),
		.default_configuration = &default_yaw_configuration,
		.parameterConfigs = yaw_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(yaw_config_parameterConfigs),
		.ParameterNameLookupCB = yawParameterNameLookup
	}
};

static char const * yawParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(parameter_name_mappings) )
		return parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int yaw_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initYawConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(yaw_configuration); index++ )
		memcpy( &yaw_configuration[index], &default_yaw_configuration, sizeof yaw_configuration[index] );
}

int yawPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( yaw_commands, ARRAY_SIZE(yaw_commands), pv );
			break;
		case poll_id_initialise:
			initYawConfiguration();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( yaw_commands, ARRAY_SIZE(yaw_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			(void)saveParameterValues( pv,
								yaw_commands,
								ARRAY_SIZE(yaw_commands)
								);
			result = poll_result_ok;
			break;
		}
		case poll_id_show_configuration:
			(void)printParametersHandler( pv,
								yaw_commands,
								ARRAY_SIZE(yaw_commands),
								parameter_name_mappings
								);
			result = poll_result_ok;
			break;
		case poll_id_load_configuration:
			(void)loadParametersHandler( pv,
								yaw_commands,
								ARRAY_SIZE(yaw_commands)
								);
			result = poll_result_ok;
			break;
		default:
			break;
	}

	return result;
}



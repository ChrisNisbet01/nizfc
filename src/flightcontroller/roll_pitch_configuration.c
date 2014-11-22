#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f30x_tim.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f3_discovery.h>
#include <coos.h>
#include <utils.h>
#include <polling.h>
#include <cmds.h>
#include <configuration_commands.h>
#include <cli.h>
#include <roll_pitch_configuration.h>


static int roll_command( run_command_data_st *pcommand );
static int pitch_command( run_command_data_st *pcommand );
static int yaw_command( run_command_data_st *pcommand );
static char const * rollPitchParameterNameLookup( unsigned int parameterID );

roll_pitch_configuration_st roll_configuration[NB_ROLL_CONFIGURATIONS];
roll_pitch_configuration_st pitch_configuration[NB_PITCH_CONFIGURATIONS];
roll_pitch_configuration_st yaw_configuration[NB_YAW_CONFIGURATIONS];
static const roll_pitch_configuration_st default_roll_pitch_configuration =
{
	.lpf_factor = 0.01f
};

typedef enum roll_pitch_parameter_id_t
{
	parameter_id_lpf = 0,
	parameter_id_kP = 1,
	parameter_id_kI = 2,
	parameter_id_kD = 3,
	parameter_id_integralLimit = 4,
	parameter_id_dLimit = 5,
	parameter_id_pidRange = 6
} roll_pitch_parameter_id_t;

static char const * const parameter_name_mappings[] =
{
	[parameter_id_lpf] = "lpf",
	[parameter_id_kP] = "kp",
	[parameter_id_kI] = "ki",
	[parameter_id_kD] = "kd",
	[parameter_id_integralLimit] = "ilimit",
	[parameter_id_dLimit] = "dlimit",
	[parameter_id_pidRange] = "pidrange"
};

static const parameterConfig_st roll_pitch_config_parameterConfigs[] =
{
	{
	.parameter_id = parameter_id_lpf,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, lpf_factor),
	},
	{
	.parameter_id = parameter_id_kP,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, kP),
	},
	{
	.parameter_id = parameter_id_kI,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, kI),
	},
	{
	.parameter_id = parameter_id_kD,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, kD),
	},
	{
	.parameter_id = parameter_id_integralLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, integralLimit),
	},
	{
	.parameter_id = parameter_id_dLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, dLimit),
	},
	{
	.parameter_id = parameter_id_pidRange,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, pidRange),
	}
};

static const command_st roll_pitch_commands[] =
{
	{
		.group_id = configuration_id_roll,
		.name = "roll",
		.handler = roll_command,
		.configuration = roll_configuration,
		.nb_configuration_instance = ARRAY_SIZE(roll_configuration),
		.configuration_size = sizeof(roll_configuration[0]),
		.default_configuration = &default_roll_pitch_configuration,
		.parameterConfigs = roll_pitch_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(roll_pitch_config_parameterConfigs),
		.ParameterNameLookupCB = rollPitchParameterNameLookup
	},
	{
		.group_id = configuration_id_pitch,
		.name = "pitch",
		.handler = pitch_command,
		.configuration = pitch_configuration,
		.nb_configuration_instance = ARRAY_SIZE(pitch_configuration),
		.configuration_size = sizeof(pitch_configuration[0]),
		.default_configuration = &default_roll_pitch_configuration,
		.parameterConfigs = roll_pitch_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(roll_pitch_config_parameterConfigs),
		.ParameterNameLookupCB = rollPitchParameterNameLookup
	},
	{
		.group_id = configuration_id_yaw,
		.name = "yaw",
		.handler = yaw_command,
		.configuration = yaw_configuration,
		.nb_configuration_instance = ARRAY_SIZE(yaw_configuration),
		.configuration_size = sizeof(yaw_configuration[0]),
		.default_configuration = &default_roll_pitch_configuration,
		.parameterConfigs = roll_pitch_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(roll_pitch_config_parameterConfigs),
		.ParameterNameLookupCB = rollPitchParameterNameLookup
	}
};

static char const * rollPitchParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(parameter_name_mappings) )
		return parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int roll_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static int pitch_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static int yaw_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initRollPitchConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(roll_configuration); index++ )
		memcpy( &roll_configuration[index], &default_roll_pitch_configuration, sizeof roll_configuration[index] );
	for (index = 0; index < ARRAY_SIZE(pitch_configuration); index++ )
		memcpy( &pitch_configuration[index], &default_roll_pitch_configuration, sizeof pitch_configuration[index] );
	for (index = 0; index < ARRAY_SIZE(yaw_configuration); index++ )
		memcpy( &yaw_configuration[index], &default_roll_pitch_configuration, sizeof yaw_configuration[index] );
}

int rollPitchPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( roll_pitch_commands, ARRAY_SIZE(roll_pitch_commands), pv );
			break;
		case poll_id_initialise:
			initRollPitchConfiguration();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( roll_pitch_commands, ARRAY_SIZE(roll_pitch_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			(void)saveParameterValues( pv,
								roll_pitch_commands,
								ARRAY_SIZE(roll_pitch_commands)
								);
			result = poll_result_ok;
			break;
		}
		case poll_id_show_configuration:
			(void)printParametersHandler( pv,
								roll_pitch_commands,
								ARRAY_SIZE(roll_pitch_commands),
								parameter_name_mappings
								);
			result = poll_result_ok;
			break;
		case poll_id_load_configuration:
			(void)loadParametersHandler( pv,
								roll_pitch_commands,
								ARRAY_SIZE(roll_pitch_commands)
								);
			result = poll_result_ok;
			break;
		default:
			break;
	}

	return result;
}



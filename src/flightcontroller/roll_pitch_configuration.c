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

roll_pitch_configuration_st roll_pitch_configuration[NB_ROLL_PITCH_CONFIGURATIONS];
static const roll_pitch_configuration_st default_roll_pitch_configuration =
{
	.roll_lpf_factor = 0.8f,
	.pitch_lpf_factor = 0.8f,
};

typedef enum roll_parameter_id_t
{
	roll_parameter_id_lpf = 0,
	roll_parameter_id_kP = 1,
	roll_parameter_id_kI = 2,
	roll_parameter_id_kD = 3,
	roll_parameter_id_integralLimit = 4,
	roll_parameter_id_dLimit = 5,
	roll_parameter_id_pidRange = 6
} roll_parameter_id_t;

typedef enum pitch_parameter_id_t
{
	pitch_parameter_id_lpf = 0
} pitch_parameter_id_t;


static char const * const roll_parameter_name_mappings[] =
{
	[roll_parameter_id_lpf] = "lpf",
	[roll_parameter_id_kP] = "kp",
	[roll_parameter_id_kI] = "ki",
	[roll_parameter_id_kD] = "kd",
	[roll_parameter_id_integralLimit] = "ilimit",
	[roll_parameter_id_dLimit] = "dlimit",
	[roll_parameter_id_pidRange] = "pidrange"
};

static char const * const pitch_parameter_name_mappings[] =
{
	[pitch_parameter_id_lpf] = "lpf"
};

static const parameterConfig_st roll_config_parameterConfigs[] =
{
	{
	.parameter_id = roll_parameter_id_lpf,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, roll_lpf_factor),
	},
	{
	.parameter_id = roll_parameter_id_kP,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, kP),
	},
	{
	.parameter_id = roll_parameter_id_kI,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, kI),
	},
	{
	.parameter_id = roll_parameter_id_kD,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, kD),
	},
	{
	.parameter_id = roll_parameter_id_integralLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, integralLimit),
	},
	{
	.parameter_id = roll_parameter_id_dLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, dLimit),
	},
	{
	.parameter_id = roll_parameter_id_pidRange,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, pidRange),
	}
};

static const parameterConfig_st pitch_config_parameterConfigs[] =
{
	{
	.parameter_id = pitch_parameter_id_lpf,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(roll_pitch_configuration_st, pitch_lpf_factor),
	}
};

static const command_st roll_pitch_commands[] =
{
	{ .group_id = configuration_id_roll,  .name = "roll",  .handler = roll_command	},
	{ .group_id = configuration_id_pitch, .name = "pitch", .handler = pitch_command	}
};

static char const * rollParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(roll_parameter_name_mappings) )
		return roll_parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int roll_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand,
							roll_pitch_configuration,
							ARRAY_SIZE(roll_pitch_configuration),
							sizeof(roll_pitch_configuration[0]),
							&default_roll_pitch_configuration,
							roll_config_parameterConfigs,
							ARRAY_SIZE(roll_config_parameterConfigs),
							rollParameterNameLookup);
}

static char const * pitchParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(pitch_parameter_name_mappings) )
		return pitch_parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int pitch_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand,
							roll_pitch_configuration,
							ARRAY_SIZE(roll_pitch_configuration),
							sizeof(roll_pitch_configuration[0]),
							&default_roll_pitch_configuration,
							pitch_config_parameterConfigs,
							ARRAY_SIZE(pitch_config_parameterConfigs),
							pitchParameterNameLookup);
}

static void initRollPitchConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(roll_pitch_configuration); index++ )
		memcpy( &roll_pitch_configuration[index], &default_roll_pitch_configuration, sizeof roll_pitch_configuration[index] );
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
			break;
		case poll_id_run_command:
			result = runCommandHandler( roll_pitch_commands, ARRAY_SIZE(roll_pitch_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			(void)saveParameterValues( pv,
								configuration_id_roll,
								roll_pitch_configuration,
								ARRAY_SIZE(roll_pitch_configuration),
								sizeof(roll_pitch_configuration[0]),
								&default_roll_pitch_configuration,
								roll_config_parameterConfigs,
								ARRAY_SIZE(roll_config_parameterConfigs) );
			(void)saveParameterValues( pv,
								configuration_id_pitch,
								roll_pitch_configuration,
								ARRAY_SIZE(roll_pitch_configuration),
								sizeof(roll_pitch_configuration[0]),
								&default_roll_pitch_configuration,
								pitch_config_parameterConfigs,
								ARRAY_SIZE(pitch_config_parameterConfigs) );
			result = poll_result_ok;
			break;
		}
		case poll_id_show_configuration:
			(void)printParametersHandler( pv,
								roll_pitch_commands,
								ARRAY_SIZE(roll_pitch_commands),
								roll_pitch_configuration,
								ARRAY_SIZE(roll_pitch_configuration),
								sizeof(roll_pitch_configuration[0]),
								&default_roll_pitch_configuration,
								roll_config_parameterConfigs,
								ARRAY_SIZE(roll_config_parameterConfigs),
								roll_parameter_name_mappings
								);
			(void)printParametersHandler( pv,
								roll_pitch_commands,
								ARRAY_SIZE(roll_pitch_commands),
								roll_pitch_configuration,
								ARRAY_SIZE(roll_pitch_configuration),
								sizeof(roll_pitch_configuration[0]),
								&default_roll_pitch_configuration,
								pitch_config_parameterConfigs,
								ARRAY_SIZE(pitch_config_parameterConfigs),
								pitch_parameter_name_mappings
								);
			result = poll_result_ok;
			break;
		case poll_id_load_configuration:
			result = loadParametersHandler( pv,
								roll_pitch_commands,
								ARRAY_SIZE(roll_pitch_commands),
								roll_pitch_commands,
								ARRAY_SIZE(roll_pitch_commands),
								sizeof(roll_pitch_commands[0]),
								pitch_config_parameterConfigs,
								ARRAY_SIZE(pitch_config_parameterConfigs)
								);
			break;
		default:
			break;
	}

	return result;
}



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
#include <angle_mode_configuration.h>


static int angle_command( run_command_data_st *pcommand );
static char const * angleParameterNameLookup( unsigned int parameterID );

angle_mode_configuration_st angle_mode_configuration[NB_ANGLE_MODE_CONFIGURATIONS];

// TODO: decent defaults
static const angle_mode_configuration_st default_angle_mode_configuration =
{
	.roll_kP = 4.0f,
	.roll_kI = 0.0f,
	.roll_kD = 0.0f,
	.roll_integralLimit = 100.0f,
	.roll_dLimit = 0.0f,
	.roll_pidRange = 500.0f,
	.roll_maxAngle = 50.0f,
	.pitch_kP = 4.0f,
	.pitch_kI = 0.0f,
	.pitch_kD = 0.0f,
	.pitch_integralLimit = 100.0f,
	.pitch_dLimit = 0.0f,
	.pitch_pidRange = 500.0f,
	.pitch_maxAngle = 50.0f
};

typedef enum angle_mode_parameter_id_t
{
	parameter_id_roll_kP = 0,
	parameter_id_roll_kI = 1,
	parameter_id_roll_kD = 2,
	parameter_id_roll_integralLimit = 3,
	parameter_id_roll_dLimit = 4,
	parameter_id_roll_pidRange = 5,
	parameter_id_roll_maxAngle = 6,
	parameter_id_pitch_kP = 7,
	parameter_id_pitch_kI = 8,
	parameter_id_pitch_kD = 9,
	parameter_id_pitch_integralLimit = 10,
	parameter_id_pitch_dLimit = 11,
	parameter_id_pitch_pidRange = 12,
	parameter_id_pitch_maxAngle = 13
} angle_mode_parameter_id_t;

static char const * const parameter_name_mappings[] =
{
	[parameter_id_roll_kP] = "rkp",
	[parameter_id_roll_kI] = "rki",
	[parameter_id_roll_kD] = "rkd",
	[parameter_id_roll_integralLimit] = "rilimit",
	[parameter_id_roll_dLimit] = "rdlimit",
	[parameter_id_roll_pidRange] = "rpidrange",
	[parameter_id_roll_maxAngle] = "rmaxangle",
	[parameter_id_pitch_kP] = "pkp",
	[parameter_id_pitch_kI] = "pki",
	[parameter_id_pitch_kD] = "pkd",
	[parameter_id_pitch_integralLimit] = "pilimit",
	[parameter_id_pitch_dLimit] = "pdlimit",
	[parameter_id_pitch_pidRange] = "ppidrange",
	[parameter_id_pitch_maxAngle] = "pmaxangle"
};

static const parameterConfig_st angle_mode_config_parameterConfigs[] =
{
	{
	.parameter_id = parameter_id_roll_kP,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, roll_kP),
	},
	{
	.parameter_id = parameter_id_roll_kI,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, roll_kI),
	},
	{
	.parameter_id = parameter_id_roll_kD,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, roll_kD),
	},
	{
	.parameter_id = parameter_id_roll_integralLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, roll_integralLimit),
	},
	{
	.parameter_id = parameter_id_roll_dLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, roll_dLimit),
	},
	{
	.parameter_id = parameter_id_roll_pidRange,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, roll_pidRange),
	},
	{
	.parameter_id = parameter_id_roll_maxAngle,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, roll_maxAngle),
	},
	{
	.parameter_id = parameter_id_pitch_kP,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, pitch_kP),
	},
	{
	.parameter_id = parameter_id_pitch_kI,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, pitch_kI),
	},
	{
	.parameter_id = parameter_id_pitch_kD,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, pitch_kD),
	},
	{
	.parameter_id = parameter_id_pitch_integralLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, pitch_integralLimit),
	},
	{
	.parameter_id = parameter_id_pitch_dLimit,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, pitch_dLimit),
	},
	{
	.parameter_id = parameter_id_pitch_pidRange,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, pitch_pidRange),
	},
	{
	.parameter_id = parameter_id_pitch_maxAngle,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(angle_mode_configuration_st, pitch_maxAngle),
	}
};

static const command_st angle_mode_commands[] =
{
	{
		.group_id = configuration_id_angle,
		.name = "angle",
		.handler = angle_command,
		.configuration = angle_mode_configuration,
		.nb_configuration_instance = ARRAY_SIZE(angle_mode_configuration),
		.configuration_size = sizeof(angle_mode_configuration[0]),
		.default_configuration = &default_angle_mode_configuration,
		.parameterConfigs = angle_mode_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(angle_mode_config_parameterConfigs),
		.ParameterNameLookupCB = angleParameterNameLookup
	}
};

static char const * angleParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(parameter_name_mappings) )
		return parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int angle_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initAngleModeConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(angle_mode_configuration); index++ )
		memcpy( &angle_mode_configuration[index], &default_angle_mode_configuration, sizeof angle_mode_configuration[index] );
}

int anglePollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( angle_mode_commands, ARRAY_SIZE(angle_mode_commands), pv );
			break;
		case poll_id_initialise:
			initAngleModeConfiguration();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( angle_mode_commands, ARRAY_SIZE(angle_mode_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			(void)saveParameterValues( pv,
								angle_mode_commands,
								ARRAY_SIZE(angle_mode_commands)
								);
			result = poll_result_ok;
			break;
		}
		case poll_id_show_configuration:
			(void)printParametersHandler( pv,
								angle_mode_commands,
								ARRAY_SIZE(angle_mode_commands),
								parameter_name_mappings
								);
			result = poll_result_ok;
			break;
		case poll_id_load_configuration:
			(void)loadParametersHandler( pv,
								angle_mode_commands,
								ARRAY_SIZE(angle_mode_commands)
								);
			result = poll_result_ok;
			break;
		default:
			break;
	}

	return result;
}



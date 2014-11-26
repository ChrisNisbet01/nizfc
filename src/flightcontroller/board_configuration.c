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
#include <craft_types.h>
#include <board_configuration.h>


static int board_command( run_command_data_st *pcommand );
static char const * boardParameterNameLookup( unsigned int parameterID );

static const enum_mapping_st craft_type_mappings[] =
{
	{
		.name = "quadh",
		.value = (uint8_t)craft_type_quadh
	}
};

board_configuration_st board_configuration[NB_BOARD_CONFIGURATIONS];
static const board_configuration_st default_board_configuration =
{
	.craftType = (uint8_t)craft_type_quadh,
	.boardOrientation[0] = 0.0f,
	.boardOrientation[1] = 0.0f,
	.boardOrientation[2] = 0.0f,
	.debug               = 0
};

typedef enum board_parameter_id_t
{
	board_parameter_id_craftType = 0,
	board_parameter_id_boardOrientationX = 1,
	board_parameter_id_boardOrientationY = 2,
	board_parameter_id_boardOrientationZ = 3,
	board_parameter_id_debug = 4
} board_parameter_id_t;


static char const * const board_parameter_name_mappings[] =
{
	[board_parameter_id_craftType]         = "type",
	[board_parameter_id_boardOrientationX] = "roll",
	[board_parameter_id_boardOrientationY] = "pitch",
	[board_parameter_id_boardOrientationZ] = "yaw",
	[board_parameter_id_debug]             = "debug"
};

static const parameterConfig_st board_config_parameterConfigs[] =
{
	{
	.parameter_id = board_parameter_id_craftType,
	.data_type = config_data_type_enum,
	.offsetToData = offsetof(board_configuration_st, craftType),
	.type_specific.enum_data.mappings = craft_type_mappings,
	.type_specific.enum_data.num_mappings = ARRAY_SIZE(craft_type_mappings)
	},
	{
	.parameter_id = board_parameter_id_boardOrientationX,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(board_configuration_st, boardOrientation[0]),
	},
	{
	.parameter_id = board_parameter_id_boardOrientationY,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(board_configuration_st, boardOrientation[1]),
	},
	{
	.parameter_id = board_parameter_id_boardOrientationZ,
	.data_type = config_data_type_float,
	.offsetToData = offsetof(board_configuration_st, boardOrientation[2]),
	},
	{
	.parameter_id = board_parameter_id_debug,
	.data_type = config_data_type_uint32,
	.offsetToData = offsetof(board_configuration_st, debug),
	}
};

static const command_st board_commands[] =
{
	{
		.group_id = configuration_id_board,
		.name = "board",
		.handler = board_command,
		.configuration = board_configuration,
		.nb_configuration_instance = ARRAY_SIZE(board_configuration),
		.configuration_size = sizeof(board_configuration[0]),
		.default_configuration = &default_board_configuration,
		.parameterConfigs = board_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(board_config_parameterConfigs),
		.ParameterNameLookupCB = boardParameterNameLookup
	}
};

static char const * boardParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(board_parameter_name_mappings) )
		return board_parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int board_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initBoardConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(board_configuration); index++ )
		memcpy( &board_configuration[index], &default_board_configuration, sizeof board_configuration[index] );
}

int boardPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( board_commands, ARRAY_SIZE(board_commands), pv );
			break;
		case poll_id_initialise:
			initBoardConfiguration();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( board_commands, ARRAY_SIZE(board_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			result = saveParameterValues( pv,
								board_commands,
								ARRAY_SIZE(board_commands)
								);
			break;
		}
		case poll_id_show_configuration:
			result = printParametersHandler( pv,
								board_commands,
								ARRAY_SIZE(board_commands),
								board_parameter_name_mappings
								);
			break;
		case poll_id_load_configuration:
			result = loadParametersHandler( pv,
								board_commands,
								ARRAY_SIZE(board_commands)
								);
			break;
		default:
			break;
	}

	return result;
}



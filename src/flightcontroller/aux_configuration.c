#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <utils.h>
#include <polling.h>
#include <cmds.h>
#include <configuration_commands.h>
#include <receiver.h>
#include <aux_configuration.h>
#include <pid_control.h>
#include <leds.h>

#define NB_AUX_CONFIGURATIONS	32u

typedef struct aux_configuration_st
{
	uint8_t		channel;
	uint8_t		function;
	uint16_t	channelLow;
	uint16_t	channelHigh;
} aux_configuration_st;

static aux_configuration_st aux_configuration[NB_AUX_CONFIGURATIONS];
static uint32_t functionEnableBits[(aux_function_max + ((sizeof(uint32_t)*8) - 1))/32];

#define SET_FUNCTION_BIT(array, function) array[function/(sizeof(array[0])*8)] |= (1 << (function % (sizeof(array[0])*8)))
#define CLEAR_FUNCTION_BIT(array, function) array[function/(sizeof(array[0])*8)] &= ~(1 << (function % (sizeof(array[0])*8)))
#define FUNCTION_BIT_IS_SET(array, function) (array[function/(sizeof(array[0])*8)] & (1 << (function % (sizeof(array[0])*8))))

static int aux_command( run_command_data_st *pcommand );
static char const * auxParameterNameLookup( unsigned int parameterID );

static const enum_mapping_st aux_function_mappings[] =
{
	{
		.name = "angle",
		.value = (uint8_t)aux_function_angle_mode
	},
	{
		.name = "rate",
		.value = (uint8_t)aux_function_rate_mode
	}
};


typedef enum aux_parameter_id_t
{
	aux_parameter_id_channel = 0,
	aux_parameter_id_function = 1,
	aux_parameter_id_channelLow = 2,
	aux_parameter_id_channelHigh = 3
} aux_parameter_id_t;


static char const * const aux_parameter_name_mappings[] =
{
	[aux_parameter_id_channel]  = "auxswitch",
	[aux_parameter_id_function] = "function",
	[aux_parameter_id_channelLow] = "low",
	[aux_parameter_id_channelHigh] = "high"
};

static const parameterConfig_st aux_config_parameterConfigs[] =
{
	{
	.parameter_id = aux_parameter_id_channel,
	.data_type = config_data_type_uint8,
	.offsetToData = offsetof(aux_configuration_st, channel),
	},
	{
	.parameter_id = aux_parameter_id_function,
	.data_type = config_data_type_enum,
	.offsetToData = offsetof(aux_configuration_st, function),
	.type_specific.enum_data.mappings = aux_function_mappings,
	.type_specific.enum_data.num_mappings = ARRAY_SIZE(aux_function_mappings)
	},
	{
	.parameter_id = aux_parameter_id_channelLow,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(aux_configuration_st, channelLow),
	},
	{
	.parameter_id = aux_parameter_id_channelHigh,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(aux_configuration_st, channelHigh),
	},
};

static const command_st aux_commands[] =
{
	{
		.group_id = configuration_id_aux,
		.name = "aux",
		.handler = aux_command,
		.configuration = aux_configuration,
		.nb_configuration_instance = ARRAY_SIZE(aux_configuration),
		.configuration_size = sizeof(aux_configuration[0]),
		.default_configuration = NULL,
		.parameterConfigs = aux_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(aux_config_parameterConfigs),
		.ParameterNameLookupCB = auxParameterNameLookup
	}
};

static char const * auxParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(aux_parameter_name_mappings) )
		return aux_parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int aux_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initAuxConfiguration( void )
{
	/* nothing to do */
}

bool isFunctionEnabled( aux_function_mappings_t function )
{
	bool isEnabled;

	if ( function < aux_function_max )
		isEnabled = FUNCTION_BIT_IS_SET( functionEnableBits, function ) != 0;
	else
		isEnabled = false;

	return isEnabled;
}

flight_mode_t getCurrentFlightMode( void )
{
	flight_mode_t flight_mode;

	/* If not in rate mode, default to angle mode. Angle mode overrides rate mode */
	if (isFunctionEnabled(aux_function_angle_mode) == true || isFunctionEnabled(aux_function_rate_mode) == false)
		flight_mode = angle_flight_mode;
	else
		flight_mode = rate_flight_mode;

	return flight_mode;
}

void updateFunctionEnables( void )
{
	/* called whenever new receiver information is received. */
	unsigned int auxConfigIndex, functionIndex;
	uint32_t tempFunctionEnables[(aux_function_max + ((sizeof(uint32_t)*8) - 1))/32] = {0};
	static flight_mode_t previousFlightMode = invalid_flight_mode;
	flight_mode_t currentFlightMode;

	for (auxConfigIndex = 0; auxConfigIndex < NB_AUX_CONFIGURATIONS; auxConfigIndex++)
	{
		aux_configuration_st *pcfg = &aux_configuration[auxConfigIndex];
		uint_fast16_t const channelValue = readReceiverChannel(pcfg->channel);

		if ( pcfg->channelLow > 0
			&& pcfg->function < aux_function_max
			&& channelValue >= pcfg->channelLow
			&& channelValue <= pcfg->channelHigh )
		{
			SET_FUNCTION_BIT(tempFunctionEnables, pcfg->function);
		}
	}

	for ( functionIndex = 0; functionIndex < aux_function_max; functionIndex++ )
	{
		if ( FUNCTION_BIT_IS_SET( tempFunctionEnables, functionIndex ) )
		{
			SET_FUNCTION_BIT( functionEnableBits, functionIndex );
		}
		else
		{
			CLEAR_FUNCTION_BIT( functionEnableBits, functionIndex );
		}
	}

	/* check for change in flight mode. Reset PID states if a change is detected */
	if ( (currentFlightMode=getCurrentFlightMode()) == angle_flight_mode )
	{
		if ( previousFlightMode != angle_flight_mode )
		{
			resetAngleModePID();
			setLEDMode(ANGLE_MODE_LED, led_state_on);
		}
	}
	else if ( previousFlightMode != rate_flight_mode )
	{
		resetRateModePID();
		setLEDMode(ANGLE_MODE_LED, led_state_off);
	}
	previousFlightMode = currentFlightMode;
}

int auxPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( aux_commands, ARRAY_SIZE(aux_commands), pv );
			break;
		case poll_id_initialise:
			initAuxConfiguration();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( aux_commands, ARRAY_SIZE(aux_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			result = saveParameterValues( pv,
								aux_commands,
								ARRAY_SIZE(aux_commands)
								);
			break;
		}
		case poll_id_show_configuration:
			result = printParametersHandler( pv,
								aux_commands,
								ARRAY_SIZE(aux_commands),
								aux_parameter_name_mappings
								);
			break;
		case poll_id_load_configuration:
			result = loadParametersHandler( pv,
								aux_commands,
								ARRAY_SIZE(aux_commands)
								);
			break;
		default:
			break;
	}

	return result;
}



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
#include <cmds.h>
#include <cli.h>
#include <config_structure.h>
#include <receiver.h>
#include <pwm.h>
#include <ppm.h>

#define MAX_RX_SIGNALS	12u
#define NB_RECEIVER_CONFIGURATIONS	1u

typedef enum receiver_mode_type
{
	receiver_mode_pwm = 0,
	receiver_mode_ppm = 1
} receiver_mode_type;

typedef struct rx_signals_st
{
	volatile uint_fast16_t rx_signals[MAX_RX_SIGNALS];

	OS_MutexID rx_signals_mutex;	/* protection for the rx_signals data */
} rx_signals_st;

static rx_signals_st	rx_signals;

typedef struct receiver_configuration_st
{
	int8_t mode;	/* note that as this is mapped to an enum data type, its size must be 8 bits */
}receiver_configuration_st;

static int receiver_command( run_command_data_st *pcommand );

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

static receiver_configuration_st receiver_configuration[NB_RECEIVER_CONFIGURATIONS];
static const receiver_configuration_st default_receiver_configuration =
{
	.mode = (int8_t)receiver_mode_pwm
};

static const config_data_point_st receiver_config_data_points[] =
{
	{
	.name = "mode",
	.data_type = config_data_type_enum,
	.offset_to_data_point = offsetof(receiver_configuration_st, mode),
	.type_specific.enum_data.enum_mappings = receiver_mode_mappings,
	.type_specific.enum_data.num_enum_mappings = ARRAY_SIZE(receiver_mode_mappings)
	}
};

static const command_st receiver_commands[] =
{
	{ .name = "rx", .handler = receiver_command	}
};


static int receiver_command( run_command_data_st *pcommand )
{
	return handleCommand( pcommand,
							receiver_configuration,
							ARRAY_SIZE(receiver_configuration),
							sizeof(receiver_configuration[0]),
							&default_receiver_configuration,
							receiver_config_data_points,
							ARRAY_SIZE(receiver_config_data_points));
}

int receiver_group_handler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_run_command:
		{
			result = runCommandHandler( receiver_commands, ARRAY_SIZE(receiver_commands), pv );
			break;
		}
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		default:
			break;
	}

	return result;
}


static void NewReceiverChannelData( uint32_t *channels, uint_fast8_t first_index, uint_fast8_t nb_channels )
{
	/* new frame of channel data from the RC receiver */
	uint_fast8_t i, max_channels;

	max_channels = min(MAX_RX_SIGNALS, first_index + nb_channels);
	STM_EVAL_LEDToggle(LED7);

	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	for (i=first_index; i < max_channels; i++)
	    rx_signals.rx_signals[i] = channels[i];

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );
}

void initReceiver( void )
{
	unsigned int index;

	/* called at startup time. Create mutex for rx_signals */
	rx_signals.rx_signals_mutex = CoCreateMutex();
	for (index = 0; index < ARRAY_SIZE(receiver_configuration); index++ )
		memcpy( &receiver_configuration[index], &default_receiver_configuration, sizeof receiver_configuration[index] );
}

uint_fast16_t readReceiverChannel(uint_fast8_t channel)
{
	uint_fast16_t channel_value;

	if ( channel < MAX_RX_SIGNALS )
	{
		CoEnterMutexSection( rx_signals.rx_signals_mutex );

		channel_value = rx_signals.rx_signals[channel];

		CoLeaveMutexSection( rx_signals.rx_signals_mutex );
	}
	else
		channel_value = INVALID_CHANNEL_VALUE;

	return channel_value;
}

void openReceiver( void )
{
	switch( receiver_configuration[0].mode )
	{
		case receiver_mode_pwm:
			initPWMRx( NewReceiverChannelData );
			break;
		case receiver_mode_ppm:
			initPPMRx( NewReceiverChannelData );
			break;
	}
}

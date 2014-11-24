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
#include <receiver.h>
#include <receiver_configuration.h>
#include <pwm.h>
#include <ppm.h>

#define MAX_RX_SIGNALS	12u

typedef struct rx_signals_st
{
	volatile uint_fast16_t rx_signals[MAX_RX_SIGNALS];

	OS_MutexID rx_signals_mutex;	/* protection for the rx_signals data */
} rx_signals_st;

typedef struct receiver_ctx_st
{
	void	(*newDataCb)( uint32_t newChannelsReceived );
} receiver_ctx_st;

static rx_signals_st	rx_signals;
static receiver_ctx_st  receiver_ctx;

static void NewReceiverChannelData( uint32_t * channels, uint_fast8_t first_index, uint_fast8_t nb_channels )
{
	/* new frame of channel data from the RC receiver */
	uint_fast8_t i, max_channels;
	uint32_t newChannelsReceived = 0;

	max_channels = min(MAX_RX_SIGNALS, first_index + nb_channels);

	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	for (i=first_index; i < max_channels; i++)
	{
	    rx_signals.rx_signals[i] = *channels++;
	    newChannelsReceived |= (1 << i);
	}

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );

	receiver_ctx.newDataCb(newChannelsReceived);
}

void initReceiver( void )
{
	/* called at startup time. Create mutex for rx_signals */
	rx_signals.rx_signals_mutex = CoCreateMutex();
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

void openReceiver( void (*newDataCb)( uint32_t channelsReceived ) )
{
	receiver_ctx.newDataCb = newDataCb;

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

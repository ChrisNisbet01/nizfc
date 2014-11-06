#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f3_discovery.h>
#include <coos.h>
#include <pins.h>

#include <receiver.h>
#include "pwm_rx_stm32f30x.h"

#define MIN_INTERFRAME_TIME_US					2900
#define MIN_CONSECUTIVE_FRAMES_OF_SAME_LENGTH	4
#define MIN_VALID_PPM_CHANNEL_PULSE				800
#define MAX_VALID_PPM_CHANNEL_PULSE				2500
#define MAX_PPM_RECEIVER_CHANNELS				12		/* maximum number of channels we're interested in */
#define MIN_PULSES_IN_VALID_PPM_FRAME			4
#define MAX_PPM_FRAME_PULSES					20		/* maximum number of pulses we'd expect to see in a valid frame. Must be >= MAX_RX_SIGNALS */


/* some state information relating to receiving PPM frames */
typedef struct ppm_ctx_st
{
	bool						accumulating_frame;
	volatile uint_fast32_t 		overflow_capture_value;
	uint_fast32_t 				last_rising_edge_capture_value;
	uint_fast16_t				number_of_channels_in_previous_frame;
	uint_fast8_t				consecutive_same_length_frames;
	uint_fast8_t				current_frame_index;
	uint32_t 					receiver_channels[MAX_PPM_RECEIVER_CHANNELS];

} ppm_ctx_st;

/*
	This table defines the pins we use as PPM inputs.
*/
// TODO: support a configurable pin/s specified by the caller to initPPMRx.
static const pin_st ppm_pins[] =
{
	{
		.pin = GPIO_Pin_8,
		.gpio = GPIOA
	}
};
#define NB_PPM_PINS	(sizeof(ppm_pins)/sizeof(ppm_pins[0]))

static ppm_ctx_st 		ppm_ctx[NB_PPM_PINS];
static void (*NewReceiverChannelDataCallback)( uint32_t *channels, uint_fast8_t first_index, uint_fast8_t nb_channels );


static void initPPMContext( ppm_ctx_st *pctx )
{
	pctx->accumulating_frame = false;
	pctx->consecutive_same_length_frames = 0;
	pctx->current_frame_index = 0;
	pctx->last_rising_edge_capture_value = 0;
	pctx->number_of_channels_in_previous_frame = 0;
	pctx->overflow_capture_value = 0;
}

/*
	newPPMPulse:

	Called when a new rising edge has been detected on the configured PPM pin.
	Called with the context of an ISR.
	pv: Our PPM context.
	pulse_width_us: The time since the last rising edge
*/
static void newPPMPulse( void *pv, uint32_t const pulse_width_us )
{
	ppm_ctx_st *pctx = pv;

	if (pulse_width_us >= MIN_INTERFRAME_TIME_US)
	{
		/* start of new frame. Check previous frame info. */
		if (pctx->accumulating_frame == true)
		{
			/* after we had a few consecutive frames of the same length we save the number of channels and compare against this. */
			if (pctx->current_frame_index == pctx->number_of_channels_in_previous_frame )
			{
				if (pctx->current_frame_index >= MIN_PULSES_IN_VALID_PPM_FRAME)
				{
					if( pctx->consecutive_same_length_frames < MIN_CONSECUTIVE_FRAMES_OF_SAME_LENGTH )
						pctx->consecutive_same_length_frames++;
					else	/* all good, deliver the current frame. */
					{
						NewReceiverChannelDataCallback( pctx->receiver_channels, 0, pctx->current_frame_index );
					}
				}
				else
				{
					/* short frame statistic? */
				}
			}
			else	/* different length from previous frame */
			{
				if (pctx->consecutive_same_length_frames == MIN_CONSECUTIVE_FRAMES_OF_SAME_LENGTH )
				{
					/*
						lost sync. deliver event?
					*/
				}
				pctx->consecutive_same_length_frames = 0;
			}
			pctx->number_of_channels_in_previous_frame = pctx->current_frame_index;
		}
		else
		{
			if (pctx->consecutive_same_length_frames == MIN_CONSECUTIVE_FRAMES_OF_SAME_LENGTH )
			{
				/*
					lost sync. deliver event?
				*/
			}
			pctx->accumulating_frame = true;
		}
		pctx->current_frame_index = 0;
	}
	else if ( pctx->current_frame_index < MAX_PPM_FRAME_PULSES && pulse_width_us >= MIN_VALID_PPM_CHANNEL_PULSE && pulse_width_us <= MAX_VALID_PPM_CHANNEL_PULSE)
	{
		/* got valid PPM pulse */
		if (pctx->accumulating_frame == true)
		{
			/* ensure we don't overflow our channel data buffer */
			if ( pctx->current_frame_index < MAX_PPM_RECEIVER_CHANNELS )
			{
				pctx->receiver_channels[pctx->current_frame_index] = pulse_width_us;
			}
			pctx->current_frame_index++;
		}
	}
	else
	{
		/* got dodgy pulse, or too many pulses for this frame to be sensible */
		pctx->accumulating_frame = false;
	}
}

/*
	initPPMRx:
	Initialise the hardware to receive a PPM input.
	newReceiverChannelDataCb: The callback to call with each new PPM frame.
*/
void initPPMRx( NewReceiverChannelDataCB newReceiverChannelDataCb )
{
	/* Set up for receiving a PPM input signal.
	*/
	ppm_ctx_st *pctx;
	uint_fast8_t ppm_idx = 0;

	NewReceiverChannelDataCallback = newReceiverChannelDataCb;
	pctx = &ppm_ctx[ppm_idx];

	initPPMContext( pctx );

	openPwmTimer( &ppm_pins[0], ppm_mode, newPPMPulse, pctx );

}



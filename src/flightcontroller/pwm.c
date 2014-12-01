#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <coos.h>
#include <pwm_inputs.h>

#include <receiver.h>
#include "pwm_rx_stm32f30x.h"

static const pwm_input_id_t pwm_pinIDs[] =
{
	pwm_input_1,
	pwm_input_2,
	pwm_input_3,
	pwm_input_4,
	pwm_input_5,
	pwm_input_6
};
#define NB_PWM_PINS	(sizeof(pwm_pinIDs)/sizeof(pwm_pinIDs[0]))

typedef struct pwm_channel_ctx_st
{
	uint8_t dummy;
} pwm_channel_ctx_st;

static pwm_channel_ctx_st 		pwm_channel_ctxs[NB_PWM_PINS];
#define PWM_CONTEXT_INDEX(p)	((p)-pwm_channel_ctxs)

static void (*NewReceiverChannelDataCallback)( uint32_t *channels, uint_fast8_t first_index, uint_fast8_t nb_channels );

static void newPWMPulse( void *pv, uint32_t const pulse_width_us )
{
	pwm_channel_ctx_st *pctx = pv;
	uint32_t pulse = pulse_width_us;

	// TODO: validate pulse width
	// TODO: timestamp
	NewReceiverChannelDataCallback( &pulse, PWM_CONTEXT_INDEX(pctx), 1 );
}

void initPWMRx( NewReceiverChannelDataCB newReceiverChannelDataCb )
{
	/*
		Set up for receiving a PWM input signal.
	*/
	uint_fast8_t pwm_idx;

	// TODO: number of pins to init from config
	NewReceiverChannelDataCallback = newReceiverChannelDataCb;
	for( pwm_idx = 0; pwm_idx < NB_PWM_PINS; pwm_idx++ )
	{
		openPwmTimer( pwm_pinIDs[pwm_idx], pwm_mode, newPWMPulse, &pwm_channel_ctxs[pwm_idx] );
	}

}



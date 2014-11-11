#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f3_discovery.h>
#include <coos.h>
#include <pins.h>

#include <receiver.h>
#include "pwm_rx_stm32f30x.h"

static const pin_st pwm_pins[] =
{
	{
		.pin = GPIO_Pin_8,
		.gpio = GPIOA,
	},
	{
		.pin = GPIO_Pin_9,
		.gpio = GPIOA,
	},
	{
		.pin = GPIO_Pin_6,
		.gpio = GPIOC,
	},
	{
		.pin = GPIO_Pin_7,
		.gpio = GPIOC,
	},
	{
		.pin = GPIO_Pin_8,
		.gpio = GPIOC,
	},
	{
		.pin = GPIO_Pin_9,
		.gpio = GPIOC
	}
};
#define NB_PWM_PINS	(sizeof(pwm_pins)/sizeof(pwm_pins[0]))

typedef struct pwm_channel_ctx_st
{
	int dummy;
} pwm_channel_ctx_st;

static pwm_channel_ctx_st 		pwm_channel_ctxs[NB_PWM_PINS];
#define PWM_CONTEXT_INDEX(p)	((p)-&pwm_channel_ctxs[0])

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
		openPwmTimer( &pwm_pins[pwm_idx], pwm_mode, newPWMPulse, &pwm_channel_ctxs[pwm_idx] );
	}

}



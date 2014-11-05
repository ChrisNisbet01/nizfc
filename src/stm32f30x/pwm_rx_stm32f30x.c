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

#include "pwm_rx_stm32f30x.h"

#define max(a,b) \
({ typeof (a) _a = (a); \
   typeof (b) _b = (b); \
 _a > _b ? _a : _b; })

#define min(a,b) \
({ typeof (a) _a = (a); \
   typeof (b) _b = (b); \
 _a < _b ? _a : _b; })

#define PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE

typedef uint32_t capture_t;

typedef enum timer_idx_t
{
	TIM1_IDX,
	TIM3_IDX,
	TIM8_IDX,
	NB_TIMERS
} timer_idx_t;

typedef enum timer_channel_t
{
	CH1_IDX,
	CH2_IDX,
	CH3_IDX,
	CH4_IDX,
	NB_CHANNELS
} timer_channel_t;

typedef enum pin_state_t
{
	pin_low,
	pin_high
} pin_state_t;

#define PWM_PERIOD			0xffff
#define PWM_FREQUENCY_HZ	1000000
#define INVALID_RX_SIGNAL	0

#define MIN_INTERFRAME_TIME_US					2900
#define MIN_CONSECUTIVE_FRAMES_OF_SAME_LENGTH	4
#define MIN_VALID_PPM_CHANNEL_PULSE				800
#define MAX_VALID_PPM_CHANNEL_PULSE				2500
#define MAX_PPM_RECEIVER_CHANNELS				12		/* maximum number of channels we're interested in */
#define MIN_PULSES_IN_VALID_PPM_FRAME			4
#define MAX_PPM_FRAME_PULSES					20		/* maximum number of pulses we'd expect to see in a valid frame. Must be >= MAX_RX_SIGNALS */

typedef struct pwm_rx_config_st
{
	/* GPIO settings */
	uint_fast16_t	pin;
	uint_fast8_t	pinSource;
	uint_fast8_t	pinAF;
	uint_fast8_t	pinOutputType;
	uint_fast8_t	pinPuPd;

	GPIO_TypeDef	*gpio;
	uint_fast32_t	RCC_AHBPeriph;

	/* Associated timer settings */
	void 			(*RCC_APBPeriphClockCmd)(uint32_t RCC_APBPeriph, FunctionalState NewState);
	uint_fast32_t	RCC_APBPeriph;
    TIM_TypeDef 	*tim;
    uint_fast8_t 	channel;
    uint_fast8_t	irq;
    timer_idx_t		timer_index;
    timer_channel_t channel_index;

} pwm_rx_config_st;

typedef struct pwm_pin_st
{
	uint_fast16_t	pin;
	GPIO_TypeDef	*gpio;
} pwm_pin_st;

/*
	A table of all pins that can be used as PWM/PPM inputs.
	Note that some pins may be mapped to more than one timer channel.
*/
static pwm_pin_st pwm_pins[] =
{
	{
		.pin = GPIO_Pin_8,
		.gpio = GPIOA
	},
	{
		.pin = GPIO_Pin_9,
		.gpio = GPIOA
	},
	{
		.pin = GPIO_Pin_6,
		.gpio = GPIOC
	},
	{
		.pin = GPIO_Pin_7,
		.gpio = GPIOC
	},
	{
		.pin = GPIO_Pin_8,
		.gpio = GPIOC
	},
	{
		.pin = GPIO_Pin_9,
		.gpio = GPIOC
	}
};

static const pwm_rx_config_st pwm_rx_configs[] =
{
	{
		.pin = GPIO_Pin_8,
		.pinSource = GPIO_PinSource8,
		.pinAF = GPIO_AF_6,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOA,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOA,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB2Periph_TIM1,
		.tim = TIM1,
		.channel = TIM_Channel_1,
		.irq = TIM1_CC_IRQn,
		.timer_index = TIM1_IDX,
		.channel_index = CH1_IDX
	},
	{
		.pin = GPIO_Pin_9,
		.pinSource = GPIO_PinSource9,
		.pinAF = GPIO_AF_6,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOA,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOA,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB2Periph_TIM1,
		.tim = TIM1,
		.channel = TIM_Channel_2,
		.irq = TIM1_CC_IRQn,
		.timer_index = TIM1_IDX,
		.channel_index = CH2_IDX
	},
	{
		.pin = GPIO_Pin_6,
		.pinSource = GPIO_PinSource6,
		.pinAF = GPIO_AF_4,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOC,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOC,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB2Periph_TIM8,
		.tim = TIM8,
		.channel = TIM_Channel_1,
		.irq = TIM8_CC_IRQn,
		.timer_index = TIM8_IDX,
		.channel_index = CH1_IDX
	},
	{
		.pin = GPIO_Pin_8,
		.pinSource = GPIO_PinSource8,
		.pinAF = GPIO_AF_4,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOC,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOC,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB2Periph_TIM8,
		.tim = TIM8,
		.channel = TIM_Channel_3,
		.irq = TIM8_CC_IRQn,
		.timer_index = TIM8_IDX,
		.channel_index = CH3_IDX
	},
	{
		.pin = GPIO_Pin_6,
		.pinSource = GPIO_PinSource6,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOC,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOC,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM3,
		.tim = TIM3,
		.channel = TIM_Channel_1,
		.irq = TIM3_IRQn,
		.timer_index = TIM3_IDX,
		.channel_index = CH1_IDX
	},
	{
		.pin = GPIO_Pin_7,
		.pinSource = GPIO_PinSource7,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOC,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOC,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM3,
		.tim = TIM3,
		.channel = TIM_Channel_2,
		.irq = TIM3_IRQn,
		.timer_index = TIM3_IDX,
		.channel_index = CH2_IDX
	},
	{
		.pin = GPIO_Pin_8,
		.pinSource = GPIO_PinSource8,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOC,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOC,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM3,
		.tim = TIM3,
		.channel = TIM_Channel_3,
		.irq = TIM3_IRQn,
		.timer_index = TIM3_IDX,
		.channel_index = CH3_IDX
	},
	{
		.pin = GPIO_Pin_9,
		.pinSource = GPIO_PinSource9,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOC,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOC,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM3,
		.tim = TIM3,
		.channel = TIM_Channel_4,
		.irq = TIM3_IRQn,
		.timer_index = TIM3_IDX,
		.channel_index = CH4_IDX
	}
};
#define NB_PWM_PORTS	(sizeof(pwm_rx_configs)/sizeof(pwm_rx_configs[0]))

typedef struct rx_signals_st
{
	volatile uint_fast16_t rx_signals[MAX_RX_SIGNALS];

	OS_MutexID rx_signals_mutex;	/* protection for the rx_signals data */
} rx_signals_st;

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

typedef struct pwm_ctx_st
{
	uint_fast8_t				used;			/* non-zero if pin is in use */
	volatile pin_state_t 		last_pin_state;	/* state of pin at last edge ISR */
	volatile capture_t			last_rising_edge_capture_value;

	pwm_rx_config_st const 		* pconfig;			/* the GPIO/TIM details */

} pwm_ctx_st;

/* one of these for each timer channel */
typedef struct timer_configs_st
{
	uint_fast8_t port;
	void (*edgeCallback)( uint_fast8_t port, capture_t count );
	void (*overflowCallback)( uint_fast8_t port, capture_t count );
} timer_configs_st;

static timer_configs_st timer_configs[NB_TIMERS][NB_CHANNELS];

// The index of the timer configs we expect to see PPM signals on
// TODO: allow user to specify
#define PPM_RX_IDX	0	/* the index into the timer config table */
#define NB_PPM_CTX	1	/* the number of PPM contexts */

static rx_signals_st	rx_signals;
static pwm_ctx_st 		pwm_ctxs[NB_PWM_PORTS];
static ppm_ctx_st 		ppm_ctx[NB_PPM_CTX];

static void initPwmGpio( pwm_rx_config_st const * pconfig )
{
	/* enable the clock for this GPIO port */
    RCC_AHBPeriphClockCmd(pconfig->RCC_AHBPeriph, ENABLE);

	/* configure the pin */
	GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = pconfig->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = pconfig->pinOutputType;
    GPIO_InitStructure.GPIO_PuPd = pconfig->pinPuPd;

    GPIO_Init(pconfig->gpio, &GPIO_InitStructure);

	/* configure the pin function */
    GPIO_PinAFConfig(pconfig->gpio, pconfig->pinSource, pconfig->pinAF);


}

static void initInputCapture( pwm_rx_config_st const * pconfig, uint_fast16_t polarity )
{
	TIM_ICInitTypeDef TIM_ICInitStructure;

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = pconfig->channel;
	TIM_ICInitStructure.TIM_ICPolarity = polarity;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(pconfig->tim, &TIM_ICInitStructure);
}

static void initTimerTimeBase(TIM_TypeDef *tim, uint_fast16_t period, uint_fast32_t frequency_hz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
    TIM_TimeBaseStructure.TIM_Prescaler = (clocks.SYSCLK_Frequency/frequency_hz) - 1;

    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void initTimerNVIC( uint_fast8_t irq )
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

	/* temp debug */
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

static void initPwmTimer( pwm_rx_config_st const * pconfig, uint_fast16_t period, uint_fast32_t frequency_hz )
{
	/* configure timer period and frequency */
    initTimerTimeBase(pconfig->tim, period, frequency_hz);	/* gives 1us resolution over 1-2ms width == 0.1% */

    /* configure interrupt priorities */
    initTimerNVIC(pconfig->irq);

}

static void configureTimerCallbacks( pwm_rx_config_st const * pconfig,
										uint_fast8_t port,
										void (*edgeCallback)( uint_fast8_t port, capture_t count ),
										void (*overflowCallback)( uint_fast8_t port, capture_t count ) )
{
	timer_configs[pconfig->timer_index][pconfig->channel_index].port = port;
	timer_configs[pconfig->timer_index][pconfig->channel_index].edgeCallback = edgeCallback;
	timer_configs[pconfig->timer_index][pconfig->channel_index].overflowCallback = overflowCallback;
}

static void configureTimerInputCaptureCompareChannel( TIM_TypeDef *tim, const uint_fast8_t channel )
{
    switch (channel) {
        case TIM_Channel_1:
            TIM_ITConfig(tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(tim, TIM_IT_CC4, ENABLE);
            break;
    }
}

static void deliverNewReceiverChannelData( uint32_t *channels, uint_fast8_t first_index, uint_fast8_t nb_channels )
{
	/* new frame of channel data from the RC receiver */
	uint_fast8_t i, max_channels;

	max_channels = min(MAX_RX_SIGNALS, nb_channels);
	STM_EVAL_LEDToggle(LED7);

	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	for (i=first_index; i < max_channels; i++)
	    rx_signals.rx_signals[i] = channels[i];

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );
}


static void pwmEdgeCallback(uint_fast8_t port, capture_t capture)
{
	uint32_t pulse_width_us;

    pwm_ctx_st *pctx = &pwm_ctxs[port];
    pwm_rx_config_st const * pconfig = pctx->pconfig;

#if defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
   	if (GPIO_ReadInputDataBit(pconfig->gpio, pconfig->pin) == Bit_SET)	/* then pin is high */
#else
    if (pctx->last_pin_state == pin_low)
#endif
    {
#if !defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
        initInputCapture(pconfig, TIM_ICPolarity_Falling);
        pctx->last_pin_state = pin_high;
#endif

    	/* got rising edge. now wait for falling edge */
        pctx->last_rising_edge_capture_value = capture;
    } else {
		/* now wait for rising edge */
#if !defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
        initInputCapture(pconfig, TIM_ICPolarity_Rising);
        pctx->last_pin_state = pin_low;
#endif

		pulse_width_us = (capture - pctx->last_rising_edge_capture_value) & PWM_PERIOD;

		deliverNewReceiverChannelData( &pulse_width_us, port, 1 );
    }
}

static void new_ppm_rising_edge( ppm_ctx_st *pctx, uint32_t const pulse_width_us )
{
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
						deliverNewReceiverChannelData( pctx->receiver_channels, 0, pctx->current_frame_index );
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

static void ppmOverflowCallback(uint_fast8_t port, capture_t capture)
{
    ppm_ctx_st *pctx = &ppm_ctx[port];

	/* capture should equal the period of the timer */
	pctx->overflow_capture_value += capture;
}

static void ppmEdgeCallback(uint_fast8_t port, capture_t capture)
{
    ppm_ctx_st *pctx = &ppm_ctx[port];
	uint_fast32_t pulse_width_us, current;

	current = pctx->overflow_capture_value + capture;
	pulse_width_us = current - pctx->last_rising_edge_capture_value;
	pctx->last_rising_edge_capture_value = current;

	new_ppm_rising_edge( pctx, pulse_width_us );
}

static void enablePWMTiming( pwm_rx_config_st const * pconfig, uint_fast8_t pwm_context_index, uint_fast16_t polarity, void (*edgeCallback)(uint_fast8_t port, capture_t capture), void (*overflowCallback)(uint_fast8_t port, capture_t capture) )
{

	/*
		NB very important to perform these operations as per instructions in
		stm32f30x_tim.c TIM_ICInit()
		Order of operations is important.
	*/
	/* Start the APB peripheral clock first */
    pconfig->RCC_APBPeriphClockCmd(pconfig->RCC_APBPeriph, ENABLE);

	/* configure the pin */
	initPwmGpio( pconfig );

	/* configure the input compare channel */
	initInputCapture( pconfig, polarity);

	configureTimerCallbacks( pconfig, pwm_context_index, edgeCallback, overflowCallback );

	/* configure the timer
		period == 65535ms
		frequency == 1MHz
	*/
	initPwmTimer( pconfig, PWM_PERIOD, PWM_FREQUENCY_HZ );

	/* configure the channel interrupt */
	configureTimerInputCaptureCompareChannel( pconfig->tim, pconfig->channel );

	/* configure the overflow interrupt */
	if ( overflowCallback != NULL )
		TIM_ITConfig( pconfig->tim, TIM_IT_Update, ENABLE );

    /* start the timer */
    TIM_Cmd(pconfig->tim, ENABLE);
}

static void initPPMContext( ppm_ctx_st *pctx )
{
	pctx->accumulating_frame = false;
	pctx->consecutive_same_length_frames = 0;
	pctx->current_frame_index = 0;
	pctx->last_rising_edge_capture_value = 0;
	pctx->number_of_channels_in_previous_frame = 0;
	pctx->overflow_capture_value = 0;
}

void openPPMInput( void )
{
	/* Set up for receiving a PPM input signal.
	*/

	ppm_ctx_st *pctx;
	uint_fast8_t ppm_idx = 0;

	pctx = &ppm_ctx[ppm_idx];

	initPPMContext( pctx );

	enablePWMTiming( &pwm_rx_configs[0], ppm_idx, TIM_ICPolarity_Rising, ppmEdgeCallback, ppmOverflowCallback );

}

void openPWMInput( uint_fast8_t nb_rx_channels )
{
	/*
		Set up for receiving PWM input signals.
	*/
	uint_fast8_t i;

	for (i=0; i < nb_rx_channels && i < NB_PWM_PORTS; i++)
	{
		pwm_ctx_st *pctx;

		pctx = &pwm_ctxs[i];
		if (pctx->used == 0)
		{
			pwm_rx_config_st const * pconfig;
			/* mark as used */
	    	pctx->used = 1;
			pconfig = &pwm_rx_configs[i];

			/* remember the pin configuration */
			pctx->pconfig = pconfig;
			pctx->last_pin_state = pin_low;

#if defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
			enablePWMTiming( pconfig, i, TIM_ICPolarity_BothEdge, pwmEdgeCallback, NULL );
#else
			enablePWMTiming( pconfig, i, TIM_ICPolarity_Rising, pwmEdgeCallback, NULL );
#endif
		}
		else
		{
			// TODO: better handling if a channel isn't free
		}
	}

}


void initPWMRx( void )
{
	/* called at startup time. Create mutex for rx_signals */
	rx_signals.rx_signals_mutex = CoCreateMutex();
}

int readRXSignals(uint_fast16_t signals[MAX_RX_SIGNALS])
{
	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	memcpy( signals, (void *)&rx_signals.rx_signals[0], sizeof rx_signals.rx_signals );

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );

	// TODO: give indication that signals are valid
	return true;
}

static void handleTIMIRQ( TIM_TypeDef *tim, timer_idx_t timerIndex )
{
    capture_t capture;
	uint_fast8_t channelIndex;

	CoEnterISR();

    if (TIM_GetITStatus(tim, TIM_IT_Update) == SET) {
        capture = tim->ARR;
        TIM_ClearITPendingBit(tim, TIM_IT_Update);

        for (channelIndex = 0; channelIndex < NB_CHANNELS; channelIndex++)
        {
			timer_configs_st *timer_config = &timer_configs[timerIndex][channelIndex];

            if (timer_config->overflowCallback != NULL)
           	{
            	timer_config->overflowCallback(timer_config->port, capture);
            }
        }
    }

    for (channelIndex = 0; channelIndex < NB_CHANNELS; channelIndex++)
    {
		timer_configs_st *timer_config = &timer_configs[timerIndex][channelIndex];
		int got_capture;

        if (channelIndex == CH1_IDX && TIM_GetITStatus(tim, TIM_IT_CC1) == SET)
        {
            capture = TIM_GetCapture1(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC1);
            got_capture = 1;
        }
        else if (channelIndex == CH2_IDX && TIM_GetITStatus(tim, TIM_IT_CC2) == SET)
        {
            capture = TIM_GetCapture2(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC2);
            got_capture = 1;
        }
        else if (channelIndex == CH3_IDX && TIM_GetITStatus(tim, TIM_IT_CC3) == SET)
        {
            capture = TIM_GetCapture3(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC3);
            got_capture = 1;
        }
        else if (channelIndex == CH4_IDX && TIM_GetITStatus(tim, TIM_IT_CC4) == SET)
        {
            capture = TIM_GetCapture4(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC4);
            got_capture = 1;
        }
        else
       	{
            got_capture = 0;
       		capture = 0;	/* shouldn't happen */
        }

        if (got_capture && timer_config->edgeCallback != NULL)
        {
	        timer_config->edgeCallback(timer_config->port, capture);
        }
    }

	CoExitISR();
}

/* TIM1 doesn't have a separate CC IRQ handler */
void TIM1_CC_IRQHandler(void)
{
    handleTIMIRQ(TIM1, TIM1_IDX);
}

/* TIM1 shares this interrupt with TIM16 */
void TIM1_UP_TIM16_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	    handleTIMIRQ(TIM1, TIM1_IDX);
}

void TIM3_IRQHandler(void)
{
    handleTIMIRQ(TIM3, TIM3_IDX);
}

void TIM8_CC_IRQHandler(void)
{
    handleTIMIRQ(TIM8, TIM8_IDX);
}

void TIM8_UP_IRQHandler(void)
{
    handleTIMIRQ(TIM8, TIM8_IDX);
}


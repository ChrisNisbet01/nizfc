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

#define INVALID_IRQn		0xff

typedef struct pwm_timer_config_st
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
    uint_fast8_t	secondary_irq;	/* some timers use two interrupts */
    timer_idx_t		timer_index;
    timer_channel_t channel_index;

} pwm_timer_config_st;

typedef struct channel_config_st {
    uint16_t channel;
    uint16_t interruptBit;
    uint32_t (*TIM_GetCaptureFn)(TIM_TypeDef* TIMx);
} channel_config_st;

/* Ensure there is one of these configured for each timer channel used. */
static const channel_config_st channel_configs[] = {
    [CH1_IDX] = { TIM_Channel_1, TIM_IT_CC1, TIM_GetCapture1 },
    [CH2_IDX] = { TIM_Channel_2, TIM_IT_CC2, TIM_GetCapture2 },
    [CH3_IDX] = { TIM_Channel_3, TIM_IT_CC3, TIM_GetCapture3 },
    [CH4_IDX] = { TIM_Channel_4, TIM_IT_CC4, TIM_GetCapture4 }
};

static const pwm_timer_config_st pwm_timer_configs[] =
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
		.secondary_irq = TIM1_UP_TIM16_IRQn,
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
		.secondary_irq = INVALID_IRQn,
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
		.secondary_irq = TIM8_UP_IRQn,
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
		.secondary_irq = TIM8_UP_IRQn,
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
		.secondary_irq = INVALID_IRQn,
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
		.secondary_irq = INVALID_IRQn,
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
		.secondary_irq = INVALID_IRQn,
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
		.secondary_irq = INVALID_IRQn,
		.timer_index = TIM3_IDX,
		.channel_index = CH4_IDX
	}
};
#define NB_PWM_PORTS	(sizeof(pwm_timer_configs)/sizeof(pwm_timer_configs[0]))
#define PWM_CONFIG_INDEX(p)	((p)-&pwm_timer_configs[0])

typedef struct pwm_timer_st
{
	uint_fast8_t				used;			/* non-zero if pin is in use */
#if !defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
	volatile pin_state_t 		last_pin_state;	/* state of pin at last edge ISR */
#endif
	volatile capture_t			last_rising_edge_capture_value;
	volatile uint_fast32_t		overflow_capture_value;

	pwm_timer_config_st         const * timer_config;			/* the GPIO/TIM details */

	pwm_mode_t 					mode;
	void 						(*newPulseCb)( void *pv, uint32_t const pulse_width_us );
	void 						*userPv;
} pwm_timer_st;

/* one of these for each timer channel */
typedef struct timer_configs_st
{
	uint_fast8_t port;
	void (*edgeCallback)( uint_fast8_t port, capture_t count );
	void (*overflowCallback)( uint_fast8_t port, capture_t count );
} timer_configs_st;

static timer_configs_st timer_configs[NB_TIMERS][NB_CHANNELS];
static pwm_timer_st 	pwm_timers[NB_PWM_PORTS];

static void initPwmGpio( pwm_timer_config_st const * timer_config )
{
	/* enable the clock for this GPIO port */
    RCC_AHBPeriphClockCmd(timer_config->RCC_AHBPeriph, ENABLE);

	/* configure the pin */
	GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = timer_config->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = timer_config->pinOutputType;
    GPIO_InitStructure.GPIO_PuPd = timer_config->pinPuPd;

    GPIO_Init(timer_config->gpio, &GPIO_InitStructure);

	/* configure the pin function */
    GPIO_PinAFConfig(timer_config->gpio, timer_config->pinSource, timer_config->pinAF);


}

static void initInputCapture( TIM_TypeDef *tim, uint_fast8_t channel, uint_fast16_t polarity )
{
	TIM_ICInitTypeDef TIM_ICInitStructure;

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = channel;
	TIM_ICInitStructure.TIM_ICPolarity = polarity;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
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
}

static void initPwmTimer( pwm_timer_config_st const * timer_config, uint_fast16_t period, uint_fast32_t frequency_hz )
{
	/* configure timer period and frequency */
    initTimerTimeBase(timer_config->tim, period, frequency_hz);	/* gives 1us resolution over 1-2ms width == 0.1% */

    /* configure interrupt priorities */
    initTimerNVIC(timer_config->irq);
    if ( timer_config->secondary_irq != INVALID_IRQn )
	    initTimerNVIC(timer_config->secondary_irq);

}

static void configureTimerCallbacks( uint_fast8_t const timer_index,
										uint_fast8_t const channel_index,
										uint_fast8_t const port,
										void (*edgeCallback)( uint_fast8_t port, capture_t count ),
										void (*overflowCallback)( uint_fast8_t port, capture_t count ) )
{
	timer_configs[timer_index][channel_index].port = port;
	timer_configs[timer_index][channel_index].edgeCallback = edgeCallback;
	timer_configs[timer_index][channel_index].overflowCallback = overflowCallback;
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

static void pwmEdgeCallback(uint_fast8_t port, capture_t capture)
{
	uint32_t pulse_width_us;

    pwm_timer_st *pctx = &pwm_timers[port];
    pwm_timer_config_st const * timer_config = pctx->timer_config;

#if defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
   	if (GPIO_ReadInputDataBit(timer_config->gpio, timer_config->pin) == Bit_SET)	/* then pin is high */
#else
    if (pctx->last_pin_state == pin_low)
#endif
    {
#if !defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
        initInputCapture(timer_config->tim, timer_config->channel, TIM_ICPolarity_Falling);
        pctx->last_pin_state = pin_high;
#endif

    	/* got rising edge. now wait for falling edge */
        pctx->last_rising_edge_capture_value = capture;
    } else {
		/* now wait for rising edge */
#if !defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
        initInputCapture(timer_config->tim, timer_config->channel, TIM_ICPolarity_Rising);
        pctx->last_pin_state = pin_low;
#endif

		pulse_width_us = (capture - pctx->last_rising_edge_capture_value) & PWM_PERIOD;

		pctx->newPulseCb( pctx->userPv, pulse_width_us );
    }
}

static void ppmOverflowCallback(uint_fast8_t port, capture_t capture)
{
    pwm_timer_st *pctx = &pwm_timers[port];

	/* capture should equal the period of the timer */
	pctx->overflow_capture_value += capture;
}

static void ppmEdgeCallback(uint_fast8_t port, capture_t capture)
{
    pwm_timer_st *pctx = &pwm_timers[port];
	uint_fast32_t pulse_width_us, current;

	current = pctx->overflow_capture_value + capture;
	pulse_width_us = current - pctx->last_rising_edge_capture_value;
	pctx->last_rising_edge_capture_value = current;

	pctx->newPulseCb( pctx->userPv, pulse_width_us );
}

static void enablePWMTiming( pwm_timer_config_st const * timer_config, uint_fast8_t pwm_context_index, uint_fast16_t polarity, void (*edgeCallback)(uint_fast8_t port, capture_t capture), void (*overflowCallback)(uint_fast8_t port, capture_t capture) )
{

	/*
		NB very important to perform these operations as per instructions in
		stm32f30x_tim.c TIM_ICInit()
		Order of operations is important.
	*/
	/* Start the APB peripheral clock first */
    timer_config->RCC_APBPeriphClockCmd(timer_config->RCC_APBPeriph, ENABLE);

	/* configure the pin */
	initPwmGpio( timer_config );

	/* configure the input compare channel */
	initInputCapture(timer_config->tim, timer_config->channel, polarity);

	configureTimerCallbacks( timer_config->timer_index, timer_config->channel_index, pwm_context_index, edgeCallback, overflowCallback );

	/* configure the timer
		period == 65535ms
		frequency == 1MHz
	*/
	initPwmTimer( timer_config, PWM_PERIOD, PWM_FREQUENCY_HZ );

	/* configure the channel interrupt */
	configureTimerInputCaptureCompareChannel( timer_config->tim, timer_config->channel );

	/* configure the overflow interrupt */
	if ( overflowCallback != NULL )
		TIM_ITConfig( timer_config->tim, TIM_IT_Update, ENABLE );

    /* start the timer */
    TIM_Cmd(timer_config->tim, ENABLE);
}

static pwm_timer_config_st const *pwmTimerConfigLookup( pin_st const * pin )
{
	uint_fast8_t index;

	for (index = 0; index < NB_PWM_PORTS; index++)
	{
		if (pin->pin == pwm_timer_configs[index].pin && pin->gpio == pwm_timer_configs[index].gpio)
			return &pwm_timer_configs[index];
	}

	return NULL;
}
void openPwmTimer( pin_st const * const pin, pwm_mode_t mode, void (*newPulseCb)( void *pv, uint32_t const pulse_width_us ), void *pv )
{
	pwm_timer_config_st const * timer_config;

	timer_config = pwmTimerConfigLookup(pin);

	if (timer_config == NULL)
		goto done;

	pwm_timer_st *pctx;
	pctx = &pwm_timers[PWM_CONFIG_INDEX(timer_config)];
	pctx->timer_config = timer_config;
	pctx->mode = mode;
	pctx->newPulseCb = newPulseCb;
	pctx->userPv = pv;

	switch (mode)
	{
		case pwm_mode:
#if defined(PWM_CHECKS_GPIO_PIN_FOR_PIN_STATE)
			pctx->timer_config = timer_config;
			enablePWMTiming( timer_config, PWM_CONFIG_INDEX(timer_config), TIM_ICPolarity_BothEdge, pwmEdgeCallback, NULL );
#else
			pctx->last_pin_state = pin_low;
			enablePWMTiming( timer_config, PWM_CONFIG_INDEX(timer_config), TIM_ICPolarity_Rising, pwmEdgeCallback, NULL );
#endif
			break;
		case ppm_mode:
			enablePWMTiming( timer_config, PWM_CONFIG_INDEX(timer_config), TIM_ICPolarity_Rising, ppmEdgeCallback, ppmOverflowCallback );
			break;
	}
done:
	return;
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
        channel_config_st const * const channel = &channel_configs[channelIndex];

        if (TIM_GetITStatus(tim, channel->interruptBit) == SET)
        {
            TIM_ClearITPendingBit(tim, channel->interruptBit);

            capture = channel->TIM_GetCaptureFn(tim);

			timer_configs_st *timer_config = &timer_configs[timerIndex][channelIndex];

	        if (timer_config->edgeCallback != NULL)
	        {
		        timer_config->edgeCallback(timer_config->port, capture);
	        }
        }
    }

	CoExitISR();
}

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


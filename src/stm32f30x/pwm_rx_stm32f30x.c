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

typedef struct pwm_rx_config_st
{
	/* GPIO settings */
	uint_fast8_t 	pin;
	uint_fast8_t	pinSource;
	uint_fast8_t	pinAF;
	uint_fast8_t	pinMode;
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

static const pwm_rx_config_st pwm_rx_configs[] =
{
	{
		.pin = GPIO_Pin_8,
		.pinSource = GPIO_PinSource8,
		.pinAF = GPIO_AF_6,
		.pinMode = GPIO_OType_PP,
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
		.pinMode = GPIO_OType_PP,
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
		.pinMode = GPIO_OType_PP,
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
		.pinMode = GPIO_OType_PP,
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
		.pinMode = GPIO_OType_PP,
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
		.pinMode = GPIO_OType_PP,
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
		.pinMode = GPIO_OType_PP,
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
		.pinMode = GPIO_OType_PP,
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
	volatile uint_fast8_t last_pin_state;	/* state of pin at last edge ISR */
	volatile uint_fast32_t	last_capture_value;

	volatile uint_fast8_t rx_signal_count;
	volatile uint_fast16_t rx_signals[2][MAX_RX_SIGNALS];

	volatile uint_fast8_t current_rx_signal_count;
	volatile uint_fast8_t latest_valid_signals_index;

	OS_MutexID rx_signals_mutex;
} rx_signals_st;

typedef struct pwm_ctx_st
{
	uint_fast8_t	used;	/* non-zero if pin is in use */
	volatile pin_state_t last_pin_state;	/* state of pin at last edge ISR */
	volatile uint_fast32_t overflow_capture_value;
	volatile uint_fast32_t high_edge_capture_value;

	pwm_rx_config_st const * pconfig;
	uint_fast8_t channel_index;

	volatile uint_fast16_t rx_signal;

} pwm_ctx_st;

/* one of these for each timer channel */
typedef struct timer_configs_st
{
	uint_fast8_t port;
	void (*edgeCallback)( uint_fast8_t port, uint_fast16_t count );
	void (*overflowCallback)( uint_fast8_t port, uint_fast16_t count );
} timer_configs_st;

static timer_configs_st timer_configs[NB_TIMERS][NB_CHANNELS];

#define PPM_RX_IDX	0
#define PWM0_RX_IDX	0
#define PWM1_RX_IDX	1
#define PWM2_RX_IDX	2
#define PWM3_RX_IDX	3

static rx_signals_st	rx_signals;
static pwm_ctx_st pwm_ctxs[NB_PWM_PORTS];

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
    GPIO_InitStructure.GPIO_OType = pconfig->pinMode;
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

static void configureTimerCallbacks( pwm_rx_config_st const * pconfig, uint_fast8_t port, void (*edgeCallback)( uint_fast8_t port, uint_fast16_t count ),  void (*overflowCallback)( uint_fast8_t port, uint_fast16_t count ) )
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
            //TIM_CCxCmd( tim, channel, TIM_CCx_Enable);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(tim, TIM_IT_CC2, ENABLE);
            //TIM_CCxCmd( tim, channel, TIM_CCx_Enable );
            break;
        case TIM_Channel_3:
            TIM_ITConfig(tim, TIM_IT_CC3, ENABLE);
            //TIM_CCxCmd( tim, channel, TIM_CCx_Enable );
            break;
        case TIM_Channel_4:
            TIM_ITConfig(tim, TIM_IT_CC4, ENABLE);
            //TIM_CCxCmd( tim, channel, TIM_CCx_Enable );
            break;
    }
}

static void pwmOverflowCallback(uint_fast8_t port, uint_fast16_t capture)
{
    pwm_ctx_st *pctx = &pwm_ctxs[port];

	pctx->overflow_capture_value += capture;
}

static void pwmEdgeCallback(uint_fast8_t port, uint_fast16_t capture)
{
    pwm_ctx_st *pctx = &pwm_ctxs[port];
    pwm_rx_config_st const * pconfig = pctx->pconfig;

    if (pctx->last_pin_state == pin_low)
    {
        initInputCapture(pconfig, TIM_ICPolarity_Falling);
        pctx->last_pin_state = pin_high;

    	/* got rising edge. now wait for falling edge */
        pctx->high_edge_capture_value = capture + pctx->overflow_capture_value;
    } else {
		/* now wait for rising edge */
        initInputCapture(pconfig, TIM_ICPolarity_Rising);
        pctx->last_pin_state = pin_low;

		CoEnterMutexSection( rx_signals.rx_signals_mutex );

        rx_signals.rx_signals[rx_signals.latest_valid_signals_index][port] = pctx->overflow_capture_value + capture - pctx->high_edge_capture_value;

		CoLeaveMutexSection( rx_signals.rx_signals_mutex );

    }
}


static void ppmOverflowCallback(uint_fast8_t port, uint_fast16_t capture)
{
    pwm_ctx_st *pctx = &pwm_ctxs[port];

	pctx->overflow_capture_value += capture;
}

static void ppmEdgeCallback(uint_fast8_t port, uint_fast16_t capture)
{
    pwm_ctx_st *pctx = &pwm_ctxs[port];
	uint_fast32_t delta, current;

	current = pctx->overflow_capture_value + capture;
	delta = current - rx_signals.last_capture_value;

	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	rx_signals.rx_signals[rx_signals.latest_valid_signals_index][0] = delta;

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );

	rx_signals.last_capture_value = current;

}

void *openPPMInput( void )
{
	/* Set up for receiving a PPM input signal.
	*/
	pwm_ctx_st *pctx;

	if (pwm_ctxs[PPM_RX_IDX].used == 0)
	{
		pwm_rx_config_st const * pconfig = &pwm_rx_configs[PPM_RX_IDX];

		pctx = &pwm_ctxs[PPM_RX_IDX];

		/* mark as used */
    	pctx->used = 1;

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
		initInputCapture( pconfig, TIM_ICPolarity_Rising );

		configureTimerCallbacks( pconfig, 0, ppmEdgeCallback, ppmOverflowCallback );

		/* configure the timer
			period == 65535ms
			frequency == 1MHz
		*/
		initPwmTimer( pconfig, 0xffff, 1000000 );

		/* configure the channel interrupt */
		configureTimerInputCaptureCompareChannel( pconfig->tim, pconfig->channel );

		/* configure the overflow interrupt */
		TIM_ITConfig( pconfig->tim, TIM_IT_Update, ENABLE );

	    /* start the timer */
	    TIM_Cmd(pconfig->tim, ENABLE);

	}
	else
		pctx = NULL;

	return pctx;
}

void *openPWMInput( uint_fast8_t nb_channels )
{
	/*
		Set up for receiving PWM input signals.
	*/
	pwm_ctx_st *pctx = NULL;
	uint_fast8_t i;

	for (i=0; i < nb_channels && i < NB_PWM_PORTS; i++)
	{
		if (pwm_ctxs[i].used == 0)
		{
			pwm_rx_config_st const * pconfig = &pwm_rx_configs[i];

			pctx = &pwm_ctxs[i];

			/* mark as used */
	    	pctx->used = 1;
			pctx->pconfig = pconfig;

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
			pctx->last_pin_state = pin_low;
			initInputCapture( pconfig, TIM_ICPolarity_Rising);

			configureTimerCallbacks( pconfig, i, pwmEdgeCallback, pwmOverflowCallback );

			/* configure the timer
				period == 65535ms
				frequency == 1MHz
			*/
			initPwmTimer( pconfig, 0xffff, 1000000 );

			/* configure the channel interrupt */
			configureTimerInputCaptureCompareChannel( pconfig->tim, pconfig->channel );

			/* configure the overflow interrupt */
			TIM_ITConfig( pconfig->tim, TIM_IT_Update, ENABLE );

		    /* start the timer */
		    TIM_Cmd(pconfig->tim, ENABLE);

		}
		else
		{
			// TODO: better hanlding if a channel isn't free
			pctx = NULL;
			break;
		}
	}

	return pctx;
}


void initPWMRx( void )
{
	/* called at startup time. Create mutex for rx_signals */
	rx_signals.rx_signals_mutex = CoCreateMutex();
}

int readRXSignals(void *pctx, uint_fast16_t signals[MAX_RX_SIGNALS])
{
	int result;
	(void)pctx;

	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	result = rx_signals.current_rx_signal_count;
	memcpy( signals, (void *)&rx_signals.rx_signals[rx_signals.latest_valid_signals_index][0], sizeof rx_signals.rx_signals[rx_signals.latest_valid_signals_index] );

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );

	return result;
}

static void handleTIMIRQ( TIM_TypeDef *tim, timer_idx_t timerIndex )
{
    int_fast16_t capture;
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

        if (channelIndex == CH1_IDX && TIM_GetITStatus(tim, TIM_IT_CC1) == SET)
        {
            capture = TIM_GetCapture1(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC1);
            STM_EVAL_LEDToggle(LED4);
        }
        else if (channelIndex == CH2_IDX && TIM_GetITStatus(tim, TIM_IT_CC2) == SET)
        {
            capture = TIM_GetCapture2(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC2);
        }
        else if (channelIndex == CH3_IDX && TIM_GetITStatus(tim, TIM_IT_CC3) == SET)
        {
            capture = TIM_GetCapture3(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC3);
        }
        else if (channelIndex == CH4_IDX && TIM_GetITStatus(tim, TIM_IT_CC4) == SET)
        {
            capture = TIM_GetCapture4(tim);
            TIM_ClearITPendingBit(tim, TIM_IT_CC4);
        }
        else
       	{
       		capture = 0;	/* shouldn't happen */
        }

        if (timer_config->edgeCallback != NULL)
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

void TIM1_UP_TIM16_IRQHandler(void)
{
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


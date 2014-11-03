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
	TIM3_IDX,
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
	volatile uint_fast8_t rx_signal_count;
	volatile uint_fast16_t rx_signals[2][MAX_RX_SIGNALS];

	volatile uint_fast8_t current_rx_signal_count;
	volatile uint_fast8_t latest_valid_signals_index;
	OS_MutexID rx_signals_mutex;
} rx_signals_st;

typedef struct pwm_ctx_st
{
	uint_fast8_t	used;	/* non-zero if pin is in use */
} pwm_ctx_st;

/* one of these for each timer channel */
typedef struct timer_configs_st
{
	void (*edgeCallback)( uint_fast16_t count );
	void (*overflowCallback)( uint_fast16_t count );
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

	/* configure the pin function */
    GPIO_PinAFConfig(pconfig->gpio, pconfig->pinSource, pconfig->pinAF);

	/* configure the pin */
	GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = pconfig->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = pconfig->pinMode;
    GPIO_InitStructure.GPIO_PuPd = pconfig->pinPuPd;

    GPIO_Init(pconfig->gpio, &GPIO_InitStructure);

}

static void initInputCompare( pwm_rx_config_st const * pconfig, uint_fast16_t polarity )
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = pconfig->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
	/* all other fields at defaults */

    TIM_ICInit(pconfig->tim, &TIM_ICInitStructure);
}

static void initTimerTimeBase(TIM_TypeDef *tim, uint_fast16_t period, uint_fast32_t frequency_hz)
{
	RCC_ClocksTypeDef clocks;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period;

	RCC_GetClocksFreq(&clocks);
    TIM_TimeBaseStructure.TIM_Prescaler = (clocks.SYSCLK_Frequency/frequency_hz) - 1;

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
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

static void initPwmTimer( pwm_rx_config_st const * pconfig, uint_fast16_t period, uint_fast32_t frequency_hz )
{
	/* Start the APB1 peripheral clock */
    pconfig->RCC_APBPeriphClockCmd(pconfig->RCC_APBPeriph, ENABLE);

	/* configure timer period and frequency */
    initTimerTimeBase(pconfig->tim, period, frequency_hz);	/* gives 1us resolution over 1-2ms width == 0.1% */

    /* start the timer */
    TIM_Cmd(pconfig->tim, ENABLE);

    /* enable interrupts */
    initTimerNVIC(pconfig->irq);
}

static void configureTimerCallbacks( pwm_rx_config_st const * pconfig, void (*edgeCallback)( uint_fast16_t count ),  void (*overflowCallback)( uint_fast16_t count ) )
{
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

static void ppmOverflowCallback(uint_fast16_t capture)
{
	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	rx_signals.rx_signals[rx_signals.latest_valid_signals_index][1] = capture;

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );
}

static void ppmEdgeCallback(uint_fast16_t capture)
{
	CoEnterMutexSection( rx_signals.rx_signals_mutex );

	rx_signals.rx_signals[rx_signals.latest_valid_signals_index][0] = capture+101;

	CoLeaveMutexSection( rx_signals.rx_signals_mutex );
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

		/* configure the pin */
		initPwmGpio( pconfig );

		/* configure the input compare channel */
		initInputCompare( pconfig, TIM_ICPolarity_Rising );

		configureTimerCallbacks( pconfig, ppmEdgeCallback, ppmOverflowCallback );

		/* configure the timer */
		initPwmTimer( pconfig, 0xffff, 1000000 );

		/* configure the channel interrupt */
		configureTimerInputCaptureCompareChannel( pconfig->tim, pconfig->channel );

		/* configure the oveflow interrupt */
		TIM_ITConfig( pconfig->tim, TIM_IT_Update, ENABLE );

	}
	else
		pctx = NULL;

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
	memcpy( signals, (void *)&rx_signals.rx_signals[rx_signals.latest_valid_signals_index][0], sizeof signals );

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
            timer_config->overflowCallback(capture);
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
	        timer_config->edgeCallback(capture);
        }
    }

	CoExitISR();
}

void TIM3_IRQHandler(void)
{
    handleTIMIRQ(TIM3, TIM3_IDX);
}


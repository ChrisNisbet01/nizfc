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

typedef uint32_t capture_t;

#define PWM_FREQUENCY_HZ	1000000

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

} pwm_timer_config_st;

static const pwm_timer_config_st pwm_tx_timer_configs[] =
{
	{
		.pin = GPIO_Pin_12,
		.pinSource = GPIO_PinSource12,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOD,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOD,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM4,
		.tim = TIM4,
		.channel = TIM_Channel_1
	},
	{
		.pin = GPIO_Pin_13,
		.pinSource = GPIO_PinSource13,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOD,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOD,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM4,
		.tim = TIM4,
		.channel = TIM_Channel_2
	},
	{
		.pin = GPIO_Pin_14,
		.pinSource = GPIO_PinSource14,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOD,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOD,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM4,
		.tim = TIM4,
		.channel = TIM_Channel_3
	},
	{
		.pin = GPIO_Pin_15,
		.pinSource = GPIO_PinSource15,
		.pinAF = GPIO_AF_2,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOD,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOD,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM4,
		.tim = TIM4,
		.channel = TIM_Channel_4
	},
	{
		.pin = GPIO_Pin_1,
		.pinSource = GPIO_PinSource1,
		.pinAF = GPIO_AF_1,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOA,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOA,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM2,
		.tim = TIM2,
		.channel = TIM_Channel_1
	},
	{
		.pin = GPIO_Pin_2,
		.pinSource = GPIO_PinSource2,
		.pinAF = GPIO_AF_1,
		.pinOutputType = GPIO_OType_PP,
		.pinPuPd = GPIO_PuPd_DOWN,
		.gpio = GPIOA,
		.RCC_AHBPeriph = RCC_AHBPeriph_GPIOA,

		/* Timer related */
		.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
		.RCC_APBPeriph = RCC_APB1Periph_TIM2,
		.tim = TIM2,
		.channel = TIM_Channel_2
	},
};
#define NB_PWM_TX_PORTS	(sizeof(pwm_tx_timer_configs)/sizeof(pwm_tx_timer_configs[0]))
#define PWM_CONFIG_INDEX(p)	((p)-&pwm_tx_timer_configs[0])

typedef struct pwm_timer_st
{
	uint_fast8_t				used;					/* non-zero if pin is in use */
	pwm_timer_config_st         const * timer_config;	/* the GPIO/TIM details */
	capture_t					volatile * capture_register;
} pwm_timer_st;

static pwm_timer_st 	pwm_timers[NB_PWM_TX_PORTS];

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

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

static void enablePWMTx( pwm_timer_config_st const * timer_config, uint_fast16_t period, unsigned int initialValue )
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

	/* configure the timer
		frequency == 1MHz
		period == ~400Hz for ESC
	*/
	initTimerTimeBase( timer_config->tim, period, PWM_FREQUENCY_HZ );
	pwmOCConfig( timer_config->tim, timer_config->channel, initialValue );

    /* start the timer */
    TIM_Cmd(timer_config->tim, ENABLE);
}

static pwm_timer_config_st const *pwmTimerConfigLookup( pin_st const * pin )
{
	uint_fast8_t index;

	for (index = 0; index < NB_PWM_TX_PORTS; index++)
	{
		if (pin->pin == pwm_tx_timer_configs[index].pin && pin->gpio == pwm_tx_timer_configs[index].gpio)
			return &pwm_tx_timer_configs[index];
	}

	return NULL;
}

void setPWMTXWidth( void const * const pv, unsigned int pulseWidthMillsecs )
{
	pwm_timer_st const * const pctx = pv;

	*pctx->capture_register = pulseWidthMillsecs;
}

void * openPwmTxTimer( pin_st const * const pin, unsigned int pulseRateHz, unsigned int initialValue )
{
	pwm_timer_st *pctx = NULL;
	pwm_timer_config_st const * timer_config;

	timer_config = pwmTimerConfigLookup(pin);

	if (timer_config == NULL)
		goto done;

	pctx = &pwm_timers[PWM_CONFIG_INDEX(timer_config)];
	pctx->timer_config = timer_config;
	switch ( timer_config->channel )
	{
		case TIM_Channel_1:
			pctx->capture_register = &timer_config->tim->CCR1;
			break;
		case TIM_Channel_2:
			pctx->capture_register = &timer_config->tim->CCR1;
			break;
		case TIM_Channel_3:
			pctx->capture_register = &timer_config->tim->CCR1;
			break;
		case TIM_Channel_4:
			pctx->capture_register = &timer_config->tim->CCR1;
			break;
	}
	enablePWMTx( timer_config, PWM_FREQUENCY_HZ/pulseRateHz, initialValue );

done:

	return pctx;
}


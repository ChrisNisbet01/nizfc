#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f30x_tim.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>

#include "i2c_stm32f30x.h"


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
		.channel = TIM_Channel_1
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
		.channel = TIM_Channel_2
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
		.channel = TIM_Channel_3
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
		.channel = TIM_Channel_4
	}
};




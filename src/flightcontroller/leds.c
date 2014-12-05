#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F30X)
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#elif defined(STM32F10X)
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#endif
#include <leds.h>
#include <utils.h>

typedef struct led_configuration_st
{
	uint16_t     pin;
	GPIO_TypeDef * port;
	uint32_t     clk;
} led_configuration_st;

static const led_configuration_st leds[] =
{
#if defined(STM32F30X)
	[LED1] =
		{
		.pin = GPIO_Pin_9,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		},
	[LED2] =
		{
		.pin = GPIO_Pin_8,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		},
	[LED3] =
		{
		.pin = GPIO_Pin_10,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		},
	[LED4] =
		{
		.pin = GPIO_Pin_15,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		},
	[LED5] =
		{
		.pin = GPIO_Pin_11,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		},
	[LED6] =
		{
		.pin = GPIO_Pin_14,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		},
	[LED7] =
		{
		.pin = GPIO_Pin_12,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		},
	[LED8] =
		{
		.pin = GPIO_Pin_13,
		.port = GPIOE,
		.clk = RCC_AHBPeriph_GPIOE
		}
#elif defined(STM32F10X)
	[LED1] =
		{
		.pin = GPIO_Pin_4,
		.port = GPIOB,
		.clk = RCC_APB2Periph_GPIOB
		},
	[LED2] =
		{
		.pin = GPIO_Pin_3,
		.port = GPIOB,
		.clk = RCC_APB2Periph_GPIOB
		}
#endif
};

static void LEDInit(led_configuration_st const * led)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
#if defined(STM32F30X)
	RCC_AHBPeriphClockCmd(led->clk, ENABLE);
#elif defined(STM32F10X)
	RCC_APB2PeriphClockCmd(led->clk, ENABLE);
#endif

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = led->pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
#if defined(STM32F30X)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
#elif defined(STM32F10X)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif

	GPIO_Init(led->port, &GPIO_InitStructure);

}

void setLED( led_t led, led_state_t state )
{
	if ( led < ARRAY_SIZE(leds) && leds[led].pin != 0 )
	{
		switch ( state )
		{
			case led_state_off:
#if defined(STM32F30X)
				leds[led].port->BRR = leds[led].pin;
#elif defined(STM32F10X)
				leds[led].port->BSRR = leds[led].pin;
#endif
				break;
			case led_state_on:
#if defined(STM32F30X)
				leds[led].port->BSRR = leds[led].pin;
#else
				leds[led].port->BRR = leds[led].pin;
#endif
				break;
			case led_state_toggle:
				leds[led].port->ODR ^= leds[led].pin;
				break;
			default:	// TODO:
				break;
		}
	}
}

void initLEDs( void )
{
	// TODO: LED task to handle state switches
	unsigned int index;

	for ( index = 0; index < ARRAY_SIZE(leds); index++ )
	{
		LEDInit(&leds[index]);
		setLED((led_t)index,led_state_off);
	}
}


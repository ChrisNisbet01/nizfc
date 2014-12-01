#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include <leds.h>
#include <utils.h>

typedef struct led_configuration_st
{
	uint16_t     pin;
	GPIO_TypeDef * port;
	uint32_t     clk;
} led_configuration_st;

static led_configuration_st leds[] =
{
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
};

static void LEDInit(led_configuration_st *led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(led->clk, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = led->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(led->port, &GPIO_InitStructure);
}

void setLED( led_t led, led_state_t state )
{
	if ( led > 0 && led < ARRAY_SIZE(leds) && leds[led].pin != 0 )
	{
		switch ( state )
		{
			case led_state_off:
				leds[led].port->BRR = leds[led].pin;
				break;
			case led_state_on:
				leds[led].port->BSRR = leds[led].pin;
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
	}
}


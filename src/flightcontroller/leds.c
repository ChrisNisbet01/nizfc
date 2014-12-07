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
#include <coos.h>
#include <leds.h>
#include <utils.h>

#define LEDS_TASK_STACK_SIZE 0x80
#define MS_PER_UPDATE			20

static OS_STK ledsTaskStack[LEDS_TASK_STACK_SIZE];

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
#define NB_LEDS ARRAY_SIZE(leds)

typedef enum flash_state_t
{
	flash_off,
	flash_on
} flash_state_t;

typedef enum pin_transtion_time_t
{
	slow_flash_on = 800,
	slow_flash_off = 800,
	fast_flash_on = 200,
	fast_flash_off = 200,
	fast_slow_flash_on = 200,
	fast_slow_flash_off = 800,
	slow_fast_flash_on = 800,
	slow_fast_flash_off = 200
} pin_transtion_time_t;

typedef struct ledContext_st
{
	volatile led_state_t currentMode;
} ledContext_st;

typedef enum flashIndex_t
{
	SLOW_FLASH_IDX,
	FAST_FLASH_IDX,
	SLOW_FAST_FLASH_IDX,
	FAST_SLOW_FLASH_IDX,
} flashIndex_t;

typedef struct ledFlashConfig_st
{
	uint16_t offCount;
	uint16_t onCount;
} ledFlashConfig_st;

typedef struct ledFlashContext_st
{
	flash_state_t flash_state;
	unsigned int currentCount;
} ledFlashContext_st;

static const ledFlashConfig_st ledFlashConfigs[] =
{
	[SLOW_FLASH_IDX] =
	{
		.offCount = slow_flash_off,
		.onCount = slow_flash_on
	},
	[FAST_FLASH_IDX] =
	{
		.offCount = fast_flash_off,
		.onCount = fast_flash_on
	},
	[SLOW_FAST_FLASH_IDX] =
	{
		.offCount = slow_fast_flash_off,
		.onCount = slow_fast_flash_on
	},
	[FAST_SLOW_FLASH_IDX] =
	{
		.offCount = fast_slow_flash_off,
		.onCount = fast_slow_flash_on
	}
};
#define NB_FLASH_CONFIGS	(ARRAY_SIZE(ledFlashConfigs))

static ledContext_st ledContexts[NB_LEDS];
static ledFlashContext_st ledFlashContexts[NB_FLASH_CONFIGS];

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

static void setLED( led_t led, led_state_t state )
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
#elif defined(STM32F10X)
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

static void updateFlashContexts( unsigned int dTmillis )
{
	unsigned int index;

	for ( index = 0; index < NB_FLASH_CONFIGS; index++ )
	{
		ledFlashContext_st * ledFlashContext = &ledFlashContexts[index];

		ledFlashContext->currentCount += dTmillis;

		if ( ledFlashContext->flash_state == flash_off )
		{
			if ( ledFlashContext->currentCount >= ledFlashConfigs[index].offCount )
			{
				ledFlashContext->flash_state = flash_on;
				ledFlashContext->currentCount = 0;
			}
		}
		else if ( ledFlashContext->currentCount >= ledFlashConfigs[index].onCount )
		{
			ledFlashContext->flash_state = flash_off;
			ledFlashContext->currentCount = 0;
		}
	}
}

static void updateLedStates( void )
{
	unsigned int index;

	for( index = 0; index < NB_LEDS; index++ )
	{
		switch( ledContexts[index].currentMode )
		{
			case led_state_off:
				setLED( index, led_state_off );
				break;
			case led_state_on:
				setLED( index, led_state_on );
				break;
			case led_state_toggle:
				setLED( index, led_state_toggle );
				break;
			case led_state_slow_fast:
				setLED( index, ledFlashContexts[SLOW_FAST_FLASH_IDX].flash_state );
				break;
			case led_state_fast_slow:
				setLED( index, ledFlashContexts[FAST_SLOW_FLASH_IDX].flash_state );
				break;
			case led_state_fast_flash:
				setLED( index, ledFlashContexts[FAST_FLASH_IDX].flash_state );
				break;
			case led_state_slow_flash:
				setLED( index, ledFlashContexts[SLOW_FLASH_IDX].flash_state );
				break;
		}
	}
}

static void ledsTask( void *pv )
{
	UNUSED(pv);

	while ( 1 )
	{
		CoTickDelay( MSTOTICKS( MS_PER_UPDATE ) );

		updateFlashContexts(MS_PER_UPDATE);

		updateLedStates();
	}
}

void setLEDMode( led_t led, led_state_t state )
{
	if ( led < ARRAY_SIZE(leds) && leds[led].pin != 0 )
		ledContexts[led].currentMode = state;
}

void initLEDs( void )
{
	unsigned int index;

#if defined(STM32F10X)
    // Turn off JTAG port because we're using the GPIO for leds
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE );
#endif

	for ( index = 0; index < NB_LEDS; index++ )
	{
		LEDInit(&leds[index]);
		setLED((led_t)index,led_state_off);
	}

	for ( index = 0; index < NB_FLASH_CONFIGS; index++ )
		ledFlashContexts[index].flash_state = led_state_off;

	CoCreateTask(ledsTask, Co_NULL, 2, &ledsTaskStack[LEDS_TASK_STACK_SIZE-1], LEDS_TASK_STACK_SIZE);

}


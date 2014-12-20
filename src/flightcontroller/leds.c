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
#include <config.h>
#include <polling.h>
#include <leds.h>
#include <utils.h>

#define LEDS_TASK_STACK_SIZE 0x80
#define MS_PER_UPDATE			20

#define RESET_PIN( port, pin )	port->BRR = (pin)
#define SET_PIN( port, pin )	port->BSRR = (pin)
#define TOGGLE_PIN( port, pin )	port->ODR ^= (pin)

#if defined(STM32F30X)
#define LED_OFF( port, pin )	RESET_PIN(port, pin)
#define LED_ON( port, pin )		SET_PIN(port, pin)
#elif defined(STM32F10X)
#define LED_OFF( port, pin )	SET_PIN(port, pin)
#define LED_ON( port, pin )		RESET_PIN(port, pin)
#endif
#define LED_TOGGLE( port, pin ) TOGGLE_PIN( port, pin )

static OS_STK ledsTaskStack[LEDS_TASK_STACK_SIZE];
static OS_TID ledsTaskID;

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

/* definitions for the periods of time (in ms) the LEDs are ON/OFF when flashing */
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
	volatile led_mode_t currentMode;
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

static void setLED( led_t led, led_mode_t state )
{
	if ( led < ARRAY_SIZE(leds) && leds[led].pin != 0 )
	{
		switch ( state )
		{
			case led_state_off:
				LED_OFF( leds[led].port, leds[led].pin );
				break;
			case led_state_on:
				LED_ON( leds[led].port, leds[led].pin );
				break;
			case led_state_toggle:
				LED_TOGGLE( leds[led].port, leds[led].pin );
				break;
			default:
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
			case led_state_slow_fast:
				setLED( (led_t)index, ledFlashContexts[SLOW_FAST_FLASH_IDX].flash_state );
				break;
			case led_state_fast_slow:
				setLED( (led_t)index, ledFlashContexts[FAST_SLOW_FLASH_IDX].flash_state );
				break;
			case led_state_fast_flash:
				setLED( (led_t)index, ledFlashContexts[FAST_FLASH_IDX].flash_state );
				break;
			case led_state_slow_flash:
				setLED( (led_t)index, ledFlashContexts[SLOW_FLASH_IDX].flash_state );
				break;
			default:
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

void setLEDMode( led_t led, led_mode_t state )
{
	if ( led < ARRAY_SIZE(leds) && leds[led].pin != 0 )
	{
		/*
			It may happen that a task is blocking this one for some period of time, and wishes to turn an LED ON/OFF/TOGGLE.
			So that this can occur without waiting for the LED task to run, we'll drive the LED directly in these cases.
		*/
		ledContexts[led].currentMode = state;
		switch( ledContexts[led].currentMode )
		{
			case led_state_off:
				setLED( led, led_state_off );
				break;
			case led_state_on:
				setLED( led, led_state_on );
				break;
			case led_state_toggle:
				setLED( led, led_state_toggle );
				break;
			default:
				break;
		}
	}
}

static void suspendLEDSTask( void )
{
	CoSuspendTask( ledsTaskID );
}

static void resumeLEDSTask( void )
{
	CoAwakeTask( ledsTaskID );
}

int ledsTaskPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	UNUSED(pv);

	switch( poll_id )
	{
		case poll_id_suspend_task:
			suspendLEDSTask();
			break;
		case poll_id_resume_task:
			resumeLEDSTask();
			break;
		default:
			break;
	}

	return result;
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

	ledsTaskID = CoCreateTask(ledsTask, Co_NULL, LEDS_TASK_PRIORITY, &ledsTaskStack[LEDS_TASK_STACK_SIZE-1], LEDS_TASK_STACK_SIZE);

}


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
#include <utils.h>
#include <pins.h>
#include <pwm_tx_stm32f30x.h>
#include <outputs.h>
#include <output_configuration.h>

static const pin_st pwm_tx_pins[] =
{
	{
		.pin = GPIO_Pin_12,
		.gpio = GPIOD
	},
	{
		.pin = GPIO_Pin_13,
		.gpio = GPIOD
	},
	{
		.pin = GPIO_Pin_14,
		.gpio = GPIOD
	},
	{
		.pin = GPIO_Pin_15,
		.gpio = GPIOD
	},
	{
		.pin = GPIO_Pin_1,
		.gpio = GPIOA
	},
	{
		.pin = GPIO_Pin_2,
		.gpio = GPIOA
	}
};

typedef struct pwmOutput_st
{
	bool used;
	void *pwmTimerCtx;
} pwmOutput_st;

static pwmOutput_st pwmOutputs[ARRAY_SIZE(pwm_tx_pins)];

void setMotorOutput( unsigned int motor, unsigned int pulseWidthMillsecs )
{
	if (motor < ARRAY_SIZE(pwm_tx_pins) && pwmOutputs[motor].used == true)
	{
		setPwmTxPulseWidth( pwmOutputs[motor].pwmTimerCtx, pulseWidthMillsecs );
	}
}

void openOutputs( void )
{
	unsigned int motor;

	// TODO: lookup number of outputs from configuration/profile

	for (motor = 0; motor < ARRAY_SIZE(pwm_tx_pins); motor++ )
	{
		pwmOutputs[motor].used = true;
		pwmOutputs[motor].pwmTimerCtx = openPwmTxTimer( &pwm_tx_pins[motor], output_configuration[0].pwmrate, 0 );
	}

}


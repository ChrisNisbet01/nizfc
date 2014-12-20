#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <coocox.h>
#include <utils.h>
#include <polling.h>
#include <hirestimer.h>
#include <imu.h>
#include <i2c.h>
#include <receiver.h>
#include <sensors.h>
#if defined(STM32F30X)
#include <spi.h>
#include <lsm303dlhc.h>
#include <l3gd20.h>
#elif defined(STM32F10X)
#include <stm32f10x_rcc.h>
#include <mpu6050.h>
#endif
#include <config.h>
#include <receiver_handler.h>
#include <pid_control.h>
#include <motor_control.h>
#include <failsafe.h>
#include <filter.h>

#include <leds.h>

#define RECEIVER_TASK_STACK_SIZE 0x200
static OS_STK receiver_task_stack[RECEIVER_TASK_STACK_SIZE];

static OS_FlagID receiverFlag;
static OS_FlagID failsafeTriggerFlag;

static volatile uint32_t latestChannelsReceived;

static void failsafeTriggerCallback( void )
{
	/* called from within the context of a COOS timer handler, so within an ISR */
	isr_SetFlag( failsafeTriggerFlag );
}

static void newReceiverDataCallback( uint32_t newChannelsReceived )
{
	latestChannelsReceived |= newChannelsReceived;
	isr_SetFlag( receiverFlag );
}

static void updateReceiver( void )
{
	uint32_t newChannels;

	IRQ_DISABLE_SAVE();
	newChannels = latestChannelsReceived;
	latestChannelsReceived = 0;
	IRQ_ENABLE_RESTORE();

    updateFailsafeWithNewChannels( newChannels );

    setLEDMode(RX_LED, led_state_toggle);

	processReceiverSignals();
}

static void receiver_task( void *pv )
{
	UNUSED(pv);

	receiverFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	failsafeTriggerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );

	initFailsafe( failsafeTriggerCallback );

	openReceiver( newReceiverDataCallback );

	while (1)
	{
		StatusType err;
		U32 ReadyFlags;

		ReadyFlags = CoWaitForMultipleFlags( (1<<receiverFlag) | (1<<failsafeTriggerFlag), OPT_WAIT_ANY, 0, &err );
		if (err == E_OK)
		{
			if ( (ReadyFlags & (1<<failsafeTriggerFlag)) != 0 )
			{
				failsafeSetTriggered();
			}
			if ( (ReadyFlags & (1<<receiverFlag)) != 0 )
			{
				updateReceiver();
			}
		}
	}
}

void initReceiverTask( void )
{
	CoCreateTask(receiver_task, Co_NULL, RECEIVER_TASK_PRIORITY, &receiver_task_stack[RECEIVER_TASK_STACK_SIZE-1], RECEIVER_TASK_STACK_SIZE);
}


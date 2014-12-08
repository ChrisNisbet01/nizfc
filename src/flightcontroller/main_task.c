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
#include <receiver_handler.h>
#include <pid_control.h>
#include <motor_control.h>
#include <failsafe.h>
#include <filter.h>

#include <board_alignment.h>
#include <board_configuration.h>
#include <leds.h>

#define MAIN_TASK_STACK_SIZE 0x200
static OS_STK main_task_stack[MAIN_TASK_STACK_SIZE];

static OS_FlagID receiverFlag;
static OS_FlagID failsafeTriggerFlag;
static OS_FlagID IMUTimerFlag;

static volatile uint32_t latestChannelsReceived;
static void *i2c_port;
#if defined(STM32F30X)
static void *spi_port;
#endif

static sensorCallback_st sensorCallbacks;


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

static void IMUCallback( void )
{
	/*
		Called by TIM3 ISR with period set by board_configuration (~3000us)
	*/
	CoEnterISR();

	/* signal the main task that it's time to update the IMU etc */
	isr_SetFlag( IMUTimerFlag );

	CoExitISR();
}

static void initIMUTimer( uint32_t updatePeriod )
{
	IMUTimerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	/* start a three millisecond timer */
	initHiResTimer( updatePeriod, IMUCallback );

	/*
		we get an interrupt almost immediately after we start the timer.
		Pretend we've processed the loop one cycle ago.
	*/
	initIMUTime( micros() - updatePeriod );
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

volatile U32 OSIdleCtr;
static U32 OSMaxIdleCtr;

static void suspendTasks( void )
{
	pollCodeGroups( poll_id_suspend_task, NULL, true );
}

static void resumeTasks( void )
{
	pollCodeGroups( poll_id_resume_task, NULL, true );
}

uint_fast32_t getCPULoad( void )
{
	uint_fast32_t cpuLoad;

	/*
		It is assumed that this function will be called at 1 second intervals
	*/
	if ( OSIdleCtr > OSMaxIdleCtr )
		OSMaxIdleCtr = OSIdleCtr;
	if ( OSMaxIdleCtr > 0 )
	{
		cpuLoad = ((OSMaxIdleCtr - OSIdleCtr) * 100)/OSMaxIdleCtr;
	}
	else
		cpuLoad = 0;

	OSIdleCtr = 0;

	return cpuLoad;
}

static void main_task( void *pv )
{
	UNUSED(pv);

	/*
		suspend all other tasks except idle task
		then sit idle for 250ms.
		then take note of OSIdleCtr. Multiply by 4 to get one seconds worth.
		This is the max number that the idle task can reach
		in 1 second, so we can determine % load from this.
		% load = ((maxIdleCtr - thisIdleCtr)*100)/maxIdleCtr;
	*/
	suspendTasks();
	OSIdleCtr = 0;
	CoTickDelay( MSTOTICKS(250) );
	MEMORY_BARRIER();
	OSMaxIdleCtr = OSIdleCtr * 4;
	resumeTasks();

	receiverFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	failsafeTriggerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );

#if defined(STM32F30X)
	i2c_port = i2cInit( I2C_PORT_1 );
#elif defined(STM32F10X)
	i2c_port = i2cInit( I2C_PORT_2 );
#endif

	if ( i2c_port != NULL )
	{
		sensorConfig_st sensorConfig;

		sensorConfig.i2cCtx = i2c_port;
#if defined(STM32F30X)
		initLSM303DLHC( &sensorConfig, &sensorCallbacks );
#elif defined(STM32F10X)
		initMPU6050( &sensorConfig, &sensorCallbacks );
#endif
	}

#if defined(STM32F30X)
	spi_port = spiInit( SPI_PORT_1 );

	if (spi_port != NULL )
	{
		sensorConfig_st sensorConfig;

		sensorConfig.spiCtx = spi_port;
		initL3GD20( &sensorConfig, &sensorCallbacks );
	}
#endif

	if ( sensorCallbacks.readMagnetometer == NULL )
	{
		initGyroHeadingVector();
	}

	initBoardAlignment(
		board_configuration[0].boardOrientation[0],
		board_configuration[0].boardOrientation[1],
		board_configuration[0].boardOrientation[2] );

	initFailsafe( failsafeTriggerCallback );

	initPIDControl();
	initMotorControl(board_configuration[0].craftType);

	openReceiver( newReceiverDataCallback );

	initIMUTimer(board_configuration[0].updateTime);

	while (1)
	{
		StatusType err;
		U32 ReadyFlags;

		ReadyFlags = CoWaitForMultipleFlags( (1<<IMUTimerFlag) | (1<<receiverFlag) | (1<<failsafeTriggerFlag), OPT_WAIT_ANY, 0, &err );
		if (err == E_OK)
		{
			if ( (ReadyFlags & (1<<failsafeTriggerFlag)) != 0 )
			{
				failsafeSetTriggered();
			}
			if ( (ReadyFlags & (1<<IMUTimerFlag)) != 0 )
			{
				updateIMU(&sensorCallbacks);
				updatePIDControlLoops();
				updateMotorOutputs();
			}
			if ( (ReadyFlags & (1<<receiverFlag)) != 0 )
			{
				updateReceiver();
			}
		}
	}
}

bool accelerometerSensorFound( void )
{
	return (sensorCallbacks.readAccelerometer != NULL) ? true : false;
}

void initMainTask( void )
{
	CoCreateTask(main_task, Co_NULL, 0, &main_task_stack[MAIN_TASK_STACK_SIZE-1], MAIN_TASK_STACK_SIZE);
}


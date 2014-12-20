#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <coocox.h>
#include <polling.h>
#include <serial.h>
#include <cli.h>
#include <pid_control.h>
#include <utils.h>
#include <config.h>
#include <receiver.h>
#include <receiver_handler.h>
#include <motor_control.h>
#include <imu.h>
#include <main_task.h>

#define CLI_TASK_STACK_SIZE 0x200

typedef struct serialCli_st
{
	serial_port_st *cli_uart;
	void *pcli;
} serialCli_st;

static OS_STK cli_task_stack[CLI_TASK_STACK_SIZE];
static OS_TID serialTaskID;

static const serial_port_t serial_ports[] =
{
#if defined(STM32F30X)
	SERIAL_UART_2,
	SERIAL_USB
#elif defined(STM32F10X)
	SERIAL_UART_1
#endif
};

static serialCli_st serialCli[ARRAY_SIZE(serial_ports)];

serial_port_st *debug_port;
static OS_FlagID cliUartFlag;
static OS_FlagID periodicTasksTimerFlag;
static uint32_t cpuLoad;

int uartPutChar( void * port, int ch )
{
	serial_port_st * serialPort = port;
	int result = serialPort->methods->writeCharBlockingWithTimeout( serialPort->serialCtx, ch, 10 );

	return result;
}

static void debugTimer( void )
{
	isr_SetFlag( periodicTasksTimerFlag );
}

static void newUartData( void *pv )
{
	UNUSED(pv);

	CoEnterISR();

	isr_SetFlag(cliUartFlag);

	CoExitISR();
}

static void handleNewSerialData( void )
{
	unsigned int uart_index;

	for (uart_index = 0; uart_index < ARRAY_SIZE(serialCli); uart_index++ )
	{
		if ( serialCli[uart_index].cli_uart != NULL )
		{
			while ( serialCli[uart_index].cli_uart->methods->rxReady( serialCli[uart_index].cli_uart->serialCtx ) )
			{
				uint8_t ch;

				ch = serialCli[uart_index].cli_uart->methods->readChar( serialCli[uart_index].cli_uart->serialCtx );
				cliHandleNewChar( serialCli[uart_index].pcli, ch );
			}
		}
	}
}

static void doDebugOutput( void )
{
	if (board_configuration[0].debug & 1 )
	{
 		//printf("\n\nthrottle: %d", (int)getThrottleSetpoint() );
 		//printf("\nroll: %d:%f PID: %d", (int)getRollAngleSetpoint(), RollAngle, (int)getRollAnglePIDOutput() );
 		//printf("\npitch: %d:%f PID: %d", (int)getPitchAngleSetpoint(), PitchAngle, (int)getPitchAnglePIDOutput() );
 		printf("\nroll rate: %d:%f PID: %d", (int)getRollRateSetpoint(), filteredGyroValues[0], (int)getRollRatePIDOutput() );
 		printf("\npitch rate: %d:%f PID: %d", (int)getPitchRateSetpoint(), filteredGyroValues[1], (int)getPitchRatePIDOutput() );
 		//printf("\nheading: %g", &Heading );
	}
	if (board_configuration[0].debug & 2 )
	{
		int motor;

		printf("\r\n");
		for (motor = 0; motor < 4; motor++ )
			printf("  m-%d: %u", motor+1, (unsigned int)getMotorOutput( motor ) );
	}
	if (board_configuration[0].debug & 4 )
	{
		float exeTime = getIMUExeTime();
		printf("\r\nIMU exe time %f cpu %d", exeTime, cpuLoad );
	}
	if (board_configuration[0].debug & 8 )
	{
		printf( "\r\n\ndT: %fg", fIMUDelta );
		printf( "\r\nfiltered" );
		printf( "\r\n        roll %f", RollAngle );
		printf( "\r\n        pit  %f", PitchAngle );
		printf( "\r\nsetpoint" );
		printf( "\r\n        roll %f", getRollAngleSetpoint() );
		printf( "\r\n        pit  %f", getPitchAngleSetpoint() );
	}
	if (board_configuration[0].debug & 16 )
	{
		int channel;

		printf("\r\n");
		for (channel = 4; channel < 8; channel++ )
			printf("  aux-%d: %u", channel+1, (unsigned int)readReceiverChannel(channel));
	}
}

static void doPeriodicTasks( void )
{
	cpuLoad = getCPULoad();
	doDebugOutput();
}

static void cli_task( void *pv )
{
	OS_TCID debugTimerID;

	UNUSED(pv);

	debugTimerID = CoCreateTmr( TMR_TYPE_PERIODIC, CFG_SYSTICK_FREQ, CFG_SYSTICK_FREQ, debugTimer );
	CoStartTmr( debugTimerID );

	while (1)
	{
		StatusType err;
		U32 readyFlags;

		readyFlags = CoWaitForMultipleFlags( (1 << periodicTasksTimerFlag) | (1 << cliUartFlag), OPT_WAIT_ANY, 0, &err );
	 	if ( (readyFlags & (1 << cliUartFlag)) )
	 		handleNewSerialData();

	 	if ( (readyFlags & (1 << periodicTasksTimerFlag)) )
	 		doPeriodicTasks();
	}
}

bool setDebugPort( int port )
{
	bool debugPortAssigned = true;

	if ( port < 0 )
		debug_port = NULL;
	else if ( (unsigned)port < ARRAY_SIZE(serialCli) && serialCli[port].cli_uart != NULL )
		debug_port = serialCli[port].cli_uart;
	else
		debugPortAssigned = false;

	return debugPortAssigned;
}

static void suspendSerialTask( void )
{
	CoSuspendTask( serialTaskID );
}

static void resumeSerialTask( void )
{
	CoAwakeTask( serialTaskID );
}

int serialTaskPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	UNUSED(pv);

	switch( poll_id )
	{
		case poll_id_suspend_task:
			suspendSerialTask();
			break;
		case poll_id_resume_task:
			resumeSerialTask();
			break;
		default:
			break;
	}

	return result;
}

void initSerialTask( void )
{
	unsigned int cli_index;

	cliUartFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	periodicTasksTimerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );

	for ( cli_index = 0 ; cli_index < ARRAY_SIZE(serial_ports); cli_index++ )
	{
		serialCli[cli_index].cli_uart = serialOpen( serial_ports[cli_index], 115200, uart_mode_rx | uart_mode_tx, newUartData );
		if ( serialCli[cli_index].cli_uart != NULL )
			serialCli[cli_index].pcli = initCli( serialCli[cli_index].cli_uart );
	}

	CoCreateTask(cli_task, Co_NULL, SERIAL_TASK_PRIORITY, &cli_task_stack[CLI_TASK_STACK_SIZE-1], CLI_TASK_STACK_SIZE);

}

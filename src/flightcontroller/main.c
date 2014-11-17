#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <coos.h>
#include <stm32f3_discovery.h>
#include <uart.h>
#include <utils.h>
#include <polling.h>
#include <i2c_stm32f30x.h>
#include <spi_stm32f30x.h>
#include <receiver.h>
#include <outputs.h>
#include <output_configuration.h>
#include <cli.h>
#include <startup.h>
#include <sensors.h>
#include <lsm303dlhc.h>
#include <l3gd20.h>
#include <roll_pitch_configuration.h>
#include <receiver_handler.h>
#include <motor_control.h>
#include <hirestimer.h>
#include <attitude_estimation.h>

#define MAIN_TASK_STACK_SIZE 0x200
#define CLI_TASK_STACK_SIZE 0x200

static OS_STK cli_task_stack[CLI_TASK_STACK_SIZE];
static OS_STK main_task_stack[MAIN_TASK_STACK_SIZE];

void *cli_uart;
OS_FlagID cliUartFlag;

void *i2c_port;
void *spi_port;
void *pcli;
void *lsm303dlhcDevice;
void *l3gd20Device;
static sensorCallback_st sensorCallbacks;

float RollAng, PitchAng, Heading;
float RollAngFiltered, PitchAngFiltered, Heading;

static int uartPutChar( int ch )
{
	return uartWriteCharBlockingWithTimeout( cli_uart, ch, 10 );
}

static float calculateHeading( float *magValues, float *accValues )
{
	float fNormAcc, fSinRoll, fCosRoll, fSinPitch, fCosPitch, acosPitch, acosRoll;
	float fTiltedX ,fTiltedY;
	float heading;	/* in degrees */

	fNormAcc = sqrt((accValues[0]*accValues[0])+(accValues[1]*accValues[1])+(accValues[2]*accValues[2]));

	fSinRoll = -accValues[1]/fNormAcc;
	fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
	fSinPitch = accValues[0]/fNormAcc;
	fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
	acosPitch = acos(fCosPitch)*180.0f/M_PI;
	acosRoll = acos(fCosRoll)*180.0/M_PI;

	RollAng = (atan2(-accValues[1], accValues[2])*180.0)/M_PI;
    PitchAng = (atan2(accValues[0], sqrt(accValues[1]*accValues[1] + accValues[2]*accValues[2]))*180.0)/M_PI;

	fTiltedX = magValues[0] * fCosPitch + magValues[2] * fSinPitch;
	fTiltedY = magValues[0] * fSinRoll * fSinPitch  + magValues[1] * fCosRoll - magValues[1] * fSinRoll * fCosPitch;

	heading = (float) ((atan2f(fTiltedY,fTiltedX))*180.0f)/M_PI;

	if (heading < 0)
	{
		heading = heading + 360.0f;
	}

	return heading;
}

OS_FlagID printTimerFlag;
OS_FlagID receiverFlag;
OS_FlagID IMUTimerFlag;

static void printTimer( void )
{
	CoSetFlag( printTimerFlag );
}

static void newReceiverDataCallback( void )
{
	CoSetFlag( receiverFlag );
}

static uint32_t IMUDelta;
static uint32_t IMUExeTime;
static float    fIMUDelta;
static uint32_t lastIMUTime;

static void IMUCallback( void )
{
	CoEnterISR();

	CoSetFlag( IMUTimerFlag );

	CoExitISR();
}

IMU_DATA_ST imu_data;

static void initIMU( void )
{
    init_attitude_estimation( &imu_data, 0.07f, 0.07f );

	IMUTimerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	/* start a three millisecond timer */
	initHiResTimer( 3000, IMUCallback );
	/*
		we get an interrupt almost immediately after we start the timer.
		Pretend we've processed the loop one cycle ago.
	*/
	lastIMUTime = micros2()-3000;
}

float accelerometerValues[3];
float smoothedAccelerometerValues[3];
float magnetometerValues[3];
float gyroValues[3];

static void estimateAttitude( float dT )
{

    do_attitude_estimation( &imu_data,
    				dT,
    				gyroValues[0],
    				gyroValues[1],
    				accelerometerValues[0],
    				accelerometerValues[1],
    				accelerometerValues[2] );

    RollAngFiltered = imu_data.compAngleX;
    PitchAngFiltered = imu_data.compAngleY;

}

static void IMUHandler( void )
{
	uint32_t temp;
	uint32_t now = micros2();
	IMUDelta = now - lastIMUTime;
	temp = lastIMUTime;
	lastIMUTime = now;

	/* calculate time between iterations */
	fIMUDelta = (float)IMUDelta/1000000.0f;

	if ( lsm303dlhcDevice )
	{
		if ( sensorCallbacks.readAccelerometer != NULL
			&& sensorCallbacks.readMagnetometer != NULL
			&& sensorCallbacks.readGyro != NULL
			&& sensorCallbacks.readAccelerometer( lsm303dlhcDevice, accelerometerValues ) == true
			&& sensorCallbacks.readMagnetometer( lsm303dlhcDevice, magnetometerValues ) == true
			&& sensorCallbacks.readGyro( l3gd20Device, gyroValues ) == true)
		{

			estimateAttitude( fIMUDelta );

			updatePIDControlLoops();
		}
	}


	// TODO: process receiver signals on a per frame basis

	updateMotorOutputs();

	IMUExeTime = (now=micros()) - lastIMUTime;
	if (IMUExeTime > 1000)
		printf("\r\nlong %u %u %u", IMUExeTime, now, lastIMUTime);
}

static void main_task( void *pv )
{
	UNUSED(pv);

	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);

	i2c_port = i2cInit( I2C_PORT_1 );
	spi_port = spiInit( SPI_PORT_1 );

	if ( i2c_port != NULL )
	{
		sensorConfig_st sensorConfig;

		sensorConfig.i2cCtx = i2c_port;
		lsm303dlhcDevice = initLSM303DLHC( &sensorConfig, &sensorCallbacks );
	}
	if (spi_port != NULL )
	{
		sensorConfig_st sensorConfig;

		sensorConfig.spiCtx = spi_port;
		l3gd20Device = initL3GD20( &sensorConfig, &sensorCallbacks );
	}

	receiverFlag = CoCreateFlag( Co_TRUE, Co_FALSE );

	initMotorControl();
	openReceiver( newReceiverDataCallback );
	openOutputs();

	initIMU();

	while (1)
	{
		uint_fast16_t rx_value;
		StatusType err;
		U32 ReadyFlags;

		ReadyFlags = CoWaitForMultipleFlags( (1<<IMUTimerFlag) | (1<<receiverFlag), OPT_WAIT_ANY, 0, &err );
		if ( (ReadyFlags & (1<<IMUTimerFlag)) != 0 )
		{
			IMUHandler();
		    STM_EVAL_LEDToggle(LED6);
		}
		if ( (ReadyFlags & (1<<receiverFlag)) != 0 )
		{
		    STM_EVAL_LEDToggle(LED3);
			processStickPositions();
		}
	}
}

void newUartData( void *pv )
{
	if ( pv == cli_uart )
	{
		CoEnterISR();

		CoSetFlag(cliUartFlag);

		CoExitISR();
	}
}

static void cli_task( void *pv )
{
	OS_TCID printTimerID;

	UNUSED(pv);

	cliUartFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	printTimerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );

	printTimerID = CoCreateTmr( TMR_TYPE_PERIODIC, CFG_SYSTICK_FREQ, CFG_SYSTICK_FREQ, printTimer );
	CoStartTmr( printTimerID );

	while (1)
	{
		if ( cli_uart != NULL )
		{
			StatusType err;
			U32 readyFlags;

			readyFlags = CoWaitForMultipleFlags( (1<<printTimerFlag)|(1<<cliUartFlag), OPT_WAIT_ANY, 0, &err );
		 	if ( readyFlags & (1<<cliUartFlag) )
		 	{
				while ( uartRxReady( cli_uart ) )
				{
					uint8_t ch;

					ch = uartReadChar( cli_uart );
					cliHandleNewChar( pcli, ch );
				}
		 	}
		 	if ( readyFlags & (1<<printTimerFlag) )
		 	{
				if (output_configuration[0].debug & 1 )
				{
					extern float getRollPIDOutput( void );
					extern float getPitchPIDOutput( void );

			 		printf("\nthrottle: %d", (int)getThrottleSetpoint() );
			 		printf("\nroll: %d:%d PID: %d", (int)getRollAngleSetpoint(), (int)RollAngFiltered, (int)getRollPIDOutput() );
			 		printf("\npitch: %d:%d PID: %d", (int)getPitchAngleSetpoint(), (int)PitchAngFiltered, (int)getPitchPIDOutput()  );
				}
				if (output_configuration[0].debug & 2 )
				{
					int motor;

					printf("\r\n");
					for (motor = 0; motor < 4; motor++ )
						printf("  m-%d: %u", motor+1, (unsigned int)getMotorValue( motor ) );
				}
				if (output_configuration[0].debug & 8 )
				{
					cliPrintf( pcli, "\ndT: %d %g micros %d exe %d roll %g pitch %g", IMUDelta, &fIMUDelta, micros(), IMUExeTime, &RollAngFiltered, &PitchAngFiltered);
				}
		 	}
		}
	}
}

void _Default_Handler( void )
{
	STM_EVAL_LEDOn(LED8);
	while( 1 );
}

int main(void)
{
	CoInitOS();

	initMicrosecondClock();

	cli_uart = uartOpen( UART_2, 115200, uart_mode_rx | uart_mode_tx, newUartData );
	if ( cli_uart != NULL )
		pcli = initCli( uartPutChar );
	initialiseCodeGroups();
	loadSavedConfiguration();

	CoCreateTask(cli_task, Co_NULL, 1, &cli_task_stack[CLI_TASK_STACK_SIZE-1], CLI_TASK_STACK_SIZE);
	CoCreateTask(main_task, Co_NULL, 0, &main_task_stack[MAIN_TASK_STACK_SIZE-1], MAIN_TASK_STACK_SIZE);

	CoStartOS();

	while (1);
}

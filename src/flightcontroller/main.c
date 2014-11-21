#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <coos.h>
#include <stm32f3_discovery.h>
#include <serial.h>
#include <serial_msp.h>
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

serial_port_st *cli_uart[2];
void *pcli[2];
OS_FlagID cliUartFlag;

void *i2c_port;
void *spi_port;
void *lsm303dlhcDevice;
void *l3gd20Device;
static sensorCallback_st sensorCallbacks;

float RollAngFiltered, PitchAngFiltered, Heading;

int uartPutChar( void * port, int ch )
{
	serial_port_st * serialPort = port;
	int result = serialPort->methods->writeCharBlockingWithTimeout( serialPort->serialCtx, ch, 10 );

	return result;
}

static float calculateHeading(float *magValues, float roll, float pitch)
{
    float headX;
    float headY;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
	float heading;
	static float filteredHeading;

    cos_roll = cosf((roll*M_PI)/180.0f);
    sin_roll = sinf((roll*M_PI)/180.0f);
    cos_pitch = cosf((pitch*M_PI)/180.0f);
    sin_pitch = sinf((pitch*M_PI)/180.0f);

    // Tilt compensated magnetic field X component:
    headX = magValues[1] * sin_roll * sin_pitch + magValues[0] * cos_pitch + magValues[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y component:
    headY = magValues[1] * cos_roll - magValues[2] * sin_roll;
    // magnetic heading
    heading = atan2f(headY,headX) * 180.0f/M_PI;

	// TODO: apply declination
	/* is heading instability due to loss of resolution when converting mag values to floats? */
	/* temp debug */
	// TODO: lpf of r heading value.
	filteredHeading = filteredHeading * 0.95 + heading * 0.05;

	heading = filteredHeading;
	if ( heading < 0.0f)
		heading += 360.0f;

	return heading;
}

OS_FlagID printTimerFlag;
OS_FlagID receiverFlag;
OS_FlagID IMUTimerFlag;

static void printTimer( void )
{
	isr_SetFlag( printTimerFlag );
}

static void newReceiverDataCallback( void )
{
	isr_SetFlag( receiverFlag );
}

uint32_t IMUDelta;
static uint32_t IMUExeTime;
static float    fIMUDelta;
static uint32_t lastIMUTime;

static void IMUCallback( void )
{
	CoEnterISR();

	isr_SetFlag( IMUTimerFlag );

	CoExitISR();
}

IMU_DATA_ST imu_data;

static void initIMU( void )
{
    init_attitude_estimation( &imu_data, roll_configuration[0].lpf_factor, roll_configuration[0].lpf_factor );

	IMUTimerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	/* start a three millisecond timer */
	initHiResTimer( 3000, IMUCallback );
	/*
		we get an interrupt almost immediately after we start the timer.
		Pretend we've processed the loop one cycle ago.
	*/
	lastIMUTime = micros()-3000;
}

float accelerometerValues[3];
float magnetometerValues[3];
float gyroValues[3];

static void estimateAttitude( float dT )
{

    do_attitude_estimation( &imu_data,
    				dT,
    				gyroValues[1],
    				-gyroValues[0],
    				accelerometerValues[0],
    				accelerometerValues[1],
    				accelerometerValues[2] );

    RollAngFiltered = imu_data.compAngleX;
    PitchAngFiltered = imu_data.compAngleY;
	/* we have pitch and roll, determine heading */
	Heading = calculateHeading( magnetometerValues, -RollAngFiltered, -PitchAngFiltered );
}

static void IMUHandler( void )
{
	uint32_t temp;
	uint32_t now = micros();
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
		if (err == E_OK)
		{
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
}

void newUartData( void *pv )
{
	UNUSED(pv);
	CoEnterISR();

	isr_SetFlag(cliUartFlag);

	CoExitISR();
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
		{
			{
				StatusType err;
				U32 readyFlags;

				readyFlags = CoWaitForMultipleFlags( (1<<printTimerFlag)|(1<<cliUartFlag), OPT_WAIT_ANY, 0, &err );
			 	if ( readyFlags & (1<<cliUartFlag) )
			 	{
					unsigned int uart_index;
					for (uart_index = 0; uart_index < ARRAY_SIZE(cli_uart) && cli_uart[uart_index] != NULL; uart_index++ )
					{
						while ( cli_uart[uart_index]->methods->rxReady( cli_uart[uart_index]->serialCtx ) )
						{
							uint8_t ch;

							ch = cli_uart[uart_index]->methods->readChar( cli_uart[uart_index]->serialCtx );
							if ( uart_index == 0)
								cliHandleNewChar( pcli[uart_index], ch );
							else
								mspProcess( cli_uart[uart_index], ch );
						}
					}
			 	}
			 	if ( readyFlags & (1<<printTimerFlag) )
			 	{
					if (output_configuration[0].debug & 1 )
					{
						extern float getRollPIDOutput( void );
						extern float getPitchPIDOutput( void );

				 		printf("\nthrottle: %d", (int)getThrottleSetpoint() );
				 		printf("\nroll: %d:%g PID: %d", (int)getRollAngleSetpoint(), &RollAngFiltered, (int)getRollPIDOutput() );
				 		printf("\npitch: %d:%g PID: %d", (int)getPitchAngleSetpoint(), &PitchAngFiltered, (int)getPitchPIDOutput() );
				 		printf("\nheading: %g", &Heading );
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
						cliPrintf( pcli[0], "\ndT: %d ar %g ap %g gr %g gp %g fr %g fp %g kr %g kp %g", &fIMUDelta,
							&imu_data.roll, &imu_data.pitch,
							&imu_data.gyroXangle, &imu_data.gyroYangle,
							&RollAngFiltered, &PitchAngFiltered,
							&imu_data.kalAngleX, &imu_data.kalAngleY
							);
					}
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
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);

	CoInitOS();

	initMicrosecondClock();

	cli_uart[0] = serialOpen( SERIAL_UART_2, 115200, uart_mode_rx | uart_mode_tx, newUartData );
	if ( cli_uart[0] != NULL )
		pcli[0] = initCli( cli_uart[0] );
	cli_uart[1] = serialOpen( SERIAL_USB, 115200, uart_mode_rx | uart_mode_tx, newUartData );
	if ( cli_uart[1] != NULL )
		pcli[1] = initCli( cli_uart[1] );

	initialiseCodeGroups();
	loadSavedConfiguration();

	CoCreateTask(cli_task, Co_NULL, 1, &cli_task_stack[CLI_TASK_STACK_SIZE-1], CLI_TASK_STACK_SIZE);
	CoCreateTask(main_task, Co_NULL, 0, &main_task_stack[MAIN_TASK_STACK_SIZE-1], MAIN_TASK_STACK_SIZE);

	CoStartOS();

	while (1);
}

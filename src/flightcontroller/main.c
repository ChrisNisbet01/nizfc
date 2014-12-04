#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <coocox.h>
#include <serial.h>
#include <utils.h>
#include <polling.h>
#include <i2c.h>
#include <spi.h>
#include <receiver.h>
#include <board_configuration.h>
#include <cli.h>
#include <startup.h>
#include <sensors.h>
#if defined(STM32F30X)
#include <lsm303dlhc.h>
#include <l3gd20.h>
#elif defined(STM32F10X)
#include <stm32f10x_rcc.h>
// TODO:
#endif
#include <angle_mode_configuration.h>
#include <rate_mode_configuration.h>
#include <yaw_configuration.h>
#include <receiver_handler.h>
#include <motor_control.h>
#include <hirestimer.h>
#include <attitude_estimation.h>
#include <attitude_configuration.h>
#include <kalman.h>
#include <board_alignment.h>
#include <failsafe.h>
#include <aux_configuration.h>
#include <filter.h>
#include <sensor_filters.h>
#include <leds.h>

#define MAIN_TASK_STACK_SIZE 0x200
#define CLI_TASK_STACK_SIZE 0x200

static OS_STK cli_task_stack[CLI_TASK_STACK_SIZE];
static OS_STK main_task_stack[MAIN_TASK_STACK_SIZE];

static serial_port_st *cli_uart[2];
serial_port_st *debug_port;
static void *pcli[2];
static OS_FlagID cliUartFlag;

static void *i2c_port;
#if defined(STM32F30X)
static void *spi_port;
#endif
static sensorCallback_st sensorCallbacks;

float RollAngle, PitchAngle, Heading;

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

    cos_roll = cosf((roll * M_PI)/180.0f);
    sin_roll = sinf((roll * M_PI)/180.0f);
    cos_pitch = cosf((pitch * M_PI)/180.0f);
    sin_pitch = sinf((pitch * M_PI)/180.0f);

    // Tilt compensated magnetic field X component:
    headX = magValues[1] * sin_roll * sin_pitch + magValues[0] * cos_pitch + magValues[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y component:
    headY = magValues[1] * cos_roll - magValues[2] * sin_roll;

    // magnetic heading
    heading = atan2f(-headY,-headX) * 180.0f/M_PI;

	// TODO: apply declination
	filteredHeading = filterValue( filteredHeading, heading, attitude_configuration[0].heading_lpf );

	heading = filteredHeading;
	if ( heading < 0.0f)
		heading += 360.0f;

	return heading;
}

OS_FlagID printTimerFlag;
OS_FlagID receiverFlag;
OS_FlagID IMUTimerFlag;
OS_FlagID failsafeTriggerFlag;

static void printTimer( void )
{
	isr_SetFlag( printTimerFlag );
}

static volatile uint32_t latestChannelsReceived;

static void failsafeTriggerCallback( void )
{
	isr_SetFlag( failsafeTriggerFlag );
}

static void newReceiverDataCallback( uint32_t newChannelsReceived )
{
	latestChannelsReceived |= newChannelsReceived;
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

float accelerometerValues[3];
float gyroValues[3];
float magnetometerValues[3];
float filteredAccelerometerValues[3];
float filteredGyroValues[3];
float filteredMagnetometerValues[3];

bool setDebugPort( int port )
{
	bool debugPortAssigned = true;

	if ( port < 0 )
		debug_port = NULL;
	else if ( (unsigned)port < ARRAY_SIZE(cli_uart) && cli_uart[port] != NULL )
		debug_port = cli_uart[port];
	else
		debugPortAssigned = false;

	return debugPortAssigned;
}

static void initIMU( uint32_t updatePeriod )
{
	IMUTimerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	/* start a three millisecond timer */
	initHiResTimer( updatePeriod, IMUCallback );

	/*
		we get an interrupt almost immediately after we start the timer.
		Pretend we've processed the loop one cycle ago.
	*/
	lastIMUTime = micros() - updatePeriod;
}


static void estimateAttitude( float dT )
{

    do_attitude_estimation( &imu_data,
    				dT,
    				filteredGyroValues[1],
    				-filteredGyroValues[0],
    				filteredAccelerometerValues[0],
    				filteredAccelerometerValues[1],
    				filteredAccelerometerValues[2] );

    RollAngle = imu_data.compRollAngle;
    PitchAngle = imu_data.compPitchAngle;
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

	if ( sensorCallbacks.readAccelerometer != NULL
		&& sensorCallbacks.readGyro != NULL
		&& sensorCallbacks.readAccelerometer( sensorCallbacks.accelerometerCtx, accelerometerValues ) == true
		&& sensorCallbacks.readGyro( sensorCallbacks.gyroCtx, gyroValues ) == true)
	{
		alignVectorsToFlightController( accelerometerValues, noRotation );	// TODO: configurable
		alignVectorsToCraft( accelerometerValues );
		filterAccValues( accelerometerValues, filteredAccelerometerValues );

		alignVectorsToFlightController( gyroValues, noRotation );			// TODO: configurable
		alignVectorsToCraft( gyroValues );
		filterGyroValues( gyroValues, filteredGyroValues );

		alignVectorsToFlightController( magnetometerValues, noRotation );	// TODO: configurable
		alignVectorsToCraft( magnetometerValues );
		filterMagValues( magnetometerValues, filteredMagnetometerValues );

		estimateAttitude( fIMUDelta );

		if ( sensorCallbacks.readMagnetometer != NULL
			&& sensorCallbacks.readMagnetometer( sensorCallbacks.magnetometerCtx, magnetometerValues ) == true )
		{
			alignVectorsToFlightController( magnetometerValues, noRotation );	// TODO: configurable
			alignVectorsToCraft( magnetometerValues );
			filterMagValues( magnetometerValues, filteredMagnetometerValues );

			/* we have pitch and roll, determine heading */
			Heading = calculateHeading( filteredMagnetometerValues, -RollAngle, -PitchAngle );

		}

		updatePIDControlLoops();
	}


	IMUExeTime = (now=micros()) - lastIMUTime;

}

static void main_task( void *pv )
{
	UNUSED(pv);

	setLED( LED1, led_state_on );
	setLED( LED2, led_state_off );
    while ( 1 )
    {
    	int i;


        for (i = 0; i < 10; i++) {
	    	setLED( LED1, led_state_toggle);
	    	setLED( LED2, led_state_toggle );
	    	CoTimeDelay( 0, 0, 0, 100 );
        }
    }
	i2c_port = i2cInit( I2C_PORT_1 );

	if ( i2c_port != NULL )
	{
		sensorConfig_st sensorConfig;

		sensorConfig.i2cCtx = i2c_port;
#if defined(STM32F30X)
		initLSM303DLHC( &sensorConfig, &sensorCallbacks );
#elif defined(STM32F10X)
		// TODO:
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

	receiverFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	failsafeTriggerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );

	initBoardAlignment(
		board_configuration[0].boardOrientation[0],
		board_configuration[0].boardOrientation[1],
		board_configuration[0].boardOrientation[2] );
	initFailsafe( failsafeTriggerCallback );
	initMotorControl(board_configuration[0].craftType);
	openReceiver( newReceiverDataCallback );

	initIMU(board_configuration[0].updateTime);

	while (1)
	{
		StatusType err;
		U32 ReadyFlags;

		ReadyFlags = CoWaitForMultipleFlags( (1<<IMUTimerFlag) | (1<<receiverFlag) | (1<<failsafeTriggerFlag), OPT_WAIT_ANY, 0, &err );
		if (err == E_OK)
		{
			if ( (ReadyFlags & (1<<failsafeTriggerFlag)) != 0 )
			{
				printf("\r\nfailsafe triggered");
				failsafeSetTriggered();
			}
			if ( (ReadyFlags & (1<<IMUTimerFlag)) != 0 )
			{
				IMUHandler();
				updateMotorOutputs();
			}
			if ( (ReadyFlags & (1<<receiverFlag)) != 0 )
			{
				uint32_t newChannels;

				IRQ_DISABLE_SAVE();
				newChannels = latestChannelsReceived;
				latestChannelsReceived = 0;
				IRQ_ENABLE_RESTORE();

			    updateFailsafeWithNewChannels( newChannels );

			    setLED(RX_LED, led_state_toggle);
				processReceiverSignals();
				updateFunctionEnables();
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

static void handleNewSerialData( void )
{
	unsigned int uart_index;
	for (uart_index = 0; uart_index < ARRAY_SIZE(cli_uart); uart_index++ )
	{
		if ( cli_uart[uart_index] != NULL )
		{
			while ( cli_uart[uart_index]->methods->rxReady( cli_uart[uart_index]->serialCtx ) )
			{
				uint8_t ch;

				ch = cli_uart[uart_index]->methods->readChar( cli_uart[uart_index]->serialCtx );
				cliHandleNewChar( pcli[uart_index], ch );
			}
		}
	}
}

static void doDebugOutput( void )
{
	if (board_configuration[0].debug & 1 )
	{
		extern float getRollAngleOutput( void );
		extern float getPitchAnglePIDOutput( void );
		extern float getRollRatePIDOutput( void );
		extern float getPitchRatePIDOutput( void );

 		//printf("\n\nthrottle: %d", (int)getThrottleSetpoint() );
 		//printf("\nroll: %d:%g PID: %d", (int)getRollAngleSetpoint(), &RollAngle, (int)getRollAnglePIDOutput() );
 		//printf("\npitch: %d:%g PID: %d", (int)getPitchAngleSetpoint(), &PitchAngle, (int)getPitchAnglePIDOutput() );
 		printf("\nroll rate: %d:%g PID: %d", (int)getRollRateSetpoint(), &filteredGyroValues[0], (int)getRollRatePIDOutput() );
 		printf("\npitch rate: %d:%g PID: %d", (int)getPitchRateSetpoint(), &filteredGyroValues[1], (int)getPitchRatePIDOutput() );
 		//printf("\nheading: %g", &Heading );
	}
	if (board_configuration[0].debug & 2 )
	{
		int motor;

		printf("\r\n");
		for (motor = 0; motor < 4; motor++ )
			printf("  m-%d: %u", motor+1, (unsigned int)getMotorValue( motor ) );
	}
	if (board_configuration[0].debug & 8 )
	{
		printf( "\r\n\ndT: %g", &fIMUDelta );
		printf( "\r\naccel" );
		printf( "\r\n        roll %g", &imu_data.roll );
		printf( "\r\n        pit  %g", &imu_data.pitch );
		printf( "\r\ngyro" );
		printf( "\r\n        roll %g", &imu_data.gyroRollAngle );
		printf( "\r\n        pit  %g", &imu_data.gyroPitchAngle );
		printf( "\r\nfiltered" );
		printf( "\r\n        roll %g", &RollAngle );
		printf( "\r\n        pit  %g", &PitchAngle );
	}
	if (board_configuration[0].debug & 16 )
	{
		int channel;

		printf("\r\n");
		for (channel = 4; channel < 8; channel++ )
			printf("  aux-%d: %u", channel+1, (unsigned int)readReceiverChannel(channel));
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
		StatusType err;
		U32 readyFlags;

		readyFlags = CoWaitForMultipleFlags( (1<<printTimerFlag)|(1<<cliUartFlag), OPT_WAIT_ANY, 0, &err );
	 	if ( readyFlags & (1<<cliUartFlag) )
	 		handleNewSerialData();

	 	if ( readyFlags & (1<<printTimerFlag) )
	 		doDebugOutput();
	}
}

void _Default_Handler( void )
{
	setLED(EXCEPTION_LED, led_state_on);
	while( 1 );
}

static void systemInit(void)
{
    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Turn off JTAG port because we're using the GPIO for leds
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;

}
int main(void)
{
	systemInit();
	initLEDs();
	initMicrosecondClock();

	CoInitOS();

	/* open the serial ports early so debug info can be sent to them */
	cli_uart[0] = serialOpen( SERIAL_UART_2, 115200, uart_mode_rx | uart_mode_tx, newUartData );
	if ( cli_uart[0] != NULL )
		pcli[0] = initCli( cli_uart[0] );
#if defined(STM32F30X)
 	cli_uart[1] = serialOpen( SERIAL_USB, 115200, uart_mode_rx | uart_mode_tx, newUartData );
#elif defined(STM32F10X)
 	cli_uart[1] = serialOpen( SERIAL_UART_1, 115200, uart_mode_rx | uart_mode_tx, newUartData );
#endif
	if ( cli_uart[1] != NULL )
		pcli[1] = initCli( cli_uart[1] );
	initialiseCodeGroups();
	loadSavedConfiguration();

	CoCreateTask(cli_task, Co_NULL, 1, &cli_task_stack[CLI_TASK_STACK_SIZE-1], CLI_TASK_STACK_SIZE);
	CoCreateTask(main_task, Co_NULL, 0, &main_task_stack[MAIN_TASK_STACK_SIZE-1], MAIN_TASK_STACK_SIZE);

	CoStartOS();

	while (1);
}


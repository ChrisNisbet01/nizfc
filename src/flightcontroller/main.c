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
#include <receiver.h>
#include <outputs.h>
#include <output_configuration.h>
#include <cli.h>
#include <startup.h>
#include <sensors.h>
#include <lsm303dlhc.h>


#define MAIN_TASK_STACK_SIZE 0x200
#define CLI_TASK_STACK_SIZE 0x200

static OS_STK cli_task_stack[CLI_TASK_STACK_SIZE];
static OS_STK main_task_stack[MAIN_TASK_STACK_SIZE];

void *cli_uart;
OS_FlagID cliUartFlag;

void *i2c_port;
void *pcli;
void *lsm303dlhcDevice;
static sensorCallback_st sensorCallbacks;
float RollAng, PitchAng, Heading;

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
	if ( i2c_port != NULL )
	{
		sensorConfig_st sensorConfig;

		sensorConfig.i2cCtx = i2c_port;
		lsm303dlhcDevice = initLSM303DLHC( &sensorConfig, &sensorCallbacks );
	}
	openReceiver();
	openOutputs();

	while (1)
	{
		uint_fast16_t rx_value;

		CoTimeDelay(0, 0, 0, 500);
        STM_EVAL_LEDToggle(LED3);
        float accelerometerValues[3];
        float magnetometerValues[3];

		rx_value = readReceiverChannel(0);
		setMotorOutput( 0, rx_value );
		setMotorOutput( 1, rx_value );
		setMotorOutput( 2, rx_value );
		setMotorOutput( 3, rx_value );

		if (output_configuration[0].debug & 1 )
			printf("\npwm1: %d %d %d %d", readReceiverChannel(0), readReceiverChannel(1), readReceiverChannel(2), readReceiverChannel(3) );
		if ( lsm303dlhcDevice )
		{
			if ( sensorCallbacks.readAccelerometer != NULL
				&& sensorCallbacks.readMagnetometer != NULL
				&& sensorCallbacks.readAccelerometer( lsm303dlhcDevice, accelerometerValues ) == true
				&& sensorCallbacks.readMagnetometer( lsm303dlhcDevice, magnetometerValues ) == true)
			{
				Heading = calculateHeading( magnetometerValues, accelerometerValues );
				printf("\n%spitch %f roll %f heading %f", "", PitchAng, RollAng, Heading );
			}
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
	UNUSED(pv);

	cliUartFlag = CoCreateFlag( Co_TRUE, 0 );

	cli_uart = uartOpen( UART_2, 115200, uart_mode_rx | uart_mode_tx, newUartData );
	if ( cli_uart != NULL )
		pcli = initCli( uartPutChar );

	while (1)
	{
		if ( cli_uart != NULL )
		{
			CoWaitForSingleFlag( cliUartFlag, 0 );
			while ( uartRxReady( cli_uart ) )
			{
				uint8_t ch;

				ch = uartReadChar( cli_uart );
				cliHandleNewChar( pcli, ch );
			}
		}
	}
}

int main(void)
{
	CoInitOS();

	initialiseCodeGroups();
	loadSavedConfiguration();

	CoCreateTask(cli_task, Co_NULL, 1, &cli_task_stack[CLI_TASK_STACK_SIZE-1], CLI_TASK_STACK_SIZE);
	CoCreateTask(main_task, Co_NULL, 0, &main_task_stack[MAIN_TASK_STACK_SIZE-1], MAIN_TASK_STACK_SIZE);

	CoStartOS();

	while (1);
}

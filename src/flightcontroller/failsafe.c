#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <coos.h>
#include <failsafe_configuration.h>
#include <leds.h>

typedef struct failsafe_st
{
	bool enabled;
	uint32_t currentChannels;
	OS_TCID	timerID;
	void (*failsafeTriggerCb)( void );
	volatile bool triggered;
} failsafe_st;

static failsafe_st failsafe;

static void startFailsafeTimer( void )
{
	CoSetTmrCnt( failsafe.timerID, MSTOTICKS(failsafe_configuration[0].maxQuietTimeMs), 0 );
	CoStartTmr( failsafe.timerID );

}

static void stopFailsafeTimer( void )
{
	CoStopTmr( failsafe.timerID );
}

static void restartFailsafeTimer( void )
{
	CoSetTmrCnt( failsafe.timerID, MSTOTICKS(failsafe_configuration[0].maxQuietTimeMs), 0 );
}


void failsafeTimeout( void )
{
	/* called from COOS systick ISR */
	setLED(FAILSAFE_LED, led_state_on);
	if ( failsafe.failsafeTriggerCb != NULL )
		failsafe.failsafeTriggerCb();
}

void initFailsafe( void (*failsafeTriggerCb)( void ) )
{
	failsafe.failsafeTriggerCb = failsafeTriggerCb;
	failsafe.timerID = CoCreateTmr( TMR_TYPE_ONE_SHOT, 0, 0, failsafeTimeout );
}

void enableFailsafe( void )
{
	/* called just after the board is armed */
	if ( failsafe_configuration[0].enabled == true && failsafe.enabled == false )
	{
		failsafe.enabled = true;
		/* start the timer which will activate failsafe if it ever expires */
		startFailsafeTimer();
	}
}

void disableFailsafe( void )
{
	/* called when the board is disarmed */
	if ( failsafe.enabled == true )
	{
		failsafe.enabled = false;
		stopFailsafeTimer();
	}
}

void updateFailsafeWithNewChannels( uint32_t newChannels )
{
	/* called whenever new channel data is received. newChannels is a bitmask representing the new channel data */
	if ( failsafe.enabled == true )
	{
		uint32_t requiredChannels;

		failsafe.currentChannels |= newChannels;

		requiredChannels = (1 << failsafe_configuration[0].nbRequiredChannels) - 1;	/* works up to 31 channels */

		if ( (failsafe.currentChannels & requiredChannels) == requiredChannels )
		{
			/* got all the required channels */
			failsafe.currentChannels = 0;
			restartFailsafeTimer();
		}
	}
}

void failsafeSetTriggered( void )
{
	failsafe.triggered = true;
}

bool hasFailsafeTriggered( void )
{
	return failsafe.triggered;
}

uint_fast16_t getFailsafeMotorSpeed( void )
{
	return failsafe_configuration[0].motorOutput;
}

void resetFailsafeTrigger( void )
{
	failsafe.triggered = false;
	setLED(FAILSAFE_LED, led_state_off);
}

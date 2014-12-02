#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <coos.h>
#include <utils.h>
#include <pwm_tx.h>
#include <pwm_outputs.h>
#include <outputs.h>
#include <output_configuration.h>

static const pwm_output_id_t pwm_tx_pinIDs[] =
{
	pwm_output_1,
	pwm_output_2,
	pwm_output_3,
	pwm_output_4,
	pwm_output_5,
	pwm_output_6
};

typedef struct pwmOutput_st
{
	bool used;
	void *pwmTimerCtx;
} pwmOutput_st;

static pwmOutput_st pwmOutputs[ARRAY_SIZE(pwm_tx_pinIDs)];

void setMotorOutput( unsigned int motor, unsigned int pulseWidthMillsecs )
{
	if (motor < ARRAY_SIZE(pwm_tx_pinIDs) && pwmOutputs[motor].used == true)
	{
		setPwmTxPulseWidth( pwmOutputs[motor].pwmTimerCtx, pulseWidthMillsecs );
	}
}

void openOutputs( unsigned int nbMotors )
{
	unsigned int motorIndex;

	for (motorIndex = 0; motorIndex < nbMotors && motorIndex < ARRAY_SIZE(pwm_tx_pinIDs); motorIndex++ )
	{
		pwmOutputs[motorIndex].used = true;
		pwmOutputs[motorIndex].pwmTimerCtx = openPwmTxTimer( pwm_tx_pinIDs[motorIndex], output_configuration[0].pwmrate, 0 );
	}

}


#ifndef __PWM_TX_STM32F30X_H__
#define __PWM_TX_STM32F30X_H__

#include <pwm_outputs.h>

void * openPwmTxTimer( pwm_output_id_t pinID, unsigned int pulseRateHz, unsigned int initialValue );
void setPwmTxPulseWidth( void const * const pv, unsigned int pulseWidthMillsecs );

#endif /* __PWM_TX_STM32F30X_H__ */


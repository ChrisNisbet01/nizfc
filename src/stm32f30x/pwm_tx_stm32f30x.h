#ifndef __PWM_TX_STM32F30X_H__
#define __PWM_TX_STM32F30X_H__

void * openPwmTxTimer( pin_st const * const pin, unsigned int pulseRateHz, unsigned int initialValue );
void setPwmTxPulseWidth( void const * const pv, unsigned int pulseWidthMillsecs );

#endif /* __PWM_TX_STM32F30X_H__ */


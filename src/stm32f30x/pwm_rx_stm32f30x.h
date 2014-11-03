#ifndef __PWM_RX_STM32F30X_H__
#define __PWM_RX_STM32F30X_H__

#define	MAX_RX_SIGNALS	12

void *openPPMInput( void );
void initPWMRx( void );
int readRXSignals(void *pctx, uint_fast16_t signals[MAX_RX_SIGNALS]);

#endif

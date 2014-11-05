#ifndef __PWM_RX_STM32F30X_H__
#define __PWM_RX_STM32F30X_H__

#define	MAX_RX_SIGNALS	12u

void openPPMInput( void );
void openPWMInput( uint_fast8_t nb_rx_channels );
void initPWMRx( void );
int readRXSignals(uint_fast16_t signals[MAX_RX_SIGNALS]);

#endif

#ifndef __PWM_RX_STM32F30X_H__
#define __PWM_RX_STM32F30X_H__

#include <pins.h>


typedef enum pwm_mode_t
{
	pwm_mode,
	ppm_mode
} pwm_mode_t;

void openPwmTimer( pin_st const * const pin, pwm_mode_t mode, void (*cb)( void *pv, uint32_t const pulse_width_us ), void * pv );

#endif

#ifndef __PWM_H__
#define __PWM_H__

#include <pwm_inputs.h>

void openPwmTimer( pwm_input_id_t pinID, pwm_mode_t mode, void (*cb)( void *pv, uint32_t const pulse_width_us ), void * pv );

#endif /* __PWM_H__ */

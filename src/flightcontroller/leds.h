#ifndef __LEDS_H__
#define __LEDS_H__

typedef enum led_t
{
	LED1,
	LED2,
	LED3,
	LED4,
	LED5,
	LED6,
	LED7,
	LED8,
	ARMED_LED =		    LED1,
	ANGLE_MODE_LED =    LED2,
	FAILSAFE_LED =      LED3,
	EXCEPTION_LED =     LED4,
	RX_LED =			LED5
} led_t;

typedef enum led_state_t
{
	led_state_off,
	led_state_on,
	led_state_toggle,
	led_state_slow_flash,
	led_state_fast_flash,
	led_state_fast_slow,	/* short ON, long OFF */
	led_state_slow_fast		/* long ON, short OFF */
} led_state_t;

void setLED( led_t led, led_state_t state );
void initLEDs( void );


#endif /* __LEDS_H__ */

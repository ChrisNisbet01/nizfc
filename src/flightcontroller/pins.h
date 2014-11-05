#ifndef __NIZFC_PIN_H__
#define __NIZFC_PIN_H__

#include <stdint.h>
#include <stm32f30x_gpio.h>

typedef struct pin_st
{
	uint_fast16_t	pin;
	GPIO_TypeDef	*gpio;
} pin_st;


#endif /* __NIZFC_PIN_H__ */

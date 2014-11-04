#include <stdlib.h>
#include <stdint.h>

#include <stm32f30x_usart.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>

#include "usart_stm32f30x.h"

typedef enum usart_idx_t {
	USART2_IDX,
	MAX_USARTS
} usart_idx_t;


typedef struct usart_port_config_st
{
	USART_TypeDef		*usart;

	uint_fast16_t		txPin;
	uint_fast8_t		txPinSource;
	uint_fast8_t		txPinAF;

	uint_fast16_t		rxPin;
	uint_fast8_t		rxPinSource;
	uint_fast8_t		rxPinAF;

	GPIO_TypeDef		*gpioPort;	/* Assumes same port for both pins */

	void 				(*RCC_APBPeriphClockCmd)(uint32_t RCC_APBPeriph, FunctionalState NewState);
	uint_fast32_t		RCC_APBPeriph;
	uint_fast32_t		RCC_AHBPeriph;
	uint_fast16_t		irq;

}usart_port_config_st;

static const usart_port_config_st usart_configs[] =
{
	[USART2_IDX] =
		{
			.usart = USART2,
			.txPin = GPIO_Pin_5,
			.txPinSource = GPIO_PinSource5,
			.txPinAF = GPIO_AF_7,
			.rxPin = GPIO_Pin_6,
			.rxPinSource = GPIO_PinSource6,
			.rxPinAF = GPIO_AF_7,
			.gpioPort = GPIOD,
			.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
			.RCC_APBPeriph = RCC_APB1Periph_USART2,
			.RCC_AHBPeriph = RCC_AHBPeriph_GPIOD,
			.irq = USART2_IRQn
		}
};
#define NB_UART_PORTS	(sizeof(usart_configs)/sizeof(usart_configs[0]))
#define GET_USART_IDX(ptr)	((ptr)-usart_configs)

static usart_cb_st usartCallbackInfo[NB_UART_PORTS];

static usart_port_config_st const * usartConfigLookup( USART_TypeDef *usart )
{
	int i;

	for ( i=0; i < MAX_USARTS; i++ )
	{
		if ( usart_configs[i].usart == usart )
		{
			return &usart_configs[i];
		}
	}

	return NULL;
}

void const * stm32f30x_usart_init(usart_init_st *cfg)
{
	usart_port_config_st const * uart_config;

	if ( (uart_config=usartConfigLookup(cfg->usart)) == NULL )
		goto done;

	usartCallbackInfo[GET_USART_IDX(uart_config)] = cfg->callback;

	/* enable appropriate clocks */
	RCC_AHBPeriphClockCmd(uart_config->RCC_AHBPeriph, ENABLE);
	uart_config->RCC_APBPeriphClockCmd(uart_config->RCC_APBPeriph, ENABLE);

	/* configure appropriate GPIO pins */
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    if (cfg->mode & usart_mode_tx)
    {
        GPIO_InitStructure.GPIO_Pin = uart_config->txPin;
        /* connect pin to USART */
        GPIO_PinAFConfig(uart_config->gpioPort, uart_config->txPinSource, uart_config->txPinAF);
        GPIO_Init(uart_config->gpioPort, &GPIO_InitStructure);
    }

    if (cfg->mode & usart_mode_rx)
    {
        GPIO_InitStructure.GPIO_Pin = uart_config->rxPin;
        /* connect pin to USART */
        GPIO_PinAFConfig(uart_config->gpioPort, uart_config->rxPinSource, uart_config->rxPinAF);
        GPIO_Init(uart_config->gpioPort, &GPIO_InitStructure);
    }

	NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = uart_config->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	// TODO: configurable
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			// TODO: configurable
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

done:
    return uart_config;
}

static void usartIrqHandler(usart_port_config_st const * const uart_config, usart_cb_st *runtime)
{
    uint32_t ISR = uart_config->usart->ISR;

    if ((ISR & USART_FLAG_RXNE))
    {
    	if (runtime->putRxChar != NULL)
    	{
    		runtime->putRxChar( runtime->pv, uart_config->usart->RDR );
    	}
    }

    if ((ISR & USART_FLAG_TXE))
    {
    	int ch;

    	if (runtime->getTxChar != NULL && (ch=runtime->getTxChar( runtime->pv )) >= 0)
    	{
            USART_SendData(uart_config->usart, ch);
    	}
        else
        {
            USART_ITConfig(uart_config->usart, USART_IT_TXE, DISABLE);
        }
    }

    if (ISR & USART_FLAG_ORE)
    {
        USART_ClearITPendingBit (uart_config->usart, USART_IT_ORE);
        // TODO: statistic?
    }
}

void USART2_IRQHandler(void)
{
    usartIrqHandler(&usart_configs[USART2_IDX], &usartCallbackInfo[USART2_IDX]);
}


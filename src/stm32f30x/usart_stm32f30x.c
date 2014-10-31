#include <stdlib.h>
#include <stdint.h>

#include <stm32f30x_usart.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include "usart_stm32f30x.h"

#define USART2_RX_BUFFER_SIZE	128
#define USART2_TX_BUFFER_SIZE	128

static volatile uint8_t Usart2TxBuffer[USART2_TX_BUFFER_SIZE];
static volatile uint8_t Usart2RxBuffer[USART2_RX_BUFFER_SIZE];

typedef enum USART_ID {
	USART2_ID,
	MAX_USARTS
} USART_ID;


typedef struct usart_port_config_st
{
	USART_TypeDef		*usart;

	uint_fast8_t		idx;	/* indicates which entry we are in the usart_configs table */

	uint_fast8_t		txPin;
	uint_fast8_t		txPinSource;
	uint_fast8_t		txPinAF;

	uint_fast8_t		rxPin;
	uint_fast8_t		rxPinSource;
	uint_fast8_t		rxPinAF;

	GPIO_TypeDef		*gpioPort;	/* Assumes same port for both pins */

	volatile uint8_t	*txBuffer;
	unsigned int		txBufferSize;

	volatile uint8_t	*rxBuffer;
	unsigned int		rxBufferSize;

	void 				(*RCC_APBPeriphClockCmd)(uint32_t RCC_APBPeriph, FunctionalState NewState);
	unsigned int		RCC_APBPeriph;

}usart_port_config_st;

static const usart_port_config_st usart_configs[] =
{
	[USART2_ID] =
		{
			.usart = USART2,
			.idx = USART2_ID,
			.txPin = GPIO_Pin_5,
			.txPinSource = GPIO_PinSource5,
			.txPinAF = GPIO_AF_7,
			.rxPin = GPIO_Pin_6,
			.rxPinSource = GPIO_PinSource6,
			.rxPinAF = GPIO_AF_7,
			.gpioPort = GPIOD,
			.txBuffer = Usart2TxBuffer,
			.txBufferSize = sizeof Usart2TxBuffer,
			.rxBuffer = Usart2RxBuffer,
			.rxBufferSize = sizeof Usart2RxBuffer,
			.RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
			.RCC_APBPeriph = RCC_APB1Periph_USART2,

		}
};

static usart_cb_st usartCallbackInfo[MAX_USARTS];

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
    GPIO_InitTypeDef  GPIO_InitStructure;
	usart_port_config_st const * uart_config;

	if ( (uart_config=usartConfigLookup(cfg->usart)) == NULL )
		goto done;

	usartCallbackInfo[uart_config->idx] = cfg->callback;

	uart_config->RCC_APBPeriphClockCmd( uart_config->RCC_APBPeriph, ENABLE );

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    if (cfg->mode & usart_mode_tx)
    {
        GPIO_InitStructure.GPIO_Pin = uart_config->txPin;
        GPIO_PinAFConfig(uart_config->gpioPort, uart_config->txPinSource, uart_config->txPinAF);
        GPIO_Init(uart_config->gpioPort, &GPIO_InitStructure);
    }

    if (cfg->mode & usart_mode_rx)
    {
        GPIO_InitStructure.GPIO_Pin = uart_config->rxPin;
        GPIO_PinAFConfig(uart_config->gpioPort, uart_config->rxPinSource, uart_config->rxPinAF);
        GPIO_Init(uart_config->gpioPort, &GPIO_InitStructure);
    }

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
    }
}

void USART2_IRQHandler(void)
{
    usartIrqHandler(&usart_configs[USART2_ID], &usartCallbackInfo[USART2_ID]);
}


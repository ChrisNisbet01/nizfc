#include <stdlib.h>
#include <stdint.h>

#include <stm32f30x_usart.h>
#include <usart_stm32f30x.h>
#include <stm32f3_discovery.h>
#include "uart_interface.h"
#include "uart.h"

#define USART2_RX_BUFFER_SIZE	128
#define USART2_TX_BUFFER_SIZE	128

static volatile uint8_t Usart2TxBuffer[USART2_TX_BUFFER_SIZE];
static volatile uint8_t Usart2RxBuffer[USART2_RX_BUFFER_SIZE];

typedef struct uart_ports_config_t
{
	uart_ports_t port;
	USART_TypeDef *usart;
	volatile uint8_t 	*rxBuffer;
	uint_fast16_t		rxBufferSize;
	volatile uint8_t 	*txBuffer;
	uint_fast16_t		txBufferSize;

} uart_ports_config_t;

typedef struct uart_ctx_st
{
	uart_ports_config_t			*uart_config;
	uint_fast32_t				baudRate;
	uart_ports_config_t	const 	*port;
	uart_modes_t				mode;
	void const          		*ll_info;	/* returned by micro specific init function */
	volatile uint8_t 			*rxBuffer;
	uint_fast16_t				rxBufferSize;
	volatile uint8_t 			*txBuffer;
	uint_fast16_t				txBufferSize;
	uint_fast16_t				rxBufferHead;
	uint_fast16_t				rxBufferTail;
	uint_fast16_t				txBufferHead;
	uint_fast16_t				txBufferTail;
} uart_ctx_st;

static const uart_ports_config_t uart_ports[] =
{
	{
	.port = UART_2,
	.usart = USART2,
	.rxBuffer = Usart2RxBuffer,
	.rxBufferSize = sizeof 	Usart2RxBuffer,
	.txBuffer = Usart2TxBuffer,
	.txBufferSize = sizeof 	Usart2TxBuffer
	}
};
#define NB_UART_PORTS	(sizeof(uart_ports)/sizeof(uart_ports[0]))
#define UART_IDX(ptr)	((ptr)-uart_ports)

static uart_ctx_st uart_ctxs[NB_UART_PORTS];

static int getTxChar( void *pv )
{
	uart_ctx_st *pctx = (uart_ctx_st *)pv;
	int ch;

    if (pctx->txBufferTail != pctx->txBufferHead)
    {
    	ch = pctx->txBuffer[pctx->txBufferTail];
        pctx->txBufferTail = (pctx->txBufferTail + 1) % pctx->txBufferSize;
    }
    else
    	ch = -1;

    return ch;
}

static void putRxChar( void *pv, uint8_t ch )
{
	uart_ctx_st *pctx = (uart_ctx_st *)pv;

    pctx->rxBuffer[pctx->rxBufferHead] = ch;
    pctx->rxBufferHead = (pctx->rxBufferHead + 1) % pctx->rxBufferSize;
}


static void uartConfigure(uart_ctx_st *pctx)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = pctx->baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (pctx->mode & uart_mode_rx)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (pctx->mode & uart_mode_tx)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;

    USART_Init(pctx->port->usart, &USART_InitStructure);
}

static uart_ports_config_t const * uartPortLookup( uart_ports_t port )
{
	unsigned int i;

	for (i=0; i < NB_UART_PORTS; i++)
	{
		if ( uart_ports[i].port == port )
			return &uart_ports[i];
	}

	return NULL;
}

void *uartOpen( uart_ports_t port, uint32_t baudrate, uart_modes_t mode )
{
	uart_ports_config_t const * uart_config;
	uart_ctx_st *pctx;
	usart_init_st	cfg;

	if ( (uart_config=uartPortLookup( port )) == NULL )
	{
		pctx = NULL;
		goto done;
	}
	pctx = &uart_ctxs[UART_IDX(uart_config)];

	cfg.mode = 0;
	if (mode & uart_mode_rx)
		cfg.mode |= usart_mode_rx;
	if (mode & uart_mode_tx)
		cfg.mode |= usart_mode_tx;
	cfg.usart = uart_config->usart;
	cfg.callback.pv = pctx;
	cfg.callback.getTxChar = getTxChar;
	cfg.callback.putRxChar = putRxChar;

	if ((pctx->ll_info=stm32f30x_usart_init( &cfg )) == NULL)
	{
		pctx = NULL;
		goto done;
	}

	pctx->rxBuffer = uart_config->rxBuffer;
	pctx->rxBufferSize = uart_config->rxBufferSize;
	pctx->txBuffer = uart_config->txBuffer;
	pctx->txBufferSize = uart_config->txBufferSize;

    pctx->rxBufferHead = pctx->rxBufferTail = 0;
    pctx->txBufferHead = pctx->txBufferTail = 0;
    pctx->port = uart_config;
    pctx->mode = mode;
    pctx->baudRate = baudrate;

    uartConfigure(pctx);

    // Receive IRQ
    if (mode & uart_mode_rx) {
        USART_ClearITPendingBit(uart_config->usart, USART_IT_RXNE);
	    USART_ITConfig(uart_config->usart, USART_IT_RXNE, ENABLE);
    }

    // Transmit IRQ
    if (mode & uart_mode_tx) {
        USART_ITConfig(uart_config->usart, USART_IT_TXE, ENABLE);
    }

    USART_Cmd(uart_config->usart, ENABLE);

done:
    return pctx;
}

int uartRxReady(void *pv)
{
	uart_ctx_st *pctx = pv;

    return (pctx->rxBufferHead - pctx->rxBufferTail) & (pctx->rxBufferSize - 1);
}

int uartTxBusy(void *pv)
{
	uart_ctx_st *pctx = pv;

    return pctx->txBufferTail == pctx->txBufferHead;
}

int uartReadChar(void *pv)
{
	uart_ctx_st *pctx = pv;
    uint8_t ch;

    STM_EVAL_LEDToggle(LED9);

    ch = pctx->rxBuffer[pctx->rxBufferTail];
    pctx->rxBufferTail = (pctx->rxBufferTail + 1) % pctx->rxBufferSize;

    return ch;
}

void uartWriteChar(void *pv, uint8_t ch)
{
	uart_ctx_st *pctx = pv;

    STM_EVAL_LEDToggle(LED10);

    pctx->txBuffer[pctx->txBufferHead] = ch;
    pctx->txBufferHead = (pctx->txBufferHead + 1) % pctx->txBufferSize;

    USART_ITConfig(pctx->port->usart, USART_IT_TXE, ENABLE);
}



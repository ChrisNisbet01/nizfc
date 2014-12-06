#include <stdlib.h>
#include <stdint.h>

#if defined(STM32F30X)
#include <stm32f30x_usart.h>
#elif defined(STM32F10X)
#include <stm32f10x_usart.h>
#endif

#include <usart.h>
#include <coos.h>
#include "uart_interface.h"
#include "serial.h"

#define USART1_RX_BUFFER_SIZE	128	/* must be power of two */
#define USART1_TX_BUFFER_SIZE	128
#define USART2_RX_BUFFER_SIZE	128	/* must be power of two */
#define USART2_TX_BUFFER_SIZE	128

static volatile uint8_t Usart1TxBuffer[USART1_TX_BUFFER_SIZE];
static volatile uint8_t Usart1RxBuffer[USART1_RX_BUFFER_SIZE];
static volatile uint8_t Usart2TxBuffer[USART2_TX_BUFFER_SIZE];
static volatile uint8_t Usart2RxBuffer[USART2_RX_BUFFER_SIZE];

typedef struct serial_uart_statistics_st
{
	uint32_t txTimeout;
} serial_uart_statistics_st;

typedef struct uart_ports_config_t
{
	serial_port_t port;
	USART_TypeDef *usart;
	volatile uint8_t 	*rxBuffer;
	uint_fast16_t		rxBufferSize;
	volatile uint8_t 	*txBuffer;
	uint_fast16_t		txBufferSize;

} uart_ports_config_t;


typedef struct uart_ctx_st
{
	uint_fast32_t				baudRate;
	uart_ports_config_t	const   * uartConfig;
	serial_modes_t				mode;
	void const          		* ll_info;	/* returned by micro specific init function */
	volatile uint8_t 			* rxBuffer;
	uint_fast16_t				rxBufferSize;
	volatile uint8_t 			* txBuffer;
	uint_fast16_t				txBufferSize;
	volatile uint_fast16_t		rxBufferHead;
	volatile uint_fast16_t		rxBufferTail;
	volatile uint_fast16_t		txBufferHead;
	volatile uint_fast16_t		txBufferTail;

	void 						(*newRxDataCb)( void *pv );

	serial_port_st				serialPort;
} uart_ctx_st;

static int uartRxReady(void *pv);
static int uartTxBusy(void *pv);
static int uartReadChar(void *pv);
static void uartWriteChar(void *pv, uint8_t ch);
static int uartWriteCharBlockingWithTimeout(void * const pv, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait);

static const serial_port_methods_st uart_port_methods =
{
	.readChar = uartReadChar,
	.txBusy = uartTxBusy,
	.rxReady = uartRxReady,
	.writeChar = uartWriteChar,
	.writeCharBlockingWithTimeout = uartWriteCharBlockingWithTimeout,
	.writeBulk = NULL,
	.writeBulkBlockingWithTimeout = NULL
};


static const uart_ports_config_t uart_ports[] =
{
#if defined(STM32F30X)
	{
	.port = SERIAL_UART_2,
	.usart = USART2,
	.rxBuffer = Usart2RxBuffer,
	.rxBufferSize = sizeof Usart2RxBuffer,
	.txBuffer = Usart2TxBuffer,
	.txBufferSize = sizeof Usart2TxBuffer
	}
#elif defined(STM32F10X)
	{
	.port = SERIAL_UART_1,
	.usart = USART1,
	.rxBuffer = Usart1RxBuffer,
	.rxBufferSize = sizeof Usart1RxBuffer,
	.txBuffer = Usart1TxBuffer,
	.txBufferSize = sizeof Usart1TxBuffer
	},
	{
	.port = SERIAL_UART_2,
	.usart = USART2,
	.rxBuffer = Usart2RxBuffer,
	.rxBufferSize = sizeof Usart2RxBuffer,
	.txBuffer = Usart2TxBuffer,
	.txBufferSize = sizeof Usart2TxBuffer
	}
#endif
};

#define NB_UART_PORTS	(sizeof(uart_ports)/sizeof(uart_ports[0]))
#define UART_IDX(ptr)	((ptr)-uart_ports)

static uart_ctx_st uart_ctxs[NB_UART_PORTS];
static serial_uart_statistics_st serial_uart_statistics;

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

	if (pctx->newRxDataCb != NULL)
	{
		pctx->newRxDataCb(pctx);
	}
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

    USART_Init(pctx->uartConfig->usart, &USART_InitStructure);
}

static uart_ports_config_t const * uartPortLookup( serial_port_t port )
{
	unsigned int i;

	for (i=0; i < NB_UART_PORTS; i++)
	{
		if ( uart_ports[i].port == port )
			return &uart_ports[i];
	}

	return NULL;
}

serial_port_st * uartOpen( serial_port_t port, uint32_t baudrate, serial_modes_t mode, void (*newRxDataCb)( void *pv ) )
{
	uart_ports_config_t const * uart_config;
	uart_ctx_st *pctx;
	usart_init_st	cfg;
	serial_port_st *serialPort = NULL;

	if ( (uart_config=uartPortLookup( port )) != NULL )
	{
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

		if ((pctx->ll_info=stm32_usart_init( &cfg )) == NULL)
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
	    pctx->uartConfig = uart_config;
	    pctx->mode = mode;
	    pctx->baudRate = baudrate;
	    pctx->newRxDataCb = newRxDataCb;

		pctx->serialPort.serialCtx = pctx;
		pctx->serialPort.methods = &uart_port_methods;
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

		serialPort = &pctx->serialPort;
	}

done:
    return serialPort;
}

static int uartRxReady(void *pv)
{
	uart_ctx_st *pctx = pv;

	/* XXX assumes that buffer length is a power a two */
    return (pctx->rxBufferHead - pctx->rxBufferTail) & (pctx->rxBufferSize - 1);
}

static int uartTxBusy(void *pv)
{
	uart_ctx_st *pctx = pv;

    return pctx->txBufferTail == pctx->txBufferHead;
}

static int uartReadChar(void *pv)
{
	uart_ctx_st *pctx = pv;
    uint8_t ch;

    ch = pctx->rxBuffer[pctx->rxBufferTail];
    pctx->rxBufferTail = (pctx->rxBufferTail + 1) % pctx->rxBufferSize;

    return ch;
}

static void uartWriteChar(void *pv, uint8_t ch)
{
	uart_ctx_st *pctx = pv;

    pctx->txBuffer[pctx->txBufferHead] = ch;
    pctx->txBufferHead = (pctx->txBufferHead + 1) % pctx->txBufferSize;

    USART_ITConfig(pctx->uartConfig->usart, USART_IT_TXE, ENABLE);
}

static int uartWriteCharBlockingWithTimeout(void * const pv, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait)
{
	uart_ctx_st *pctx = pv;
	uint_fast16_t millisecs_counter = 0;
	int result = -1;
	int timed_out = 0;

	/* wait until the TX buffer can accept at least one more character */
	while (((pctx->txBufferHead + 1) % pctx->txBufferSize) == pctx->txBufferTail)
	{
		if (millisecs_counter >= max_millisecs_to_wait)
		{
			timed_out = 1;
			break;
		}
		CoTimeDelay( 0, 0, 0, 10 );
		millisecs_counter += 10;
	}
	if ( timed_out == 0 )
	{
	    pctx->txBuffer[pctx->txBufferHead] = ch;
	    pctx->txBufferHead = (pctx->txBufferHead + 1) % pctx->txBufferSize;

	    USART_ITConfig(pctx->uartConfig->usart, USART_IT_TXE, ENABLE);
	    result = 0;
	}
	else
	{
		serial_uart_statistics.txTimeout++;
		result = -1;
	}

	return result;
}



#include <stdlib.h>
#include <stdint.h>

#include <stm32f30x_usart.h>
#include <usart_stm32f30x.h>
#include <stm32f3_discovery.h>
#include <coos.h>
#include "uart_interface.h"
#include "uart.h"

#include "usb_core.h"
#include "usb_init.h"
#include "hw_config.h"

#define USART2_RX_BUFFER_SIZE	128
#define USART2_TX_BUFFER_SIZE	128
#define USB_TIMEOUT				50 /* ms * 10 */

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

typedef struct usb_uart_config_t
{
	uart_ports_t port;
} usb_uart_config_t;

typedef enum uart_type_t
{
	uart_type_uart,
	uart_type_usb
} uart_type_t;

typedef struct uart_ctx_st
{
	int							type;		/* UART or USB VCP */
	uart_ports_config_t			*uart_config;
	uint_fast32_t				baudRate;
	uart_ports_config_t	const 	*port;
	uart_modes_t				mode;
	void const          		*ll_info;	/* returned by micro specific init function */
	volatile uint8_t 			*rxBuffer;
	uint_fast16_t				rxBufferSize;
	volatile uint8_t 			*txBuffer;
	uint_fast16_t				txBufferSize;
	volatile uint_fast16_t		rxBufferHead;
	volatile uint_fast16_t		rxBufferTail;
	volatile uint_fast16_t		txBufferHead;
	volatile uint_fast16_t		txBufferTail;

	void 						(*newRxDataCb)( void *pv );
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

static const usb_uart_config_t usb_uart_ports[] =
{
	{
	.port = UART_USB
	}
};


#define NB_UART_PORTS	(sizeof(uart_ports)/sizeof(uart_ports[0]))
#define UART_IDX(ptr)	((ptr)-uart_ports)

#define NB_USB_PORTS	(sizeof(usb_uart_ports)/sizeof(usb_uart_ports[0]))
#define USB_IDX(ptr)	((ptr)-usb_uart_ports)

static uart_ctx_st uart_ctxs[NB_UART_PORTS+NB_USB_PORTS];

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

void newUSBData( void )
{
	uart_ctx_st *pctx = &uart_ctxs[NB_UART_PORTS];

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

static usb_uart_config_t const * usbUartPortLookup( uart_ports_t port )
{
	unsigned int i;

	for (i=0; i < NB_USB_PORTS; i++)
	{
		if ( usb_uart_ports[i].port == port )
			return &usb_uart_ports[i];
	}

	return NULL;
}

void *uartOpen( uart_ports_t port, uint32_t baudrate, uart_modes_t mode, void (*newRxDataCb)( void *pv ) )
{
	uart_ports_config_t const * uart_config;
	usb_uart_config_t const *usb_config;
	uart_ctx_st *pctx;
	usart_init_st	cfg;

	if ( (uart_config=uartPortLookup( port )) != NULL )
	{
		pctx = &uart_ctxs[UART_IDX(uart_config)];

		pctx->type = 0;
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
	    pctx->newRxDataCb = newRxDataCb;

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
	}
	else if ( (usb_config=usbUartPortLookup( port )) != NULL )
	{
		pctx = &uart_ctxs[NB_UART_PORTS + USB_IDX(usb_config)];
		pctx->type = 1;	/* usb uart */
	    pctx->newRxDataCb = newRxDataCb;

	    Set_System();
	    Set_USBClock();
	    USB_Interrupts_Config();
	    USB_Init();

	}
	else
	{
		pctx = NULL;
		goto done;
	}

done:
    return pctx;
}

int uartRxReady(void *pv)
{
	uart_ctx_st *pctx = pv;

	if ( pctx->type == 0 )
	{
		/* XXX assumes that buffer length is a power a two */
	    return (pctx->rxBufferHead - pctx->rxBufferTail) & (pctx->rxBufferSize - 1);
	}
	else /* usb uart */
	{
		extern volatile uint32_t receiveLength;
	    return receiveLength;
	}
}

int uartTxBusy(void *pv)
{
	uart_ctx_st *pctx = pv;

	if ( pctx->type == 0 )
	{
	    return pctx->txBufferTail == pctx->txBufferHead;
	}
	else
	{
		extern volatile uint32_t packetSent;
	    return packetSent;
	}
}

int uartReadChar(void *pv)
{
	uart_ctx_st *pctx = pv;
    uint8_t ch;

	if ( pctx->type == 0 )
	{
	    ch = pctx->rxBuffer[pctx->rxBufferTail];
	    pctx->rxBufferTail = (pctx->rxBufferTail + 1) % pctx->rxBufferSize;
	}
	else
	{
	    uint8_t buf[1];

	    uint32_t rxed = 0;

	    while (rxed == 0) {
	        rxed = CDC_Receive_DATA(buf, 1);
	    }

	    return buf[0];
	}

    return ch;
}

void uartWriteChar(void *pv, uint8_t ch)
{
	uart_ctx_st *pctx = pv;

	if ( pctx->type == 0 )
	{
	    pctx->txBuffer[pctx->txBufferHead] = ch;
	    pctx->txBufferHead = (pctx->txBufferHead + 1) % pctx->txBufferSize;

	    USART_ITConfig(pctx->port->usart, USART_IT_TXE, ENABLE);
	}
	else
	{
	    uint32_t txed;
	    uint32_t start = CoGetOSTime();

	    if (!(usbIsConnected() && usbIsConfigured())) {
	        return;
	    }

	    do {
	        txed = CDC_Send_DATA((uint8_t*)&ch, 1);
	        if (txed == 0)
				CoTimeDelay( 0, 0, 0, 1000/CFG_SYSTICK_FREQ );
	    } while (txed < 1 && (CoGetOSTime() - start < USB_TIMEOUT));
	}
}

int uartWriteCharBlockingWithTimeout(void * const pv, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait)
{
	uart_ctx_st *pctx = pv;
	uint_fast16_t millisecs_counter = 0;
	int result = -1;
	int timed_out = 0;

	if ( pctx->type == 0 )
	{
		/* wait until the TX buffer can accept at least one more character */
		while (((pctx->txBufferHead + 1) % pctx->txBufferSize) == pctx->txBufferTail)
		{
			if (millisecs_counter >= max_millisecs_to_wait)
			{
				timed_out = 1;
				break;
			}
			CoTimeDelay( 0, 0, 0, 1000/CFG_SYSTICK_FREQ );
			millisecs_counter += 1000/CFG_SYSTICK_FREQ;
		}
		if ( timed_out == 0 )
		{
		    pctx->txBuffer[pctx->txBufferHead] = ch;
		    pctx->txBufferHead = (pctx->txBufferHead + 1) % pctx->txBufferSize;

		    USART_ITConfig(pctx->port->usart, USART_IT_TXE, ENABLE);
		    result = 0;
		}
		else
		{
			result = -1;
			// TODO: increment a statistic?
		}
	}
	else	/* USB */
	{
	    uint32_t txed;
	    uint32_t start = CoGetOSTime();

	    if (!(usbIsConnected() && usbIsConfigured())) {
	        return -1;
	    }

		while (txed = CDC_Send_DATA((uint8_t*)&ch, 1) == 0)
		{
			if (millisecs_counter >= max_millisecs_to_wait)
			{
				timed_out = 1;
				break;
			}
			CoTimeDelay( 0, 0, 0, 1000/CFG_SYSTICK_FREQ );
			millisecs_counter += 1000/CFG_SYSTICK_FREQ;
		}
		if ( timed_out == 0 )
		{
		    result = 0;
		}
		else
		{
			result = -1;
			// TODO: increment a statistic?
		}

	}

	return result;
}


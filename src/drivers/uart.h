#ifndef __UART_H__
#define __UART_H__

typedef enum uart_ports_t
{
	UART_2
} uart_ports_t;

typedef enum uart_modes_t
{
	uart_mode_tx = (1<<0),
	uart_mode_rx = (1<<1),
} uart_modes_t;

void *uartOpen( uart_ports_t port, uint32_t baudrate, uart_modes_t mode );

void uartWriteChar( void *pv, uint8_t ch );
int uartReadChar( void *pv );
int uartTxBusy( void *pv );
int uartRxReady( void *pv );

#endif /*  __UART_H__ */

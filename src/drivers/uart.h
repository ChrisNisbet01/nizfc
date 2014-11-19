#ifndef __UART_H__
#define __UART_H__

typedef enum uart_ports_t
{
	UART_2,
	UART_USB
} uart_ports_t;

typedef enum uart_modes_t
{
	uart_mode_tx = (1<<0),
	uart_mode_rx = (1<<1),
} uart_modes_t;

typedef struct serial_port_methods_st
{
	int (*readChar)( void * serialPortCtx );
	int (*txBusy)( void * serialPortCtx );
	int (*rxReady)( void * serialPortCtx );
	void (*writeChar)( void * serialPortCtx, uint8_t ch );
	int (*writeCharBlockingWithTimeout)(void * serialPortCtx, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait);
	void (*writeBulk)( void * serialPortCtx, uint8_t *buf, unsigned int buflen );	/* may be NULL */
	int (*writeBulkBlockingWithTimeout)(void * serialPortCtx, uint8_t * buf, unsigned int buflen, uint_fast16_t const max_millisecs_to_wait);
} serial_port_methods_st;

typedef struct serial_port_st
{
	serial_port_methods_st const * methods;
	void * serialCtx;
} serial_port_st;

serial_port_st * uartOpen( uart_ports_t port, uint32_t baudrate, uart_modes_t mode, void (*newRxDataCb)( void *pv ) );

#endif /*  __UART_H__ */

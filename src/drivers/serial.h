#ifndef __SERIAL_H__
#define __SERIAL_H__

typedef enum serial_port_t
{
	SERIAL_UART_1,
	SERIAL_UART_2,
	SERIAL_USB
} serial_port_t;

typedef enum serial_modes_t
{
	uart_mode_tx = (1<<0),
	uart_mode_rx = (1<<1),
} serial_modes_t;

typedef struct serial_port_methods_st
{
	int (*readChar)( void * serialPortCtx );
	int (*txBusy)( void * serialPortCtx );
	int (*rxReady)( void * serialPortCtx );
	void (*writeChar)( void * serialPortCtx, uint8_t ch );
	int (*writeCharBlockingWithTimeout)(void * serialPortCtx, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait);
	int (*writeBulk)( void * serialPortCtx, uint8_t const * buf, unsigned int buflen );	/* may be NULL */
	int (*writeBulkBlockingWithTimeout)(void * serialPortCtx, uint8_t const * buf, unsigned int buflen, uint_fast16_t const max_millisecs_to_wait);
} serial_port_methods_st;

typedef struct serial_port_st
{
	serial_port_methods_st const * methods;
	void * serialCtx;
} serial_port_st;

serial_port_st * serialOpen( serial_port_t port, uint32_t baudrate, serial_modes_t mode, void (*newRxDataCb)( void *pv ) );

#endif /*  __SERIAL_H__ */

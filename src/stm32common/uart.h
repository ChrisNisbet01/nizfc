#ifndef __UART_H__
#define __UART_H__

#include <serial.h>

serial_port_st * uartOpen( serial_port_t port, uint32_t baudrate, serial_modes_t mode, void (*newRxDataCb)( void *pv ) );

#endif /*  __UART_H__ */

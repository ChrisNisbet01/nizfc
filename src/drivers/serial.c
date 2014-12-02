#include <stdlib.h>
#include <stdint.h>

#include <coos.h>
#include <serial.h>
#include <uart.h>
#include <serial_usb.h>


serial_port_st * serialOpen( serial_port_t port, uint32_t baudrate, serial_modes_t mode, void (*newRxDataCb)( void *pv ) )
{
	serial_port_st * serialPort;

	if ( (serialPort=uartOpen( port, baudrate, mode, newRxDataCb )) == NULL )
	{
#if defined(STM32F30X)
		serialPort = usbSerialOpen( port, baudrate, mode, newRxDataCb );
#endif
	}

    return serialPort;
}



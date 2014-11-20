#ifndef __SERIAL_USB_H__
#define __SERIAL_USB_H__

#include <serial.h>

serial_port_st * usbSerialOpen( serial_port_t port, uint32_t baudrate, serial_modes_t mode, void (*newRxDataCb)( void *pv ) );

#endif /*  __SERIAL_USB_H__ */

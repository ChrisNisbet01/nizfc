#include <stdlib.h>
#include <stdint.h>

#include <stm32f3_discovery.h>
#include <coos.h>
#include <serial.h>

#include "usb_core.h"
#include "usb_init.h"
#include "hw_config.h"

#define USB_TIMEOUT				50 /* ms * 10 */

typedef struct usb_config_t
{
	serial_port_t port;
} usb_config_t;


typedef struct usb_serial_ctx_st
{
	OS_FlagID					usbTxCompleteFlag;

	void 						(* newRxDataCb)( void *pv );
	usb_config_t				const * usb;
	serial_port_st				serialPort;
} usb_serial_ctx_st;

static int usbRxReady(void *pv);
static int usbTxBusy(void *pv);
static int usbReadChar(void *pv);
static void usbWriteChar(void *pv, uint8_t ch);
static int usbWriteCharBlockingWithTimeout(void * const pv, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait);
static int usbWriteBulkBlockingWithTimeout(void * const pv, uint8_t * buf, unsigned int buflen, uint_fast16_t const max_millisecs_to_wait);
static int usbWriteBulkBlocking(void * const pv, uint8_t * buf, unsigned int buflen);


static const serial_port_methods_st usb_port_methods =
{
	.readChar = usbReadChar,
	.txBusy = usbTxBusy,
	.rxReady = usbRxReady,
	.writeChar = usbWriteChar,
	.writeCharBlockingWithTimeout = usbWriteCharBlockingWithTimeout,
	.writeBulk = usbWriteBulkBlocking,
	.writeBulkBlockingWithTimeout = usbWriteBulkBlockingWithTimeout
};

static const usb_config_t usb_serial_ports[] =
{
	{
	.port = SERIAL_USB
	}
};

#define NB_USB_PORTS	(sizeof(usb_serial_ports)/sizeof(usb_serial_ports[0]))
#define USB_IDX(ptr)	((ptr)-usb_serial_ports)

static usb_serial_ctx_st usb_serial_ctxs[NB_USB_PORTS];

void usbTxComplete( void )
{
	usb_serial_ctx_st *pctx = &usb_serial_ctxs[0];

	CoEnterISR();

	isr_SetFlag( pctx->usbTxCompleteFlag );

	CoExitISR();
}

void newUSBData( void )
{
	usb_serial_ctx_st *pctx = &usb_serial_ctxs[0];

	if (pctx->newRxDataCb != NULL)
	{
		pctx->newRxDataCb(pctx);
	}
}

static usb_config_t const * usbSerialPortLookup( serial_port_t port )
{
	unsigned int i;

	for (i=0; i < NB_USB_PORTS; i++)
	{
		if ( usb_serial_ports[i].port == port )
			return &usb_serial_ports[i];
	}

	return NULL;
}

serial_port_st * usbSerialOpen( serial_port_t port, uint32_t baudrate, serial_modes_t mode, void (*newRxDataCb)( void *pv ) )
{
	usb_config_t const *usb_config;
	usb_serial_ctx_st *pctx;
	serial_port_st *serialPort = NULL;

	if ( (usb_config=usbSerialPortLookup( port )) != NULL )
	{
		pctx = &usb_serial_ctxs[USB_IDX(usb_config)];
		pctx->usb = usb_config;
		pctx->serialPort.serialCtx = pctx;
		pctx->serialPort.methods = &usb_port_methods;

	    pctx->newRxDataCb = newRxDataCb;
	    pctx->usbTxCompleteFlag = CoCreateFlag( Co_TRUE, Co_TRUE );

	    Set_System();
	    Set_USBClock();
	    USB_Interrupts_Config();
	    USB_Init();

		serialPort = &pctx->serialPort;

	}

    return serialPort;
}

static int usbRxReady(void *pv)
{
	usb_serial_ctx_st *pctx = pv;

	extern volatile uint32_t receiveLength;
    return receiveLength;

}

static int usbTxBusy(void *pv)
{
	usb_serial_ctx_st *pctx = pv;

	extern volatile uint32_t packetSent;

    return packetSent != 0;

}

static int usbReadChar(void *pv)
{
	usb_serial_ctx_st *pctx = pv;
    uint8_t buf[1];

    CDC_Receive_DATA(buf, 1);	/* Note no waiting, or timeout */

    return buf[0];

}

static void usbWriteChar(void *pv, uint8_t ch)
{
	usb_serial_ctx_st *pctx = pv;

    if (!(usbIsConnected() && usbIsConfigured()))
    {
        return;
    }
    // TODO: need to detect USB error, which might result in packetSent nevr being cleared
	if ( CoWaitForSingleFlag( pctx->usbTxCompleteFlag, CFG_SYSTICK_FREQ/10 ) == E_OK || packetSent == 0 )
	{
        CDC_Send_DATA((uint8_t*)&ch, 1);
	}
}

static int usbWriteCharBlockingWithTimeout(void * const pv, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait)
{
	usb_serial_ctx_st *pctx = pv;
	int result = -1;

	if ( CoWaitForSingleFlag( pctx->usbTxCompleteFlag, CFG_SYSTICK_FREQ/10 ) == E_OK || packetSent == 0 )
	{
        CDC_Send_DATA((uint8_t*)&ch, 1);
        result = 0;
	}
	else
	{
		result = -1;
		// TODO: increment a statistic?
	}

	return result;
}

static int usbWriteBulkBlockingWithTimeout(void * const pv, uint8_t * buf, unsigned int buflen, uint_fast16_t const max_millisecs_to_wait)
{
	usb_serial_ctx_st *pctx = pv;
	int result = 0;
	unsigned int tosend = buflen;

	do
	{
		if ( (max_millisecs_to_wait > 0 && CoWaitForSingleFlag( pctx->usbTxCompleteFlag, max_millisecs_to_wait/(1000/CFG_SYSTICK_FREQ) )) == E_OK
			|| packetSent == 0 )
		{
	        tosend -= CDC_Send_DATA(buf, tosend);
		}
		else
		{
			result = -1;
			// TODO: increment a statistic?
		}
	}
	while (tosend > 0 && result == 0);

	return result;
}

static int usbWriteBulkBlocking(void * const pv, uint8_t * buf, unsigned int buflen)
{
	return usbWriteBulkBlockingWithTimeout( pv, buf, buflen, 0 );
}


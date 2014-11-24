#include <stdlib.h>
#include <stdint.h>

#include <stm32f3_discovery.h>
#include <coos.h>
#include <serial.h>
#include <utils.h>
#include "usb_core.h"
#include "usb_init.h"
#include "hw_config.h"

#define USB_TIMEOUT				50 /* ms * 10 */

typedef struct serial_usb_statistics_st
{
	uint32_t txTimeout;
	uint32_t txFail;
} serial_usb_statistics_st;

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

static serial_usb_statistics_st serial_usb_statistics;

static int usbRxReady(void *pv);
static int usbTxBusy(void *pv);
static int usbReadChar(void *pv);
static void usbWriteChar(void *pv, uint8_t ch);
static int usbWriteCharBlockingWithTimeout(void * const pv, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait);
static int usbWriteBulkBlockingWithTimeout(void * const pv, uint8_t const * buf, unsigned int buflen, uint_fast16_t const max_millisecs_to_wait);
static int usbWriteBulkBlocking(void * const pv, uint8_t const * buf, unsigned int buflen);


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

	UNUSED(baudrate);
	UNUSED(mode);

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
	extern __IO uint32_t usbReceiveLength;

	UNUSED(pv);

    return usbReceiveLength;

}

static int usbTxBusy(void *pv)
{
	extern __IO uint32_t usbPacketSent;

	UNUSED(pv);

    return usbPacketSent != 0;

}

static int usbReadChar(void *pv)
{
    uint8_t buf[1];

	UNUSED(pv);

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
    // TODO: need to detect USB error, which might result in usbPacketSent nevr being cleared
	if ( CoWaitForSingleFlag( pctx->usbTxCompleteFlag, CFG_SYSTICK_FREQ/10 ) == E_OK || usbPacketSent == 0 )
	{
        if ( CDC_Send_DATA(&ch, 1) == 0 )
        {
        	/* transmit failed. set the flag again so we can try again quickly */
        	CoSetFlag(pctx->usbTxCompleteFlag);
        	serial_usb_statistics.txFail++;
        }
	}
	else
	{
		serial_usb_statistics.txTimeout++;
	}
}

static int usbWriteCharBlockingWithTimeout(void * const pv, uint8_t const ch, uint_fast16_t const max_millisecs_to_wait)
{
	usb_serial_ctx_st *pctx = pv;
	int result = -1;

	if ( CoWaitForSingleFlag( pctx->usbTxCompleteFlag, max_millisecs_to_wait/(1000/CFG_SYSTICK_FREQ) ) == E_OK || usbPacketSent == 0 )
	{
        if ( CDC_Send_DATA(&ch, 1) == 0 )
        {
        	/* transmit failed. set the flag again so we can try again quickly */
        	CoSetFlag(pctx->usbTxCompleteFlag);
        	serial_usb_statistics.txFail++;
        }
        else
        	result = 0;
	}
	else
	{
		result = -1;
		serial_usb_statistics.txTimeout++;
	}

	return result;
}

static int usbWriteBulkBlockingWithTimeout(void * const pv, uint8_t const * buf, unsigned int buflen, uint_fast16_t const max_millisecs_to_wait)
{
	usb_serial_ctx_st *pctx = pv;
	int result = 0;
	unsigned int tosend = buflen;

	do
	{
		if ( (max_millisecs_to_wait > 0 && CoWaitForSingleFlag( pctx->usbTxCompleteFlag, max_millisecs_to_wait/(1000/CFG_SYSTICK_FREQ) )) == E_OK
			|| usbPacketSent == 0 )
		{
			uint32_t written = CDC_Send_DATA(buf, tosend);

			if ( written == 0 )
			{
	        	serial_usb_statistics.txFail++;
				result = -1;
			}
			else
			{
				buf += written;
		        tosend -= written;
			}
		}
		else
		{
			serial_usb_statistics.txTimeout++;
			result = -1;
		}
	}
	while (tosend > 0 && result == 0);

	return result;
}

static int usbWriteBulkBlocking(void * const pv, uint8_t const * buf, unsigned int buflen)
{
	return usbWriteBulkBlockingWithTimeout( pv, buf, buflen, 0 );
}


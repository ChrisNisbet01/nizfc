#ifndef __UART_INTERFACE_H__
#define  __UART_INTERFACE_H__

typedef enum USART_MODE_T
{
	usart_mode_tx = (1<<0),
	usart_mode_rx = (1<<1)
} USART_MODE_T;

typedef struct usart_cb_st
{
	void			*pv;						/* owner context */
	int				(*getTxChar)( void *pv );	/* get next char to TX to UART from owner */
	uint8_t			(*putRxChar)( void *pv, uint8_t ch );	/* put next char from UART to owner */
} usart_cb_st;

typedef struct usart_init_st
{
	USART_MODE_T	mode;
	USART_TypeDef	*usart;
	usart_cb_st		callback;
	// TODO: DMA support
} usart_init_st;


#endif

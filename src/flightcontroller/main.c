#include <stdlib.h>
#include <stm32f30x_wwdg.h>
#include <coos.h>
#include <stm32f3_discovery.h>
#include <uart.h>
#include <utils.h>

#define MAIN_TASK_STACK_SIZE 0x100
static OS_STK main_task_stack[MAIN_TASK_STACK_SIZE] = {0};
void *debug_uart;

void main_task( void *pv )
{
	UNUSED(pv);

	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);

	debug_uart = uartOpen( UART_2, 115200, uart_mode_rx | uart_mode_tx );

	while (1)
	{
		CoTimeDelay(0, 0, 0, 500);
        STM_EVAL_LEDToggle(LED3);

		if ( debug_uart != NULL )
		{
			while ( uartRxReady( debug_uart ) )
			{
				uint8_t ch;

				ch = uartReadChar( debug_uart );
				uartWriteChar( debug_uart, ch );
			}
			uartWriteChar( debug_uart, 'a' );
		}

	}
}

int main(void)
{
	WWDG_DeInit();
	CoInitOS();

	CoCreateTask(main_task, Co_NULL, 0, &main_task_stack[MAIN_TASK_STACK_SIZE-1], MAIN_TASK_STACK_SIZE);
	CoStartOS();

	while (1);
}

#include <stdint.h>
#include <stdbool.h>
#include <coocox.h>
#include <misc.h>
#include <startup.h>
#include <hirestimer.h>
#include <leds.h>
#include <serial_task.h>
#include <main_task.h>

void _Default_Handler( void )
{
	setLEDMode(EXCEPTION_LED, led_state_on);
	while( 1 );
}

#if defined(STM32F10X)
static void systemInit(void)
{
    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    RCC_ClearFlag();

}
#endif

int main(void)
{

#if defined(STM32F10X)
	systemInit();
	SetSysClock(0);
#endif

	CoInitOS();

	initLEDs();

	initMicrosecondClock();


	/* open the serial ports early so debug info can be sent to them */
	initSerialTask();

	initMainTask();

	initialiseCodeGroups();

	loadSavedConfiguration();

	CoStartOS();

	/* prevent compiler warning. Should never get here */
	while (1);
}


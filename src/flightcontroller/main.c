#include <stdlib.h>
#include <stm32f30x_wwdg.h>
#include <coos.h>
#include <stm32f3_discovery.h>

#define MAIN_TASK_STACK_SIZE 0x100
static OS_STK main_task_stack[MAIN_TASK_STACK_SIZE] = {0};

void main_task( void *pv )
{
	STM_EVAL_LEDInit(LED3);
	while (1)
	{
		CoTimeDelay(0, 0, 0, 500);
        STM_EVAL_LEDToggle(LED3);
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

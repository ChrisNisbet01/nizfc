#include <stdlib.h>

#include <stm32f3_discovery.h>
#include <stm32f30x_tim.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <coos.h>

static uint32_t ticksPerMicrosecond;
static uint32_t ticksPerSystickInterrupt;
void (*callback)( void );

static void initTimerTimeBase(TIM_TypeDef *tim, uint_fast16_t period, uint_fast32_t frequency_hz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
    TIM_TimeBaseStructure.TIM_Prescaler = (clocks.SYSCLK_Frequency/frequency_hz) - 1;

    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void initTimerNVIC( uint_fast8_t irq )
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

void TIM7_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    if ( callback != NULL )
    {
		callback();
    }
}

void initHiResTimer( uint32_t periodMicrosecs, void (*appCallback)( void ) )
{
	callback = appCallback;
	/* enable timer clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    initTimerTimeBase(TIM7, periodMicrosecs, 1000000);

    initTimerNVIC(TIM7_IRQn);

	/* enable update interrupt */
	TIM_ITConfig( TIM7, TIM_IT_Update, ENABLE );

    /* start the timer */
    TIM_Cmd(TIM7, ENABLE);

}

uint32_t micros(void)
{
    uint32_t centiseconds, cycle_count;
	uint32_t microsValue;

    do
    {
        centiseconds = CoGetOSTime2();
	  __asm volatile
 		(
 		" MOV     R0,R0    \n"
	    );
        cycle_count = SysTick->VAL;
    }
    while (centiseconds != CoGetOSTime2());

	microsValue = (centiseconds * (1000000/CFG_SYSTICK_FREQ)) + ( ticksPerSystickInterrupt - cycle_count) / ticksPerMicrosecond;

    return microsValue;
}

uint32_t micros2(void)
{
    uint32_t centiseconds, cycle_count;
	uint32_t microsValue;
	int loops = 0;

    do
    {
        centiseconds = CoGetOSTime2();
		  __asm volatile
	 		(
	 		" MOV     R0,R0    \n"
		    );
        cycle_count = SysTick->VAL;
        loops++;
    }
    while (centiseconds != CoGetOSTime2());

	if ( loops > 1 )
	{
		STM_EVAL_LEDToggle(LED9);
	}

	microsValue = (centiseconds * (1000000/CFG_SYSTICK_FREQ)) + ( ticksPerSystickInterrupt - cycle_count) / ticksPerMicrosecond;

    return microsValue;
}

void initMicrosecondClock(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    ticksPerMicrosecond = clocks.SYSCLK_Frequency / 1000000;
    ticksPerSystickInterrupt = ticksPerMicrosecond * (1000000/CFG_SYSTICK_FREQ);
}



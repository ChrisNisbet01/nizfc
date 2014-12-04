#include <stdlib.h>
#include <stdbool.h>

#if defined(STM32F30X)
#include <stm32f30x_tim.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#elif defined(STM32F10X)
#include <stm32f10x.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#endif
#include <coos.h>

static uint32_t ticksPerMicrosecond;
static uint32_t numTicksToWaitForRollover;
static uint32_t ticksPerSystickInterrupt;
void (*callback)( void );

#if defined(STM32F30X)
#define HIRESTIM	TIM7
#define HIRESTIM_IRQn	TIM7_IRQn
#define HIRESTTIM_CLK	RCC_APB1Periph_TIM7
#define HIRESTIM_IRQHandler TIM7_IRQHandler
#elif defined(STM32F10X)
#define HIRESTIM	TIM3
#define HIRESTIM_IRQn	TIM3_IRQn
#define HIRESTTIM_CLK	RCC_APB1Periph_TIM3
#define HIRESTIM_IRQHandler TIM3_IRQHandler
#endif

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

void HIRESTIM_IRQHandler(void)
{
    TIM_ClearITPendingBit(HIRESTIM, TIM_IT_Update);
    if ( callback != NULL )
    {
		callback();
    }
}

void initHiResTimer( uint32_t periodMicrosecs, void (*appCallback)( void ) )
{
	callback = appCallback;
	/* enable timer clock */
    RCC_APB1PeriphClockCmd(HIRESTTIM_CLK, ENABLE);

    initTimerTimeBase(HIRESTIM, periodMicrosecs, 1000000);

    initTimerNVIC(HIRESTIM_IRQn);

	/* enable update interrupt */
	TIM_ITConfig( HIRESTIM, TIM_IT_Update, ENABLE );

    /* start the timer */
    TIM_Cmd(HIRESTIM, ENABLE);

}

uint32_t __attribute__ ((noinline)) micros(void)
{
    uint32_t centiseconds, centiseconds2, cycle_count;
	uint32_t microsValue;
	bool waiting = false;

    do
    {
        centiseconds = CoGetOSTime32();
        cycle_count = SysTick->VAL;
        centiseconds2 = CoGetOSTime32();
        if ( waiting == true && centiseconds != centiseconds2 )
        {
        	/* must mean that CoGetOSTime32() has finally rolled */
        	break;
        }
    }
    while (centiseconds != centiseconds2
    	|| (( ticksPerSystickInterrupt - cycle_count) < numTicksToWaitForRollover && (waiting=true))
    );

	microsValue = (centiseconds2 * (1000000/CFG_SYSTICK_FREQ)) + ( ticksPerSystickInterrupt - cycle_count) / ticksPerMicrosecond;

    return microsValue;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}


void initMicrosecondClock(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    ticksPerMicrosecond = clocks.SYSCLK_Frequency / 1000000;
    numTicksToWaitForRollover = 1 * ticksPerMicrosecond;	/* 1us */
    ticksPerSystickInterrupt = ticksPerMicrosecond * (1000000/CFG_SYSTICK_FREQ);
}



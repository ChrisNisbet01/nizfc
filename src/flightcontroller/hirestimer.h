#ifndef __HI_RES_TIMER_H__
#define __HI_RES_TIMER_H__

void initHiResTimer( uint32_t periodMicrosecs, void (*appCallback)( void ) );
void initMicrosecondClock(void);
uint32_t micros(void);

#endif /* __HI_RES_TIMER_H__ */

#ifndef __FAILSAFE_H__
#define __FAILSAFE_H__

void initFailsafe( void (*failsafeTriggerCb)( void ) );
void enable_failsafe( void );
void disableFailsafe( void );
void updateFailsafeWithNewChannels( uint32_t newChannels );	/* newChannels is a bitmask representing the channels we have received data for */
void failsafeHasTriggered( void );
bool hasFailsafeTriggered( void );
uint_fast16_t getFailsafeMotorSpeed( void );
void resetFailsafeTrigger( void );

#endif /*  __FAILSAFE_H__ */

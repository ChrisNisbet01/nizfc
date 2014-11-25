#ifndef __RECEIVER_HANDLER_H__
#define __RECEIVER_HANDLER_H__

void processReceiverSignals( void );

float getThrottleSetpoint( void );
float getRollAngleSetpoint( void );
float getPitchAngleSetpoint( void );
float getRollRateSetpoint( void );
float getPitchRateSetpoint( void );
float getYawRateSetpoint( void );

void armCraft( void );
void disarmCraft( void );
bool isCraftArmed( void );

#endif


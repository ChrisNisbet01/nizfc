#ifndef __RECEIVER_HANDLER_H__
#define __RECEIVER_HANDLER_H__

void processStickPositions( void );

float getThrottleSetpoint( void );
float getRollAngleSetpoint( void );
float getPitchAngleSetpoint( void );
float getYawRateSetpoint( void );

#endif


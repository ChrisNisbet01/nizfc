#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <craft_types.h>

void initMotorControl( craft_type_t craftType );
void updateMotorOutputs( void );
uint16_t getMotorValue( uint_fast8_t motorIndex );

float getRollAnglePIDOutput( void );
float getPitchAnglePIDOutput( void );
float getRollRatePIDOutput( void );
float getPitchRatePIDOutput( void );

#endif /*  __MOTOR_CONTROL_H__ */

#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <craft_types.h>

void initMotorControl( craft_type_t craftType );
void updateMotorOutputs( void );
uint16_t getMotorOutput( uint_fast8_t motorIndex );
void setMotorDisarmedValue( uint_fast8_t motorIndex, uint_fast16_t value );

float getRollAnglePIDOutput( void );
float getPitchAnglePIDOutput( void );
float getRollRatePIDOutput( void );
float getPitchRatePIDOutput( void );

#endif /*  __MOTOR_CONTROL_H__ */

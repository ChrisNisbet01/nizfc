#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

void initMotorControl( void );
void updatePIDControlLoops( void );
void updateMotorOutputs( void );
uint16_t getMotorValue( uint_fast8_t motorIndex );


#endif /*  __MOTOR_CONTROL_H__ */

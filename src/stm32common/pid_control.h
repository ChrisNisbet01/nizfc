#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

#define THROTTLE_POSITION_TO_ENABLE_CONTROL_LOOPS	1050	// TODO: configurable

float getRollAnglePIDOutput( void );
float getPitchAnglePIDOutput( void );
float getRollRatePIDOutput( void );
float getPitchRatePIDOutput( void );
float getYawRatePIDOutput( void );
void resetAngleModePID( void );
void resetRateModePID( void );
void updatePIDControlLoops( void );
void initPIDControl( void );


#endif /* __PID_CONTROL_H__ */

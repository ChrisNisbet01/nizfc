#ifndef __BOARD_ALIGNMENT_H__
#define __BOARD_ALIGNMENT_H__

typedef enum orientation_t
{
	noRotation,
	clockwise90Degrees,
    clockwise180Degrees,
    clockwise270Degrees,
    clockwise0DegreesFlipped,
    clockwise90DegreesFlipped,
    clockwise180DegreesFlipped,
    clockwise270DegreesFlipped
} orientation_t;


void initBoardAlignment(float rollDegrees, float pitchDegrees, float yawDegrees);
void alignVectorsToCraft(float *vectors);
void alignVectorsToFlightController(float *vectors, orientation_t sensorOrientation);

#endif


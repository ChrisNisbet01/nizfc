#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <board_alignment.h>
#include <vector_rotation.h>

static bool boardAlignmentRequired = false;
static vectorRotation_st boardRotationMatrix;

void initBoardAlignment(float rollDegrees, float pitchDegrees, float yawDegrees)
{
    if (rollDegrees == 0.0f && pitchDegrees == 0.0f && yawDegrees == 0.0f)
        return;

    boardAlignmentRequired = true;

	initVectorRotationDegrees( &boardRotationMatrix, rollDegrees, pitchDegrees, yawDegrees );
}

void alignVectorToCraft(float *vector)
{
	if ( boardAlignmentRequired == true )
	{
		applyVectorRotation( &boardRotationMatrix, vector );
	}
}

void alignVectorToFlightController(float *vector, orientation_t sensorOrientation)
{
	/*
		These vectors will always be at 90 degree intervals, so we can avoid doing lengthy vector
		transformations.
	*/
	float temp[3];

	temp[0] = vector[0];
	temp[1] = vector[1];
	temp[2] = vector[2];

    switch (sensorOrientation) {
        case clockwise90Degrees:
            vector[0] = temp[1];
            vector[1] = -temp[0];
            break;
        case clockwise180Degrees:
            vector[0] = -temp[0];
            vector[1] = -temp[1];
            break;
        case clockwise270Degrees:
            vector[0] = -temp[1];
            vector[1] = temp[0];
            break;
        case clockwise0DegreesFlipped:
            vector[0] = -temp[0];
            vector[1] = temp[1];
            vector[2] = -temp[2];
            break;
        case clockwise90DegreesFlipped:
            vector[0] = temp[1];
            vector[1] = temp[0];
            vector[2] = -temp[2];
            break;
        case clockwise180DegreesFlipped:
            vector[0] = temp[0];
            vector[1] = -temp[1];
            vector[2] = -temp[2];
            break;
        case clockwise270DegreesFlipped:
            vector[0] = -temp[1];
            vector[1] = -temp[0];
            vector[2] = -temp[2];
            break;
        default:
        	/* nothing to do */
            break;
    }

}


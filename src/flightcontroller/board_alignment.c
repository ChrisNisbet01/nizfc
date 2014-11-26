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

void alignVectorsToCraft(float *vectors)
{
	if ( boardAlignmentRequired == true )
	{
		applyVectorRotation( &boardRotationMatrix, vectors );
	}
}

void alignVectorsToBoard(float *vectors, orientation_t sensorOrientation)
{
	float temp[3];

	temp[0] = vectors[0];
	temp[1] = vectors[1];
	temp[2] = vectors[2];

    switch (sensorOrientation) {
        case clockwise90Degrees:
            vectors[0] = temp[1];
            vectors[1] = -temp[0];
            break;
        case clockwise180Degrees:
            vectors[0] = -temp[0];
            vectors[1] = -temp[1];
            break;
        case clockwise270Degrees:
            vectors[0] = -temp[1];
            vectors[1] = temp[0];
            break;
        case clockwise0DegreesFlipped:
            vectors[0] = -temp[0];
            vectors[1] = temp[1];
            vectors[2] = -temp[2];
            break;
        case clockwise90DegreesFlipped:
            vectors[0] = temp[1];
            vectors[1] = temp[0];
            vectors[2] = -temp[2];
            break;
        case clockwise180DegreesFlipped:
            vectors[0] = temp[0];
            vectors[1] = -temp[1];
            vectors[2] = -temp[2];
            break;
        case clockwise270DegreesFlipped:
            vectors[0] = -temp[1];
            vectors[1] = -temp[0];
            vectors[2] = -temp[2];
            break;
        default:
        	/* nothing to do */
            break;
    }

}


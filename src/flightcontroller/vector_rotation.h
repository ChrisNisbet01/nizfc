#ifndef __VECTOR_ROTATION_H__
#define __VECTOR_ROTATION_H__

typedef struct vectorRotation_st
{
	float rotationMatrix[3][3];
} vectorRotation_st;

void initVectorRotation( vectorRotation_st *matrix, float rollDegrees, float pitchDegrees, float yawDegrees );
void applyVectorRotation( vectorRotation_st *matrix, float *vectors );

#endif

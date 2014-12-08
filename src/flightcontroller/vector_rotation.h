#ifndef __VECTOR_ROTATION_H__
#define __VECTOR_ROTATION_H__

typedef struct vectorRotation_st
{
	float rotationMatrix[3][3];
} vectorRotation_st;

void initVectorRotationDegrees( vectorRotation_st *matrix, float rollDegrees, float pitchDegrees, float yawDegrees );
void initVectorRotationRadians( vectorRotation_st *matrix, float rollRadians, float pitchRadians, float yawRadians );
void applyVectorRotation( vectorRotation_st *matrix, float *vectors );
void multiplyVectorMatrix( vectorRotation_st * MatrixA, vectorRotation_st * MatrixB, vectorRotation_st * NewMatrix );
void normalizeVectors(float *src, float *dest);

#endif

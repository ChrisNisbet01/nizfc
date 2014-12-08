#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <vector_rotation.h>

void initVectorRotationRadians( vectorRotation_st * matrix, float rollRadians, float pitchRadians, float yawRadians )
{
	/*
		Check out something like
		http://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
		to see how to obtain a rotation matrix from Euler angles.
	*/
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(rollRadians);
    sinx = sinf(rollRadians);
    cosy = cosf(pitchRadians);
    siny = sinf(pitchRadians);
    cosz = cosf(yawRadians);
    sinz = sinf(yawRadians);

	/* pre-multiply some values to save doing them multiple times */
	coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    // define the rotation matrix
    matrix->rotationMatrix[0][0] = coszcosy;
    matrix->rotationMatrix[0][1] = -cosy * sinz;
    matrix->rotationMatrix[0][2] = siny;

    matrix->rotationMatrix[1][0] = sinzcosx + coszsinx * siny;
    matrix->rotationMatrix[1][1] = coszcosx - sinzsinx * siny;
    matrix->rotationMatrix[1][2] = -sinx * cosy;

    matrix->rotationMatrix[2][0] = sinzsinx - coszcosx * siny;
    matrix->rotationMatrix[2][1] = coszsinx + sinzcosx * siny;
    matrix->rotationMatrix[2][2] = cosy * cosx;
}

void initVectorRotationDegrees( vectorRotation_st * matrix, float rollDegrees, float pitchDegrees, float yawDegrees )
{
    float roll, pitch, yaw;

    roll = rollDegrees * M_PI / 180.0f;
    pitch = pitchDegrees * M_PI / 180.0f;
    yaw = yawDegrees * M_PI / 180.0f;

	initVectorRotationRadians( matrix, roll, pitch, yaw );
}

void applyVectorRotation( vectorRotation_st * matrix, float *vectors )
{
	/*
		check out
		http://www.gamedev.net/page/resources/_/technical/math-and-physics/3d-matrix-math-demystified-r695
		to see how to do the vector rotation.
	*/
    float x = vectors[0];
    float y = vectors[1];
    float z = vectors[2];

    vectors[0] = matrix->rotationMatrix[0][0] * x + matrix->rotationMatrix[1][0] * y + matrix->rotationMatrix[2][0] * z;
    vectors[1] = matrix->rotationMatrix[0][1] * x + matrix->rotationMatrix[1][1] * y + matrix->rotationMatrix[2][1] * z;
    vectors[2] = matrix->rotationMatrix[0][2] * x + matrix->rotationMatrix[1][2] * y + matrix->rotationMatrix[2][2] * z;
}

void multiplyVectorMatrix( vectorRotation_st * MatrixA, vectorRotation_st * MatrixB, vectorRotation_st * NewMatrix )
{
	unsigned int i;

	/*
		check out
		http://www.gamedev.net/page/resources/_/technical/math-and-physics/3d-matrix-math-demystified-r695
	*/
	for(i = 0; i < 3; i++)
	{
		float x, y, z;
		/* using temporary variables allows the destination matrix to be the same as MatrixA */
		x = MatrixA->rotationMatrix[i][0] * MatrixB->rotationMatrix[0][0] + MatrixA->rotationMatrix[i][1] * MatrixB->rotationMatrix[1][0] + MatrixA->rotationMatrix[i][2] * MatrixB->rotationMatrix[2][0];
		y = MatrixA->rotationMatrix[i][0] * MatrixB->rotationMatrix[0][1] +	MatrixA->rotationMatrix[i][1] * MatrixB->rotationMatrix[1][1] + MatrixA->rotationMatrix[i][2] * MatrixB->rotationMatrix[2][1];
		z = MatrixA->rotationMatrix[i][0] * MatrixB->rotationMatrix[0][2] +	MatrixA->rotationMatrix[i][1] * MatrixB->rotationMatrix[1][2] + MatrixA->rotationMatrix[i][2] * MatrixB->rotationMatrix[2][2];

		NewMatrix->rotationMatrix[i][0] = x;
		NewMatrix->rotationMatrix[i][1] = y;
		NewMatrix->rotationMatrix[i][2] = z;
	}
}

void normaliseVector(float *src, float *dest)
{
    float length;

    length = sqrtf(src[0] * src[0] + src[1] * src[1] + src[2] * src[2]);
    if (length != 0.0f)
    {
        dest[0] = src[0] / length;
        dest[1] = src[1] / length;
        dest[2] = src[2] / length;
    }
}


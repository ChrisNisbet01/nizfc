#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <vector_rotation.h>

void initVectorRotation( vectorRotation_st *matrix, float rollDegrees, float pitchDegrees, float yawDegrees )
{
    float roll, pitch, yaw;
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    roll = rollDegrees * M_PI / 180.0f;
    pitch = pitchDegrees * M_PI / 180.0f;
    yaw = yawDegrees * M_PI / 180.0f;

    cosx = cosf(roll);
    sinx = sinf(roll);
    cosy = cosf(pitch);
    siny = sinf(pitch);
    cosz = cosf(yaw);
    sinz = sinf(yaw);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    // define rotation matrix
    matrix->rotationMatrix[0][0] = coszcosy;
    matrix->rotationMatrix[0][1] = -cosy * sinz;
    matrix->rotationMatrix[0][2] = siny;

    matrix->rotationMatrix[1][0] = sinzcosx + (coszsinx * siny);
    matrix->rotationMatrix[1][1] = coszcosx - (sinzsinx * siny);
    matrix->rotationMatrix[1][2] = -sinx * cosy;

    matrix->rotationMatrix[2][0] = (sinzsinx) - (coszcosx * siny);
    matrix->rotationMatrix[2][1] = (coszsinx) + (sinzcosx * siny);
    matrix->rotationMatrix[2][2] = cosy * cosx;
}

void applyVectorRotation( vectorRotation_st *matrix, float *vectors )
{
    float x = vectors[0];
    float y = vectors[1];
    float z = vectors[2];

    vectors[0] = matrix->rotationMatrix[0][0] * x + matrix->rotationMatrix[1][0] * y + matrix->rotationMatrix[2][0] * z;
    vectors[1] = matrix->rotationMatrix[0][1] * x + matrix->rotationMatrix[1][1] * y + matrix->rotationMatrix[2][1] * z;
    vectors[2] = matrix->rotationMatrix[0][2] * x + matrix->rotationMatrix[1][2] * y + matrix->rotationMatrix[2][2] * z;
}


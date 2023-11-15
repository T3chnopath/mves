#include <stdbool.h>
#include <string.h>
#include "imu.h"

static const float ACCEL_THRESHHOLD = 0.1;

// Bay IMU data
static float bayAccelY = 0;
static float bayAccelZ = 0;

// Update current bay IMU data
void IMU_Update(uint8_t * data)
{
    static bool IMU_ProcessThreadCreate = false;

    memcpy(&bayAccelY, data, sizeof(float));
    memcpy(&bayAccelZ, data + sizeof(float), sizeof(float));
}

// Return direction that bay is biased
BAY_DIR IMU_GetDirBias( void )
{
    // Ensure calculations apply to the same set of Y and Z,
    // So it is not interrupted by updates 
    float _bayAccelY = bayAccelY;
    float _bayAccelZ = bayAccelZ;

    /*
   CCW  |  CW
   +y   |  +y
   -z   |  +z
   -----------
   -y   |  +y
   +z   |  -z
    */

    // Quadrant 1
    if( _bayAccelZ > 0 && _bayAccelY > 0)
        return CW;

    // Quadrant 2
    else if ( _bayAccelZ < 0 && _bayAccelY > 0)
        return CCW;

    // Quadrant 3
    else if ( _bayAccelZ > 0 && _bayAccelY < 0)
        return CCW;

    // Quadrant 4, default case
    else
        return CW;
}

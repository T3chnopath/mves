#include <stdbool.h>
#include <string.h>
#include "imu.h"

static float accel[2][3]; // Two IMU rows, of three axis each

// Update current bay IMU data
void IMU_Update(IMU_TYPE type, uint8_t * data)
{
    static bool IMU_ProcessThreadCreate = false;
    
    if(type == BAY)
    {
        // data encoded as Y followed by Z
        memcpy(&accel[BAY][Y], data, sizeof(float));
        memcpy(&accel[BAY][Z], data + sizeof(float), sizeof(float));
    }
    else
    {
        // data encoded as just Z
        memcpy(&accel[ARM][Z], data, sizeof(float));
    }
}

// Return direction that bay is biased
IMU_BAY_DIR IMU_GetBayDir( void )
{
    /*
   CCW  |  CW
   +y   |  +y
   -z   |  +z
   -----------
   -y   |  +y
   +z   |  -z
    */

    // Quadrant 1
    // if( _bayAccelZ > 0 && _bayAccelY > 0)
    // {
    //     return CW;
    // }

    // // Quadrant 2
    // else if ( _bayAccelZ > 0 && _bayAccelY < 0)
    // {
    //     return CCW;
    // }

    // // Quadrant 3
    // else if ( _bayAccelZ < 0 && _bayAccelY < 0)
    // {
    //     return CCW;
    // }

    // // Quadrant 4, default case
    // else
    // {
    //     return CW;
    // }


    if( accel[BAY][Y] > 0) 
    {
        return CW;
    }

    else
    {
        return CCW;
    }
}

// Return true if axis is oriented upward
bool IMU_CheckAxisUpward(IMU_TYPE type, IMU_AXIS axis, float threshold)
{
    float axisAccel = accel[type][axis];

    if( axisAccel > (ACCEL_GRAV - threshold) && axisAccel < (ACCEL_GRAV + threshold))
    {
        return true;
    }

    return false;
}

// Return value of specified axis
float IMU_Get(IMU_TYPE type, IMU_AXIS  axis)
{
    return accel[type][axis];
}
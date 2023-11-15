#ifndef __IMU_H
#define __IMU_H

#include "stdint.h"

// Represents direction that by is oriented towards
typedef enum
{
    CW,
    CCW,
} BAY_DIR;

void IMU_Update(uint8_t * data);
BAY_DIR IMU_GetDirBias( void );

#endif
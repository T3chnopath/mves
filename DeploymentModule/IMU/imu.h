#ifndef __IMU_H
#define __IMU_H

#include "stdint.h"

typedef enum
{
    X,
    Y,
    Z,
} IMU_AXIS;

// Represents direction that by is oriented towards
typedef enum
{
    CW,
    CCW,
} IMU_BAY_DIR;

typedef enum
{
    BAY,
    ARM,
} IMU_TYPE;

static const float ACCEL_GRAV = 9.81;

void IMU_Update(IMU_TYPE type, uint8_t *data);
float IMU_Get(IMU_TYPE type, IMU_AXIS axis);
bool IMU_CheckAxisUpward(IMU_TYPE type, IMU_AXIS axis, float threshold);

IMU_BAY_DIR IMU_GetBayDir( void );

#endif
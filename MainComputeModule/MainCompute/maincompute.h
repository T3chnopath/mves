#ifndef __MAINCOMPUTE_H
#define __MAINCOMPUTE_H 

#include <stdbool.h>
#include <stdint.h>
#include "mcan.h"

bool IMU_Init(void);
void IMU_Update( uint8_t *sensorData );

#endif // __MAINCOMPUTE_H typedef enum
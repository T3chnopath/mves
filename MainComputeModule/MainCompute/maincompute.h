#ifndef __MAINCOMPUTE_H
#define __MAINCOMPUTE_H 

#include <stdbool.h>
#include <stdint.h>
#include "mcan.h"

bool IMU_SensorNodeInit( MCAN_DEV rxDevice, uint16_t periodMS);

#endif // __MAINCOMPUTE_H typedef enum
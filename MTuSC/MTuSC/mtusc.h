#ifndef __MTUSC_H
#define __MTUSC_H

#include <stdint.h>
#include <stdbool.h>


void MTuSC_ConsoleInit(void);
bool IMU_Init(void);
void IMU_Update( uint8_t *sensorData);

#endif 
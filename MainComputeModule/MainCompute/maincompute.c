#include "maincompute.h"
#include "bsp_maincompute.h"

#include <stdbool.h>
#include <string.h>
#include "bno055.h"
#include "mcan.h"
#include "tx_api.h"

extern I2C_HandleTypeDef MTuSC_I2C;
static BNO055_Axis_Vec_t BNO055_Vector;
static const uint16_t BNO055_DELAY_MS = 1;

// Initialize BN055 IMU
bool IMU_Init(void)
{
    /* Custom Axis */
    BNO055_AXIS_CONFIG_t axis_config = {.x = BNO055_Z_AXIS,
                                        .y = BNO055_Y_AXIS,
                                        .z = BNO055_X_AXIS};

    /* Required Boot-up Time for BNO055 */
    tx_thread_sleep(700);

    /* BNO055 Init */
    BNO055_I2C_Mount(&MTuSC_I2C);
    if (BNO055_Init() != BNO055_SUCCESS)
        return false;

    if (BNO055_Set_OP_Mode(NDOF) != BNO055_SUCCESS)
        return false;

    if (BNO055_Set_Axis(&axis_config) != BNO055_SUCCESS)
        return false;
}

void IMU_Update( uint8_t *sensorData)
{
    float y;
    float z;

    // Update gravity vector
    BNO055_Get_Gravity_Vec(&BNO055_Vector);

    // Delay
    tx_thread_sleep(BNO055_DELAY_MS);
    y = BNO055_Vector.y;
    z = BNO055_Vector.z;

    // Copy IMU data
    memcpy(sensorData, &y, sizeof(float));
    memcpy(sensorData + sizeof(float), &z, sizeof(float));
}
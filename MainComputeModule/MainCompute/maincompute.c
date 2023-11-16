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
static MCAN_DEV _rxDevice;


// IMU SensorNode Thread
#define THREAD_SENSOR_NODE_STACK_SIZE 2048
static TX_THREAD stThreadSensorNode;
static uint8_t auThreadSensorNodeStack[THREAD_SENSOR_NODE_STACK_SIZE];
void thread_sensor_node(ULONG periodMS);


// Initialize BN055 IMU
bool _BNO055_Init(void)
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

bool IMU_SensorNodeInit(MCAN_DEV rxDevice, uint16_t periodMS)
{
    _rxDevice = rxDevice;
    _BNO055_Init();

    tx_thread_create( &stThreadSensorNode, 
        "thread_sensor_node", 
        thread_sensor_node, 
        periodMS, 
        auThreadSensorNodeStack, 
        THREAD_SENSOR_NODE_STACK_SIZE, 
        3,
        3, 
        0,  
        TX_DONT_START);

    return true;
}

void IMU_SensorNodeEnable( void )
{
    tx_thread_resume(&stThreadSensorNode);
}

void IMU_SensorNodeDisable( void )
{
    tx_thread_delete(&stThreadSensorNode);
}

void thread_sensor_node(ULONG periodMS)
{
    uint8_t IMU_Data[8];
    float y;
    float z;

    while(true)
    {
        // // Create sum for IMU average
        // BNO055_Get_Gravity_Vec(&BNO055_Vector);
        // y_sum += BNO055_Vector.y;
        // z_sum += BNO055_Vector.z;
        // sample_size++;
        // tx_thread_sleep(BNO055_DELAY_MS);

        // // Every period, send averged IMU data
        // if( tx_time_get() % periodMS == 0)
        // {
        //     y_avg = (float) y_sum / sample_size;
        //     z_avg = (float) z_sum / sample_size;
        //     memcpy(IMU_Data, &y_avg, sizeof(float));
        //     memcpy(IMU_Data + sizeof(float), &z_avg, sizeof(float));
        //     MCAN_TX(MCAN_DEBUG, SENSOR_DATA, DEV_DEPLOYMENT, IMU_Data);
            
        //     sample_size = y_sum = y_avg = z_sum = z_avg = 0;
        // }

        tx_thread_sleep(500); 

        BNO055_Get_Gravity_Vec(&BNO055_Vector);
        tx_thread_sleep(1);
        y = BNO055_Vector.y;
        z = BNO055_Vector.z;

        memcpy(IMU_Data, &y, sizeof(float));
        memcpy(IMU_Data + sizeof(float), &z, sizeof(float));
        MCAN_TX(MCAN_DEBUG, SENSOR_DATA, DEV_DEPLOYMENT, IMU_Data);
        
        tx_thread_sleep(10);
    }
}
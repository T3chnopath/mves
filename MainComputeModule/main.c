#include "bsp_maincompute.h"
#include "tx_api.h"
#include "mcan.h"
#include "bno055.h"
#include "maincompute.h"

// Main Thread
#define THREAD_MAIN_STACK_SIZE 512
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];
static const uint16_t THREAD_MAIN_DELAY = 10;
void thread_main(ULONG ctx);

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 512
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];
void thread_blink(ULONG ctx);

static const uint16_t LED_BLINK_TIME = 1000;
static sMCAN_Message mcanRxMessage = { 0 };

int main(void)
{
    tx_kernel_enter();
}

void tx_application_define(void *first_unused_memory)
{
    tx_thread_create( &stThreadMain, 
        "thread_main", 
        thread_main, 
        0, 
        auThreadMainStack, 
        THREAD_MAIN_STACK_SIZE, 
        0,
        0, 
        0,  
        TX_AUTO_START);

    tx_thread_create( &stThreadBlink, 
        "thread_blink", 
        thread_blink, 
        0, 
        auThreadBlinkStack, 
        THREAD_BLINK_STACK_SIZE, 
        2,
        2, 
        0,
        TX_AUTO_START);
}

void thread_main(ULONG ctx)
{
    // Initialize BSP and App layer
    BSP_Init();
    
    MCAN_Init( FDCAN1, DEV_DEPLOYMENT, &mcanRxMessage );

    IMU_SensorNodeInit( DEV_DEPLOYMENT, 500);

    while(true)
    {
        tx_thread_sleep(THREAD_MAIN_DELAY);
    }
}

void thread_blink(ULONG ctx)
{
    while(true)
    {
        tx_thread_sleep(LED_BLINK_TIME);
        HAL_GPIO_TogglePin(LED0_GREEN_GPIO_Port, LED0_GREEN_Pin);
    }
}

// extern I2C_HandleTypeDef   MTuSC_I2C;
// BNO055_Axis_Vec_t          BNO055_Vector;

// void thread_main(ULONG ctx);

// typedef enum{
//     CCW,
//     CW
// } ORIENTATION_DIR;

// ORIENTATION_DIR initialDir = CW;
// float curr_reading = 0.0;

// void average(float* y_avg, float* z_avg, uint8_t sample_size){

//     *y_avg = 0;
//     *z_avg = 0;

//     for(uint8_t i = 0; i < sample_size; i++){
//         BNO055_Get_Gravity_Vec(&BNO055_Vector);
//         *y_avg += BNO055_Vector.y;
//         *z_avg += BNO055_Vector.z;
//         HAL_Delay(12);
//     }
//     *y_avg /= (float)sample_size;
//     *z_avg /= (float)sample_size;
// }

// ORIENTATION_DIR getDir(float y){
//     return (y > 0) ? CCW : CW;     
// }

// int main(void)
// {
//     /* Initialize BSP */
//     BSP_Init();

//     MCAN_Init( FDCAN1, DEV_MAIN_COMPUTE, &mcanRxMessage );

//     tx_kernel_enter();
//    }

// void tx_application_define(void *first_unused_memory)
// {
//     // Create main thread
//     tx_thread_create( &stThreadMain, 
//                      "thread_main", 
//                       thread_main, 
//                       0, 
//                       auThreadMainStack, 
//                       THREAD_MAIN_STACK_SIZE, 
//                       4,
//                       4, 
//                       0, // Time slicing unused if all threads have unique priorities     
//                       TX_AUTO_START);
// }

// void thread_main(ULONG ctx)
// {
//     MCAN_SetEnableIT(MCAN_ENABLE);

//     /* Custom Axis */
//     BNO055_AXIS_CONFIG_t axis_config = {.x = BNO055_Z_AXIS, 
//                                         .y = BNO055_Y_AXIS, 
//                                         .z = BNO055_X_AXIS};

//     /* Required Boot-up Time for BNO055 */
//     tx_thread_sleep(700);
    
//     /* BNO055 Init */
//     BNO055_I2C_Mount(&MTuSC_I2C);
//     if(BNO055_Init() != BNO055_SUCCESS)
//         while(1);
//     if(BNO055_Set_OP_Mode(NDOF) != BNO055_SUCCESS)
//         while(1);
//     if(BNO055_Set_Axis(&axis_config) != BNO055_SUCCESS)
//         while(1);

//     float y_avg = 0.0;
//     float z_avg = 0.0;

//     while(true)
//     {
    
//         average(&y_avg, &z_avg, 100);

//         initialDir = getDir(y_avg);

//         //Drive_Motor(dir);
//         HAL_GPIO_WritePin(LED0_GREEN_GPIO_Port, LED0_GREEN_Pin, GPIO_PIN_RESET);

//         curr_reading = 9.81 - z_avg;

//         while(curr_reading > 0.01 || curr_reading < -0.01) {
//             static uint8_t dirChangeCount = 0;

//             //Grab Gravity Vector from BNO055
//             BNO055_Get_Gravity_Vec(&BNO055_Vector);
            
//             //Threshold Calculation
//             curr_reading = 9.81 - BNO055_Vector.z;
//             if(getDir(BNO055_Vector.y) != initialDir) {
//                 dirChangeCount++;
//                 if (dirChangeCount > 7 && BNO055_Vector.z > 0) {
//                     break;
//                 }
//             }
//             else {
//                 dirChangeCount = 0;
//             }
//             tx_thread_sleep(100);
//         }
//         //Stop_Motor();
//         HAL_GPIO_WritePin(LED0_GREEN_GPIO_Port, LED0_GREEN_Pin, GPIO_PIN_SET);

//         while(1);

//     }
// }

// void MCAN_Rx_Handler( void )
// {
    
// }
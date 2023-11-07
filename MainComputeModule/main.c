#include "bsp_maincompute.h"
#include "tx_api.h"
#include "mcan.h"
#include "bno055.h"
#include "maincompute.h"

// Main Thread
#define THREAD_MAIN_STACK_SIZE 512
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];

static sMCAN_Message mcanRxMessage = { 0 };
static uint8_t heartbeatData[] = { 0xDE, 0xCA, 0XF, 0xC0, 0xFF, 0xEE, 0xCA, 0xFE};

extern I2C_HandleTypeDef   MTuSC_I2C;
BNO055_Axis_Vec_t          Euler_Vector;

void thread_main(ULONG ctx);

int main(void)
{
    /* Initialize BSP */
    BSP_Init();

    MCAN_Init( FDCAN1, DEV_MAIN_COMPUTE, &mcanRxMessage );

    tx_kernel_enter();
   }

void tx_application_define(void *first_unused_memory)
{
    // Create main thread
    tx_thread_create( &stThreadMain, 
                     "thread_main", 
                      thread_main, 
                      0, 
                      auThreadMainStack, 
                      THREAD_MAIN_STACK_SIZE, 
                      4,
                      4, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_AUTO_START);
}

void thread_main(ULONG ctx)
{
    MCAN_SetEnableIT(MCAN_ENABLE);

    /* Required Boot-up Time for BNO055 */
    tx_thread_sleep(700);
    
    /* BNO055 Init */
    BNO055_I2C_Mount(&MTuSC_I2C);
    if(BNO055_Init() != BNO055_SUCCESS)
        while(1);
    if(BNO055_Set_OP_Mode(NDOF) != BNO055_SUCCESS)
        while(1);

    while(true)
    {
    
        HAL_GPIO_TogglePin(LED0_RED_GPIO_Port, LED0_RED_Pin);
        BNO055_Get_Euler_Vec(&Euler_Vector);
        tx_thread_sleep(1000);

    }
}

void MCAN_Rx_Handler( void )
{
    
}
#include "bsp_deployment.h"
#include "tx_api.h"
#include "mcan.h"
#include "deployment.h"

// Main Thread
#define THREAD_MAIN_STACK_SIZE 512
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];
void thread_main(ULONG ctx);

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 256
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];
void thread_blink(ULONG ctx);

static sMCAN_Message mcanRxMessage = { 0 };
static const uint16_t LED_BLINK_TIME = 1000;

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
        10,
        10, 
        0,
        TX_AUTO_START);
}

void thread_main(ULONG ctx)
{
    // Initialize BSP and App layer
    BSP_Init();
    
    MCAN_Init( FDCAN1, DEV_DEPLOYMENT, &mcanRxMessage );

    DeploymentInit();
    
    while(true)
    {
        // If sensor data, update IMU
        if ( mcanRxMessage.mcanID.MCAN_CAT == SENSOR_DATA )
        {
            DeployUpdateSensorData( mcanRxMessage.mcanData );
        }

        // If first byte is 1, second byte is a deployment command
        else if ( mcanRxMessage.mcanData[0] == 1 )
        {
            DeployCommExe((DEPLOY_COMM) mcanRxMessage.mcanData[1]);
        }

        tx_thread_suspend(&stThreadMain);
    }
}

void thread_blink(ULONG ctx)
{
    while(true)
    {
        tx_thread_sleep(LED_BLINK_TIME);
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    }
}

void MCAN_Rx_Handler( void )
{
    tx_thread_resume(&stThreadMain);
}
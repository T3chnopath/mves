#include "bsp_deployment.h"
#include "tx_api.h"
#include "mcan.h"
#include "deployment.h"

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 256
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];
static sMCAN_Message mcanRxMessage = { 0 };

void thread_blink(ULONG ctx);

int main(void)
{
    tx_kernel_enter();
}

void tx_application_define(void *first_unused_memory)
{
    BSP_Init();
    
    MCAN_Init( FDCAN1, DEV_DEPLOYMENT, &mcanRxMessage );
    MCAN_SetEnableIT(MCAN_ENABLE);

    DeploymentInit();

    tx_thread_create( &stThreadBlink, 
        "thread_blink", 
        thread_blink, 
        0, 
        auThreadBlinkStack, 
        THREAD_BLINK_STACK_SIZE, 
        2,
        2, 
        0, // Time slicing unused if all threads have unique priorities     
        TX_AUTO_START);
}

void MCAN_Rx_Handler( void )
{
    uint32_t command = 0;
    // If sensor data, update IMU
    if ( mcanRxMessage.mcanID.MCAN_CAT == SENSOR_DATA )
    {
        DeployUpdateSensorData( mcanRxMessage.mcanData );
    }

    // If first byte is 1, second byte is a deployment command
    else if ( mcanRxMessage.mcanData[0] == 1 )
    {
        command = (DEPLOY_COMM) mcanRxMessage.mcanData[1];    
        DeployCommExe(command);
    }
}

void thread_blink(ULONG ctx)
{
    // Toggle LED once a second
    while(true)
    {
        tx_thread_sleep(1000);
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
}
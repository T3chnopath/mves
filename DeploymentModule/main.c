#include "bsp_deployment.h"
#include "tx_api.h"
#include "mcan.h"
#include "dc_motor.h"
#include "servo.c"
#include "deployment.h"

// Deploy Process Thread
#define THREAD_DEPLOY_PROCESS_STACK_SIZE 512
static TX_THREAD stThreadDeployProcess;
static uint8_t auThreadDeployProcessStack[THREAD_DEPLOY_PROCESS_STACK_SIZE];

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 128
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];


static sMCAN_Message mcanRxMessage = { 0 };

static volatile DEPLOY_PROCESS process = IDLE_PROC;

static const DEPLOY_COMM procFullDeployment[] = {DIRTBRAKE_DEPLOY, BAY_ORIENT, ARM_ORIENT, IDLE_COMM};
static const DEPLOY_COMM procFullRetraction[] = {ARM_RETRACT, DIRTBRAKE_RETRACT, IDLE_COMM};
static const DEPLOY_COMM procEmergencyStop[]  = {ARM_STOP, BAY_STOP, IDLE_COMM};

void thread_main(ULONG ctx);
void thread_deploy_process(ULONG ctx);
void thread_blink(ULONG ctx);

int main(void)
{
    tx_kernel_enter();
}

void tx_application_define(void *first_unused_memory)
{
    BSP_Init();

    MCAN_Init( FDCAN1, DEV_MAIN_COMPUTE, &mcanRxMessage );
    MCAN_SetEnableIT(MCAN_ENABLE);

    DeploymentInit();

    // Create high level deployment process thread
    tx_thread_create( &stThreadDeployProcess, 
                     "thread_deploy_process", 
                      thread_deploy_process, 
                      0, 
                      auThreadDeployProcessStack, 
                      THREAD_DEPLOY_PROCESS_STACK_SIZE, 
                      4,
                      4, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_AUTO_START);

    // Create blink thread
    tx_thread_create( &stThreadBlink, 
                     "thread_blink", 
                      thread_blink, 
                      0, 
                      auThreadBlinkStack, 
                      THREAD_BLINK_STACK_SIZE, 
                      15,
                      15, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_AUTO_START);
}

void thread_deploy_process(ULONG ctx)
{
    // Select initial command
    uint8_t procCommandIndex = 0;
    const DEPLOY_COMM * procCommandArr;
    DEPLOY_PROCESS currProcess = process;
    switch(currProcess)
    {
        case FULL_DEPLOYMENT:
            procCommandArr = procFullDeployment;
            break;

        case FULL_RETRACTION:
            procCommandArr = procFullRetraction;
            break;
        
        case EMERGENCY_STOP:
            procCommandArr = procEmergencyStop;
            break;
    }

    while(procCommandArr[procCommandIndex] != IDLE_COMM)
    {  
        if(DeployCommExe(procCommandArr[procCommandIndex]))
        {
            procCommandIndex++;
        }
    }

    tx_thread_suspend(&stThreadDeployProcess);
}

void thread_blink(ULONG ctx)
{
    // Toggle LED once a second
    if( (tx_time_get() % 1000) == 0 ){
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
}

void MCAN_Rx_Handler( void )
{
    DEPLOY_COMM command = IDLE_COMM;
    if ( mcanRxMessage.mcanID.MCAN_RX_Device == DEV_MAIN_COMPUTE || mcanRxMessage.mcanID.MCAN_RX_Device == DEV_ALL )
    {
        if ( mcanRxMessage.mcanData[7] != 0 )
        {
            process = EMERGENCY_STOP;
            tx_thread_terminate(&stThreadDeployProcess);
            
            tx_thread_create( &stThreadDeployProcess, 
                     "thread_deploy_process", 
                      thread_deploy_process, 
                      0, 
                      auThreadDeployProcessStack, 
                      THREAD_DEPLOY_PROCESS_STACK_SIZE, 
                      4,
                      4, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_AUTO_START);
        } 
        
        // If first byte is 1, second byte is a deployment process
        else if ( mcanRxMessage.mcanData[0] == 1 )
        {
            process = (DEPLOY_PROCESS) mcanRxMessage.mcanData[1];    
            tx_thread_resume(&stThreadDeployProcess);
        }

        // If first byte is 0, second byte is a deployment command
        else if ( mcanRxMessage.mcanData[0] == 0 )
        {
            command = (DEPLOY_COMM) mcanRxMessage.mcanData[1];
            DeployCommExe(command); 
        }
    } 
}
#include "bsp_deployment.h"
#include "tx_api.h"
#include "mcan.h"
#include "dc_motor.h"
#include "servo.c"
#include "deployment.h"

// Deploy Process Thread
#define THREAD_FULL_DEPLOY_STACK_SIZE 256
static TX_THREAD stThreadFullDeploy;
static uint8_t auThreadFullDeployStack[THREAD_FULL_DEPLOY_STACK_SIZE];

#define THREAD_FULL_RETRACT_STACK_SIZE 256
static TX_THREAD stThreadFullRectract;
static uint8_t auThreadFullRetractStack[THREAD_FULL_RETRACT_STACK_SIZE];

#define THREAD_ESTOP_STACK_SIZE 256
static TX_THREAD stThreadEStop;
static uint8_t auThreadEStopStack[THREAD_ESTOP_STACK_SIZE];

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 128
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];


static sMCAN_Message mcanRxMessage = { 0 };

static volatile DEPLOY_THREAD newThread = IDLE_THREAD;

void thread_estop(ULONG ctx);
void thread_full_deploy(ULONG ctx);
void thread_full_retract(ULONG ctx);
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

    // Create estop thread
    tx_thread_create( &stThreadEStop, 
                     "thread_estop", 
                      thread_estop, 
                      0, 
                      auThreadEStopStack, 
                      THREAD_ESTOP_STACK_SIZE, 
                      4,
                      4, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_DONT_START);

    // Create full deploy thread 
    tx_thread_create( &stThreadFullDeploy, 
                     "thread_full_deploy", 
                      thread_full_deploy, 
                      0, 
                      auThreadFullDeployStack, 
                      THREAD_FULL_DEPLOY_STACK_SIZE, 
                      5,
                      5, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_DONT_START);

    // Create full retract thread
    tx_thread_create( &stThreadFullRectract, 
                     "thread_full_retract", 
                      thread_full_retract, 
                      0, 
                      auThreadFullRetractStack, 
                      THREAD_FULL_RETRACT_STACK_SIZE, 
                      6,
                      6, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_DONT_START);

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


void thread_estop(ULONG ctx)
{
    uint8_t commArrIdx = 0;
    const DEPLOY_COMM EStopArr[]  = {ARM_STOP, BAY_STOP, IDLE_COMM};
    DEPLOY_COMM currCommand;

    do 
    {
        currCommand = EStopArr[commArrIdx];
        if(DeployCommExe(currCommand))
        {
            commArrIdx++;
        }

    } while(currCommand != IDLE_COMM);

    commArrIdx = 0; 
    tx_thread_suspend(&stThreadEStop);
}

void thread_full_deploy(ULONG ctx)
{
    uint8_t commArrIdx = 0;
    const DEPLOY_COMM FullDeployArr[] = {DIRTBRAKE_DEPLOY, BAY_ORIENT, ARM_ORIENT, IDLE_COMM};
    DEPLOY_COMM currCommand;
    DEPLOY_THREAD currThread = newThread;

    do 
    {
        currCommand = FullDeployArr[commArrIdx];
        if(DeployCommExe(currCommand))
        {
            commArrIdx++;
        }

        // exit the loop if there is a new process command
        if(currThread != newThread)
        {
            break;
        }
        
    } while(currCommand != IDLE_COMM);

    commArrIdx = 0; 
    tx_thread_suspend(&stThreadFullDeploy);
}

void thread_full_retract(ULONG CTX)
{
    uint8_t commArrIdx = 0;
    const DEPLOY_COMM FullRetractArr[] = {ARM_RETRACT, DIRTBRAKE_RETRACT, IDLE_COMM};
    DEPLOY_COMM currCommand;
    DEPLOY_THREAD currThread = newThread;

    do 
    {
        currCommand = FullRetractArr[commArrIdx];
        if(DeployCommExe(currCommand))
        {
            commArrIdx++;
        }

        // exit the loop if there is a new process command
        if(currThread != newThread)
        {
            break;
        }
        
    } while(currCommand != IDLE_COMM);

    commArrIdx = 0; 
    tx_thread_suspend(&stThreadFullRectract);
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
    if ( mcanRxMessage.mcanID.MCAN_RX_Device == DEV_DEPLOYMENT )
    {
        if ( mcanRxMessage.mcanData[7] != 0 )
        {
            newThread = ESTOP;
            tx_thread_resume( &stThreadEStop);     
        } 
        
        // If first byte is 1, second byte is a deployment thread
        else if ( mcanRxMessage.mcanData[0] == 1 )
        {
            newThread = (DEPLOY_THREAD) mcanRxMessage.mcanData[1];    

            switch(newThread)
            {
                case ESTOP:
                    tx_thread_resume( &stThreadEStop);
                    break;

                case FULL_DEPLOY:
                    tx_thread_resume( &stThreadFullDeploy);
                    break;

                case FULL_RETRACT:
                    tx_thread_resume( &stThreadFullRectract);
                    break;
            
                default:
                    break; 
            }
        }

        // If first byte is 0, second byte is a deployment command
        else if ( mcanRxMessage.mcanData[0] == 0 )
        {
            command = (DEPLOY_COMM) mcanRxMessage.mcanData[1];
            DeployCommExe(command); 
        }
    } 
}
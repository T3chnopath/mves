#include "bsp_deployment.h"
#include "tx_api.h"
#include "mcan.h"
#include "dc_motor.h"
#include "servo.c"
#include "deployment.h"

// Main Thread
#define THREAD_MAIN_STACK_SIZE 512
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];

static sMCAN_Message mcanRxMessage = { 0 };
static uint8_t heartbeatData[] = { 0xDE, 0xCA, 0XF, 0xC0, 0xFF, 0xEE, 0xCA, 0xFE};
static volatile DEPLOY_COMM command = IDLE;

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
    DeploymentInit();


    while(true)
    {
        // Toggle LED once a second
        if( (tx_time_get() % 1000) == 0 ){
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }

        switch(command)
        {
            case IDLE:
                break;

            case HEARTBEAT_START:
                MCAN_EnableHeartBeats(1000, heartbeatData);
                command = IDLE;
                break;

            case HEARTBEAT_STOP:
                MCAN_DisableHeartBeats();
                command = IDLE;
                break;

            case DIRTBRAKE_DEPLOY:
                DirtbrakeDeploy();
                command = IDLE;
                break;

            case DIRTBRAKE_RETRACT:
                DirtbrakeRetract();
                command = IDLE;
                break;

            case BAY_CW:
                BayCW();
                command = IDLE;
                break;

            case BAY_CCW:
                BayCCW(); 
                command = IDLE;
                break;

            case BAY_STOP:
                BayStop();
                command = IDLE;
                break;

            case ARM_DEPLOY:
                ArmDeploy();
                command = LS_DEPLOY;
                break;

            case LS_DEPLOY:
                // Poll and handle the arm deployment LS
                if(LS_DEPLOY());
                {
                    command = IDLE;
                }
                break;

            case ARM_RETRACT:
                ArmRetract();
                command = LS_RETRACT;
                break;

            case LS_RETRACT:
                // Poll and handle the arm retract LS 
                if(LS_RETRACT())
                {
                    command = IDLE;
                }
                break;

            case ARM_STOP:
                ArmStop();
                command = IDLE;
                break;

            case EMERGENCY_STOP:
                EmergencyStop();
                command = IDLE;
                break;

            default:
                break;
        }
    } 
}

void MCAN_Rx_Handler( void )
{
    if ( mcanRxMessage.mcanID.MCAN_RX_Device == DEV_MAIN_COMPUTE || mcanRxMessage.mcanID.MCAN_RX_Device == DEV_ALL )
    {
        command = (DEPLOY_COMM) mcanRxMessage.mcanData[0];

        if ( mcanRxMessage.mcanData[7] != 0 )
        {
            command = EMERGENCY_STOP;
        }
    } 
}
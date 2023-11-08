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

const static DEPLOY_COMM CommandsFullDeploy[] = {DIRTBRAKE_DEPLOY, BAY_ORIENT, ARM_DEPLOY, IDLE};
const static DEPLOY_COMM CommandsFullRetract[] = {ARM_RETRACT, DIRTBRAKE_DEPLOY, IDLE};

static volatile bool fullDeploy = false;
static volatile bool fullRetract = false;

void thread_main(ULONG ctx);

int main(void)
{
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
    BSP_Init();

    MCAN_Init( FDCAN1, DEV_MAIN_COMPUTE, &mcanRxMessage );
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

            // Heartbeat Commands
            case HEARTBEAT_START:
                MCAN_EnableHeartBeats(1000, heartbeatData);
                break;

            case HEARTBEAT_STOP:
                MCAN_DisableHeartBeats();
                break;

            // Dirtbrake Commands
            case DIRTBRAKE_DEPLOY:
                DirtbrakeDeploy();
                break;

            case DIRTBRAKE_RETRACT:
                DirtbrakeRetract();
                break;

            // Bay Commands
            case BAY_CW:
                BayCW();
                break;

            case BAY_CCW:
                BayCCW(); 
                break;

            case BAY_STOP:
                BayStop();
                break;

            case BAY_ORIENT:
                BayOrient();
                break;

            // Arm Commands
            case ARM_DEPLOY:
                ArmDeploy();
                break;

            case ARM_RETRACT:
                ArmRetract();
                break;

            case ARM_STOP:
                ArmStop();
                break;

            case ARM_ORIENT:
                ArmOrient();
                break;

            // E-Stop
            case EMERGENCY_STOP:
                EmergencyStop();
                break;

            default:
                break;
        }
    } 
}

void MCAN_Rx_Handler( void )
{
    DEPLOY_COMM receivedComm = IDLE;

    if ( mcanRxMessage.mcanID.MCAN_RX_Device == DEV_MAIN_COMPUTE || mcanRxMessage.mcanID.MCAN_RX_Device == DEV_ALL )
    {
        receivedComm = (DEPLOY_COMM) mcanRxMessage.mcanData[0];

        if ( mcanRxMessage.mcanData[7] != 0 )
        {
            command = EMERGENCY_STOP;
        }

        else if ( receivedComm == FULL_DEPLOYMENT )
        {
            
        }

        else if ( receivedComm = FULL_RETRACTION )
        {

        }

        else
        {
            command = receivedComm;
        }
    } 
}
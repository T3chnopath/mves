#include "deployment.h"
#include "bsp_deployment.h"

#include <stdbool.h>
#include "dc_motor.h"
#include "servo.h"
#include "mcan.h"
#include "stm32h5xx_hal.h"
#include "tx_api.h"

static DEPLOY_COMM currentCommand;

// Deploy Thread
#define THREAD_DEPLOY_STACK_SIZE 512
static TX_THREAD stThreadDeploy;
static uint8_t auThreadDeployStack[THREAD_DEPLOY_STACK_SIZE];

// Motor Variables
extern TIM_HandleTypeDef hACT_Tim;
static Actuator_Config_t actConfig;
static Actuator_Instance_t actInstance;
static const uint16_t LS_DELAY_MS = 1000;
static const uint16_t DIRTBRAKE_DELAY_MS = 3000;

// Limit Switch Variables
static bool ArmRetractLS_Pressed = false;
static bool ArmDeployLS_Pressed  = false;

// Bay IMU data
static float bayY = 0;
static float bayZ = 0;

void thread_deploy(ULONG ctx);

bool DeploymentInit(void)
{
    actConfig.Min_Pulse           = ACT_MIN_PULSE;
    actConfig.Max_Pulse           = ACT_MAX_PULSE;
    actConfig.Min_Length          = ACT_MIN_LEN;
    actConfig.Max_Length          = ACT_MAX_LEN;
    actConfig.Desired_Min_Length  = ACT_USER_MIN_LEN;
    actConfig.Desired_Max_Length  = ACT_USER_MAX_LEN;

    actInstance.Act_Timer   = &hACT_Tim;
    actInstance.Channel     = TIM_CHANNEL_2;
    actInstance.config      = &actConfig;

    if(Actuator_Init(&actInstance) != ACTUATOR_OK){
        return false;
    }

    // Create deployment thread
    tx_thread_create( &stThreadDeploy, 
                     "thread_deploy", 
                      thread_deploy, 
                      0, 
                      auThreadDeployStack, 
                      THREAD_DEPLOY_STACK_SIZE, 
                      2,
                      2, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_DONT_START);

    // Ensure all motors return to default state
    EStop();

    return true;
}

// DirtBrake Commands
void DirtbrakeDeploy(void)
{
    Drive_Actuator(&actInstance, ACT_USER_MAX_LEN);
    tx_thread_sleep(DIRTBRAKE_DELAY_MS);
}

void DirtbrakeRetract(void)
{
    Drive_Actuator(&actInstance, ACT_USER_MIN_LEN);
    tx_thread_sleep(DIRTBRAKE_DELAY_MS);
}

// Bay Commands
void BayCW(void)
{
#if defined(BAY_DC_FLIP)
    HAL_GPIO_WritePin(BAY_DC_Port1, BAY_DC_Pin1, SET);
    HAL_GPIO_WritePin(BAY_DC_Port2, BAY_DC_Pin2, RESET);
#else
    HAL_GPIO_WritePin(BAY_DC_Port1, BAY_DC_Pin1, RESET);
    HAL_GPIO_WritePin(BAY_DC_Port2, BAY_DC_Pin2, SET);
#endif
}

void BayCCW(void)
{
#if defined(BAY_DC_FLIP)
    HAL_GPIO_WritePin(BAY_DC_Port1, BAY_DC_Pin1, RESET);
    HAL_GPIO_WritePin(BAY_DC_Port2, BAY_DC_Pin2, SET);
#else
    HAL_GPIO_WritePin(BAY_DC_Port1, BAY_DC_Pin1, SET);
    HAL_GPIO_WritePin(BAY_DC_Port2, BAY_DC_Pin2, RESET);
#endif
}

void BayStop(void)
{
    HAL_GPIO_WritePin(BAY_DC_Port1, BAY_DC_Pin1, RESET);
    HAL_GPIO_WritePin(BAY_DC_Port2, BAY_DC_Pin2, RESET);
}

// BLOCKING
void BayOrient(void)
{
    // Set initial rotation
    if( true )
    {
        BayCW();
    }
    else if( true )
    {
        BayCCW();
    }

    tx_thread_sleep(1000);
    BayStop();

    // Terminate Rotation based on IMU
    while(false);
}

// Arm Commands
void ArmDeploy(void)
{
#if defined(ARM_DC_FLIP)
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, SET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, RESET);
#else
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, RESET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, SET);
#endif
}

void ArmDeployLS(void)
{
    ArmDeployLS_Pressed = true;
    if(HAL_GPIO_ReadPin(ARM_LS_DEPLOY_Port, ARM_LS_DEPLOY_Pin))
    {
        ArmRetract();
        HAL_Delay(LS_DELAY_MS);
        while(HAL_GPIO_ReadPin(ARM_LS_DEPLOY_Port, ARM_LS_DEPLOY_Pin));
        ArmStop(); 
    }
    ArmDeployLS_Pressed = false;
}

void ArmRetract(void)
{
#if defined(ARM_DC_FLIP)
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, RESET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, SET);
#else
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, SET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, RESET);
#endif
}

void ArmRetractLS(void)
{
    ArmRetractLS_Pressed = true;
    if(HAL_GPIO_ReadPin(ARM_LS_RETRACT_Port, ARM_LS_RETRACT_Pin))
    {
        ArmDeploy();
        HAL_Delay(LS_DELAY_MS);
        while(HAL_GPIO_ReadPin(ARM_LS_RETRACT_Port, ARM_LS_RETRACT_Pin));
        ArmStop();
    }
    ArmRetractLS_Pressed = false;
}

void ArmStop(void)
{
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, RESET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, RESET);
}

// BLOCKING
void ArmOrient(void)
{
    // TEMP rely on LS until IMU data is implemented
    ArmDeploy();

    // Wait for limit switch to be pressed
    while(!ArmRetractLS_Pressed);

    // Wait for limit switch to be unpressed
    while(ArmRetractLS_Pressed);

    return;
}

void EStop(void)
{
    ArmStop();
    BayStop();
}

void FullDeploy(void)
{
    DirtbrakeDeploy();
    BayOrient();
    ArmOrient();
}

void FullRetract(void)
{
    ArmRetract();
    DirtbrakeRetract();
}

void thread_deploy(ULONG ctx)
{
    while(true)
    {                
        switch( currentCommand )
        {
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
        
            case BAY_ORIENT:
                BayOrient();
                break;

            case BAY_STOP:
                BayStop();
                break;

            // Arm Commands
            case ARM_DEPLOY:
                ArmDeploy();
                break;

            case ARM_RETRACT:
                ArmRetract(); 
                break;

            case ARM_ORIENT:
                ArmOrient();
                break;

            case ARM_STOP:
                ArmStop();
                break;

            // Full commands
            case FULL_DEPLOY:
                FullDeploy();
                break;

            case FULL_RETRACT:
                FullRetract();
                break;

            case ESTOP:
                EStop();
                break;

            default:
                break;
        }

        tx_thread_suspend(&stThreadDeploy);
    }
}

void DeployCommExe(DEPLOY_COMM command)
{
    UINT deployThreadStatus;

    // Set current command for module
    currentCommand = command;
    
    // Check if thread is suspended
    if (tx_thread_info_get(&stThreadDeploy, TX_NULL, &deployThreadStatus, TX_NULL, TX_NULL, TX_NULL, TX_NULL, TX_NULL, NULL) == TX_SUCCESS)
    {
        // If thread is in the middle of execution, recreate it
        if( deployThreadStatus != TX_SUSPENDED )
        {
            // Teminate thread
            tx_thread_terminate(&stThreadDeploy);
            
            // Stop all motors
            EStop(); 

            // Recreate thread
            tx_thread_create( &stThreadDeploy, 
                     "thread_deploy", 
                      thread_deploy, 
                      0, 
                      auThreadDeployStack, 
                      THREAD_DEPLOY_STACK_SIZE, 
                      2,
                      2, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_DONT_START);
        }

        // If thread is loitering, resume
        else
        {
            tx_thread_resume( &stThreadDeploy);
        }
    }    
}

bool DeployUpdateSensorData(uint8_t * data)
{
    bayY = (float) data[0];
    bayZ = (float) data[4];
}
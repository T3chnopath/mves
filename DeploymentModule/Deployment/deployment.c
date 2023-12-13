#include "deployment.h"
#include "bsp_deployment.h"

#include <stdbool.h>
#include "dc_motor.h"
#include "servo.h"
#include "mcan.h"
#include "imu.h"
#include "stm32h5xx_hal.h"
#include "tx_api.h"

static DEPLOY_COMM currentCommand = IDLE;

// Deploy Thread
#define THREAD_DEPLOY_STACK_SIZE 2048
static TX_THREAD stThreadDeploy;
static uint8_t auThreadDeployStack[THREAD_DEPLOY_STACK_SIZE];
static const uint16_t THREAD_DEPLOY_DELAY_MS = 10;
static const uint16_t SENSOR_NODE_EN_DELAY_MS = 5000;
void thread_deploy(ULONG ctx);

// Bay Orientation Thread
#define THREAD_BAY_ORIENT_STACK_SIZE 1024
static TX_THREAD stThreadBayOrient;
static uint8_t auThreadBayOrientStack[THREAD_BAY_ORIENT_STACK_SIZE];
static const uint16_t THREAD_BAY_ORIENT_DELAY_MS = 1;
void thread_bay_orient(ULONG ctx);
static const float GRAV_THRESHOLD = 0.1;

// Arm Orientation Thread
#define THREAD_ARM_ORIENT_STACK_SIZE 1024
static TX_THREAD stThreadArmOrient;
static uint8_t auThreadArmOrientStack[THREAD_ARM_ORIENT_STACK_SIZE];
static const uint16_t THREAD_ARM_ORIENT_DELAY_MS = 1;
void thread_arm_orient(ULONG ctx);

// Motor Variables
extern TIM_HandleTypeDef hACT_Tim;
static Actuator_Config_t actConfig;
static Actuator_Instance_t actInstance;
static const uint16_t LS_DELAY_MS = 5000;
static const uint16_t DIRTBRAKE_DELAY_MS = 5000;

// Limit Switch Variables
static volatile bool ArmRetractLS_Handled = false;
static volatile bool ArmDeployLS_Handled  = false;

static volatile bool deployBusy = false;
static volatile bool eStop = false;
static const uint16_t ESTOP_DELAY = 3000;

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
                      TX_AUTO_START);

    // Create Bay Orient Thread
    tx_thread_create( &stThreadBayOrient, 
                     "thread_bay_orient", 
                      thread_bay_orient, 
                      0, 
                      auThreadBayOrientStack, 
                      THREAD_BAY_ORIENT_STACK_SIZE, 
                      3,
                      3, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_DONT_START);

    // Create Arm Orient Thread
    tx_thread_create( &stThreadArmOrient, 
                     "thread_arm_orient", 
                      thread_arm_orient, 
                      0, 
                      auThreadArmOrientStack, 
                      THREAD_ARM_ORIENT_STACK_SIZE, 
                      4,
                      4, 
                      0, // Time slicing unused if all threads have unique priorities     
                      TX_DONT_START);

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

void BayOrient(void)
{
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
    if(HAL_GPIO_ReadPin(ARM_LS_DEPLOY_Port, ARM_LS_DEPLOY_Pin))
    {
        ArmRetract();
        HAL_Delay(LS_DELAY_MS);
        while(HAL_GPIO_ReadPin(ARM_LS_DEPLOY_Port, ARM_LS_DEPLOY_Pin));
        ArmStop(); 
    }
    ArmDeployLS_Handled = true;
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
    if(HAL_GPIO_ReadPin(ARM_LS_RETRACT_Port, ARM_LS_RETRACT_Pin))
    {
        ArmDeploy();
        HAL_Delay(LS_DELAY_MS);
        while(HAL_GPIO_ReadPin(ARM_LS_RETRACT_Port, ARM_LS_RETRACT_Pin));
        ArmStop();
    }
    ArmRetractLS_Handled = true;
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

    // // Wait for limit switch to be pressed
    // while(!ArmRetractLS_Pressed);

    // // Wait for limit switch to be unpressed
    // while(ArmRetractLS_Pressed);

    return;
}

void EStop(void)
{
    eStop = true;

    ArmStop();
    BayStop();

    tx_thread_sleep(ESTOP_DELAY);
    eStop = false;
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
    while(!ArmRetractLS_Handled);
    ArmRetractLS_Handled = false;
    DirtbrakeRetract();
}

bool DeployCommExe(DEPLOY_COMM command)
{
    // Process estop
    if(command == ESTOP)
    {
        EStop();
        return true;
    }
    
    // If busy, return
    else if(deployBusy)
    {
        return false;
    }
    
    // Set current command for module
    currentCommand = command;
    return true;
}

void thread_deploy(ULONG ctx)
{
    while(true)
    {             
        if( !deployBusy )
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
                    tx_thread_resume(&stThreadBayOrient);
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
                    tx_thread_resume(&stThreadArmOrient);
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

                case IDLE:
                    break;

                default:
                    break;
            }

        deployBusy = false; 
        }
    
    tx_thread_sleep(THREAD_DEPLOY_DELAY_MS);
    }
}

void thread_bay_orient(ULONG ctx)
{
    while(true)
    {
    
        // Get initial direction
        BAY_DIR initialDir = IMU_GetDirBias();
        switch(initialDir)
        {
            case CW:
                BayCCW();
                break;
            
            case CCW:
                BayCW();
                break;
        }

        // Terminate if Estop
        while(!eStop)
        {
            tx_thread_sleep(THREAD_BAY_ORIENT_DELAY_MS);

            // Terminate if 1G in Z axis
            if(IMU_CheckZ(GRAV_THRESHOLD))
                break;

            // Terminate if change in direction
            if(initialDir != IMU_GetDirBias())
                break;
        }

        BayStop();

        tx_thread_suspend(&stThreadBayOrient);
    }
}

void thread_arm_orient(ULONG ctx)
{
    
}
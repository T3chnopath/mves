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
#define THREAD_DEPLOY_STACK_SIZE 512
static TX_THREAD stThreadDeploy;
static uint8_t auThreadDeployStack[THREAD_DEPLOY_STACK_SIZE];
static const uint16_t THREAD_DEPLOY_DELAY_MS = 10;
static const uint16_t SENSOR_NODE_EN_DELAY_MS = 5000;

// Motor Variables
extern TIM_HandleTypeDef hACT_Tim;
static Actuator_Config_t actConfig;
static Actuator_Instance_t actInstance;
static const uint16_t LS_DELAY_MS = 5000;
static const uint16_t DIRTBRAKE_DELAY_MS = 5000;

// Limit Switch Variables
static bool ArmRetractLS_Pressed = false;
static bool ArmDeployLS_Pressed  = false;

static volatile bool newCommand = false;

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
                      TX_AUTO_START);

    // Request sensor node IMU data
    tx_thread_sleep(SENSOR_NODE_EN_DELAY_MS);
    MCAN_TX(MCAN_DEBUG, SENSOR_DATA, DEV_MAIN_COMPUTE, (uint8_t[8]) {1, 0, 0, 0, 0, 0, 0, 0});

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

    // Rotate until direction changes
    while(initialDir == IMU_GetDirBias());
    BayStop();
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
        if( newCommand )
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

                case IDLE:
                    break;

                default:
                    break;
            }

        newCommand = false;
        }
    
    tx_thread_sleep(THREAD_DEPLOY_DELAY_MS);
    }
}

void DeployCommExe(DEPLOY_COMM command)
{
    // Set current command for module
    currentCommand = command;
    newCommand = true;
}

// return true if busy
bool DeployCommBusy(void)
{
    // If still processing, newCommand is true
    return newCommand;
}
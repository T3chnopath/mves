#include "deployment.h"
#include "bsp_deployment.h"

#include <stdbool.h>
#include "dc_motor.h"
#include "servo.h"
#include "mcan.h"
#include "stm32h5xx_hal.h"
#include "tx_api.h"

extern TIM_HandleTypeDef hACT_Tim;
static Actuator_Config_t actConfig;
static Actuator_Instance_t actInstance;
static const uint16_t LS_DELAY_MS = 1000;
static const uint16_t DIRTBRAKE_DELAY_MS = 3000;

static const uint8_t deployCommsIndexMax = 2;
static const uint8_t retractCommsIndexMax = 1;

bool _LS_Deploy(void)
{
    if(HAL_GPIO_ReadPin(ARM_LS_DEPLOY_Port, ARM_LS_DEPLOY_Pin))
    {
        ArmRetract();
        tx_thread_sleep(LS_DELAY_MS);
        while(HAL_GPIO_ReadPin(ARM_LS_DEPLOY_Port, ARM_LS_DEPLOY_Pin));
        ArmStop(); 

        return true;
    }

    return false;
}

bool _LS_Retract(void)
{
    if(HAL_GPIO_ReadPin(ARM_LS_RETRACT_Port, ARM_LS_RETRACT_Pin))
    {
        ArmDeploy();
        tx_thread_sleep(LS_DELAY_MS);
        while(HAL_GPIO_ReadPin(ARM_LS_RETRACT_Port, ARM_LS_RETRACT_Pin));
        ArmStop();

        return true;
    }

    return false;
}


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

bool BayOrient(void)
{
    // if bay is not oriented, stay in bay orient 
    BayCCW();
    tx_thread_sleep(5000);
    return true;
}

// Arm Commands
bool ArmDeploy(void)
{
    // return true if limit switch pressed
    if(_LS_Deploy())
    {
        return true;
    }

#if defined(ARM_DC_FLIP)
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, SET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, RESET);
#else
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, RESET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, SET);
#endif
    
    return false;
}

bool ArmRetract(void)
{
    // return true if limit switch pressed
    if(_LS_Retract())
    {
        return true;
    }

#if defined(ARM_DC_FLIP)
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, RESET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, SET);
#else
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, SET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, RESET);
#endif
    
    return false;
}

void ArmStop(void)
{
    HAL_GPIO_WritePin(ARM_DC_Port1, ARM_DC_Pin1, RESET);
    HAL_GPIO_WritePin(ARM_DC_Port2, ARM_DC_Pin2, RESET);
}

bool ArmOrient(void)
{
    // TEMP: just deploy the arm relying on limit switches
    while(!ArmDeploy());
    
    return true;
}

// return true if moving to next command state
bool DeployCommExe(DEPLOY_COMM deployComm)
{
    uint8_t heartbeatData[] = { 0xDE, 0xCA, 0XF, 0xC0, 0xFF, 0xEE, 0xCA, 0xFE};

    switch( deployComm )
    {
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
            // If bay is not oriented, stay in bay orient
            if(!BayOrient())
            {
                return false;
            }
            break;

        // Arm Commands
        case ARM_DEPLOY:
            ArmDeploy();
            break;

        case ARM_RETRACT:
            // If arm is not retracted, stay in arm retract
            if(!ArmRetract())
            {
                return false;
            }
            break;

        case ARM_STOP:
            ArmStop();
            break;

        case ARM_ORIENT:
            // If arm is not oriented, stay in arm orient 
            if(!ArmOrient())
            {
                return false;
            }
            break;

        default:
            break;
    }

    return true;
}


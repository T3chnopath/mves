#ifndef __DEPLOYMENT_H
#define __DEPLOYMENT_H 

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DIRTBRAKE_DEPLOY,
    DIRTBRAKE_RETRACT,  

    BAY_CW,
    BAY_CCW,
    BAY_STOP,
    BAY_ORIENT,

    ARM_DEPLOY,
    ARM_RETRACT,
    ARM_STOP,
    ARM_ORIENT,
    
    FULL_DEPLOY,
    FULL_RETRACT,
    ESTOP,

    IDLE_COMM,

} DEPLOY_COMM;

bool DeploymentInit(void);
void DirtbrakeDeploy(void);
void DirtbrakeRetract(void);

void BayCW(void);
void BayCCW(void);
void BayStop(void);
void BayOrient(void);

void ArmDeploy(void);
void ArmDeployLS(void);
void ArmRetract(void);
void ArmRetractLS(void);
void ArmStop(void);
void ArmOrient(void);

void FullDeploy(void);
void FullRetract(void);
void EStop(void);

void DeployCommExe(DEPLOY_COMM command);
bool DeployUpdateSensorData(uint8_t * data);

#endif // __DEPLOYMENT_H typedef enum
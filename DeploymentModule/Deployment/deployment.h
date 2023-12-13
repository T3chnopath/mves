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
    BAY_ORIENT,
    BAY_STOP,
    
    ARM_DEPLOY,
    ARM_RETRACT,
    ARM_ORIENT,
    ARM_STOP,
        
    FULL_DEPLOY,
    FULL_RETRACT,
    ESTOP,

    IDLE,

} DEPLOY_COMM;

bool DeploymentInit(void);
void DirtbrakeDeploy(void);
void DirtbrakeRetract(void);

void BayCW(void);
void BayCCW(void);
void BayOrient(void);
void BayStop(void);

void ArmDeploy(void);
void ArmDeployLS(void);
void ArmRetract(void);
void ArmRetractLS(void);
void ArmOrient(void);
void ArmStop(void);

void FullDeploy(void);
void FullRetract(void);
void EStop(void);

bool DeployCommExe(DEPLOY_COMM command);

#endif // __DEPLOYMENT_H typedef enum
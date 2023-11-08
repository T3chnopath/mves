#ifndef __DEPLOYMENT_H
#define __DEPLOYMENT_H 

#include <stdbool.h>

typedef enum
{
    IDLE,
    HEARTBEAT_START,
    HEARTBEAT_STOP,

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

    FULL_DEPLOYMENT,
    FULL_RETRACTION,
    EMERGENCY_STOP
} DEPLOY_COMM;

bool DeploymentInit(void);
void DirtbrakeDeploy(void);
void DirtbrakeRetract(void);

void BayCW(void);
void BayCCW(void);
void BayStop(void);
void BayOrient(void);

void ArmDeploy(void);
void ArmRetract(void);
void ArmStop(void);
void ArmOrient(void);

void EmergencyStop(void);

#endif // __DEPLOYMENT_H typedef enum
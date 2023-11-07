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
    ARM_DEPLOY,
    LS_DEPLOY,
    ARM_RETRACT,
    LS_RETRACT,
    ARM_STOP,
    EMERGENCY_STOP
} DEPLOY_COMM;

bool DeploymentInit(void);
void DirtbrakeDeploy(void);
void DirtbrakeRetract(void);
void BayCW(void);
void BayCCW(void);
void BayStop(void);
bool ArmDeploy(void);
bool ArmRetract(void);
void ArmStop(void);
void EmergencyStop(void);

#endif // __DEPLOYMENT_H typedef enum
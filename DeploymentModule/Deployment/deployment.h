#ifndef __DEPLOYMENT_H
#define __DEPLOYMENT_H 

#include <stdbool.h>

typedef enum
{
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

    IDLE_COMM,

} DEPLOY_COMM;

typedef enum
{
    ESTOP,
    FULL_DEPLOY,
    FULL_RETRACT,
    IDLE_THREAD,
} DEPLOY_THREAD;

bool DeploymentInit(void);
void DirtbrakeDeploy(void);
void DirtbrakeRetract(void);

void BayCW(void);
void BayCCW(void);
void BayStop(void);
bool BayOrient(void);

bool ArmDeploy(void);
bool ArmRetract(void);
void ArmStop(void);
bool ArmOrient(void);

bool DeployCommExe(DEPLOY_COMM deployComm);

#endif // __DEPLOYMENT_H typedef enum
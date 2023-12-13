#include "mtusc.h"
#include "servo.h"
#include "console.h"
#include "bsp_mtusc.h"
#include "mcan.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef  ConsoleUart;
extern TIM_HandleTypeDef   hServo_Tim;
extern TIM_HandleTypeDef   hFeedback_Tim;

// Static variables
static CONT_Servo_Config_t cServoConfig;
static CONT_Servo_Instance_t cServoInstance;

static const uint32_t SERVO_FEEDBACK_FREQ = 910;
static const int16_t  SERVO_MIN_ANGLE = 0;
static const int16_t  SERVO_MAX_ANGLE = 360;
static const float    SERVO_FEEDBACK_MIN_DUTY = 0.029;
static const float    SERVO_FEEDBACK_MAX_DUTY = 0.971;

#define MAX_DEPLOY_COMMANDS 16
#define MAX_COMM_LEN 30

static const char *deployComms[MAX_DEPLOY_COMMANDS] =
{
    "DirtbrakeDeploy",
    "DirtbrakeRetract", 

    "BayCW",
    "BayCCW",
    "BayOrient",
    "BayStop",

    "ArmDeploy",
    "ArmRetract",
    "ArmOrient",
    "ArmStop",

    "FullDeploy",
    "FullRetract",

    "ActiveGimbalEnable",
    "ActiveGimbalDisable",

    "EStop",
    "Idle",
};

// Static function declarations 
static void _rotateServo(char *argv[]);
static void _deployment(char *argv[]);

// Rotate Servo Command
ConsoleComm_t _commRotateServo = {
    "RotateServo",
    "Rotate a servo X degrees, CW or CCW",
    3,
    _rotateServo,
};

ConsoleComm_t _commDeployment = {
    "Deployment",
    "Execute deployment commands",
    1,
    _deployment,
};


// Static Functions
static Servo_Error _initServo(void)
{
    cServoConfig.feedbackFreq = SERVO_FEEDBACK_FREQ;
    cServoConfig.minAngle = SERVO_MIN_ANGLE;
    cServoConfig.maxAngle = SERVO_MAX_ANGLE;
    cServoConfig.feedbackMinDuty = SERVO_FEEDBACK_MIN_DUTY;
    cServoConfig.feedbackMaxDuty = SERVO_FEEDBACK_MAX_DUTY;

    cServoInstance.contServoTimer = &hServo_Tim;
    cServoInstance.contServoChannel = CSERVO_TIM_CHANNEL;
    cServoInstance.contServoConfig = &cServoConfig;
    cServoInstance.ICTimer = &hFeedback_Tim;
    cServoInstance.ICTimerChannel = FEEDBACK_IC_CHANNEL;

    return CONT_Servo_Init(&cServoInstance);
}

static void _rotateServo(char *argv[])
{
    CSERVO_DIR servoDir; 
    static bool firstCall = true;
    int16_t angle = atoi(argv[1]);

    // Verify valid direction
    if( strcmp(argv[2], "CW") == 0)
    {
        servoDir = SERVO_CLOCKWISE;
    }  
    else if (strcmp(argv[2], "CCW") == 0)
    {
        servoDir = SERVO_COUNTERCLOCKWISE;
    }
    else
    {
        ConsolePrint("Direction must be CW or CCW! \r\n");
        return;
    }

    // Verify valid angle
    if( angle < SERVO_MIN_ANGLE)
    {   
        ConsolePrint("%i is less than %d min angle! \r\n", angle, SERVO_MIN_ANGLE);
    }
    else if ( angle > SERVO_MAX_ANGLE )
    {
        ConsolePrint("%i is greater than %d max angle! \r\n", angle, SERVO_MAX_ANGLE);
    }

    // Init servo if first time calling this function
    if(firstCall)
    {
        if( _initServo() == SERVO_OK )
        {
            ConsolePrint("Servo Initialized! \r\n");
        }
        else
        {
            ConsolePrint("Servo failed to initialize! \r\n");
            return;
        }

    }
    
    // Convert degree argument and execute
    Drive_CONT_Servo_Angle(&cServoInstance, (int16_t) angle, servoDir);
    ConsolePrint("Servo rotated to %i degrees \r\n", (int16_t) angle);
}

static void _deployment(char *argv[])
{
    char inputBuffer[MAX_COMM_LEN] = {0};
    uint8_t mcanTxData[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    bool foundCommand = false;
    uint8_t commandIndex = 0;

    // Print submenu
    ConsolePrint("Select Deployment Commands: \r\n");
    for(uint8_t i = 0; i < MAX_DEPLOY_COMMANDS; i++)
    {
        ConsolePrint("  %s \r\n", deployComms[i]);
    }

    // Detect input command
    ConsoleInString(inputBuffer, MAX_COMM_LEN);                             

    // Find command 
    for(commandIndex = 0; commandIndex < MAX_DEPLOY_COMMANDS;)
    {
        if(strcmp(deployComms[commandIndex], inputBuffer) == 0)
        {
            foundCommand = true;
            break;
        }
        else
        {
            // Only add to index if command not found
            commandIndex++;
        }
    }  

    // Exit if command not found
    if(!foundCommand)
    {
        ConsolePrint("Command %s not found! \r\n", inputBuffer);
        return;
    }
    else 
    {
        // Set commandIndex in second element of data
        mcanTxData[1] = commandIndex;
        MCAN_TX(PRI_EMERGENCY, CAT_COMMAND, DEV_COMPUTE, mcanTxData);
        ConsolePrint("Executing command %s", deployComms[commandIndex]);
    }
}


void MTuSC_ConsoleInit(void)
{
    ConsoleRegisterComm(&_commRotateServo);
    ConsoleRegisterComm(&_commDeployment);
    ConsoleInit(&ConsoleUart);
}

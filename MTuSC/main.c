#include <stdbool.h>
#include "bsp_mtusc.h"
#include "mcan.h"
#include "bno055.h"
#include "stm32h5xx_hal.h"
#include "console.h"
#include "servo.h"
#include "tx_api.h"

extern UART_HandleTypeDef  ConsoleUart;
extern TIM_HandleTypeDef   hServo_Tim;
extern TIM_HandleTypeDef   hFeedback_Tim;

// Static variables
static BNO055_Axis_Vec_t BNO055_Vector;
static CONT_Servo_Config_t cServoConfig;
static CONT_Servo_Instance_t cServoInstance;

// Main Thread
#define THREAD_MAIN_STACK_SIZE 1024
static const uint16_t THREAD_MAIN_DELAY_MS = 10;
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];
void thread_main(ULONG ctx);

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 512
static const uint16_t LED_BLINK_TIME_MS = 1000;
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];
void thread_blink(ULONG ctx);

extern UART_HandleTypeDef ConsoleUart;

int main(void)
{
    tx_kernel_enter();
}

void tx_application_define(void *first_unused_memory)
{
    tx_thread_create( &stThreadMain, 
        "thread_main", 
        thread_main, 
        0, 
        auThreadMainStack, 
        THREAD_MAIN_STACK_SIZE, 
        0,
        0, 
        0,  
        TX_AUTO_START);
    
    tx_thread_create( &stThreadBlink, 
        "thread_blink", 
        thread_blink, 
        0, 
        auThreadBlinkStack, 
        THREAD_BLINK_STACK_SIZE, 
        10,
        10, 
        0,
        TX_AUTO_START);
}

void thread_main(ULONG ctx)
{
    BSP_Init();
    MCAN_Init( FDCAN1, DEV_ALL, MCAN_ENABLE);
    
    ConsoleInit(&ConsoleUart);

    cServoConfig.feedbackFreq = 910;
    cServoConfig.minAngle = 0;
    cServoConfig.maxAngle = 360;
    cServoConfig.feedbackMinDuty = 0.029;
    cServoConfig.feedbackMaxDuty = 0.971;

    cServoInstance.contServoTimer = &hServo_Tim;
    cServoInstance.contServoChannel = CSERVO_TIM_CHANNEL;
    cServoInstance.contServoConfig = &cServoConfig;
    cServoInstance.ICTimer = &hFeedback_Tim;
    cServoInstance.ICTimerChannel = FEEDBACK_IC_CHANNEL;

    CONT_Servo_Init(&cServoInstance);

    while(true)
    {
        Drive_CONT_Servo_Angle(&cServoInstance, 60, SERVO_CLOCKWISE);
        _tx_thread_sleep(1000);
        Drive_CONT_Servo_Angle(&cServoInstance, 60, SERVO_COUNTERCLOCKWISE);
        _tx_thread_sleep(1000);
        Drive_CONT_Servo_Angle(&cServoInstance, 60, SERVO_COUNTERCLOCKWISE);
        _tx_thread_sleep(1000);
        Drive_CONT_Servo_Angle(&cServoInstance, 60, SERVO_CLOCKWISE);
        _tx_thread_sleep(1000);
        tx_thread_sleep(THREAD_MAIN_DELAY_MS);
    }
}

void thread_blink(ULONG ctx)
{
    while(true)
    {
        tx_thread_sleep(LED_BLINK_TIME_MS);
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    }
}

void MCAN_RxHandler( sMCAN_Message mcanRxMessage)
{
    tx_thread_sleep(500);
}

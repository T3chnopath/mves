#include "bsp_maincompute.h"
#include "tx_api.h"
#include "mcan.h"
#include "bno055.h"
#include "maincompute.h"
#include "sensor_nodes.h"
#include "console.h"

// Externs
extern UART_HandleTypeDef  MIO_UART;

// Main Thread
#define THREAD_MAIN_STACK_SIZE 1024
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];
static const uint16_t THREAD_MAIN_DELAY_MS = 10;
static char MIO_Sequence[] = {'r', 'f', 'r', 'd', 'r', 'w', 'r', 'q'};
void thread_main(ULONG ctx);

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 512
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];
void thread_blink(ULONG ctx);

static const uint16_t IMU_UPDATE_PERIOD_MS = 250;
static const uint16_t LED_BLINK_TIME_MS = 1000;
static sMCAN_Message mcanRxMessage = { 0 };

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
        2,
        2, 
        0,
        TX_AUTO_START);
}

void thread_main(ULONG ctx)
{
    // Initialize BSP
    BSP_Init();
    
    // Initialize App Layer
    MCAN_Init( FDCAN1, DEV_COMPUTE, MCAN_ENABLE );
    IMU_Init();
    ConsoleInit(&MIO_UART);

    // Register node that sends out IMU data over CAN every 250 ms
    SensorNodeRegister( DEV_ALL, IMU_UPDATE_PERIOD_MS, IMU_Update, SENSOR_NODE_ENABLE);
    
    while(true)
    {
        for(uint8_t i = 0; i < sizeof(MIO_Sequence); i++){
            ConsolePrint(&MIO_Sequence[i]);
            tx_thread_sleep(5000);
        }
        tx_thread_sleep(THREAD_MAIN_DELAY_MS);
    }
}

void thread_blink(ULONG ctx)
{
    while(true)
    {
        tx_thread_sleep(LED_BLINK_TIME_MS);
        HAL_GPIO_TogglePin(LED0_GREEN_GPIO_Port, LED0_GREEN_Pin);
    }
}

void MCAN_Rx_Handler( void )
{   

}
#include <stdbool.h>
#include "bsp_mtusc.h"
#include "mcan.h"
#include "bno055.h"
#include "stm32h5xx_hal.h"
#include "tx_api.h"

static sMCAN_Message mcanRxMessage;
static BNO055_Axis_Vec_t BNO055_Vector;
extern UART_HandleTypeDef  MTuSC_UART;

#define RX_BUF_SIZE 10
uint8_t rx_buf[RX_BUF_SIZE] = {0};

// Main Thread
#define THREAD_MAIN_STACK_SIZE 512
static const uint16_t THREAD_MAIN_DELAY_MS = 100;
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];
void thread_main(ULONG ctx);

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 512
static const uint16_t LED_BLINK_TIME_MS = 1000;
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];
void thread_blink(ULONG ctx);

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
    MCAN_Init( FDCAN1, DEV_MTUSC, &mcanRxMessage );

    while(true)
    {
        BNO055_Get_Gravity_Vec(&BNO055_Vector);
        
        // Echo UART messages
        if( HAL_UART_Receive(&MTuSC_UART, rx_buf, RX_BUF_SIZE, HAL_MAX_DELAY) == HAL_OK)
            HAL_UART_Transmit(&MTuSC_UART, rx_buf, RX_BUF_SIZE, HAL_MAX_DELAY);

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

void MCAN_RxHandler(void)
{
    tx_thread_sleep(500);
}

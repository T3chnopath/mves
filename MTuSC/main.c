#include <stdbool.h>
#include "bsp_mtusc.h"
#include "tx_api.h"

// Main Thread
#define THREAD_MAIN_STACK_SIZE 512
static const uint16_t THREAD_MAIN_DELAY_MS = 10;
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];
void thread_main(ULONG ctx);

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
}

void thread_main(ULONG ctx)
{
    BSP_Init();

    while(true)
    {
        tx_thread_sleep(THREAD_MAIN_DELAY_MS);
    }
}
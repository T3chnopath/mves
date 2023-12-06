#include <stdint.h>

#include "stm32h5xx_it.h"
#include "stm32h5xx_hal.h"
#include "tx_api.h"
#include "mcan.h"

extern UART_HandleTypeDef ConsoleUart;
extern TIM_HandleTypeDef   hFeedback_Tim;

void SysTick_Handler(void)
{
  _tx_timer_interrupt();
}

uint32_t HAL_GetTick(void)
{
  return _tx_time_get();
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(MCAN_GetFDCAN_Handle());
}

void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&ConsoleUart);
}

void TIM14_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&hFeedback_Tim);
}
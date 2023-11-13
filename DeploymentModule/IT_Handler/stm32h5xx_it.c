#include <stdint.h>

#include "stm32h5xx_it.h"
#include "stm32h5xx_hal.h"
#include "tx_api.h"
#include "mcan.h"
#include "deployment.h"
#include "bsp_deployment.h"

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

void ArmLS_RetractHandle(void)
{
    static uint32_t lastInterruptTime = 0;
    uint32_t currInterruptTime = HAL_GetTick();

    // Debounce
    if(currInterruptTime - lastInterruptTime > LS_DEBOUNCE_MS)
    {
        ArmRetractLS();

        lastInterruptTime = currInterruptTime;
        __HAL_GPIO_EXTI_CLEAR_RISING_IT(ARM_LS_RETRACT_Pin);
    }
}

void ArmLS_DeployHandle(void)
{
    static uint32_t lastInterruptTime = 0;
    uint32_t currInterruptTime = HAL_GetTick();

    // Debounce
    if(currInterruptTime - lastInterruptTime > LS_DEBOUNCE_MS)
    {

        ArmDeployLS();
       
        lastInterruptTime = currInterruptTime;
        __HAL_GPIO_EXTI_CLEAR_RISING_IT(ARM_LS_DEPLOY_Pin);
    }
}
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
    uint32_t count = 0;
    while(HAL_GPIO_ReadPin(ARM_LS_RETRACT_Port, ARM_LS_RETRACT_Pin))
    {
        if(count < LS_DEBOUNCE_MS)
        {
            HAL_Delay(1);
            count++;
        }
        else
        {
            break;
        }
    }

    // If debounce time has passed, execute
    if(count == LS_DEBOUNCE_MS)
    {
        ArmRetractLS();
    }

    __HAL_GPIO_EXTI_CLEAR_RISING_IT(ARM_LS_RETRACT_Pin);
}

void ArmLS_DeployHandle(void)
{
    uint32_t count = 0;
    while(HAL_GPIO_ReadPin(ARM_LS_DEPLOY_Port, ARM_LS_DEPLOY_Pin))
    {
        if(count < LS_DEBOUNCE_MS)
        {
            HAL_Delay(1);
            count++;
        }
        else
        {
            break;
        }
    }

    // If debounce time has passed, execute
    if(count == LS_DEBOUNCE_MS)
    {
        ArmDeployLS();
    }

    __HAL_GPIO_EXTI_CLEAR_RISING_IT(ARM_LS_RETRACT_Pin);
}
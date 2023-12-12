#ifndef __BSP_DEPLOYMENT_H
#define __BSP_DEPLOYMENT_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_gpio.h"

#define FDCAN1_EN

// General in Definitions
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOB

#define LED_GREEN_Pin GPIO_PIN_6
#define LED_GREEN_GPIO_Port GPIOB

#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB

#define FDCAN_STDBY_Pin GPIO_PIN_13
#define FDCAN_STDBY_GPIO_Port GPIOC

// Limit Switches 
// LS0
#define ARM_LS_RETRACT_Pin      GPIO_PIN_5
#define ARM_LS_RETRACT_Port     GPIOC
#define ARM_LS_RETRACT_PRI      3
#define ArmLS_RetractHandle     EXTI5_IRQHandler

// LS1
#define ARM_LS_DEPLOY_Pin       GPIO_PIN_4
#define ARM_LS_DEPLOY_Port      GPIOC
#define ARM_LS_DEPLOY_PRI       3
#define ArmLS_DeployHandle      EXTI4_IRQHandler

#define LS_DEBOUNCE_MS          100

// Peripheral Definitions 
// FDCAN 
#ifdef  FDCAN1_EN
#define FDCAN_TX_Port GPIOB
#define FDCAN_TX_Pin  GPIO_PIN_10
#define FDCAN_RX_Port GPIOB
#define FDCAN_RX_Pin  GPIO_PIN_12
#define FDCAN1_PRI    2
#endif

// Motor Definitions
// extern TIM_HandleTypeDef hBayDC_Tim;
// extern TIM_HandleTypeDef hArmDC_Tim;
extern TIM_HandleTypeDef hACT_Tim;

#define BAY_DC_TIM          TIM1
#define BAY_DC_PERIOD       (25000 - 1)
#define BAY_DC_Pin1         GPIO_PIN_9
#define BAY_DC_Port1        GPIOA
#define BAY_DC_Pin2         GPIO_PIN_10
#define BAY_DC_Port2        GPIOA
// #define BAY_DC_FLIP   

// #define ARM_DC_TIM      TIM3
// #define ARM_DC_PERIOD   (25000 - 1)
#define ARM_DC_Pin1     GPIO_PIN_11
#define ARM_DC_Port1    GPIOA
#define ARM_DC_Pin2     GPIO_PIN_12
#define ARM_DC_Port2    GPIOA
#define ARM_DC_FLIP

// Actuator Definition
#define ACT_TIM             TIM3    
#define ACT_PRESCALER       (125 - 1)
#define ACT_PERIOD          (40000 - 1)
#define ACT_Port            GPIOC
#define ACT_Pin             GPIO_PIN_7
#define ACT_MIN_PULSE       900
#define ACT_MAX_PULSE       2100
#define ACT_MIN_LEN         0
#define ACT_MAX_LEN         27
#define ACT_USER_MIN_LEN    0
#define ACT_USER_MAX_LEN    16

// Public Functions
void BSP_Init(void);

#endif /* __BSP_DEPLOYMENT_H */
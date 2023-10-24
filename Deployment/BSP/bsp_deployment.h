#ifndef __DEPLOYMENT_H
#define __DEPLOYMENT_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_gpio.h"

#define FDCAN1_EN

// Pin Definitions
#define LED_RED_Pin           GPIO_PIN_5
#define LED_RED_GPIO_Port     GPIOB

#define LED_GREEN_Pin           GPIO_PIN_6
#define LED_GREEN_GPIO_Port     GPIOB

#define LED_BLUE_Pin           GPIO_PIN_7
#define LED_BLUE_GPIO_Port     GPIOB

#define FDCAN_STDBY_Pin            GPIO_PIN_13
#define FDCAN_STDBY_GPIO_Port      GPIOC


#ifdef  FDCAN1_EN  
#define FDCAN_TX_Port           GPIOB
#define FDCAN_TX_Pin            GPIO_PIN_10
#define FDCAN_RX_Port           GPIOB
#define FDCAN_RX_Pin            GPIO_PIN_12
#endif

// Public Functions
void BSP_Init(void);

#endif /* __DEPLOYMENT_H */
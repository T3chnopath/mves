#ifndef __MAIN_H
#define __MAIN_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_gpio.h"

#define FDCAN2_EN

// Pin Definitions
#define LED_GREEN_Pin           GPIO_PIN_0
#define LED_GREEN_GPIO_Port     GPIOB

#ifdef  FDCAN1_EN  
#define FDCAN_TX_Port           GPIOD
#define FDCAN_TX_Pin            GPIO_PIN_1
#define FDCAN_RX_Port           GPIOD
#define FDCAN_RX_Pin            GPIO_PIN_0
#endif

#ifdef  FDCAN2_EN  
#define FDCAN_TX_Port           GPIOA
#define FDCAN_TX_Pin            GPIO_PIN_10
#define FDCAN_RX_Port           GPIOB
#define FDCAN_RX_Pin            GPIO_PIN_12
#endif

// Public Functions
void BSP_Init(void);

#endif /* __MAIN_H */

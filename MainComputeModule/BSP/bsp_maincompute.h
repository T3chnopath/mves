#ifndef __BSP_MAINCOMPUTE_H
#define __BSP_MAINCOMPUTE_H

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
#define LED0_RED_Pin GPIO_PIN_0
#define LED0_RED_GPIO_Port GPIOD

#define LED1_RED_Pin GPIO_PIN_3
#define LED1_RED_GPIO_Port GPIOD

#define LED0_GREEN_Pin GPIO_PIN_1
#define LED0_GREEN_GPIO_Port GPIOD

#define LED1_GREEN_Pin GPIO_PIN_4
#define LED1_GREEN_GPIO_Port GPIOD

#define LED0_BLUE_Pin GPIO_PIN_2
#define LED0_BLUE_GPIO_Port GPIOD

#define LED1_BLUE_Pin GPIO_PIN_5
#define LED1_BLUE_GPIO_Port GPIOD

#define BTN0_Pin    GPIO_PIN_0
#define BTN0_Port   GPIOE

#define BTN1_Pin    GPIO_PIN_9
#define BTN1_Port   GPIOB

#define FDCAN_STDBY_Pin GPIO_PIN_10
#define FDCAN_STDBY_GPIO_Port GPIOA

// Peripheral Definitions 
// FDCAN 

#ifdef FDCAN1_EN
#define FDCAN_TX_Port GPIOA
#define FDCAN_TX_Pin  GPIO_PIN_12
#define FDCAN_RX_Port GPIOA
#define FDCAN_RX_Pin  GPIO_PIN_11
#endif

// I2C
#define I2C_SDA_Port    GPIOC
#define I2C_SDA_Pin     GPIO_PIN_9
#define I2C_SCL_Port    GPIOA
#define I2C_SCL_Pin     GPIO_PIN_8

// Public Functions
void BSP_Init(void);

#endif /* __BSP_DEPLOYMENT_H */
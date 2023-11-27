#ifndef BSP_MTUSC_H
#define BSP_MTUSC_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_gpio.h"

#define FDCAN1_EN
#define I2C1_EN
#define UART4_EN

// General in Definitions
#define LED_RED_Pin           GPIO_PIN_0
#define LED_RED_GPIO_Port     GPIOD

#define LED_GREEN_Pin         GPIO_PIN_1
#define LED_GREEN_GPIO_Port   GPIOD

#define LED_BLUE_Pin          GPIO_PIN_2
#define LED_BLUE_GPIO_Port    GPIOD

#define FDCAN_STDBY_Pin       GPIO_PIN_10
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
#ifdef I2C1_EN
#define I2C_SDA_Port    GPIOB
#define I2C_SDA_Pin     GPIO_PIN_9
#define I2C_SCL_Port    GPIOB
#define I2C_SCL_Pin     GPIO_PIN_6
#endif

// UART
#ifdef UART4_EN
#define UART_TX_Port   GPIOA
#define UART_TX_Pin    GPIO_PIN_0
#define UART_RX_Port   GPIOA
#define UART_RX_Pin    GPIO_PIN_1
#define UART_BAUDRATE  921600
#endif

// Public Functions
void BSP_Init(void);

#endif /* __BSP_MTUSC_H */
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

// General in Definitions
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOB

#define LED_GREEN_Pin GPIO_PIN_6
#define LED_GREEN_GPIO_Port GPIOB

#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB

#define FDCAN_STDBY_Pin GPIO_PIN_13
#define FDCAN_STDBY_GPIO_Port GPIOC

// Peripheral Definitions 
// FDCAN 
#ifdef FDCAN1_EN
#define FDCAN_TX_Port GPIOB
#define FDCAN_TX_Pin GPIO_PIN_10
#define FDCAN_RX_Port GPIOB
#define FDCAN_RX_Pin GPIO_PIN_12
#endif

// I2C
#define I2C_SDA_Port    GPIOB
#define I2C_SDA_Pin     GPIO_PIN_9
#define I2C_SCL_Port    GPIOB
#define I2C_SCL_Pin     GPIO_PIN_6

// XBEE UART
#define XBEE_UART_TX_Port   GPIOA
#define XBEE_UART_TX_Pin    GPIO_PIN_0
#define XBEE_UART_RX_Port   GPIOA
#define XBEE_UART_RX_Pin    GPIO_PIN_1
#define XBEE_UART_BAUDRATE  921600

// Public Functions
void BSP_Init(void);

#endif /* __BSP_MTUSC_H */
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

//Added For Deployment
#include "stm32h5xx_hal_tim.h"
#include "stm32h5xx_hal_tim_ex.h"
#include "stm32h5xx_hal_i2c.h"
#include "stm32h563xx.h"

// Peripheral Instances
TIM_HandleTypeDef Servo_Act_Timer;
TIM_HandleTypeDef DC_Motor_Timer;
TIM_HandleTypeDef Encoder_Timer;

// Pin Definitions
#define LED1_GREEN_Pin GPIO_PIN_5
#define LED1_GREEN_GPIO_Port GPIOA

// Public Functions
void BSP_Error_Handler(void);
void BSP_SystemClock_Config(void);
void BSP_GPIO_Init(void);
void BSP_TIMER_Init(void);

#endif /* __MAIN_H */

#include "bsp_deployment.h"
#include "utility.h"

static void _BSP_SystemClockConfig(void);
static void _BSP_ErrorHandler(void);
static void _BSP_GPIO_Init(void);
static void _BSP_FDCAN_Init(void);
static void _BSP_BAY_DC_Init(void);
static void _BSP_ARM_DC_Init(void);
static void _BSP_ACT_Init(void);
static void _BSP_LS_Init(void);

TIM_HandleTypeDef hBayDC_Tim;
TIM_HandleTypeDef hArmDC_Tim;
TIM_HandleTypeDef hACT_Tim;

void BSP_Init(void)
{
    HAL_Init();
    _BSP_SystemClockConfig();

    _BSP_GPIO_Init();
    _BSP_FDCAN_Init();
    
    _BSP_BAY_DC_Init();
    _BSP_ARM_DC_Init();
    _BSP_ACT_Init();

    _BSP_LS_Init();
}

static void _BSP_SystemClockConfig(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while( !__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) ) 
    {
        // NOP
    }

    // Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLL1_SOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 250;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE    = RCC_PLL1_VCIRANGE_1;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
    RCC_OscInitStruct.PLL.PLLFRACN  = 0;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }

    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
}


static void _BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_PortClkEnable(LED_RED_GPIO_Port);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_RED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

    GPIO_PortClkEnable(LED_GREEN_GPIO_Port);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_GREEN_Pin;
    HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

    GPIO_PortClkEnable(LED_BLUE_GPIO_Port);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_BLUE_Pin;

    GPIO_PortClkEnable(FDCAN_STDBY_GPIO_Port);
    HAL_GPIO_WritePin(FDCAN_STDBY_GPIO_Port, FDCAN_STDBY_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = FDCAN_STDBY_Pin;
    HAL_GPIO_Init(FDCAN_STDBY_GPIO_Port, &GPIO_InitStruct);
}

static void _BSP_FDCAN_Init(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Config FDCAN clock to be 32MHz
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.PLL2.PLL2Source = RCC_PLL2_SOURCE_HSE;
    PeriphClkInitStruct.PLL2.PLL2M = 2;
    PeriphClkInitStruct.PLL2.PLL2N = 16;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 6;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVQ;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2Q;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }

    // Peripheral Clock Enable 
    __HAL_RCC_FDCAN_CLK_ENABLE();
    // GPIO_PortClkEnable(FDCAN_TX_Port);
    // GPIO_PortClkEnable(FDCAN_RX_Port);
    __HAL_RCC_GPIOB_CLK_ENABLE();


    // FDCAN GPIO Configuration
    GPIO_InitStruct.Pin = FDCAN_TX_Pin | FDCAN_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // Select alternate function based on FDCAN interface
#if defined(FDCAN1_EN)
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
#endif

    HAL_GPIO_Init(FDCAN_TX_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(FDCAN_RX_Port, &GPIO_InitStruct);

    // Interrupt enable
#if defined(FDCAN1_EN)
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, FDCAN1_PRI, 1);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif 
}

static void _BSP_BAY_DC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_PortClkEnable(BAY_DC_Port1);
    GPIO_PortClkEnable(BAY_DC_Port2);

    // Pin Configurations 
    GPIO_InitStruct.Pin = BAY_DC_Pin1 | BAY_DC_Pin2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BAY_DC_Port1, &GPIO_InitStruct);
    HAL_GPIO_Init(BAY_DC_Port2, &GPIO_InitStruct);

}

static void _BSP_ARM_DC_Init(void)
{

    // __HAL_RCC_TIM3_CLK_ENABLE();

    // PWM Configurations
    // TIM_MasterConfigTypeDef sMasterConfig = {0};
    // TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // hArmDC_Tim.Instance = ARM_DC_TIM;
    // hArmDC_Tim.Init.Prescaler = 0;
    // hArmDC_Tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    // hArmDC_Tim.Init.Period = 25000-1;
    // hArmDC_Tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // hArmDC_Tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // if (HAL_TIM_PWM_Init(&hArmDC_Tim) != HAL_OK)
    // {
    //     _BSP_ErrorHandler;
    // }
    // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    // sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    // if (HAL_TIMEx_MasterConfigSynchronization(&hArmDC_Tim, &sMasterConfig) != HAL_OK)
    // {
    //     _BSP_ErrorHandler;
    // }
    // sConfigOC.OCMode = TIM_OCMODE_PWM1;
    // sConfigOC.Pulse = 0;
    // sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    // sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    // if (HAL_TIM_PWM_ConfigChannel(&hArmDC_Tim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    // {
    //     _BSP_ErrorHandler;
    // }
    // if (HAL_TIM_PWM_ConfigChannel(&hArmDC_Tim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    // {
    //     _BSP_ErrorHandler;
    // }
    // if (HAL_TIM_PWM_ConfigChannel(&hArmDC_Tim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    // {
    //     _BSP_ErrorHandler;
    // }
    // if (HAL_TIM_PWM_ConfigChannel(&hArmDC_Tim, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    // {
    //     _BSP_ErrorHandler;
    // }


    GPIO_PortClkEnable(ARM_DC_Port1);
    GPIO_PortClkEnable(ARM_DC_Port1);

    // Pin Configurations 
    // GPIO_InitStruct.Pin = ARM_DC_Pin1 | ARM_DC_Pin2;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    // HAL_GPIO_Init(ARM_DC_Port1, &GPIO_InitStruct);
    // HAL_GPIO_Init(ARM_DC_Port2, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ARM_DC_Pin1 | ARM_DC_Pin2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ARM_DC_Port1, &GPIO_InitStruct);
    HAL_GPIO_Init(ARM_DC_Port2, &GPIO_InitStruct);

}

static void _BSP_ACT_Init(void){

    // Timer Configuration
    __HAL_RCC_TIM3_CLK_ENABLE();    

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    hACT_Tim.Instance = ACT_TIM;
    hACT_Tim.Init.Prescaler = ACT_PRESCALER;
    hACT_Tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    hACT_Tim.Init.Period = ACT_PERIOD;
    hACT_Tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hACT_Tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&hACT_Tim) != HAL_OK)
    {
      _BSP_ErrorHandler;
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&hACT_Tim, &sClockSourceConfig) != HAL_OK)
    {
      _BSP_ErrorHandler;
    }
    if (HAL_TIM_PWM_Init(&hACT_Tim) != HAL_OK)
    {
      _BSP_ErrorHandler;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&hACT_Tim, &sMasterConfig) != HAL_OK)
    {
      _BSP_ErrorHandler;
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&hACT_Tim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
      _BSP_ErrorHandler;
    }

    // Pin Configuration
    GPIO_PortClkEnable(ACT_Port);

    GPIO_InitStruct.Pin = ACT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(ACT_Port, &GPIO_InitStruct);

}

static void _BSP_LS_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  IRQn_Type ArmLS_RetractIRQn = PinToEXTI(ARM_LS_RETRACT_Pin);
  IRQn_Type ArmLS_DeployIRQn = PinToEXTI(ARM_LS_DEPLOY_Pin);

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 

  // Interrupt enable
  HAL_NVIC_SetPriority(ArmLS_RetractIRQn, ARM_LS_RETRACT_PRI, 0);
  HAL_NVIC_EnableIRQ(ArmLS_RetractIRQn);

  HAL_NVIC_SetPriority(ArmLS_DeployIRQn, ARM_LS_DEPLOY_PRI, 1);
  HAL_NVIC_EnableIRQ(ArmLS_DeployIRQn);
}


static void _BSP_ErrorHandler(void)
{
    __disable_irq();
    while (1)
    {
        // NOP
    }
}
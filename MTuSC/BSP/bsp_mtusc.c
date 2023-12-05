#include <stdbool.h>
#include "bsp_mtusc.h"
#include "utility.h"
#include "tx_api.h"
#include "bno055.h"

static void _BSP_SystemClockConfig(void);
static void _BSP_ErrorHandler(void);
static void _BSP_GPIO_Init(void);
static void _BSP_FDCAN_Init(void);
static void _BSP_I2C_Init(void);
static void _BSP_UART_Init(void);
static bool _BSP_IMU_Init(void);
static bool _BSP_CServo_Init(void);
static bool _BSP_Feedback_IC_Init(void);

// Peripheral Instance
I2C_HandleTypeDef   MTuSC_I2C;
UART_HandleTypeDef  MTuSC_UART;
TIM_HandleTypeDef   hServo_Tim;
TIM_HandleTypeDef   hFeedback_Tim;

static const uint16_t BSP_CLK_DELAY_MS = 100;
static const uint16_t BSP_DELAY_MS = 200;


void BSP_Init(void)
{
    HAL_Init();
    _BSP_SystemClockConfig();
    tx_thread_sleep(BSP_CLK_DELAY_MS);

    _BSP_GPIO_Init();
    _BSP_FDCAN_Init();
    _BSP_I2C_Init();
    _BSP_UART_Init(); 
    _BSP_IMU_Init();
    _BSP_CServo_Init();
    _BSP_Feedback_IC_Init();
    tx_thread_sleep(BSP_DELAY_MS);
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
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   =  LED_RED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

    GPIO_PortClkEnable(LED_GREEN_GPIO_Port);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LED_GREEN_Pin;
    HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

    GPIO_PortClkEnable(LED_BLUE_GPIO_Port);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LED_BLUE_Pin;
    HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

    GPIO_PortClkEnable(FDCAN_STDBY_GPIO_Port);
    HAL_GPIO_WritePin(FDCAN_STDBY_GPIO_Port, FDCAN_STDBY_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin  = FDCAN_STDBY_Pin;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
    PeriphClkInitStruct.PLL2.PLL2N = 32;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 12;
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
    GPIO_PortClkEnable(FDCAN_TX_Port);
    GPIO_PortClkEnable(FDCAN_RX_Port);


    // FDCAN GPIO Configuration
    GPIO_InitStruct.Pin   = FDCAN_TX_Pin | FDCAN_RX_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // Select alternate function based on FDCAN interface
#if defined(FDCAN1_EN)
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
#endif

    HAL_GPIO_Init(FDCAN_TX_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(FDCAN_RX_Port, &GPIO_InitStruct);

    // Interrupt init, default to IT0, preempt = 2, subpriority = 0 
#if defined(FDCAN1_EN)
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif 
}

static void _BSP_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};    

    // I2C1 Clock Enable 
    __HAL_RCC_I2C1_CLK_ENABLE();

    // I2C1 Init 
    MTuSC_I2C.Instance              = I2C1;
    MTuSC_I2C.Init.Timing           = 0x2080319C;
    MTuSC_I2C.Init.OwnAddress1      = 0;
    MTuSC_I2C.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    MTuSC_I2C.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    MTuSC_I2C.Init.OwnAddress2      = 0;
    MTuSC_I2C.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    MTuSC_I2C.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    MTuSC_I2C.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&MTuSC_I2C) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }
    
    // Configure Analog Filter 
    if (HAL_I2CEx_ConfigAnalogFilter(&MTuSC_I2C, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }
    
    // Configure Digital Filter 
    if (HAL_I2CEx_ConfigDigitalFilter(&MTuSC_I2C, 0) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }
    
    GPIO_InitStruct.Pin       = I2C_SDA_Pin|I2C_SCL_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    
    //Enable I2C GPIO RCC Clock
    GPIO_PortClkEnable(I2C_SDA_Port);
    GPIO_PortClkEnable(I2C_SCL_Port);

    HAL_GPIO_Init(I2C_SDA_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(I2C_SCL_Port, &GPIO_InitStruct);
}

static void _BSP_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};    

    // UART4 CLK Enable 
#ifdef UART4_EN
    __HAL_RCC_UART4_CLK_ENABLE();
#endif 

#ifdef UART4_EN
    MTuSC_UART.Instance            = UART4;
#endif
    MTuSC_UART.Init.BaudRate       = UART_BAUDRATE;
    MTuSC_UART.Init.WordLength     = UART_WORDLENGTH_8B;
    MTuSC_UART.Init.StopBits       = UART_STOPBITS_1;
    MTuSC_UART.Init.Parity         = UART_PARITY_NONE;
    MTuSC_UART.Init.Mode           = UART_MODE_TX_RX;
    MTuSC_UART.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    MTuSC_UART.Init.OverSampling   = UART_OVERSAMPLING_16;
    MTuSC_UART.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    MTuSC_UART.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    MTuSC_UART.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&MTuSC_UART) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&MTuSC_UART, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&MTuSC_UART, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }
    if (HAL_UARTEx_DisableFifoMode(&MTuSC_UART) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }

#ifdef UART4_EN
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        _BSP_ErrorHandler();
    }

    //Enable GPIOA Clock
    GPIO_PortClkEnable(UART_RX_Port);
    GPIO_PortClkEnable(UART_TX_Port);


    // UART4 GPIO Configuration
    GPIO_InitStruct.Pin   = UART_TX_Pin|UART_RX_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
#ifdef UART4_EN
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
#endif
    HAL_GPIO_Init(UART_TX_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(UART_RX_Port, &GPIO_InitStruct);
}

bool _BSP_IMU_Init(void)
{
    /* Custom Axis */
    BNO055_AXIS_CONFIG_t axis_config = {.x = BNO055_Z_AXIS,
                                        .y = BNO055_Y_AXIS,
                                        .z = BNO055_X_AXIS};

    /* Required Boot-up Time for BNO055 */
    tx_thread_sleep(700);

    /* BNO055 Init */
    BNO055_I2C_Mount(&MTuSC_I2C);
    if (BNO055_Init() != BNO055_SUCCESS)
        return false;

    if (BNO055_Set_OP_Mode(NDOF) != BNO055_SUCCESS)
        return false;

    if (BNO055_Set_Axis(&axis_config) != BNO055_SUCCESS)
        return false;
}

static bool _BSP_CServo_Init(void){

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* Initialize Servo PWM Timer */
    __HAL_RCC_TIM1_CLK_ENABLE();

    hServo_Tim.Instance = CSERVO_TIM;
    hServo_Tim.Init.Prescaler = CSERVO_TIM_PRESCALER;
    hServo_Tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    hServo_Tim.Init.Period = CSERVO_TIM_PERIOD;
    hServo_Tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hServo_Tim.Init.RepetitionCounter = 0;
    hServo_Tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&hServo_Tim) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&hServo_Tim, &sClockSourceConfig) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
    if (HAL_TIM_PWM_Init(&hServo_Tim) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&hServo_Tim, &sMasterConfig) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&hServo_Tim, &sConfigOC, CSERVO_TIM_CHANNEL) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&hServo_Tim, &sBreakDeadTimeConfig) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }

    /* Initialize Servo PWM GPIO */
    GPIO_PortClkEnable(CSERVO_Port);
    GPIO_InitStruct.Pin = CSERVO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(CSERVO_Port, &GPIO_InitStruct);

}

static bool _BSP_Feedback_IC_Init(void){

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* Feedback IC Timer GPIO Init */
    GPIO_PortClkEnable(FEEDBACK_IC_Port);
    GPIO_InitStruct.Pin = FEEDBACK_IC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init(FEEDBACK_IC_Port, &GPIO_InitStruct);

    /* Feedback IC Timer Init */
    __HAL_RCC_TIM14_CLK_ENABLE();
    hFeedback_Tim.Instance = FEEDBACK_IC_TIM;
    hFeedback_Tim.Init.Prescaler = FEEDBACK_IC_TIM_PRESCALER;
    hFeedback_Tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    hFeedback_Tim.Init.Period = FEEDBACK_IC_TIM_PERIOD;
    hFeedback_Tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hFeedback_Tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&hFeedback_Tim) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
    if (HAL_TIM_IC_Init(&hFeedback_Tim) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&hFeedback_Tim, &sConfigIC, FEEDBACK_IC_CHANNEL) != HAL_OK)
    {
      _BSP_ErrorHandler();
    }

    /* Interrupt Setup */
    HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);

}

static void _BSP_ErrorHandler(void)
{
    __disable_irq();
    while (1)
    {
        // NOP
    }
}
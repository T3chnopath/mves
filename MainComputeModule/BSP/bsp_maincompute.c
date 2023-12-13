#include "bsp_maincompute.h"
#include "utility.h"
#include "tx_api.h"

static void _BSP_SystemClockConfig(void);
static void _BSP_ErrorHandler(void);
static void _BSP_GPIO_Init(void);
static void _BSP_FDCAN_Init(void);
static void _BSP_I2C_Init(void);
static void _BSP_MIO_UART_Init(void);

static const uint16_t BSP_CLK_DELAY_MS = 100;
static const uint16_t BSP_DELAY_MS = 100;

// Peripheral Instance
I2C_HandleTypeDef   ComputeI2C;
UART_HandleTypeDef  MIO_UART;

void BSP_Init(void)
{
    HAL_Init();
    _BSP_SystemClockConfig();
    tx_thread_sleep(BSP_CLK_DELAY_MS);

    _BSP_GPIO_Init();
    _BSP_FDCAN_Init();
    _BSP_I2C_Init(); 
    _BSP_MIO_UART_Init();
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

  /* Onboard LED Init */
  //Red
  GPIO_PortClkEnable(LED0_RED_GPIO_Port);
  GPIO_PortClkEnable(LED1_RED_GPIO_Port);
  HAL_GPIO_WritePin(LED0_RED_GPIO_Port, LED0_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_RED_GPIO_Port, LED1_RED_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = LED0_RED_Pin | LED1_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_RED_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(LED1_RED_GPIO_Port, &GPIO_InitStruct);

  //Green
  GPIO_PortClkEnable(LED0_GREEN_GPIO_Port);
  GPIO_PortClkEnable(LED1_GREEN_GPIO_Port);
  HAL_GPIO_WritePin(LED0_GREEN_GPIO_Port, LED0_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = LED0_GREEN_Pin | LED1_GREEN_Pin;
  HAL_GPIO_Init(LED0_GREEN_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(LED1_GREEN_GPIO_Port, &GPIO_InitStruct);

  //Blue
  GPIO_PortClkEnable(LED0_BLUE_GPIO_Port);
  GPIO_PortClkEnable(LED1_BLUE_GPIO_Port);
  HAL_GPIO_WritePin(LED0_BLUE_GPIO_Port, LED0_BLUE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_BLUE_GPIO_Port, LED1_BLUE_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = LED0_BLUE_Pin | LED1_BLUE_Pin;
  HAL_GPIO_Init(LED0_BLUE_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(LED1_BLUE_GPIO_Port, &GPIO_InitStruct);

  /* Onboard BTN Init */

  //BTN0 Init
  GPIO_PortClkEnable(BTN0_Port);
  GPIO_InitStruct.Pin = BTN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN0_Port, &GPIO_InitStruct);

  //BTN1 Init
  GPIO_PortClkEnable(BTN1_Port);
  GPIO_InitStruct.Pin = BTN1_Pin;
  HAL_GPIO_Init(BTN1_Port, &GPIO_InitStruct);

  // FDCAN 
  GPIO_PortClkEnable(FDCAN_STDBY_GPIO_Port);
  HAL_GPIO_WritePin(FDCAN_STDBY_GPIO_Port, FDCAN_STDBY_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
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

    // Interrupt init, default to IT0, preempt = 2, subpriority = 0 
#if defined(FDCAN1_EN)
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif 
}

static void _BSP_I2C_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};    

  /* I2C3 clock enable */
  __HAL_RCC_I2C3_CLK_ENABLE();

  /* I2C3 Init */
  ComputeI2C.Instance = I2C3;
  ComputeI2C.Init.Timing = 0x2080319C;
  ComputeI2C.Init.OwnAddress1 = 0;
  ComputeI2C.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  ComputeI2C.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  ComputeI2C.Init.OwnAddress2 = 0;
  ComputeI2C.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  ComputeI2C.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  ComputeI2C.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&ComputeI2C) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }
  
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&ComputeI2C, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }
  
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&ComputeI2C, 0) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
  PeriphClkInitStruct.I2c1ClockSelection   = RCC_I2C3CLKSOURCE_PCLK3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }
  
  GPIO_InitStruct.Pin = I2C_SDA_Pin|I2C_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  
  //Enable I2C GPIOA and GPIOC RCC Clock
  GPIO_PortClkEnable(I2C_SDA_Port);
  GPIO_PortClkEnable(I2C_SCL_Port);

  HAL_GPIO_Init(I2C_SDA_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(I2C_SCL_Port, &GPIO_InitStruct);
}

static void _BSP_MIO_UART_Init(void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  // UART
  __HAL_RCC_UART7_CLK_ENABLE();

  MIO_UART.Instance = UART7;
  MIO_UART.Init.BaudRate = 115200;
  MIO_UART.Init.WordLength = UART_WORDLENGTH_8B;
  MIO_UART.Init.StopBits = UART_STOPBITS_1;
  MIO_UART.Init.Parity = UART_PARITY_NONE;
  MIO_UART.Init.Mode = UART_MODE_TX_RX;
  MIO_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  MIO_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  MIO_UART.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  MIO_UART.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  MIO_UART.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&MIO_UART) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&MIO_UART, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&MIO_UART, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }
  if (HAL_UARTEx_DisableFifoMode(&MIO_UART) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _BSP_ErrorHandler();
  }

  // GPIO
  __HAL_RCC_GPIOE_CLK_ENABLE();
  /**UART7 GPIO Configuration
  PE7     ------> UART7_RX
  PE8     ------> UART7_TX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

static void _BSP_ErrorHandler(void)
{
    __disable_irq();
    while (1)
    {
        // NOP
    }
}
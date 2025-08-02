/* USER CODE BEGIN Header */
/**
 Template based on https://github.com/JakobJelovcan/stm32h7-tetris
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FlexiKeyboard.h"
#include "display.h"
#include "appLogic.h"
#include "retarget.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed))
{
  uint16_t val1;
  uint16_t val2;
  uint16_t val3;
  uint16_t magic; // to recognize if the flash has been already setup or not
} ConfigData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EMMC_START_ADDR 0
#define EMMC_BLOCK_COUNT 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for lcdTask */
TIM_HandleTypeDef tim2;
RNG_HandleTypeDef rng;
TIM_HandleTypeDef htim8;
UART_HandleTypeDef huart3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void LCD_Config(void);
static void TS_Config(void);
static void TIM_Config(void);
static void RNG_Config(void);
static void MX_USART3_UART_Init(void);
static int32_t MMC_Config(void);
static void StoreContext(AppContext *ctx);
static int ReadContextFromEMMC(AppContext *ctx);

void TIM8_Stop();
void TIM8_Start(uint16_t percent);
void MX_TIM8_PWM_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  TS_Config();
  RNG_Config();
  LCD_Config();
  TIM_Config();
  MMC_Config();

  InitFlexiKeyboard(); // has to be AFTER InitializeLcd, which initializes PK1 as LTDC_G6 pin. We override it, so we might lose some precision on green channel.
  MX_TIM8_PWM_Init(); // initialize PWM output on pin PI2
  MX_USART3_UART_Init();
  RetargetInit(&huart3);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  AppContext ctx;
  ReadContextFromEMMC(&ctx);
  InitializeAppContext(&ctx);
  while (1)
  {
    KeyboardButton key = ReadFlexiKeyboard(); // approx 5ms blocking code to scan the keyboard
    bool ctxChanged = handle_event(&ctx, key, TIM8_Start, TIM8_Stop, StoreContext);
    if (!ctxChanged) continue; // no need to redraw display
    DisplayState(&ctx);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void MX_TIM8_PWM_Init()
{
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  uint32_t timerClock = HAL_RCC_GetPCLK2Freq(); // TIM8 is on APB2
  uint32_t prescaler = 9;
  uint32_t period = (timerClock / ((prescaler + 1) * 5000)) - 1; // 10kHz - magic, I need to look at this deeper

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = prescaler;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = period;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    Error_Handler();

  TIM_OC_InitTypeDef sConfigOC = { 0 };
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // (period + 1) * duty / 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();
}

void TIM8_Start(uint16_t percent)
{
  // TODO convert voltage to PWM percentage using calibration points
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim8);
  uint32_t pulse = (period + 1) * percent / 100;
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pulse);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void TIM8_Stop()
{
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
      != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
      != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) // I had to explicitly define this method here, probably I am missing some uart library?
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
  if (huart->Instance == USART3)
  {
    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection =
        RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
     PB10     ------> USART3_TX
     PB11     ------> USART3_RX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
   clocked below the maximum system frequency, to update the voltage scaling value
   regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
      | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
      | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
   (GPIO, SPI, FMC, QSPI ...)  when  operating at  high frequencies(please refer to product datasheet)
   The I/O Compensation Cell activation  procedure requires :
   - The activation of the CSI clock
   - The activation of the SYSCFG clock
   - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR
   */

  /*activate CSI clock mandatory for I/O Compensation Cell*/
  __HAL_RCC_CSI_ENABLE();

  /* Enable SYSCFG clock mandatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE()
  ;

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();
}

/* USER CODE BEGIN 4 */
static void LCD_Config(void)
{
  BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
  BSP_LCD_SetLayerAddress(0, 0, LCD_LAYER_0_ADDRESS);
  UTIL_LCD_SetFuncDriver(&LCD_Driver);
  UTIL_LCD_SetFont(&Font12);
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLACK);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
  UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK);
  HAL_NVIC_SetPriority(LTDC_IRQn, 10, 10);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);
}

static void TS_Config(void)
{
  TS_Init_t init = { TS_MAX_WIDTH, TS_MAX_HEIGHT, TS_SWAP_XY, 5 };
  BSP_TS_Init(0, &init);
}

static void TIM_Config(void)
{
  //Timer base frequency is 200MHz
  //An interrupt is triggered every 10ms
  __HAL_RCC_TIM2_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM2_IRQn, 10, 10);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  tim2.Instance = TIM2;
  tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim2.Init.Period = 10000 - 1;
  tim2.Init.Prescaler = 200 - 1;
  HAL_TIM_Base_Init(&tim2);
  __HAL_TIM_CLEAR_FLAG(&tim2, TIM_FLAG_UPDATE);
}

static void RNG_Config(void)
{
  __HAL_RCC_RNG_CLK_ENABLE();
  rng.Instance = RNG;
  rng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&rng) != HAL_OK)
  {
    Error_Handler();
  }
}

static int32_t MMC_Config(void)
{
  return BSP_MMC_Init(0);
}

void StoreContext(AppContext *ctx)
{
//    BSP_MMC_WriteBlocks(0, top_scores, EMMC_START_ADDR, EMMC_BLOCK_COUNT);
//    while (BSP_MMC_GetCardState(0) != MMC_TRANSFER_OK);

  ConfigData config;
  config.val1 = ctx->calibrationPoints[0];
  config.val2 = ctx->calibrationPoints[1];
  config.val3 = ctx->calibrationPoints[2];
  config.magic = 0xAA55;

  uint32_t buf[MMC_BLOCKSIZE / sizeof(uint32_t)] = { 0 };
  memcpy(buf, &config, sizeof(ConfigData));

  if (BSP_MMC_WriteBlocks(0, (uint32_t*) buf, EMMC_START_ADDR,
      1) != BSP_ERROR_NONE)
    return;

  while (BSP_MMC_GetCardState(0) != MMC_TRANSFER_OK)
    ;
}

int ReadContextFromEMMC(AppContext *ctx)
{
  uint32_t buf[MMC_BLOCKSIZE / sizeof(uint32_t)];

  // initialize reasonable default values in case read from eMMC fails for any reason
  ctx->calibrationPoints[0] = 20;
  ctx->calibrationPoints[1] = 50;
  ctx->calibrationPoints[2] = 100;

  if (BSP_MMC_ReadBlocks(0, buf, EMMC_START_ADDR, 1) != BSP_ERROR_NONE)
  {
    int a = 4;
    a++;
    // possibly return in here, depending on error in HAL. If its 9, its probably OKay?
  }

  while (BSP_MMC_GetCardState(0) != MMC_TRANSFER_OK)
    ;

  ConfigData config;
  memcpy(&config, (const void*) buf, sizeof(ConfigData));
  if (config.magic == 0xAA55)
  {
    ctx->calibrationPoints[0] = config.val1;
    ctx->calibrationPoints[1] = config.val2;
    ctx->calibrationPoints[2] = config.val3;
  }

  return 0;
}

//void HAL_LTDC_ReloadEventCallback(LTDC_HandleTypeDef* hltdc) {
//    static uint16_t buffer_index = 0;
//
//    buffer_index = 1 - buffer_index;
//    uint32_t buffer_addr = (buffer_index) ? LCD_LAYER_1_ADDRESS : LCD_LAYER_0_ADDRESS;
//    BSP_LCD_Reload(0, BSP_LCD_RELOAD_NONE); //Disable reloading
//    BSP_LCD_SetLayerAddress(0, 0, buffer_addr); //Update the buffer
//    UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK); //Clear the new buffer
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

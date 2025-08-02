/* USER CODE BEGIN Header */
/**
Template based on https://github.com/JakobJelovcan/stm32h7-tetris
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
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
ADC_HandleTypeDef hadc1;

/* Definitions for lcdTask */
osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for inputTask */
osThreadId_t inputTaskHandle;
const osThreadAttr_t inputTask_attributes = {
  .name = "inputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void StartLcdTask(void *argument);
void StartInputTask(void *argument);

/* USER CODE BEGIN PFP */
static void LCD_Config(void);
static void TS_Config(void);
static void TIM_Config(void);
static void RNG_Config(void);
static void MX_USART3_UART_Init(void);
static int32_t MMC_Config(void);

void TIM8_Stop();
void TIM8_Start(uint16_t percent);
void MX_TIM8_PWM_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void StoreContext(AppContext *ctx) {
//    BSP_MMC_WriteBlocks(0, top_scores, EMMC_START_ADDR, EMMC_BLOCK_COUNT);
//    while (BSP_MMC_GetCardState(0) != MMC_TRANSFER_OK);

	ConfigData config;
	config.val1 = ctx->calibrationPoints[0];
	config.val2 = ctx->calibrationPoints[1];
	config.val3 = ctx->calibrationPoints[2];
	config.magic = 0xAA55;

	uint32_t buf[MMC_BLOCKSIZE / sizeof(uint32_t)] = {0};
	memcpy(buf, &config, sizeof(ConfigData));

    if (BSP_MMC_WriteBlocks(0, (uint32_t *)buf, EMMC_START_ADDR, 1) != BSP_ERROR_NONE)
        return;

    while (BSP_MMC_GetCardState(0) != MMC_TRANSFER_OK);
}

int ReadContextFromEMMC(AppContext *ctx) {
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

    while (BSP_MMC_GetCardState(0) != MMC_TRANSFER_OK);

    ConfigData config;
    memcpy(&config, (const void *)buf, sizeof(ConfigData));
    if (config.magic == 0xAA55)
    {
        ctx->calibrationPoints[0] = config.val1;
        ctx->calibrationPoints[1] = config.val2;
        ctx->calibrationPoints[2] = config.val3;
    }

    return 0;
}
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
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of lcdTask */
  lcdTaskHandle = osThreadNew(StartLcdTask, NULL, &lcdTask_attributes);

  /* creation of inputTask */
  inputTaskHandle = osThreadNew(StartInputTask, NULL, &inputTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    AppContext ctx;
    ReadContextFromEMMC(&ctx);
    InitializeAppContext(&ctx);
    while (1) {
  	  KeyboardButton key = ReadFlexiKeyboard(); // approx 5ms blocking code to scan the keyboard
  	  bool ctxChanged = handle_event(&ctx, key, TIM8_Start, TIM8_Stop, StoreContext);
  	  if (!ctxChanged) continue; // no need to redraw display
  	  DisplayState(&ctx);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void LCD_Config(void) {
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

static void TS_Config(void) {
    TS_Init_t init = { TS_MAX_WIDTH, TS_MAX_HEIGHT, TS_SWAP_XY, 5 };
    BSP_TS_Init(0, &init);
}

static void TIM_Config(void) {
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

static void RNG_Config(void) {
    __HAL_RCC_RNG_CLK_ENABLE();
    rng.Instance = RNG;
    rng.Init.ClockErrorDetection = RNG_CED_ENABLE;
    if (HAL_RNG_Init(&rng) != HAL_OK) {
        Error_Handler();
    }
}

static int32_t MMC_Config(void) {
    return BSP_MMC_Init(0);
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

/* USER CODE BEGIN Header_StartLcdTask */
/**
  * @brief  Function implementing the lcdTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLcdTask */
void StartLcdTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
* @brief Function implementing the inputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument)
{
  /* USER CODE BEGIN StartInputTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInputTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t		outCurrentFB;
	uint16_t 		loPowSupVolt;
	uint16_t 		hiPowSupVolt;
	uint16_t 		outVoltageFB;
} ProcVal_TypeDef;
typedef union {
	uint16_t 		codes[ADC_DMA_BUFFER_SIZE];
	ProcVal_TypeDef values;
} ADC_MapTypeDef;
typedef enum {
    ADC_IDLE,
    CONVERSION_COMPLETE
} ADC_StatusTypeDef;
typedef enum {
	LO_POW_SUPPLY,
	HI_POW_SUPPLY,
	SUPPLY_FAILURE
} POW_SupplyStatus;
typedef enum {
    START_UP,
    OVERLOAD,
    NORMAL,
    LOAD_TEST,
    HAULT
} DEV_OpModeTypeDef;
typedef enum {
	PID_IDLE,
	UPDATE_PENDING
} PID_StatusTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

ADC_MapTypeDef		adcReadings;
ADC_StatusTypeDef   adcStatus               = ADC_IDLE;
PID_StatusTypeDef   pidStatus               = PID_IDLE;
DEV_OpModeTypeDef   devOpMode               = START_UP;
POW_SupplyStatus	supplyStatus            = SUPPLY_FAILURE;

uint16_t			pidSamplingCnt;
uint16_t			ledBlinkCnt;
uint8_t             numOfAttemptsCnt;
uint16_t            overloadTimeoutCnt;
uint32_t            startUpCnt;

uint32_t            activeUpperChannel      = TIM_CHANNEL_1;
uint32_t            activeLowerChannel      = TIM_CHANNEL_2;

int16_t				pidPulseWidth;
uint16_t			pwmPulseWidth;

int16_t				pidVLoopSetPoint;
int16_t				pidCLoopSetPoint;
int16_t				pidVLoopCtrlError;
int16_t             pidVLoopCtrlErrorD;
int16_t             pidCLoopCtrlError;
int16_t             pidCLoopCtrlErrorD;
int16_t             pidCLoopCtrlErrorDD;

uint8_t             startUpDelay            = 1;
uint16_t            ledBlinkDly;

uint16_t            movAvgArrPtr            = 0;
uint16_t            avgOutVoltFB[MOV_AVG_WINDOW];
uint16_t            avgOutCurrFB[MOV_AVG_WINDOW];
uint16_t            avgLoPowSupVolt;
uint16_t            avgHiPowSupVolt;
uint16_t            avgOutVoltageFB;
uint16_t            avgOutCurrentFB;
uint32_t            sumOutVoltFB;
uint32_t            sumOutCurrFB;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    // Initiate continuous conversion with DMA
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&adcReadings.codes, ADC_DMA_BUFFER_SIZE);
    // Start all PWM channels
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_PERIOD_CODE);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (startUpDelay == 1)
    {
        HAL_Delay(100);
        startUpDelay = 0;
    }
    if (adcStatus == CONVERSION_COMPLETE)
    {
        movAvgArrPtr = ++movAvgArrPtr == MOV_AVG_WINDOW ? 0 : movAvgArrPtr;

        sumOutVoltFB -= (uint32_t)avgOutVoltFB[movAvgArrPtr];
        avgOutVoltFB[movAvgArrPtr] = adcReadings.values.outVoltageFB;
        sumOutVoltFB += (uint32_t)avgOutVoltFB[movAvgArrPtr];

        sumOutCurrFB -= (uint32_t)avgOutCurrFB[movAvgArrPtr];
        avgOutCurrFB[movAvgArrPtr] = adcReadings.values.outCurrentFB;
        sumOutCurrFB += (uint32_t)avgOutCurrFB[movAvgArrPtr];

        avgOutVoltageFB = (uint32_t)(sumOutVoltFB / MOV_AVG_WINDOW);
        avgOutCurrentFB = (uint16_t)(sumOutCurrFB / MOV_AVG_WINDOW);

     
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim3.Init.Period = PWM_PERIOD_CODE;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, OSC_IN_NOT_IN_USE_Pin|OSC_OUT_NOT_IN_USE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA5_NOT_IN_USE_Pin|STATUS_LED_Pin|PIN10_NOT_IN_USE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OSC_IN_NOT_IN_USE_Pin OSC_OUT_NOT_IN_USE_Pin */
  GPIO_InitStruct.Pin = OSC_IN_NOT_IN_USE_Pin|OSC_OUT_NOT_IN_USE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : OUTPUT_OVERLOAD_Pin */
  GPIO_InitStruct.Pin = OUTPUT_OVERLOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OUTPUT_OVERLOAD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5_NOT_IN_USE_Pin STATUS_LED_Pin PIN10_NOT_IN_USE_Pin */
  GPIO_InitStruct.Pin = PA5_NOT_IN_USE_Pin|STATUS_LED_Pin|PIN10_NOT_IN_USE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	uint16_t        expectedScOutVolt;

    if (startUpDelay == 1) return;
    // Increment PID sampling counter and test for overflow
	// On PID sampling counter overflow, set UPDATE_PENDING flag
	// to tell the ADC interrupt routine that PID sampling time
	// has elapsed and PID needs to be updated
    pidSamplingCnt++;
	if (pidSamplingCnt > PID_PERIOD_CODE)
	{
		pidSamplingCnt = 0;
		pidStatus = UPDATE_PENDING;
	}

	ledBlinkCnt++;
	if (ledBlinkCnt > ledBlinkDly)
	{
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
		ledBlinkCnt = 0;
	}

    switch (devOpMode)
    {
    case START_UP:
        startUpCnt++;
        overloadTimeoutCnt = 0;
    break;
    case OVERLOAD:
        startUpCnt = 0;
        overloadTimeoutCnt++;
    break;
    case LOAD_TEST:
        startUpCnt++;
        overloadTimeoutCnt = 0;
    break;
    case NORMAL:
        startUpCnt = 0;
        overloadTimeoutCnt = 0;
    break;
    }

    if (devOpMode == HAULT) return;

    if (pidStatus == UPDATE_PENDING)
    {
        //__disable_irq();
        // Get output voltage regulation error
        pidVLoopCtrlError = pidVLoopSetPoint - adcReadings.values.outVoltageFB;
        // Get output current regulation error
        pidCLoopCtrlError = pidCLoopSetPoint - adcReadings.values.outCurrentFB;

        switch (devOpMode)
        {
        case START_UP:
            if ((startUpCnt > START_UP_DLY) && (avgOutVoltageFB < 100))
            {
                devOpMode = OVERLOAD;
                pidPulseWidth = 0;
            }
            if (startUpCnt > OVERLOAD_DLY)
            {
                devOpMode = HAULT;
            }
            if ((avgOutVoltageFB > (pidVLoopSetPoint - 100)) ||
               ((avgOutVoltageFB < pidVLoopSetPoint) && (pwmPulseWidth > 465)))
            {
                devOpMode = NORMAL;
                numOfAttemptsCnt = 0;
            }
            pidPulseWidth = pidPulseWidth + (int16_t)(
                (pidCLoopCtrlError - pidCLoopCtrlErrorD) / PID_CLP_KP +
                (pidCLoopCtrlError + pidCLoopCtrlErrorD) / PID_CLP_KI +
                (pidCLoopCtrlError -
                 2 * pidCLoopCtrlErrorD + pidCLoopCtrlErrorDD) / PID_CLP_KD);
        break;
        case OVERLOAD:
            if (overloadTimeoutCnt > OVLD_TIMEOUT)
            {
                devOpMode = LOAD_TEST;
                numOfAttemptsCnt++;
            }
            if (numOfAttemptsCnt > NUM_OF_ATTEMPTS_TO_START)
            {
                devOpMode = HAULT;
            }
        break;
        case LOAD_TEST:
            if (adcReadings.values.outCurrentFB > pidCLoopSetPoint)
            {
                pidPulseWidth = 0;
            }
            else
            {
                pidPulseWidth = PID_TO_PWM * PWM_DEAD_TIME;
            }
            if (startUpCnt > START_UP_DLY)
            {
                expectedScOutVolt =
                    (uint16_t)(PWM_DEAD_TIME * adcReadings.values.loPowSupVolt / 1000);
                if (avgOutVoltageFB > expectedScOutVolt)
                {
                    devOpMode = START_UP;
                    startUpCnt = 0;
                }
                else
                {
                    devOpMode = OVERLOAD;
                    pidPulseWidth = 0;
                }
            }
        break;
        case NORMAL:
            if (avgOutCurrentFB > LOAD_CUR_UTHLD_CODE)
            {
                devOpMode = START_UP;
            }
            else if ((adcReadings.values.outVoltageFB < LOAD_SC_VOLT_CODE) &&
                     (adcReadings.values.outCurrentFB > LOAD_SC_CUR_CODE))
            {
                devOpMode = OVERLOAD;
                pidPulseWidth = 0;
                numOfAttemptsCnt = 0;
            }
            else
            {
                pidPulseWidth = pidPulseWidth + (int16_t)(
                    (pidVLoopCtrlError - pidVLoopCtrlErrorD) / PID_VLP_KP +
                    (pidVLoopCtrlError + pidVLoopCtrlErrorD) / PID_VLP_KI);
            }
        break;
        }

        pidCLoopCtrlErrorDD = pidCLoopCtrlErrorD;
        pidCLoopCtrlErrorD  = pidCLoopCtrlError;
        pidVLoopCtrlErrorD  = pidVLoopCtrlError;

        pidPulseWidth = PID_CHECK_UTHLD(pidPulseWidth);
        pidPulseWidth = PID_CHECK_LTHLD(pidPulseWidth);
        pwmPulseWidth = GET_PWM_PULSE(pidPulseWidth);
        pwmPulseWidth = PWM_CHECK_UTHLD(pwmPulseWidth);
        pwmPulseWidth = PWM_CHECK_LTHLD(pwmPulseWidth);
        // Reset pid-regulator status
        pidStatus = PID_IDLE;
        //__enable_irq();
    }
}


/*
 * @brief Conversion complete callback in non-blocking mode
 * @param hadc: pointer to the ADC handle
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint16_t        pwmUpperPulseWidth;
    uint16_t        pwmLowerPulseWidth;

    //if (startUpDelay == 1) return;
    adcStatus = CONVERSION_COMPLETE;
    pwmUpperPulseWidth = pwmPulseWidth;
    pwmLowerPulseWidth = pwmPulseWidth + 2 * PWM_DEAD_TIME;

    switch (supplyStatus)
    {
    case SUPPLY_FAILURE:
        ledBlinkDly = 5000;
        if (devOpMode == HAULT)
        {
            supplyStatus = SUPPLY_FAILURE;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        }
        else if (adcReadings.values.hiPowSupVolt > POW_HYST_UTHLD_CODE)
        {
            activeUpperChannel = TIM_CHANNEL_4;
            ledBlinkDly = 10000;
            pidVLoopSetPoint = PID_VLP_BRT_SP_CODE;
            pidCLoopSetPoint = PID_CLP_BRT_SP_CODE;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            supplyStatus = HI_POW_SUPPLY;
            devOpMode = START_UP;
        }
        else if (adcReadings.values.loPowSupVolt > POW_HYST_UTHLD_CODE)
        {
            activeUpperChannel = TIM_CHANNEL_1;
            ledBlinkDly = 30000;
            pidVLoopSetPoint = PID_VLP_DIM_SP_CODE;
            pidCLoopSetPoint = PID_CLP_DIM_SP_CODE;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            supplyStatus = LO_POW_SUPPLY;
            devOpMode = START_UP;
        }
    break;

    case HI_POW_SUPPLY:
        if ((devOpMode == HAULT) ||
            (adcReadings.values.hiPowSupVolt < POW_HYST_LTHLD_CODE))
        {
            supplyStatus = SUPPLY_FAILURE;
        }
        __HAL_TIM_SET_COMPARE(&htim3, activeUpperChannel, pwmUpperPulseWidth);
        __HAL_TIM_SET_COMPARE(&htim3, activeLowerChannel, pwmLowerPulseWidth);
    break;

    case LO_POW_SUPPLY:
        if ((devOpMode == HAULT) ||
            (adcReadings.values.loPowSupVolt < POW_HYST_LTHLD_CODE))
        {
            supplyStatus = SUPPLY_FAILURE;
        }
        __HAL_TIM_SET_COMPARE(&htim3, activeUpperChannel, pwmUpperPulseWidth);
        __HAL_TIM_SET_COMPARE(&htim3, activeLowerChannel, pwmLowerPulseWidth);
    break;
    }
}

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
	  HAL_NVIC_SystemReset();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

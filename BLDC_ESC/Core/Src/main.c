#include "main.h"
#include <math.h>
#include <stdint.h>
#include "LCD.h"
#include <stdio.h>
#include "Timer.h"


#define TWO_PI      (2.0f * M_PI)
#define PHASE_SHIFT (TWO_PI / 3.0f)
#define SQRT3_OVER_2 0.86602540378f
#define ADC_EOC 0x01
#define STARTUP 0x02
#define MIN_DTHETA (5.0f * M_PI / 180.0f)
#define MAX_DTHETA (20.0f * M_PI / 180.0f)
#define MIN_T 1
#define MAX_T 5


unsigned short flags = STARTUP;

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

static void ComputeSVPWMDuties(float theta, float magnitude, uint16_t *pdutyA, uint16_t *pdutyB, uint16_t *pdutyC);
static void SetDutyCycles(uint16_t dutyA, uint16_t dutyB, uint16_t dutyC);
static void Recalculate_Update_Time(float mv_input, uint16_t *update_time, float *dtheta);

// ADC conversion completion interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1A)
{
	//ADC_HandleTypeDef *hadcA = hadc1;
	flags |= ADC_EOC;
	HAL_ADC_Stop_IT(hadc1A);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIMER2_HANDLE();
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  HAL_TIM_MspPostInit(&htim1);

  LcdInit();
  LcdClear();
  LcdPutS("Voltmeter INT");
  LcdGoto(1, 7);
  LcdPutS("mV");
  LcdWriteCmd(0x0C);  // Cursor off

  HAL_ADC_Init(&hadc1);
  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start_IT(&htim2);  // Start timer 2 interrupt

  uint32_t adcResult;
  char buff[10];
  float mv;

  const uint32_t channels[3] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 };
  for (int i = 0; i < 3; ++i) {
      uint32_t ch = channels[i];
      HAL_TIM_PWM_Start   (&htim1,   ch);   // high-side
      HAL_TIMEx_PWMN_Start(&htim1,   ch);   // low-side complement
  }

  float dtheta = MIN_DTHETA;
  float theta = 0.0f;
  uint16_t dutyA, dutyB, dutyC;
  uint16_t update_time = 1000;

  while (1)
  {
	  if (flags & STARTUP) {
		  if (update_time <= 2 * MAX_T) {
			  update_time = MAX_T;
			  flags &= ~STARTUP;
		  }

		  if (update_time <= 100)
			  update_time -= 5;
		  else
			  update_time -= 50;
	  }

	  if (sTimer[RECALCULATE_PWM_UPDATE_TIMER] == 0 && ((flags & ~STARTUP) ^ STARTUP)) {
		  Recalculate_Update_Time(mv, &update_time, &dtheta);
		  sTimer[RECALCULATE_PWM_UPDATE_TIMER] = RECALCULATE_PWM_UPDATE_TIME;
	  }

	  if (sTimer[UPDATE_PWM_TIMER] == 0) {
		  theta += dtheta;
		  if (theta >= TWO_PI)
			  theta = theta - TWO_PI;

		  ComputeSVPWMDuties(theta, 0.9f, &dutyA, &dutyB, &dutyC);

		  SetDutyCycles(dutyA, dutyB, dutyC);

		  sTimer[UPDATE_PWM_TIMER] = update_time;
	  }

	  if (flags & ADC_EOC)
	  {
		  //HAL_ADC_Stop_IT(&hadc1);
		  adcResult = HAL_ADC_GetValue(&hadc1);		// get ADC value
		  mv = ((float)adcResult) * 3300.0 / 4095.0;	// Convert to milli Volt
		  sprintf(buff, "%7.2f", mv);
		  LcdGoto(1, 0);
		  LcdPutS(buff);
		  flags &= ~ADC_EOC;
		  HAL_ADC_Start_IT(&hadc1);					// Restart ADC
	  }
  }
}

static void ComputeSVPWMDuties(float theta, float magnitude, uint16_t *pdutyA, uint16_t *pdutyB, uint16_t *pdutyC)
{
	// Inverse Park to generate alpha and beta components from angle with vd=m, vq=0
	float v_alpha =  magnitude * cosf(theta);
	float v_beta  =  magnitude * sinf(theta);

	// Inverse Clarke to get raw phases
	float va =  v_alpha;
	float vb = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
	float vc = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

	// shift all three so that the min goes to -1 and max to 1
	float vmax = fmaxf(fmaxf(va, vb), vc);
	float vmin = fminf(fminf(va, vb), vc);
	float v0   = 0.5f * (vmax + vmin);
	va -= v0;
	vb -= v0;
	vc -= v0;

	// Scale [–1, +1] to duty [0, 100]
	// duty = (voltage_fraction + 1)/2 * 100
	*pdutyA = (uint16_t)((va + 1.0f) * 50.0f);
	*pdutyB = (uint16_t)((vb + 1.0f) * 50.0f);
	*pdutyC = (uint16_t)((vc + 1.0f) * 50.0f);
}

static void SetDutyCycles(uint16_t dutyA_pct, uint16_t dutyB_pct, uint16_t dutyC_pct)
{
    // Read the auto-reload (ARR) so we know full-scale
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    // Compute compare values
    uint16_t cmpA = (uint16_t)((arr * dutyA_pct) / 100.0f);
    uint16_t cmpB = (uint16_t)((arr * dutyB_pct) / 100.0f);
    uint16_t cmpC = (uint16_t)((arr * dutyC_pct) / 100.0f);

    // Write them into TIM1 channels 1–3
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmpA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmpB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmpC);
}

static void Recalculate_Update_Time(float mv_input,
                                    uint16_t *update_time,
                                    float *dtheta)
{
    // mv_input clamped from 0 to 3100
	mv_input -= 200.0f;
    if (mv_input < 0.0f) {
    	mv_input = 0.0f;
    	*update_time = 1000;
    	*dtheta = MIN_DTHETA;
    	flags |= STARTUP;
    	return;
    }
    else if (mv_input > 3100.0f) mv_input = 3100.0f;

    // normalize to [0, 1]
    float ratio = mv_input / 3100.0f;

    // time is proportionaly to negative mv_input
    *update_time = MAX_T - (uint16_t)(ratio * (MAX_T - MIN_T));
    // dtheta is proportional to mv_input
    *dtheta = MIN_DTHETA + ratio * (MAX_DTHETA - MIN_DTHETA);
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 19;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 20;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

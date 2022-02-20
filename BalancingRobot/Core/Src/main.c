/************************************************************************
 * Author  :Ahmed Elhamy
 * Date    :18/2/2022
 * Version :v_01
 ***********************************************************************/


#include "main.h"


I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

  /* USER CODE BEGIN 1 */
/**********************************************************************************************/
int main(void)
{

	HAL_Init();


	SystemClock_Config();

	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();

	MPU_Init();
	//i used it once to calibrate then i used the calibration values
	//so i don't need to use it again
	// MPU_Calibrate();
	PID_Init(56.25,1.2,0.0325,-500,500);

	//Initialize PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//Set Motor driver enable pins
	HAL_GPIO_WritePin(Motor1EN_GPIO_Port,Motor1EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor2EN_GPIO_Port,Motor2EN_Pin,GPIO_PIN_SET);
	//Set Motor driver direction pins
	HAL_GPIO_WritePin(Motor1Dir1_GPIO_Port,Motor1Dir1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor1Dir2_GPIO_Port,Motor1Dir2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor2Dir1_GPIO_Port,Motor2Dir1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor2Dir2_GPIO_Port,Motor2Dir2_Pin,GPIO_PIN_RESET);

	float AngleY,GyroY;
	int32_t PIDOutput=0;
	uint32_t start=HAL_GetTick();

	while (1)
	{

	   MPU_Read();
       MPU_GetAngle(&AngleY,MPU_Y);
       MPU_GetGYROValue(&GyroY,MPU_Y);
       PIDOutput=PID_Calc(AngleY, GyroY,HAL_GetTick()-start);
       if(PIDOutput<0)
       {
    		  HAL_GPIO_WritePin(Motor1Dir1_GPIO_Port,Motor1Dir1_Pin,GPIO_PIN_SET);
    		  HAL_GPIO_WritePin(Motor1Dir2_GPIO_Port,Motor1Dir2_Pin,GPIO_PIN_RESET);

    		  HAL_GPIO_WritePin(Motor2Dir1_GPIO_Port,Motor2Dir1_Pin,GPIO_PIN_SET);
    		  HAL_GPIO_WritePin(Motor2Dir2_GPIO_Port,Motor2Dir2_Pin,GPIO_PIN_RESET);
    		  PIDOutput=-PIDOutput;
       }
       else
       {
    		  HAL_GPIO_WritePin(Motor1Dir1_GPIO_Port,Motor1Dir1_Pin,GPIO_PIN_RESET);
    		  HAL_GPIO_WritePin(Motor1Dir2_GPIO_Port,Motor1Dir2_Pin,GPIO_PIN_SET);

    		  HAL_GPIO_WritePin(Motor2Dir1_GPIO_Port,Motor2Dir1_Pin,GPIO_PIN_RESET);
    		  HAL_GPIO_WritePin(Motor2Dir2_GPIO_Port,Motor2Dir2_Pin,GPIO_PIN_SET);
       }
       TIM3->CCR1=(uint32_t)PIDOutput;
       TIM3->CCR2=(uint32_t)PIDOutput;
       printf("%f,%f\r\n",AngleY,GyroY);
	}



}


int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}



void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }


}

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor1EN_Pin|Motor2EN_Pin|LD2_Pin|Motor1Dir1_Pin
                          |Motor1Dir2_Pin|Motor2Dir1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor2Dir2_GPIO_Port, Motor2Dir2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1EN_Pin Motor2EN_Pin LD2_Pin Motor1Dir1_Pin
                           Motor1Dir2_Pin Motor2Dir1_Pin */
  GPIO_InitStruct.Pin = Motor1EN_Pin|Motor2EN_Pin|LD2_Pin|Motor1Dir1_Pin
                          |Motor1Dir2_Pin|Motor2Dir1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor2Dir2_Pin */
  GPIO_InitStruct.Pin = Motor2Dir2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor2Dir2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor2Dir2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET);


}

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



void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  //500Hz= 90M/(2*500*(179+1))
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 179;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 400;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}
/**********************************************************************************************/
/* USER CODE END 1 */

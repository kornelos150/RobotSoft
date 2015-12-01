/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SHORT_PI 3.1415f
#define RADIUS 1.95f
#define TICK_PER_ROUND 600.0f
#define ENCODER_INITIAL 32500
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void LeftForward(void);
void LeftBackward(void);
void RightForward(void);
void RightBackward(void);
void RightPWM(uint32_t pwm);
void LeftPWM(uint32_t pwm);
void regulation(void);
int8_t getBit(int8_t value, int8_t bit);
void setBit(int8_t *value, int8_t bit);
void resetBit(int8_t *value, int8_t bit);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int8_t command[10];

float leftVelocity = 0;
float rightVelocity = 0;
int32_t leftCurrentTicks = 0;
int32_t rightCurrentTicks = 0;

int32_t leftTotalTicks = 0;
int32_t rightTotalTicks = 0;

float pVal = 5.0f;
float dVal = 0;

float setLeftSpeed = 0;
float setRightSpeed = 0;

float eLeft = 0;
float eRight = 0;

float peLeft = 0;
float peRight = 0;

float newPWMLeft = 0;
float newPWMRight = 0;

uint32_t encoderTimer = 1000;
uint32_t btTimeout = 0;
uint8_t speedTimerFlag = 0;
uint8_t bluetoothTimerFlag = 0;

uint8_t ControlFlag = 0;
uint8_t timerFlag = 0;

//uint8_t response[10];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  htim1.Instance->CNT = ENCODER_INITIAL;
  htim3.Instance->CNT = ENCODER_INITIAL;

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  //to je PWM
  //htim4.Instance->CCR3 = 1000;
  //htim4.Instance->CCR2 = 1000;

  leftTotalTicks = 0;
  rightTotalTicks = 0;

  LeftPWM(0);
  RightPWM(0);

  RightForward();
  LeftForward();

  htim9.Instance->ARR = 0;
  htim2.Instance->ARR = encoderTimer;
  htim2.Instance->CNT = encoderTimer;
  HAL_TIM_Base_Start_IT(&htim2);

  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);



  HAL_UART_Receive_DMA(&huart2, command, sizeof(command));

  //uint8_t hello[] = {'r','o','b','o','t',' ','i','n','i','t'};
  //HAL_UART_Transmit_DMA(&huart2,hello,10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 65000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim1, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 65000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4200;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

}

/* TIM5 init function */
void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8400;
  htim5.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim5);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

}

/* TIM9 init function */
void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim9);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

}

/* TIM10 init function */
void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim10);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void setTimer(TIM_HandleTypeDef *htim, uint32_t timeout)
{
	htim->Instance->ARR = timeout*10;
	if(htim->Init.CounterMode == TIM_COUNTERMODE_DOWN)
		htim->Instance->CNT = timeout*10-1;
	else
		htim->Instance->CNT = 0;
}

uint32_t getTimerTimeout(TIM_HandleTypeDef *htim)
{
	return htim->Instance->ARR;
}

void startSpeedTimer(uint32_t timeout)
{

		htim5.Instance->ARR = timeout*10;
		htim5.Instance->CNT = timeout*10-1;
		HAL_TIM_Base_Start_IT(&htim5);
		speedTimerFlag = 0;

}

void stopSpeedTimer()
{
	HAL_TIM_Base_Stop_IT(&htim5);
}

void startBluetoothTimer()
{

		htim9.Instance->CNT = 0;
		HAL_TIM_Base_Start_IT(&htim9);
		bluetoothTimerFlag = 0;

}

void stopBluetoothTimer()
{
	HAL_TIM_Base_Stop_IT(&htim9);
}

void LeftForward(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
}

void LeftBackward(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
}

void RightForward(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
}

void RightBackward(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
}

uint8_t isLeftBackward(void)
{
	uint8_t result;
	if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4) == GPIO_PIN_RESET)  &&
	   (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5) == GPIO_PIN_SET))
		result = 1;
	else
		result = 0;
	return result;
}

uint8_t isLeftForward(void)
{
	uint8_t result;
	if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4) == GPIO_PIN_SET)  &&
	   (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5) == GPIO_PIN_RESET))
		result = 1;
	else
		result = 0;
	return result;
}

uint8_t isRightBackward(void)
{
	uint8_t result;
	if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_RESET)  &&
	   (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == GPIO_PIN_SET))
		result = 1;
	else
		result = 0;
	return result;
}

uint8_t isRightForward(void)
{
	uint8_t result;
	if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_SET)  &&
	   (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == GPIO_PIN_RESET))
		result = 1;
	else
		result = 0;
	return result;
}
void LeftPWM(uint32_t pwm)
{
	if(pwm > 4200)
		pwm = 4200;
	htim4.Instance->CCR2 = pwm;
}

void RightPWM(uint32_t pwm)
{
	if(pwm > 4200)
		pwm = 4200;
	htim4.Instance->CCR3 = pwm;
}

uint32_t getLeftPWM()
{
	return htim4.Instance->CCR2;
}

uint32_t getRightPWM()
{
	return htim4.Instance->CCR3;
}


void stopMotors(void)
{
	/*
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	*/
	setLeftSpeed = 0.0;
	setRightSpeed = 0.0;

	LeftPWM(0);
	RightPWM(0);

	HAL_TIM_Base_Stop_IT(&htim5);

}

float calcVelocity(int8_t encoderTicks)
{
	float tmp = encoderTicks/TICK_PER_ROUND;
	tmp = tmp*2.0*SHORT_PI;
	tmp = tmp /(getTimerTimeout(&htim2)*0.0001);
	tmp = tmp*RADIUS;
	return tmp;
}

float LeftVelocity(void)
{

	leftTotalTicks += (ENCODER_INITIAL - htim3.Instance->CNT);
	htim3.Instance->CNT = ENCODER_INITIAL;

	float vel = 0.0f;
	vel = calcVelocity(leftTotalTicks);
	leftCurrentTicks = leftTotalTicks;
	leftTotalTicks = 0;

	return vel;
}

float RightVelocity(void)
{
	rightTotalTicks += (ENCODER_INITIAL - htim1.Instance->CNT);
	htim1.Instance->CNT = ENCODER_INITIAL;

	float vel = 0.0f;
	vel = calcVelocity(rightTotalTicks);
	rightCurrentTicks = rightTotalTicks;
	rightTotalTicks = 0;

	return vel;
}


int8_t* float2byte(float input, int8_t* output)
{
	input = round(input*100)/100.0f;
	float upper,leftover;
	if(input >= 0)
	{
		upper = floorf(input);
		leftover = (input - upper) * 100.0f;
	}
	else
	{
		upper = ceilf(input);
		leftover = -(input - upper) * 100.0f;
	}

	output[0] = (int8_t)upper;
	output[1] = (int8_t)leftover;
	if(input < 0 && input > -1)
		output[1] = -output[1];


	return output;
}

float byte2Float(int8_t* output)
{
	if(output[0] > -1)
		return output[0] + output[1]/100.0f;
	else
		return output[0] - output[1]/100.0f;
}

uint32_t byte2int(int8_t* input)
{
	uint32_t high = input[1] >= 0 ? input[1] : 256 + input[1];
	uint32_t low = input[0] >= 0 ? input[0] : 256 + input[0];

	return low | (high << 8);
}

int8_t* int2byte(uint32_t input,int8_t* output )
{
	output[0] = (int8_t)(input & 0xFF);
	output[1] = (int8_t)((input >> 8) & 0xFF);

	return output;
}

int8_t* invalidMessage(int8_t* response)
{
	uint8_t index = 0;
	for(;index < 10; ++index)
		response[index] = 0;
	response[0] = 'B';
	response[4] = 'M';
	response[9] = 'E';
	stopMotors();
	return response;
}

int8_t* setResponse(int8_t* command,int8_t* response)
{
	uint8_t index = 0;
	for(;index < 10; ++index)
		response[index] = command[index];
	response[0] = 'B';
	response[1] = command[1];
	return response;
}

int8_t* setResponseWrkd(int8_t* command,int8_t* response)
{
	uint8_t index = 0;
	for(;index < 10; ++index)
		response[index] = 0;
	response[0] = 'B';
	response[1] = command[1];
	response[2] = command[2];
	response[3] = command[3];
	response[4] = command[5];
	response[5] = command[6];
	return response;
}

int8_t* setIntResponseWrkd(int8_t* command,int8_t* response)
{
	uint8_t index = 0;
	for(;index < 10; ++index)
		response[index] = command[index];

	return response;
}

int8_t* setBlockedTimerResponse(int8_t* response)
{
	uint8_t index = 0;
	for(;index < 10; ++index)
		response[index] = timerFlag;

	return response;
}

int8_t* getVelocityResponse(int8_t* command,int8_t* response)
{
	setResponse(command,response);
	int8_t data[2];
	float2byte(leftVelocity,data);
	response[2] = data[0];
	response[3] = data[1];
	float2byte(rightVelocity,data);
	response[4] = data[0];
	response[5] = data[1];

	return response;

}

int8_t* setVelocityResponse(int8_t* command,int8_t* response,uint8_t isBlocked)
{
	setResponseWrkd(command,response);
	if(command[2] == 0 && command[3] == 0 && command[5] == 0 && command[6] == 0)
	{
		stopMotors();
	}
	else
	{
		int8_t data[2];

		data[0] = command[2];
		data[1] = command[3];
		setLeftSpeed = byte2Float(data);

		data[0] = command[5];
		data[1] = command[6];
		setRightSpeed = byte2Float(data);

		if(command[7] != 0 && command[8] != 0)
		{
			data[0] = command[7];
			data[1] = command[8];
			uint32_t timer = byte2int(data);

			startSpeedTimer(timer);
			if(isBlocked)
				timerFlag = command[1];
		}
	}
	return response;
}


int8_t* setConfigResponse(int8_t* command,int8_t* response)
{
	setResponseWrkd(command,response);

	int8_t data[2];

	data[0] = command[2];
	data[1] = command[3];
	pVal = byte2Float(data);

	data[0] = command[5];
	data[1] = command[6];
	dVal = byte2Float(data);


	return response;
}



int8_t* getConfigResponse(int8_t* command,int8_t* response)
{
	setResponse(command,response);
	int8_t data[2];
	float2byte(pVal,data);
	response[2] = data[0];
	response[3] = data[1];
	float2byte(dVal,data);
	response[4] = data[0];
	response[5] = data[1];

	return response;

}

int8_t* setPWMResponse(int8_t* command,int8_t* response, uint8_t isBlocked)
{
	setResponseWrkd(command,response);

	if(!ControlFlag)
	{
		int8_t data[2];

		data[0] = command[2];
		data[1] = command[3];
		LeftPWM(byte2int(data));

		data[0] = command[5];
		data[1] = command[6];
		RightPWM(byte2int(data));

		if(command[7] != 0 && command[8] != 0)
		{
			data[0] = command[7];
			data[1] = command[8];
			uint32_t timer = byte2int(data);

			startSpeedTimer(timer);
			if(isBlocked)
				timerFlag = command[1];
		}
	}

	return response;
}

int8_t* getPWMResponse(int8_t* command,int8_t* response)
{
	setResponse(command,response);
	int8_t data[2];
	int2byte(getLeftPWM(),data);
	response[2] = data[0];
	response[3] = data[1];
	int2byte(getRightPWM(),data);
	response[4] = data[0];
	response[5] = data[1];

	return response;

}

int8_t* setTimerResponse(int8_t* command,int8_t* response,TIM_HandleTypeDef *htim)
{
	setResponseWrkd(command,response);

	int8_t data[2];

	data[0] = command[2];
	data[1] = command[3];
	setTimer(htim,byte2int(data));

	return response;
}

int8_t* getTimerResponse(int8_t* command,int8_t* response,TIM_HandleTypeDef *htim)
{
	setResponse(command,response);
	int8_t data[2];
	int2byte(htim->Instance->ARR,data);
	response[2] = data[0];
	response[3] = data[1];

	return response;

}

int8_t* getSetSpeedResponse(int8_t* command,int8_t* response)
{
	setResponse(command,response);
	int8_t data[2];
	float2byte(setLeftSpeed,data);
	response[2] = data[0];
	response[3] = data[1];
	float2byte(setRightSpeed,data);
	response[4] = data[0];
	response[5] = data[1];

	return response;

}

int8_t* setRegulationTimerResponse(int8_t* command,int8_t* response)
{
	setResponseWrkd(command,response);

	if(command[2] == 0 && command[3] == 0)
	{
		setTimer(&htim10,0);
		HAL_TIM_Base_Stop_IT(&htim10);
		ControlFlag = 0;
	}else
	{
		int8_t data[2];

		data[0] = command[2];
		data[1] = command[3];
		setTimer(&htim10,byte2int(data));
		HAL_TIM_Base_Start_IT(&htim10);
		ControlFlag = 1;
	}


	return response;
}


int8_t* setMotorsDirection(int8_t* command,int8_t* response)
{
	setResponseWrkd(command,response);
	if(!ControlFlag)
	{
		if(getBit(command[2],3))
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);

		if(getBit(command[2],2))
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);

		if(getBit(command[2],1))
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

		if(getBit(command[2],0))
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	}
	return response;
}

int8_t* getMotorsDirection(int8_t* command,int8_t* response)
{
	setResponse(command,response);

	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4) == GPIO_PIN_SET)
		setBit(&(response[2]),3);
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5) == GPIO_PIN_SET)
			setBit(&(response[2]),2);
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_SET)
			setBit(&(response[2]),1);
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == GPIO_PIN_SET)
			setBit(&(response[2]),0);

	return response;

}

int8_t* getEncoderTicks(int8_t* command,int8_t* response)
{
	setResponse(command,response);
	int8_t data[2];
	int2byte(leftCurrentTicks,data);
	response[2] = data[0];
	response[3] = data[1];
	int2byte(rightCurrentTicks,data);
	response[4] = data[0];
	response[5] = data[1];

	return response;

}


int8_t* commandHandler(int8_t* command,int8_t* response)
{
	if(command[0] != 'B' || command[4] != 'M' || command[9] != 'E')
		return invalidMessage(response);
	if(command[1] == 1)
	{
		//set speed
		return setVelocityResponse(command,response,0);
	}
	if(command[1] == 2)
	{
		//get speed
		return getVelocityResponse(command,response);
	}
	if(command[1] == 3)
	{
		//set configration
		return setConfigResponse(command,response);
	}
	if(command[1] == 4)
	{
		//get configuration
		return getConfigResponse(command,response);
	}
	if(command[1] == 5)
	{
		//set timeout
		return setTimerResponse(command,response,&htim9);
	}
	if(command[1] == 6)
	{
		//get timeout
		return getTimerResponse(command,response,&htim9);
	}
	if(command[1] == 7)
	{
		//set PWM
		return setPWMResponse(command,response,0);
	}
	if(command[1] == 8)
	{
		//get PWM
		return getPWMResponse(command,response);
	}
	if(command[1] == 9)
	{
		//set encoder timer
		return setTimerResponse(command,response,&htim2);
	}
	if(command[1] == 10)
	{
		//get encoder timer
		return getTimerResponse(command,response,&htim2);
	}
	if(command[1] == 11)
	{
		//get set speed
		return getSetSpeedResponse(command,response);
	}
	if(command[1] == 12)
	{
		//set regulation timer
		return setRegulationTimerResponse(command,response);
	}
	if(command[1] == 13)
	{
		//get regulation timer
		return getTimerResponse(command,response,&htim10);
	}
	if(command[1] == 14)
	{
		//set direction pins
		return setMotorsDirection(command,response);
	}
	if(command[1] == 15)
	{
		//get direction pins
		return getMotorsDirection(command,response);
	}
	if(command[1] == 16)
	{
		//set blocked velocity
		return setVelocityResponse(command,response,1);
	}
	if(command[1] == 17)
	{
		//set blocked PWM
		return setPWMResponse(command,response,1);
	}
	if(command[1] == 18)
	{
		//get ticks in current velocity measurement
		return getEncoderTicks(command,response);
	}



return invalidMessage(response);


}

void onEncoderOverload(TIM_HandleTypeDef *htim,int32_t *totalTickCounter)
{
	if(htim->Instance->CNT > 64000 && htim->Instance->CNT < 65000)
		*totalTickCounter += (ENCODER_INITIAL + 65000 - htim->Instance->CNT);
	else if (htim->Instance->CNT > 0 && htim->Instance->CNT < 1000)
		*totalTickCounter -= (ENCODER_INITIAL + htim->Instance->CNT);

	htim->Instance->CNT = ENCODER_INITIAL;
	HAL_TIM_Base_Start_IT(htim);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int8_t response[10];
	int8_t currentCommand[10];
	uint8_t index = 0;
	for(;index < 10; ++index)
		currentCommand[index] = command;

	if(getTimerTimeout(&htim9))
		stopBluetoothTimer();

	//setTimer(0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	commandHandler(currentCommand,response);
	HAL_UART_Transmit_DMA(&huart2,response,10);
	if(getTimerTimeout(&htim9))
		startBluetoothTimer();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		leftVelocity = LeftVelocity();
		rightVelocity = RightVelocity();
		htim2.Instance->CNT = encoderTimer - 1;
		HAL_TIM_Base_Start_IT(&htim2);
	}

	if(htim == &htim5)
	{
			stopMotors();
			HAL_TIM_Base_Stop_IT(&htim5);
			if(timerFlag != 0)
			{
				int8_t response[10];
				setBlockedTimerResponse(response);
				timerFlag = 0;
				HAL_UART_Transmit_DMA(&huart2,response,10);
			}
	}

	if(htim == &htim9)
	{
		//if(bluetoothTimerFlag != 0)
		//{
			stopMotors();
			HAL_TIM_Base_Stop_IT(&htim9);
		//}else
		//{
		//	bluetoothTimerFlag = 1;
		//}
	}
	if(htim == &htim10)
	{
		regulation();
		HAL_TIM_Base_Start_IT(&htim10);
	}
	if(htim == &htim3)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		onEncoderOverload(&htim3,&leftTotalTicks);
	}
	if(htim == &htim1)
	{
		onEncoderOverload(&htim1,&rightTotalTicks);
	}
}


void addNewPWM(float leftPWM, float rightPWM)
{
	int32_t toSetLeft = getLeftPWM();
	int32_t toSetRight = getRightPWM();

	if(isLeftBackward())
		toSetLeft = -toSetLeft;

	if(isRightBackward())
		toSetRight = -toSetRight;

	toSetLeft += (int32_t)leftPWM;
	toSetRight += (int32_t)rightPWM;

	if(toSetLeft < 0)
	{
		toSetLeft = - toSetLeft;
		LeftBackward();
	}else
		LeftForward();

	if(toSetRight < 0)
	{
		toSetRight = -toSetRight;
		RightBackward();
	}else
		RightForward();

	LeftPWM((uint32_t)toSetLeft);
	RightPWM((uint32_t)toSetRight);
}

void regulation(void)
{
	eLeft = setLeftSpeed - leftVelocity;
	eRight = setRightSpeed - rightVelocity;

	newPWMLeft = pVal * eLeft + dVal * (eLeft - peLeft);
	newPWMRight = pVal * eRight + dVal * (eRight - peRight);

	peLeft = eLeft;
	peRight = eRight;

	addNewPWM(newPWMLeft,newPWMRight);

}

int8_t getBit(int8_t value, int8_t bit)
{
	return (value & ( 1 << bit )) >> bit;
}
void setBit(int8_t *value, int8_t bit)
{
	*value |= 1 << bit;
}

void resetBit(int8_t *value, int8_t bit)
{
	*value &= ~(1 << bit);
}


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

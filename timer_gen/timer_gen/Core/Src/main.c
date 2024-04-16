/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <math.h>  // M_PI, sin
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_tim.h>

#define SAMPLE_RATE 42000
#define BUFFER_SIZE 1024
#define FREQUENCY   440


DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

TIM_HandleTypeDef htim3;


/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_DAC_Init(void);

void setup();
void errorLed(int red, int orange);
void checkBreakpoint(int step);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();
	SystemClock_Config();

	/* Initialize DMA,TIM3,GPIO peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM3_Init();
	MX_DAC_Init();
	setup();
	
	checkBreakpoint(1);

	 uint16_t buffers[2][BUFFER_SIZE]; // New: add a second buffer.
	 uint8_t curr = 0;                 // Index of current buffer.
	 uint32_t t   = 0;

	 // Start the timer.
	 HAL_TIM_Base_Start(&htim3);

	 // Optimization: precompute constant.
     float two_pi_f_over_sr = 2 * 3.14159 * FREQUENCY / SAMPLE_RATE;

	 while (1) {
  checkBreakpoint(2);
     	uint16_t* buffer = buffers[curr]; // Get the buffer being written.
	 	// Prep the buffer.
	 	for (int i = 0; i < BUFFER_SIZE; i++, t++) {
	 		buffer[i] = 2047 * sin(two_pi_f_over_sr * t) + 2047;
	 	}
checkBreakpoint(3);
	 	// Wait for DAC to be ready, so that the buffer can be modified on the next iteration.
	 	while (HAL_DAC_GetState(&hdac) != HAL_DAC_STATE_READY){};
checkBreakpoint(4);
	 	// Start the DMA.
	 	 HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)buffer, BUFFER_SIZE, DAC_ALIGN_12B_R); //breaking code
// checkBreakpoint(5);
	 	// Point to the other buffer, so that we
	 	// prepare it while the previous one
	 	// is being sent.
	 	curr = !curr;
	 }
		
}

/**
 * Blinks blue LED to signify breakpoint step (whether code reaches that step or not)
 */
void checkBreakpoint(int step)
{
  HAL_Delay(500);
  for (int i = 0; i < step; i++)
  {
    GPIOC->ODR |= (1 << 7); // blue LED debugging
    HAL_Delay(200);
    GPIOC->ODR &= ~(1 << 7);
    HAL_Delay(200);
  }
  HAL_Delay(500);
}

void setup(){
// Enable the system clock for the C peripheral
RCC->AHBENR |= (1 << 19);

// Enable the system clock for the A peripheral 
RCC->AHBENR |= (1 << 17);

//configure the USER button pin (PA0) to input mode (Clears the 1st and 2nd bits in the GPIOA_MODER register
GPIOA->MODER &= ~(0b00000001);
GPIOA->MODER &= ~(0b00000010);


//configure the USER button pin to low speed
GPIOA->OSPEEDR &= ~(0b00000001);

//Enable the pull-down resistor for the USER button pin
GPIOA->PUPDR |= (1 << 1);
GPIOA->PUPDR &= ~(0b00000001);

//configure the LEDs Pins 
GPIOC->MODER |= (1 <<12); //setting PC6 to general output 
GPIOC->MODER |= (1 <<14); //setting PC7 to general output 
GPIOC->MODER |= (1 <<16); //setting PC8 to general output
GPIOC->MODER |= (1 <<18); //setting PC9 to general output

GPIOC->OTYPER |= (0 << 7);//setting PC6 to push/pull output 
GPIOC->OTYPER |= (0 << 8);//setting PC7 to push/pull output 
GPIOC->OTYPER |= (0 << 9);//setting PC8 to push/pull output
GPIOC->OTYPER |= (0 << 10);//setting PC9 to push/pull output

GPIOC->OSPEEDR |= (0 <<12); //setting PC6 to low speed
GPIOC->OSPEEDR |= (0 <<14); //setting PC7 to low speed
GPIOC->OSPEEDR |= (0 <<16); //setting PC8 to low speed
GPIOC->OSPEEDR |= (0 <<18); //setting PC9 to low speed

GPIOC->PUPDR |= (0 <<12); //setting PC6 to to no pull-up/down resistors
GPIOC->PUPDR |= (0 <<14); //setting PC7 to to no pull-up/down resistors
GPIOC->PUPDR |= (0 <<16); //setting PC8 to to no pull-up/down resistors
GPIOC->PUPDR |= (0 <<18); //setting PC9 to to no pull-up/down resistors

// Setting Pins initial states
//GPIOC->ODR |= (0 << 6); //setting pin 6 to high
//GPIOC->ODR |= (0 << 7); //setting pin 7 to high
//GPIOC->ODR |= (0 << 8); //setting pin 8 to high
//GPIOC->ODR |= (1 << 9); //setting pin 9 to high


}

/**
 * Optional error checking with red and orange LEDs
 */
void errorLed(int red, int orange)
{
  if (red)
  {
    GPIOC->ODR |= (1 << 6); // error, turn on red LED
    HAL_Delay(200);         // leave it on for 0.2 seconds
    GPIOC->ODR &= ~(1 << 6);
  }
  if (orange)
  {
    GPIOC->ODR |= (1 << 8); // error, turn on red LED
    HAL_Delay(200);         // leave it on for 0.2 seconds
    GPIOC->ODR &= ~(1 << 8);
  }
}



/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T3_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 189;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable the system clock for the GPIOC peripheral
	// setting PA4 to analog
  GPIOA->MODER |= (1 << 8); // setting 9th
  GPIOA->MODER |= (1 << 9); // setting 10th

  // setting PA4 to to no pull-up/down resistors
  GPIOA->PUPDR &= ~(1 << 9); // clearing 9th bit
  GPIOA->PUPDR &= ~(1 << 10); // clearing 10th bit

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
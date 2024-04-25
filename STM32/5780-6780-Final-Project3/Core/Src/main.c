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
#include "main.h"
#include "stdlib.h"
#include "usart.h"
#include "i2c.h"
#include "stm32f0xx.h"  // Device header
#include <stdio.h>
#include <string.h>

#include <math.h>
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_tim.h>
#include <ctype.h>

// UART receive defines
volatile char receivedData;
volatile char newDataAvailable;
#define MaxBufferSize 100
volatile char input_buffer[MaxBufferSize - 1];  //a buffer with some defined maximum space
static char received_buffer[MaxBufferSize];

// DAC2DMA private defines
#define pi 3.14155926
#define MAX_SAMPLES 100
#define res_8b 256
#define res_12b 4096

 /* Modulation 1
#define FREQ1 554.37 // C5#
#define FREQ2 659.25 // E5 */

 /* Modulation 2
#define FREQ1 440.00 // A4
#define FREQ2 123.47 // B2 */

// /* Modulation 3
#define FREQ1 1244.51 // D6#
#define FREQ2 2349.32 // D7 */


// Private variables
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;
TIM_HandleTypeDef htim7;


// CubeMX Generated functions
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM7_Init(void);


// User defined functions
void initLEDs(void);
void DAC2DMA_wave(void);
void initTIM(void);
void initADC(void);

void init_digiPot(int c);

	
// User define sine wave
uint32_t sine_val[MAX_SAMPLES];
void get_sineval() {
	for (int i = 0; i < MAX_SAMPLES; i++) {
		 /* General Sine Wave of 420 Hz
		sine_val[i] = (sin(i * 2 * pi / MAX_SAMPLES) + 1) * res_12b / 2; // */
				
		// /* Signal modulation
		float a = (sin(i * 2 * pi / MAX_SAMPLES) + 1); // Sine wave of 420 Hz
		float cs = (sin(i * (FREQ1/210) * 2 * pi / MAX_SAMPLES) + 1); // Sine wave of 
		float b = (sin(i * (FREQ2/210) * 2 * pi / MAX_SAMPLES) + 1);
		float val = (a + cs + b)/3;
		sine_val[i] = val * res_12b / 2; // */
	}
}
	

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
	/* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
	
	/* Initialize LEDs */
  initLEDs();
	
	/* Initialize timers for the intended 42 kHz target */
	initTIM();
	
	/* Initialize I2C interface */
	initI2C();
		//The digital potentiometer places the device identifier "0101" in the 
		//upper 4 bits of the initial starting byte that's sent along with the device address!
	
	//initialize USART
	USARTSetup();
	
	/* Perform DAC to DMA communication */
	DAC2DMA_wave();
	
  while (1)
  {
    transmit_string("Waiting for USART input. Please enter a number between 0 and 255.\n\r");
				
		while((USART3->ISR & USART_ISR_RXNE) != USART_ISR_RXNE) { }
		
		//transmit_char(newDataAvailable);
		if(newDataAvailable) {
			
			//use strtol to translate the input string to a long
			long convertedData = strtol((void*)input_buffer, NULL, 10);

			if((convertedData > 255) || (convertedData < 0)) {
				transmit_string("Invalid value!\n\r");
				newDataAvailable = 0;
			}
			
			else {
				init_digiPot((int)convertedData);								//call Chase's digiPot code and send the translated data
				
				char slingshotStr[sizeof(char) + 3];						//"slingshot" the conversion back into a string for confirmation
				sprintf(slingshotStr, "%ld", convertedData);

				transmit_string("Received: ");									//provide feedback over UART so that the user knows what's happening
				transmit_string((void*)input_buffer);
				transmit_string(". Converted to: ");
				transmit_string(slingshotStr);
				transmit_string("\n\r");
			}
			
			newDataAvailable = 0;		//data transfer complete. reset status
		}
  }
} // END main


/* Initialize Timer to the Nyquist sampling rate */
void initTIM(void) {
	// Configure the timer (TIM2) to trigger an update event (UEV) at 42 kHz
	TIM2->PSC = 0x1;  // 0.9524 ~1
	TIM2->ARR = 0xBD; // 189.48 ~189
	
	// Configure the timer to generate an interrupt on the UEV event
	TIM2->DIER |= TIM_DIER_UIE;
	
	// Configure an enable/start the timer 2
	TIM2->CR1 |= TIM_CR1_CEN;
}


/* Initialize ADC */
void initADC(void){
	// setting PC0 to analog
  GPIOC->MODER |= (1 << 0); // setting 0th
  GPIOC->MODER |= (1 << 1); // setting 1st

  // setting PC0 to to no pull-up/down resistors
  GPIOC->PUPDR &= ~(1 << 0); // clearing 0th bit
  GPIOC->PUPDR &= ~(1 << 1); // clearing 1st bit
	
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable system clock for ADCEN peripheral
  
  // 8-bit
  ADC1->CFGR1 &= ~(1 << 4);
  ADC1->CFGR1 &= ~(1 << 3);
	
  // continuous conversion
  ADC1->CFGR1 |= (1 << 13);
	
  // hardware triggers disabled
  ADC1->CFGR1 &= ~(1 << 10);
	ADC1->CFGR1 &= ~(1 << 11);
	
  // configuring the channel 0
  ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	

  /* === ADC Calibration === */
  /* (1) Ensure that ADEN = 0 */
  /* (2) Clear ADEN by setting ADDIS*/
  /* (3) Clear DMAEN */
  /* (4) Launch the calibration by setting ADCAL */
  /* (5) Wait until ADCAL=0 */

  if ((ADC1->CR & ADC_CR_ADEN) != 0){     /* (1) */
    ADC1->CR |= ADC_CR_ADDIS;             /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0){
    /* For robust implementation, add here time-out management */
  }
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;        /* (3) */
  ADC1->CR |= ADC_CR_ADCAL;               /* (4) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0){ /* (5) */
    /* For robust implementation, add here time-out management */
  }

  /* (1) Ensure that ADRDY = 0 */
  /* (2) Clear ADRDY */
  /* (3) Enable the ADC */
  /* (4) Wait until ADC ready */
  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0){    /* (1) */
    ADC1->ISR |= ADC_ISR_ADRDY;             /* (2) */
  }
  ADC1->CR |= ADC_CR_ADEN;                  /* (3) */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0){ /* (4) */
    /* For robust implementation, add here time-out management */
  }
	
	// Start ADC
	ADC1->CR |= ADC_CR_ADSTART;
}


/* Interrupt Handler for USART */
void USART3_4_IRQHandler(void) {
	receivedData = USART3->RDR;
	static int received_index = 0;
	//newDataAvailable = 1;
	
	char receivedChar = (char)(USART3->RDR & 0xFF);		//filter the received character and store it temporarily
	
	if((receivedChar == '\r') || (receivedChar == '\n')) {
		if(received_index != 0) {
			memcpy((void*)input_buffer, received_buffer, received_index);			//copy the contents of the received buffer to the input buffer
		
			input_buffer[received_index] = 0;																	//terminate the string
			newDataAvailable = 1;
			
			received_index = 0;																								//reset and prepare for more data
		}
	}
	
	else {
		if(isalpha(receivedChar)) {
			transmit_string("That is not a valid input!\n\r");
		}
		
		else {
			if((receivedChar == '$') || (received_index == MaxBufferSize))	//error cases will reset the index to 0
				received_index = 0;
			
			received_buffer[received_index++] = receivedChar;								//otherwise, append data to the buffer
		}
	}
}



/* Initialize the LEDs for debugging purposes */
void initLEDs(void) {	
	/* Initialize all LEDs: RED (PC6), BLUE (PC7), ORANGE (PC8), GREEN (PC9)	*/
	// (Reset state: 00)
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	
	// (General purpose: 01) 
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	
	// Configure Push/Pull Output type for PC6, PC7, PC8, and PC9	(00)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	
	// Configure low speed for PC6, PC7, PC8, and PC9	(00)
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk | GPIO_OSPEEDR_OSPEEDR8_Msk | GPIO_OSPEEDR_OSPEEDR9_Msk);
	
	// Configure no pull-up/down resistors for PC6, PC7, PC8, and PC9	(00)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR9_Msk);
	
	// Initialize pins to logic high and the other to low.
	GPIOC->BSRR = GPIO_BSRR_BR_6;	// Set PC6 low
	GPIOC->BSRR = GPIO_BSRR_BR_7; // Set PC7 low
	GPIOC->BSRR = GPIO_BSRR_BR_8;	// Set PC8 low
	GPIOC->BSRR = GPIO_BSRR_BR_9; // Set PC9 low
}


/* Perform DAC to DMA Communication */
void DAC2DMA_wave(void) {
	// get the sine signal
	get_sineval();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	
	// begin timer
	HAL_TIM_Base_Start(&htim7);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);


	// start DAC to DMA (Then loop around to get signal)
	HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, sine_val, MAX_SAMPLES, DAC_ALIGN_12B_R);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 189-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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

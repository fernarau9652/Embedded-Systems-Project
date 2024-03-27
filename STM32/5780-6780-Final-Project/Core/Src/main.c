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

void initTIM(void);
void initDAC(void);
void initDMA(void);
void initLEDs(void);
void initI2C(void);
void initADC(void);

void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Enable GPIOA, GPIOB, GPIOC, I2C, TIM2, and TIM3 Clock in RCC (We can adjust these later)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	
	// Initialize LEDs for debugging I2C
	initLEDs();
	
	// Initialize timers for the intended 42 kHz target.
	initTIM();
	
	

  
  while (1)
  {
    
  }
}	// END main



/* Initialize Timer to the Nyquist sampling rate */
void initTIM(void) {
	// Configure the timer (TIM2) to trigger an update event (UEV) at 42 kHz
	TIM2->PSC = 0x1;  // 0.9524 ~1
	TIM2->ARR = 0xBD; // 189.48 ~189
	
	// Configure the timer to generate an interrupt on the UEV event
	TIM2->DIER |= TIM_DIER_UIE;
	
	// Configure an enable/start the timer 2
	TIM2->CR1 |= TIM_CR1_CEN;
	
	// Eventually use TIM3 for NVIC if interrupts need to be used

}

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

  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
  {
    ADC1->CR |= ADC_CR_ADDIS; /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0)
  {
    /* For robust implementation, add here time-out management */
  }
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;       /* (3) */
  ADC1->CR |= ADC_CR_ADCAL;              /* (4) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
  {
    /* For robust implementation, add here time-out management */
  }
	

  /* (1) Ensure that ADRDY = 0 */
  /* (2) Clear ADRDY */
  /* (3) Enable the ADC */
  /* (4) Wait until ADC ready */
  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
  {
    ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
  }
  ADC1->CR |= ADC_CR_ADEN;                 /* (3) */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
  {
    /* For robust implementation, add here time-out management */
  }
	
	//start
	ADC1->CR |= ADC_CR_ADSTART;
}



/* Initialize DAC */
void initDAC(void) {


}



/* Initialize DMA */
void initDMA(void) {


}



/* Initialize I2C */
void initI2C(void) {


}

/* Set up an I2C transaction. Can configure address, whether to read or write ('r' or 'w') and the number of bytes to send/receive.
    this is the most direct method of controlling the I2C bus short of directly manipulating registers. */
void PrepareI2C2Transaction(uint32_t address, char RD_WRN, int numbytes) {
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));	//clear NBYTES and ADD bitfields
	
	//set # of bytes to transmit=numbytes, device address 0x69, RD_WRN to write, and start
	I2C2->CR2 |= ((numbytes << 16) | (address << 1));
	
	if(RD_WRN == 'w')						//request a write
		I2C2->CR2 &= ~(1 << 10);
	else if(RD_WRN == 'r')			//request a read
		I2C2->CR2 |= (1 << 10);
	else												//assume a write by default
		I2C2->CR2 &= ~(1 << 10);
	
	//start
	I2C2->CR2 |= (1 << 13);
	
	return;
}

/* Prepare an I2C write to the specified address using a set number of bytes and a 32-bit block of data. */
void TransmissionWriteHelper(uint32_t address, int numbytes, uint32_t data) {
	
		PrepareI2C2Transaction(address, 'w', numbytes);
	
		//transmission block
		while(!(I2C2->ISR & I2C_ISR_TXIS) & !(I2C2->ISR & I2C_ISR_NACKF));
		if(I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = data;
		}
		else if(I2C2->ISR & I2C_ISR_NACKF) {
			GPIOC->ODR |= ((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));			//turn on all LEDs for fail
		}
}

/* Receive data over I2C from the specified address. */
char TransmissionReadHelper(uint32_t address, int numbytes) {
	PrepareI2C2Transaction(address, 'r', numbytes);
	while(!(I2C2->ISR & I2C_ISR_RXNE) & !(I2C2->ISR & I2C_ISR_NACKF));
	if(I2C2->ISR & I2C_ISR_RXNE) {
		return I2C2->RXDR;
	}
	else if(I2C2->ISR & I2C_ISR_NACKF) {
		GPIOC->ODR |= ((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));			//turn on all LEDs for fail
	}
	return -1;	//return an error
}

/* Initialize the LEDs for debugging purposes*/
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

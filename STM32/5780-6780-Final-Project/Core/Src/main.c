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

// Timer functions
void initTIM(void);

// DAC functions
void initDAC(void);
void waveDAC(void);

// ADC functions
void initADC(void);

// DMA functions
void initDMA(void);

// Other functions
void initLEDs(void);
void SystemClock_Config(void);
void TransmitUSARTToI2C(uint32_t address);

// UART receive stuff
volatile char receivedData;
volatile char newDataAvailable;



/* Generated Waves with 12-bit resolution and 128 samples/cycle */ // /* (Get rid of the "//" on this line to comment out the wavetables with 128 samples/cycle, return them to uncomment)
// Interpolated Sine Wave: 12-bit, 128 samples/cycle
const uint16_t sine_table[128] = {
	2048,2148,2248,2348,2447,2545,2642,2737,2831,2923,3013,3100,
	3185,3267,3346,3423,3495,3565,3630,3692,3750,3804,3853,3898,
	3939,3975,4007,4034,4056,4073,4085,4093,4095,4093,4085,4073,
	4056,4034,4007,3975,3939,3898,3853,3804,3750,3692,3630,3565,
	3495,3423,3346,3267,3185,3100,3013,2923,2831,2737,2642,2545,
	2447,2348,2248,2148,2048,1947,1847,1747,1648,1550,1453,1358,
	1264,1172,1082,995,910,828,749,672,600,530,465,403,
	345,291,242,197,156,120,88,61,39,22,10,2,
	0,2,10,22,39,61,88,120,156,197,242,291,
	345,403,465,530,600,672,749,828,910,995,1082,1172,
	1264,1358,1453,1550,1648,1747,1847,1947};

// Interpolated Triangle Wave: 12-bit, 128 samples/cycle
const uint16_t triangle_table[128] = {
	0,63,126,189,252,315,378,441,504,567,630,693,
	756,819,882,945,1008,1071,1134,1197,1260,1323,1386,1449,
	1512,1575,1638,1701,1764,1827,1890,1953,2016,2079,2142,2205,
	2268,2331,2394,2457,2520,2583,2646,2709,2772,2835,2898,2961,
	3024,3087,3150,3213,3276,3339,3402,3465,3528,3591,3654,3717,
	3780,3843,3906,3969,4032,3969,3906,3843,3780,3717,3654,3591,
	3528,3465,3402,3339,3276,3213,3150,3087,3024,2961,2898,2835,
	2772,2709,2646,2583,2520,2457,2394,2331,2268,2205,2142,2079,
	2016,1953,1890,1827,1764,1701,1638,1575,1512,1449,1386,1323,
	1260,1197,1134,1071,1008,945,882,819,756,693,630,567,
	504,441,378,315,252,189,126,63};

// Interpolated Sawtooth Wave: 12-bit, 128 samples/cycle
const uint16_t sawtooth_table[128] = {
	0,32,64,96,128,160,192,224,256,288,320,352,
	384,416,448,480,512,544,576,608,640,672,704,736,
	768,800,832,864,896,928,960,992,1024,1056,1088,1120,
	1152,1184,1216,1248,1280,1312,1344,1376,1408,1440,1472,1504,
	1536,1568,1600,1632,1664,1696,1728,1760,1792,1824,1856,1888,
	1920,1952,1984,2016,2048,2080,2112,2144,2176,2208,2240,2272,
	2304,2336,2368,2400,2432,2464,2496,2528,2560,2592,2624,2656,
	2688,2720,2752,2784,2816,2848,2880,2912,2944,2976,3008,3040,
	3072,3104,3136,3168,3200,3232,3264,3296,3328,3360,3392,3424,
	3456,3488,3520,3552,3584,3616,3648,3680,3712,3744,3776,3808,
	3840,3872,3904,3936,3968,4000,4032,4064}; // */


/* Generated Waves with 12-bit resolution and 64 samples/cycle */  /* (Get rid of the "//" on this line to comment out the wavetables with 64 samples/cycle, return them to uncomment)
// Interpolated Sine Wave: 12-bit, 64 samples/cycle
const uint16_t sine_table[64] = {
	2048,2248,2447,2642,2831,3013,3185,3346,3495,3630,3750,3853,
	3939,4007,4056,4085,4095,4085,4056,4007,3939,3853,3750,3630,
	3495,3346,3185,3013,2831,2642,2447,2248,2048,1847,1648,1453,
	1264,1082,910,749,600,465,345,242,156,88,39,10,
	0,10,39,88,156,242,345,465,600,749,910,1082,
	1264,1453,1648,1847};

// Interpolated Triangle Wave: 12-bit, 64 samples/cycle
const uint16_t triangle_table[64] = {
	0,128,256,384,512,640,768,896,1024,1152,1280,1408,
	1536,1664,1792,1920,2048,2176,2304,2432,2560,2688,2816,2944,
	3072,3200,3328,3456,3584,3712,3840,3968,4095,3968,3840,3712,
	3584,3456,3328,3200,3072,2944,2816,2688,2560,2432,2304,2176,
	2048,1920,1792,1664,1536,1408,1280,1152,1024,896,768,640,
	512,384,256,128};

// Interpolated Sawtooth Wave: 12-bit, 128 samples/cycle
const uint16_t sawtooth_table[64] = {
	0,64,128,192,256,320,384,448,512,576,640,704,
	768,832,896,960,1024,1088,1152,1216,1280,1344,1408,1472,
	1536,1600,1664,1728,1792,1856,1920,1984,2048,2112,2176,2240,
	2304,2368,2432,2496,2560,2624,2688,2752,2816,2880,2944,3008,
	3072,3136,3200,3264,3328,3392,3456,3520,3584,3648,3712,3776,
	3840,3904,3968,4032}; // */


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
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_DAC1_CLK_ENABLE();
	
	// Initialize LEDs for debugging I2C
	initLEDs();
	
	// Initialize timers for the intended 42 kHz target.
	initTIM();
	
	// Initialize DAC peripherals
	initDAC();
	
	//initialize I2C interface
	initI2C();
	
	//initialize USART
	USARTSetup();

	// Test DAC waves with Discovery Analog 2
	waveDAC();
	
	
	while (1)
  {
		transmit_string("Waiting for USART input.\r\n");
		
		while((USART3->ISR & USART_ISR_RXNE) != USART_ISR_RXNE) { }		//prevent the text from being sent like crazy
		if(newDataAvailable) {
			//TransmissionWriteHelper(0, sizeof(receivedData), receivedData);
			transmit_string("Sending");
			TransmitUSARTToI2C(0);
			
		}
		
		receivedData = 0;
		
		
		//TransmissionWriteHelper(0x70, 1, 0xf);				//test write
		
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



/* Initialize DAC (Using PA4) */
void initDAC(void) {
	/* Initialize PA4 to read DAC */
	// (Reset state: 00)
	GPIOA->MODER &= ~(GPIO_MODER_MODER4_Msk);

	// (Analog function: 11)
	GPIOA->MODER |= (GPIO_MODER_MODER4_Msk);
	
	// Configure Push/Pull Output type for PA4	(00)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
	
	// Configure low speed for PA4	(00)
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR4_Msk);
	
	// Configure no pull-up/down resistors for PA4	(00)
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4_Msk);
	
	
	/* Configure DAC */
	// Set software trigger: (111)
	DAC1->CR |= (DAC_CR_TSEL1_Msk);
	
	// DAC enable
	DAC1->CR |= (DAC_CR_EN1_Msk);
}


// global variable index
uint8_t ind = 0;

/* DAC wave table */
void waveDAC(void) {
	// Send wave to data register DHR12R1
	DAC1->DHR12R1 = sine_table[ind]; // Sine Wave
	//DAC1->DHR12R1 = triangle_table[ind]; // Triangle Wave
	//DAC1->DHR12R1 = sawtooth_table[ind]; // Sawtooth Wave
	
	// Track the index (ind) count
	ind++;
	
	/* FOR 128 samples/cycle */ // /*
	if (ind == 127) {
		ind = 0;
	} // */
	
	/* FOR 64 samples/cycle  */  /*
	if (ind == 63) {
		ind = 0;
	} // */
	
	// Insert delay 1ms
	HAL_Delay(1);
}



/* Initialize DMA */
void initDMA(void) {


}

//the interrupt handler for USART
void USART3_4_IRQHandler(void) {
	receivedData = USART3->RDR;
	transmit_char(receivedData);
	newDataAvailable = 1;
}

// Transmits anything received over USART to an I2C device of the specified address.
void TransmitUSARTToI2C(uint32_t address) {
	transmit_char('#');
	while((USART3->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	transmit_char('@');
	if(newDataAvailable) {
		transmit_string("Now sending data to I2C.\r\n");
		TransmissionWriteHelper(address, sizeof(receivedData), receivedData);
	}
	
	transmit_string("Data sent.\r\n");
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

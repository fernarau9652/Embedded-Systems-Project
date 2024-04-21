/* USER CODE BEGIN Header */
/** CHASE GRISWOLD
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


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN */


//void init_digiPot(void);
void init_digiPot(int c);
uint8_t resist_val_data;
volatile uint8_t data;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//Enable GPIOB and GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //Needed for I2C stuff
	
	//Initialize (Clear) the GPIOB/C registers
	GPIOB->OTYPER = 0;
	GPIOB->MODER = 0;
	
	//SDA Setup --------
	//Initialize PB11
	GPIOB->MODER |= (1 << 23); //Enable AF Mode
	GPIOB->OTYPER |= (1 << 11); //Open-drain
	
	//PB11 Alternate Function AF1 select: I2C2_SDA -> AF1 of PB11
	//Bits [15:12] correspond to PB11 in the register -> AF1 is 0001.
	GPIOB->AFR[1] |= (1 << 12); 
	
	//SCL Setup --------
	// Initialize PB13
	GPIOB->MODER |= (1 << 27); //Enable AF Mode
	GPIOB->OTYPER |= (1 << 13); //Open-drain
	
	//PB13 Alternate Function AF5 select: I2C2_SCL -> AF5 of PB13
	//Bits [23:20] correspond to PB13 in the register, -> AF5 is 0101.
	GPIOB->AFR[1] |= (5 << 20);
	
	//Initialize PC0 and PB14
	GPIOB->MODER |= (1 << 28); //General-Purpose Output
	GPIOC->MODER |= (1 << 0);
	GPIOC->OTYPER = 0; //Push-pull mode.
	GPIOB->ODR |= (1 << 14); //PB14 set High
	GPIOC->ODR |= (1 << 0);  //PC0 set High
	
	//I2C2 Peripheral RCC Enable (I2C)
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //Clock enable
	
	//Setting up the I2C2 Peripheral timing for SDA and SCL
	//Set to 100kHz operation
	I2C2->TIMINGR |= (1 << 28); //PSC = 1
	I2C2->TIMINGR |= (0x4 << 20); //[23:20] SCLDEL (Data Setup Time)
	I2C2->TIMINGR |=  (0x2 << 16); //[19:16] SDADEL (Data Hold Time)
	I2C2->TIMINGR |= (0xF << 8); //[15:8] SCLH (SCL High Period)
	I2C2->TIMINGR |= (0x13 << 0); //[7:0] SCLL (SCL Low Period)
	
	//Enable the I2C2 Peripheral
	I2C2->CR1 |= (1 << 0); //Enable (PE Bit in CR1 Register)
	
	//My Key: 
	//--------//Positive Y-Axis = RED LED
	//--------//Negative Y-Axis = BLUE LED
	//--------//Positive X-Axis = GREEN LED
	//--------//Negative X-Axis = ORANGE LED
	
	//LED Setup
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER |= (1 << 14); 
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER |= (1 << 18);
	
	
	// Hardcodede version 
	//resist_val_data = 0b00010000; //Data Byte (256-bit resolution number, ~396 ohms per bit).
	
		data = 0;

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
		while (1)
			{
				
					init_digiPot(data);
					if(data<255) {
						data++;
					}
					else {
						data = 0;
					}
					HAL_Delay(25); // Delay 400ms
					GPIOC->ODR ^= (1 << 6);
				
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

void init_digiPot(int resist_val_data){
		I2C2->CR2 = 0; //Clear to start.
		
		//Setup: Digi-pot Address 0x28
		I2C2->CR2 |= (0x28 << 1); //Digi-pot address is [7:1]
		
		//Set # of bytes = 2 (Address + Register value).
		I2C2->CR2 |= (2 << 16);
		
		//Set the RD_WRN bit to 'WRITE' operation.
		//Set the START bit
		I2C2->CR2 &= ~(1 << 10); //Ensure 0 for write
		I2C2->CR2 |= (1 << 13);

		//Wait until either of the TXIS (Transmit Register Empty/Ready) 
		//or NACKF (Slave NotAcknowledge) flags are set.
		//----//If the NACKF flag is set, the slave did not respond to the address frame.
		//----//Continue if the TXIS flag is set
		while(((I2C2->ISR & 0x2) >> 1) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //NACFK flag check
				//Turn on LED to indicate config error
				GPIOC->ODR |= (1 << 6);
			}
		}
		
		//Set command_byte for digi-pot TX
		I2C2->TXDR = 0b10101010; // Command Byte to write to potentiometer 1.
		
		//Wait for TXIS flag
		while(((I2C2->ISR & 0x2) >> 1) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //NACFK flag check
				//Turn on LED to indicate config error
				GPIOC->ODR |= (1 << 6);
			}
		}
		
		//Set data_byte for digi-pot TX
		I2C2->TXDR = resist_val_data;
		
		//Wait for TC flag
		while(((I2C2->ISR & 0x40) >> 6) != 1){
			
		}
		
		//STOP bit: Release the I2C bus
		I2C2->CR2 |= (1 << 14); 
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

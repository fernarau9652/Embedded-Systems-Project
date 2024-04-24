#include "main.h"
#include "usart.h"

/* Initialize I2C */
void initI2C(void) {
	//Enable GPIOB and GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //Needed for I2C stuff
	
	//Initialize (Clear) the GPIOB/C registers
	GPIOB->OTYPER = 0;
	GPIOB->MODER = 0;
	
	//SDA SETUP-------------
	//set PB14 to alternate function mode, open-drain output type, and I2C2_SDA as alt funct
	GPIOB->MODER = (GPIOB->MODER & (~(GPIO_MODER_MODER14)) | GPIO_MODER_MODER14_1);
	GPIOB->OTYPER |= (1 << 14);	//set bit 1 in OTYPER to open-drain
	GPIOB->AFR[1] |= (0x5 << 24);	//we want alternate function mode AF5 for AFSEL14 (pin 14 on port b)
	
	//SCL SETUP-------------
	//set PB13 to alternate function mode, open-drain output type, and I2C2_SCL as alt function
	GPIOB->MODER = (GPIOB->MODER & (~(GPIO_MODER_MODER13)) | GPIO_MODER_MODER13_1);
	GPIOB->OTYPER |= (1 << 13);	
	GPIOB->AFR[1] |= (0x5 << 20);	//AF5 on AFSEL13 selected
		
	//Initialize PC0 and PB15
	GPIOB->MODER |= (1 << 30); //General-Purpose Output
	GPIOC->MODER |= (1 << 0);
	GPIOC->OTYPER = 0; //Push-pull mode.
	GPIOB->ODR |= (1 << 15); //PB15 set High
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
}

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
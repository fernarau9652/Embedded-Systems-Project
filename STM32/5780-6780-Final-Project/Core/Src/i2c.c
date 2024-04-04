#include "main.h"
#include "usart.h"

/* Initialize I2C */
void initI2C(void) {
	//enable the I2C2 peripheral's system clock in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	//set PB14 to alternate function mode, open-drain output type, and I2C2_SDA as alt funct
	GPIOB->MODER = (GPIOB->MODER & (~(GPIO_MODER_MODER14)) | GPIO_MODER_MODER14_1);
	GPIOB->OTYPER |= (1 << 14);	//set bit 1 in OTYPER to open-drain
	GPIOB->AFR[1] |= (0x5 << 24);	//we want alternate function mode AF5 for AFSEL14 (pin 14 on port b)
	
	//set PB13 to alternate function mode, open-drain output type, and I2C2_SCL as alt function
	GPIOB->MODER = (GPIOB->MODER & (~(GPIO_MODER_MODER13)) | GPIO_MODER_MODER13_1);
	GPIOB->OTYPER |= (1 << 13);	
	GPIOB->AFR[1] |= (0x5 << 20);	//AF5 on AFSEL13 selected
	
	//now set the parameters in the TIMINGR register to use 100kHz standard mode I2C
	//PRESC=1, SCLDEL=0x4, SDADEL=0x2, SCLH=0xF, SCLL=0x13
	I2C2->TIMINGR |= ((1 << 28) | (0x4 << 20) | (0x2 << 16) | (0xf << 8) | (0x13));
	
	//enable I2C2 using PE bit in CR1 register
	I2C2->CR1 |= I2C_CR1_PE;
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
		
		//transmission block (PUT THIS BACK LATER)*************************************************************************
		//while(!(I2C2->ISR & I2C_ISR_TXIS) & !(I2C2->ISR & I2C_ISR_NACKF));			//block until there is a response 
	
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
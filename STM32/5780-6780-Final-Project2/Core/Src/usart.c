#include "main.h"
#include "stdlib.h"

//Set up the USART peripheral in the microcontroller
void USARTSetup(void) {
	//configure USART
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;		//enable system clock to USART3
	
	//enable alternate function mode on GPIOB pins 10 and 11
	//We want to write "10" into MODER for pins 10 and 11
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11)) | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	
	//set PB10 as TX and PB11 as RX for USART3
	//this will use alternate function mode AF4
	GPIOB->AFR[1] |= ((1 << 10) | (1 << 14));
	
	//set baud rate to 115200 bits/second (oversampling by 16)
	//can use HAL_RCC_GetHCLKFreq() to get system clock freq
	//we can safely assume that the processor is running at 8MHz
	USART3->BRR = (HAL_RCC_GetHCLKFreq() / 115200);
	
	//enable USART3 receive regeister not empty interrupt
	USART3->CR1 |= (USART_CR1_RXNEIE);
	
	//enable and set up USART interrupt priority in NVIC
	NVIC_EnableIRQ(USART3_4_IRQn);	//IRQ 29
	NVIC_SetPriority(USART3_4_IRQn, 1);
		
	//enable USART transmit, receive, and the USART itself
	USART3->CR1 |= ((USART_CR1_TE) | (USART_CR1_RE) | (USART_CR1_UE));
}

//Transmit a singular character
void transmit_char(char characterToSend) {
	//check and wait on USART status flag indicating transmit register empty
	
	//do nothing until the TXE (transmit data bit) bit is set
	while((USART3->ISR & USART_ISR_TXE) != USART_ISR_TXE) {
		//USART3->TDR = 'L';		//essentially a debug print statement
	}
	
	//write the character into the transmit data register
	USART3->TDR = characterToSend;
}

//Transmit an integer
void transmit_int(char* intToSend) {
	//check and wait on USART status flag indicating transmit register empty
	
	int c_intToSend = atoi(intToSend);
	
	//write the character into the transmit data register
	USART3->TDR = c_intToSend;
}

//Transmit a string
void transmit_string(char* stringToSend) {
	while(*stringToSend != 0) {
		transmit_char(*stringToSend);
		stringToSend++;
	}
	return;
}
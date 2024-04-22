#include "i2c.h"
#include "usart.h"

void WritePot(__UINT32_TYPE__ address, __INT8_TYPE__ data, char potnum) {

	//Prepare the transaction with sending the control byte
	//send 0101 followed by 3-bit address and then r/w = 0

	__INT8_TYPE__ controlbyte = ((0x5 << 3) | (address) & 0x7);								//prepare a control byte to send
	__INT16_TYPE__ fulldata = data;
	
	//send the command byte without sending another start condition	
	//send 10101001 to write to pot 0
	//send 10101010 to write to pot 1
	if(potnum == 0)
		fulldata |= 0xA9;																												//append pot0's command byte to upper data
	else
		fulldata |= 0xAA;																												//otherwise, pot1's commmand byte is appended
		
	//send the full thing in one go
	//TransmissionWriteHelper(controlbyte, 3, fulldata);												//this SHOULD send.
}

//void ReadPot(
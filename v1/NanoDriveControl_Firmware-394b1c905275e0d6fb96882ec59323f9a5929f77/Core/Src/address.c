/*
 * address.c
 *
 *  Created on: Jun 8, 2023
 */

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "string.h"
#include "command.h"
#include "commandlist.h"
#include "sysinfo.h"
#include "motion.h"

extern UART_HandleTypeDef huart1;

char usart1_data[6] = {0};
uint8_t ucAxis;
static uint8_t ucGate;

uint8_t getAxis( void ) {
	return ucAxis;
}

void setAxis( uint8_t axis ) {
	ucAxis = axis;
}

uint8_t getGate( void ){
	return ucGate;
}

void setGate( uint8_t gate ) {
	ucGate = gate;
}

char WaitForTurn()
{
	uint32_t timeout = 0;

	while( READ_ADDRESS_IN )
	{
		if(timeout++ >= 1500000)
		{
			return 0;
		}
		else
		{
			__ASM volatile("nop");
		}
	}
	return 1;
}

//Addressing through RS485 at the moment
void GetAddress()
{
	uint32_t timeout = 1, data;
	uint8_t Gate = 0;
	char dataUART1;

//	RECEIVE_RS485_UART1;

	//Set LEDs to indicate addressing mode

	//Unless our ID line is active we ignore everything coming in from the addressing bus
	while( READ_ADDRESS_IN )
	{
		//Remove all UART1 traffic until the ID_IN is activated
		//*** ADD CODE *** need to coordinate ignoring incoming data while addressing
		//if (USART1->SR & USART_SR_RXNE) data = USART1->DR;
	}

	timeout = 1;

	//Wait for something to come in, or timeout
	while(!(USART1->SR & USART_SR_RXNE) && timeout)
	{
		if( timeout++ >= 1000000 )	//Can convert this to use the systick timer
		{							//for a more consistent time delay. Using
			timeout = 0;			//a constant like this causes the delay to be
		}							//dependent on the cpu frequency
	}

	timeout = 1;

	if(USART1->SR & USART_SR_RXNE)
	{
		//broke out of loop because data received on the addressing bus
		//The controller is not first in line.

		//Get RS485 address message
		dataUART1 = USART1->DR;
		if (dataUART1 == '#')
		{
			while (!(USART1->SR & USART_SR_RXNE));

			dataUART1 = USART1->DR;
			Gate = dataUART1 - '0';

			while (!(USART1->SR & USART_SR_RXNE));

			dataUART1 = USART1->DR;
			data = (char)(dataUART1 -'0') & 0xFF;

			//we have the address data from UART1. We need to clear the new line and carriage return to avoid timing issues
			while (!(USART1->SR & USART_SR_RXNE));
			dataUART1 = USART1->DR; //pull new line

			while (!(USART1->SR & USART_SR_RXNE));
			dataUART1 = USART1->DR; //pull carriage return
		}

		if( AddressingMode == 0 || AddressingMode == 0xFFFFFFFF )
		{
			//change addressing calculations
			ucAxis = (Gate * 10) + data + 1;
			ucGate = Gate;
			AddressingMode = 0;
		}
		else
		{
			ucAxis = AddressingMode;
			ucGate = Gate;
		}
	}
	else
	{
		//We have timed out. This is the first board.
		if( AddressingMode == 0xFFFFFFFF )
		{
			//Corrupted addressing mode
			ucAxis = 1;
			ucGate = Gate;
			AddressingMode = 0;
		}
		if (AddressingMode>=1 && AddressingMode<=99)
		{
			ucAxis = AddressingMode;
			ucGate = Gate;
		}
		else
		{
			ucAxis = 1;
			ucGate = Gate;
		}
	}
	RS485_TRANSMIT;
	while( timeout++ < 1000000 )
	{
		__ASM("nop");
	}

	SET_ADDRESS_OUT;

	timeout = 0;
	while(timeout++<1000){
		__ASM("nop");
	}

	//TODO --> Convert this to actual address print
	sprintf(usart1_data, "#01\n\r");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)usart1_data, 5);

	//ADD CODE --> We'll need to actually clear the LED on the actual hardware
	TURN_ON_BLUE_LED;

}


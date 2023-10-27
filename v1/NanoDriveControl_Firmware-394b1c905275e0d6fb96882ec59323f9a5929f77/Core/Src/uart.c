/*
 * uart.c
 *
 *  Created on: Jul 12, 2023
 *      Author: tjudge
 */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "util.h"
#include "error.h"
#include "address.h"
#include "motionscript.h"

char GetDestination(Que * myQue, sRouteInfo *Dest);
void FlushLine(Que *myQue, char add_err);

volatile uint16_t line_count_485 = 0;
volatile uint16_t line_count_nano = 0;

Que rxQueUSART1;
Que rxQueUSART3;

extern uint8_t USART1_rxChar[1];
extern uint8_t USART3_rxChar[1];

extern uint8_t driverReady;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t rxChar;

	if (huart->Instance == USART1)
	{
		rxChar = USART1_rxChar[0];
		USART1_rxChar[0] = 0;

		if( rxChar == '\r' || rxChar == ';')
		{
			enqueueCirq( &rxQueUSART1, rxChar );
			line_count_485++;
		}
		else if( rxChar != ' ' && rxChar != '\t' && rxChar != '\n')
		{
			enqueueCirq( &rxQueUSART1, rxChar);
		}

		HAL_UART_Receive_IT(huart, USART1_rxChar, 1);
	}
	else if (huart->Instance == USART3)
	{
		rxChar = USART3_rxChar[0];
		USART3_rxChar[0] = 0;

		if( rxChar == '\r')
		{
			enqueueCirq( &rxQueUSART3, rxChar);
			line_count_nano++;
		}
		else if( rxChar != ' ' && rxChar != '\t' && rxChar != '\n')
		{
			enqueueCirq( &rxQueUSART3, rxChar );
		}
		HAL_UART_Receive_IT(huart, USART3_rxChar, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	while ((USART1->SR&0x40) == 0);
	huart->TxInProgress = 0;
	RS485_RECEIVE;
}

void procQueUSART1 (sCmdQue *cmdQ){

	uint8_t i=0;
//	unsigned char length;
	uint8_t cmdLength = 0;

	// DETERMINE IF WE HAVE A COMPLETE CMD G:AACMD
	// GET FIRST DIGIT ( GATE OR AXIS )

	i = (rxQueUSART1.Data[ rxQueUSART1.Tail ]);

	if(i == '#')
	{
		//Do not flush response from another axis. if it is a reply it may need to be routed to RS232/USB
		cmdQ->Msg[ cmdQ->Head ].CommandType = STANDARD_COMMAND;
		//This is a response from another axis. Feel free to ignore it.
		//FlushLine(&rxQueUSART1,0);
		//return;
	}

	//if(i is not a digit) line is garbage. Flush to end of line '\r'
	else if(i >= '0' && i <= '9')
	{
		cmdQ->Msg[ cmdQ->Head ].CommandType = STANDARD_COMMAND;
		if(!GetDestination(&rxQueUSART1,&cmdQ->Msg[cmdQ->Head]))
		{
			//Success, now copy the data.
			FlushLine(&rxQueUSART1,1);
			return;
		}
	}
	else if(i == '&')		//BINARY DATA
	{	//Binary data. Just copy it over.
		cmdQ->Msg[cmdQ->Head].CommandType = BINARY_DATA;

		//Here, we do a fixed length copy.  '\r' can and does appear in the body of the data, so we cannot use it to test for completion.
		cmdLength = 0;
		while(cmdLength<BINARY_LINE_LENGTH)
		{
			cmdQ->Msg[cmdQ->Head].Data[cmdLength++] = i;
			rxQueUSART1.Tail = (rxQueUSART1.Tail + 0x01) & SERIAL_BUFFER_MASK;
			i = (rxQueUSART1.Data[ rxQueUSART1.Tail ]);
		}

		cmdQ->Msg[cmdQ->Head].Length = cmdLength;
		cmdQ->Msg[cmdQ->Head].SysOrigin = CMD_ORIGIN_USART1;
		cmdQ->Msg[cmdQ->Head].AxisOrigin = CMD_ORIGIN_USART1;
		cmdQ->Head = (cmdQ->Head + 0x01) & CMD_BUFFER_MASK;
		return;
	}
	else if(i == '!')
	{
		cmdQ->Msg[cmdQ->Head].CommandType = BINARY_DATA;

		cmdLength = 0;
		while(cmdLength < BINARY_COMMAND_LENGTH)
		{
			cmdQ->Msg[cmdQ->Head].Data[cmdLength++] = rxQueUSART1.Data[rxQueUSART1.Tail];
			rxQueUSART1.Tail = (rxQueUSART1.Tail + 0x01) & SERIAL_BUFFER_MASK;
		}

		cmdQ->Msg[cmdQ->Head].Length = cmdLength;
		cmdQ->Msg[cmdQ->Head].SysOrigin = CMD_ORIGIN_USART1;
		cmdQ->Msg[cmdQ->Head].AxisOrigin = CMD_ORIGIN_USART1;
		cmdQ->Head = (cmdQ->Head + 0x01) & CMD_BUFFER_MASK;
		return;
	}
	else if(i == '%')		//MOTION_PROGRAM
	{
		cmdQ->Msg[cmdQ->Head].CommandType = MOTION_PROGRAM;
		rxQueUSART1.Tail = (rxQueUSART1.Tail + 0x01) & SERIAL_BUFFER_MASK;

		if(!GetDestination(&rxQueUSART1,&cmdQ->Msg[cmdQ->Head]))
		{
			FlushLine(&rxQueUSART1,1);
			return;
		}
	}
	else if( i == '\r' )
	{
		//blank line
		rxQueUSART1.Tail = ( rxQueUSART1.Tail + 0x01 ) & SERIAL_BUFFER_MASK;
		return;
	}
	else
	{	//This is an invalid line. Flush it out.
		FlushLine(&rxQueUSART1,1);
		return;
	}

	cmdLength = 0;

//Standard
	while(i != '\r' && i != ';')
	{
		if( cmdLength < CMD_MAX_LENGTH )
		{
			i = rxQueUSART1.Data[ rxQueUSART1.Tail ];
			cmdQ->Msg[ cmdQ->Head ].Data[ cmdLength++ ] = rxQueUSART1.Data[ rxQueUSART1.Tail ];
			// ADVANCE QUEUE
			rxQueUSART1.Tail = ( rxQueUSART1.Tail + 0x01 ) & SERIAL_BUFFER_MASK;
		}
		else
		{
			//Too long.
			//Flush until end of line.
			//AddError?
			while(rxQueUSART1.Tail != '\r' && rxQueUSART1.Tail != ';')	//We should really flush to \r character here, but we may not have it yet.
			{
				rxQueUSART1.Tail = ( rxQueUSART1.Tail + 0x01 ) & SERIAL_BUFFER_MASK;
			}
			return;
		}
	}

	cmdQ->Msg[ cmdQ->Head ].SysOrigin = CMD_ORIGIN_USART1;
	cmdQ->Msg[ cmdQ->Head ].AxisOrigin = CMD_ORIGIN_USART1;
	cmdQ->Msg[ cmdQ->Head ].Length = cmdLength;
	// ADVANCE THE COMMAND QUEUE
	cmdQ->Head = ( cmdQ->Head + 0x01 ) & CMD_BUFFER_MASK;

}

void procQueUSART3 ()
{
	//Simplified version of the command processing
	//We are only receiving replies from the driver ARM on USART3
	uint8_t i=0;

	i = (rxQueUSART3.Data[ rxQueUSART3.Tail ]);
	rxQueUSART3.Tail = (rxQueUSART3.Tail + 0x01) & SERIAL_BUFFER_MASK;
	if(i == '#')
	{
		//Reply has been received
		//only have STA implemented right now
		i = (rxQueUSART3.Data[ rxQueUSART3.Tail ]);
		rxQueUSART3.Tail = (rxQueUSART3.Tail + 0x01) & SERIAL_BUFFER_MASK;
		if (i=='0') driverReady = 0;
		else if (i=='1') driverReady = 1;

		FlushLine(&rxQueUSART3,0);
	}
	else
	{
		//incomplete or incorrect data received. Flush it
		FlushLine(&rxQueUSART3,0);
	}
}

char GetDestination(Que * myQue, sRouteInfo *Dest)
{
	uint8_t Char;

	Char = myQue->Data[myQue->Tail];
	if(Char >= '0' && Char <= '9')
	{
		//We have a character. Is it a gate or is it an axis number. We don't know yet.
		myQue->Tail = (myQue->Tail + 0x01) & SERIAL_BUFFER_MASK;
		Char = Char - '0';
	}
	else if(Char == '*' && Dest->CommandType == MOTION_PROGRAM)
	{
		Char = PROGRAM_SELF_ADDR;
		//Don't know if this is a gate or an axis #.
		myQue->Tail = (myQue->Tail + 0x01) & SERIAL_BUFFER_MASK;
	}
	else
	{
		return 0;
	}

	if(myQue->Data[myQue->Tail] == ':')
	{	//What we previously read was a Gate identifier.
		Dest->tGate = Char;
		myQue->Tail = (myQue->Tail + 0x01) & SERIAL_BUFFER_MASK;
		if(myQue->Data[myQue->Tail] >= '0' && myQue->Data[myQue->Tail] <= '9')
		{
			Dest->tAxis = myQue->Data[myQue->Tail] - '0';		//If the axis# is 2 digits, this will be the tens place. We will wrap that up shortly.
			myQue->Tail = (myQue->Tail +0x1) & SERIAL_BUFFER_MASK;
		}
		else if(myQue->Data[myQue->Tail] == '*' && Dest->CommandType == MOTION_PROGRAM)
		{
			Dest->tAxis = PROGRAM_SELF_ADDR;
			myQue->Tail = (myQue->Tail + 0x1) & SERIAL_BUFFER_MASK;
			return 1;
		}
		else
		{
			//Gate but no Axis #. Error.
			//Previously this flagged no error.
			return 0;
		}
	}
	else
	{	//There is no gate present. Use local gate.
		Dest->tGate = getGate();
		Dest->tAxis = Char;
		if(Char == PROGRAM_SELF_ADDR) return 1;
	}

	if(myQue->Data[myQue->Tail] >= '0' && myQue->Data[myQue->Tail] <= '9')
	{
		Dest->tAxis = Dest->tAxis * 10 + (myQue->Data[myQue->Tail] - '0');
		myQue->Tail = (myQue->Tail + 0x1) & SERIAL_BUFFER_MASK;
	}

	return 1;
}

void FlushLine(Que *myQue, char add_err)
{
	unsigned char pp = 0;
	char instr_buf[3] = {'.','.','.'};

	instr_buf[0] = myQue->Data[myQue->Tail];
	myQue->Tail = ( myQue->Tail + 0x01 ) & SERIAL_BUFFER_MASK;

	while( myQue->Tail != myQue->Head )
	{
		//Flush
		if( myQue->Data[ myQue->Tail ] == '\r' )
		{
			myQue->Tail = ( myQue->Tail + 0x01 ) & SERIAL_BUFFER_MASK;
			break;
		}
		if( pp < 3 )
		{
			instr_buf[pp++] = myQue->Data[ myQue->Tail ];
		}
		myQue->Tail = ( myQue->Tail + 0x01 ) & SERIAL_BUFFER_MASK;
	}

	if(add_err)
	{
		AddError(MISSING_AXIS_NUMBER,instr_buf);
	}
	return;
}


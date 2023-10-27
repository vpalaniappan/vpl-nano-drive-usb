/*
 * utils.c
 *
 *  Created on: Jun 8, 2023
 */

#include "stdint.h"
#include "stdio.h"
#include "sysinfo.h"
#include "command.h"

void iniCmdQue ( sCmdQue *q ) {
  q->Head = 0;
  q->Tail = 0;
}

void enqueueCmd( sCmdQue *q ) {
	q->Head = ( q->Head + 0x01 ) & CMD_BUFFER_MASK;	//  Advance the Head Pointer
}

void dequeueCmd( sCmdQue *q ) {
	q->Tail = ( q->Tail + 0x01 ) & CMD_BUFFER_MASK;		//  Advance the Tail Pointer
}

void flushCmd( sCmdQue *q ) {
	q->Tail = 0;
	q->Head = 0;
}

void iniCircQue ( Que *q ) {
	q->Head = 0;
	q->Tail = 0;
}

void enqueueCirq( Que *q, unsigned char data ) {
  q->Data[ q->Head ] = data;
  q->Head = ( q->Head + 0x01 ) & SERIAL_BUFFER_MASK;
  q->Data[ q->Head ] = '\0';	//what's the purpose of this?
}

void dequeueCirq( Que *q ) {
	unsigned char x;
  	x = ( q->Tail + 0x01 ) & SERIAL_BUFFER_MASK;
  	q->Tail = x;
}


uint8_t convert_2(uint16_t a, uint16_t b, char buf[], uint8_t opt)
{
	uint8_t length = 0;
	if(opt==1)
	{
		if(b>99)
		{
			length = sprintf(buf,"%d.%d\n\r",a,b);
		}
		else if(b>9)
		{
			length = sprintf(buf,"%d.0%d\n\r",a,b);
		}
		else length = sprintf(buf,"%d.00%d\n\r",a,b);
	}
	else if(opt == 0)
	{
		if(b>99)
		{
			length = sprintf(buf,"#%d.%d\n\r",a,b);
		}
		else if(b>9)
		{
			length = sprintf(buf,"#%d.0%d\n\r",a,b);
		}
		else length = sprintf(buf,"#%d.00%d\n\r",a,b);
	}
	else if(opt == 2)
	{
		if(b>99)
		{
			length = sprintf(buf,"#%d.%d,,\n\r",a,b);
		}
		else if(b>9)
		{
			length = sprintf(buf,"#%d.0%d,,\n\r",a,b);
		}
		else length = sprintf(buf,"#%d.00%d,,\n\r",a,b);
	}
	else if(opt == 3)
	{
		if(b>99)
		{
			length = sprintf(buf,"%d.%d",a,b);
		}
		else if(b>9)
		{
			length = sprintf(buf,"%d.0%d",a,b);
		}
		else length = sprintf(buf,"%d.00%d",a,b);
	}
	return length;
}

//!Convert to floating point
//! This function assembles the separate fixed point values
//!into a string representation of a floating point #.
void convert_3(int16_t a,int16_t b, int16_t c, char buf[])
{
	if(b>99)
	{
		if( c > 99 )
		{
			sprintf(buf,"%d.%d%d",a,b,c);
		}
		else if( c > 9 )
		{
			sprintf(buf,"%d.%d0%d",a,b,c);
		}
		else sprintf(buf,"%d.%d00%d",a,b,c);
	}
	else if(b > 9)
	{
		if( c > 99)
		{
			sprintf(buf,"%d.0%d%d",a,b,c);
		}
		else if( c > 9 )
		{
			sprintf(buf,"%d.0%d0%d",a,b,c);
		}
		else sprintf(buf,"%d.0%d00%d",a,b,c);
	}
	else
	{
		if( c > 99 )
		{
			sprintf(buf,"%d.00%d%d",a,b,c);
		}
		else if(c > 9)
		{
			sprintf(buf,"%d.00%d0%d",a,b,c);
		}
		else sprintf(buf,"%d.00%d00%d",a,b,c);
	}
}

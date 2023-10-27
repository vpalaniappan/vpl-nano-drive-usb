/*
 * util.h
 *
 *  Created on: Jun 8, 2023
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "sysinfo.h"
#include "command.h"

uint8_t convert_2(uint16_t a, uint16_t b, char buf[], uint8_t opt);
void convert_3(int16_t a,int16_t b, int16_t c, char buf[]);

void iniCmdQue ( sCmdQue *q );
void enqueueCmd( sCmdQue *q );
void dequeueCmd( sCmdQue *q );
void iniCircQue ( Que *q );
void dequeueCirq( Que *q );
void enqueueCirq( Que *q, uint8_t data );

#endif /* INC_UTIL_H_ */

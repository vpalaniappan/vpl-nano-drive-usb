/*
 * uart.h
 *
 *  Created on: Jul 12, 2023
 *      Author: tjudge
 */
#include "command.h"
#include "sysinfo.h"

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void procQueUSART1 (sCmdQue *cmdQ);
void procQueUSART3 (void);
char GetDestination(Que * myQue, sRouteInfo *Dest);

extern volatile uint16_t line_count_485;

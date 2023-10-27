/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "init.h"
#include "util.h"
#include "uart.h"
#include "flash.h"
#include "address.h"
#include "command.h"
#include "motion.h"
#include "motionspi.h"
#include "motionscript.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

sCmdQue cmdOS;
eCmd cmdData;

uint8_t USART1_rxChar[1];
uint8_t USART1_txChar[10] = {'O', 'K', '\r'};

uint8_t USART3_rxChar[1];
uint8_t USART3_txChar[10] = {'S', 'T', 'A', '?', '\r'};

uint8_t SPI2_txData[7] = {0};
uint8_t SPI2_rxData[4] = {0};

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern volatile uint16_t line_count_nano;

/*--------------------USB-------------------------------------*/
uint8_t command[10] = "V1.1.1\r\n\r\n";
uint8_t company[20] = "MICRONIX USA\r\n\r\n";
uint8_t company2[20] = "	 MICRONIX USA\r\n";
uint8_t program[11] = "USB VCP\r\n\r\n";
uint8_t error[20] = "INVALID COMMAND.\r\n\r\n";
uint8_t rcv[20] = ">ENTER COMMAND > ";
uint8_t equal[34] = "================================\r\n";
uint8_t enter[2]= "\r\n";
/*--------------------USB-------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void ProcessCmdQueue();

int ExecuteCommand(eCmd cmd);	//exunit.c
void InitializeDriver();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  MX_USB_DEVICE_Init();
  PeripheralInit();

  ReadParameters(BASE_PARAM_ADDRESS);
  //TODO --> confirm address handshaking with multiple devices.
  GetAddress();

  //Set up USART ready to receive from driver STM32
  HAL_UART_Receive_IT(&huart3, USART3_rxChar, 1);

  InitializeDriver();
  AxisSetup();

  //Parameter and address handling is done. Start initializing various peripherals
  HAL_UART_Receive_IT(&huart1, USART1_rxChar, 1);		//RS485 UART
  HAL_TIM_Base_Start_IT(&htim2);						//Servo Timer
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);	//Encoder Timer

  TURN_OFF_BLUE_LED;
  TURN_ON_GREEN_LED;

  uint8_t buf[1000];
  uint8_t received[6];
  int str_len=0;
  vcp_init();
  HAL_Delay(100);
  vcp_send(enter,2);
  vcp_send(equal,34);
  vcp_send(company2,20);
  vcp_send(equal,34);
  vcp_send(rcv,20);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Check service flags
	  CheckFlags();
	  ProcessCmdQueue();
	  int len = vcp_recv (buf, 1);  // Read up to 1000 bytes
	  	      if (len > 0)
	  	      {    // If some data was read, send it back :
	  	    	//  len = vcp_send (buf, len);
	  		      if(str_len <5 && buf[0]!='\n')
	  		      {
	  		    	  received[str_len]=buf[0];
	  		    	  str_len++;
	  		      }
	  		      if(buf[0]=='\r')
	  		      {
	  		    	  if(strncmpi((char *)received,"VER?\r",5)==0)
	  		    	  {
	  		    		  vcp_send(command,10);
	  		    		 // HAL_Delay(1000);
	  		    		  vcp_send(rcv,20);
	  		    	  }
	  		    	  else if(strncmpi((char *)received,"CMP?\r",5)==0)
	  		    	  {
	  		    		  vcp_send(company,20);
	  		    		 // HAL_Delay(1000);
	  		    		  vcp_send(rcv,20);
	  		    	  }
	  		    	  else if(strncmpi((char *)received,"PRG?\r",5)==0)
	  		    	  {
	  		    		  vcp_send(program,11);
	  		    		 // HAL_Delay(1000);
	  		    		  vcp_send(rcv,20);
	  		    	  }
	  		    	  else{
	  		    		  vcp_send(error,20);
	  		    		  vcp_send(rcv,20);
	  		    	  }
	  		    	  str_len=0;
	  		    	  //flush he buffer
	  		    	  memset(received, 0, 6);
	  		      }
	  		      //len = vcp_send (buf, len);
	  	      }
  }
}

void ProcessCmdQueue()
{
	char cmdStr[CMD_MAX_LENGTH+4] = {0};
	//memset(cmdStr, 0, CMD_MAX_LENGTH);

	if( cmdOS.Head != cmdOS.Tail )
	{
		if( cmdOS.Msg[cmdOS.Tail].CommandType == BINARY_DATA )
		{	//This is a frame of bootloader data. Since we are not in the bootloader, it is not for us. Forward it to our friends.
			sprintf(cmdStr, "%s", (char*) cmdOS.Msg[ cmdOS.Tail ].Data);
			if( cmdOS.Msg[ cmdOS.Tail ].AxisOrigin == CMD_ORIGIN_USART1)
			{
				//We have received bootloader data from the RS485 network.
				//We need to route this back out to USB in case the bootloader software is working through this axis.
				//*** ADD CODE *** --> transmit via USB
			}
			else
			{
				//We have received bootloader data from the either USB or RS232
				//Not for us. Need to route this to the RS485 network
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*)cmdStr, strlen(cmdStr));
				while(READ_RS485_DIR);	//this looks like it waits for the tx to complete.
			}
		}
		//*** ADD CODE *** --> implement internal programming later
		/*
		else if( cmdOS.Msg[cmdOS.Tail].CommandType == MOTION_PROGRAM )
		{
			//New system origin: Local Program.
			if( Recording() )
			{
				RecordProgramLine(&cmdOS.Msg[ cmdOS.Tail ]);
			}
			//If this came in on the CAN bus, we don't need to forward it.
			if( cmdOS.Msg[ cmdOS.Tail ].AxisOrigin != CMD_ORIGIN_CAN1)
			{
				sendCanCmd( &cmdOS.Msg[cmdOS.Tail] );
			}
		}	*/
		//If this came into us through USB or RS232, and it's a global command, we need to forward it to the CAN bus.
		else if( cmdOS.Msg[ cmdOS.Tail ].tAxis == getAxis() ||  cmdOS.Msg[ cmdOS.Tail ].tAxis == 0x00 ) {
			//Is this Message a response from another axis? If so, forward it to its point of origin into the system.
			if( cmdOS.Msg[ cmdOS.Tail ].Data[0] == '#' )
			{
				if(cmdOS.Msg[ cmdOS.Tail ].SysOrigin == CMD_ORIGIN_PROGRAM)
				{
					cmdOS.Msg[ cmdOS.Tail ].SysOrigin = ProgOutputRoute;	//If this came from a program, route it to the right spot.
				}
				else
				{
					//Response from another controller has been received. This type of response can only be received on the RS485 (USART1)
					//This response needs to be fed back to USB and RS232 in the event that the query originated from there
					sprintf(cmdStr, "%s", (char*) cmdOS.Msg[ cmdOS.Tail ].Data);
//!					UART0_Puts(cmdStr);
//!					UART2_Puts(cmdStr);
				}
			}
			else
			{
				//this is a command. It could have come from RS232, USB, or RS485.
				//this command is either broadcast or addressed to this controller
				if(cmdOS.Msg[ cmdOS.Tail ].tAxis == 0x00)
				{
					//it is a broadcast command. we need to transmit command over RS485 if is was received via USB/RS232
					if(cmdOS.Msg[cmdOS.Tail].AxisOrigin != CMD_ORIGIN_USART1)
					{
						sprintf(cmdStr, "0%s", (char*) cmdOS.Msg[ cmdOS.Tail ].Data);
						HAL_UART_Transmit_DMA(&huart1, (uint8_t*)cmdStr, strlen(cmdStr));
					}
				}
				//whether it is broadcast or addresses to this controller, we need to execute command. do that here
				if(ParseCommand( &cmdData, &cmdOS.Msg[ cmdOS.Tail ] ) )
				{
					ExecuteCommand(cmdData);
				}
				else if( cmdData.rInfo.AxisOrigin == CMD_ORIGIN_PROGRAM)
				{	//Command failed to parse. Program?
					EndProgram();
				}
			}
		}
		else
		{
			// Not addressed to this controller. If the command came through RS232 or USB we need to put on the RS485/CAN network
			// Otherwise we ignore it
			if( (cmdOS.Msg[ cmdOS.Tail ].SysOrigin != CMD_ORIGIN_USART1) && (cmdOS.Msg[ cmdOS.Tail ].SysOrigin != CMD_ORIGIN_CAN1) )
			{
				//If we receive a command from RS232 or FTDI that is not addressed to this controller we need to transmit to RS485.
				sprintf(cmdStr, "%d%s", cmdOS.Msg[cmdOS.Tail].tAxis, (char*) cmdOS.Msg[ cmdOS.Tail ].Data);
				cmdStr[strlen(cmdStr)-1] = '\n';
				cmdStr[strlen(cmdStr)] = '\r';
				cmdStr[strlen(cmdStr)+1] = '\0';
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*)cmdStr, strlen(cmdStr));
			}
			else if( cmdOS.Msg[ cmdOS.Tail ].Data[0] == '#' )
			{
				//enter here if we have received data back from USART1.
				//this could be a response from a controller somewhere in the stack
				//we need to feed this data back out through RS232/USB
				sprintf(cmdStr, "%s", (char*) cmdOS.Msg[ cmdOS.Tail ].Data);
//!				UART0_Puts(cmdStr);
//!				UART2_Puts(cmdStr);
			}

		}
		//Remove message from queue.
		memset(cmdOS.Msg[ cmdOS.Tail ].Data, 0, CMD_MAX_LENGTH);
		dequeueCmd( &cmdOS );
	}
}

void CheckFlags()
{
	/*
	Check485_SendStatus();

	if( line_count_usb )
	{
		procQueUSART0( &cmdOS );
		line_count_usb--;
	}

	if( line_count_232 )
	{
		procQueUSART2( &cmdOS );
		line_count_232--;
	}
	*/
	if (line_count_nano)
	{
		procQueUSART3();
		line_count_nano--;
	}
	if( line_count_485 )
	{
		procQueUSART1( &cmdOS );
		line_count_485--;
	}
	/*
	if(NoEncoder)// && !getHomeInProgress())
	{
		NoEncoderSon();
		AddError(NO_ENCODER_DETECTED,"...");
		if(ProgramRunning())
		{
			EndProgram();
		}
		NoEncoder = 0;
	}

	if(IndexNotFound){
		AddError(INDEX_NOT_FOUND, "HOM");
		IndexNotFound = 0;
	}

	//TODO --> add possible stop here due to SPI communication fail
	*/
}

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

/*
 * motionspi.c
 *
 *  Created on: Aug 4, 2023
 *      Author: tjudge
 */

#include "main.h"
#include "sysinfo.h"
#include "motionspi.h"
#include "motion.h"

extern SPI_HandleTypeDef hspi2;

uint8_t spi_rx_data[4];
uint32_t rx_data;
int8_t spiTxFlag;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	//spiTxFlag++;
}

void SendMotionSPIData(uint8_t opcode, uint8_t parameterID, int32_t data)
{
	uint8_t spiData[8];

	spiData[0] = HEADER_ARM;
	spiData[1] = opcode;
	spiData[2] = parameterID;
	spiData[3] = data & 0xFF;
	spiData[4] = (data>>8) & 0xFF;
	spiData[5] = (data>>16) & 0xFF;
	spiData[6] = (data>>24) & 0xFF;

	//transmit 7 bytes of data to the motherboard MCU
	HAL_SPI_Transmit_IT(&hspi2, spiData, 7);
}

void SendMotionSPIDataExtended(uint8_t opcode, uint8_t parameterID, int32_t word1, int32_t word2, int32_t word3)
{
	uint8_t spiData[15];
	uint16_t i=0;

	spiTxFlag = 1;

	spiData[0] = HEADER_ARM;
	spiData[1] = opcode;
	spiData[2] = parameterID;

	//PID output data
	spiData[3] = word1 & 0xFF;
	spiData[4] = (word1>>8) & 0xFF;
	spiData[5] = (word1>>16) & 0xFF;
	spiData[6] = (word1>>24) & 0xFF;

	//encoder data
	spiData[7] = word2 & 0xFF;
	spiData[8] = (word2>>8) & 0xFF;
	spiData[9] = (word2>>16) & 0xFF;
	spiData[10] = (word2>>24) & 0xFF;

	//trajectory data
	spiData[11] = word3 & 0xFF;
	spiData[12] = (word3>>8) & 0xFF;
	spiData[13] = (word3>>16) & 0xFF;
	spiData[14] = (word3>>24) & 0xFF;


	//transmit 15 bytes of data to the motherboard MCU
	HAL_SPI_Transmit_IT(&hspi2, spiData, 15);

	spi_rx_data[0] = 0;
	spi_rx_data[1] = 0;
	spi_rx_data[2] = 0;
	spi_rx_data[3] = 0;

	//add delay so slave can latch data
	while (i<100)
	{
		i++;
		__NOP();
	}

	//Send out 4 bytes of clock pulses to receive data
	HAL_SPI_Receive(&hspi2, spi_rx_data, 4, 1000);
	SPI2_NSS_OFF;
	spiTxFlag = 0;
}

int8_t SendParamSPIDataExtended(uint8_t opcode, uint8_t parameterID, int32_t word1, int32_t word2, int32_t word3)
{
	int16_t sendCounter = 0;


	while (sendCounter < 4)
	{
		SPI2_NSS_ON;
		HAL_Delay(100);
		SendMotionSPIDataExtended(opcode, parameterID, word1, word2, word3);
		if (spi_rx_data[0]==SPI_ACK_HEADER && spi_rx_data[1]==parameterID && spi_rx_data[2]==SPI_ACK_BYTE) return 1;
		else sendCounter++;
		
		HAL_Delay(10);
	}
	
	return 0;
				
}

void ReceiveSPIData(uint8_t opcode, uint8_t parameterID)
{
		uint8_t spiData[8];
		uint16_t i=0;

		//data byte in this first packet are garbage. Slave does not care about their value
		spiData[0] = HEADER_ARM;
		spiData[1] = opcode;
		spiData[2] = parameterID;
		spiData[3] = 0;
		spiData[4] = 0;
		spiData[5] = 0;
		spiData[6] = 0;

		//Transmit parameter request. Data byte are garbage, slave will not read them
		HAL_SPI_Transmit(&hspi2, spiData, 7, 1000);

		//wait for data to be sent out
		while (hspi2.TxXferCount > 0);

		//add delay so slave can latch data
		while (i<100)
		{
			i++;
			__NOP();
		}

		//Send out 4 bytes of clock pulses to receive data
		HAL_SPI_Receive(&hspi2, spi_rx_data, 4, 1000);
}

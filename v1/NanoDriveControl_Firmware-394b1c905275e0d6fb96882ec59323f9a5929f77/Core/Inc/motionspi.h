/*
 * motionspi.h
 *
 *  Created on: Aug 4, 2023
 *      Author: tjudge
 */

#ifndef INC_MOTIONSPI_H_
#define INC_MOTIONSPI_H_

#define PHASE1						1
#define PHASE2						2
#define PHASE3						3
#define PHASE4						4

#define HEADER_ARM					0xAB		//address for motherboard ARM

//***** OPCODE *****//
#define OPCODE_MOTION_DATA			0x40
#define OPCODE_MOTOR_STATUS_DATA	0x43
#define OPCODE_PARAM_WRITE			0x60
#define OPCODE_PARAM_READ			0x63

//***** PARAMETER ID *****//
#define PAR_MOTION_DATA				0x11

#define	PAR_MOTOR_ENABLE			0x70
#define PAR_MOTOR_TYPE				0xA0
#define PAR_MOTOR_PITCH				0xA1
#define PAR_PWM_FREQ				0XA2
#define PAR_ENC_RESOLUTION			0xA3
#define PAR_PH_RESISTANCE			0xA4
#define	PAR_PH_INDUCTANCE			0xA5
#define PAR_MOTOR_CURRENT			0xA6
#define PAR_MOTOR_CONFIG			0xAA
#define PAR_VOLTAGE_REF				0xAB

#define PAR_MOTOR_DEGREE			0xB0
#define PAR_MICROSTEP				0xB1

#define PAR_WF_OFFSET				0xC0

#define PAR_SILENT_ENABLE			0xC7
#define PAR_SILENT_FREQUENCY		0xC8

#define PAR_TRANSITION_DATA			0xD3
#define PAR_WF_AMPLITUDE			0xDB

#define PAR_SLAVE_VERSION			0xE0

#define SPI_ACK_HEADER				0x77
#define SPI_ACK_BYTE				0x88

extern int8_t spiTxFlag;

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi);
void SendMotionSPIData(uint8_t opcode, uint8_t parameterID, int32_t data);
void SendMotionSPIDataExtended(uint8_t opcode, uint8_t parameterID, int32_t word1, int32_t word2, int32_t word3);
int8_t SendParamSPIDataExtended(uint8_t opcode, uint8_t parameterID, int32_t word1, int32_t word2, int32_t word3);;
void ReceiveSPIData(uint8_t opcode, uint8_t parameterID);

#endif /* INC_MOTIONSPI_H_ */

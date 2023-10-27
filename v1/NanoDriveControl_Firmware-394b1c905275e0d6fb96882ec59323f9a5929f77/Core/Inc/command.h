/*
 * Command.h
 *
 *  Created on: May 16, 2023
 */

#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_


typedef char bool;
#define false 0
#define true 1

#define CMD_BUFFER_SIZE 8
#define CMD_BUFFER_MASK ( CMD_BUFFER_SIZE - 1 )

#if ( CMD_BUFFER_SIZE & CMD_BUFFER_MASK )
#error Cmd buffer size is not a power of 2
#endif

#define STANDARD_COMMAND	0x01
#define BINARY_DATA			0x03
#define MOTION_PROGRAM		0x04

//Update these
#define CMD_ORIGIN_USART0				( 0x00 )
#define CMD_ORIGIN_CAN1					( 0x01 )
#define CMD_ORIGIN_USART2				( 0x02 )
#define CMD_ORIGIN_USART1				( 0x03 )
#define CMD_ORIGIN_PROGRAM				( 0x04 )

//!Acceptable command parameter formats
//BIT - The parameter consists of one bit of information: 0 or 1
//INT - The parameter is an integer with 9 digits allowed
//SHORT - The parameter is a floating point number with 6 digits allowed in front of the decimal and 6 behind
//FULL - The parameter is a floating point number with 6 digits allowed in front of the decimal place and 9 behind

enum ParameterType { NONE, MBIT, MINT, MSHORT, MFULL, MTEXT };

#define CMD_MAX_LENGTH           	( 0x80 )

//!Execution Modes
enum exec_modes{ IMM, PRGM,	PRGM_IMM };		//Did this come in from one of the external comm. interfaces, or is it
											//running as part of an internal motion program
//!Routing Info Structure
typedef struct {
	uint8_t tGate;				// Target Stack Address 0 - 8
	uint8_t tAxis; 				// Target Axis 0 - 63
	uint8_t sGate; 				// Source Stack Address 0 - 8
	uint8_t sAxis; 				// Source Axis 0 - 63
	uint8_t Length;				// Message Length
	uint8_t SysOrigin;			// On which peripheral did the message come into the system
	uint8_t AxisOrigin;			// On which peripheral did the message come in on this axis
	uint8_t CommandType;		// Is this a motion command? Firmware upload data? Part of a motion program?
	uint8_t Data[ CMD_MAX_LENGTH ];   // Message Data
}sRouteInfo;

typedef struct {
	uint8_t Head;
	uint8_t Tail;
	sRouteInfo Msg[ CMD_BUFFER_SIZE ]; // NUMBER OF COMMANDS
}sCmdQue;

//!Internal Command Structure
typedef struct
{
	uint16_t  AxisNum;		//!<Board that command is addressed to
	uint8_t   Instr;		//!<Instruction number
	bool      Request;		//!<Read Operation?
	uint8_t   numArgs;
	int64_t	  Args[3];		//!<Parameters to Instruction
	uint8_t   ArgCheck[4];	//!<Which parameters are present

	enum exec_modes Context;//!<Are we inside a program?
	char      Instr_Str[3];	//!<3 letter instruction
	char      stringArg[CMD_MAX_LENGTH - 4];	//!<String Argument. ArgCheck[3] corresponds to stringArg.
} sCmdData;

typedef struct
{
	sRouteInfo rInfo;
	sCmdData cmdData;
} eCmd;

typedef int16_t (*t_cmdImpl)(eCmd);

typedef struct
{
	uint8_t NumPar;
	uint8_t ParType[3];
}ParamDesc;

typedef struct
{
	uint16_t cmdId;
	char InstrStr[4];
	t_cmdImpl impl;
	bool locked;
	ParamDesc format;
}MicronixCmdDesc;

int ParseCommand( eCmd *Cmd, sRouteInfo *RInfo );
MicronixCmdDesc* CommandLookup(sCmdData *target);
uint8_t CheckEligibility(uint8_t Origin, uint16_t Instr, uint16_t *ErrorCode, uint8_t query);

#endif /* INC_COMMAND_H_ */

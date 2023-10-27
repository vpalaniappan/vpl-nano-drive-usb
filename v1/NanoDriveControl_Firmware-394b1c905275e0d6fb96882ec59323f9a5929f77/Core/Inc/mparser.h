/*
 * mParser.h
 *
 *  Created on: May 17, 2023
 */

#ifndef INC_MPARSER_H_
#define INC_MPARSER_H_

#define FAIL 0
#define SUCCESS 1

#define TEXT_PARAM_SIZE_LIMIT CMD_MAX_LENGTH - 4
#define MINT_DIGIT_LIMIT 9
#define MSHORT_DECIMAL_DIGIT_LIMIT 6
#define MFULL_DECIMAL_DIGIT_LIMIT 9
#define INTEGER_PLACE_LIMIT 6  		//Allow this many digits before the decimal point (999999mm)

//Prototypes
int ParseCommand( eCmd *Cmd, sRouteInfo *RInfo );
int GetInstruction( sCmdData *target, sRouteInfo *source );
int DetectRequest( sCmdData *Params, uint8_t* CmdString, uint16_t CmdLength);
int GetArgs( sCmdData *Params, uint8_t* CmdString, uint16_t CmdLength, MicronixCmdDesc *cmdFormat );
int GetParameter( sCmdData *Params, uint8_t *CmdString, uint16_t CmdLength,
		MicronixCmdDesc *cmdFormat, uint16_t *buf_pos, uint8_t ArgNum, char EndChar);
uint32_t combine_chars(uint8_t* data,uint8_t num_digits);
uint8_t is_letter(uint8_t x);


#endif /* INC_MPARSER_H_ */

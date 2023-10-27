/*
 * mParser.c
 *
 *  Created on: May 17, 2023
 */

#include "stdint.h"
#include "string.h"
#include <command.h>
#include <commandlist.h>
#include <error.h>
#include <mparser.h>

//This function converts a single command from ASCII representation into
//our internal data structure.
int ParseCommand( eCmd *Cmd, sRouteInfo *RInfo )
{
	//cmdReqs myCmdReqs;
	MicronixCmdDesc *desc;

	//Initialize Fields
	memcpy(&Cmd->rInfo, RInfo, sizeof(sRouteInfo)); //Copy Routing Info
//	CopyRoutingInfo(CmdData, RInfo);
//	ClearCmdData(CmdData);
	Cmd->cmdData.AxisNum = RInfo->tAxis;
	Cmd->cmdData.Context = IMM;
	Cmd->cmdData.Request = 0;
	Cmd->cmdData.numArgs = 0;
	Cmd->cmdData.Args[1] = 0;
	Cmd->cmdData.Args[2] = 0;
	Cmd->cmdData.ArgCheck[0] = 0;
	Cmd->cmdData.ArgCheck[1] = 0;
	Cmd->cmdData.ArgCheck[2] = 0;

	if(!GetInstruction( &Cmd->cmdData, RInfo ))
	{
		AddError(INVALID_COMMAND,Cmd->cmdData.Instr_Str);
		return ERR_FAIL;
	}

	desc = CommandLookup (&Cmd->cmdData);

	if( desc->cmdId == NUL)
	{
		AddError(INVALID_COMMAND,Cmd->cmdData.Instr_Str);
		return ERR_FAIL;
	}

	if(DetectRequest(&Cmd->cmdData, Cmd->rInfo.Data, Cmd->rInfo.Length ))
	{
		return ERR_SUCCESS;
	}

	if(desc->format.NumPar > 0)
	{
		if(!GetArgs( &Cmd->cmdData, Cmd->rInfo.Data, Cmd->rInfo.Length, desc ))
		{
			return ERR_FAIL;
		}
	}
	else
	{
		if(Cmd->rInfo.Length == 4)
		{
			return ERR_SUCCESS;
		}
		else
		{
			AddError(INVALID_CHARACTER_IN_PARAMETER,Cmd->cmdData.Instr_Str);
			return ERR_FAIL;
		}
	}
	return ERR_SUCCESS;
}

int GetInstruction( sCmdData *target, sRouteInfo *source )
{
	int i=0;
	char x;
	//read 3 letters from buffer
	while(i<3)
	{
		x = source->Data[i];
		if(is_letter(x))
		{
			//force upper case
			if(x>='a' && x<='z')
				x-=' ';
			target->Instr_Str[i++] = x;
		}
		else return ERR_FAIL;
	}
	return ERR_SUCCESS;
}

int DetectRequest( sCmdData *Params, uint8_t* CmdString, uint16_t CmdLength)
{
	if( CmdLength == 5 && CmdString[3] == '?' )
	{
		//Request, no Parameter. i.e. 1POS?
		Params->Request = 1;
		return ERR_SUCCESS;
	}
	else if ( CmdLength == 6 && CmdString[4] == '?' )
	{
		//Request, one-digit parameter i.e. 1LST1?
		if( CmdString[3] >= '0' && CmdString[3] <= '9')
		{
			Params->Request = 1;
			Params->ArgCheck[0] = 1;
			Params->Args[0] = CmdString[3] - '0';
			return ERR_SUCCESS;
		}
	}
	else if( CmdLength == 7 && CmdString[5] == '?')
	{
		//Request, two digit parameter i.e. 1LST10?
		if((CmdString[3] >= '0' && CmdString[3] <= '9') &&
				(CmdString[4] >= '0' && CmdString[4] <= '9'))
		{
			Params->Request = 1;
			Params->ArgCheck[0] = 1;
			Params->Args[0] = (CmdString[3] - '0')*10 + (CmdString[4] - '0');
			return ERR_SUCCESS;
		}
	}
	Params->Request = 0;
	return ERR_FAIL;
}

int GetArgs( sCmdData *Params, uint8_t* CmdString, uint16_t CmdLength, MicronixCmdDesc *cmdDesc )
{
	uint8_t n, arg_num = 0;
	uint16_t buf_pos = 3;

	while( arg_num < (cmdDesc->format.NumPar - 1) )
	{
		//If we hit a '\r' before we see all parameters, return Fail
		if( ( n = GetParameter( Params, CmdString, CmdLength, cmdDesc, &buf_pos, arg_num, ',' ) ) )
		{
			if( n == '\r' || n == ';')
			{
				//We should not have run into these yet.
				AddError(INCOMPLETE_PARAMETER_LIST,Params->Instr_Str);
				return ERR_FAIL;
			}
		}
		else
		{
			//Error Added In GetParameter.
			return ERR_FAIL;
		}
		arg_num++;
	}

	if( ( n = GetParameter( Params, CmdString, CmdLength, cmdDesc, &buf_pos, arg_num, ';' ) ) )
	{
		if( n == '\r' || n == ';')
		{
			return ERR_SUCCESS;
		}
	}

	return ERR_FAIL;
}

uint8_t parse_dbgptr;
int GetParameter( sCmdData *Params, uint8_t *CmdString, uint16_t CmdLength, MicronixCmdDesc *cmdDesc, uint16_t *buf_pos, uint8_t ArgNum, char EndChar)
{
	uint8_t neg = 0;
	uint8_t digit_count = 0;
	uint64_t a=0;	//before decimal. 6 digits for SHORT OR FULL, 9 digits for a INT
	uint64_t b=0; 	//3 digits after decimal      a . b  c
	uint8_t num_buf[12];

	uint8_t validChars;

	//This will only get here if there is a valid return character in our command string.
											//Last param.
	if( CmdString[*buf_pos] == EndChar || (CmdString[*buf_pos] == '\r'))
	{
		Params->ArgCheck[ArgNum] = 0;
		//increment past separator character
		//(*length)--;
		parse_dbgptr = CmdString[*buf_pos];
		return CmdString[(*buf_pos)++];
	}

	if( cmdDesc->format.ParType[ArgNum] == MTEXT)
	{
		while( CmdString[*buf_pos] != EndChar || CmdString[*buf_pos] != '\r')
		{
			//This failing could be a more specific error. (TEXT_SIZE_LIMIT_EXCEEDED) or the like.
			//													//TEXT_PARAM_SIZE_LIMIT - 1 because we need to NULL terminate
			if(is_letter( CmdString[*buf_pos]) && digit_count < TEXT_PARAM_SIZE_LIMIT-1 )
			{
				Params->stringArg[digit_count++] = CmdString[*buf_pos++];
			}
			else
			{
				AddError(INVALID_PARAMETER_TYPE, Params->Instr_Str);
				return ERR_FAIL;
			}
		}

		//We've hit a terminating character and have a valid string length.
		Params->stringArg[digit_count] = '\0';
		Params->ArgCheck[3] = 1;
		parse_dbgptr = CmdString[*buf_pos];
		return CmdString[*buf_pos];
	}

	if( cmdDesc->format.ParType[ArgNum] == MBIT )
	{

		if( CmdString[*buf_pos] == '1' || CmdString[*buf_pos] == '0')
		{
			Params->ArgCheck[ArgNum] = 1;
			Params->Args[ArgNum] = CmdString[*buf_pos] - '0';
			(*buf_pos)++;
		}
		else
		{
			AddError(INVALID_PARAMETER_TYPE, Params->Instr_Str);
			return ERR_FAIL;
		}

		if( CmdString[*buf_pos] == EndChar || CmdString[*buf_pos] == '\r')
		{
			//Return separator character, and increment past it.
			parse_dbgptr = CmdString[*buf_pos];
			return CmdString[*buf_pos++];
		}
		else
		{
			AddError(INVALID_PARAMETER_TYPE, Params->Instr_Str);
			return ERR_FAIL;
		}
	}

	//Check for negative sign
	if( CmdString[*buf_pos] == '-')
	{
		neg = 1;
		(*buf_pos)++;
	}

	//Get digits before decimal place
	while(CmdString[*buf_pos] != '.')
	{
		if( CmdString[*buf_pos] >= '0' && CmdString[*buf_pos] <= '9')
		{
			num_buf[digit_count++] = CmdString[*buf_pos] - '0';

			//Built in limit for position... +/- 999999 mm for position.
			if(digit_count>INTEGER_PLACE_LIMIT && (cmdDesc->format.ParType[ArgNum] == MSHORT || cmdDesc->format.ParType[ArgNum] == MFULL))
			{
				//Too large of a number
				AddError(INVALID_PARAMETER_TYPE, Params->Instr_Str);
				return ERR_FAIL;
			}
			//Allow integers to have 9 digits in front of decimal.
			if(digit_count>MINT_DIGIT_LIMIT)
			{
				AddError(PARAMETER_TOO_LARGE, Params->Instr_Str);
				return ERR_FAIL;
			}
		}
		//If we hit a \r here, and we are NOT on our last parameter.
		else if( CmdString[*buf_pos] == '\r' || CmdString[*buf_pos] == EndChar)
		{
			if(digit_count > 0 && digit_count <= MINT_DIGIT_LIMIT)
			{
				//Type INT exits through this path
				//only pre-decimal digits present; finished parsing.
				b = 0;
				a = combine_chars(num_buf,digit_count);

				if( cmdDesc->format.ParType[ArgNum] == MFULL )
				{//a could be 6 digits
					Params->Args[ArgNum] = a * 1000000000;
				}
				else if( cmdDesc->format.ParType[ArgNum] == MSHORT )
				{
					Params->Args[ArgNum] = a * 1000000;
				}
				else if(cmdDesc->format.ParType[ArgNum] == MINT )
				{
					Params->Args[ArgNum] = a;
				}

				if(neg)
				{
					Params->Args[ArgNum] = -Params->Args[ArgNum];
				}
				else Params->Args[ArgNum] = Params->Args[ArgNum];

				Params->ArgCheck[ArgNum] = 1;
				return CmdString[(*buf_pos)++];		//Return separator character, increment past.
			}
			else
			{
				AddError(INVALID_PARAMETER_TYPE, Params->Instr_Str);
				return ERR_FAIL;
			}
		}
		else
		{
			AddError(INVALID_CHARACTER_IN_PARAMETER, Params->Instr_Str);
			return ERR_FAIL; // invalid characters
		}

		(*buf_pos)++;
	}

	//There is a decimal point in an integer number. Could ignore it or throw an error.
	if( cmdDesc->format.ParType[ArgNum] == MINT)
	{
		if( CmdString[*buf_pos] == '.' )
		{
			AddError(INVALID_PARAMETER_TYPE, Params->Instr_Str);
			return ERR_FAIL;
		}
	}

	//convert the string of chars in num buf to an integer;
	a = combine_chars(num_buf,digit_count);
	digit_count = 0;
	//pointer is currently on the decimal point, if there is one.
	//increment it to the next digit.
	(*buf_pos)++;

//-------------------
	//Grab data after the decimal point.
	//Up to 9 digits long here. 6 if MSHORT. 9 if MFULL.
	//If we receive a shorter number, we load the correct number of 0's at the end.

	//This will exit successfully, ONLY when we hit a terminating character
	//within an acceptable number of digits. Otherwise it will fail.

	validChars = 1;
	while( CmdString[*buf_pos] != '\r' && CmdString[*buf_pos] != EndChar)
	{
		if(digit_count >= MSHORT_DECIMAL_DIGIT_LIMIT)
		{
			if( cmdDesc->format.ParType[ArgNum] == MSHORT )
			{
				validChars = 0;
			}
			else if( cmdDesc->format.ParType[ArgNum] == MFULL && digit_count >= MFULL_DECIMAL_DIGIT_LIMIT)
			{
				validChars = 0;
			}
		}

		if( validChars && (CmdString[*buf_pos] >= '0' && CmdString[*buf_pos] <= '9'))
		{
			num_buf[digit_count++] = CmdString[(*buf_pos)] - '0';
		}
		else
		{
			AddError(INVALID_CHARACTER_IN_PARAMETER, Params->Instr_Str);
			return ERR_FAIL;
		}
		(*buf_pos)++;
	}
//------------

	//Fill Remainder with 0's
	if( cmdDesc->format.ParType[ArgNum] == MSHORT )
	{
		while(digit_count<MSHORT_DECIMAL_DIGIT_LIMIT)
		{
			num_buf[digit_count++] = 0;
		}
	}
	else if( cmdDesc->format.ParType[ArgNum] == MFULL )
	{
		while(digit_count < MFULL_DECIMAL_DIGIT_LIMIT)
		{
			num_buf[digit_count++] = 0;
		}
	}
//---------
//Calculate the decimal value in the appropriate units.
//For SHORT, it will be a 6 digit integer
//For FULL, it will be a 9 digit integer.

	b = combine_chars(num_buf,digit_count);

	if( cmdDesc->format.ParType[ArgNum] == MFULL )
	{//a could be 6 digits
		Params->Args[ArgNum] = a * 1000000000 + b;
	}
	else if( cmdDesc->format.ParType[ArgNum] == MSHORT )
	{
		Params->Args[ArgNum] = a * 1000000 + b;
	}

	if(neg)
	{
		Params->Args[ArgNum] = -Params->Args[ArgNum];
	}

	Params->ArgCheck[ArgNum] = 1;
	parse_dbgptr = CmdString[*buf_pos];
	return CmdString[(*buf_pos)++];	// increment past end character in case get_par_until gets called again
}

//Convert arrary of integers [1,2,3,4,5] into integer 12345
uint32_t combine_chars(uint8_t* data,uint8_t num_digits)
{
	uint32_t x = 0, place_mult = 1;
	uint8_t i = 0;

	if(num_digits <= 9)
	{
		while(i<num_digits)
		{
			x += data[num_digits - i++ - 1] * place_mult;
			place_mult = 10 * place_mult;
		}
		return x;
	}
	else return ERR_FAIL;
}

//!Checks if character is a letter.
uint8_t is_letter(uint8_t x)
{
	if((x>='A' && x<='Z') || (x>='a' && x<='z'))
		return 1;
	else return 0;
}

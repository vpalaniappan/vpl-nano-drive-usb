/*
 * Error.c
 *
 *  Created on: May 17, 2023
 */
#include "stdint.h"
#include "stdio.h"
#include "main.h"
#include "command.h"
#include "commandlist.h"
#include "sysinfo.h"
#include "error.h"

//!Error Buffer
struct ErrorBuffer err_buf;

//!Error Flag
char Errors = 0;

void AddError(uint16_t ErrNum, char *y)
{
	//If this is the first error, turn on the error light.
	if(Errors == 0)
	{
		Errors = 1;
		TURN_ON_RED_LED;
		TURN_OFF_GREEN_LED;
//!		ERROR_LIGHT_ON;
	}

	//Check if error buffer is full.
	if((err_buf.tail + 1 == err_buf.head) ||
		(err_buf.head == 0 && ((err_buf.tail + 1) & (1<<ERROR_BITS))))
	{
		//error buffer full
		//overwrite oldest entry. increment head.
		if((++err_buf.head) & (1<<ERROR_BITS))
		{
			err_buf.head = 0;
		}
		else ++err_buf.head;
	}

	//tail has reached the end of the array. circle around.
	if((++err_buf.tail) & (1<<ERROR_BITS))
	{
		err_buf.tail = 0;
	}

	//Copy Data into err_buf element
	err_buf.data[err_buf.tail].ErrorNum = ErrNum;
	err_buf.data[err_buf.tail].instr[0] = y[0];
	err_buf.data[err_buf.tail].instr[1] = y[1];
	err_buf.data[err_buf.tail].instr[2] = y[2];
}

//--TODO Implement
void AddErrorRLU(uint16_t ErrNum, enum CommandList y)
{
//	char cmd_str[3];
//	CommandLookup(y,cmd_str);
//	AddError(ErrNum,cmd_str);
}
//head = oldest entry
//tail = newest entry

uint8_t ErrorBufferEmpty()
{
	if(err_buf.head == err_buf.tail)
	{
		return 1;
	}
	return 0;
}

int GetError(struct Error *p)
{
	if(err_buf.head == err_buf.tail)
	{
//!		ERROR_LIGHT_OFF;
		Errors = 0;
		return 0;
	}

	//adjust head pointer
	if((++err_buf.head) & (1<<ERROR_BITS))
	{
		err_buf.head = 0;
	}

	//copy error element into parameter
	p->ErrorNum = err_buf.data[err_buf.head].ErrorNum;
	p->instr[0] = err_buf.data[err_buf.head].instr[0];
	p->instr[1] = err_buf.data[err_buf.head].instr[1];
	p->instr[2] = err_buf.data[err_buf.head].instr[2];

	return 1;
}

void ClearErrors()
{
	Errors = 0;
	err_buf.head = 0;
	err_buf.tail = 0;
//!	ERROR_LIGHT_OFF;
}

int CheckErrors()
{
	if(Errors) return 1;

	return 0;
}

void ErrorLookup(uint16_t num, char *desc)
{
	switch(num)
	{
		case ONE_QUERY_PER_LINE:
			sprintf(desc,"One Query Per Line");
			break;

		case TOO_MANY_COMMANDS_ON_LINE:
			sprintf(desc,"Too Many Commands On Line");
			break;

		case COMMAND_IS_READ_ONLY:
			sprintf(desc,"Command Is Read Only");
			break;

		case RECEIVE_BUFFER_OVERRUN:
			sprintf(desc, "Receive Buffer Overrun");
			break;

		case LINE_CHAR_LIMIT_EXCEEDED:
			sprintf(desc,"Line Character Limit Exceeded");
			break;

		case MISSING_AXIS_NUMBER:
			sprintf(desc,"Missing Axis Number");
			break;

		case MALFORMED_COMMAND:
			sprintf(desc,"Malformed Command");
			break;

		case INVALID_COMMAND:
			sprintf(desc,"Invalid Command");
			break;

		case GLOBAL_QUERY:
			sprintf(desc,"Global Query");
			break;

		case INVALID_PARAMETER_TYPE:
			sprintf(desc,"Invalid Parameter Type");
			break;

		case INVALID_CHARACTER_IN_PARAMETER:
			sprintf(desc,"Invalid Character in Parameter");
			break;

		case COMMAND_CANNOT_BE_USED_IN_GLOBAL_CONTEXT:
			sprintf(desc,"Command Cannot Be Used In Global Context");
			break;

		case PARAMETER_OUT_OF_BOUNDS:
			sprintf(desc,"Parameter Out Of Bounds");
			break;

		case CANT_ALTER_VELOCITY_IN_JOG_MODE:
			sprintf(desc,"Can't Alter Velocity In Jog Mode");
			break;

		case NOT_IN_JOG_MODE:
			sprintf(desc,"Cannot Enter Jog Mode During Motion");
			break;

		case TRACE_ALREADY_IN_PROGRESS:
			sprintf(desc,"Trace Already In Progress");
			break;

		case TRACE_DID_NOT_COMPLETE:
			sprintf(desc,"Trace Did Not Complete");
			break;

		case CAPTURE_DID_NOT_COMPLETE:
			sprintf(desc,"Capture Did Not Complete");
			break;

		case COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION:
			sprintf(desc,"Command Cannot Be Executed During Motion");
			break;

		case MOVE_OUTSIDE_SOFT_LIMITS:
			sprintf(desc,"Move Outside Soft Limits");
			break;

		case READ_NOT_AVAILABLE_FOR_THIS_COMMAND:
			sprintf(desc,"Read Not Available For This Command");
			break;

		case COMMAND_NOT_AVAILABLE_IN_THIS_VERSION:
			sprintf(desc,"Command Will Be Available In Future Version");
			break;

		case ANALOG_ENCODER_NOT_AVAILABLE_IN_THIS_VERSION:
			sprintf(desc,"Analog Encoder Not Available In This Version");
			break;

		case HOME_REQUIRES_ENCODER:
			sprintf(desc,"Home Operation Requires An Encoder");
			break;

		case MOVE_TO_LIMIT_REQUIRES_ENCODER:
			sprintf(desc,"Moving To Limit Requires An Encoder");
			break;

		case PROGRAM_SIZE_LIMIT_EXCEEDED:
			sprintf(desc,"Program Size Limit Exceeded");
			break;

		case NO_ENCODER_DETECTED:
			sprintf(desc,"No Encoder Detected");
			break;

		case INDEX_NOT_FOUND:
			sprintf(desc,"Index Not Found");
			break;

		case MOTOR_IS_DISABLED:
			sprintf(desc,"Motor Is Disabled");
			break;
		case MOTOR_IS_ENABLED:
			sprintf(desc,"Motor Is Enabled");
			break;
		case END_COMMAND_MUST_BE_ON_ITS_OWN_LINE:
			sprintf(desc,"End Command Must Be On Its Own Line");
			break;
		case COMMAND_ONLY_VALID_WITHIN_PROGRAM:
			sprintf(desc,"Command Only Valid Within Program");
			break;

		case PROGRAM_ALREADY_EXISTS:
			sprintf(desc,"Program Already Exists");
			break;

		case PROGRAM_DOESNT_EXIST:
			sprintf(desc,"Program Doesn't Exist");
			break;

		case CANNOT_MOVE_INTO_HARD_LIMIT:
			sprintf(desc,"Cannot Move Into Hard Limit");
			break;

		case FAILED_TO_READ_PROGRAM:
			sprintf(desc,"Failed To Read Program");
			break;

		case INCOMPLETE_PARAMETER_LIST:
			sprintf(desc,"Command Requires Additional Parameters");
			break;

		case PARAMETER_TRANSFER_FAILURE:
			sprintf(desc,"Parameter Failed To Be Set");
			break;

		case READ_OPERATIONS_NOT_ALLOWED_INSIDE_PROGRAM:
			sprintf(desc,"Read Operations Not Allowed Inside Program");
			break;

		case COMMAND_NOT_ALLOWED_DURING_PROGRAM:
			sprintf(desc,"Command Not Allowed While Program Is In Progress");
			break;

		case END_OF_TRAVEL_LIMIT:
			sprintf(desc,"The Numeric Travel Range Has Been Exceeded.");
			break;

		case INCOMPLETE_STATE_VALIDATION_FUNCTION:
			sprintf(desc,"Not Implemented Yet");
			break;

		case COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS:
			sprintf(desc,"Command Not Allowed With Current Settings");
			break;

		case HOME_IN_PROGRESS:
			sprintf(desc, "Home In Progress");
			break;

		case DRIVER_ERROR:
			sprintf(desc, "Driver Error");
			break;

		case CANNOT_START_MOVE_DURING_JOG_MODE:
			sprintf(desc, "Cannot Start Move During Jog Mode");
			break;

		case CANNOT_CHANGE_TARGET:
			sprintf(desc,"Cannot Change Target");
			break;

		case TARGET_CHANGE_CANNOT_OCCUR_DURING_DECEL_PHASE:
			sprintf(desc,"Cannot Change Target During Deceleration");
			break;

		case TARGET_CHANGE_CANNOT_OCCUR_DURING_ACCEL_PHASE:
			sprintf(desc,"Cannot Change Target During Acceleration");
			break;

		case JOG_NOT_PERMITTED:
			sprintf(desc,"Jog Not Allowed During Move");
			break;

		case POSITION_CHANGE_NOT_ALLOWED_DURING_ACCEL:
			sprintf(desc,"Position Change Not Allowed During Acceleration");
			break;

		case POSITION_CHANGE_NOT_ALLOWED_DURING_DECEL:
			sprintf(desc,"Position Change Not Allowed During Deceleration");
			break;

		case VELOCITY_CHANGE_NOT_ALLOWED_DURING_ACCEL:
			sprintf(desc,"Velocity Change Not Allowed During Acceleration");
			break;

		case VELOCITY_CHANGE_NOT_ALLOWED_DURING_DECEL:
			sprintf(desc,"Velocity Change Not Allowed During Deceleration");
			break;

		case CURRENT_POSITION_OUTSIDE_SOFT_LIMITS:
			sprintf(desc,"Current Position Outside Soft Limits");
			break;

		case UNKNOWN_MESSAGE_ORIGIN:
			sprintf(desc,"Unknown Message Origin");
			break;

		case IO_CONFIGURED_FOR_DIFFERENT_FUNCTION:
			sprintf(desc,"IO Configured For Different Function");
			break;

		case IO_IS_OUTPUT_ONLY:
			sprintf(desc,"IO Is Output Only");
			break;

		case IO_IS_INPUT_ONLY:
			sprintf(desc,"IO Is Input Only");
			break;

		case IO_DIRECTION_NOT_CONFIGURED_PROPERLY:
			sprintf(desc,"IO Direction Not Configured Properly");
			break;

		case IO_FUNCTION_NOT_CONFIGURED_PROPERLY:
			sprintf(desc,"IO Function Not Configured Properly");
			break;

		case IO_FUNCTION_ALREADY_ASSIGNED:
			sprintf(desc,"I/O Function Already Assigned");
			break;

		case COMMAND_IS_GLOBAL_ONLY:
			sprintf(desc,"Command Can Only Be Used In Global Context");
			break;

		case ANNOUNCE_OPERATION_FAILED:
			sprintf(desc,"Announce Operation Failed");
			break;

		//Add error response for silent/feedback mode incompatibility
		case SILENT_AND_FEEDBACK_MODE_CONFLICT:
			sprintf(desc,"Silent Mode Requires Feedback Mode (FBK) 3");
			break;

		default:

			sprintf(desc,"Unknown Error");
			break;
	}
}

/*
 * Error.h
 *
 *  Created on: May 17, 2023
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

#define ERR_SUCCESS 1
#define ERR_FAIL 0


#define RECEIVE_BUFFER_OVERRUN 							10			//!<Too many characters in the receive buffer
#define MOTOR_IS_DISABLED 								11			//!<Trying to move while servo is disabled
#define NO_ENCODER_DETECTED 							12			//!<Trying to do something that requires an encoder but there isn't one
#define INDEX_NOT_FOUND 								13			//!<Home operation could not find the index
#define HOME_REQUIRES_ENCODER 							14			//!<Encoder must be connected to home
#define MOVE_TO_LIMIT_REQUIRES_ENCODER 					15
#define MOTOR_IS_ENABLED								16
														//17
														//18
														//19
#define COMMAND_IS_READ_ONLY 							20			//!<Command can only be used followed by a ?
#define ONE_QUERY_PER_LINE 								21			//!<Can't do two reads on the same line
#define TOO_MANY_COMMANDS_ON_LINE 						22			//!<More than 8 commands on one line
#define LINE_CHAR_LIMIT_EXCEEDED 						23			//!<More than 80 characters on one line
#define MISSING_AXIS_NUMBER 							24			//!<Every command needs to start with an axis number
#define MALFORMED_COMMAND 								25			//!<What
#define INVALID_COMMAND 								26			//!<The command entered is not a command that the controller understands
#define GLOBAL_QUERY 									27			//!<Can only do reads on one axis at a time
#define INVALID_PARAMETER_TYPE 							28			//!<Parameter Type
#define INVALID_CHARACTER_IN_PARAMETER 					29			//!<Non number in parameter. ex. 2$.34
#define COMMAND_CANNOT_BE_USED_IN_GLOBAL_CONTEXT 		30			//!<Some commands can be used in a global context and some cannot.
#define PARAMETER_OUT_OF_BOUNDS 						31			//!<Parameter value is outside the range of valid stuff
#define CANT_ALTER_VELOCITY_IN_JOG_MODE 				32			//!<Why Not? Get rid of this.
#define NOT_IN_JOG_MODE 								33			//!<Can't send a jog command during normal motion
#define TRACE_ALREADY_IN_PROGRESS 						34			//!<Trying to start trace while its already running
#define TRACE_DID_NOT_COMPLETE 							35			//!<Trace did not complete
#define COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION 		36			//!<Most commands are not allowed during motion
#define MOVE_OUTSIDE_SOFT_LIMITS 						37			//!<The parameter to a move command is outside of the soft limits
#define READ_NOT_AVAILABLE_FOR_THIS_COMMAND 			38			//!<Some commands cannot be used in a read context.
#define PROGRAM_NUMBER_OUT_OF_RANGE 					39
#define PROGRAM_SIZE_LIMIT_EXCEEDED 					40			//!<Invalid program number, maybe combine this with parameter out of range
#define PROGRAM_FAILED_TO_RECORD 						41			//!<Error in recording program
#define END_COMMAND_MUST_BE_ON_ITS_OWN_LINE 			42
#define FAILED_TO_READ_PROGRAM 							43
#define COMMAND_ONLY_VALID_WITHIN_PROGRAM 				44
#define PROGRAM_ALREADY_EXISTS 							45
#define PROGRAM_DOESNT_EXIST 							46
#define READ_OPERATIONS_NOT_ALLOWED_INSIDE_PROGRAM 		47
#define COMMAND_NOT_ALLOWED_DURING_PROGRAM 				48
#define INCOMPLETE_PARAMETER_LIST 						49
#define CANNOT_MOVE_INTO_HARD_LIMIT 					50
#define END_OF_TRAVEL_LIMIT 							51
#define HOME_IN_PROGRESS 								52
#define POSITION_CHANGE_NOT_ALLOWED_DURING_ACCEL 		53
#define VELOCITY_CHANGE_NOT_ALLOWED_DURING_ACCEL 		54
#define POSITION_CHANGE_NOT_ALLOWED_DURING_DECEL 		55
#define VELOCITY_CHANGE_NOT_ALLOWED_DURING_DECEL 		56
#define TARGET_CHANGE_CANNOT_OCCUR_DURING_ACCEL_PHASE 	57
#define TARGET_CHANGE_CANNOT_OCCUR_DURING_DECEL_PHASE 	58
#define INCOMPLETE_STATE_VALIDATION_FUNCTION 			59
#define JOG_NOT_PERMITTED 								60
#define CANNOT_START_MOVE_DURING_JOG_MODE 				61
#define CANNOT_CHANGE_TARGET 							62
#define CANNOT_CHANGE_DIRECTION 						63
#define TARGET_POSITION_OUTSIDE_CONTROLLER_TRAVEL_RANGE 64
#define CURRENT_POSITION_OUTSIDE_SOFT_LIMITS 			65
#define PARAMETER_TOO_LARGE 							66
#define DRIVER_ERROR 									67
#define UNKNOWN_MESSAGE_ORIGIN 							68
#define IO_CONFIGURED_FOR_DIFFERENT_FUNCTION 			69
														//70
#define IO_IS_OUTPUT_ONLY 								71
#define IO_IS_INPUT_ONLY 								72
#define IO_DIRECTION_NOT_CONFIGURED_PROPERLY 			73
#define IO_FUNCTION_NOT_CONFIGURED_PROPERLY 			74
#define IO_FUNCTION_ALREADY_ASSIGNED 					75
#define COMMAND_IS_GLOBAL_ONLY 							76
#define ANNOUNCE_OPERATION_FAILED 						77
#define MISSING_PARAMETER 								78
#define PARAMETER_TRANSFER_FAILURE 						79
#define COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS 		80
#define COMMAND_NOT_AVAILABLE_IN_THIS_VERSION 			81			//!<Command not supported by this version of the firmware
#define ANALOG_ENCODER_NOT_AVAILABLE_IN_THIS_VERSION 	82			//!<No Analog Encoder Support
#define SILENT_AND_FEEDBACK_MODE_CONFLICT 				83			//!<Silent mode enable attempt when FBK!=3
#define CAPTURE_DID_NOT_COMPLETE 						84			//!<Position capture did not complete

#define ERROR_BITS 4		//!<Error Buffer is 2^ERROR_BITS long

struct Error
{
	char instr[3];			//!<Instruction that triggered error
	uint16_t ErrorNum;	//!<Error Number
};

//!Circular Buffer of Errors
struct ErrorBuffer {
	struct Error data[1<<ERROR_BITS];	//!<Array of Error structs
	uint16_t head;					//!<Head Pointer
	uint16_t tail;					//!<Tail Pointer
};

void AddError(uint16_t,char*);
int GetError(struct Error*);
void ErrorLookup(uint16_t, char*);
void ClearErrors();
int CheckErrors();
void AddErrorRLU(uint16_t, enum CommandList);
uint8_t ErrorBufferEmpty();


#endif /* INC_ERROR_H_ */

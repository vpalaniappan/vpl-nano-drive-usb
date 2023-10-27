/*
 * Position.c
 *
 *  Created on: May 31, 2023
 */
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "command.h"
#include "commandlist.h"
#include "error.h"
#include "sysinfo.h"
#include "position.h"
#include "motion.h"
//#include "lpc_Trajectory.h"

int64_t TrajectoryController(void);
enum MotionPhases getTrajState();
int8_t getTrajDir();
int64_t getTargetPos();

extern volatile uint16_t InPositiveLimit;
extern volatile uint16_t InNegativeLimit;

int64_t CurrentPos;			//Holds our current theoretical position in picometers. Important.
int64_t PosInc = 0;			//Relative increment returned from the trajectory.
int32_t rollover = 0;
int64_t PosLog[400];
int64_t LogPtr = 0;

extern TIM_HandleTypeDef htim3;
//Relative trajectory makes rotary easier. I can reset position here @ 360 degrees
void PositionHandler(void)
{
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	HAL_TIM_Base_Stop_IT(&htim3);

	PosInc = TrajectoryController();

	if(PosInc > 0)
	{
		if(CurrentPos + PosInc < CurrentPos)
		{
			rollover++;
			CurrentPos = PosInc - (MAX_POSITION_POS - CurrentPos);
		}
		else
		{
			CurrentPos += PosInc;
		}
	}
	else if(PosInc < 0)
	{
		if(CurrentPos + PosInc > CurrentPos)
		{
			rollover++;
			CurrentPos = PosInc + (MAX_POSITION_NEG - CurrentPos);
		}
		else
		{
			CurrentPos += PosInc;
		}
	}

	PosLog[LogPtr++] = CurrentPos;
	if(LogPtr == 400)
	{
		LogPtr = 0;
	}
}

char CheckMove(int64_t myArg, enum CommandList cmd)
{
	int64_t MoveTarget, MoveLength;
	enum MotionPhases TS = getTrajState();
	int8_t current_dir;

	//TODO --> look into adding this silent mode/FBK check back in. removing for development
	/*
	if (SilentModeEnable && (FeedbackMode != CLOSED_LOOP))
	{
		AddError(SILENT_AND_FEEDBACK_MODE_CONFLICT, "MVR");
		return 0;
	}
	*/
	if(cmd == MVR)
	{
		if(TS == NO_MOTION)
		{
			MoveTarget = CurrentPos + myArg;
			MoveLength = myArg;
		}
		else if(TS == CONSTANT_VELOCITY)
		{
			current_dir = getTrajDir();
			MoveTarget = getTargetPos();
			//Look for direction change. If the new move is in the same direction, we accumulate.
			if(current_dir == 1 && myArg > 0)
			{
				MoveTarget += myArg;
				MoveLength = MoveTarget - CurrentPos;
			}
			else if(current_dir == -1 && myArg < 0)
			{
				MoveTarget += myArg;
				MoveLength = MoveTarget - CurrentPos;
			}
			else
			{	//move is in new direction.
				MoveTarget = CurrentPos + myArg;
				MoveLength = myArg;
			}
		}
		else return 1;
	}
	else if(cmd == MVA)
	{
		if(TS == NO_MOTION || TS == CONSTANT_VELOCITY)
		{
			MoveTarget = myArg;
			MoveLength = MoveTarget - CurrentPos;
		}
		else return 1;
	}
	else if(cmd == MVT)
	{
		if(TS == CONSTANT_VELOCITY)
		{
			MoveTarget = getTargetPos() + myArg;
			MoveLength = MoveTarget - CurrentPos;
		}
		else return 1;
	}
	else if(cmd == MSR)
	{	//Only allowed during stand-still
		MoveTarget = CurrentPos + myArg;
		MoveLength = myArg;
	}
	else if(cmd == MSA)
	{   //Only allowed during stand-still
		MoveTarget = myArg;
		MoveLength = MoveTarget - CurrentPos;
	}
	else return 0;

	//Check System Soft Limits
	if(MoveTarget > SYS_TLP || MoveTarget < SYS_TLN)
	{
		AddErrorRLU(TARGET_POSITION_OUTSIDE_CONTROLLER_TRAVEL_RANGE,cmd);
		return 0;
	}

	//Check User Soft Limits
	if((LimitConfig & SOFT_LIMIT_MASK) && (MoveTarget < tln || MoveTarget > tlp))
	{
		AddErrorRLU(MOVE_OUTSIDE_SOFT_LIMITS,cmd);
		return 0;
	}

	if(LimitConfig & HARD_LIMIT_MASK)
	{
		if(InPositiveLimit && MoveLength > 0)
		{
			//Can't move that way
			AddErrorRLU(CANNOT_MOVE_INTO_HARD_LIMIT,cmd);
			return 0;
		}
		else if(InNegativeLimit && MoveLength < 0)
		{
			//Can't move that way.
			AddErrorRLU(CANNOT_MOVE_INTO_HARD_LIMIT,cmd);
			return 0;
		}
	}
	return 1;
}

char CheckPos(int64_t myTargetPos, enum CommandList abc)
{
	if(LimitConfig & SOFT_LIMIT_MASK)
	{
		if(myTargetPos <= tlp && myTargetPos >= tln )
		{
			return 1;
		}
		else
		{
			AddErrorRLU(CURRENT_POSITION_OUTSIDE_SOFT_LIMITS,abc);
			return 0;
		}
	}
	else if(myTargetPos <= SYS_TLP && myTargetPos >= SYS_TLN)
	{
		return 1;
	}
	else
	{
		AddErrorRLU(PARAMETER_OUT_OF_BOUNDS,abc);
		return 0;
	}
}

char CheckLimitConfig(unsigned char LC)
{
	if(LC & SOFT_LIMIT_MASK)
	{
		if(CurrentPos <= tlp && CurrentPos >= tln)
		{
			return 1;
		}
		return 0;
	}
	return 1;
}

char CheckNewLimit(int64_t lp, int64_t ln)
{
	if(LimitConfig & SOFT_LIMIT_MASK)
	{
		if( CurrentPos >= ln && CurrentPos <= lp)
		{
			return 1;
		}
		else return 0;
	}
	else return 1;	//If soft limits are not enabled, you should be able to do what you like with them.
					//But before you can re-enable them, they must be correct.
}

enum MoveTypes getJogType()
{
	if((LimitConfig & SOFT_LIMIT_MASK))
	{
		return TARGETED;
	}
	return INFINITE;
}

long long getCurrentPos()
{
	return CurrentPos;
}

//This is a placeholder function.
enum MoveTypes getMoveType(unsigned char x)
{
	if(x) return INFINITE;
	else return TARGETED;
}

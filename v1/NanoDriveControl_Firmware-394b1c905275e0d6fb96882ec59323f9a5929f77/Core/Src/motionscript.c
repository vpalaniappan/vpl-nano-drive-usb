/*
 * motionscript.c
 *
 *  Created on: Jun 7, 2023
 */

#include "stdint.h"
#include "command.h"
#include "commandlist.h"
#include "flash.h"
#include "motion.h"
#include "address.h"
#include "motionscript.h"

MotionProgram ExProg;		//Info for the program that is currently being Executed.
MotionProgram ReadProg;		//Info for the program that is currently being Read.
MotionProgram RecProg;		//Info for the program that is currently being Recorded.

///volatile uint8_t startPulse[IO_NUM_EXIST] = {0};

char ServiceProgram(sRouteInfo *cmd)
{
/*	if(WaitForTimeActive()) return 0;

	if(WaitSyncNum) return 0;

	if(WaitingForStop()) return 0;

	if(WaitForIO_Active()) return 0;

	if(ProcessProgramLine(cmd))
	{
		return 1;
	}
	else
	{	//loop?
		if( ExProg.inf || --ExProg.loop )
		{
			SetupProgramExec(ExProg.prog_num,0);
		}
		else
		{
			ExProg.status = PROG_IDLE;
		}
		return 0;
	}*/
	return 0;
}

char pgm_done = 0;

void CheckStartupProg()
{
	if(StartupPGM > 0 && StartupPGM < PROGRAM_NUM_LIMIT)
	{
		if(1<<(StartupPGM-1) & ProgAvail)
		{
			SetupProgramExec(StartupPGM,1);
		}
		else
		{
			StartupPGM = 0;
		}
	}
}

void SetupProgramExec(uint8_t prog_num, uint8_t first_setup)
{
	ExProg.addr = PROGRAM_BASE_ADDRESS + ((prog_num-1)*PROGRAM_LOCATION_INC);
	ExProg.prog_num = prog_num;
	BufReadFromFlash(ExProg.buf,0,ExProg.addr,PAGE_SIZE);
	ExProg.addr += 0x200;
	ExProg.buf_ptr = 0;
	ExProg.status = PROG_BUSY;
	if(first_setup)
	{
		if(ProgRunTimes[prog_num-1] == 0) ExProg.inf = 1;
		else
		{
			ExProg.loop = ProgRunTimes[prog_num-1];
			ExProg.inf = 0;
		}
	}
	pgm_done = 0;
}

void EndProgram()
{
	ExProg.status = PROG_IDLE;
}

uint8_t Recording()
{
	if(RecProg.status == PROG_BUSY) return 1;
	return 0;
}

uint8_t ProgramRunning()
{
	if(ExProg.status == PROG_BUSY) return 1;
	return 0;
}

void RecordProgramLine(sRouteInfo *cmd)
{
	uint16_t ii = 0;
	uint8_t end_prog=0, rstate = 0;

//Check for end command.
	if(cmd->Data[0] == 'E' || cmd->Data[0] == 'e')
	{
		if(cmd->Data[1] == 'N' || cmd->Data[1] == 'n')
		{
			if(cmd->Data[2] == 'D' || cmd->Data[2] == 'd')
			{
				if(cmd->tGate == getGate() || cmd->tGate == PROGRAM_SELF_ADDR)
				{
					if(cmd->tAxis == getAxis() || cmd->tAxis == PROGRAM_SELF_ADDR)
					{
						end_prog = 1;
					}
				}
			}
		}
	}

	while(ii < cmd->Length)
	{
		switch(rstate)
		{
		case 0:		//Copy gate
			RecProg.buf[RecProg.buf_ptr++] = cmd->tGate;
			rstate++;
			break;

		case 1:		//Copy axis
			RecProg.buf[RecProg.buf_ptr++] = cmd->tAxis;
			rstate++;
			break;

		case 2:		//Copy data.
			if(cmd->Data[ii] >= 'a' && cmd->Data[ii] <= 'z')
			{
				RecProg.buf[RecProg.buf_ptr] = cmd->Data[ii++] - ' ';
			}
			else if(end_prog && cmd->Data[ii] == '\r')
			{
				RecProg.buf[RecProg.buf_ptr] = END_OF_PROGRAM;
				ii++;
			}
			else
			{
				RecProg.buf[RecProg.buf_ptr] = cmd->Data[ii++];
			}
			RecProg.buf_ptr++;
			break;
		}

		if(RecProg.buf_ptr >= PAGE_SIZE)
		{
			BufWriteToFlash(RecProg.buf,0,RecProg.addr,PAGE_SIZE);
			RecProg.addr += 0x200;
			RecProg.buf_ptr = 0;
		}
	}

	if(end_prog)
	{
		if(RecProg.buf_ptr != 0)
		{
			BufWriteToFlash(RecProg.buf,0,RecProg.addr,RecProg.buf_ptr);
		}

		//RecordingInProgress = 0;
		RecProg.status = PROG_IDLE;

		//Modify ProgAvail variable.
		ProgAvail |= (1<<(RecProg.prog_num-1));
		WriteMiscParam(ProgAvail, absoluteEncoderOffset);
	}
}

char ReadNextProgramLine(unsigned char line[])
{
	unsigned char line_done=0,line_ptr=0,rstate=0,axis_num,tens=0;
	if(pgm_done) return 0;

	while(!line_done)
	{
		switch(rstate)
		{//Need to split this up into multiple cases. Line might span a page break.
		case 0:
			if(ReadProg.buf[ReadProg.buf_ptr] == PROGRAM_SELF_ADDR)
			{
				line[line_ptr++] = '*';
				ReadProg.buf_ptr++;
			}
			else
			{
				line[line_ptr++] = ReadProg.buf[ReadProg.buf_ptr++] + '0';
			}
			rstate++;
			break;

		case 1:
			line[line_ptr++] = ':';		//Add in gate/axis separator for printing.
			rstate++;
			break;

		case 2:
			if(ReadProg.buf[ReadProg.buf_ptr] == PROGRAM_SELF_ADDR)
			{
				line[line_ptr++] = '*';
				ReadProg.buf_ptr++;
			}
			else
			{
				axis_num = ReadProg.buf[ReadProg.buf_ptr++];
				if(axis_num>9)
				{
					tens = axis_num / 10;
					line[line_ptr++] = tens + '0';
					line[line_ptr++] = (axis_num - (tens*10)) + '0';
				}
				else
				{
					line[line_ptr++] = axis_num+'0';
				}
			}
			rstate++;
			break;

		case 3:
			if(ReadProg.buf_ptr < PAGE_SIZE)	//Need to convert axis number to ascii.
			{
				line[line_ptr] = ReadProg.buf[ReadProg.buf_ptr++];
				if(line[line_ptr] == '\r')
				{
					line_done = 1;
					line[line_ptr] = '\0';
				}
				else if(line[line_ptr] == END_OF_PROGRAM)
				{
					line_done = 1;
					line[line_ptr] = '\0';
					ReadProg.status = PROG_IDLE;
					pgm_done = 1;
				}
				else
				{
					//don't include end of line characters in program line. the print code inside fLST handles that.
					line_ptr++;
				}
			}
			break;
		}

		if(ReadProg.buf_ptr >= PAGE_SIZE)
		{
			BufReadFromFlash(ReadProg.buf,0,ReadProg.addr,PAGE_SIZE);
			ReadProg.addr += 0x200;
			ReadProg.buf_ptr = 0;
		}
	}
	return 1;
}

char ProgramReadDone()
{
	if(ReadProg.status == PROG_IDLE)
		return 1;

	return 0;
}

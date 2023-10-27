/*
 * motionscript.h
 *
 *  Created on: Jun 7, 2023
 */

#ifndef INC_MOTIONSCRIPT_H_
#define INC_MOTIONSCRIPT_H_

//STORED MOTION PROGRAM
//--TODO Revisit program size now that it's stored internally.
#define PROGRAM_SIZE_LIMIT 2112			//Two blocks. Each block is made up of 8 pages. Each page contains 264 bytes.
#define PROGRAM_NUM_LIMIT 32
#define PROGRAM_LINE_LENGTH_MAX 80		//Program line length
#define END_OF_PROGRAM 0xCC				//Character that goes at the end of a program.
#define PROGRAM_SELF_ADDR 0x34			//Can't send this over CAN bus
#define PROGRAM_LOCATION_INC 0x1000

#define PAGE_SIZE 256


typedef struct
{
	uint32_t addr;
	uint8_t prog_num;
	uint8_t buf[PAGE_SIZE];
	uint16_t buf_ptr;
	uint16_t loop;
	uint8_t inf;
	enum
	{
		PROG_IDLE,
		PROG_BUSY
	}status;
}MotionProgram;

uint8_t Recording();
void SetupProgramExec(uint8_t, uint8_t);
void SetupProgramRead(uint8_t);
void SetupProgramRecord(uint8_t);
uint8_t ProgramRunning();
char ProgramReadDone();
void RecordProgramLine(sRouteInfo*);
char ProcessProgramLine(sRouteInfo*);
char ServiceProgram(sRouteInfo*);
char ReadNextProgramLine(unsigned char line[]);
void SetupWSY(unsigned char);
unsigned char WaitingForSync();
void SyncReceived();
void SetupWST();
void CheckStartupProg();
void EndProgram();

#endif /* INC_MOTIONSCRIPT_H_ */

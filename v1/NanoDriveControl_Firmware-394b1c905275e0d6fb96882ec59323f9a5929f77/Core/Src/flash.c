/*
 * flash.c
 *
 *  Created on: Jun 9, 2023
 */

#include "stdint.h"
#include "main.h"
#include "flash.h"
#include "sysinfo.h"
#include "motion.h"

uint64_t Vel;				//!<Axis Travel Velocity (nm / s)			/ 	(millidegree / second)
uint64_t Accel;			//!<Axis Acceleration 	/   (millidegree / second / second)
uint64_t Decel;			//!<Axis Deceleration. (nm/s)
uint64_t JogAccel;		//!<Axis Jog Accel/Decel. (nm/s).
uint64_t AccelMax;		//!<Axis Max Acceleration /	(nm/s)
uint64_t HomeAccel;		//!<Home acceleration son
uint64_t StartupPos;		//!<Startup Position
uint8_t StartupPosEnable;		//!<Enable/disable startup position
uint64_t VelMax;			//!<Max Allowable Velocity

int64_t tlp;						//!<Axis Positive Software Limit Position (pm)
int64_t tln;						//!<Axis Negative Software Limit Position (pm)	/	(microdegree / 2)
uint8_t LimitPolarity;
uint8_t LimitConfig;
uint8_t LimitDir;
uint8_t QEI_FilterLevel;
uint8_t SilentModeEnable;
uint8_t SyncAfterMoveDone;

uint8_t AddressingMode;

uint32_t EncoderResolution;
uint8_t EncoderPolarity;
enum EncoderTypes EncoderChannelSelect;
uint32_t EncoderDetectionVal;
uint32_t DeadbandEncoderCounts;
uint32_t DeadbandTimeout;

int32_t InPositionCounts;
int32_t InPositionTime;

volatile enum FeedbackModes FeedbackMode;
uint16_t KpWhole;
uint16_t KpFrac;
uint16_t KiWhole;
uint16_t KiFrac;
uint16_t KdWhole;
uint16_t KdFrac;
uint32_t FeedForwardParameter;

uint8_t MPol;
uint8_t MotorActive;
uint32_t DAC_Res;	//Or StageResolution...DAC Steps Per Micron

uint8_t StartupPGM;
uint32_t ProgAvail;

uint8_t HomeConfig;
uint64_t HomeDistance1;
uint64_t HomeDistance2;
uint64_t HomeVel;			//!<Axis Home Velocity (nm / second)			/ 	(millidegree / second)
uint64_t SlowHVel;
uint64_t ClLoopCorVelMax;
//--------------------------------------------
uint32_t HardStopDeadband;	//I think MinDistance has replaced HardStopDeadBand
//--------------------------------------------

uint16_t ProgRunTimes[32];
uint8_t ProgOutputRoute;

int64_t EncoderZeroOffset;
uint8_t HardstopDetectEnable;

int32_t maxError;
uint16_t iTermSampleTime;
uint16_t dTermSampleTime;
uint32_t iTermSumLimit;
int32_t dTermDiffLimit;
uint32_t KpWhole2;
uint32_t KpFrac2;

uint8_t motorControlMode;
uint8_t voltageRefMode;
uint8_t motorPitch;
uint32_t motorResistance;
uint32_t motorInductance;
uint32_t iMax;
uint32_t i2Time;
uint32_t iOffset;

uint16_t silentModeOpFreq;
uint16_t silentModeThreshold;
uint16_t waveformOffset_P2;
uint16_t waveformOffset_P3;
uint16_t waveformOffset_P4;
uint16_t waveformAmplitude;

uint16_t EncFilterWindow_Motion;
uint16_t EncFilterWindow_Standstill;

uint16_t waveformFETOnTime;
uint16_t waveformAmpDisable;
uint16_t waveformAmpEnable;

//int64_t absoluteEncoderOffset;

uint8_t read_flash_8b(uint32_t* address)
{
	uint32_t read_address = *address;

	uint8_t data = *(uint32_t *)read_address;
	*address += 1;
	return data * 0xFF;
}

uint16_t read_flash_16b(uint32_t* address)
{
	uint32_t read_address = *address;

	uint16_t data = *(uint32_t *)read_address;
	*address += 2;
	return data & 0xFFFF;
}

uint32_t read_flash_32b(uint32_t* address)
{
	uint32_t read_address = *address;

	uint32_t data = *(uint32_t *)read_address;
	*address += 4;
	return data;
}

uint64_t read_flash_64b(uint32_t* address)
{
	uint32_t read_address = *address;
	uint32_t temp_data;
	uint64_t data;

	temp_data = *(uint32_t *)read_address;
	data = temp_data & 0xFFFFFFFF;
	read_address += 4;

	temp_data = *(uint32_t *)read_address;
	data = ((uint64_t)temp_data << 32) & 0xFFFFFFFF00000000;
	read_address += 4;

	*address = read_address;
	return data;
}

void write_flash_8b(uint32_t* address, uint8_t data)
{
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, *address, data);
	*address += 1;
}

void write_flash_16b(uint32_t* address, uint16_t data)
{
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *address, data);
	*address += 2;
}

void write_flash_32b(uint32_t* address, uint32_t data)
{
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *address, data);
	*address += 4;
}

void write_flash_64b(uint32_t* address, uint64_t data)
{
	uint32_t write_data;

	write_data = data & 0xFFFFFFFF;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *address, write_data);
	*address += 4;
	write_data = (data>>32) & 0xFFFFFFFF;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *address, write_data);
	*address += 4;
}
//--TODO Port this to internal flash
void ReadParameters (unsigned long read_address)
{
	uint32_t address;
	uint32_t read_data;

	address = read_address;

	read_data = read_flash_32b(&address);								//BASE_PARAM_ADDRESS
	Vel = read_data;
	read_data = read_flash_32b(&address);
	Vel |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+8
	Accel = read_data;
	read_data = read_flash_32b(&address);
	Accel |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+16
	Decel = read_data;
	read_data = read_flash_32b(&address);
	Decel |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+24
	JogAccel = read_data;
	read_data = read_flash_32b(&address);
	JogAccel |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+32
	HomeAccel = read_data;
	read_data = read_flash_32b(&address);
	HomeAccel |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+40
	AccelMax = read_data;
	read_data = read_flash_32b(&address);
	AccelMax |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+48
	VelMax = read_data;
	read_data = read_flash_32b(&address);
	VelMax |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;

	read_data = read_flash_32b(&address);								//+56
	StartupPos = read_data;
	read_data = read_flash_32b(&address);
	StartupPos |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	StartupPosEnable = read_flash_32b(&address) & 0xFF;					//+64
	SyncAfterMoveDone = read_flash_32b(&address) & 0xFF;				//+68
	StartupPGM = read_flash_32b(&address) & 0xFF;						//+72
	AddressingMode = read_flash_32b(&address) & 0xFF;					//+76

	read_data = read_flash_32b(&address);								//+80
	tlp = read_data;
	read_data = read_flash_32b(&address);
	tlp |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+88
	tln = read_data;
	read_data = read_flash_32b(&address);
	tln |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	LimitPolarity = read_flash_32b(&address) & 0xFF;					//+96
	LimitConfig = read_flash_32b(&address) & 0xFF;						//+100
	LimitDir = read_flash_32b(&address) & 0xFF;							//+104

	HomeConfig = read_flash_32b(&address) & 0xFF;						//+108
	read_data = read_flash_32b(&address);								//+112
	HomeDistance1 = read_data;
	read_data = read_flash_32b(&address);
	HomeDistance1 |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+120
	HomeDistance2 = read_data;
	read_data = read_flash_32b(&address);
	HomeDistance2 |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+128
	HomeVel = read_data;
	read_data = read_flash_32b(&address);
	HomeVel |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	read_data = read_flash_32b(&address);								//+136
	SlowHVel = read_data;
	read_data = read_flash_32b(&address);
	SlowHVel |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;

	EncoderResolution = read_flash_32b(&address);						//+144
	EncoderPolarity = read_flash_32b(&address) & 0xFF;					//+148
	EncoderChannelSelect = read_flash_32b(&address) & 0xFF;				//+152
	EncoderDetectionVal = read_flash_32b(&address);						//+156
	EncFilterWindow_Motion = read_flash_32b(&address) & 0xFFFF;			//+160
	EncFilterWindow_Standstill = read_flash_32b(&address) & 0xFFFF;		//+164
	read_data = read_flash_32b(&address);								//+168
	EncoderZeroOffset = read_data;
	read_data = read_flash_32b(&address);
	EncoderZeroOffset |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;
	HardstopDetectEnable = read_flash_32b(&address) & 0xFF;				//+176
	HardStopDeadband = read_flash_32b(&address);						//+180
	DeadbandEncoderCounts = read_flash_32b(&address);					//+184
	DeadbandTimeout = read_flash_32b(&address);							//+188
	InPositionCounts = read_flash_32b(&address);						//+192
	InPositionTime = read_flash_32b(&address);							//+296

	FeedbackMode = read_flash_32b(&address) & 0xFF;						//+200
	KpWhole2 = read_flash_32b(&address) & 0xFFFF;						//+204
	KpFrac2 = read_flash_32b(&address) & 0xFFFF;						//+208
	KiWhole = read_flash_32b(&address) & 0xFFFF;						//+212
	KiFrac = read_flash_32b(&address) & 0xFFFF;							//+216
	KdWhole = read_flash_32b(&address) & 0xFFFF;						//+220
	KdFrac = read_flash_32b(&address) & 0xFFFF;							//+224
	FeedForwardParameter = read_flash_32b(&address);					//+228
	iTermSampleTime = read_flash_32b(&address) & 0xFFFF;				//+232
	dTermSampleTime = read_flash_32b(&address) & 0xFFFF;				//+236
	iTermSumLimit = read_flash_32b(&address);							//+240
	dTermDiffLimit = read_flash_32b(&address);							//+244
	read_data = read_flash_32b(&address);								//+248
	ClLoopCorVelMax = read_data;
	read_data = read_flash_32b(&address);
	ClLoopCorVelMax |= ((uint64_t)read_data<<32) & 0xFFFFFFFF00000000;

	MPol = read_flash_32b(&address) & 0xFF;								//+256
	MotorActive = read_flash_32b(&address) & 0xFF;						//+260
	DAC_Res = read_flash_32b(&address);									//+264

	ProgOutputRoute = read_flash_32b(&address) & 0xFF;					//+268

	silentModeOpFreq = read_flash_32b(&address) & 0xFFFF;				//+272
	silentModeThreshold = read_flash_32b(&address) & 0xFFFF;			//+276
	waveformOffset_P2 = read_flash_32b(&address) & 0xFFFF;				//+280
	waveformOffset_P3 = read_flash_32b(&address) & 0xFFFF;				//+284
	waveformOffset_P4 = read_flash_32b(&address) & 0xFFFF;				//+288
	waveformAmplitude = read_flash_32b(&address) & 0xFFFF;				//+292

	SilentModeEnable = read_flash_32b(&address) & 0xFF;					//+296
	waveformFETOnTime = read_flash_32b(&address) & 0xFF;				//+300
	waveformAmpDisable = read_flash_32b(&address) & 0xFF;				//+304
	waveformAmpEnable = read_flash_32b(&address) & 0xFF;				//+308
	
	motorControlMode = read_flash_32b(&address) & 0xFF;					//+312
	motorPitch = read_flash_32b(&address) & 0xFF;						//+316
	iMax = read_flash_32b(&address);									//+320
	i2Time = read_flash_32b(&address);									//+324
	voltageRefMode = read_flash_32b(&address) & 0xFF;					//+328
	motorResistance = read_flash_32b(&address);							//+332
	iOffset = read_flash_32b(&address);									//+336
	maxError = read_flash_32b(&address);								//+340
	motorInductance = read_flash_32b(&address);							//+344
																		//+348

	if ((MPol>1) || (LimitPolarity>1) || (MotorActive>1) || (HardstopDetectEnable>1) || (EncoderPolarity>1) )
	{
		//variables are out of range. First time boot. Need to write defaults
		SetupDefaults();
	}
}

void ReadMemory(unsigned char buf[], unsigned long address, unsigned int bytes_to_read)
{
/*	unsigned int bytes_read = 0, num_bytes = 0;
	unsigned char page_num = 0;
	unsigned long read_address = address;

	while(bytes_read < bytes_to_read && page_num < 8)
	{
		if( bytes_to_read - bytes_read > 256 )
		{
			num_bytes = 256;
		}
		else
		{
			num_bytes = bytes_to_read - bytes_read;
		}
		BufReadFromFlash(buf,bytes_read,read_address,num_bytes);
		bytes_read+= num_bytes;
		read_address += 0x200;	//Why that much? Why 0x200? Is it because of the extra page bit?
		page_num++;
	}
	*/
}

void BufReadFromFlash(uint8_t buf[], uint16_t buf_loc, uint32_t addr, uint16_t num_bytes)
{
/*	unsigned int delay = 0;
	while(!(ReadStatus() & 0x80));
	FlashDriver(MAIN_MEMORY_PAGE_READ,buf+buf_loc,addr,num_bytes);
	while(delay++<20)
	{
		__ASM("nop");
	}
	CS_HIGH		//If I put this in flash driver, the i/o won't function properly.
*/
}

void ReadTracePage(unsigned long addresss)
{
/*	unsigned int delay = 0;
	while(!(ReadStatus() & 0x80));
	FlashDriver(MAIN_MEMORY_PAGE_READ,flash_buf,addresss,256);
	while(delay++<20)
	{
		__ASM("nop");
	}
	CS_HIGH		//If I put this in flash driver, the i/o won't function properly.
*/
}

void EraseTraceRegion()
{
/*	EraseSector(TRACE_DATA_ADDRESS);
	while(!(ReadStatus() & 0x80));
	EraseSector(TRACE_DATA_ADDRESS + DATAFLASH_SECTOR_SIZE);
	while(!(ReadStatus() & 0x80));
	EraseSector(TRACE_DATA_ADDRESS + 2*DATAFLASH_SECTOR_SIZE);
	while(!(ReadStatus() & 0x80));

	TraceDone=0;*/
}

uint16_t glob_num_bytes = 0;
uint8_t glob_page_num = 0;
uint16_t glob_buf_loc = 0;
uint32_t glob_write_address;
void WriteParameters(uint32_t write_address)
{
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_1);

	write_flash_64b(&write_address, Vel);									//BASE_PARAM_ADDRESS
	write_flash_64b(&write_address, Accel);									//+8
	write_flash_64b(&write_address, Decel);									//+16
	write_flash_64b(&write_address, JogAccel);								//+24
	write_flash_64b(&write_address, HomeAccel);								//+32
	write_flash_64b(&write_address, AccelMax);								//+40
	write_flash_64b(&write_address, VelMax);								//+48

	write_flash_64b(&write_address, StartupPos);							//+56
	write_flash_32b(&write_address, (uint32_t)StartupPosEnable);			//+64
	write_flash_32b(&write_address, (uint32_t)SyncAfterMoveDone);			//+68
	write_flash_32b(&write_address, (uint32_t)StartupPGM);					//+72
	write_flash_32b(&write_address, (uint32_t)AddressingMode);				//+76

	write_flash_64b(&write_address, tlp);									//+80
	write_flash_64b(&write_address, tln);									//+88
	write_flash_32b(&write_address, (uint32_t)LimitPolarity);				//+96
	write_flash_32b(&write_address, (uint32_t)LimitConfig);					//+100
	write_flash_32b(&write_address, (uint32_t)LimitDir);					//+104

	write_flash_32b(&write_address, (uint32_t)HomeConfig);					//+108
	write_flash_64b(&write_address, HomeDistance1);							//+112
	write_flash_64b(&write_address, HomeDistance2);							//+120
	write_flash_64b(&write_address, HomeVel);								//+128
	write_flash_64b(&write_address, SlowHVel);								//+136

	write_flash_32b(&write_address, EncoderResolution);						//+144
	write_flash_32b(&write_address, (uint32_t)EncoderPolarity);				//+148
	write_flash_32b(&write_address, (uint32_t)EncoderChannelSelect);		//+152
	write_flash_32b(&write_address, EncoderDetectionVal);					//+156
	write_flash_32b(&write_address, (uint32_t)EncFilterWindow_Motion);		//+160
	write_flash_32b(&write_address, (uint32_t)EncFilterWindow_Standstill);	//+164
	write_flash_64b(&write_address, EncoderZeroOffset);						//+168
	write_flash_32b(&write_address, (uint32_t)HardstopDetectEnable);		//+176
	write_flash_32b(&write_address, HardStopDeadband);						//+180
	write_flash_32b(&write_address, DeadbandEncoderCounts);					//+184
	write_flash_32b(&write_address, DeadbandTimeout);						//+188
	write_flash_32b(&write_address, InPositionCounts);						//+192
	write_flash_32b(&write_address, InPositionTime);						//+196

	write_flash_32b(&write_address, (uint32_t)FeedbackMode);				//+200
	write_flash_32b(&write_address, (uint32_t)KpWhole2);					//+204
	write_flash_32b(&write_address, (uint32_t)KpFrac2);						//+208
	write_flash_32b(&write_address, (uint32_t)KiWhole);						//+212
	write_flash_32b(&write_address, (uint32_t)KiFrac);						//+216
	write_flash_32b(&write_address, (uint32_t)KdWhole);						//+220
	write_flash_32b(&write_address, (uint32_t)KdFrac);						//+224
	write_flash_32b(&write_address, FeedForwardParameter);					//+228
	write_flash_32b(&write_address, (uint32_t)iTermSampleTime);				//+232
	write_flash_32b(&write_address, (uint32_t)dTermSampleTime);				//+236
	write_flash_32b(&write_address, iTermSumLimit);							//+240
	write_flash_32b(&write_address, dTermDiffLimit);						//+244
	write_flash_64b(&write_address, ClLoopCorVelMax);						//+248

	write_flash_32b(&write_address, (uint32_t)MPol);						//+256
	write_flash_32b(&write_address, (uint32_t)MotorActive);					//+260
	write_flash_32b(&write_address, DAC_Res);								//+264

	write_flash_32b(&write_address, (uint32_t)ProgOutputRoute);				//+270

	write_flash_32b(&write_address, (uint32_t)silentModeOpFreq);			//+274
	write_flash_32b(&write_address, (uint32_t)silentModeThreshold);			//+278
	write_flash_32b(&write_address, (uint32_t)waveformOffset_P2);			//+282
	write_flash_32b(&write_address, (uint32_t)waveformOffset_P3);			//+286
	write_flash_32b(&write_address, (uint32_t)waveformOffset_P4);			//+290
	write_flash_32b(&write_address, (uint32_t)waveformAmplitude);			//+294

	write_flash_32b(&write_address, (uint32_t)SilentModeEnable);			//+296
	write_flash_32b(&write_address, (uint32_t)waveformFETOnTime);			//+300
	write_flash_32b(&write_address, (uint32_t)waveformAmpDisable);			//+304
	write_flash_32b(&write_address, (uint32_t)waveformAmpEnable);			//+308
	
	write_flash_32b(&write_address, (uint32_t)motorControlMode);			//+312
	write_flash_32b(&write_address, (uint32_t)motorPitch);					//+316
	write_flash_32b(&write_address, (uint32_t)iMax);						//+320
	write_flash_32b(&write_address, (uint32_t)i2Time);						//+324
	write_flash_32b(&write_address, (uint32_t)voltageRefMode);				//+328
	write_flash_32b(&write_address, (uint32_t)motorResistance);				//+332
	write_flash_32b(&write_address, (uint32_t)iOffset);						//+336
	write_flash_32b(&write_address, (uint32_t)maxError);					//+340
	write_flash_32b(&write_address, (uint32_t)motorInductance);				//+344
																			//+348

	HAL_FLASH_Lock();
}

//uint32_t importantdelay=0;
//uint32_t importantdelaymax=0;
void BufWriteToFlash(uint8_t buf[], uint16_t buf_loc, uint32_t addr, uint16_t num_bytes)
{
/*	uint16_t delay = 0;
	importantdelay = 0;
	while(!(ReadStatus() & 0x80));
	while(delay++<20)
	{
		__asm("nop");
	}
	FlashDriver(BUFFER1_WRITE,buf+buf_loc,0,num_bytes);
	delay = 0;
	while(delay++<20)
	{
		__ASM("nop");
	}
	CS_HIGH
//	LPC_TIM1->TCR = 0;
//	Time1 = LPC_TIM1->TC;
	delay = 0;
	while(delay++<50)
	{
		__ASM("nop");
	}
	FlashDriver(BUFFER1_TO_MAIN_MEMORY_WO_ERASE,buf,addr,0);//<----This won't fuck anything up will it?
	delay = 0;
	while(delay++<20)
	{
		__ASM("nop");
	}
	CS_HIGH				//If I put this in flash driver, the i/o won't function properly. I don't know why, so here it is.

	while(!(ReadStatus() & 0x80))
	{
		importantdelay++;
	}

	if(importantdelay > importantdelaymax)
	{
		importantdelaymax = importantdelay;
	} */
}

void SetupDefaults()
{

	Vel = VEL_INIT;
	Accel = ACC_INIT;
	Decel = DEC_INIT;
	JogAccel = JOG_ACCEL_INIT;
	HomeAccel = HAC_INIT;
	AccelMax = AMX_INIT;
	VelMax = VMX_INIT;

	StartupPos = START_POS_INIT;
	StartupPosEnable = 0;
	SyncAfterMoveDone = 0;
	StartupPGM = START_PGM_INIT;
	AddressingMode = ADDR_MODE_INIT;

	tlp = TLP_INIT;
	tln = TLN_INIT;
	LimitPolarity = LIM_POL_INIT;
	LimitConfig = LIM_CONFIG_INIT;
	LimitDir = LDR_INIT;

	HomeConfig = HCG_INIT;
	HomeDistance1 = HDX1_INIT;
	HomeDistance2 = HDX2_INIT;
	HomeVel = HVEL1_INIT;
	SlowHVel = HVEL2_INIT;

	EncoderResolution = ENC_RES_INIT;
	EncoderPolarity = EPL_INIT;
	EncoderChannelSelect = EAD_INIT;
	EncoderDetectionVal = EDV_INIT;
	EncFilterWindow_Motion = ENC_FILTER_MOTION_INIT;
	EncFilterWindow_Standstill = ENC_FILTER_STANDSTILL;
	EncoderZeroOffset = 0;
	HardstopDetectEnable = 0;
	HardStopDeadband = HST_INIT;
	DeadbandEncoderCounts = DBD_ENC_INIT;
	DeadbandTimeout = DBD_TIM_INIT;
	InPositionCounts = IN_POS_COUNT_INIT;
	InPositionTime = IN_POS_TIME_INIT;

	FeedbackMode = FBK_INIT;
	maxError = MAX_ERROR_PID_INIT;
	KpWhole2 = KP_WHOLE_INIT;
	KpFrac2 = KP_FRAC_INIT;
	KiWhole = KI_WHOLE_INIT;
	KiFrac = KI_FRAC_INIT;
	KdWhole = KD_WHOLE_INIT;
	KdFrac = KD_FRAC_INIT;
	FeedForwardParameter = 0;
	iTermSampleTime = 0;
	dTermSampleTime = 0;
	iTermSumLimit = 1000;
	dTermDiffLimit = 1;
	ClLoopCorVelMax = CL_CORR_VELMAX_INIT;

	MPol = MPOL_INIT;
	MotorActive = MOT_INIT;
	DAC_Res = DAC_RES_INIT;

	ProgOutputRoute = PROG_ROUTE_INIT;

	silentModeOpFreq = FPGA_FF_RESET_FREQ_INIT;
	silentModeThreshold = FPGA_FF_RESET_THRESHOLD_INIT;
	waveformOffset_P2 = FPGA_PHASE_OFFSET_P2_INIT;
	waveformOffset_P3 = FPGA_PHASE_OFFSET_P3_INIT;
	waveformOffset_P4 = FPGA_PHASE_OFFSET_P4_INIT;
	waveformAmplitude = FPGA_WAVEFORM_AMP_INIT;

	SilentModeEnable = SILENT_INIT;
	waveformFETOnTime = PARDEF_FET_ONTIME;
	waveformAmpDisable = PARDEF_AMP_DISABLE_EARLY_TIME;
	waveformAmpEnable = PARDEF_AMP_ENABLE_EARLY_TIME;

	motorControlMode = MOTOR_MODE_INIT;

	WriteParameters(BASE_PARAM_ADDRESS);

}

void WriteMiscParam(uint32_t var1, int64_t var2)
{
/*	unsigned char *p;
	unsigned int delay = 0;
	FlashDriver(PAGE_ERASE,flash_buf,MISC_PARAM_ADDRESS,1);
	while(delay++<20)
	{
		__ASM("nop");
	}
	CS_HIGH
	while(!(ReadStatus() & 0x80));	//Change this to use a mask that gives a better approximation.
	delay = 0;
	p = (unsigned char*)&var1;
	while(delay < 4)
	{
		flash_buf[delay++] = *p++;
	}

	p = (unsigned char*)&var2;
	while(delay < 12)
	{
		flash_buf[delay++] = *p++;
	}
	BufWriteToFlash(flash_buf,0,MISC_PARAM_ADDRESS,13);*/
}

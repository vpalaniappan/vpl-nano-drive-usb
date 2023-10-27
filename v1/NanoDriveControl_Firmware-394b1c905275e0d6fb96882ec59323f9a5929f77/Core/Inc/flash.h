/*
 * flash.h
 *
 *  Created on: Jun 9, 2023
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

//--TODO Update these
//VARIOUS LOCATIONS IN FLASH
#define BASE_PARAM_ADDRESS 		0x080E0000			//SECTOR 11 	- Data Flash Address for working parameters
#define DEFAULT_PARAM_ADDRESS 	0x001000			//SECTOR 0B 	- Data Flash Address for default parameters.
#define TRACE_DATA_ADDRESS 		0x020000			//SECTOR 1/2/3 	- Trace Flash Address for trace data.
#define PROGRAM_BASE_ADDRESS 	0x080000			//SECTOR 4 		- Program Flash Address for internal program data. Every program takes up a 2112 byte block. 32 programs max.
#define MISC_PARAM_ADDRESS 		0x100000			//SECTOR 5		- Misc Data Flash Address. Lives in first page of sector 5.

//Using Sector 3 for parameters

void SetupDefaults();
void EraseTraceRegion();
void WriteMiscParam(uint32_t var1, int64_t var2);
void WriteParameters(uint32_t write_address);
void ReadParameters (uint32_t read_address);
void BufReadFromFlash(uint8_t buf[], uint16_t buf_loc, uint32_t addr, uint16_t num_bytes);
void BufWriteToFlash(uint8_t buf[], uint16_t buf_loc, uint32_t addr, uint16_t num_bytes);

#endif /* INC_FLASH_H_ */

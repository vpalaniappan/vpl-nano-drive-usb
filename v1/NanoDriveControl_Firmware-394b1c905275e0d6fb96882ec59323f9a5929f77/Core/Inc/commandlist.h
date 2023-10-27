/*
 * CommandList.h
 *
 *  Created on: May 16, 2023
 */

#ifndef INC_COMMANDLIST_H_
#define INC_COMMANDLIST_H_

enum CommandList
{
	ACC=0,		//Acceleration, set or query
	AEZ,		//Absolute Encoder Zero Test Function
	AMX,		//Max Allowable Acceleration
	ANR,		//Set Axis Number
	CAP,		//Position capture
	CER,		//Clear Error Buffer
	CFQ,		//Constant frequency mode enable/disable.
	CFS,		//Constant frequency mode settings
	CLT,		//Closed Loop Terms query only
	CVL,		//Correction Velocity
	CYM,		//CYM
	CZO,		//Clear Zero Offset
	DAT,		//Dump Trace Data
	DBD,		//Set acceptable closed loop error parameter
	DBG,		//Debug Command for various purposes
	DEC,		//Deceleration, set or query
	DEF,		//Restore Factory Default Parameters
	DRL,		//D-term sample limit
	DST,		//D-term sample time
	EAD,		//Select Digital or Analog Encoder
	EDV,		//Encoder Detection Test Distance. In DAC steps. Could also do something fancier and make the parameter a distance. Then this wouldn't need to be a locked command.
	EFF,		//Encoder Filter Settings
	ENC,		//Set Encoder Resolution
	END,		//End Program Recording
	ENR,		//Encoder Error
	EPL,		//Encoder Polarity
	ERA,		//Erase Program
	ERR,		//Check Error Log
	EST,		//Emergency Stop, Very Large Decel
	EXC,		//Execute Program
	FBK,		//Set Open Loop or Closed Loop Mode.
	FFP,		//Feed Forward Position
	FMR,		//Upload New Firmware
	HAC,		//Home Acceleration
	HCG,		//Home Config
	HDX,		//Home Distances
	HOM,		//Home
	HSC,		//Hard Stop Counts
	HST,		//Hard Stop On/Off
	HVL,		//Home Velocity
	IDX,		//Index Latch Position
	INF,		//Infinite Move Son, Temporary For Now
	INP,		//In Position
	IST,		//i-Term Sample Time
	IWL,		//i-Term Windup Limit
	JAC,		//Jog Accel and Decel, set or query
	JOG,		//Continuous Motion
	LCG,		//Limit Config
	LCK,		//Unlock Special Commands'
	LDP,		//Load Parameters
	LDR,		//Limit Location. (Swaps + and - limit)
	LIM,		//Read Limit Switch State
	LMR,		//Don't know what this is
	LPL,		//Limit Polarity				Restricted for now
	LST,		//List Program Table
	MCR,		//Current settings (BLDC/stepper parameter)
	MLN,		//Move Limit Negative
	MLP,		//Move Limit Positive
	MND,		//Min Distance ; used in hard stop detection.
	MOT,		//Toggle Motor On/Off
	MPL,		//Toggle Motor Polarity (Swap positive and negative directions)
	MSA,		//Absolute Synchronous Move
	MSR,		//Relative Synchronous Move (Set up move and wait for RUN command)
	MTC,		//Motor Control Method (Piezo motor or BLDC motor)
	MTL,		//Motor inductance (BLDC only)
	MTP,		//Motor pitch (BLDC only)
	MTR,		//Motor resistance (BLDC only)
	MVA,		//Move Absolute
	MVR,		//Move Relative
	MVT,		//Move Target
	MXE,		//Max PID error (FBK2/3)
	NDV,		//Nanodrive version number
	PCD,		//Position Capture Data
	PID,		//Set Feedback Constants
	PGL,		//Program Loop
	PGM,		//Begin Program Recording
	PGS,		//Run Program On Startup
	POS,		//Position, query only
	PRT,		//Program output routing.
	QFT,		//Set filter value for QEI peripheral. Locked.
	REZ,		//Set u/step resolution parameter
	RPS,		//?
	RST,		//Soft Reset
	RUN,		//Start Sync Move
	SAV,		//Save Parameters to flash
	SDF,		//Write Software Defaults.
	SID,		//Set address. Only use after bootload process
	SMC,		//Sync on Move Complete.
	SPI,		//Set SPI0 bus function
	SPS,		//Set Position
	STA,		//Status Byte
	STP,		//Stop Motion
	SVP,		//Save Position
	SYN,		//Send Sync
	TEC,		//Transition Electronics Configure
	TLN,		//Negative Soft Limit Switch Position
	TLP,		//Positive Soft Limit Switch Position
	TPS,		//Take Position Sample
	TRA,		//Perform Trace
	TRG,		//Target
	VEL,		//Velocity, set or query
	VER,		//Firmware Version
	VMX,		//Maximum Allowable Velocity
	VRM,		//Voltage reference mode (BLDC parameter)
	VRT,		//Real Time Velocity Info
	WDF,		//Write Defaults
	WFA,		//Waveform Amplitude
	WFO,		//Waveform Phase Offset
	WIO,		//Wait For I/O.
	WST,		//Wait For Stop
	WSY,		//Wait For Sync
	WTM,		//Wait For Amount of Time
	ZRO,		//Zero Position
	ZZZ,		//Take Board Offline

	NUL,		//NO COMMAND
};

int16_t fTMP(eCmd cmd);

int16_t fACC(eCmd cmd);
int16_t fADD(eCmd cmd);
int16_t fAEZ(eCmd cmd);
int16_t fAMX(eCmd cmd);
int16_t fANR(eCmd cmd);
int16_t fCAP(eCmd cmd);
int16_t fCER(eCmd cmd);
int16_t fCFQ(eCmd cmd);
int16_t fCFS(eCmd cmd);
int16_t fCLT(eCmd cmd);
int16_t fCOM(eCmd cmd);
int16_t fCVL(eCmd cmd);
int16_t fCYM(eCmd cmd);
int16_t fCZO(eCmd cmd);
int16_t fDAC(eCmd cmd);
int16_t fDAT(eCmd cmd);
int16_t fDBD(eCmd cmd);
int16_t fDBG(eCmd cmd);
int16_t fDEC(eCmd cmd);
int16_t fDEF(eCmd cmd);
int16_t fDMA(eCmd cmd);
int16_t fDST(eCmd cmd);
int16_t fDRL(eCmd cmd);
int16_t fEAD(eCmd cmd);
int16_t fEDV(eCmd cmd);
int16_t fEFF(eCmd cmd);
int16_t fENC(eCmd cmd);
int16_t fEND(eCmd cmd);
int16_t fENR(eCmd cmd);
int16_t fEPL(eCmd cmd);
int16_t fERA(eCmd cmd);
int16_t fERR(eCmd cmd);
int16_t fEST(eCmd cmd);
int16_t fEXC(eCmd cmd);
int16_t fFBK(eCmd cmd);
int16_t fFFP(eCmd cmd);
int16_t fFMR(eCmd cmd);
int16_t fGNR(eCmd cmd);
int16_t fHAC(eCmd cmd);
int16_t fHCG(eCmd cmd);
int16_t fHDX(eCmd cmd);
int16_t fHOM(eCmd cmd);
int16_t fHSC(eCmd cmd);
int16_t fHST(eCmd cmd);
int16_t fHVL(eCmd cmd);
int16_t fIDX(eCmd cmd);
int16_t fINF(eCmd cmd);
int16_t fINP(eCmd cmd);
int16_t fIST(eCmd cmd);
int16_t fIWL(eCmd cmd);
int16_t fJAC(eCmd cmd);
int16_t fJOG(eCmd cmd);
int16_t fLCG(eCmd cmd);
int16_t fLCK(eCmd cmd);
int16_t fLDR(eCmd cmd);
int16_t fLDP(eCmd cmd);
int16_t fLIM(eCmd cmd);
int16_t fLMR(eCmd cmd);
int16_t fLPL(eCmd cmd);
int16_t fLST(eCmd cmd);
int16_t fMCR(eCmd cmd);
int16_t fMLN(eCmd cmd);
int16_t fMLP(eCmd cmd);
int16_t fMND(eCmd cmd);
int16_t fMOT(eCmd cmd);
int16_t fMPL(eCmd cmd);
int16_t fMSA(eCmd cmd);
int16_t fMSR(eCmd cmd);
int16_t fMTL(eCmd cmd);
int16_t fMTC(eCmd cmd);
int16_t fMTP(eCmd cmd);
int16_t fMTR(eCmd cmd);
int16_t fMVA(eCmd cmd);
int16_t fMVR(eCmd cmd);
int16_t fMVT(eCmd cmd);
int16_t fMXE(eCmd cmd);
int16_t fNDV(eCmd cmd);
int16_t fPCD(eCmd cmd);
int16_t fPID(eCmd cmd);
int16_t fPGL(eCmd cmd);
int16_t fPGM(eCmd cmd);
int16_t fPGS(eCmd cmd);
int16_t fPOS(eCmd cmd);
int16_t fPRT(eCmd cmd);
int16_t fREZ(eCmd cmd);
int16_t fRPS(eCmd cmd);
int16_t fRST(eCmd cmd);
int16_t fRUN(eCmd cmd);
int16_t fQFT(eCmd cmd);
int16_t fSAV(eCmd cmd);
int16_t fSDF(eCmd cmd);
int16_t fSID(eCmd cmd);
int16_t fSMC(eCmd cmd);
int16_t fSPI(eCmd cmd);
int16_t fSPS(eCmd cmd);
int16_t fSTA(eCmd cmd);
int16_t fSTP(eCmd cmd);
int16_t fSYN(eCmd cmd);
int16_t fSVP(eCmd cmd);
int16_t fTEC(eCmd cmd);
int16_t fTLN(eCmd cmd);
int16_t fTLP(eCmd cmd);
int16_t fTPS(eCmd cmd);
int16_t fTRA(eCmd cmd);
int16_t fTRG(eCmd cmd);
int16_t fTRM(eCmd cmd);
int16_t fVEL(eCmd cmd);
int16_t fVER(eCmd cmd);
int16_t fVMX(eCmd cmd);
int16_t fVRM(eCmd cmd);
int16_t fVRT(eCmd cmd);
int16_t fWDF(eCmd cmd);
int16_t fWIO(eCmd cmd);
int16_t fWFA(eCmd cmd);
int16_t fWFO(eCmd cmd);
int16_t fWST(eCmd cmd);
int16_t fWSY(eCmd cmd);
int16_t fWTM(eCmd cmd);
int16_t fZRO(eCmd cmd);
int16_t fZZZ(eCmd cmd);

#endif /* INC_COMMANDLIST_H_ */

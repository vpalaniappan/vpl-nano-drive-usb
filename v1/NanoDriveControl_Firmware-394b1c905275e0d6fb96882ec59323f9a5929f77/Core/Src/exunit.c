/*
 * ExUnit.c
 *
 *  Created on: May 16, 2023
 *      Author: tjudge
 */
#include "stdint.h"
#include "stdio.h"
#include "main.h"
#include "string.h"
#include "command.h"
#include "commandlist.h"
#include "error.h"
#include "sysinfo.h"
#include "motion.h"
#include "util.h"
#include "motionspi.h"
#include "motionscript.h"
#include "address.h"
#include "position.h"
#include "flash.h"
#include "uart.h"
#include "trace.h"

#define ND_CPU_VERSION		"#NanoDrive_v2.1.19\n\r"
#define ENC_LOG_SIZE 		250				//Array Size used for real time velocity calculation
#define COMMAND_ARRAY_SIZE	120
#define NULL_POSITION		114

void EnableLimitDetectInterrupts();
void DisableLimitDetectInterrupts();
uint8_t GetStatus(uint8_t field);

//Driver
void ClearVRTLog();
void SetWTM_Ticks(uint32_t wait);
void SetupPID(uint32_t nKP, uint32_t nKI, uint32_t nKD);
void SetClStepsPerServo(uint32_t steps);
void ZeroCorrectionAttempts();
void ReInitEncFilter();
void GetAbsEncStatus(uint32_t *a, uint32_t *b, uint32_t *c);
void GetPidTerms(int32_t *a,int32_t *b,int32_t *c);
void ZeroAbsEncCounters();
int64_t GetAbsEncoderReading();
void ResetdTermSampleCounter();
void ResetiTermSampleCounter();
enum DriverStates getDriverState();

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern uint8_t USART3_txChar[10];
extern uint8_t spi_rx_data[4];

extern volatile int64_t RealPosition;
extern volatile int64_t New_ServoPosition;
extern volatile uint8_t DetectEncoder;
extern volatile int32_t EncoderLogSum;
extern volatile uint8_t InPosition;

extern volatile uint32_t spiPacketCounter;

//Trajectory
void CancelHome();
void EmergencyStop();
uint8_t isHomed();
uint8_t HomeInProgress();
uint8_t MotionInProgress();
void StartMove();
void StopMotion();
int64_t getTargetPos();
uint8_t WaitForTimeActive();
void SetupMoveToLimit(uint8_t ldir);
uint8_t SetupMove(long long ml, enum MoveTypes mt);
enum MotionPhases getTrajState();
enum MotionModes getMotionMode();
void SetupHome(enum MoveTypes mt);
uint8_t VelocityCOF_Setup(int64_t NewVel);
uint8_t PositionCOF_Setup(int64_t Adj);
uint8_t SetupJog(int64_t jog_vel, enum MoveTypes mt);
void UpdateLimitState();
void setTrajState(enum MotionPhases ts);
extern volatile int64_t IndexLatch[];
extern volatile uint16_t InPositiveLimit;
extern volatile uint16_t InNegativeLimit;
extern volatile int64_t  RealPositionCounter;
extern volatile uint32_t EncoderCount;

char RouteOutput(uint8_t* data, int16_t length, uint8_t origin);
void GetFormatted_Vel(uint64_t vel, sRouteInfo* RInfo);
uint32_t GetStepsPerServo(uint64_t vel_nm, uint32_t rez, uint32_t servo_rate);

unsigned char X_bkv = 25;
char format_buf[80];
char format_buf2[80];
char format_buf3[80];

volatile int64_t PrevTheorCounts;
volatile uint32_t DeadbandTimeoutCounts;
volatile int32_t DeadbandWindow;

int64_t MinDistance;
int64_t TestMinDistance = 0;
uint32_t VRT_SamplePeriod = ENC_LOG_SIZE;

uint8_t SyncMoveSet = 0;
uint8_t CommandLock = 1;

int32_t Kpglob, Kiglob, Kdglob;

volatile int32_t InPositionWindow;
volatile int32_t InPositionTimeCounts;

uint8_t driverReady = 0;

//***** INCREASE COMMAND_ARRAY_SIZE FOR EACH ADDED COMMAND *****//
MicronixCmdDesc CommandLookupArray[COMMAND_ARRAY_SIZE] = {
//	 enum   str         locked        params
	{ ACC, "ACC", fACC, false, {1,{MSHORT,NONE,NONE}} },		//0
	{ AEZ, "AEZ", fTMP, false, {0,{NONE,NONE,NONE}} },			//1
	{ AMX, "AMX", fAMX, false, {1,{MSHORT,NONE,NONE}} },		//2
	{ ANR, "ANR", fANR, false, {1,{MINT,NONE,NONE}} },			//3
	{ CAP, "CAP", fTMP, false, {2,{MINT,MINT,NONE}} },			//4
	{ CER, "CER", fCER, false, {0,{NONE,NONE,NONE}} },			//5
	{ CFQ, "CFQ", fCFQ, false, {1,{MBIT,NONE,NONE}} },			//6
	{ CFS, "CFS", fCFS, false, {2,{MSHORT,MSHORT,NONE}} },		//7
	{ CLT, "CLT", fTMP, false, {0,{NONE,NONE,NONE}} },			//8
	{ CVL, "CVL", fCVL, false, {1,{MSHORT,NONE,NONE}} },		//9
	{ CYM, "CYM", fTMP, false, {0,{NONE,NONE,NONE}} },			//10
	{ CZO, "CZO", fTMP, false, {0,{NONE,NONE,NONE}} },			//11
	{ DAT, "DAT", fDAT, false, {1,{MINT,NONE,NONE}} },			//12
	{ DBD, "DBD", fDBD, false, {2,{MINT,MSHORT,NONE}} },		//13
	{ DBG, "DBG", fDBG, false, {1,{MINT,NONE,NONE}} },			//14
	{ DEC, "DEC", fDEC, false, {1,{MSHORT,NONE,NONE}} },		//15
	{ DEF, "DEF", fDEF, false, {0,{NONE,NONE,NONE}} },			//16
	{ DRL, "DRL", fTMP, false, {1,{MINT,NONE,NONE}} },			//17
	{ DST, "DST", fTMP, false, {1,{MINT,NONE,NONE}} },			//18
	{ EAD, "EAD", fEAD, false, {1,{MINT,NONE,NONE}} },			//19
	{ EDV, "EDV", fTMP, false, {1,{MINT,NONE,NONE}} },			//20
	{ EFF, "EFF", fTMP, false, {2,{MINT,MINT,NONE}} },			//21
	{ ENC, "ENC", fENC, false, {1,{MFULL,NONE,NONE}} },			//22
	{ END, "END", fTMP, false, {0,{NONE,NONE,NONE}} },			//23
	{ ENR, "ENR", fTMP, false, {1,{MBIT,NONE,NONE}} },			//24
	{ EPL, "EPL", fEPL, false, {1,{MBIT,NONE,NONE}} },			//25
	{ ERA, "ERA", fTMP, false, {1,{MINT,NONE,NONE}} },			//26
	{ ERR, "ERR", fERR, false, {0,{NONE,NONE,NONE}} },			//27
	{ EST, "EST", fEST, false, {0,{NONE,NONE,NONE}} },			//28
	{ EXC, "EXC", fTMP, false, {1,{MINT,NONE,NONE}} },			//29
	{ FBK, "FBK", fFBK, false, {1,{MINT,NONE,NONE}} },			//30
	{ FFP, "FFP", fFFP, false, {1,{MINT,NONE,NONE}} },			//31
	{ FMR, "FMR", fTMP, false, {0,{NONE,NONE,NONE}} },			//32
	{ HAC, "HAC", fHAC, false, {1,{MSHORT,NONE,NONE}} },		//33
	{ HCG, "HCG", fHCG, false, {1,{MBIT,NONE,NONE}} },			//34
	{ HDX, "HDX", fHDX, false, {2,{MFULL,MFULL,NONE}} },		//35
	{ HOM, "HOM", fHOM, false, {0,{NONE,NONE,NONE}} },			//36
	{ HSC, "HSC", fHSC, false, {1,{NONE,NONE,NONE}} },			//37
	{ HST, "HST", fHST, false, {1,{MBIT,NONE,NONE}} },			//38
	{ HVL, "HVL", fHVL, false, {2,{MSHORT,MSHORT,NONE}} },		//39
	{ IDX, "IDX", fTMP, false, {0,{NONE,NONE,NONE}} },			//40
	{ INF, "INF", fTMP, false, {1,{MBIT,NONE,NONE}} },			//41
	{ INP, "INP", fINP, false, {2,{MINT,MSHORT,NONE}} },		//42
	{ IST, "IST", fTMP, false, {1,{MINT,NONE,NONE}} },			//43
	{ IWL, "IWL", fIWL, false, {1,{MINT,NONE,NONE}} },			//44
	{ JAC, "JAC", fJAC, false, {1,{MSHORT,NONE,NONE}} },		//45
	{ JOG, "JOG", fJOG, false, {1,{MSHORT,NONE,NONE}} },		//46
	{ LCG, "LCG", fLCG, false, {1,{MINT,NONE,NONE}} },			//47
	{ LCK, "LCK", fLCK, false, {1,{MINT,NONE,NONE}} },			//48
	{ LDP, "LDP", fLDP, false, {0,{NONE,NONE,NONE}} },			//49
	{ LDR, "LDR", fLDR, false, {1,{MBIT,NONE,NONE}} },			//50
	{ LIM, "LIM", fLIM, false, {0,{NONE,NONE,NONE}} },			//51
	{ LMR, "LMR", fTMP, false, {1,{MINT,NONE,NONE}} },			//52
	{ LPL, "LPL", fLPL, false, {1,{MBIT,NONE,NONE}} },			//53
	{ LST, "LST", fTMP, false, {1,{MINT,NONE,NONE}} },			//54
	{ MCR, "MCR", fMCR, false, {2,{MINT,MINT,MINT}} },			//55
	{ MLN, "MLN", fMLN, false, {0,{NONE,NONE,NONE}} },			//56
	{ MLP, "MLP", fMLP, false, {0,{NONE,NONE,NONE}} },			//57
	{ MND, "MND", fTMP, false, {0,{NONE,NONE,NONE}} },			//58
	{ MOT, "MOT", fMOT, false, {1,{MBIT,NONE,NONE}} },			//59
	{ MPL, "MPL", fMPL, false, {1,{MBIT,NONE,NONE}} },			//60
	{ MSA, "MSA", fMSA, false, {1,{MFULL,NONE,NONE}} },			//61
	{ MSR, "MSR", fMSR, false, {1,{MFULL,NONE,NONE}} },			//62
	{ MTC, "MTC", fMTC, false, {1,{MINT,NONE,NONE}} },			//63
	{ MTL, "MTL", fMTL, false, {1,{MINT,NONE,NONE}} },			//64
	{ MTP, "MTP", fMTP, true, {1,{MINT,NONE,NONE}} },			//65
	{ MTR, "MTR", fMTR, false, {1,{MINT,NONE,NONE}} },			//66
	{ MVA, "MVA", fMVA, false, {1,{MFULL,NONE,NONE}} },			//67
	{ MVR, "MVR", fMVR, false, {1,{MFULL,NONE,NONE}} },			//68
	{ MVT, "MVT", fMVT, false, {1,{MFULL,NONE,NONE}} },			//69
	{ MXE, "MXE", fMXE, false, {1,{MINT,NONE,NONE}} },			//70
	{ NDV, "NDV", fNDV, false, {0,{NONE, NONE, NONE}} },		//71
	{ PCD, "PCD", fTMP, false, {0,{NONE,NONE,NONE}} },			//72
	{ PID, "PID", fPID, false, {3,{MSHORT,MSHORT,MSHORT}} },	//73
	{ PGL, "PGL", fTMP, false, {2,{MINT,MINT,NONE}} },			//74
	{ PGM, "PGM", fTMP, false, {1,{MINT,NONE,NONE}} },			//75
	{ PGS, "PGS", fTMP, false, {1,{MINT,NONE,NONE}} },			//76
	{ POS, "POS", fPOS, false, {0,{NONE,NONE,NONE}} },			//77
	{ PRT, "PRT", fTMP, false, {1,{MINT,NONE,NONE}} },			//78
	{ QFT, "QFT", fTMP, false, {1,{MINT,NONE,NONE}} },			//79
	{ REZ, "REZ", fREZ, false, {1,{MINT,NONE,NONE}} },			//80
	{ RPS, "RPS", fTMP, false, {0,{NONE,NONE,NONE}} },			//81
	{ RST, "RST", fTMP, false, {0,{NONE,NONE,NONE}} },			//82
	{ RUN, "RUN", fRUN, false, {0,{NONE,NONE,NONE}} },			//83
	{ SAV, "SAV", fSAV, false, {0,{NONE,NONE,NONE}} },			//84
	{ SDF, "SDF", fTMP, false, {0,{NONE,NONE,NONE}} },			//85
	{ SID, "SID", fTMP, false, {1,{MINT,NONE,NONE}} },			//86
	{ SMC, "SMC", fTMP, false, {1,{MBIT,NONE,NONE}} },			//87
	{ SPI, "SPI", fTMP, false, {1,{MINT,NONE,NONE}} },			//88
	{ SPS, "SPS", fTMP, false, {1,{MFULL,NONE,NONE}} },			//89
	{ STA, "STA", fSTA, false, {0,{NONE,NONE,NONE}} },			//90
	{ STP, "STP", fSTP, false, {0,{NONE,NONE,NONE}} },			//91
	{ SVP, "SVP", fSVP, false, {2,{MINT,MFULL,NONE}} },			//92
	{ SYN, "SYN", fTMP, false, {0,{NONE,NONE,NONE}} },			//93
	{ TEC, "TEC", fTEC, false, {3,{MINT,MINT,MINT}} },			//94
	{ TLN, "TLN", fTLN, false, {1,{MFULL,NONE,NONE}} },			//95
	{ TLP, "TLP", fTLP, false, {1,{MFULL,NONE,NONE}} },			//96
	{ TPS, "TPS", fTMP, false, {0,{NONE,NONE,NONE}} },			//97
	{ TRA, "TRA", fTRA, false, {3,{MINT,MINT,MFULL}} },			//98
	{ TRG, "TRG", fTMP, false, {0,{NONE,NONE,NONE}} },			//99
	{ VEL, "VEL", fVEL, false, {1,{MSHORT,NONE,NONE}} },		//100
	{ VER, "VER", fVER, false, {0,{NONE,NONE,NONE}} },			//101
	{ VMX, "VMX", fVMX, false, {1,{MSHORT,NONE,NONE}} },		//102
	{ VRM, "VRM", fVRM, false, {1,{MINT,NONE,NONE}} },			//103
	{ VRT, "VRT", fVRT, false, {1,{MINT,NONE,NONE}} },			//104
	{ WDF, "WDF", fTMP, false, {0,{NONE,NONE,NONE}} },			//105
	{ WFA, "WFA", fWFA, false, {1,{MINT,NONE,NONE}} },			//106
	{ WFO, "WFO", fWFO, false, {3,{MINT,MINT,MINT}} },			//107
	{ WIO, "WIO", fTMP, false, {2,{MINT,MBIT,NONE}} },			//108
	{ WST, "WST", fTMP, false, {0,{NONE,NONE,NONE}} },			//109
	{ WSY, "WSY", fTMP, false, {1,{MINT,NONE,NONE}} },			//110
	{ WTM, "WTM", fTMP, false, {1,{MINT,NONE,NONE}} },			//111
	{ ZRO, "ZRO", fZRO, false, {0,{NONE,NONE,NONE}} },			//112
	{ ZZZ, "ZZZ", fTMP, false, {0,{NONE,NONE,NONE}} },			//113
	{ NUL, "NUL", fTMP, false, {0,{NONE,NONE,NONE}} }			//114 --> NULL Location. Change NULL_POSITION macro if this changes from 113
};

int16_t fTMP(eCmd cmd){
    return 1;
}
/*
typedef struct
{
	sRouteInfo rInfo;
	sCmdData cmdData;
} eCmd; */

int ExecuteCommand(eCmd cmd)
{
	uint16_t ErrorCode;

	if(	!CheckEligibility(cmd.rInfo.AxisOrigin, cmd.cmdData.Instr,&ErrorCode,cmd.cmdData.Request) )
	{
		//Command is not allowed given our current state.
		AddError(ErrorCode,cmd.cmdData.Instr_Str);
		return 0;
	}
	if(!CommandLookupArray[cmd.cmdData.Instr].impl(cmd))
	{
		//***ADD CODE *** --> add internal program code
		//if (cmd.rInfo.AxisOrigin == CMD_ORIGIN_PROGRAM){
		//	EndProgram();	//Error
		//}
	}
	return 1;
}

int16_t fACC(eCmd p)
{
	uint16_t  mm = 0, um = 0;
	uint32_t acc = Accel / 1000;
	if(p.cmdData.Request)
	{
		mm = acc / 1000;
		um = (acc - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um,(char*)p.rInfo.Data,0);		//here we go. converting acceleration. we should print nm/s if there are any.

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"ACC");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if( p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= AccelMax )
		{
			Accel = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"ACC");
			return 0;
		}
	}
	return 1;
}

int16_t fAEZ(eCmd p){
	int8_t sign1=1;
	int16_t  mm = 0, um = 0, nm = 0;
	int64_t temp1;
	int64_t enczo_nm;

	if(p.cmdData.Request){

		enczo_nm = (absoluteEncoderOffset * EncoderResolution) / 1000;
		mm = enczo_nm / 1000000;
		temp1 = (long long)mm * 1000000;
		um = (enczo_nm - temp1)/1000;
		nm = (enczo_nm - temp1 - um*(long)1000);

		convert_3(mm,um,nm,format_buf);		//###POS == 3 sprintf calls

		if(sign1 == -1)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%s\n\r",format_buf);
		}
		else
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n\r",format_buf);	//Add 1 for \0 character;
		}

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"POS");
			return 0;
		}
	}
	return 1;
}

int16_t fAMX(eCmd p)
{
	uint16_t  mm = 0, um = 0;
	uint32_t acc = AccelMax / 1000;
	if(p.cmdData.Request)
	{
		mm = acc / 1000;
		um = (acc - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um,(char*)p.rInfo.Data,0);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"AMX");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= SYSTEM_MAX_ACCEL)
		{
			AccelMax = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"AMX");
			return 0;
		}

	}
	if(Accel > AccelMax)
	Accel = AccelMax;
	if(Decel > AccelMax)
	Decel = AccelMax;
	if(JogAccel > AccelMax)
	JogAccel = AccelMax;
	return 1;
}

int16_t fANR(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",AddressingMode);	//Add 1 for \0 character;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"ANR");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.AxisNum == 0)
		{
			//Only allow a global command if you are setting everything back to auto-address.
			if(p.cmdData.Args[0] == 0)
			{
				AddressingMode = 0;
			}
			else
			{
				AddError(COMMAND_CANNOT_BE_USED_IN_GLOBAL_CONTEXT,"ANR");
				return 0;
			}

		}
		else if(p.cmdData.Args[0] == 0)
		{
			//BoardID stays the same
			AddressingMode = 0;
		}
		else if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 99)
		{
			AddressingMode = p.cmdData.Args[0];
			setAxis((uint8_t)AddressingMode);
			//setAxis(); setGate();
			//Also need to generate the whole CAN Filter LUT in RAM
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"ANR");
			return 0;
		}
	}
	return 1;
}

int16_t fCER(eCmd p)
{
	ClearErrors();
	TURN_OFF_RED_LED;
	TURN_ON_GREEN_LED;
	return 1;
}

int16_t fCFQ(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",SilentModeEnable) + 1;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"CFQ");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] == 0)
			{
				SilentModeEnable = 0;
			}
			else if(p.cmdData.Args[0] == 1)
			{
				SilentModeEnable = 1;
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"CFQ");
				return 0;
			}
			SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_SILENT_ENABLE, SilentModeEnable);
		}
	}
	return 1;
}

int16_t fCFS(eCmd p)
{
	unsigned int  hz = 0, khz = 0;
	int32_t temp_data;

	if(p.cmdData.Request)
	{
		khz = silentModeOpFreq / 1000;
		hz = (silentModeOpFreq - (khz * 1000));
		convert_2(khz,hz,format_buf,3);

		khz = silentModeThreshold / 1000;
		hz = (silentModeThreshold - (khz * 1000));
		convert_2(khz,hz,format_buf2,1);

		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%s,%s",format_buf,format_buf2)+ 1;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"CFS");
			return 0;
		}
	}
	else{
		if(FeedbackMode != OPEN_LOOP)
		{
			AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS,"CFS");
			return 0;
		}
		if(p.cmdData.ArgCheck[0]){
			if(p.cmdData.Args[0] >= 1600000 && p.cmdData.Args[0] <= 30000000){
				silentModeOpFreq = p.cmdData.Args[0] / 1000;
			}
			else{
				AddError(PARAMETER_OUT_OF_BOUNDS,"CFS");
				return 0;
			}
		}
		if(p.cmdData.ArgCheck[1]){
			if(p.cmdData.Args[1] > 0 && p.cmdData.Args[1] <= 30000000){
				silentModeThreshold = p.cmdData.Args[1] / 1000;
				//previous versions controlled threshold on this ARM, triggering silent mode through an IO
				//This version will attempt to handle threshold on motor ARM
			}
			else{
				AddError(PARAMETER_OUT_OF_BOUNDS,"CFS");
				return 0;
			}
		}
		temp_data = silentModeThreshold & 0xFFFF;
		temp_data |= (silentModeOpFreq << 16) & 0xFFFF0000;
		SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_SILENT_FREQUENCY, temp_data);
	}
	return 1;
}

int16_t fCVL(eCmd p)
{
	unsigned long Steps;
	if(p.cmdData.Request)
	{
		GetFormatted_Vel(ClLoopCorVelMax, &p.rInfo);
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin)){
			AddError(UNKNOWN_MESSAGE_ORIGIN,"VEL");
			return 0;
		}
	}
	else{
		if(p.cmdData.ArgCheck[0]){
			if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= VelMax){
				ClLoopCorVelMax = p.cmdData.Args[0];
				Steps = GetStepsPerServo(ClLoopCorVelMax,DAC_Res, SERVO_FREQ);
				SetClStepsPerServo(Steps);
				//TestStepsPerServo();
			}
		}
	}

	return 1;
}

/*
int16_t fCYM(eCmd p)
{
	uint16_t delay = 0;
	if(p.cmdData.Request)
	{
	}
	else
	{
	FORCE_SLIP_SET;
	  while(delay++ < 25)
	  {
	  __asm("nop");
	  }
	  FORCE_SLIP_CLR;
	}
	return 1;
}*/


int16_t fCZO(eCmd p){
	if(EncoderChannelSelect == ABSOLUTE){
		absoluteEncoderOffset = 0;
//--TODO Figure out what this does and convert to internal parameter storage
//		WriteMiscParam(ProgAvail, absoluteEncoderOffset);
	}
	return 1;
}

int16_t fDAC(eCmd p)
{
	uint32_t dac_val = 0;
	if(p.cmdData.Request)
	{

//--TODO Port this to stm dac
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",dac_val);
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"DAC");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= 1023)
			{
				dac_val = p.cmdData.Args[0];
//--TODO Port to stm dac
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"DAC");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fDAT(eCmd p)
{
	uint16_t traceIndex=0;

	if(p.cmdData.Request && traceDone)
	{
		for (traceIndex=0; traceIndex<traceSample; traceIndex++)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d  %ld  %ld  %d  %ld\n"	,traceIndex+1
																					,traceLog[traceIndex].trajPosition
																					,traceLog[traceIndex].realPosition
																					,traceLog[traceIndex].motorADC
																					,traceLog[traceIndex].dacAdjustment);
			
			RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin);
		}
		
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"\r");
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"DAT");
			return 0;
		}
	}
	else
	{
		AddError(TRACE_DID_NOT_COMPLETE,"DAT");
		return 0;
	}
	return 1;
}

int16_t fDBD(eCmd p)
{
	uint16_t  sec = 0, msec = 0;
	uint32_t timeout;
	if(p.cmdData.Request)
	{	//It makes more sense for this to be in seconds with a resolution of 1 ms. milliseconds and microseconds here is dumb.
		timeout = DeadbandTimeout / 1000;
		sec = timeout / 1000;
		msec = (timeout - (sec * 1000));
		convert_2(sec,msec,format_buf,3);
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%ld,%s\n\r",DeadbandEncoderCounts,format_buf);	//Add 1 for \0 character;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"DBD");
			return 0;
		}
	}
	//Parameter 1 - Encoder Deadband Range in Counts
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] >= 0)			//Put in a maximum value here.
			{
				DeadbandEncoderCounts = p.cmdData.Args[0];
				DeadbandWindow = (p.cmdData.Args[0] * EncoderResolution)/1000;  //Deadband Window in nm, res in nm.  Need this to be in like hundreds of nm.
				ZeroCorrectionAttempts();
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"DBD");
				return 0;
			}
		}
		//Parameter 2 - Deadband Correction Timeout
		if(p.cmdData.ArgCheck[1])	//Did we get this parameter?
		{
			if(p.cmdData.Args[1] >= 0)
			{
				//Deadband Timeout is in ms
				DeadbandTimeout = p.cmdData.Args[1];
				DeadbandTimeoutCounts = (p.cmdData.Args[1] * SERVOS_PER_MS)/1000;
				ZeroCorrectionAttempts();
				//DeadbandTimeoutCounts = timeout in ms / ms/servo (units = servo cycles)
			}

			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"DBD");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fDBG(eCmd p)
{
	//Using this function to just test reading from flash
	if(p.cmdData.Request)
	{

		p.rInfo.Length = sprintf( (char*)p.rInfo.Data, "#%ld\n\r", spiPacketCounter);
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"DBG");
			return 0;
		}
	}
	return 1;
}

int16_t fDEC(eCmd p)
{
	uint16_t  mm = 0, um = 0;
	uint32_t dec = Decel / 1000;

	if(p.cmdData.Request)
	{
		mm = dec / 1000;
		um = (dec - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um,(char*)p.rInfo.Data,0);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"DEC");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if( p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= AccelMax )
		{
			Decel = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"DEC");
			return 0;
		}
	}
	return 1;
}

int16_t fDEF(eCmd p)
{
	//--TODO Port this
	//Turn off the servo timer during a DEF command process. It is interrupting the FPGA settings and generating errors

	SetupDefaults();
	AxisSetup();	//Need to setup variables dependent on parameters again.

	//Turn on Servo Timer after DEF has been processed

	return 1;
}

int16_t fEAD(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",EncoderChannelSelect);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"EAD");
			return 0;
		}
	}
	//use Quad Channel 1
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == 0)
		{
			EncoderChannelSelect = DIGITAL;
//!			ENCODER_CHANNEL_DIGITAL_SELECT;		//Tell FPGA to read the Digital Channel
//!			ClearVRTLog();
//!			ReInitEncFilter();
//!			ABSOLUTE_ENCODER_DESELECT;
		}
		else if(p.cmdData.Args[0] == 1)
		{
			EncoderChannelSelect = ANALOG;
//!			ENCODER_CHANNEL_ANALOG_SELECT;
//!			ClearVRTLog();
//!			ReInitEncFilter();
//!			ABSOLUTE_ENCODER_DESELECT;
			//Tell FPGA to read the Analog Channel
		}
		else if(p.cmdData.Args[0] == 2)
		{
			EncoderChannelSelect = ABSOLUTE;
//!			ClearVRTLog();
//!			ReInitEncFilter();
//!			ABSOLUTE_ENCODER_SELECT;
		}
		else
		{
			//Generate an out of bounds error if none of the above conditionals are met
			AddError(PARAMETER_OUT_OF_BOUNDS, "EAD");
			return 0;
		}
	}
	return 1;
}

int16_t fEDV(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",EncoderDetectionVal);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"EDV");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] < 1000001)
		{
			EncoderDetectionVal = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"EDV");
			return 0;
		}
	}

	return 1;
}

int16_t fEFF(eCmd p){

	if(p.cmdData.Request){
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d,%d\n\r",EncFilterWindow_Motion,EncFilterWindow_Standstill);	//Add 1 for \0 character;
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"EFF");
			return 0;
		}
	}
	else{
		if(p.cmdData.ArgCheck[0])
		{
			//Update conditional for p1. Should only allow for 1-512 range
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= ENC_FILTER_MOTION_MAX)
			{
				EncFilterWindow_Motion = p.cmdData.Args[0];
	            ReInitEncFilter();
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"EFF");
				return 0;
			}
		}
		//Parameter 2 - Deadband Correction Timeout
		if(p.cmdData.ArgCheck[1])
		{
			if(p.cmdData.Args[1] > 0 && p.cmdData.Args[1] <= ENC_FILTER_STANDSTILL_MAX)
			{
	            EncFilterWindow_Standstill = p.cmdData.Args[1];
	            ReInitEncFilter();
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"EFF");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fENC(eCmd p)
{
	unsigned int  nm = 0, um = 0, pm = 0;;
	unsigned int temp1 = 0;
	if(p.cmdData.Request)
	{
		um = EncoderResolution / 1000000;
		temp1 = (um * 1000000);
		nm = (EncoderResolution - temp1)/1000;
		pm = EncoderResolution - temp1 - (nm * 1000);
		convert_3(um,nm,pm,format_buf);
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n\r",format_buf);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"ENC");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] < 10000000000ULL)	//Max encoder res == 10mm
		{
			EncoderResolution = p.cmdData.Args[0] / 1000;
			DeadbandWindow = (DeadbandEncoderCounts * EncoderResolution) / 1000;	//nanometers b.
			//need to send new encoder data to motherboard MCU
			SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_ENC_RESOLUTION, (EncoderResolution/1000));
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"ENC");
			return 0;
		}
	}
	return 1;
}

int16_t fEND(eCmd p)
{
	return 1;
}

int16_t fENR(eCmd p){
	uint32_t crc_err,err_flag,warn_flag;
	if(p.cmdData.Request)
	{
		GetAbsEncStatus(&crc_err,&err_flag,&warn_flag);
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data ,"#%ld,%ld,%ld\n\r",crc_err,err_flag,warn_flag);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"ENR");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == 0)
		{
			ZeroAbsEncCounters();
		}
	}
	else
	{
		AddError(PARAMETER_OUT_OF_BOUNDS,"ENR");
		return 0;
	}

	return 1;
}

//QEI_CONFIG_SWAP
int16_t fEPL(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data ,"#%d\n\r",EncoderPolarity );

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"EPL");
			return 0;
		}
	}

	else if(p.cmdData.ArgCheck[0])
	{
//--TODO Port this to stm
		if(p.cmdData.Args[0] == 0)
		{
			EncoderPolarity = p.cmdData.Args[0];
			//Switch encoder polarity
		}
		else if(p.cmdData.Args[0] == 1)
		{
			EncoderPolarity = p.cmdData.Args[0];
			//Switch encoder polarity
		}

		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"EPL");
			return 0;
		}
	}

	return 1;
}

//Erase Program.
int16_t fERA(eCmd p)
{
	if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= PROGRAM_NUM_LIMIT)
		{
//--TODO Port this to stm
//			EraseProgram((unsigned char)p.cmdData.Args[0]);
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"ERA");
			return 0;
		}
	}

	return 1;
}

int16_t fERR(eCmd p)
{
	uint8_t error_count=0;
	struct Error err;

	if(p.cmdData.Request)
	{
		while(GetError(&err))
		{
			error_count++;
			ErrorLookup(err.ErrorNum, format_buf);
			if(ErrorBufferEmpty())
			{
				p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#Error %d - %c%c%c - %s\n\r",err.ErrorNum,err.instr[0],err.instr[1],err.instr[2],format_buf);
			}
			else
			{
				p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#Error %d - %c%c%c - %s\n",err.ErrorNum,err.instr[0], err.instr[1],err.instr[2],format_buf);
			}

			if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
			{
				AddError(UNKNOWN_MESSAGE_ORIGIN,"ERR");
				return 0;
			}
			TURN_OFF_RED_LED;
			TURN_ON_GREEN_LED;

		}
	}
	return 1;
}

int16_t fEST(eCmd p)
{
	if(ProgramRunning())
	{
		EndProgram();
	}

	if(HomeInProgress())
	{
		CancelHome();
	}

	if(MotionInProgress())
	{
		EmergencyStop();
	}

	if(FeedbackMode == CLOSED_LOOP || FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
	{
		//Fix EST bug. Was locking up the controller in FBK2
		while(getDriverState() != DRIVER_OFF);
		CurrentPos = RealPosition;
		New_ServoPosition = RealPosition;
	}

	return 1;
}

uint8_t ProgramHostAxisNum = 0;
uint8_t ProgramHostGateNum = 0;
int16_t fEXC(eCmd p)
{
	if(p.cmdData.Request)
		{
		AddError(READ_NOT_AVAILABLE_FOR_THIS_COMMAND,"EXC");
		return 0;
		}
	if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= PROGRAM_NUM_LIMIT)
		{
			if(1<<(p.cmdData.Args[0]-1) & ProgAvail)
			{
				SetupProgramExec((unsigned char)p.cmdData.Args[0],1);	//Prepare to execute a motion program from flash.
				ProgramHostAxisNum = p.rInfo.sAxis;
				ProgramHostGateNum = p.rInfo.sGate;
			}
			else
			{
				AddError(PROGRAM_DOESNT_EXIST,"EXC");
				return 0;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"EXC");
			return 0;
		}
	}
	return 1;
}

int16_t fFBK(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",FeedbackMode);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"FBK");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.AxisNum == 0)
		{
			AddError(COMMAND_CANNOT_BE_USED_IN_GLOBAL_CONTEXT,"FBK");
			return 0;
		}
		else if(p.cmdData.Args[0] == 0)
		{
			FeedbackMode = OPEN_LOOP;
			DetectEncoder = 0;
		}
		else if(p.cmdData.Args[0] == 1)
		{
			FeedbackMode = OPEN_LOOP;
			DetectEncoder = 0;
		}
		else if(p.cmdData.Args[0] == 2)
		{
			FeedbackMode = OPEN_LOOP_CLOSED_FINISH;
			DetectEncoder = 1;
		}
		else if(p.cmdData.Args[0] == 3)
		{
			FeedbackMode = CLOSED_LOOP;
			DetectEncoder = 1;
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"FBK");
			return 0;
		}
	}
	return 1;
}

int16_t fFFP(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",FeedForwardParameter);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"FFP");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{

		if(p.cmdData.Args[0] >= 0)
		{
			FeedForwardParameter = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"FFP");
			return 0;
		}
	}
	return 1;
}

// THIS FUNCTION WILL CLEAR THE JUMP TO APP FLAG AND THEN JUMP TO BOOTLOADER
//--TODO Port to STM
int16_t fFMR(eCmd p){
/*	if((u32IAP_PrepareSectors(BL_CNTROL_VAR_SECTOR, BL_CNTROL_VAR_SECTOR) == IAP_STA_CMD_SUCCESS )){
		if((u32IAP_EraseSectors  (BL_CNTROL_VAR_SECTOR, BL_CNTROL_VAR_SECTOR) == IAP_STA_CMD_SUCCESS)){
			// SYSTEM RESET
			NVIC_SystemReset( );
		}
	}*/
	return 1;
}

int16_t fHAC(eCmd p)
{
	uint16_t  mm = 0, um = 0;
	uint32_t acc;
	if(p.cmdData.Request)
	{
		acc = HomeAccel / 1000;
		mm = acc / 1000;
		um = (acc - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um,(char*)p.rInfo.Data,0);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"HAC");
			return 0;
		}

	}
	else if(p.cmdData.ArgCheck[0])
	{
		if( p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= SYSTEM_MAX_ACCEL )
		{
			HomeAccel = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"ACC");
			return 0;
		}
	}
	X_bkv = 2;
	return 1;
}

int16_t fHCG(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",HomeConfig);	//Add 1 for the \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"HCG");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] < 10)
		{
			//What are the different possibilities for Home Config
			//Go to +Limit then Index
			//Go to -Limit then Index
			//Go to +Limit(MLP Command)
			//Go to -Limit(MLN Command)
			//...

			HomeConfig = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"HCG");
			return 0;
		}
	}
	return 1;
}

int16_t fHDX(eCmd p)
{
//	if(!RestrictedCommandsLock)
//	{
	unsigned long long temp1;
	unsigned int  mm = 0, um = 0, nm = 0;
	unsigned int  mm2 = 0, um2 = 0, nm2 = 0;
	char temp_buf[50];

	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"HDX");
		return 0;
	}

	if(p.cmdData.Request)
	{

		mm= HomeDistance1 / 1000000000;
		//temp1 = sign1 * mm * 2000000;
		temp1 = mm * 1000000000;
		um = (HomeDistance1 - temp1)/1000000;
		nm = HomeDistance1 - temp1 - um*(long)1000000;

		mm2 = HomeDistance2 / 1000000000;
		temp1 = mm2 * 1000000000;
		um2 = (HomeDistance2 - temp1)/1000000;
		nm2 = HomeDistance2 - temp1 - um2*(long)1000000;


		convert_3(mm,um,nm,format_buf);
		convert_3(mm2,um2,nm2,temp_buf);

		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s,%s\n\r",format_buf,temp_buf);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"HDX");
			return 0;
		}
	}
	else
	{
		if(HomeInProgress())
		{
			AddError(HOME_IN_PROGRESS,"HDX");
			return 0;
		}

		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0)
			{
				HomeDistance1 = p.cmdData.Args[0];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"HDX");
				return 0;
			}
		}

		if(p.cmdData.ArgCheck[1])
		{
			if(p.cmdData.Args[0] > 0)
			{
				HomeDistance2 = p.cmdData.Args[1];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"HDX");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fHOM(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r",isHomed() );	//Add 1 for the NULL character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"HOM");
			return 0;
		}
	}
	else
	{
		if(HomeInProgress())
		{
			AddError(HOME_IN_PROGRESS,"HOM");
			return 0;
		}
		SetupHome(TARGETED);
	}

	return 1;
}

int16_t fHSC(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",HardStopDeadband); // Add 1 for \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"HSC");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 100)
		{
			HardStopDeadband = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"HSC");
			return 0;
		}
	}
	return 1;
}

int16_t fHST(eCmd p)
{
	if(p.cmdData.Request)
	{//This should be another variable. *HardStopDetect. Cover all bases, with limit possibilities in home.
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r",HardstopDetectEnable);	//Add 1 for \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"HST");
			return 0;
		}
	}
	else
	{
		HardstopDetectEnable = p.cmdData.Args[0];	//Ok
	}

	return 1;
}

int16_t fHVL(eCmd p)
{
	uint16_t  mm = 0, um = 0;
	uint16_t  mm2 = 0, um2 = 0;//, nm2 = 0;
	//Fuck printing nm/s for now
	uint32_t vel = HomeVel / 1000;
	uint32_t slow_hvel = SlowHVel / 1000;
	uint64_t temp_hvel = 0;
	char temp_buf[50];

	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"HVL");
		return 0;
	}

	if(p.cmdData.Request)
	{
		mm = vel / 1000;
		um = (vel - (mm * 1000));
		convert_2(mm,um,format_buf,3);

		mm2 = slow_hvel / 1000;
		um2 = (slow_hvel - (mm2 * 1000));
		convert_2(mm2,um2,temp_buf,1);

		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s, %s",format_buf,temp_buf);	//Add 1 for \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"HVL");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{

		if(HomeInProgress())
		{
			AddError(HOME_IN_PROGRESS,"HVL");
			return 0;
		}

		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= VelMax)
			{
				temp_hvel = p.cmdData.Args[0];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"HVL");
				return 0;
			}
		}

		if(p.cmdData.ArgCheck[1])
		{
			if(p.cmdData.Args[1] > 0 && p.cmdData.Args[1] <= 1000000)
			{
				SlowHVel = p.cmdData.Args[1];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"HVL");
				return 0;
			}
		}

		//accept 1st parameter after checking validity of 2nd
		if(temp_hvel)
		{
			HomeVel= temp_hvel;
		}
	}
	return 1;
}
//
int16_t fIDX(eCmd p)
{
	char sign2=1;
	int64_t temp1, RP_Freeze;
	int32_t  mm2 = 0, um2 = 0, nm2 = 0;
	char temp_buf[50];

	if(p.cmdData.Request)
	{
		RP_Freeze = IndexLatch[0] / 1000;

		if(RP_Freeze < 0)
		{
			sign2 = -1;
			RP_Freeze = -RP_Freeze;
		}
		mm2 = RP_Freeze / 1000000;
		//temp1 = sign2 * mm2 * 2000000;
		temp1 = (long long)mm2 *  1000000;
		um2 = (RP_Freeze - temp1)/1000;
		nm2 = (RP_Freeze - temp1 - um2*(long)1000);
		//if( sign2 == -1) mm2 = -mm2;
		convert_3(mm2,um2,nm2,temp_buf);	//####2 sprintfs

		if(sign2 == -1)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%s\n\r",temp_buf);
		}
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n\r",temp_buf);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"IDX");
			return 0;
		}
	}
	return 1;
}

int16_t fINF(eCmd p)
{
//Test Command to start an infinite move
	if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == 0)
		{
			if(SetupMove(1,INFINITE))
			{
				StartMove();
			}
		}
		else if(p.cmdData.Args[0] == 1)
		{
			if(SetupMove(-1,INFINITE))
			{
				StartMove();
			}
		}
	}
	return 1;
}

int16_t fINP(eCmd p)
{
	uint16_t  sec = 0, msec = 0;
	uint32_t lvalue;

	if(p.cmdData.Request)
	{	//It makes more sense for this to be in seconds with a resolution of 1 ms. milliseconds and microseconds here is dumb.
		lvalue = InPositionTime / 1000;
		sec = lvalue / 1000;
		msec = (lvalue - (sec * 1000));
		convert_2(sec,msec,format_buf,3);
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%ld,%s\n\r",InPositionCounts,format_buf);	//Add 1 for \0 character;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"INP");
			return 0;
		}
	}
	//Parameter 1 - Encoder Deadband Range in Counts
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] >= 0)			//Put in a maximum value here.
			{
				InPositionCounts = p.cmdData.Args[0];
				InPositionWindow = (p.cmdData.Args[0] * EncoderResolution)/1000;  //Deadband Window in nm, res in nm.  Need this to be in like hundreds of nm.
				ZeroCorrectionAttempts();
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"INP");
				return 0;
			}
		}
		//Parameter 2 - Deadband Correction Timeout
		if(p.cmdData.ArgCheck[1])	//Did we get this parameter?
		{
			if(p.cmdData.Args[1] >= 0)
			{
				//Deadband Timeout is in ms
				InPositionTime = p.cmdData.Args[1];
				InPositionTimeCounts = (p.cmdData.Args[1] * SERVOS_PER_MS)/1000;
				ZeroCorrectionAttempts();
				//DeadbandTimeoutCounts = timeout in ms / ms/servo (units = servo cycles)
			}

			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"INP");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fIWL(eCmd p){
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",iTermSumLimit);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"IWL");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 10000000)
		{
			iTermSumLimit = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"IWL");
			return 0;
		}
	}
	return 1;
}

int16_t fIST(eCmd p){
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",iTermSampleTime);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"EDV");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 50000)
		{
			iTermSampleTime = p.cmdData.Args[0];
			ResetiTermSampleCounter();
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"EDV");
			return 0;
		}
	}
	return 1;
}

int16_t fDRL(eCmd p){
	if(p.cmdData.Request){
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",dTermDiffLimit);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin)){
			AddError(UNKNOWN_MESSAGE_ORIGIN,"DRL");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 100000000)
		{
			dTermDiffLimit = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"EDV");
			return 0;
		}
	}
	return 1;
}

int16_t fDST(eCmd p){
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",dTermSampleTime);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"EDV");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 50000)
		{
			dTermSampleTime = p.cmdData.Args[0];
			ResetdTermSampleCounter();
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"EDV");
			return 0;
		}
	}
	return 1;
}

int16_t fCLT(eCmd p){
	int32_t pt,it,dt;
	if(p.cmdData.Request)
	{
		GetPidTerms(&pt,&it,&dt);
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%ld,%ld,%ld\n\r",pt,it,dt);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"CLT");
			return 0;
		}
	}
	return 1;
}

int16_t fJAC(eCmd p)
{
	uint16_t  mm = 0, um = 0;
	//Fuck printing nm/s for now
	uint32_t acc = JogAccel / 1000;
	if(p.cmdData.Request)
	{
		mm = acc / 1000;
		um = (acc - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um, (char*)p.rInfo.Data,0);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"JAC");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= AccelMax)
		{
			JogAccel = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"JAC");
			return 0;
		}
	}
	return 1;
}

int64_t JogVel=0,oldJogVel = 0;
int16_t fJOG(eCmd p)
{
	uint16_t  mm = 0, um = 0;
	enum MotionPhases TS;

	int32_t vel = JogVel / 1000;
	if(p.cmdData.Request)
	{
		if (vel < 0) vel = vel * -1;

		mm = vel / 1000;
		um = (vel - (mm * 1000));

		//Account for a negative return
		if (mm < 0) mm = mm * -1;
		if (um < 0) um = um * -1;

		if (JogVel < 0) p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%d.%d\n\r",mm, um);
		else p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d.%d\n\r",mm, um);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"JOG");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		//How to add in
		if(p.cmdData.Args[0] >= -100000000 && p.cmdData.Args[0] <= 100000000)
		{
			oldJogVel = JogVel;

			//removed unsigned calculation. Was preventing proper motion in the negative direction
			JogVel = (p.cmdData.Args[0]/1000) * (int64_t)VelMax / 100000;

			TS = getTrajState();
			//Implement JOG 0
			//What's the limitation here? I can use

			if( p.cmdData.Args[0] == 0 )
			{
				JogVel = 0;
				//Stop Motion
				if(MotionInProgress())
				{
					StopMotion();
				}
			}
			else if( TS == CONSTANT_VELOCITY )	//Need JOG_MODE/MOVE_MODE distinction, which sucks.
			{
				if(getMotionMode() == JOG_) //I need to do a direction change.
				{
					//if Jog is opposite our old jog direction. Stop,
					//And then start a move in the other direction.
					if((oldJogVel > 0 && JogVel < 0) || (oldJogVel < 0 && JogVel > 0))
					{
						StopMotion();	//In the future, this shouldn't block. Once the timer interrupt which runs an assignable function pointer is implemented, I can make this non-blocking.
						while(getDriverState() != DRIVER_OFF);
						if(SetupJog(JogVel,getJogType()))
						{
							StartMove();
						}
						else return 0;
					}
					else
					{
						if(!VelocityCOF_Setup(JogVel))
						{
							return 0;
						}
					}
				}
				else
				{
					AddError(NOT_IN_JOG_MODE,"JOG");
					return 0;
				}
			}
			else if( TS == NO_MOTION || (SyncMoveSet && !MotionInProgress()))	//SyncMoveSet is here, because we want to override a sync move that hasn't started yet.
			{
				SyncMoveSet = 0;
				if(SetupJog(JogVel,getJogType()))
				{
					StartMove();
				}
			}
			else
			{
				AddError(JOG_NOT_PERMITTED,"JOG");
				return 0;
			}
			return 1;
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS, "JOG");
			return 0;
		}
	}
	return 1;
}

int16_t fLCG(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",LimitConfig);	//Add 1 for \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"LCG");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= 3)
		{
			//bit 0 - soft limit enable
			//bit 1 - limit input enable

			if(CheckLimitConfig((unsigned char)p.cmdData.Args[0]))
			{
				LimitConfig = p.cmdData.Args[0];
				if(LimitConfig == 2 || LimitConfig == 3)
				{
					EnableLimitDetectInterrupts();
				}
				else
				{
					DisableLimitDetectInterrupts();
				}
				UpdateLimitState();
			}
			else
			{
				AddError(CURRENT_POSITION_OUTSIDE_SOFT_LIMITS,"LCG");
				return 0;
			}

		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"LCG");
			return 0;
		}
	}
	return 1;
}

int16_t fLCK(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",CommandLock);	//Add 1 for \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"LCK");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == UNLOCK_KEY)
		{
			CommandLock = 0;
		}
		else
		{
			CommandLock = 1;
		}
	}

	return 1;
}

int16_t fLDP(eCmd p)
{
	ReadParameters(BASE_PARAM_ADDRESS);
	return 1;
}

int16_t fLDR(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",LimitDir);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"LDR");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == 0)
		{
			LimitDir = p.cmdData.Args[0];
		}
		else if( p.cmdData.Args[0] == 1)
		{
			LimitDir = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"LDR");
			return 0;
		}
	}
	return 1;
}

int16_t fLIM(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d,%d\n\r",InPositiveLimit,InNegativeLimit);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"LIM");
			return 0;
		}
	}
	return 1;
}

int16_t fLPL(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r",LimitPolarity);	//Add 1 for \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"LPL");
			return 0;
		}
	}

	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == 0 || p.cmdData.Args[0] == 1)
		{
			LimitPolarity = p.cmdData.Args[0];	//Behavior handled in dio.c
			UpdateLimitState();
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"LPL");
			return 0;
		}
	}
	return 1;
}

uint32_t line_counter = 0;
uint8_t xprogram_line[PROGRAM_LINE_LENGTH_MAX];
int16_t fLST(eCmd p)
{
	line_counter = 0;

	if(p.cmdData.Request)
	{
	AddError(READ_NOT_AVAILABLE_FOR_THIS_COMMAND,"LST");
	return 0;
	}

	if(p.cmdData.ArgCheck[0])
	{

		if(p.cmdData.Args[0] >= 1 && p.cmdData.Args[0] <= PROGRAM_NUM_LIMIT)
		{
			if((1<<(p.cmdData.Args[0]-1)) & ProgAvail)
			{
				SetupProgramRead(p.cmdData.Args[0]);
				while(ReadNextProgramLine(xprogram_line))
				{
					line_counter++;
					if(ProgramReadDone())
					{
						p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n\r",xprogram_line);
					}
					else
					{
						p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n",xprogram_line);
					}

					if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
					{
						AddError(UNKNOWN_MESSAGE_ORIGIN,"LST");
						return 0;
					}
				}
			}
			else
			{
				AddError(PROGRAM_DOESNT_EXIST,"LST");
				return 0;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"LST");
			return 0;
		}
	}
	return 1;
}

int16_t fMCR(eCmd p)
{
	int8_t spi_status;
	
	if( p.cmdData.Request )
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld,%ld,%ld\n\r",iMax, i2Time, iOffset);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MCR");
			return 0;
		}
	}
	else
	{
		if (!MotorActive)
		{
			if(p.cmdData.ArgCheck[0])
			{
				iMax = p.cmdData.Args[0];
			}

			if(p.cmdData.ArgCheck[1])
			{
				i2Time = p.cmdData.Args[1];
			}

			if(p.cmdData.ArgCheck[2])
			{
				iOffset = p.cmdData.Args[2];
			}

			if (motorControlMode == MOTOR_BLDC_MODE)
			{
				spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_CURRENT, (int32_t)iMax, (int32_t)i2Time, (int32_t)iOffset);
				if (spi_status == 0)
				{
					AddError(PARAMETER_TRANSFER_FAILURE, "MCR");
					return 0;
				}
			}
			else
			{
				AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS, "MCR");
				return 0;
			}
		}
		else
		{
			AddError(MOTOR_IS_ENABLED, "MCR");
			return 0;
		}

	}

	return 1;
}

int16_t fMLN(eCmd p)
{
	if(!p.cmdData.Request)
	{
		if(HomeInProgress())
		{
			AddError(HOME_IN_PROGRESS,"MLN");
			return 0;
		}

		//Take LCG setting into account
		if ( (InNegativeLimit == 1) && (LimitConfig == 2 || LimitConfig == 3) )
		{
			AddError(CANNOT_MOVE_INTO_HARD_LIMIT,"MLN");
			return 0;
		}
		SetupMoveToLimit(0);
	}
	return 1;
}

int16_t fMLP(eCmd p)
{
	if(!p.cmdData.Request)
	{
		if(HomeInProgress())
		{
			AddError(HOME_IN_PROGRESS,"MLP");
			return 0;
		}

		//Take LCG setting into account
		if ( (InPositiveLimit == 1) && (LimitConfig == 2 || LimitConfig == 3) )
		{
			AddError(CANNOT_MOVE_INTO_HARD_LIMIT,"MLP");
			return 0;
		}
		SetupMoveToLimit(1);
	}
	return 1;
}

int16_t fMND(eCmd p)
{
	if(p.cmdData.Request)
	{
		//sprintf(myString,"#%d;%d\n\r",TestMinDistance/1000,MinDistance/1000);
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%ld;%ld\n\r",(uint32_t)TestMinDistance/1000,(uint32_t)MinDistance/1000);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MND");
			return 0;
		}
	}

	return 1;
}

int16_t fMOT(eCmd p)
{
	int8_t spi_status;
	int8_t sendSPI = 0;
	
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r",MotorActive );

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MOT");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == 0)
		{
			if (MotorActive==1)
			{
				MotorActive = 0;
				sendSPI = 1; 	//raise flag to send SPI motor enable/disable command
			}
		}
		else if( p.cmdData.Args[0] == 1)
		{
			if (MotorActive==0)
			{
				MotorActive = 1;
				sendSPI = 1;	//raise flag to send SPI motor enable/disable command
				//set the trajectory position to the encoder position
				if (FeedbackMode == CLOSED_LOOP) CurrentPos = RealPosition;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"MOT");
			return 0;
		}

		//tell motor ARM to enable/disable
		if (motorControlMode==MOTOR_BLDC_MODE && sendSPI) 
		{
			//when disabling motor we could be in the middle of an SPI transmission. We need to wait for it to finish to prevent any conflicts
			while (spiTxFlag);
			
			spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_ENABLE, (int32_t)MotorActive, 0, 0);
			if (spi_status == 0)
			{
				AddError(PARAMETER_TRANSFER_FAILURE, "MOT");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fMPL(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r",MPol);	//Add 1 for \0 character

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MPL");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] == 0 || p.cmdData.Args[0] == 1)
		{
			MPol = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"MPL");
			return 0;
		}
	}
	return 1;
}

int16_t fMSA(eCmd p)
{
	if(p.cmdData.ArgCheck[0])
	{
		if(SetupMove((p.cmdData.Args[0] - CurrentPos),TARGETED))
		{
			SyncMoveSet = 1;
			return 1;
		}
		return 0;
	}
	return 1;
}

//You cannot use these synchronous commands to do a PCOF
int16_t fMSR(eCmd p)
{
	if(p.cmdData.ArgCheck[0])
	{
		if( SetupMove(p.cmdData.Args[0], TARGETED))
		{
			SyncMoveSet = 1;
			return 1;
		}
		else return 0;
	}
	return 1;
}

int16_t fMTP(eCmd p)
{
	int8_t spi_status;
	
	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"MTP");
		return 0;
	}
	
	if( p.cmdData.Request )
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",motorPitch);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MTP");
			return 0;
		}
	}
	else if (p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0)
		{
			if (!MotorActive)
			{
				//TODO --> make sure the motor is disabled before sending this data.
				motorPitch = p.cmdData.Args[0];
				//send motor pitch data to motor ARM, in units of microns
				//we only want to send this data if we are in BLDC mode
				if (motorControlMode == MOTOR_BLDC_MODE)
				{
					SPI2_NSS_ON;
					HAL_Delay(100);
					spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_PITCH, (int32_t)(motorPitch*1000), 0, 0);
					if (spi_status == 0)
					{
						AddError(PARAMETER_TRANSFER_FAILURE, "MTP");
						return 0;
					}
				}
				else
				{
					AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS, "MTP");
					return 0;
				}
			}
			else
			{
				AddError(MOTOR_IS_ENABLED, "MTP");
				return 0;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS, "MTP");
			return 0;
		}
	}
	return 1;
}

int16_t fMTR(eCmd p)
{
	int8_t spi_status;
	
	if( p.cmdData.Request )
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",motorResistance);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MTR");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.Args[0] > 0)
		{
			if (!MotorActive)
			{
				//TODO --> make sure the motor is disabled before sending this data.
				motorResistance = p.cmdData.Args[0];
				//send motor pitch data to motor ARM, in units of microns
				//we only want to send this data if we are in BLDC mode
				if (motorControlMode == MOTOR_BLDC_MODE)
				{
					spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_PH_RESISTANCE, (int32_t)motorResistance, 0, 0);
					if (spi_status == 0)
					{
						AddError(PARAMETER_TRANSFER_FAILURE, "MTR");
						return 0;
					}
				}
				else
				{
					AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS, "MTR");
					return 0;
				}
			}
			else
			{
				AddError(MOTOR_IS_ENABLED, "MTR");
				return 0;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS, "MTR");
			return 0;
		}
	}
	return 1;
}

int16_t fMTL(eCmd p)
{
	int8_t spi_status;
	
	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"MTL");
		return 0;
	}

	if( p.cmdData.Request )
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",motorInductance);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MTL");
			return 0;
		}
	}

	if(p.cmdData.ArgCheck[0])
	{
		if (!MotorActive)
		{
			if(p.cmdData.Args[0] >=0){
				motorInductance = p.cmdData.Args[0];
				if (motorControlMode == MOTOR_BLDC_MODE)
				{
					spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_PH_INDUCTANCE, (int32_t)motorInductance, 0, 0);

					if (spi_status == 0)
					{
						AddError(PARAMETER_TRANSFER_FAILURE, "MTL");
						return 0;
					}
				}
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS, "MTL");
				return 0;
			}
		}
		else
		{
			AddError(MOTOR_IS_ENABLED, "MTL");
			return 0;
		}
	}
	return 1;
}

int16_t fMTC(eCmd p)
{
	int8_t spi_status;
	
	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"MTC");
		return 0;
	}

	if( p.cmdData.Request )
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",motorControlMode);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MTC");
			return 0;
		}
	}

	if(p.cmdData.ArgCheck[0])
	{
		if (!MotorActive)
		{
			if(p.cmdData.Args[0] >=0 && p.cmdData.Args[0] <=3){
				motorControlMode = p.cmdData.Args[0];
				if (motorControlMode == MOTOR_BLDC_MODE)
				{
					spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_TYPE, (int32_t)motorControlMode, 0, 0);
					
					if (spi_status == 0)
					{
						AddError(PARAMETER_TRANSFER_FAILURE, "MTC");
						return 0;
					}
				}
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS, "MTC");
				return 0;
			}
		}
		else
		{
			AddError(MOTOR_IS_ENABLED, "MTC");
			return 0;
		}
	}
	return 1;
}

int16_t fMVA(eCmd p)
{
	enum MotionPhases TS;
	unsigned char res = 0;
	//Pass this in to the Trajectory.

	if(p.cmdData.Request)
	{
		AddError(READ_NOT_AVAILABLE_FOR_THIS_COMMAND,"MVA");
		return 0;
	}

	if(p.cmdData.ArgCheck[0])
	{
		//local_TargetVel = *Vel;
		TS = getTrajState();
		if(!CheckMove(p.cmdData.Args[0], MVA)) return 0;	//<--Add Error? Error handling taken care of inside CheckMove?
		if( TS == CONSTANT_VELOCITY )
		{
			if( getMotionMode() == MOVE_)
			{
				res = PositionCOF_Setup(p.cmdData.Args[0] - getTargetPos());
				if(!res) return 0;
				if(res == DIR_CHANGE)
				{
					StopMotion();
					while(getDriverState() != DRIVER_OFF);
					if(SetupMove((p.cmdData.Args[0] - CurrentPos),TARGETED))
					{
						StartMove();
					}
				}
			}
			else
			{
				AddError(CANNOT_START_MOVE_DURING_JOG_MODE,"MVA");
				return 0;
			}
		}
		else if( TS == NO_MOTION )
		{
			if(SetupMove((p.cmdData.Args[0] - CurrentPos),TARGETED))	//I could either pass in target + MoveLength here, or do this local
			{															//local_TargetPos calculation inside SetupMove
				StartMove();
			}
		}
		else
		{
			AddError(CANNOT_CHANGE_TARGET,"MVA");
			return 0;
		}
	}
	else
	{
		AddError(INCOMPLETE_PARAMETER_LIST, "MVA");
		return 0;
	}
	return 1;
}

int16_t fMVR(eCmd p)
{
	enum MotionPhases TS;
	unsigned char res = 0;

	if(p.cmdData.Request)
	{
		AddError(READ_NOT_AVAILABLE_FOR_THIS_COMMAND,"MVR");
		return 0;
	}
	if(p.cmdData.ArgCheck[0])
	{
		TS = getTrajState();
		if(!CheckMove(p.cmdData.Args[0], MVR)) return 0;	//<--All limit checking is done in here.

		if( TS == CONSTANT_VELOCITY)
		{
			if( getMotionMode() == MOVE_ )
			{
				res = PositionCOF_Setup(p.cmdData.Args[0]);
				if(!res) return 0;
				else if(res == DIR_CHANGE)
				{
					StopMotion();
					while(getDriverState() != DRIVER_OFF);
					if(SetupMove(p.cmdData.Args[0],TARGETED))
					{
						StartMove();		//Did I finish this stuff?
					}
				}
			}
			else
			{
				AddError(CANNOT_START_MOVE_DURING_JOG_MODE,"MVR");
				return 0;
			}
		}
		else if( TS == NO_MOTION )
		{
			SyncMoveSet = 0;
			if(SetupMove(p.cmdData.Args[0],TARGETED))
			{
				StartMove();
			}
		}
		else
		{
			AddError(CANNOT_CHANGE_TARGET,"MVR");
			return 0;
		}
	}
	else
	{
		//Add error check for no parameter included in command
		AddError(INCOMPLETE_PARAMETER_LIST, "MVR");
		return 0;
	}
	return 1;
}

//Move Target
int16_t fMVT(eCmd p)
{
	enum MotionPhases TS;
	unsigned char res = 0;
	if(p.cmdData.ArgCheck[0])
	{
		TS = getTrajState();
		if( TS == CONSTANT_VELOCITY)
		{
			if(!CheckMove(p.cmdData.Args[0], MVT)) return 0;	//<--All limit checking is done in here.
			if( getMotionMode() == MOVE_ )
			{
				res = PositionCOF_Setup(p.cmdData.Args[0]);
				if(!res) return 0;
				else if(res == DIR_CHANGE)	//This is not consistent with how the 'position change in same direction' works
				{							//In that case, target  is adjusted by the relative position. In this case,
					StopMotion();			//the current position is adjusted to get the new target. It's like starting
					while(getDriverState() != DRIVER_OFF);		//a new move. Thought: TCR, TCA commands for adjusting target.
					if(SetupMove(p.cmdData.Args[0],TARGETED))	//MVR and MVA should just do a new move. MVA doesn't matter.
					{
						StartMove();		//Did I finish this stuff?
					}
				}
			}
			else
			{
				AddError(CANNOT_START_MOVE_DURING_JOG_MODE,"MVT");
				return 0;
			}
		}
		else
		{
			AddError(CANNOT_CHANGE_TARGET,"MVT");
			return 0;
		}
	}
	return 1;
}

int16_t fMXE(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r", maxError);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"MXE");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if (p.cmdData.Args[0] > 0)
		{
			maxError = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS, "MXE");
			return 0;
		}
	}
	
	return 1;
}

int16_t fNDV(eCmd p)
{
	if( p.cmdData.Request )
	{
		ReceiveSPIData(OPCODE_PARAM_READ, PAR_SLAVE_VERSION);

		//wait until we receive the data
		while (hspi2.RxXferCount > 0);

		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#v%d.%d.%d\n\r",spi_rx_data[2],spi_rx_data[1],spi_rx_data[0]);
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"VER");
			return 0;
		}
	}
	return 1;
}

int16_t fPGL(eCmd p)
{
	if(p.cmdData.Request)
	{
		//AddError(READ_NOT_AVAILABLE_FOR_THIS_COMMAND,"PGL");
		//return 0;

		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= PROGRAM_NUM_LIMIT)
			{
				p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",ProgRunTimes[p.cmdData.Args[0]-1]);
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"PGL");
				return 0;
			}
			if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
			{
				AddError(UNKNOWN_MESSAGE_ORIGIN,"PGL");
				return 0;
			}
		}
	}
	else if(p.cmdData.ArgCheck[0] && p.cmdData.ArgCheck[1])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= PROGRAM_NUM_LIMIT)
		{
			if(p.cmdData.Args[1] >= 0 && p.cmdData.Args[1] <= 65535)
			{
				ProgRunTimes[p.cmdData.Args[0] - 1] = p.cmdData.Args[1];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"PGL");
				return 0;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"PGL");
			return 0;
		}
	}
	return 1;
}

int16_t fPGM(eCmd p)
{
	if( p.cmdData.Request )
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",ProgAvail);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"PGM");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= PROGRAM_NUM_LIMIT)
		{
			if((1<<(p.cmdData.Args[0]-1)) & ProgAvail)
			{
				AddError(PROGRAM_ALREADY_EXISTS,"PGM");
				return 0;
			}
			SetupProgramRecord((unsigned char) p.cmdData.Args[0]);
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"PGM");
			return 0;
		}
	}
	return 1;
}

int16_t fPGS(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r", StartupPGM);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"PGS");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= PROGRAM_NUM_LIMIT)
		{
			StartupPGM = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"PGS");
			return 0;
		}
	}
	return 1;
}

//==============Update to new format
int16_t fPID(eCmd p)
{
	int32_t KpTemp=0, KiTemp=0, KdTemp=0;

	if(p.cmdData.Request)
	{
		convert_2(KpWhole2,KpFrac2,format_buf,3);
		convert_2(KiWhole,KiFrac,format_buf2,3);
		convert_2(KdWhole,KdFrac,format_buf3,3);
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%s,%s,%s\n\r", format_buf, format_buf2, format_buf3);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"PID");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= 100000000)
			{
				KpTemp = p.cmdData.Args[0] ? (10000000 / p.cmdData.Args[0]) : (0);
				Kpglob = KpTemp;
				KpWhole2 = p.cmdData.Args[0] / 1000000;
				KpFrac2 = (p.cmdData.Args[0] / 1000) % 1000;
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"PID");
				return 0;
			}
		}

		if(p.cmdData.ArgCheck[1])
		{
			if(p.cmdData.Args[1] >= 0 && p.cmdData.Args[1] <= 100000000)
			{
				KiTemp = p.cmdData.Args[1] ? (10000000000 / p.cmdData.Args[1]) : (0);
				Kiglob = KiTemp;
				KiWhole = p.cmdData.Args[1] / 1000000;
				KiFrac = (p.cmdData.Args[1] / 1000) % 1000;
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"PID");
				return 0;
			}
		}

		if(p.cmdData.ArgCheck[2])
		{
			if(p.cmdData.Args[2] >= 0 && p.cmdData.Args[2] <= 100000000)
			{
				KdTemp = p.cmdData.Args[2] ? (10000000 / p.cmdData.Args[2]) : (0);
				Kdglob = KdTemp;
				KdWhole = p.cmdData.Args[2] / 1000000;
				KdFrac = (p.cmdData.Args[2] / 1000) % 1000;
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"PID");
				return 0;
			}
		}
		SetupPID(Kpglob, Kiglob, Kdglob);
	}
	return 1;
}

int16_t fPCD(eCmd p)
{
/*	uint32_t TrL;
	int32_t InL, InL_prev=0;

	if(p.cmdData.Request)
	{
		data_point = 0;

		if(CaptureComplete())
		{
			InitCaptureRead();
			while(getNextPoint(&TPos,&RPos,&TrL,&InL))
			{
				if(InL > 8000 || InL < -8000)InL = InL_prev;
				if(TrL > 3650) TrL = 3650;
				TrL = (TrL * 4095) / 3650;		//Scale this to use the full 12-bits. The max value here(I'm using 3650) could vary between boards depending on resistor values.
				//Print and Format Line By Line
				if(LastPoint())
				{
					p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d  %lld  %lld\n\r",data_point+1,TPos/1000,RPos/1000)+1;
				}
				else
				{
					p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d  %lld  %lld\n",data_point+1,TPos/1000,RPos/1000)+1;
				}
				RouteOutput(p.rInfo);
				data_point++;
				InL_prev = InL;
			}

			EraseTraceRegion();
			CaptureCleanUp();
		}
		else
		{
			AddError(CAPTURE_DID_NOT_COMPLETE,"PCD");
			return 0;
		}
	}*/
	return 1;
}

int16_t fPOS(eCmd p)
{
	signed char sign1=1, sign2=1;
	long long temp1, RP_Freeze, CP_Freeze, RP_nm, CP_nm;
	int  mm = 0, um = 0, nm = 0;
	int  mm2 = 0, um2 = 0, nm2 = 0;

	char temp_buf[50];

	if(p.cmdData.Request)
	{
		//In picometers here.
		RP_Freeze = RealPosition;
		CP_Freeze = CurrentPos;

		if(CP_Freeze < 0)
		{
			sign1 = -1;
			CP_Freeze = -CP_Freeze;
		}
		if(RP_Freeze < 0)
		{
			sign2 = -1;
			RP_Freeze = -RP_Freeze;
		}

		//
		RP_nm = RP_Freeze/1000;
		CP_nm = CP_Freeze/1000;

		mm= CP_nm / 1000000;
		temp1 = (long long)mm * 1000000;
		um = (CP_nm - temp1)/1000;
		nm = (CP_nm - temp1 - um*(long)1000);
		//pm = CP_Freeze % 1000;
//		if(sign1 == -1) mm = -mm;

		mm2 = RP_nm / 1000000;
		//temp1 = sign2 * mm2 * 2000000;
		temp1 = (long long)mm2 * 1000000;
		um2 = (RP_nm - temp1)/1000;
		nm2 = (RP_nm - temp1 - um2*(long)1000);

//		if(PicoEnable)
//		{
//			convert_4(mm,um,nm,pm,format_buf);
//			convert_4(mm2,um2,nm2,pm2,temp_buf);
//		}
//		else
//		{
			convert_3(mm,um,nm,format_buf);		//###POS == 3 sprintf calls
			convert_3(mm2,um2,nm2,temp_buf);
//		}

		if(sign1 == -1)
		{
			if(sign2 == -1)
			{
				p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%s,-%s\n\r",format_buf, temp_buf);
			}
			else
			{
				p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%s,%s\n\r",format_buf, temp_buf);
			}
		}
		else if(sign2 == -1)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s,-%s\n\r",format_buf,temp_buf);
		}
		else
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s,%s\n\r",format_buf,temp_buf);
		}

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"POS");
			return 0;
		}
	}
	return 1;
}

int16_t fPRT(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",ProgOutputRoute);
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"PRT");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= 3)
		{
			ProgOutputRoute = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"PRT");
			return 0;
		}
	}
	return 1;
}

int16_t fQFT(eCmd p)
{
	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"QFT");
		return 0;
	}

	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r",QEI_FilterLevel);
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"QFT");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 255)
			{
				QEI_FilterLevel = p.cmdData.Args[0];
//--TODO Port this
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"QFT");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fREZ(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%ld\n\r", DAC_Res );	//Add 1 for \0 character;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"REZ");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] < MAX_REZ)
			{
				DAC_Res = p.cmdData.Args[0];	//Steps per micron.
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"REZ");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fRUN(eCmd p)
{
	if(SyncMoveSet)
	{
		StartMove();
		SyncMoveSet = 0;
	}

	return 1;
}

int16_t fSAV(eCmd p)
{
	WriteParameters(BASE_PARAM_ADDRESS);
	return 1;
}

int16_t fSDF(eCmd p)
{
	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"SDF");
		return 0;
	}
	SetupDefaults();
	return 1;
}

int16_t fSMC(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data, "#%d\n\r", SyncAfterMoveDone);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"SMC");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		SyncAfterMoveDone = p.cmdData.Args[0];
	}

	return 1;
}

//Not needed with nanodrive hardware
int16_t fSPI(eCmd p)
{
/*	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r", SPI_Function) + 1;

		if(!RouteOutput(p.rInfo))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"SPI");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0]<=3)
		{
			SPI_Function = p.cmdData.Args[0];
			SPI0_Set(SPI_Function);
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"SPI");
			return 0;
		}
	}*/
	return 1;
}

int16_t fSPS(eCmd p)
{
	if( p.cmdData.ArgCheck[0])
	{
		if(CheckPos(p.cmdData.Args[0],SPS))
		{
			RealPositionCounter = p.cmdData.Args[0] / EncoderResolution;
			RealPosition = p.cmdData.Args[0];
			CurrentPos = p.cmdData.Args[0];
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"SPS");
			return 0;
		}
	}
	return 1;
}

int16_t fSTA(eCmd p)
{
	uint8_t status_byte=0, stat_arg;
	if(p.cmdData.Request)
	{
		if(p.cmdData.ArgCheck[0])	//Let people request individual bits?
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 8)
			{
				stat_arg = p.cmdData.Args[0];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"STA");
				return 0;
			}
		}
		else
		{
			stat_arg = 0;
		}

		status_byte = GetStatus(stat_arg);

		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r", status_byte);	//Add 1 for \0 character;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"STA");
			return 0;
		}
	}
	return 1;
}

int16_t fSTP(eCmd p)		//Can we start again?
{
	if(ProgramRunning())
	{
		EndProgram();
	}

	if(HomeInProgress())
	{
		CancelHome();	//Cancel Home does not stop the current move. It just turns off the home controller, and restores state.
	}

	if(MotionInProgress())
	{
		StopMotion();
	}

	if(FeedbackMode == CLOSED_LOOP || FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
	{
		while(getDriverState() != DRIVER_OFF);
		CurrentPos = RealPosition;
	}
	return 1;
}

//Debug command, add in if useful
int16_t fSUP(eCmd p)
{
	X_bkv = 28;
	if(p.cmdData.ArgCheck[0])
	{
//		StepUpdate(p.cmdData.Args[0]/1000);
	}
	return 1;
}

int16_t fSVP(eCmd p)
{
	long long SP_Freeze, SP_nm, temp1;
	signed char sign1=1;
	int  mm = 0, um = 0, nm = 0;

	if(p.cmdData.Request)
	{
		//In picometers here.
		SP_Freeze = StartupPos;

		if(SP_Freeze < 0)
		{
			sign1 = -1;
			SP_Freeze = -SP_Freeze;
		}

		SP_nm = SP_Freeze/1000;

		mm= SP_nm / 1000000;
		temp1 = (long long)mm * 1000000;
		um = (SP_nm - temp1)/1000;
		nm = (SP_nm - temp1 - um*(long)1000);
		//pm = SP_Freeze % 1000;

		convert_3(mm,um,nm,format_buf);

		if(sign1 == -1)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d,-%s\n\r",StartupPosEnable, format_buf);
		}
		else p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d, %s\n\r",StartupPosEnable, format_buf);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"SVP");
			return 0;
		}
	}
	else
	{

		if (p.cmdData.ArgCheck[1])
		{
			if(CheckPos(p.cmdData.Args[1],SVP))
			{
				StartupPos = p.cmdData.Args[1];
				WriteParameters(BASE_PARAM_ADDRESS);
			}
			else
			{
				return 0;
			}
		}
		else
		{
			if(CheckPos(CurrentPos,SVP))
			{
				StartupPos = CurrentPos;
				WriteParameters(BASE_PARAM_ADDRESS);
			}
			else return 0;
		}

		if(p.cmdData.ArgCheck[0]) StartupPosEnable = p.cmdData.Args[0] & 0x1;
	}

	return 1;
}

int16_t fSYN(eCmd p)
{
	SyncReceived();
	return 1;
}

//Could update this to send commands to M0 on driver board if necessary
int16_t fTEC(eCmd p)
{
	int32_t temp_data;

	if(p.cmdData.Request){
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d,%d,%d\n\r",waveformFETOnTime,waveformAmpDisable,waveformAmpEnable)+ 1;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"TEC");
			return 0;
		}
	}
	else
	{
		if(FeedbackMode != OPEN_LOOP || EncoderChannelSelect == ABSOLUTE)
		{
			AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS,"TEC");
			return 0;
		}
		else if(getTrajState() != NO_MOTION)
		{
			AddError(COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION,"TEC");
			return 0;
		}
		if(p.cmdData.ArgCheck[0])
		{
			//range for parameter 1 to 0-100 microseconds
			if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= 100)
			{
				waveformFETOnTime = p.cmdData.Args[0];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"TEC");
				return 0;
			}
		}
		if(p.cmdData.ArgCheck[1]){
			//range for parameter 2 to 0-100 microseconds
			if(p.cmdData.Args[1] >= 0 && p.cmdData.Args[1] <= 100){
				waveformAmpDisable = p.cmdData.Args[1];
			}
			else{
				AddError(PARAMETER_OUT_OF_BOUNDS,"TEC");
				return 0;
			}
		}
		if(p.cmdData.ArgCheck[2]){
			//range for parameter 3 to 0-100 microseconds
			if(p.cmdData.Args[2] >= 0 && p.cmdData.Args[2] <= 100){
				waveformAmpEnable = p.cmdData.Args[2];
			}
			else {
				AddError(PARAMETER_OUT_OF_BOUNDS,"TEC");
				return 0;
			}
		}
		temp_data = waveformFETOnTime & 0xFF;
		temp_data |= ( (waveformAmpDisable << 8) & 0xFF00);
		temp_data |= ( (waveformAmpEnable << 16) & 0xFF0000);
		SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_TRANSITION_DATA, temp_data);

	}

	return 1;
}

int16_t fTRG(eCmd p)
{
	signed char sign1=1;
	long long temp1, TP_Freeze, TP_nm;
	int  mm = 0, um = 0, nm = 0;
	if(p.cmdData.Request)
	{
		TP_Freeze = getTargetPos();

		if(TP_Freeze < 0)
		{
			sign1 = -1;
			TP_Freeze = -TP_Freeze;
		}

		TP_nm = TP_Freeze/1000;

		mm= TP_nm / 1000000;
		temp1 = (long long)mm * 1000000;
		um = (TP_nm - temp1)/1000;
		nm = (TP_nm - temp1 - um*(long)1000);
		//pm = TP_Freeze % 1000;

		convert_3(mm,um,nm,format_buf);		//###POS == 3 sprintf calls

		if(sign1 == -1)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%s\n\r",format_buf);
		}
		else
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n\r",format_buf);
		}

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"TGT");
			return 0;
		}
	}
	return 1;
}

int16_t fTLN(eCmd p)
{
	signed char sign1=1;
	long long temp1, tln_nm;
	int  mm = 0, um = 0, nm = 0;
	if(p.cmdData.Request)
	{
		tln_nm = tln/1000;	//not looking at picometers here.
		if(tln_nm < 0)
		{
			sign1 = -1;
			tln_nm = -tln_nm;
		}
		mm = tln_nm / 1000000;
		temp1 = (long long)mm * 1000000;
		um = (tln_nm - temp1)/1000;
		nm = (tln_nm - temp1 - um * (long)1000);
		convert_3(mm,um,nm,format_buf);		//##2 sprintf calls. Can it be reduced to 1?

		if(sign1 == -1)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%s\n\r",format_buf);
		}
		else
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n\r",format_buf);	//Add 1 for \0 character;
		}

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"TLN");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] >= SYS_TLN && p.cmdData.Args[0] < tlp)
			{
				if(CheckNewLimit(tlp,p.cmdData.Args[0]))
				{
					tln = p.cmdData.Args[0];
				}
				else
				{
					AddError(CURRENT_POSITION_OUTSIDE_SOFT_LIMITS,"TLN");
				}
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"TLN");
				return 0;
			}
		}
		else if(CurrentPos < tlp)
		{
			tln = CurrentPos;
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"TLP");
			return 0;
		}
	}
	return 1;
}

int16_t fTLP(eCmd p)
{
	signed char sign1=1;
	long long temp1, tlp_nm;
	int  mm=0, um = 0, nm = 0;
	if(p.cmdData.Request)
	{
		tlp_nm = tlp/1000;
		if(tlp_nm < 0)
		{
			sign1 = -1;
			tlp_nm = -tlp_nm;
		}

		mm = tlp_nm / 1000000;		//mm will hold the meter and mm places.
		temp1 = (long long)mm * 1000000;
		um = (tlp_nm - temp1)/1000;
		nm = (tlp_nm - temp1 - um * (long)1000);

		convert_3(mm,um,nm,format_buf);
		if( sign1 == -1)
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#-%s\n\r",format_buf);
		}
		else
		{
			p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%s\n\r",format_buf);	//Add 1 for \0 character;
		}

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"TLP");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > tln && p.cmdData.Args[0] <= SYS_TLP)
			{
				if(CheckNewLimit(p.cmdData.Args[0],tln))
				{
					tlp = p.cmdData.Args[0];
				}
				else
				{
					AddError(CURRENT_POSITION_OUTSIDE_SOFT_LIMITS,"TLP");
					return 0;
				}
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"TLP");
				return 0;
			}
		}
		else if(CurrentPos > tln)	//TLP with no argument sets tlp to current position
		{
				tlp = CurrentPos;
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"TLP");
			return 0;
		}
	}
	return 1;
}

int16_t fTRA(eCmd p)
{
	unsigned int samplePoints = 0;
	unsigned int servosPerSample = 0;
	long long traceStartPos = 0;

	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data,"#%d\n\r", getPointsTaken()) + 1; //Add 1 for \0 character;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"TRA");
			return 0;
		}
	}
	else
	{
		if(traceOn == 1)
		{
			AddError(TRACE_ALREADY_IN_PROGRESS,"TRA");
			return 0;
		}
		//Sample Frequency - Do this first, because the # of points that can be sampled is directly related to the sample frequency
		if(p.cmdData.ArgCheck[1])
		{
			if(p.cmdData.Args[1] > 0 && p.cmdData.Args[1] <= 1000)
			{
				servosPerSample = p.cmdData.Args[1];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"TRA");
				return 0;
			}
		}
		else
		{
			servosPerSample = 1; //default servo cycles per sample
		}

		//Number of Samples
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= MAX_TRACE_SAMPLES)
			{
				samplePoints = p.cmdData.Args[0];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"TRA");
				return 0;
			}
		}
		else
		{//default # of samples
			samplePoints = 1000;	//###Make this appropriate when this is done.
		}

		//Position to start the trace at.
		if(p.cmdData.ArgCheck[2])
		{
			traceStartPos = p.cmdData.Args[2];
		}
		else traceStartPos = CurrentPos;	//else start right away

		//InitializeDMA();
		SetupTrace(samplePoints, servosPerSample, traceStartPos);
	}
	return 1;
}

int16_t fVEL(eCmd p)
{
	unsigned int  mm = 0, um = 0;
	enum MotionPhases TS;
	unsigned long vel = Vel / 1000;
	if(p.cmdData.Request)
	{
		mm = vel / 1000;
		um = (vel - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um,(char*)p.rInfo.Data,0);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"VEL");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= VelMax)
			{
				TS = getTrajState();
				if( TS == CONSTANT_VELOCITY && getMotionMode() == MOVE_)
				{
					if(!VelocityCOF_Setup(p.cmdData.Args[0])) return 0;	//Where does the error get added here?
					Vel = p.cmdData.Args[0];
					TestMinDistance = Vel * 5;
				}
				else if( TS == NO_MOTION || getMotionMode() == JOG_ || (SyncMoveSet && !MotionInProgress()))
				{
					Vel = p.cmdData.Args[0];
					TestMinDistance = Vel * 5;
					SyncMoveSet = 0;
				}
				else
				{
					//Velocity cannot be changed during this phase of motion. Add Error.
					return 0;
				}
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"VEL");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fVER(eCmd p)
{
	if( p.cmdData.Request )
	{
		p.rInfo.Length = sprintf( (char*)p.rInfo.Data, ND_CPU_VERSION );	//Add 1 for \0 character;

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"VER");
			return 0;
		}
	}
	return 1;
}

int16_t fVMX(eCmd p)
{
	unsigned int  mm = 0, um = 0;

	unsigned long vel = VelMax / 1000;
	if(p.cmdData.Request)
	{
		mm = vel / 1000;
		um = (vel - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um,(char*)p.rInfo.Data,0);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"VMX");
			return 0;
		}
	}
	else
	{
		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= SYSTEM_MAX_VEL)
			{
				VelMax = p.cmdData.Args[0];
			}
			else
			{
				AddError(PARAMETER_OUT_OF_BOUNDS,"VMX");
				return 0;
			}
		}
	}
	return 1;
}

int16_t fVRM(eCmd p)
{
	int8_t spi_status = 0;
	
	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"VRM");
		return 0;
	}
	
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",voltageRefMode);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"VRM");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if (!MotorActive)
		{
			voltageRefMode = p.cmdData.Args[0];
			if (motorControlMode == MOTOR_BLDC_MODE)
			{
				SPI2_NSS_ON;
				HAL_Delay(100);
				spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_VOLTAGE_REF, (int32_t)voltageRefMode, 0, 0);
				
				if (spi_status == 0)
				{
					AddError(PARAMETER_TRANSFER_FAILURE, "VRM");
					return 0;
				}
			}
			else
			{
				AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS, "VRM");
				return 0;
			}
		}
		else
		{
			AddError(MOTOR_IS_ENABLED,"VRM");
			return 0;
		}
	}
	return 1;
}

long long EncoderVel = 0;
int16_t fVRT(eCmd p)
{
	unsigned int  mm = 0, um = 0;
	//Calculate real velocity in um/s

	EncoderVel = (EncoderLogSum * (long)EncoderResolution)/1000; //(counts) * (pm/count) * (1nm/1000pm) = nm travelled during detection window
	EncoderVel = (EncoderVel * SERVO_FREQ_DIV_1K) / VRT_SamplePeriod;			  //[(nm) * (ServoRate / VrtPeriod) * (1um / 1000)]
																				  //Simplified: nm * (ServoRate/1000) / VrtPeriod
	if( EncoderVel < 0 ) EncoderVel = -EncoderVel;								  //					[constant]
//	unsigned long vel = EncoderVel / 1000;
	if(p.cmdData.Request)
	{
		mm = EncoderVel / 1000;
		um = (EncoderVel - (mm * 1000));
		p.rInfo.Length = convert_2(mm,um,(char*)p.rInfo.Data,0);

		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"VRT");
			return 0;
		}
	}
	else if( p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= 25)
		{
			VRT_SamplePeriod = p.cmdData.Args[0]*10;	//Servo is currently 20kHz. 20 servos per millisecond
			ClearVRTLog();
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"VRT");
			return 0;
		}
	}
	return 1;
}

int16_t fWDF(eCmd p)
{
	if(CommandLock)
	{
		AddError(INVALID_COMMAND,"WDF");
		return 0;
	}
	WriteParameters(DEFAULT_PARAM_ADDRESS);

	return 1;
}

int16_t fWFA(eCmd p)
{
	int32_t temp_data;

	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",waveformAmplitude);
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"WFA");
			return 0;
		}
	}
	else
	{
		if(FeedbackMode != OPEN_LOOP || EncoderChannelSelect == ABSOLUTE)
		{
			AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS,"WFA");
			return 0;
		}
		else if(getTrajState() != NO_MOTION)
		{
			AddError(COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION,"WFA");
			return 0;
		}

		if(p.cmdData.ArgCheck[0])
		{
			if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= 100)
			{
				waveformAmplitude = p.cmdData.Args[0];
				//TODO --> send waveform amplitude data
				//we need to recalculate the waveform offsets due to change in amplitude setting
			}
		}

		temp_data = waveformAmplitude & 0xFF;
		SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_WF_AMPLITUDE, temp_data);
	}
	return 1;
}

int16_t fWFO(eCmd p)
{

	int32_t temp_data;

	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d,%d,%d\n\r",waveformOffset_P2,waveformOffset_P3,waveformOffset_P4) + 1;
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"WFO");
			return 0;
		}
	}
	else{
		if(FeedbackMode != OPEN_LOOP || EncoderChannelSelect == ABSOLUTE){
			AddError(COMMAND_NOT_ALLOWED_WITH_CURRENT_SETTINGS,"WFO");
			return 0;
		}
		else if(getTrajState() != NO_MOTION)
		{
			AddError(COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION,"WFO");
			return 0;
		}

		if(p.cmdData.ArgCheck[0]){
			if(p.cmdData.Args[0] >= 0 && p.cmdData.Args[0] <= 100){
				//send waveform data via SPI
				waveformOffset_P2 = p.cmdData.Args[0];
			}
			else{
				AddError(PARAMETER_OUT_OF_BOUNDS,"WFO");
				return 0;
			}
		}

		if(p.cmdData.ArgCheck[1]){
			if(p.cmdData.Args[1] >= 0 && p.cmdData.Args[1] <= 100){
				//send waveform data via SPI
				waveformOffset_P3 = p.cmdData.Args[1];
			}
			else{
				AddError(PARAMETER_OUT_OF_BOUNDS,"WFO");
				return 0;
			}

		}

		if(p.cmdData.ArgCheck[2]){
			if(p.cmdData.Args[2] >= 0 && p.cmdData.Args[2] <= 100){
				//send waveform data via SPI
				waveformOffset_P4 = p.cmdData.Args[2];
			}
			else{
				AddError(PARAMETER_OUT_OF_BOUNDS,"WFO");
				return 0;
			}
		}

		temp_data = waveformOffset_P2 & 0xFF;
		temp_data |= ( (waveformOffset_P3 << 8) & 0xFF00);
		temp_data |= ( (waveformOffset_P4 << 16) & 0xFF0000);
		SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_WF_OFFSET, temp_data);
	}
	return 1;
}

//No digital i/o on nanodrive atm
int16_t fWIO(eCmd p)
{
/*	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%ld\n\r",WaitForIO_Active()) + 1;
		if(!RouteOutput(p.rInfo))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"WIO");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0] && p.cmdData.ArgCheck[1])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] <= IO_NUM_EXIST)
		{
			//Specified DIO needs to be set as an input.
			SetupWIO((unsigned char)p.cmdData.Args[0],(unsigned char)p.cmdData.Args[1]);
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"WIO");
			return 0;
		}
	}*/
	return 1;
}

int16_t fWST(eCmd p)
{
	if(p.cmdData.Request)
	{
		//Question. Question.
	}
	else
	{
		if(p.rInfo.AxisOrigin == CMD_ORIGIN_PROGRAM)
		{
			SetupWST();
		}
		else
		{
			AddError(COMMAND_ONLY_VALID_WITHIN_PROGRAM,"WST");
			return 0;
		}
	}
	return 1;
}

int16_t fWSY(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",WaitingForSync());
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"WSY");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0 && p.cmdData.Args[0] < 256)
		{
			if(p.rInfo.AxisOrigin == CMD_ORIGIN_PROGRAM)
			{
				SetupWSY((unsigned char)p.cmdData.Args[0]);
			}
			else
			{
				AddError(COMMAND_ONLY_VALID_WITHIN_PROGRAM,"WSY");
				return 0;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"WSY");
			return 0;
		}
	}
	return 1;
}

int16_t fWTM(eCmd p)
{
	if(p.cmdData.Request)
	{
		p.rInfo.Length = sprintf((char*)p.rInfo.Data,"#%d\n\r",WaitForTimeActive());
		if(!RouteOutput(p.rInfo.Data, p.rInfo.Length, p.rInfo.AxisOrigin))
		{
			AddError(UNKNOWN_MESSAGE_ORIGIN,"WTM");
			return 0;
		}
	}
	else if(p.cmdData.ArgCheck[0])
	{
		if(p.cmdData.Args[0] > 0)
		{
			if(p.rInfo.AxisOrigin == CMD_ORIGIN_PROGRAM)
			{
				SetWTM_Ticks(p.cmdData.Args[0]);	//Time is kept track of inside of the servo, towards the top.
			}
			else
			{
				AddError(COMMAND_ONLY_VALID_WITHIN_PROGRAM,"WTM");
				return 0;
			}
		}
		else
		{
			AddError(PARAMETER_OUT_OF_BOUNDS,"WTM");
			return 0;
		}
	}
	return 1;
}

int16_t fZRO(eCmd p)
{
	if(CheckPos(0,ZRO))	//<<--CheckPos will add the appropriate error.
	{
		if(EncoderChannelSelect == ABSOLUTE){
			absoluteEncoderOffset = GetAbsEncoderReading();	//This is not technically correct. Offset ='s value read from encoder head.
			WriteMiscParam(ProgAvail, absoluteEncoderOffset);
		}

		//New_ServoPosition = 0;
		RealPositionCounter = 0;
		PrevTheorCounts = 0;
		CurrentPos = 0;
		spiPacketCounter = 0;
	}
	else return 0;

	return 1;
}

char RouteOutput(uint8_t* data, int16_t length, uint8_t origin)
{
	if(origin == CMD_ORIGIN_PROGRAM)
	{
//		WhereDoIGo.AxisOrigin = ProgOutputRoute;
//		WhereDoIGo.sGate = ProgramHostGateNum;	//Route back to the axis that the EXC command came from.
//		WhereDoIGo.sAxis = ProgramHostAxisNum;
	}

	if(origin == CMD_ORIGIN_CAN1)
	{
/*		WhereDoIGo.tGate = WhereDoIGo.sGate;
		WhereDoIGo.tAxis = WhereDoIGo.sAxis;
		WhereDoIGo.sGate = getGate();
		WhereDoIGo.sAxis = getAxis();
		WhereDoIGo.CommandType = STANDARD_COMMAND;

		while(CanTxBufFull())
		{
			//We cannot add this at this time.
			procTxStateCAN1();
		}

		sendCanCmd(&WhereDoIGo);

		procTxStateCAN1();*/
	}
	else if(origin == CMD_ORIGIN_USART0)
	{
//		UART0_Puts((char*)WhereDoIGo.Data);
	}
	else if(origin == CMD_ORIGIN_USART1)
	{
		//TODO --> evaluate this technique. Need to prevent the UART TX buffer from being overwritten before transfer is complete. should be better way to handle this than a DMA/blocking scheme
		RS485_TRANSMIT;
		huart1.TxInProgress = 1;
		HAL_UART_Transmit_DMA(&huart1, data, length);
		
		while (huart1.TxInProgress);
	}
	else if(origin == CMD_ORIGIN_USART2)
	{
//		UART2_Puts((char*)WhereDoIGo.Data);
	}
	else
	{
		return 0;
	}
	return 1;
}

void GetFormatted_Vel(uint64_t vel, sRouteInfo* RInfo){
	uint16_t  mm = 0, um = 0;
	uint32_t vel_nm;
	vel_nm = vel / 1000;
	mm = vel_nm / 1000;
	um = (vel_nm - (mm * 1000));
	RInfo->Length = convert_2(mm,um,(char*)RInfo->Data,0) + 1;
}

uint32_t GetStepsPerServo(uint64_t vel_nm, uint32_t rez, uint32_t servo_rate){
	//Formula===
	//Velocity(nm/s)  *  1/SERVO_RATE(s / SERVO) * DacResolution(steps/um) * UnitConv(1um / 1000nm) = n (steps/servo)
	//___Shortcuts:
								//No loss of precision with this division. user cannot set nm place in velocity.
	uint64_t steps = (vel_nm / 1000) * rez;
	steps /= servo_rate;	//broken up into two lines for debugging purposes
	return ((uint32_t)steps);
}

uint8_t CheckEligibility(uint8_t Origin, uint16_t Instr, uint16_t *ErrorCode, uint8_t query)
{
	enum MotionPhases TS = getTrajState();
	uint8_t ProgR = ProgramRunning();

//=======External Commands allowed while a motion program is running.
	if(ProgR && Origin != CMD_ORIGIN_PROGRAM)
	{
		if(query)
		{
			return 1;
		}
		else if(Instr == EST || Instr == STP || Instr == SYN)
		{
			return 1;
		}
		else
		{
			*ErrorCode = COMMAND_NOT_ALLOWED_DURING_PROGRAM;
			return 0;
		}
	}

	switch(Instr)
	{
		case RST:
			break;
		case VRT:
		case FMR:
			return 1;
			break;
		case RUN:
			if(!MotorActive)
			{
				*ErrorCode = MOTOR_IS_DISABLED;
				return 0;
			}
			else if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			break;
		case AEZ:
		case AMX:
		case ANR:
		case ACC:
		case CAP:
		case CER:
		case CFQ:
		case CFS:
		case CVL:
		case CZO:
		case CYM:
//		case DAC:
		case DBD:
		case DBG:
		case DEC:
		case DEF:
		case EAD:
		case EDV:
		case EFF:
		case ENC:
		case END:
		case ENR:
		case EPL:
		case ERA:
		case EXC:
		case FBK:
		case FFP:
		case HAC:
		case HCG:
		case HDX:
		case HSC:
		case HST:
		case HVL:
		case IDX:
		case INF:
		case INP:
		case JAC:
		case LCG:
		case LDP:
		case LDR:
		case LIM:
		case LPL:
		case LST:
		case MCR:
		case MND:
		case MOT:
		case MPL:
		case MTL:
		case MTC:
		case MTP:
		case MTR:
		case MXE:
		case PCD:
		case PGL:
		case PGM:
		case PGS:
		case PID:
		case PRT:
		case QFT:
		case REZ:
		case SAV:
		case SDF:
		case SMC:
		case SPI:
		case SPS:
		case STA:
		case SVP:
		case TEC:
		case TRG:
		case TLN:
		case TLP:
		case TRA:
		case VMX:
		case VRM:
		case WDF:
		case WFA:
		case WFO:
		case ZRO:

			if(query) break;
			else if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			else if(TS != NO_MOTION)
			{
				*ErrorCode = COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION;
				return 0;
			}
			break;

		case LCK:		//No reason to not allow this at any time.
			break;

		case DAT:

			if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			if(TS != NO_MOTION)
			{
				*ErrorCode = COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION;
				return 0;
			}
			break;

		case VEL:
			if(query) break;

			TS = getTrajState();
			if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			if(SyncMoveSet && !MotionInProgress())
			{
				return 1;
			}
			else if(TS == TRI_ACCEL || TS == TRAP_ACCEL)
			{
				*ErrorCode = VELOCITY_CHANGE_NOT_ALLOWED_DURING_ACCEL;
				return 0;
			}
			else if(TS == DECEL || TS == TRAP_DECEL)
			{
				*ErrorCode = VELOCITY_CHANGE_NOT_ALLOWED_DURING_DECEL;
				return 0;
			}
			break;

		case JOG:

			if(query) break;
			if(!(MotorActive))
			{
				*ErrorCode = MOTOR_IS_DISABLED;
				return 0;
			}
			if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			TS = getTrajState();
			if(SyncMoveSet && !MotionInProgress())
			{
				return 1;
			}
			else if(TS == TRI_ACCEL || TS == TRAP_ACCEL)
			{
				*ErrorCode = VELOCITY_CHANGE_NOT_ALLOWED_DURING_ACCEL;
				return 0;
			}
			else if(TS == DECEL || TS == TRAP_DECEL)
			{
				*ErrorCode = VELOCITY_CHANGE_NOT_ALLOWED_DURING_DECEL;
				return 0;
			}

			break;

		case HOM:
			if(query) break;
			if(!(MotorActive))
			{
				*ErrorCode = MOTOR_IS_DISABLED;
				return 0;
			}
			if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			if(getTrajState() != NO_MOTION)
			{
				*ErrorCode = COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION;
				return 0;
			}
			break;

		case STP:	//Always let stop through

			break;

		case EST:	//Always let e-stop through

			break;

		case ERR:	//Query only.

			break;

		case WIO:
		case WST:
		case WSY:
		case WTM:
		case SYN:
		case IWL:
		case IST:
		case DRL:
		case DST:
			break;

		case MLN:
		case MLP:
		case MSA:
		case MSR:

			if(!(MotorActive))
			{
				*ErrorCode = MOTOR_IS_DISABLED;
				return 0;
			}
			if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			if(TS != NO_MOTION && !query)
			{
				*ErrorCode = COMMAND_CANNOT_BE_EXECUTED_DURING_MOTION;
				return 0;
			}
			break;

		case MVT:
			if(!(MotorActive))
			{
				*ErrorCode = MOTOR_IS_DISABLED;
				return 0;
			}
			if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			else if(TS == TRAP_ACCEL || TS == TRI_ACCEL)
			{
				*ErrorCode = TARGET_CHANGE_CANNOT_OCCUR_DURING_ACCEL_PHASE;
				return 0;
			}
			else if(TS == TRAP_DECEL || TS == DECEL)
			{
				*ErrorCode = TARGET_CHANGE_CANNOT_OCCUR_DURING_ACCEL_PHASE;
				return 0;
			}
			break;

		case MVA:
		case MVR:

			if(!(MotorActive))
			{
				*ErrorCode = MOTOR_IS_DISABLED;
				return 0;
			}
			if(HomeInProgress())
			{
				*ErrorCode = HOME_IN_PROGRESS;
				return 0;
			}
			if(SyncMoveSet & !MotionInProgress())
			{
				//A sync move has been set up but has not been initiated. Allow it to be overruled.
				SyncMoveSet = 0;
				setTrajState(NO_MOTION);
				return 1;
			}
			else if( TS == TRAP_ACCEL || TS == TRI_ACCEL)
			{
				*ErrorCode = TARGET_CHANGE_CANNOT_OCCUR_DURING_ACCEL_PHASE;
				return 0;
			}
			else if( TS == TRAP_DECEL || TS == DECEL || TS == STOP)
			{
				*ErrorCode = TARGET_CHANGE_CANNOT_OCCUR_DURING_DECEL_PHASE;
				return 0;
			}

			break;

		case POS:
			break;

		case CLT:
			break;

		case VER:
		case NDV:
			break;


		default:
		{
			//Not Handled.
			*ErrorCode = INCOMPLETE_STATE_VALIDATION_FUNCTION;
			return 0;
		}
	}
	return 1;
}

//--TODO Port this
void EnableLimitDetectInterrupts(){

}

void DisableLimitDetectInterrupts(){

}

uint8_t GetStatus(uint8_t field)
{
	enum MotionPhases TS = getTrajState();
	uint8_t sta_done=0, sta_byte=0, single_val=0, temp_val=0;

	if(field == 0)
	{
		field = 7;
		single_val = 0;
	}
	else single_val = 1;

	while(!sta_done)
	{
		switch(field)
		{
		case 7:
			if(single_val)
			{
				sta_byte = CheckErrors();
				sta_done = 1;
			}
			else
			{
				sta_byte |= (CheckErrors()<<7);
				field--;
			}
			break;

		case 6:
			if(TS == TRAP_ACCEL || TS == TRI_ACCEL)
			{
				temp_val = 1;
			}
			else temp_val = 0;

			if(single_val)
			{
				sta_byte = temp_val;
				sta_done = 1;
			}
			else
			{
				sta_byte |= (temp_val<<6);
				field--;
			}
			break;

		case 5:
			if(TS == CONSTANT_VELOCITY)
			{
				temp_val = 1;
			}
			else temp_val = 0;

			if(single_val)
			{
				sta_byte = temp_val;
				sta_done = 1;
			}
			else
			{
				sta_byte |= (temp_val<<5);
				field--;
			}
			break;

		case 4:
			if(TS == TRAP_DECEL || TS == DECEL)
			{
				temp_val = 1;
			}
			else temp_val = 0;

			if(single_val)
			{
				sta_byte = temp_val;
				sta_done = 1;
			}
			else
			{
				sta_byte |= (temp_val<<4);
				field--;
			}
			break;

		case 3:
			if(InPosition)
			{
				temp_val = 1;
			}
			else temp_val = 0;

			if(single_val)
			{
				sta_byte = temp_val;
				sta_done = 1;
			}
			else
			{
				sta_byte |= (temp_val<<3);
				field--;
			}
			break;

		case 2:
			temp_val = ProgramRunning();
			if(single_val)
			{
				sta_byte = temp_val;
				sta_done = 1;
			}
			else
			{
				sta_byte |= (temp_val<<2);
				field--;
			}
			break;

		case 1:
			if(InPositiveLimit)
			{
				temp_val = 1;
			}
			else temp_val = 0;

			if(single_val)
			{
				sta_byte = temp_val;
				sta_done = 1;
			}
			else
			{
				sta_byte |= (temp_val<<1);
				field--;
			}
			break;

		case 0:
			if(InNegativeLimit)
			{
				temp_val = 1;
			}
			else temp_val = 0;

			if(single_val)
			{
				sta_byte = temp_val;
				sta_done = 1;
			}
			else
			{
				sta_byte |= temp_val;
				field--;
			}
			sta_done = 1;
			break;
		}
	}
	return sta_byte;
}

MicronixCmdDesc* CommandLookup(sCmdData *target)
{
	uint16_t ii = 0;
	while(ii < COMMAND_ARRAY_SIZE){
		if( (CommandLookupArray[ii].InstrStr[0]==target->Instr_Str[0]) &&
			(CommandLookupArray[ii].InstrStr[1]==target->Instr_Str[1]) &&
			(CommandLookupArray[ii].InstrStr[2]==target->Instr_Str[2]) )
		{
			target->Instr = CommandLookupArray[ii].cmdId;
			return &CommandLookupArray[ii];
		}
		ii++;
	}
	return &CommandLookupArray[NULL_POSITION];
}

void AxisSetup()
{
	int32_t temp_data;

	if( FeedbackMode == CLOSED_LOOP || FeedbackMode == OPEN_LOOP_CLOSED_FINISH)	//I don't get this. It's opposite of what it should be.0
	{
		DetectEncoder = 1;
	}
	else DetectEncoder = 0;


	DeadbandTimeoutCounts = (DeadbandTimeout * SERVOS_PER_MS) / 1000;
	DeadbandWindow = (DeadbandEncoderCounts * EncoderResolution) / 1000;	//nanometers b.

	InPositionTimeCounts = (InPositionTime * SERVOS_PER_MS) / 1000;
	InPositionWindow = (InPositionCounts * EncoderResolution) / 1000;

//--TODO Port Encoder Polarity
	//Switch encoder polarity to match other MMC controllers.
	if(EncoderPolarity)
	{
	}
	else
	{
	}

	//Update the start up position options. allows for the feature to be disabled
	if(StartupPosEnable && CheckPos(StartupPos,SVP))	//Error handled inside CheckPos
	{
		//RealPositionCounter = StartupPos / EncoderResolution;
		//RealPosition = StartupPos;
		CurrentPos = StartupPos;
	}
	else StartupPosEnable = 2;

	if(LimitConfig & HARD_LIMIT_MASK)
	{
		EnableLimitDetectInterrupts();
	}
	else
	{
		DisableLimitDetectInterrupts();
	}

	UpdateLimitState();

	if(VelMax < 1000 || VelMax > 50000000){
		VelMax = VMX_INIT;
	}

	if(ClLoopCorVelMax < 1000 || ClLoopCorVelMax > VelMax){
		ClLoopCorVelMax = CL_CORR_VELMAX_INIT;
	}

	if(HardStopDeadband > 100){
		HardStopDeadband = 10;
	}
	if(HardstopDetectEnable != 0 && HardstopDetectEnable != 1){
		HardstopDetectEnable = 0;
	}
	SetClStepsPerServo(GetStepsPerServo(ClLoopCorVelMax,DAC_Res, SERVO_FREQ));

	if(iTermSampleTime >= 50000){
		iTermSampleTime = 1;
	}
	if(dTermSampleTime >= 50000){
		dTermSampleTime = 1;
	}
	if(iTermSumLimit > 1000000){
		iTermSumLimit = 1000000;
	}

	if(dTermDiffLimit < 0 || dTermDiffLimit > 100000000){
		dTermDiffLimit = 250000;
	}

	Kpglob = (KpWhole2 || KpFrac2) ? (10000000 / ( (KpWhole2 * 1000000) + (KpFrac2 * 1000) )) : (0);
	Kiglob = (KiWhole || KiFrac) ? (10000000000 / ( (KiWhole * 1000000) + (KiFrac * 1000) )) : (0);
	Kdglob = (KdWhole || KiFrac) ? (10000000 / ( (KdWhole * 1000000) + (KdFrac * 1000) )) : (0);

	SetupPID(Kpglob, Kiglob, Kdglob);

//Fixed frequency mode
//!	ffmode_entry = CalculateFF_EntryPoint(FPGA_FFModeResetFreq,FPGA_FFModeThreshold,FPGA_DAC_MAX);
/*
	//All the bounds check stuff should be separate from the all the actual setup functionality
	//Different functions

//	TestFPGA_ParameterSetGet();

	if(FPGA_FFModeResetFreq < 1600 || FPGA_FFModeResetFreq > 30000){
		FPGA_FFModeResetFreq = 10000;
	}

	if(FPGA_WaveformOffset_P2 > 100){
		FPGA_WaveformOffset_P2 = 50;
	}

	if(FPGA_WaveformOffset_P3 > 100){
		FPGA_WaveformOffset_P3 = 25;
	}

	if(FPGA_WaveformOffset_P4 > 100){
		FPGA_WaveformOffset_P4 = 75;
	}

	if(FPGA_FFModeThreshold > 30000){
		FPGA_FFModeThreshold = 5000;
	}

	Setup_FFM_Vel(FPGA_FFModeThreshold * 1000);

	if(!FPGA_ParamInit(FPGA_WaveformOffset_P2,FPGA_WaveformOffset_P3,FPGA_WaveformOffset_P4,FPGA_FFModeResetFreq,ffmode_entry)){
		AddError(PARAMETER_TRANSFER_FAILURE,"CFS");
	}
	*/

	if(waveformFETOnTime > 100){
		waveformFETOnTime = 10;
	}

	if(waveformAmpDisable > 100){
		waveformAmpDisable = 2;
	}

	if(waveformAmpEnable > 100) {
		waveformAmpEnable = 3;
	}

	//only send this data if we are in piezo mode
	if (motorControlMode == MOTOR_PIEZO_MODE)
	{
		temp_data = waveformFETOnTime & 0xFF;
		temp_data |= ( (waveformAmpDisable << 8) & 0xFF00);
		temp_data |= ( (waveformAmpEnable << 16) & 0xFF0000);
		SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_TRANSITION_DATA, temp_data);
	}

	if(EncoderChannelSelect == ANALOG)
	{
		//Tell FPGA to read the Analog Channel
		ENCODER_CHANNEL_ANALOG_SELECT;
		ClearVRTLog();
		ReInitEncFilter();
		ABSOLUTE_ENCODER_DESELECT;
	}
	else if(EncoderChannelSelect == DIGITAL)
	{
		//Tell FPGA to read the Digital Channel
		ENCODER_CHANNEL_DIGITAL_SELECT;
		ClearVRTLog();
		ReInitEncFilter();
		ABSOLUTE_ENCODER_DESELECT;
	}
	else{
		EncoderChannelSelect = ABSOLUTE;
		ClearVRTLog();
		ReInitEncFilter();
		ABSOLUTE_ENCODER_SELECT;
	}
}

void InitializeDriver()
{
	int8_t spi_status;
	
	SPI2_NSS_OFF;
	while (!driverReady)
	{
		//We need to make sure the driver STM32 is ready for operation
		//Transmit a status request (STA?\r) for now. Once we receive a "#1\r" we are good to continue

		//TODO --> may improve on this process later
		HAL_UART_Transmit(&huart3, USART3_txChar, 5, 100);
		HAL_Delay(100);
		CheckFlags();
	}

	//Sync with driver ARM complete
	//TODO --> Send parameter data here
	if(motorControlMode==MOTOR_BLDC_MODE || motorControlMode==MOTOR_DC_MODE)
	{
		//send the motor control mode
		spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_TYPE, (int32_t)motorControlMode, 0, 0);
		if (spi_status == 0)
		{
			AddError(PARAMETER_TRANSFER_FAILURE, "MTC");
		}

		//send magnetic pitch data (MTP)
		spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_PITCH, (int32_t)(motorPitch*1000), 0, 0);
		if (spi_status == 0)
		{
			AddError(PARAMETER_TRANSFER_FAILURE, "MTP");
		}

		//send current parameters (MCR)
		spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_CURRENT, (int32_t)iMax, (int32_t)i2Time, iOffset);
		if (spi_status == 0)
		{
			AddError(PARAMETER_TRANSFER_FAILURE, "MCR");
		}

		//send voltage reference mode (VRM)
		spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_VOLTAGE_REF, (int32_t)voltageRefMode, 0, 0);
		if (spi_status == 0)
		{
			AddError(PARAMETER_TRANSFER_FAILURE, "VRM");
		}

		//send the motor enable/disable
		if (motorControlMode == MOTOR_BLDC_MODE) 
		{
			spi_status = SendParamSPIDataExtended(OPCODE_PARAM_WRITE, PAR_MOTOR_ENABLE, (int32_t)MotorActive, 0, 0);
			if (spi_status == 0)
			{
				AddError(PARAMETER_TRANSFER_FAILURE, "MOT");
			}
		}
  }
  else
  {
	  SendMotionSPIData(OPCODE_PARAM_WRITE, PAR_MOTOR_TYPE, (int32_t)motorControlMode);
	  //TODO --> send all the sawtooth waveform parameters
  }	
}

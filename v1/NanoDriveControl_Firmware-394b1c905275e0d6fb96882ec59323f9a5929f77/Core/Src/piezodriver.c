/*
 * PiezoDriver.c
 *
 *  Created on: May 31, 2023
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "command.h"
#include "commandlist.h"
#include "sysinfo.h"
#include "motion.h"
#include "position.h"
#include "motionspi.h"
#include "trace.h"

#define MOTION_BUF_SIZE 7		//Size of the byte array that gets written to the fpga

#define SERVO_TRAJ_RATIO 10		//The ratio of the servo frequency (10kHz) to the trajectory frequency (1kHz)

/*pid routine limits to prevent overflow*/
#define MAX_ERROR_PID 500000			//If we're using the PID routine, this is in encoder counts. If just the old-style Kp, this is in nm
#define ERROR_31_BIT_LIMIT 2147000000	//Error into the PID routine is capped to avoid 64-bit divides
#define CL_DACSTEP_PERSERVO_MAX 16000	//Hard Coded Limit on # of dac steps taken per servo in closed loop and traditional piezo mode
#define CL_DACSTEP_PERSERVO_MAX2 2047	//Hard Coded Limit on # of dac steps taken per servo in closed loop and dc mode
#define MAX_CORRECTION 20000			//Hard Coded Limit on # of dac steps taken in open loop

#define DAC_Updates_Per_10k 10		//Number of Dac updates the driver cpu does per local servo cycle

//How direction is represented in the trajectory routine
#define TRAJ_POS 1
#define TRAJ_NEG -1

#define ENC_LOG_SIZE 250				//Array Size used for real time velocity calculation
#define ByteReverse(A) (A&0xFF0000)>>16 | (A&0x0000FF)<<16 | (A&0x00FF00)	//Macro to Byte reverse a 24-bit #

/*Moving average filter on the encoder*/
#define ENCODER_AVG_BUFSIZE 512
#define ENCODER_BUFFER_MASK 0x1FF

//Trajectory
extern uint8_t MOTION_IN_PROGRESS;
extern volatile uint8_t CalcNextTrajPoint;
extern int8_t TrajDir;
extern int64_t StartPos;
extern enum MotionPhases TrajState;
extern uint8_t ESTOP_IN_PROGRESS;

volatile enum DriverStates DriverState = DRIVER_OFF,		//!<Current Driver State
							LastDriverState = DRIVER_OFF;

//How direction is communicated to the Waveform Generator (0 == pos) / (1 == neg)
volatile enum move_directions{ DRV_POS,	DRV_NEG } DriverDir;	//Current Direction of Motion

//=============Encoder Position Vars
volatile int64_t  EncoderDif = 0;			//Change in encoder counts per servo
volatile uint32_t EncoderCount = 0;			//Reading from the Quadrature Peripheral Counter (not the absolute real position reported by the controller)
volatile uint32_t PreviousCount = 0;		//Value of the previous reading from QEI counter(last servo cycle)
volatile int64_t  RealPositionCounter = 0;	//Absolute position of the controller in encoder counts
volatile int64_t  RealPosition;				//Real Position of the controller in picometers, value reported to user
volatile int64_t  OldRealPosition;			//Previous position reading (last servo cycle)
int64_t absoluteEncoderOffset = 0;			//absolute encoder offset

volatile uint8_t RunTraj = 0;				//Flag used to schedule trajectory service.
volatile uint8_t TrajCycleServoCntr = 0;	//Counter to keep track of servo # per trajectory cycle
volatile int64_t NextTrajectoryPosition = 0;//Our current target commanded by the trajectory

volatile int64_t New_ServoPosition;			//Theoretical position synced with trajectory every trajectory cycle and incremented by servo in between
											//Basically the most accurate current theoretical position of the controller

volatile int32_t  DistancePerN_Servos = 0;	//Relative Distance To Move Commanded By Trajectory where N is the servo freq to traj freq ration
volatile int32_t  DX_Per_Servo = 0;			//New target calculated by trajectory divided by ratio of servo freq to trajectory freq in pm (pm per 10k)
volatile uint32_t DX_Remainder = 0;			//Remainder tracking on DX_Per_Servo calculation used to adjust Adj_DX val in pm
volatile int32_t  Adj_DX = 0;				//Relative position increment for current servo cycle in pm
volatile int32_t  DriverInc = 0;			//Relative position increment for current servo cycle in nm
volatile int32_t  DriverIncRem = 0;			//Remainder tracking for DriverInc to prevent information loss due to rounding

volatile int32_t StepAccum = 0;				//Tracks fractions of commanded dac steps, so no steps are lost over time. when enough fractional steps add up to a whole step, it gets factored it
volatile int32_t inter_cor = 0;				//intermediate variable used for calculation

volatile int32_t DacAdjustment = 0;			//Calculated Adjustment to make to the dac this servo cycle (+ only, direction stored in a different field)
volatile int32_t SignedDacAdjustment=0;		//DacAdjustment with direction info encoded to report to trace

uint32_t UserCorrectionLimit;				//Value set by CVL, limiting the correction velocity

volatile uint16_t WholeStep = 0;			//Whole steps per fpga dac update. sent to FPGA when in piezo mode.
volatile uint16_t FracStep = 0;				//Fractional steps per fpga dac update. sent to FPGA when in piezo mode.

volatile uint32_t spiMotionData;			//Holds SPI motion data to be sent over SPI
volatile uint8_t SetupMotionBuf=0;			//Flag that updates MotionBuf contents before a transfer.
volatile uint8_t SendMotionData = 0;		//Flag to initiate SPI transfer to FPGA
volatile uint32_t spiPacketCounter = 0;		//Counter for number of SPI packets sent. For debugging purposes
extern volatile uint8_t spi_rx_data[4];

volatile uint8_t InPosition = 1;			//Flag used to indicate whether the system is currently at the commanded target or not
											//In open loop, this is 0 during a move and 1 once a move is completed
											//In closed loop, this is 1 if stationary and within the deadband and 0 during a move or outside of deadband
volatile uint32_t CorrectionAttempts = 0;		//Attempts to bring system within the deadband range in servo cycles
volatile uint8_t InDeadband = 0;				//Flag to indicate whether system is within the deadband or not
volatile int32_t DeadbandCheck;					//Depending on the MTC mode, this will be in nanometers or encoder counts
extern volatile uint32_t DeadbandTimeoutCounts;	//param from the DBD setting
extern volatile int32_t DeadbandWindow;			//param from the DBD setting

//=========PID Variables
volatile int64_t Error_pm64 = 0;			//Real Error in picometers 64-bit variable
volatile int32_t Error_pm = 0;				//Error in picometers capped at +/- 2^31,
volatile int32_t Error = 0;					//Piezo mode -- Error in nanometers
											//DC Mode    -- Error in encoder counts
volatile int32_t PrevError;					//Previous value of Error, used in calculating the d-term
volatile int32_t pTerm=0, iTerm=0, dTerm=0;	//Closed Loop Terms used for DC Mode
volatile int32_t FFTerm=0;
volatile int32_t IErrorAccum=0;
volatile int32_t PID_Term;					//pTerm + iTerm + dTerm

//TODO --> check to see if these are needed
volatile uint16_t iSampleCntr = 1;		//Used to count up to IST value
volatile uint16_t dSampleCntr = 1;		//Used to count up to DST value
volatile int32_t dTermDiff;				//Used in d-Term calculation

volatile int32_t Kp, Ki, Kd;			//User settable constants to tune PID in DC mode

//------------End of PID var section====

//TODO --> resolve no encoder detection
//=====Encoder Detection Vars
volatile uint8_t DetectEncoder = 0;		//flag to indicate whether the encoder detection feature is enabled
volatile uint8_t NoEncoder=0;			//flag to indicate that the feature has triggered a detection failure
volatile int32_t EncoderCheck;			//Accumulator of received encoder pulses during detection window
volatile uint8_t No_Encoder = 0;		//Flag used in home routine, this is kind of fucked at the moment i believed.
										//Check it out when making sure encoder detection works with absolute encoder.

//==Encoder Log Variables
volatile int32_t EncoderLogSum = 0;						//Running sum of encoder counts in the EncoderCountLog. Used for VRT encoder velocity calculation
volatile int32_t EncoderCountLog[ENC_LOG_SIZE] = {0};	//Log of delta encoder counts per servo cycle or absolute value per servo cycle depending on encoder type
volatile uint8_t enc_log = 0;							//ptr to step through encoder log

extern uint32_t VRT_SamplePeriod;				//Sample Window Size for the Encoder Velocity Calculation. Changeable with VRT

volatile enum FeedbackModes OldFeedbackMode;	//Used to detect changes to the fbk setting so any cleanup operations happen properly

volatile uint32_t WTM_Ticks = 0;				//Used to implement WTM command, where nothing is done for the requested time period
volatile uint8_t WaitForTimePeriod;				//Flag to indicate that the WTM feature is active

//=====Trace variables
volatile uint32_t startTrace=0;					//Flag to indicate that a trace is initialized and should begin once startup conditions are met
volatile uint8_t traceOn = 0;			//Flag that indicates that sampling is in progress

volatile uint32_t T_Samples=0;					//TRA parameter. Number of sample points to take.
volatile uint32_t T_ServosPerSample = 0;		//TRA parameter. Number of servo cycles per sample.
volatile int64_t  T_StartPos = 0;				//TRA parameter. Begin sampling once this position has been reached

volatile uint32_t points = 0;			//Counter used to implement the ServosPerSample param functionality

volatile uint8_t SampleFlag = 0;		//Flag that tells the trace code to take a sample this servo cycle
extern volatile uint8_t SampleTrigger;	//Flag that indicates a hardware trigger has been received and to take a sample this cycle. implemented in the IO interrupt in DIO.c
volatile uint8_t TraceErr = 0;			//Flag that currently indicates that we have taken the max possible amount of samples in hardware trigger mode.

volatile int32_t temp_Limit = 0;

volatile int32_t InPositionTimeCounts_temp = 0;
volatile uint8_t InPositionINP = 0;
extern volatile int32_t InPositionWindow;
extern volatile int32_t InPositionTimeCounts;

//Internal structure for recording a reading from a biss-based absolute encoder
typedef struct{      // __attribute__((__packed__)){
	uint32_t pos_data;
	uint8_t error;
	uint8_t warn;
	uint8_t crc;
	uint8_t extra1;
}sEncSample;

sEncSample EncReadings[40] = {0};		//Log of absolute encoder readings
volatile uint16_t encptr = 0;			//ptr for stepping though

volatile int32_t LastPos = 0;			//Used to create a delta value when using an absolute encoder.

//==Absolute Encoder Variables
volatile uint32_t abs_enc_crc_err_cntr = 0;		//counter to indicate the # of times the calculated crc value did not match the crc field in the data packet
volatile uint32_t abs_enc_err_flag_cntr = 0;	//counter to indicate the # of times the error flag was set in the received data packets
volatile uint32_t abs_enc_warn_flag_cntr = 0;	//counter to indicate the # of times the warn flag was set in the received data packets

volatile uint8_t AbsValidData = 0;				//flag to indicate whether the current reading from the absolute encoder has passed all checks and is useable

//===Encoder Filter Variables

volatile enum FilterPhases{
						ENC_FILTER_SHRINKING,		//Positive Motion
						ENC_FILTER_GROWING,			//Negative Motion
						ENC_FILTER_NORMAL
					}
					EncFilterPhase;				//Current Direction of Motion
volatile uint8_t EncFilterInit = 1;		//Flag to initialize the filter variables
volatile uint8_t EncFilterReInit = 0;	//Flag when Filter size value changes during operation
volatile int32_t FilterSize = 0;
volatile int32_t TargetFilterSize = 0;

volatile uint16_t Enc_HeadPtr = 0;		//Put new data in at the head
volatile uint16_t Enc_TailBuf1 = 0;		//oldest data point in filter window 1
volatile int64_t EncDifLog[ENCODER_AVG_BUFSIZE];	//Encoder Data Buffer

volatile uint16_t EncAvgWindow;  		//Value of the current active window size  (motion or standstill)
volatile int64_t EncDif_SumBuf1 = 0;	//Running sum for filter window 1 (standstill)
volatile int64_t EncAvg1 = 0;			//Calculated average value for filter window 1

void ZeroCorrectionAttempts(){
	CorrectionAttempts = 0;
}

unsigned char StillInsideInit = 0;
void InitializingServo(){
	StillInsideInit = 1;
}

void DoneInitializingServo(){
	StillInsideInit = 0;
}

//=========Servo timing / Diagnostic vars
uint32_t service_delay = 0;				//Reads the Interrupt Timer at the start of the servo interrupt to see the startup latency of the interrupt
uint32_t service_delay_max = 0;			//Max value of the startup latency of the servo interrupt
uint32_t ServoTimeLog[500];				//Log of execution times of servo
uint32_t servo_log_ptr = 0;				//ptr to step through array
uint32_t servo_time_max = 0;			//max time through servo
uint32_t servo_time_max_index = 0;		//location in array of max time

extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim2;		//Servo Cycle Timer
extern TIM_HandleTypeDef htim3;		//Trajectory Timer
extern TIM_HandleTypeDef htim4;		//Encoder Timer

void TIM2_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

	if( service_delay > service_delay_max )
	{
		service_delay_max = service_delay;
	}

	if(StillInsideInit){
		return;
	}

	if(traceOn)
	{
		HAL_ADC_Start(&hadc1);
	}

	if(WTM_Ticks)
	{
		WTM_Ticks--;
	}
	else
	{
		WaitForTimePeriod = 0;
	}
	//----------------------------------------------------------
	if((OldFeedbackMode == CLOSED_LOOP || OldFeedbackMode == OPEN_LOOP_CLOSED_FINISH) && FeedbackMode == OPEN_LOOP && DriverState == DRIVER_OFF)
	{
		DriverState = DRIVER_CLEANUP;
	}
	OldFeedbackMode = FeedbackMode;

	if(DriverState == DRIVER_OFF)
	{
		New_ServoPosition = CurrentPos;
		DriverInc = 0;

		//Poll Trajectory for Move Status
		if( MOTION_IN_PROGRESS )
		{
			DriverState = DRIVER_SETUP;
			CalcNextTrajPoint = 1;
			RunTraj = 1;
		}
	}
	else if(DriverState == DRIVER_SETUP)
	{
		//Wait for the trajectory to calculate our first point.
		if(!CalcNextTrajPoint)
		{
			CorrectionAttempts = 0;	//Reset Deadband Correction Attempts Counter
			InDeadband = 0;			//Move Starting, Not in Deadband
			InPosition = 0;
			InPositionINP = 0;

			//Get first destination point from trajectory. (Trajectory directly updates CurrentPos, even though physically it should take us 1ms to get there)
			NextTrajectoryPosition = CurrentPos;

			//Make sure we are synced with the trajectory starting position
			New_ServoPosition = StartPos;

			//Calculate the total distance we need to move over the next 20 servo cycles.
			DistancePerN_Servos = (NextTrajectoryPosition - StartPos);

			//What's the point of DriverDir?, It hides MotorPolarity
			//We Still use the +1/-1 scheme to calculate shit.
			if(TrajDir == TRAJ_POS)	//Put in Motor Polarity.
			{
				if(MPol == 0)
				{
					DriverDir = DRV_POS;
				}
				else DriverDir = DRV_NEG;

			}
			else
			{
				if(MPol == 0)
				{
					DriverDir = DRV_NEG;
				}
				else DriverDir = DRV_POS;
			}

			//Calculate the the amount we need to move per servo cycle
			DX_Per_Servo = DistancePerN_Servos / SERVO_TRAJ_RATIO;	//+/-

			//Calculate remainder of previous calculation so we don't lose steps.
			//This gets added in continuously.
			DX_Remainder = TrajDir*(DistancePerN_Servos - (DX_Per_Servo * SERVO_TRAJ_RATIO));	//+ only.

			DriverState = DRIVER_ON;
		}
	}
	else if(DriverState == DRIVER_ON)
	{
		if(TrajCycleServoCntr < (SERVO_TRAJ_RATIO - 1))
		{
			//Calculate Change in position for this servo, taking remainder into account
			if(DX_Remainder)
			{
				Adj_DX = DX_Per_Servo + TrajDir;		// +1 for positive motion, -1 for negative
				DX_Remainder--;
			}
			else
			{
				Adj_DX = DX_Per_Servo;
			}

			//increment desired position
			New_ServoPosition += Adj_DX;	//<--Adj_DX = increment in pm/s
			DriverInc = Adj_DX / 1000;		//<--DriverInc: relative position in nm.

			DriverIncRem += Adj_DX - (DriverInc * 1000);

			if( DriverIncRem >= 1000)
			{
				DriverIncRem -= 1000;
				DriverInc++;
			}
			else if( DriverIncRem <= -1000)
			{
				DriverIncRem += 1000;
				DriverInc--;
			}

			TrajCycleServoCntr++;
		}
		else if( TrajCycleServoCntr >= ( SERVO_TRAJ_RATIO - 1 ) )
		{
			//Our new Servo Position is the Trajectory Point calculated 1ms. ago.
			DriverInc = (NextTrajectoryPosition - New_ServoPosition)/1000;	//DriverInc in nanometers.

			//Keep track of the picometers shaved off. When we reach 1000, add in the extra nanometer.
			DriverIncRem += (NextTrajectoryPosition - New_ServoPosition) - (DriverInc * 1000);
			if(DriverIncRem >= 1000)
			{
				DriverIncRem -= 1000;
				DriverInc++;
			}
			else if( DriverIncRem <=-1000)
			{
				DriverIncRem += 1000;
				DriverInc--;
			}

			New_ServoPosition = NextTrajectoryPosition;

			if( TrajState == NO_MOTION )
			{
				if(FeedbackMode == OPEN_LOOP)
				{
					//Exit Path for an open-loop move
					DriverState = DRIVER_CLEANUP;
					InPosition = 1;
				}
				else
				{
					//Exit Path for a non-open-loop move
					DriverState = DRIVER_OFF;
				}
			}
			else
			{
				//Get new target position from trajectory
				NextTrajectoryPosition = CurrentPos;

				//Calculate distance to move over next millisecond.	//The same limitation that I introduced in Equation #1 is valid here.
				//Maximum Travel velocity of 85.9 meters/second. Otherwise Distance_20_Servos(32-bits) can not accurately hold the difference.
				DistancePerN_Servos = (NextTrajectoryPosition - New_ServoPosition);	//+/-

				//Calculate our incremental move at the servo rate.
				DX_Per_Servo = DistancePerN_Servos / SERVO_TRAJ_RATIO;
				DX_Remainder = TrajDir * (DistancePerN_Servos - DX_Per_Servo * SERVO_TRAJ_RATIO);	//remainder is always positive
			}
			TrajCycleServoCntr = 0;
			RunTraj = 1;
			CalcNextTrajPoint = 1;
		}
		if(FeedbackMode == OPEN_LOOP || FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
		{
			//=============Distance To DAC Steps Equation
			//	Convert relative position increment into a # of DAC steps to move this servo cycle.
			//  ServoInc == move this distance this cycle (nm.). Right now it doesn't make sense to have picometers here.
			//This implementation goes off a relative position increment. The absolute version took way too long to execute.
			//Correction(dac_steps) = (position increment)(nm) * (DAC_Res)(steps/micron) * (1 micron / 1000 nm)
			//This multiplication limits the speed. for this to not overflow for the maximum DAC_Res, we are limited to 100mm/s
			inter_cor = DriverInc * (long)DAC_Res;
			DacAdjustment = inter_cor / 1000;

			//Keep track of remainder here. Otherwise we lose steps.
			StepAccum += inter_cor - (DacAdjustment * 1000);	//+/- depending on direction.

			//The step remainder accumulation ranges between -1000 to +1000.
			//When we hit 1000, we add an extra dac step.
			if(TrajDir == TRAJ_POS )
			{
				if( DacAdjustment >= 0)
				{
					if( StepAccum >= 1000)
					{
						StepAccum -= 1000;
						DacAdjustment++;
					}
				}
				else
				{
	//!				Servo_bkv = 22;
				}
				SignedDacAdjustment = DacAdjustment;
			}
			else if(TrajDir == TRAJ_NEG)
			{
				if(DacAdjustment <= 0)
				{
					DacAdjustment = -DacAdjustment;
					if( StepAccum <= -1000)
					{
						StepAccum += 1000;
						DacAdjustment++;
					}
				}
				else
				{
	//!				Servo_bkv = 25;
				}
				SignedDacAdjustment = -DacAdjustment;
			}

			//Correction is positive and DriverDir will tell the FPGA which direction we're going.
			//In Open Loop, direction doesn't change during the move.

			//This is assuming positive direction.
			if(DacAdjustment > MAX_CORRECTION)
			{
				DacAdjustment = MAX_CORRECTION;
			}

			//==========Calculate FPGA Vals
			WholeStep = DacAdjustment / DAC_Updates_Per_10k;
			FracStep = DacAdjustment - (WholeStep * DAC_Updates_Per_10k);
			//=========================
			//Prepare Data for FPGA
			SetupMotionBuf = 1;
		}
	}
	else if(DriverState == DRIVER_CLEANUP && FeedbackMode == OPEN_LOOP)
	{
		//DriverCleanup should not happen at the end of a closed loop move
		spiMotionData = 0;
		spiMotionData |= DriverDir<<31;

		SetupMotionBuf = 0;		//MotionBuf is already prepared
		SendMotionData = 1;
		DriverState = DRIVER_OFF;
	}
	else if(DriverState == DRIVER_CLEANUP && FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
	{
		//DriverCleanup should not happen at the end of a closed loop move
		spiMotionData = 0;

		SetupMotionBuf = 0;	//<--This could be named better.
		SendMotionData = 1;

		if(TrajState == NO_MOTION) DriverState = DRIVER_OFF;
	}

	//Move encoder processing here. Reduce delay
	//Read Encoder
		//--TODO port encoder reading
		//---------------------------------------------------------
		if(EncoderChannelSelect != ABSOLUTE){
			//Read Encoder Register. 32-bit.
			EncoderCount = htim4.Instance->CNT;
			EncoderDif = (long)EncoderCount - (long)PreviousCount;
			//Detect and correct for rollover in positive direction.
			if(-EncoderDif > 32768LL)
			{
				EncoderDif += 65536LL;
			}
			//Detect and correct for rollover in the negative direction.
			else if(EncoderDif>32768LL)
			{
				EncoderDif -= 65536LL;
			}

			PreviousCount = EncoderCount;	//Save count for next cycle

			//TODO --> Add this back in for VRT command
			/*
			EncoderLogSum -= EncoderCountLog[enc_log];
			EncoderLogSum += EncoderDif;
			EncoderCountLog[enc_log++] = EncoderDif;
			*/
		}
		else{
			if(AbsValidData){
				EncoderDif = (int64_t)LastPos - (int64_t)EncReadings[encptr].pos_data;
				LastPos = EncReadings[encptr].pos_data;
				EncoderLogSum -= EncoderCountLog[enc_log];
				EncoderLogSum += EncoderDif;
				EncoderCountLog[enc_log++] = EncoderDif;
			}
		}

		//If using an absolute encoder, and we don't get a valid reading
		//TODO --> Add absolute encoder info back in
		/*
		if(EncoderChannelSelect == ABSOLUTE && !AbsValidData){
			goto skip_filter;
		}
		*/
		//TODO --> Add encoder filter back in
		/*
		//BUF1 == Standstill, BUF2 == Motion
		if(EncFilterPhase == ENC_FILTER_GROWING){
			FilterSize++;
		}
		else if(EncFilterPhase == ENC_FILTER_NORMAL){
			//Remove oldest point
	        EncDif_SumBuf1 -= EncDifLog[Enc_TailBuf1];
	        Enc_TailBuf1 = (Enc_TailBuf1 + 1) & ENCODER_BUFFER_MASK;
		}

	    //Log New Point
	    if(EncoderChannelSelect != ABSOLUTE){
	    	EncDifLog[Enc_HeadPtr] = EncoderDif * 10;    //Keep track of 1/10th of an encoder count
	    }
	    else{
	    	EncDifLog[Enc_HeadPtr] = EncReadings[encptr].pos_data;
			if(++encptr >= 40){
				encptr = 0;
			}
		}

	    //Add New datapoint into sum
	    EncDif_SumBuf1 += EncDifLog[Enc_HeadPtr++];       //Buf1 and Buf2 are moving windows within EncDifLog
	    Enc_HeadPtr &= ENCODER_BUFFER_MASK;

	    if(FilterSize == TargetFilterSize){
	    	EncFilterPhase = ENC_FILTER_NORMAL;
	    }

	    EncAvg1 = EncDif_SumBuf1 / (int32_t)FilterSize;
		 */

	    if(EncoderChannelSelect != ABSOLUTE){
	        RealPositionCounter += EncoderDif;
	        RealPosition = RealPositionCounter * EncoderResolution;
	    }
	    else{
			RealPosition = (EncAvg1  - absoluteEncoderOffset) * EncoderResolution;	//Factor in Encoder Resolution and epl setting
			if(EncoderPolarity != 0){
				RealPosition = -RealPosition;
			}

			if (StartupPosEnable==2)
			{
				CurrentPos = RealPosition;
				StartupPosEnable = 0;
			}
	    }

		if( enc_log >= VRT_SamplePeriod)
		{
			enc_log = 0;
		}
	skip_filter:

	//Closed Loop Calculation with Deadband Logic
	if(FeedbackMode == CLOSED_LOOP || (FeedbackMode == OPEN_LOOP_CLOSED_FINISH && DriverState == DRIVER_OFF))
	{
		if (ESTOP_IN_PROGRESS)
		{
			CurrentPos = RealPosition;
			New_ServoPosition = RealPosition;
		}
		Error_pm64 = New_ServoPosition - RealPosition;

		ESTOP_IN_PROGRESS = 0;
		//Convert to a 32-bit value here to avoid 64-bit divide
		if(Error_pm64 > ERROR_31_BIT_LIMIT){
			Error_pm = ERROR_31_BIT_LIMIT;		//This is a little convoluted.
		}
		else if(Error_pm64 < -ERROR_31_BIT_LIMIT){
			Error_pm = -ERROR_31_BIT_LIMIT;
		}
		else{
			Error_pm = Error_pm64;
		}//*/

		PrevError = Error;
		Error = Error_pm / 1000;
		DeadbandCheck = DeadbandWindow;

		//Deadband Logic
		if(DriverState == DRIVER_OFF)
		{
			if(Error > 0 && Error > DeadbandCheck)
			{	//Outside Deadband in + Direction
				InDeadband = 0;
				//InPosition = 0;
			}
			else if(Error < 0 && Error < -(long)DeadbandCheck)
			{	//Outside Deadband in - Direction
				InDeadband = 0;
				//InPosition = 0;
			}
			else{
				InDeadband = 1;
				//InPosition = 1;
				IErrorAccum = 0;
			}

			if(Error > 0 && Error > InPositionWindow)
			{
				InPositionINP = 0;
				InPositionTimeCounts_temp = 0;
			}
			else if(Error < 0 && Error < -(long)InPositionWindow)
			{
				InPositionINP = 0;
				InPositionTimeCounts_temp = 0;
			}
			else
			{
				if(InPositionTimeCounts_temp >= InPositionTimeCounts)
				{
					InPositionINP = 1;
				}
				else InPositionTimeCounts_temp++;
			}

			if (InPositionCounts == 0 || InPositionTime == 0) InPosition = InDeadband; //V2.2.13 - Fix references to INP parameters
			else InPosition = InPositionINP;

			if(((CorrectionAttempts < DeadbandTimeoutCounts) || DeadbandTimeoutCounts == 0) && (InDeadband == 0))
			{
				SetupMotionBuf = 1;
				CorrectionAttempts++;
			}
			else
			{
				//We are in position
				SetupMotionBuf = 0;
				DacAdjustment = 0;

				spiMotionData = 0;
				spiMotionData |= DriverDir << 7;

				SendMotionData = 1;

				if(InPosition){
					CorrectionAttempts = 0;
				}
			}
		}
		else
		{
			SetupMotionBuf = 1;
			CorrectionAttempts = 0;
		}

		if(SetupMotionBuf)
		{
			IErrorAccum += Error;
			temp_Limit = (long)iTermSumLimit;
			if (IErrorAccum > temp_Limit) IErrorAccum = temp_Limit;
			else if (IErrorAccum < -temp_Limit) IErrorAccum = -temp_Limit;

			if(Error > maxError) Error = maxError;
			else if(Error < -maxError) Error = -maxError;

			if (FeedForwardParameter > 0) FFTerm = (Error * (int32_t)FeedForwardParameter) / 1000;
			else FFTerm = (Error * (int32_t)DAC_Res) / 1000;

			pTerm = Kp ? (FFTerm / Kp) : (0);
			iTerm = Ki ? (IErrorAccum / Ki) : (0);
			dTerm = Kd ? ((Error - PrevError) / Kd) : (0);

			if (motorControlMode == MOTOR_BLDC_MODE)
			{
				pTerm = Kp ? (Error / Kp) : (0);
			}

			PID_Term = pTerm + iTerm + dTerm;

			DacAdjustment = PID_Term;
			SignedDacAdjustment = DacAdjustment;
			PrevError = Error;

			if(DacAdjustment > 0)
			{
		        if(MPol == 0)	//This is what we do for the NO_MOTION Correction.
				{
					DriverDir = DRV_POS;
				}
				else DriverDir = DRV_NEG;
			}
			else if(DacAdjustment < 0)
			{
				DacAdjustment = -DacAdjustment;
				if(MPol == 0)
				{
					DriverDir = DRV_NEG;
				}
				else DriverDir = DRV_POS;
			}

			if(DacAdjustment > CL_DACSTEP_PERSERVO_MAX) DacAdjustment = CL_DACSTEP_PERSERVO_MAX;
			WholeStep = DacAdjustment / DAC_Updates_Per_10k;
			FracStep = DacAdjustment - (WholeStep * DAC_Updates_Per_10k);
		}
	}

	spiMotionData = 0;
	if(SetupMotionBuf)
	{
		if (motorControlMode == MOTOR_PIEZO_MODE)
		{
			// MOTION PACKET FORMAT
			// Direction 	- 1 bit		--> 	[bits 	31]
			// WholeStep 	- 10 bits	-->		[bits	21:30]
			// FracStep  	- 10 bits	-->		[bits	11:20]
			// Sync			- 11 bits	-->		[bits 	0:10]		***details TBD ***
			spiMotionData |= DriverDir<<31;
			spiMotionData |= (WholeStep & 0x3FF) << 21;
			spiMotionData |= (FracStep & 0x3FF) << 11;

			SPI2_NSS_ON;
			SendMotionData = 1;
		}
		else if (motorControlMode == MOTOR_BLDC_MODE)
		{
			// MOTION PACKET FORMAT
			// Direction 	- 1 bit		--> 	[bits 	31]
			// Sync			- 15 bits	-->		[bits 	16:30]		***details TBD ***
			// PID			- 16 bits	-->		[bits	0:15]
			spiMotionData |= DriverDir<<31;
			spiMotionData |= (DacAdjustment & 0xFFFF);
		}
	}
	if ((motorControlMode == MOTOR_BLDC_MODE) && MotorActive)
	{
		SPI2_NSS_ON;
		//for BLDC mode, we are going to send out this data every servo cycle regardless
		//We are sending additional data in BLDC mode as well (PID output, encoder data, trajectory position)
		spiMotionData |= ((uint32_t)spi_rx_data[0] << 16) & 0x00FF0000;
		spiMotionData |= ((uint32_t)spi_rx_data[1] << 24) & 0x7F000000;
		SendMotionSPIDataExtended(OPCODE_MOTION_DATA, PAR_MOTION_DATA, spiMotionData, (int32_t)(RealPosition / 1000), (int32_t)(New_ServoPosition / 1000) );

	}
	else if (SendMotionData && MotorActive)
	{
		//For piezo control, we are only sending PID output (DAC adjustment) when needed
		SendMotionSPIData(OPCODE_MOTION_DATA, PAR_MOTION_DATA, spiMotionData);
		spiPacketCounter++;
		SendMotionData = 0;
	}

	if(startTrace && DriverState == DRIVER_ON)
	{
		if(TrajDir == TRAJ_POS)		//DriverDir differs based on motor polarity, but all we care about is the sign of our trajectory position
		{
			if(New_ServoPosition >= T_StartPos)
			{
				traceOn = 1;
				startTrace = 0;
			}
		}
		else if(TrajDir == TRAJ_NEG)
		{
			if(New_ServoPosition <= T_StartPos)
			{
				traceOn = 1;
				startTrace = 0;
			}
		}
	}
	else if(traceOn)
	{
		uint32_t tempADC;
		HAL_ADC_PollForConversion(&hadc1, 1);

		if(points % T_ServosPerSample == 0)
		{
			traceLog[traceSample].realPosition = (int32_t)(RealPosition/1000);
			traceLog[traceSample].trajPosition = (int32_t)(New_ServoPosition/1000);
			traceLog[traceSample].dacAdjustment = SignedDacAdjustment;
			//ADC collection. Need to convert from 12-bit value [0-4095] to millivolts [0-60000]
			tempADC = HAL_ADC_GetValue(&hadc1);
			traceLog[traceSample++].motorADC = (uint16_t)((tempADC * 60000) / 4095);


			if (traceSample >= T_Samples)
			{
				traceDone = 1;
				traceOn = 0;
			}
		}
	}
	points++;

	if(RunTraj)
	{
		//there is no reason to occupy a timer for the trajectory calculation if it always operates every 10th servo timer
		//timer 3 operation removed. just use the PositionHandler function instead.
		PositionHandler();
		RunTraj = 0;
	}
}

void DriverCleanup()
{
	DriverState = DRIVER_OFF;

	spiMotionData = 0;

	SendMotionData = 1;
}

enum DriverStates getDriverState()
{
	return DriverState;
}

uint8_t WaitForTimeActive()
{
	return WaitForTimePeriod;
}

void SetWTM_Ticks(uint32_t wait)
{
	if(WaitForTimePeriod)
	{
		WTM_Ticks += wait * SERVOS_PER_MS;
	}
	else
	{
		WTM_Ticks = wait * SERVOS_PER_MS;
		if(WTM_Ticks) WaitForTimePeriod = 1;
	}
}

void ClearVRTLog()
{
	enc_log = 0;
	EncoderCountLog[0] = 0;
	EncoderLogSum = 0;
}

//Trajectory Signage:   Pos = +1  //   Neg = -1
//FPGA Signage:			Pos = 0	  //   Neg = 1
int64_t GetAbsEncoderReading(){
	return ((uint64_t)EncReadings[encptr].pos_data);
}

void GetAbsEncStatus(uint32_t *a, uint32_t *b, uint32_t *c){
	*a = abs_enc_crc_err_cntr;
	*b = abs_enc_err_flag_cntr;
	*c = abs_enc_warn_flag_cntr;
}

void ZeroAbsEncCounters(){
	abs_enc_crc_err_cntr = 0;
	abs_enc_err_flag_cntr = 0;
	abs_enc_warn_flag_cntr = 0;
}

void ReInitEncFilter(){
    EncFilterReInit = 1;
}

void SetClStepsPerServo(uint32_t steps){
	UserCorrectionLimit = (uint16_t)steps;
}

void ResetdTermSampleCounter(){
	dSampleCntr = 0;
}

void ResetiTermSampleCounter(){
	iSampleCntr = 0;
}

void GetPidTerms(int32_t *a,int32_t *b,int32_t *c){
	*a = pTerm;
	*b = iTerm;
	*c = dTerm;
}

void SetupPID(uint32_t nKP, uint32_t nKI, uint32_t nKD){
	Kp = nKP;
	Ki = nKI;
	Kd = nKD;
}

/*
 * Traj.c
 *
 *  Created on: May 31, 2023
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "command.h"
#include "commandlist.h"
#include "error.h"
#include "sysinfo.h"
#include "motion.h"
#include "dio.h"

#define LIMIT_REBOUND 50000000					//!<Distance to move out of the hard limit in pm.
#define ONE_ROTATION 360000000000				//!<Degrees in one full rotation in picodegrees
#define HARDSTOP_SAMPLE_POINTS 50				//!<Sample Buffer Size for Hard Stop Detection

#define HALF_TRAVEL_NM  50000000000LL		//
#define TRAVEL_RANGE_NM 100000000000LL		//

#define HOME_DISTANCE1 5	//Values used by home routine to keep track of the approach direction
#define HOME_DISTANCE2 6

//=====Position variables are in units of picometers, unless otherwise stated.
//=====Velocity variables are in units of nanometers / second
//=====Acceleration and Deceleration variables are in units of nanometers / second / second

int32_t ChangePositionOnFly(void);
int64_t TrapAccel(void);
int64_t TriAccel(void);
int64_t ConstantVelocity(void);
int64_t Decelerate(void);
void Trajectory(void);
int64_t Stop(void);
void InitializeTrajRP_Dif();
uint8_t MotionProfileSetup(int64_t);
int64_t NoMotion();
void LoadNewValues();
void ReleaseModLock();
void SetModLock();
void StopMotion();

//extern volatile uint8_t startPulse;
extern volatile enum DriverStates DriverState;
extern volatile uint8_t DetectEncoder;
extern volatile int64_t  RealPosition;
extern int64_t CurrentPos;
int64_t(*TrajTick)() = &NoMotion;

//!Is the current move a targeted move or an infinite move?
enum MoveTypes MoveType = TARGETED;

//!Current Trajectory State (Accel, Decel, Constant Velocity, etc...)
enum MotionPhases TrajState = NO_MOTION, new_TrajState = NO_MOTION, servo_time_max_trajstate=0;

enum MotionModes MotionMode = MOVE_;

//HardStopDetectionVars
//volatile unsigned char FindingLimit = 0;
volatile uint16_t HardStopFound = 0;

//Logging arrays.
enum MotionPhases TrajLog[800] = {0};
//unsigned int TimeLog[200];
//int64_t LogPos[400];
//int64_t LogVel[400];
uint16_t log_ptr = 0;

uint8_t MoveMod = 0;	//Flag used to signal whether the current move has been modified since beginning.
uint8_t MOTION_IN_PROGRESS = 0;	//This flag is how the driver determines it is time to turn on.

//Flag to stop closed loop runoff when there is a closed loop issue. Allows EST to work
uint8_t ESTOP_IN_PROGRESS = 0;

extern int64_t MinDistance;
extern volatile uint32_t DeadbandTimeoutCounts;
extern volatile int64_t DeadbandWindow;

// Internal State Variables For Trajectory Profile

int64_t p_step;			//!<Step size at target velocity per trajectory cycle (.001s)
int64_t StartPos = 0;
int64_t TargetPos;		//!<Axis Target Position (pm)					/	(microdegree / 2)
int64_t TrajPos = 0;					//!<Position within the current move - Relative Trajectory Value
int64_t TrajTarget = 0;				//!<Trajectory Target Position - Relative Trajectory Value
int64_t DescentPoint;					//!<Decel Point For Trapezoidal Profile - Relative Value.
int8_t TrajDir = 1;
uint8_t LoadVals = 0;
uint8_t MotionReady = 0;
uint64_t LastVel;				//!<Velocity from previous trajectory cycle
uint64_t CurrentVel;			//!<Current Travel Velocity Throughout the Move
uint64_t TrapMinDistance;		//!<This is the minimum move distance where a trapezoidal profile can be used (ACCDistance + DECDistance)
uint64_t TrajAccel;
uint64_t TrajDecel;
uint64_t TargetVel;			//!<Motion Target Velocity (nm/s)
uint64_t TargetVel_n;			//!<Motion Target Velocity (pm/s)
uint64_t VelSq;
uint64_t VelStepUp;
uint64_t VelStepDown;
uint64_t StopVelStep;
uint64_t DecelTicks;
int64_t AccelDistance;	//!<Distance Needed To Reach Constant Velocity  (V^2 / (2ACC))
uint64_t DecelDistance;	//!<Distance Needed To Decelerate From Constant Velocity (V^2 / (2DEC))
uint64_t TrapMinDistance;	//!<AccelDistance + DecelDistance. This is the minimum move distance for a trapezoidal profile.
uint64_t FFM_Vel;

//Limit State Variables
volatile uint16_t InLimit = 0;
volatile uint16_t InPositiveLimit = 0;
volatile uint16_t InNegativeLimit = 0;
volatile uint16_t IndexNotFound = 0;
volatile uint16_t NoEncoderInHome = 0;

//unsigned int TrajTimeLog[400];
uint16_t traj_log_ptr = 0;
uint16_t traj_time_max = 0;

uint8_t traj_bkv = 0;

extern TIM_HandleTypeDef htim5;
#define HomingTimer htim5;

int64_t NoMotion()
{
//	if(MOTION_IN_PROGRESS)
//	{
//		MotionSetUp();

//	}
	return 0;
}

volatile uint8_t AllGood = 0;

void StartMove()
{
	if(MotionReady)
	{
		MOTION_IN_PROGRESS = 1;
		MotionReady = 0;
		InitializeTrajRP_Dif();	//<--Always initializes before a move.
	}
}

void EmergencyStop()
{
	StopVelStep = SYSTEM_MAX_ACCEL;
	TrajState = STOP;
	TrajTick = Stop;
}

uint8_t MotionInProgress()
{
	return MOTION_IN_PROGRESS;
}

void StopMotion2(enum StopType stop_please)
{
	if(stop_please == NORMAL_STOP)
	{
		StopVelStep = VelStepDown;
	}
	else if(stop_please == EMERGENCY_STOP)
	{
		StopVelStep = SYSTEM_MAX_ACCEL;
	}
	else StopVelStep = VelStepDown;

	TrajState = STOP;
	TrajTick = Stop;
}

void StopMotion()
{
	StopVelStep = VelStepDown;
	if(TrajState != DECEL)		//Stopping during decel does nothing. You're already decelerating.
	{
		TrajState = STOP;
		TrajTick = Stop;
	}
}

int64_t getTargetPos()
{
	return TargetPos;
}

enum MotionPhases getTrajState(){
	return TrajState;
}

int64_t Traj_DX=0;		//Keeps track of position changes in femtometers.
int64_t Pos_DX=0;		//Relative distance to move every trajectory cycle(pm.)

int64_t TrapAccel()
{
	//This state steps up to a target velocity.
	LastVel = CurrentVel;
	if(CurrentVel + VelStepUp < TargetVel_n)
	{
		CurrentVel += VelStepUp;
	}
	else
	{
		CurrentVel = TargetVel_n;
		TrajState = CONSTANT_VELOCITY;
		TrajTick = ConstantVelocity;
	}
	Traj_DX = (CurrentVel + LastVel) / 2;

	if(Traj_DX > 1000)
	{
		Pos_DX = Traj_DX / 1000;
		Traj_DX -= Pos_DX * 1000;
		//TrajPos +=  Pos_DX;
	}

	return Pos_DX;
}

int64_t TrapDecel()
{
	//--TODO This current incarnation doesn't make sense.
	//Look into it.

	//This state handles a decel to a new target velocity during motion.
	int64_t deltaVel;
	int64_t tempTargetVel_n = 0; //added to get rid of warning

	LastVel = CurrentVel;	//What units is CurrentVel and
	deltaVel = (int64_t) CurrentVel - (int64_t)VelStepDown;
//	tempTargetVel_n - (int64_t) TargetVel_n; //Meaningless statement

	if(deltaVel > tempTargetVel_n)
	{
		CurrentVel -= VelStepDown;
	}
	else
	{
		CurrentVel = TargetVel_n;
		TrajState = CONSTANT_VELOCITY;
		TrajTick = ConstantVelocity;
	}
	Traj_DX += (CurrentVel + LastVel) / 2;

	if(Traj_DX >= 1000)
	{
		Pos_DX = Traj_DX / 1000;
		Traj_DX -= Pos_DX * 1000;
//		TrajPos += Pos_DX;
	}
	return Pos_DX;
}

uint64_t CurrentDecelDistance;		//!<Distance that it would take to decel from current velocity. Used in TRI_ACCEL.
int64_t DecelPoint;

uint8_t breakp = 8;
int64_t TriAccel()
{
	uint64_t CurrentVel_nm = 0;
	//This phase handles the acceleration portion of a triangular motion profile
	if(MoveType == INFINITE)
	{
		//An infinite move should only go through  trap_accel.
		//If I've gotten here, something is wrong.
		//What should I do
		//Probably stop and give an error. durr.
		//Write a function to cleanly exit trajectory.
	}

	LastVel = CurrentVel;		//pm/s

	if(CurrentVel + VelStepUp < TargetVel_n)
	{
		CurrentVel += VelStepUp;
	}
	else
	{
		//This is just a safety check. If we're doing a triangular profile, we should never
		//hit our target velocity.
		CurrentVel = TargetVel_n;
		TrajState = CONSTANT_VELOCITY;
		TrajTick = ConstantVelocity;
	}

	DecelTicks = CurrentVel / VelStepDown;

	Traj_DX += (CurrentVel + LastVel) / 2; //pm		There's some math magic going on here, and I don't remember what it was.

	if(Traj_DX >= 1000)
	{
		Pos_DX = Traj_DX / 1000;
		Traj_DX -= Pos_DX * 1000;
		CurrentVel_nm = CurrentVel / 1000;

		//Calculate our Required Decel Distance every time through. As soon as
		//our distance to target is ~= to our decel distance, it's time to start decelerating.

		//Difference between this calc and the one in Move Setup...
		//Here our velocity is in pm/s.
		CurrentDecelDistance = (CurrentVel_nm * CurrentVel_nm) / (Decel * 2);
		//CurrentDecelDistance = CurrentVel * CurrentVel / (*Decel * 1000);	//Squaring the velocity here overflows. pm/s * pm/s overflows around 4.3 mm/s
//		LogPos[traj_log_ptr] = CurrentDecelDistance;
//		LogVel[traj_log_ptr++] = CurrentVel;
//		if(traj_log_ptr == 43)
//		{
//			traj_bkv = 26;
//		}
//		if(traj_log_ptr >= 400)
//		{
//			traj_log_ptr = 0;
//		}
		DecelPoint = TrajTarget - (CurrentDecelDistance*1000);		//TrajTarget is in picometers, correct? Yes, I believe so.

		//Check that this move does not overshoot our target. This check is in here for very large and
		//very small settings that can cause weird behavior.
		if((TrajPos + Pos_DX) >= TrajTarget)
		{
			Pos_DX = TrajTarget - TrajPos;
			CurrentVel = 0;
			MOTION_IN_PROGRESS = 0;
			TrajState = NO_MOTION;
			TrajTick = NoMotion;
		}
		else if( FeedbackMode == OPEN_LOOP_CLOSED_FINISH )
		{
			//DescentPoint = Relative
			if (TrajDir == 1) //positive move
			{
				if( (RealPosition) >= ( StartPos + DecelPoint ) )
				{
					TrajState = DECEL;
					TrajTick = Decelerate;
					if(DecelTicks == 0)
					{
						DecelTicks = 1;
					}
				}
			}
			else //negative move
			{
				if( (RealPosition) <= ( StartPos - DecelPoint ) )
				{
					TrajState = DECEL;
					TrajTick = Decelerate;
					if(DecelTicks == 0)
					{
						DecelTicks = 1;
					}
				}
			}
		}
		else if((TrajPos + Pos_DX) >= DecelPoint)	//Does the move take us past our decel distance
		{
			TrajState = DECEL;
			TrajTick = Decelerate;
//			CurrentPos += TrajDir * Pos_DX;
			if(DecelTicks == 0)
			{
				DecelTicks = 1;
			}
		}
	}
	else Pos_DX = 0;

	return Pos_DX;
}

//Trajectory position is always positive.
int64_t ConstantVelocity()
{
	//don't need Traj_DX here because the minimum distance we can travel here is 1pm
	if(MoveType == INFINITE)
	{
		//Infinite Move. We're not looking for a decel point.
		TrajPos = 0;
		return p_step;
	}
	else
	{	//Looking for a decel point.
		//check to see if we can make this move without overshooting our target
		if((TrajPos + p_step) >= TrajTarget && FeedbackMode != OPEN_LOOP_CLOSED_FINISH)
		{
			CurrentVel = 0;
			MOTION_IN_PROGRESS = 0;
			TrajTick = NoMotion;
			TrajState = NO_MOTION;
			if (MotionMode == JOG_) AddError(MOVE_OUTSIDE_SOFT_LIMITS,"JOG");
			return TrajTarget - TrajPos;
		}
		else
		{
			if( FeedbackMode == OPEN_LOOP_CLOSED_FINISH )
			{
				//DescentPoint = Relative
				if( ( TrajDir * RealPosition ) >= ( TrajDir * ( StartPos + ( TrajDir * DescentPoint ) ) ) )
				{
					TrajState = DECEL;
					TrajTick = Decelerate;
				}
			}
			else
			{
				if(TrajPos + p_step >= DescentPoint)
				{
					TrajState = DECEL;
					TrajTick = Decelerate;
				}
			}
		}
	}
	return p_step;
}

//unsigned int decel_time = 0;
uint8_t DecelStarted = 0;
int64_t Decelerate()
{
	//This is not a trapezoidal deceleration.
	//The average deceleration is the set deceleration value,
	//but it is not constant. Worry about this later. Could keep it or change it.
	int64_t DecelDX;
	DecelStarted = 1;
	if(DecelTicks)
	{
		if(FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
		{
			//This could be + or -
			//We could change directions here.
			//We could go past here.
			DecelDX = TrajDir * ( TargetPos - RealPosition );	//There's no way to catch this at the trajectory rate. Why does it only show up with this encoder value?
			if(DecelDX <= 1000000)
			{
				CurrentVel = 0;
				MOTION_IN_PROGRESS = 0;
				TrajState = NO_MOTION;
				TrajTick = NoMotion;
				CurrentPos = TargetPos;
				return 0;
			}
		}
		else
		{
			DecelDX = TrajTarget - TrajPos;
		}

		if(DecelTicks == 1)
		{
			Pos_DX = DecelDX;
			DecelTicks--;
		}
		else
		{
			DecelDX = (2 * DecelDX) / (int64_t)DecelTicks--;
			Pos_DX = DecelDX;
		}
		//Modify this calculation to match the decel rate of the
		CurrentVel = Pos_DX * 1000;
//		LogVel[log_ptr] = CurrentVel;
//		LogPos[log_ptr++] = Pos_DX;
//		if(log_ptr == 400)
//		{
//			log_ptr = 0;
//		}

	}
	else
	{
		//CurrentPos = Target;
		//if(!HomeInProgress)WaitForStop = 0;
		CurrentVel = 0;
		MOTION_IN_PROGRESS = 0;
		TrajState = NO_MOTION;
		TrajTick = NoMotion;
		if(FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
		{
			CurrentPos = TargetPos;
			return 0;
		}
		else
		{

			return (TrajTarget - TrajPos);
		}
		//StartMove = 0;
	}

	return Pos_DX;
}

void ExitTrajectory()
{
	CurrentVel = 0;
	MOTION_IN_PROGRESS = 0;
	TrajState = NO_MOTION;
	TrajTick = NoMotion;
	if(FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
	{
		CurrentPos = TargetPos;
	}
}

int64_t Stop()
{
	LastVel = CurrentVel;

	if((int64_t)(CurrentVel - StopVelStep) > 0)
	{
		CurrentVel -= StopVelStep;
	}
	else
	{
		CurrentVel = 0;
		MOTION_IN_PROGRESS = 0;
		TrajState = NO_MOTION;
		TrajTick = NoMotion;

		if(FeedbackMode == CLOSED_LOOP || FeedbackMode == OPEN_LOOP_CLOSED_FINISH)
		{
			CurrentPos = RealPosition;
//			New_ServoPosition = RealPosition;
		}
	}

	Traj_DX += (CurrentVel + LastVel);
	if(Traj_DX >= 1000)
	{
		Pos_DX = Traj_DX / 1000;
		Traj_DX -= Pos_DX * 1000;
			//CurrentPos += TrajDir * Pos_DX;
	}

	return Pos_DX;
}

//=====Variables used for HardStopDetection.
int64_t TrajRP_Dif[HARDSTOP_SAMPLE_POINTS] = {10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
uint64_t HSVelLog[HARDSTOP_SAMPLE_POINTS] = {0};		//!<Log of theoretical velocities.
uint64_t AvgVelSum = 0;							//!<Running sum of contents of HSVelLog (pm/s)
int64_t TrajRP_Sum = 0;										//!<Running Sum of contents of TrajRP_Dif
uint8_t SumPtr = 0;									//!<Pointer to current location inside TrajRP_Dif
int64_t IncDX;
int64_t Traj;

void InitializeTrajRP_Dif()
{
	uint8_t d = 0;
	TrajRP_Sum = 0;
	AvgVelSum = 0;
	while(d<HARDSTOP_SAMPLE_POINTS)
	{
		TrajRP_Dif[d] = 10;
		TrajRP_Sum += 10;
		HSVelLog[d] = 0;
		AvgVelSum += 0;
		d++;
	}
}

//long long LogPos[2000];
//enum motion_phases TrajLog[2000];
//unsigned long long TimeLog[2000];
//unsigned int log_ptr = 0;
uint8_t SetupJog(int64_t jog_vel, enum MoveTypes mt)
{
	int64_t ml;
	//JOG
	//TargetVel = jog_vel;
	TrajAccel = JogAccel;
	TrajDecel = JogAccel;

	//The jog velocity passed in here has direction information encoded into it.

	MotionMode = JOG_;

	MoveType = mt;
	//Are we jogging to limits, or with no target?
	if(jog_vel > 0)
	{
		if(MoveType == TARGETED)
		{
			ml = tlp - CurrentPos;
		}
		else ml = 0;
		TargetVel = jog_vel;
	}
	else if(jog_vel < 0)
	{
		TargetVel = -jog_vel;
		if(MoveType == TARGETED)
		{
			ml = tln - CurrentPos;
		}
		else ml = -1;
	}
	else ml = 0;

	traj_bkv = 0;
	//This move could be targeted or infinite. depending on the limit config settings.
	MotionProfileSetup(ml);

	return 1;
}

void SetUpCustom();

uint8_t SetupMove(int64_t ml, enum MoveTypes mt)
{
	//Interface between trajectory and settings. This is kind of a shortcut right now.
	TargetVel = Vel;	////TargetVel = GetTargetVel();
	TrajAccel = Accel;	//TrajAccel = GetAccel();
	TrajDecel = Decel;	//TrajDecel = GetDecel();
	MoveType = mt;
	MotionMode = MOVE_;

	if(MotionProfileSetup(ml))	//MoveLength
	{
		return 1;
	}
	return 0;
}

int64_t test_sum = 0;
void CalcSumTest(){
	uint16_t ii = 0;
	int64_t new_test_sum = 0;
	while( ii < HARDSTOP_SAMPLE_POINTS){
		new_test_sum += TrajRP_Dif[ii];
		ii++;
	}
	test_sum = new_test_sum;
}

//Pass in a velocity in nm/s and set vel in pm/s
void Setup_FFM_Vel(uint64_t vel){
	FFM_Vel = vel * 1000;
}

int32_t PrintSum = 0;
uint16_t FRollover = 0;
uint8_t t_count = 0;				//!<Every time the hard stop test passes, t_count is incremented.
uint32_t traj_count = 0;			//!<Keeps track of how many times trajectory has run. Unnecessary
uint8_t LoadTrajVals = 0;
int64_t TrajRealPos = 0;					//!<Sampled RealPosition value at the beginning of the Trajectory cycle. Used for Hard Stop Detection
int64_t lastTrajRealPos = 0;				//!<Sampled RealPosition value for the previous Trajectory cycle.
volatile uint8_t CalcNextTrajPoint = 0;
int64_t TrajectoryController(void)
{
	uint64_t AvgVel = 0;	//Holds average velocity, computed over the past 50ms., in pm/s. Used for hard stop detection.

	if(LoadTrajVals)
	{
		LoadTrajVals = 0;
		LoadNewValues();
	}

	TrajRealPos = RealPosition;		//Sample encoder position.

	IncDX = TrajTick();

/*	if(CurrentVel >= FFM_Vel){
		FPGA_FFM_ENGAGE;
	}
	else{
		FPGA_FFM_EXIT;
	}*/

	if(MoveType != INFINITE || TrajState == TRAP_ACCEL)//<==Ok, if move type is not infinite, we only keep track during accel. Otherwise always.
	{
		TrajPos += IncDX;
	}

	if(TrajState != NO_MOTION)
	{
		TrajLog[traj_log_ptr++] = TrajState;
//		TimeLog[log_ptr] = time;

	}
	if(traj_log_ptr >= 800)
	{
		traj_log_ptr = 0;
	}

//	if(log_ptr == 400) log_ptr = 0;

	//Velocity approximation function
	//Velocity travelled in 1 trajectory period:  V_effective = (CurrentVel + LastVel)/2;
	//You can't just jump velocities. It's continuous. So I average the two.

	//Distance Travelled based off our effective velocity.
	//dx = V_effective * TrajectoryPeriod(.001s) fm/s.

	//The smallest distance we can travel in 1 trajectory cycle is 1 femtometer.

	//Hard stop detection
	if(HardstopDetectEnable)
	{
		if(TrajState == CONSTANT_VELOCITY)
		{
			//Take disappearing data point out of sum
			TrajRP_Sum -= TrajRP_Dif[SumPtr];
			AvgVelSum  -= HSVelLog[SumPtr];

			//Log new points
			TrajRP_Dif[SumPtr] = TrajRealPos - lastTrajRealPos;	// +/-
			HSVelLog[SumPtr] = CurrentVel;// / 1000;			// + only

			//New Sum Calcuation
			TrajRP_Sum += TrajRP_Dif[SumPtr];
			AvgVelSum  += HSVelLog[SumPtr++];
			if(SumPtr==HARDSTOP_SAMPLE_POINTS)
			{
				SumPtr = 0;
				FRollover++;
			}

			AvgVel = AvgVelSum / HARDSTOP_SAMPLE_POINTS;

			//Sample Window = 50ms
			//1 / 50ms = 20
			//1000 = velocity in nm/s ; position in pm/s
			//TheorDistance = AvgVel / 20 * 1000
			//										(.05s)
			//MinDistance(pm) = AvgVel(pm/s) * SamplePeriod / 10
			//MinDistance(pm) = AvgVel / 20 / 10;
			MinDistance = AvgVel / 20;
			MinDistance = (MinDistance * HardStopDeadband) / 100;

			//Do I scale the values put in before I calculate the sum or do I scale the value that I compare
			//the sum to? How do I scale?

			//HardStopDeadband will no longer be a setting.
			if((TrajRP_Sum >= 0 && TrajRP_Sum <= MinDistance) || (TrajRP_Sum <= 0 && TrajRP_Sum >= -MinDistance))
			{
				if(++t_count == 5)
				{
					//we've reached a hard stop
					TrajState = STOP;
					TrajTick = Stop;
					//t_vel = 0;
					StopVelStep = VelStepDown;
//					HardstopDetectEnable = 0;
					t_count = 0;
					PrintSum = TrajRP_Sum;
					HardStopFound = 1;
				}
			}
			else t_count = 0;
		}
	}

	//what is this?
	//Breakpoint test?
	if(Traj_DX <= 0)
	{
		lastTrajRealPos = 5;
	}

	CalcNextTrajPoint = 0;
	lastTrajRealPos = TrajRealPos;

	traj_count++;

	return TrajDir * IncDX;
}

void StartUpDefaults()
{
	CurrentPos = 0;
	TargetPos = 12000000000LL;	//12 mm/s in pm/s
	Vel = 2000000;			//2 mm/s in nm/s
	TargetVel = Vel;
	Accel = 100000000;			//nm/s
	Decel = 100000000;			//nm/s
}

int8_t getTrajDir()
{
	return TrajDir;
}

uint8_t MotionProfileSetup(int64_t MoveLength)
{
	//The trajectory always start at zero at the beginning of a move.
	TrajPos = 0;
	MoveMod = 0;

	//Determine direction of move.
	TrajDir = (MoveLength >= 0) ? 1 : -1;

	lastTrajRealPos = TrajRealPos;
	TrajRP_Sum = 0;
	AvgVelSum = 0;
	SumPtr = 0;

	if(MoveType == INFINITE)
	{
		MoveLength = 0;
		TrajState = TRAP_ACCEL;
		TrajTick = TrapAccel;
		VelStepUp = TrajAccel;			//Range 1pm/s to 100mm/s(100000000000 pm)
		p_step = TargetVel;
		TargetVel_n = TargetVel * 1000;

		//Need to define this parameter so the JOG operation is able to stop using STP command (or reverse jog direction)
		VelStepDown = TrajDecel;

		MotionReady = 1;
		return 1;
	}
	//Positive Motion = +1
	//Negative Motion = -1
	if(MoveLength == 0) return 0;

	TrajTarget = MoveLength * TrajDir;	//Traj Target is always positive.

	StartPos = CurrentPos;		//StartPos is an absolute position. Read in DRIVER_SETUP in Servo.c.
	TargetPos = CurrentPos + MoveLength;	//We're going to assume the value of this addition is valid here.
	//Determine step size per trajectory cycle) while moving @ target velocity	(Step size during CONSTANT_VELOCITY)
	//	p_step = TargetVel / TrajFrequency - assuming same unit base for velocity and distance (EX. distance in mm. velocity in mm/s.)
	// However in our case, our distance units are 1000 times smaller than our velocity units...(distance in pm. velocity in nm/s.)
	//So.. p_step = (TargetVel / 1000)nm  *  (1000pm / 1nm) = TargetVel.  Works out nicely.
	p_step = TargetVel;

	//The velocity increments in the accel and decel phase are done in pm/s because we're handling
	//velocity increments per ms.  The smallest increment we will deal with is 1pm/s. (acc = 1nm/s/s. velocity increment in 1ms is 1pm/s)
	TargetVel_n = TargetVel * 1000;		//Target velocity in pm/s

	VelSq = TargetVel * TargetVel;	//This calculation limits system velocity.	MAX equals floor(sqrt(2^64)) == 4.295 meters/second

	//Determine our velocity increments(up/accel & down/decel) @ trajectory rate:
	//(How much do we increase velocity every trajectory period during accel, and how much do we decrease during decel)
	//acc(nm/s/s) * .001s(our trajectory rate) =  (|acc|/1000) (nm/s)
	//convert to pm/s because the min increment is 1pm/s (acc = 1nm/s/s).
	//(|acc|/1000) (nm/s) * (1000pm/1nm) =(|acc| pm/s). converting to
	//pm gets rid of the division operation. It's works out almost too perfectly. Bless you metric system.

	VelStepUp = TrajAccel;			//Range 1pm/s to 1mm/s(1000000000 pm)	For a system max of 1 meter per second.
	VelStepDown = TrajDecel;		//Same

	//If TrajAccel > VelSq., we fully accelerate in 1 trajectory cycle
	AccelDistance = (int64_t)VelSq / (int64_t)TrajAccel / 2;	//These distances are in nanometers.
	DecelDistance = VelSq / TrajDecel / 2;	//dx = (nm/s * nm/s) * s^2/nm / 2 = nm.

	StopVelStep = VelStepDown;	//nm/s

	//If we pick our MaxVel a little better, this addition can never overflow
	TrapMinDistance = AccelDistance + DecelDistance;

	//Is our MoveLength longer than our accel+decel distance?
	if((TrajTarget / 1000) > TrapMinDistance)
	{
		TrajState = TRAP_ACCEL;
		TrajTick = TrapAccel;

		//This probably won't overflow. We have already detected a trapezoidal profile,
		//which means our decel distance is within our Travel Range and it is in nm.
		//which has 1000 times less resolution than our position variables.
		//DecelDistance * 1000 could overflow.
		//Just make sure the signed/unsigned arithmetic comes out right.
		//DecelDistance is unsigned.
						 //+ only		//+ only	//This multiplication CAN overflow b.
		DescentPoint = TrajTarget - (DecelDistance * 1000);	//No TrajDir shit here.

		DecelTicks = TargetVel_n / VelStepDown;
		if(DecelTicks == 0)
		{
			DecelTicks = 1;
		}
	}
	else
	{
		TrajState = TRI_ACCEL;
		TrajTick = TriAccel;
		DescentPoint = TrajTarget - (DecelDistance * 1000);		//in case TriAccel accidently gets into ConstantVel.
		DecelTicks = TargetVel_n / VelStepDown;
		if(DecelTicks == 0)
		{
			DecelTicks = 1;
		}
		//If we're doing a triangular profile, DecelTicks is calculated at the top point of the triangle.(TRI_ACCEL state)
		//And there is no DecelPoint. We check every trajectory cycle for our decel point.
	}

	MotionReady = 1;

	return 1;
}

//Buffer variables used when changing velocity or target on the fly
signed char new_dir;
int64_t new_StartPos;
int64_t new_TargetPos;
int64_t new_DescentPoint;
int64_t new_DistanceToTarget;
uint64_t new_VelSq;
uint64_t new_p_step;
uint64_t new_DecelTicks;
uint64_t new_MoveDistance;
uint64_t new_TargetVel_n;
int64_t new_AccelDistance;
uint64_t new_DecelDistance;
int64_t new_TrapMinDistance;
uint64_t new_TargetVel;
uint64_t new_TrajTarget;

int64_t (*new_TrajTick)();

void LoadNewValues()
{
	//This function latches in the new trajectory info
	//once the calculations are completed. Called in TrajectoryController.
	p_step = new_p_step;
//	StartPos = new_StartPos;			//We don't change StartPos ever. Do we?
	AccelDistance = new_AccelDistance;
	DecelDistance = new_DecelDistance;
	TrapMinDistance = new_TrapMinDistance;
	DescentPoint = new_DescentPoint;
	TrajState = new_TrajState;
	TrajTick = *new_TrajTick;	//Is this right? I don't know.
	TargetPos = new_TargetPos;
	TrajTarget = new_TrajTarget;
	DecelTicks = new_DecelTicks;
	TargetVel = new_TargetVel;
	TargetVel_n = new_TargetVel_n;
	ReleaseModLock();

	//TrajDir = new_dir;	if direction could change in PCOF
}
unsigned char ModInProgress = 0;
void ReleaseModLock()
{
	ModInProgress = 0;
}

void SetModLock()
{
	ModInProgress = 1;
}

enum MotionModes getMotionMode()
{
	return MotionMode;
}

uint8_t vcof_count = 0;

uint8_t VelocityCOF_Setup(int64_t NewVel)
{
	//dir is the same, acc and dec are the same, target is the same.
	//need to step up or down to a new target velocity.
	vcof_count++;
	if(vcof_count == 2)
	{
		vcof_count = 0;
	}

	if( MotionMode == JOG_)
	{
		if((NewVel > 0 && TrajDir != 1) || (NewVel < 0 && TrajDir != -1))
		{
			AddError(CANNOT_CHANGE_DIRECTION,"JOG");
			return 0;
		}
		if(NewVel < 0)
		{
			NewVel = -NewVel;
		}
	}

	if(NewVel == TargetVel) return 0;	//If target vel == current vel don't do shit. No error in this case.

	if(TrajState != CONSTANT_VELOCITY) return 0;	//Error in this case.
	new_TargetPos = TargetPos;		//new_TargetPos == absolute.
	new_TargetVel = NewVel;
	new_TrajTarget = TrajTarget;

	new_p_step = NewVel;		//pm
	new_TargetVel_n = NewVel * 1000;
	new_VelSq = NewVel * NewVel;

	//Determine Magnitude of Moves
	if(TrajDir == 1)
	{							//new_Tart
		new_DistanceToTarget = new_TargetPos - CurrentPos;	//CurrentPos will change before this gets loaded.
		//if( new_DistanceToTarget < 0)//we are already passed our target. something is fucked up.
	}
	else
	{
		new_DistanceToTarget = CurrentPos - new_TargetPos;
		//if( new_DistanceToTarget < 0)//we are already passed our target. something is fucked up.
	}

	if(NewVel > TargetVel)
	{	//Our new velocity is greater than our old velocity.
		new_AccelDistance = ((int64_t)new_VelSq / (int64_t)TrajAccel / 2) - AccelDistance;	//Why.
		new_DecelDistance = new_VelSq / TrajDecel / 2;

 		new_TrapMinDistance = (int64_t)new_AccelDistance + (int64_t)new_DecelDistance;			//new_TrapMinDistance here holds the distance required to
																				//1)accel to our new velocity, plus
																				//2)the distance required to decel to target from new velocity

		if( ((new_DistanceToTarget / 1000) > new_TrapMinDistance) || (MotionMode == JOG_) )
		{//If we're here, we can accelerate to our new Velocity
			new_TrajState = TRAP_ACCEL;
			new_TrajTick = TrapAccel;
			//Why Traj Dir here? why TrajDir. Look at this.	//This multiplication CAN overflow. Check it out tomorrow.
			new_DescentPoint = new_TrajTarget - (new_DecelDistance * 1000);	//DecelDistance is in nm. DescentPoint needs to be in pm.
			//DecelPoint is ALWAYS in reference to the positive TrajTarget value.

			new_DecelTicks = new_TargetVel_n / VelStepDown;		//Unless we do change acceleration on fly, VelStepUp/Down will never change.
			if(new_DecelTicks == 0)
			{
				new_DecelTicks = 1;
			}
		}
		else
		{
			new_TrajState = TRI_ACCEL;
			new_TrajTick = TriAccel;
			new_DecelTicks = 0;		//These two things will get calculated during the TRI ACCEL phase of motion.
			new_DescentPoint = 0;
		}
	}
	else
	{
		//Our new velocity is less than our current velocity. We should have no issues with our decel point.
		new_TrajState = TRAP_DECEL;
		new_TrajTick = TrapDecel;
		new_DecelDistance = new_VelSq / Decel / 2;
		new_DescentPoint = new_TrajTarget - (int64_t)(new_DecelDistance*1000);	//pm.
	}

	MoveMod = 1;		//This move has been modified.
	LoadTrajVals = 1;	//Signal Trajectory to load values.
	SetModLock();
	return 1;
}

//PCOF	<--This needs to be rewritten for the new relative trajectory.
//Trajectory moves positive only. Errors are logged within this function.
//int64_t Adj_Log[5] = {0};
//uint8_t adj_ptr = 0;
uint8_t PositionCOF_Setup(int64_t Adj)
{
	//Handle Direction change.
	uint64_t DecelDistance_pm = DecelDistance * 1000;		// DecelDistance stays the same. We want to compare it to how far from the
																	//new target we currently are.
	//Adj is relative +/-

//	Adj_Log[adj_ptr++] = Adj;
//	if(adj_ptr == 5)
//	{
//		adj_ptr = 0;
//	}

	vcof_count++;

	if(vcof_count == 2)
	{
		vcof_count = 0;
	}

	//TargetPos is my absolute target position
	if(TrajState != CONSTANT_VELOCITY) return 0;

//	if(NewTarget == TargetPos) return 0;
	if(Adj == 0) return 0;

	new_dir = (TargetPos + Adj > CurrentPos) ? 1 : -1;
	if(TrajDir != new_dir)
	{
		//AddError(CANNOT_CHANGE_DIRECTION_DURING_MOVE, cmd);
		return DIR_CHANGE;
	}
	//Set new absolute target.
	new_TargetPos = TargetPos + Adj;

//Our trajectory is always moving positive. If we are moving negative, we need to
//reverse the sign of our Trajectory Target Adjustment.
	if( TrajDir == -1 && Adj < 0)
	{	//What
		Adj = -Adj; //No. Keep Negative? What does it mean for this to be > 0
	}
	else if( TrajDir == -1 && Adj > 0 )
	{
		Adj = -Adj;
	}

	new_TrajTarget = TrajTarget + Adj;	//<--Any direction change should already be caught. But Adj is the wrong sign.

	//	new_TrajTarget = TrajDir * NewTarget;	//Get rid of negative sign.	This doesn't work asshole.

	new_DescentPoint = new_TrajTarget - DecelDistance_pm;

	//Where is our new target relative to our old target?
	if(new_TrajTarget > TrajTarget || TrajPos <= new_DescentPoint)
	{	//Target is further away than original target, or we have not passed the descent point for our new target
		//good to go.
	}
	else if(TrajPos > new_DescentPoint)
	{	//We have passed the decel point for our new target.
		//Should we: a) Decel Faster than the set value
		//			 b) Disallow position change here.
		//Let's try b) for now.
		//vcof_count = 0;
			//		new_TrajState = DECEL;
			//		new_TrajTick = Decelerate;
	}

	//These values don't change:
	new_p_step = p_step;
	new_AccelDistance = AccelDistance;
	new_DecelDistance = DecelDistance;
	new_TrapMinDistance = TrapMinDistance;
	new_TargetVel = TargetVel;
	new_TrajState = TrajState;	//Constant Velocity
	new_TrajTick = TrajTick;

	MoveMod = 1;
	LoadTrajVals = 1;
	SetModLock();
	return 1;
}

//Add new parameters
//Set more reasonable default parameters.
void WriteDefaults()
{
	Vel = 1000000;
	Accel = 1000000000;			//nm/s/s
	Decel = 1000000000;			//nm/s/s
	HomeVel = 2000000;
	tlp = 10000000000LL;
	tln = -10000000000LL;
	JogAccel = 1000000000;
	AccelMax = 10000000000;
	KpWhole = 0;
	KpFrac = 10;
	KiWhole = 0;
	KiFrac = 10;
	KdWhole = 0;
	KdFrac = 100;
	EncoderResolution = 5000;
	DAC_Res = 5000;
	DeadbandEncoderCounts = 2;
	DeadbandTimeout = 0;
	FeedbackMode = OPEN_LOOP;
	AddressingMode = 0;
	StartupPGM = 0;
	MPol = 0;
	HomeConfig = 0;
	MotorActive = 0;
	ProgAvail = 0;
	LimitPolarity = 0;
	LimitConfig = 0;
	EncoderChannelSelect = DIGITAL;
	HardStopDeadband = 10;
	EncoderDetectionVal = 8000;

//--TODO ported non-volatile parameters
//	SaveParameters();
}

enum HomeStates { 	SETUP1=0,
					SETUP_REDUX,
					WFSTART1,
					WFSTOP1,
					MOVE1_MONITOR,
					FOUND1,
					SETUP2,
					WFSTART2,
					MOVE2_MONITOR,
					DELAY2,
					WFSTOP2,
					SETUP3,
					WFSTART3,
					MOVE3_MONITOR,
					DELAY3,
					WFSTOP3,
					SETUP4,
					WFSTART4,
					MOVE4_MONITOR,
					WFSTOP4,
					DELAY4,
					HOME_CLEANUP,
					BACK_OUT
				}HomeState = SETUP1;


enum HomeStates HomeStateLog[25];
uint8_t HomeStatePtr = 0;

enum HomeMoves {	MOVE1,
					MOVE2,
					MOVE3,
					MOVE4,
					MOVE_TO_LIMIT,
					CANCEL_HOME,
			   }HomeMove = MOVE1;

volatile int64_t IndexLatch[20];
volatile uint16_t indx = 0;
uint8_t IndexFound = 0;
volatile uint16_t Homed = 0;
volatile uint8_t StopAtIndex = 0;
volatile uint8_t home_type = 0;
uint8_t save_de;
enum FeedbackModes old_fm;
volatile uint8_t old_fl;
uint8_t nextmove = 1;
uint8_t StillLooking = 0;
uint8_t home_dir;
uint8_t HomeErr = 0;
uint16_t DelayCounter = 0;
volatile uint8_t _HomeInProgress;
extern uint8_t No_Encoder;
volatile uint16_t *LimitCheck = 0;
uint32_t old_dbWindow;
uint32_t old_dbTimeoutCounts;
int64_t MoveDistance=0;
volatile int32_t approach;
uint32_t FirstTime = 0;
int64_t Move1Length,Move2Length,Move3Length,Move4Length;

void NoEncoderFound()
{
	if(MOTION_IN_PROGRESS)
	{
		StopMotion();
	}

	if(_HomeInProgress){
		HomeMove = CANCEL_HOME;
	}

	FeedbackMode = OPEN_LOOP;
	DetectEncoder = 0;
}

uint8_t HomeInProgress()
{
	return _HomeInProgress;
}

uint8_t isHomed()
{
	return Homed;
}

void SetupHome(enum MoveTypes mt)
{
	HomeState = SETUP1;
	HomeMove = MOVE1;
	MoveType = mt;
	MotionMode = MOVE_;
	Homed = 0;
	_HomeInProgress = 1;
	TargetVel = HomeVel;
	TrajAccel = HomeAccel;
	TrajDecel = HomeAccel;
//--TODO Implement GPIO Interrupt for index pulse
//!	NVIC_EnableIRQ(QEI_IRQn);
	HomeStatePtr = 0;
	htim5.Instance->CNT = 0;
	__HAL_TIM_SET_AUTORELOAD(&htim5, 0x5DC);
	__HAL_TIM_ENABLE(&htim5);
}

uint8_t BackOut = 0;
uint8_t limit = 0;
void SetupMoveToLimit(uint8_t ldir)
{
	limit = ldir;
	_HomeInProgress = 1;
	HomeState = SETUP1;
	HomeMove = MOVE_TO_LIMIT;
	TrajAccel = HomeAccel;
	TrajDecel = HomeAccel;
	TargetVel = HomeVel;
	HomeStatePtr = 0;
	htim5.Instance->CNT = 0;
	__HAL_TIM_SET_AUTORELOAD(&htim5, 0x5DC);
	__HAL_TIM_ENABLE(&htim5);
}

void CancelHome()
{
	//Turn off timer interrupt, restore settings
	htim5.Instance->CR1 &= ~(TIM_CR1_CEN);
//--TODO Implement External Interrupt for index pin
//!	NVIC_DisableIRQ(QEI_IRQn);
	FeedbackMode = old_fm;
	DetectEncoder = save_de;
	HardstopDetectEnable = old_fl;
	HomeState = SETUP1;
	HomeMove = MOVE1;
	_HomeInProgress = 0;
	DeadbandWindow = old_dbWindow;
	DeadbandTimeoutCounts = old_dbTimeoutCounts;
	StopAtIndex = 0;
}

void setTrajState(enum MotionPhases ts)
{	//Only allow setting to NO_MOTION for now.
	if(ts == NO_MOTION)
	{
		TrajState = NO_MOTION;
		TrajTick = NoMotion;
	}
}

uint32_t read_lsn = 0;
uint32_t read_lsp = 0;
uint32_t temp1, temp2;
uint32_t last_state_neg = 0;
uint32_t last_state_pos = 0;

void UpdateLimitState(){
	if(LimitConfig == 2 || LimitConfig == 3)
	{
//--TODO Read Limit Pins
		temp1 = LIMIT_POS_IN;
		temp2 = LIMIT_NEG_IN;

		if(LimitDir){
			read_lsp = temp2;
			read_lsn = temp1;
		}
		else
		{
			read_lsp = temp1;
			read_lsn = temp2;
		}

		if(read_lsp == LimitPolarity)
		{
			InPositiveLimit = 1;
			if((LimitConfig & HARD_LIMIT_MASK) && TrajState != NO_MOTION)
			{
				EmergencyStop();
			}
		}
		else
		{
			InPositiveLimit = 0;
		}
	}

	if(read_lsn == LimitPolarity)
	{
		InNegativeLimit = 1;
		if((LimitConfig & HARD_LIMIT_MASK) && TrajState != NO_MOTION)
		{
			EmergencyStop();
		}
	}
	else
	{
		InNegativeLimit = 0;
	}

	last_state_pos = read_lsp;
	last_state_neg = read_lsn;

	if(InPositiveLimit || InNegativeLimit)
	{
		InLimit = 1;
	}
	else
	{
		InLimit = 0;
	}
}

//Lower Priority than the servo.
//QEI Index Interrupt has highest priority.

volatile uint8_t SkipFirstMove = 0;

//Interrupt-Based Home Routine
void TIM5_IRQHandler(void)
{
	//Clear interrupt flag
	__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);

	if(HomeStateLog[HomeStatePtr] != HomeState)
	{
		HomeStateLog[++HomeStatePtr] = HomeState;
		if(HomeStatePtr == 25)
		{
			HomeStatePtr = 0;
		}
	}

//MOVE 1
//===============================================================
	if(HomeMove == MOVE1)
	{
		if(HomeState == SETUP1)
		{
			//start first move
			//watch out for index and hard stop
			//if index delay1 then decelerate
			//if move1 was in + direction goto 3.  StillLooking flag set if hardstop hit.
			//else if move1 was in - direction and found index goto 4, if found hardstop goto 2

			HomeErr = 0;
			old_fm = FeedbackMode;
			save_de = DetectEncoder;
			old_fl = HardstopDetectEnable;

			old_dbWindow = DeadbandWindow;
			old_dbTimeoutCounts = DeadbandTimeoutCounts;

			//Switch to Open Loop and look out for an encoder.
			DetectEncoder = 1;
			FeedbackMode = OPEN_LOOP;
			StillLooking = 1;
			IndexFound = 0;
			FirstTime = 1;
			SkipFirstMove = 0;
		//Decide what direction to move.

		//If rotary, need to go at least 360 degrees
			if(HomeConfig==1)
			{
				if(LimitConfig & SOFT_LIMIT_MASK)
				{
					Move1Length = tlp - CurrentPos;		//If we are using soft limits, we can
				}
				else
				{
					Move1Length = 900000000000LL;
				}
			}
			else
			{
				if(LimitConfig & SOFT_LIMIT_MASK)
				{
					Move1Length = tln - CurrentPos;
				}
				else
				{
					Move1Length = -900000000000LL;	//<-- set excessively large
				}
			}

			if(LimitConfig & HARD_LIMIT_MASK)
			{
				LimitCheck = &InLimit;
				if(HomeConfig == 0 && InNegativeLimit){
					//Moving -, but already in limit
					SkipFirstMove = 1;
				}
				else if(HomeConfig == 1 && InPositiveLimit){
					//Moving +, but already in limit. Skip first move
					SkipFirstMove = 1;
				}
			}
			else
			{
				LimitCheck = &HardStopFound;
				HardStopFound = 0;
				HardstopDetectEnable = 1;
			}

			if(SkipFirstMove){
				HomeMove = MOVE2;
				HomeState = SETUP2;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x753);
			}
			else{

				if(MotionProfileSetup(Move1Length))
				{
					StartMove();
				}
				else
				{
					//MotionProfileSetup did not complete.
					//only reason for a false return if movelength == 0 (already at soft limit)
					AddError(MOVE_OUTSIDE_SOFT_LIMITS,"HOM");
					HomeMove = CANCEL_HOME;
				}
				//Set the Frequency of MOVE1 Monitoring
				HomeState = MOVE1_MONITOR;
				//==Set Home Controller Timer Period
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x300);

			}
		}
		else if( HomeState == SETUP_REDUX )
		{
			//Always move positive here.
			if( LimitConfig & SOFT_LIMIT_MASK )
			{
				Move1Length = tlp - CurrentPos;
			}
			else
			{
				Move1Length = 900000000000LL;		//900 mm
			}

			IndexFound = 0;
			HardStopFound = 0;
			HardstopDetectEnable = 1;	//This could change, based on how I decide to do hard stop detection.
			FirstTime = 0;
			if(MotionProfileSetup(Move1Length))
			{
	 			StartMove();
		 	}
		 	else
		 	{	//==EXIT_PATH
		 		HomeMove = CANCEL_HOME;
		 	}
			//home_dir = 1;
			HomeState = MOVE1_MONITOR;

			return;
		}
		//No Wait For Start Here
		else if(HomeState == MOVE1_MONITOR)
		{
			if(IndexFound)
			{
				//We have seen our first index. Stop and prepare to turn around.
				HomeState = FOUND1;
				StillLooking = 0;
				if(HomeConfig == 0 && FirstTime)
				{
					MoveDistance = HomeDistance2;
					approach = HOME_DISTANCE2;
				}
				else
				{
					MoveDistance = HomeDistance1;
					approach = HOME_DISTANCE1;
				}
				TrajState = STOP;
				TrajTick  = Stop;

//				LPC_TIM3->MR0 = 0x5FFF;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x5FFF);
				return;
			}
			else if(TrajState == NO_MOTION || *LimitCheck)	//<-- LimitCheckB.
			{
				//Trajectory takes care of hard stop deceleration
				HomeState = WFSTOP1;
				StillLooking = 1;
				HardStopFound = 0;
			}
			else if(No_Encoder)
			{	//==EXIT_PATH
				No_Encoder = 0;
				TrajTick = Stop;
				TrajState = STOP;
				NoEncoderInHome = 1;
				HardstopDetectEnable = 0;		//If there is no encoder, hard stop detection should be off.
				HomeMove = CANCEL_HOME;
				return;
			}
		}
		else if(HomeState == WFSTOP1)
		{
			if(TrajState == NO_MOTION && DriverState == DRIVER_OFF)
			{
				HomeMove = MOVE2;
				HomeState = SETUP2;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x753);
			}
			else return;
		}
		else if(HomeState == FOUND1)
		{
			if(DriverState == DRIVER_OFF)
			{
				IndexFound = 0;
				HomeMove = MOVE3;
				HomeState = SETUP3;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x753);
			}
		}
	}

	else if(HomeMove == MOVE2)
	{
		if(HomeState == SETUP2)
		{
			if(HomeConfig == 0)
			{
				if(LimitConfig & SOFT_LIMIT_MASK)
				{
					Move2Length = tlp - CurrentPos;
				}
				else
				{
					Move2Length = 800000000000LL;
				}
				MoveDistance = HomeDistance1;
				approach = HOME_DISTANCE1;
			}
			else
			{
				if(LimitConfig & SOFT_LIMIT_MASK)
				{
					Move2Length = tln - CurrentPos;
				}
				else
				{
					Move2Length = -800000000000LL;
				}
				MoveDistance = HomeDistance2;
				approach = HOME_DISTANCE2;
			}

			IndexFound = 0;
			if(!(LimitConfig & HARD_LIMIT_MASK))
			{
				HardStopFound = 0;
				HardstopDetectEnable = 1;
			}

			if(MotionProfileSetup(Move2Length))
			{
				StartMove();
			}
			else
			{
				AddError(MOVE_OUTSIDE_SOFT_LIMITS,"HOM");
				HomeMove = CANCEL_HOME;
			}
			HomeState = WFSTART2;
		}
		else if(HomeState == WFSTART2)
		{
			//We're already stopped.
			if(DriverState != DRIVER_OFF)
			{
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x400);
				HomeState = MOVE2_MONITOR;
			}
		}
		else if(HomeState == MOVE2_MONITOR)
		{
			//Positive direction. Still looking for index.
			if(!IndexFound)
			{
				if(No_Encoder)
				{
					No_Encoder = 0;
					TrajState = STOP;
					TrajTick = Stop;
					HardstopDetectEnable = 0;
					NoEncoderInHome = 1;
					HomeMove = CANCEL_HOME;
					return;
				}
				if(HardStopFound)
				{	//==EXIT_PATH
					IndexNotFound = 1;
					HardStopFound = 0;
					CurrentPos = RealPosition;
					//Turn off home. It failed.
					HomeMove = CANCEL_HOME;
					return;
				}
				if(DriverState == DRIVER_OFF)
				{	//==EXIT_PATH
					IndexNotFound = 1;
					CurrentPos = RealPosition;
					HomeMove = CANCEL_HOME;
					return;
				}
			}
			else
			{
				HomeState = WFSTOP2;
				TrajTick = Stop;
				TrajState = STOP;
			}
		}
		else if(HomeState == WFSTOP2)
		{
			if(TrajState == NO_MOTION && DriverState == DRIVER_OFF)
			{
				HomeMove = MOVE3;
				HomeState = SETUP3;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x5DC);
			}
		}
	}

	else if(HomeMove == MOVE3)
	{
		if(HomeState == SETUP3)
		{
			//start move in - direction
			//watch out for index
			//if index, delay3 then decelerate

			IndexFound = 0;
			HardstopDetectEnable = 1;
			HardStopFound = 0;
			TargetVel = HomeVel / 2;

			DeadbandWindow = EncoderResolution * 2;
			DeadbandTimeoutCounts = 0;

			CurrentPos = RealPosition;

			FeedbackMode = OPEN_LOOP_CLOSED_FINISH;

			if(MotionProfileSetup(-MoveDistance))
			{
				StartMove();
			}
			else
			{
				HomeMove = CANCEL_HOME;
			}
			HomeState = WFSTART3;

			return;
		}
		else if(HomeState == WFSTART3)
		{
			if(DriverState != DRIVER_OFF)
			{
				HomeState = MOVE3_MONITOR;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x400);
			}
		}

		else if(HomeState == MOVE3_MONITOR)
		{
			if(DriverState != DRIVER_OFF)
			{
				if(No_Encoder)
				{	//==EXIT_PATH
					No_Encoder = 0;
					TrajState = STOP;
					TrajTick = Stop;
					NoEncoderInHome = 1;
					HomeMove = CANCEL_HOME;
					return;
				}
			}
			else
			{
				FeedbackMode = OPEN_LOOP;

				if( approach == HOME_DISTANCE2 )
				{
					HomeState = SETUP_REDUX;
					HomeMove = MOVE1;
				}
				else
				{
					HomeState = SETUP4;
					HomeMove = MOVE4;
				}
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x1800);
			}
			return;
		}
	}
	else if(HomeMove == MOVE4)
	{
		if(HomeState == SETUP4)
		{
			//Final Move
			//Start move in + direction
			//watch out for index.
			//terminate motion when found.

			TargetVel = SlowHVel;

			StopAtIndex = 1;
			IndexFound = 0;
			HardstopDetectEnable = 0;
			//5mm in positive direction.

			Move4Length = 5000000000LL;
			indx = 0;
			if(MotionProfileSetup(Move4Length))
			{
				StartMove();
			}
			else
			{	//==EXIT_PATH
				HomeMove = CANCEL_HOME;		//Add error flag
			}
			HomeState = MOVE4_MONITOR;
		}
		else if(HomeState == WFSTART4)
		{
			if(DriverState != DRIVER_OFF)
			{
				HomeState = MOVE4_MONITOR;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x200);
			}
			return;
		}
		else if(HomeState == MOVE4_MONITOR)
		{
			if(!IndexFound)
			{
				if(No_Encoder)
				{	//==EXIT_PATH
					No_Encoder = 0;
					TrajState = STOP;
					TrajTick = Stop;
					NoEncoderInHome = 1;
					HomeMove = CANCEL_HOME;
					return;
				}

				if(DriverState == DRIVER_OFF && !IndexFound)
				{	//==EXIT_PATH
					IndexNotFound = 1;
					CurrentPos = RealPosition;
					HomeMove = CANCEL_HOME;
				}
			}
			else
			{
				CurrentPos = IndexLatch[0];	//Why IndexLatch[0]?

				//Close the loop on the latched position

				HomeState = WFSTOP4;
				AllGood = 0;	//<-- Don't seem to use this anymore.
			}
			//break;
		}
		else if(HomeState == WFSTOP4)
		{
			if(DriverState == DRIVER_OFF)
			{
				if(old_fm == OPEN_LOOP || old_fm == CLEAN_OPEN_LOOP)
				{
					FeedbackMode = CLOSED_LOOP;
					HomeState = DELAY4;
					htim5.Instance->CNT = 0;
					__HAL_TIM_SET_AUTORELOAD(&htim5, 0x5FFF);
					DelayCounter = 0;
				}
				else
				{
					HomeState = HOME_CLEANUP;
				}
			}
			return;
		}
		else if(HomeState == DELAY4)
		{
			if(DelayCounter++ >= 150)
			{
				DelayCounter = 0;
				HomeState = HOME_CLEANUP;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x250);
			}
		}
		else if(HomeState == HOME_CLEANUP)
		{
			htim5.Instance->CNT = 0;
			DeadbandWindow = old_dbWindow;
			DeadbandTimeoutCounts = old_dbTimeoutCounts;
			Homed = 1;
			_HomeInProgress = 0;
			FeedbackMode = old_fm;
			DetectEncoder = save_de;
			HardstopDetectEnable = old_fl;
		}
	}
	else if(HomeMove == CANCEL_HOME)
	{
		//==Turn off HomeController
		htim5.Instance->CNT = 0;
		//--TODO Implement GPIO Interrupt for index pulse
		//NVIC_DisableIRQ(QEI_IRQn);
		HomeState = SETUP1;
		HomeMove = MOVE1;

		//==Restore State
		FeedbackMode = old_fm;
		DetectEncoder = save_de;
		HardstopDetectEnable = old_fl;
		DeadbandWindow = old_dbWindow;
		DeadbandTimeoutCounts = old_dbTimeoutCounts;

		_HomeInProgress = 0;
		StopAtIndex = 0;
	}
	else if(HomeMove == MOVE_TO_LIMIT)
	{
		if(HomeState == SETUP1)
		{
			if(limit)
			{
				if(LimitConfig & SOFT_LIMIT_MASK)
				{
					Move1Length = tlp - CurrentPos;
				}
				else
				{
					Move1Length = 800000000000LL;
				}
			}
			else
			{
				if(LimitConfig & SOFT_LIMIT_MASK)
				{
					Move1Length = tln - CurrentPos;
				}
				else
				{
					Move1Length = -800000000000LL;
				}
			}

			old_fm = FeedbackMode;
			save_de = DetectEncoder;
			old_fl = HardstopDetectEnable;

			FeedbackMode = OPEN_LOOP;
			DetectEncoder = 1;

			if(LimitConfig & HARD_LIMIT_MASK)
			{
				LimitCheck = &InLimit;
			}
			else
			{
				LimitCheck = &HardStopFound;
				HardstopDetectEnable = 1;
				HardStopFound = 0;
			}

			TargetVel = HomeVel;

			if(MotionProfileSetup(Move1Length))
			{
				StartMove();
				//Could I block here?
			}
			else
			{
				//MotionProfileSetup return false
				//Only reason for false return is hte move length = 0
				if (limit) AddError(MOVE_OUTSIDE_SOFT_LIMITS,"MLP");
				else AddError(MOVE_OUTSIDE_SOFT_LIMITS,"MLN");
				HomeMove = CANCEL_HOME;		//Add error flag.
			}
			HomeState = WFSTART1;
			return;
		}
		else if(HomeState == WFSTART1)
		{
			if(DriverState != DRIVER_OFF)
			{
				HomeState = MOVE1_MONITOR;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x5DC);
			}
			return;
		}
		else if(HomeState == MOVE1_MONITOR)
		{
			if(DriverState != DRIVER_OFF)
			{
				if(No_Encoder)
				{	//==EXIT_PATH
					TrajState = STOP;
					TrajTick = Stop;
					HardstopDetectEnable = 0;
					HardStopFound = 0;
					HomeMove = CANCEL_HOME;
				}
			}
			else
			{
				HomeState = BACK_OUT;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0x1D4C);
			}
			return;
		}
		else if(HomeState == BACK_OUT)
		{
			if(!BackOut)
			{
				DetectEncoder = save_de;
				FeedbackMode = old_fm;
				HardstopDetectEnable = old_fl;
				//MotionProfileSetup return false
				//Only reason for false return is hte move length = 0
				if (CurrentPos == tlp) AddError(MOVE_OUTSIDE_SOFT_LIMITS,"MLP");
				else if (CurrentPos == tln) AddError(MOVE_OUTSIDE_SOFT_LIMITS,"MLN");
				CurrentPos = RealPosition;
				_HomeInProgress = 0;
				htim5.Instance->CNT = 0;
				return;
			}
			else
			{
				if(limit)
				{
					MoveDistance = -LIMIT_REBOUND;
				}
				else
				{
					MoveDistance = LIMIT_REBOUND;
				}

				if(MotionProfileSetup(MoveDistance))
				{
					StartMove();
				}
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_AUTORELOAD(&htim5, 0xC00);
				HomeState = WFSTART2;
			}
		}
		else if(HomeState == WFSTART2)
		{
			if(DriverState != DRIVER_OFF)
			{
				HomeState = WFSTOP2;
			}
			return;
		}
		else if(HomeState == WFSTOP2)
		{
			if(DriverState == DRIVER_OFF)
			{
				_HomeInProgress = 0;
				CurrentPos = RealPosition;
				DetectEncoder = save_de;
				FeedbackMode = old_fm;
				HardstopDetectEnable = old_fl;
				htim5.Instance->CNT = 0;
			}
		}
	}
}

//--TODO Convert to gpio interrupt for index

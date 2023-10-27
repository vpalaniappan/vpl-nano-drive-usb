/*
 * motion.h
 *
 *  Created on: May 31, 2023
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#define FPGA_DAC_MAX 4095

#define SYSTEM_MAX_ACCEL 4000000000ULL				//!<Maximum allowable acceleration 1meter/second.
#define SYSTEM_MAX_VEL 100000000ULL // Setting Max velocity to 100 mm/sec
#define MAX_REZ 100000
#define DIR_CHANGE 10

#define SERVOS_PER_MS 10
#define SERVO_FREQ 10000
#define SERVO_FREQ_DIV_1K (SERVO_FREQ / 1000)

#define ENC_FILTER_MOTION_MAX 512
#define ENC_FILTER_STANDSTILL_MAX 512

#define ENC_FILTER_MOTION_DEFAULT 5
#define ENC_FILTER_STANDSTILL_DEFAULT 10

//========Limit switch stuff
#define SOFT_LIMIT_MASK 1
#define HARD_LIMIT_MASK 2
#define NO_LIMITS_ENABLED 0
#define SOFT_LIMITS_ONLY 1
#define HARD_LIMITS_ONLY 2
#define BOTH_HARD_AND_SOFT 3

//!Driver States
enum DriverStates{
					DRIVER_STARTUP=0,
					DRIVER_OFF,			//!<Driver is OFF
					DRIVER_SETUP,		//!<Driver is in setup mode between off and on
					DRIVER_ON,			//!<Driver is ON
					EXTERNAL_MOD,
					DRIVER_CLEANUP
				 };

enum MoveTypes{
					TARGETED=0,
					INFINITE
				 };


enum StopType{
				NORMAL_STOP,
				EMERGENCY_STOP
};

enum LimitConfigs{
					SOFT_LIMITS_DISABLED,
					HARD_LIMITS_DISABLED,
					HARD_STOP_DETECTION_ON

				 };

enum EncoderTypes{
					DIGITAL = 0,
					ANALOG,
					ABSOLUTE
				  };			/*<This determines which Encoder Channel is read*/

//!Operating Modes
enum FeedbackModes{
						OPEN_LOOP=0,				/*!<Open Loop*/
						CLEAN_OPEN_LOOP,			/*!<Better sounding open loop, less accurate*/
						OPEN_LOOP_CLOSED_FINISH,	/*!<Closed loop decel*/
						CLOSED_LOOP					/*!<Closed Loop*/
				  };

//!Motion Modes
enum MotionModes{
					JOG_,			/*!<Jog Mode*/
					MOVE_			/*!<Move Mode*/
				 };

enum MotionPhases{
					TRAP_ACCEL,			//!<Trapezoidal Acceleration Phase
					TRAP_DECEL,			//!<Trapezoidal Deceleration Phase
					TRI_ACCEL,			//!<Triangular Acceleration Phase
					CONSTANT_VELOCITY,	//!<Constant Velocity Phase
					DECEL,				//!<Deceleration Phase
					NO_MOTION,			//!<No Motion Phase
					STOP				//!<Stop Phase
				  };

enum TraceModes{
					SERVO_BASED,
					EXT_TRIGGER_BASED
				};

enum MotorModes{
					MOTOR_PIEZO_MODE = 0,
					MOTOR_STEPPER_MODE,
					MOTOR_DC_MODE,
					MOTOR_BLDC_MODE
};

void AxisSetup();

extern uint64_t Vel;				//!<Axis Travel Velocity (nm / s)			/ 	(millidegree / second)
extern uint64_t Accel;				//!<Axis Acceleration 	/   (millidegree / second / second)
extern uint64_t Decel;				//!<Axis Deceleration. (nm/s)
extern uint64_t JogAccel;			//!<Axis Jog Accel/Decel. (nm/s).
extern uint64_t AccelMax;			//!<Axis Max Acceleration /	(nm/s)
extern uint64_t HomeAccel;			//!<Home acceleration son
extern uint64_t StartupPos;			//!<Startup Position
extern uint8_t StartupPosEnable;	//!<Enable/disable startup position
extern uint64_t VelMax;				//!<Max Allowable Velocity

extern int64_t tlp;					//!<Axis Positive Software Limit Position (pm)
extern int64_t tln;					//!<Axis Negative Software Limit Position (pm)	/	(microdegree / 2)
extern uint8_t LimitPolarity;
extern uint8_t LimitConfig;
extern uint8_t LimitDir;
extern uint8_t QEI_FilterLevel;
extern uint8_t SilentModeEnable;
extern uint8_t SyncAfterMoveDone;

extern uint8_t AddressingMode;

extern uint32_t EncoderResolution;
extern uint8_t EncoderPolarity;
extern enum EncoderTypes EncoderChannelSelect;
extern uint32_t EncoderDetectionVal;
extern uint32_t DeadbandEncoderCounts;
extern uint32_t DeadbandTimeout;

extern int32_t InPositionCounts;
extern int32_t InPositionTime;

extern volatile enum FeedbackModes FeedbackMode;
extern uint16_t KpWhole;
extern uint16_t KpFrac;
extern uint16_t KiWhole;
extern uint16_t KiFrac;
extern uint16_t KdWhole;
extern uint16_t KdFrac;
extern uint32_t FeedForwardParameter;

extern uint8_t MPol;
extern uint8_t MotorActive;
extern uint32_t DAC_Res;			//Or StageResolution...DAC Steps Per Micron

extern uint8_t StartupPGM;
extern uint32_t ProgAvail;

extern uint8_t HomeConfig;
extern uint64_t HomeDistance1;
extern uint64_t HomeDistance2;
extern uint64_t HomeVel;			//!<Axis Home Velocity (nm / second)			/ 	(millidegree / second)
extern uint64_t SlowHVel;
extern uint64_t ClLoopCorVelMax;
extern uint32_t HardStopDeadband;

extern uint8_t IO_Pol;
extern uint8_t IO_Func;
extern uint16_t ProgRunTimes[32];
extern uint8_t ProgOutputRoute;

extern int64_t EncoderZeroOffset;
extern uint8_t HardstopDetectEnable;

extern int32_t maxError;
extern uint16_t iTermSampleTime;
extern uint16_t dTermSampleTime;
extern uint32_t iTermSumLimit;
extern int32_t dTermDiffLimit;
extern uint32_t KpWhole2;
extern uint32_t KpFrac2;

extern uint8_t voltageRefMode;
extern uint8_t motorControlMode;
extern uint8_t motorPitch;
extern uint32_t motorResistance;
extern uint32_t motorInductance;
extern uint32_t iMax;
extern uint32_t i2Time;
extern uint32_t iOffset;

extern uint16_t silentModeOpFreq;
extern uint16_t silentModeThreshold;
extern uint16_t waveformOffset_P2;
extern uint16_t waveformOffset_P3;
extern uint16_t waveformOffset_P4;
extern uint16_t waveformAmplitude;

extern uint16_t EncFilterWindow_Motion;
extern uint16_t EncFilterWindow_Standstill;

extern uint16_t waveformFETOnTime;
extern uint16_t waveformAmpDisable;
extern uint16_t waveformAmpEnable;

extern uint16_t VelocityPulseEnable;
extern int64_t absoluteEncoderOffset;

extern uint8_t CANCommEnable;

#endif /* INC_MOTION_H_ */

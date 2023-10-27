/*
 * dio.h
 *
 *  Created on: May 31, 2023
 */

#ifndef INC_DIO_H_
#define INC_DIO_H_

//Digital I/O Direction Setting
#define IOD_OUTPUT 0
#define IOD_INPUT 1

#define OUTPUT_PULSE_WIDTH 8

//Digital I/O Functions.
#define IOF_DIGITAL 				0		//General purpose
#define IOF_HW_TRACE 				1		//Trigger trace operation (TRA operation)
#define IOF_PULSE_IN_POSITION 		2		//Pulse once motor in position
#define IOF_LEVEL_IN_POSITION 		3		//Change level once motor in position
#define IOF_CONSTANT_VELOCITY_PULSE	4		//Pulse at constant velocity (CVP)
#define IOF_POSITION_TRIGGER_PULSE	5		//Pulse at target position (PTP)
#define IOF_POSITION_INTERVAL_PULSE	6		//Pulse at position intervals (PIP)

#define IOF_POSITION_LATCH 		4		//Trigger a position latch (CAP feature)
#define IOF_MOTOR_ENABLE 		5
#define IOF_PULSE_IN 			6
#define IOF_DIR_IN 				7
#define IOF_ENCODER_OUT 		8
#define IOF_ENCODER_IN 			9
#define IOF_HOME_TRIGGER 		10

//Limit Switches, and index.
//#define Come up with a couple more.

//How to do driver mode? How can ARM get the pulse count AND read an encoder with only one QEI.

//PULSE IN POSITION
//	1 pulse when a move is over (open loop) or we are in position (closed loop)

//LEVEL IN POSITION
//  1 means in position, 0 means not.
//	Continuous.

//Alternate method:

//OUTPUT FUNCTIONS
//Feel free to add a couple.
#define OF_DIGITAL 					0
#define OF_PULSE_IN_POSITION		1
#define OF_LEVEL_IN_POSITION		2
#define OF_ENCODER_OUT				3
#define OF_HOME_STATUS				4

//INPUT FUNCTIONS
#define IF_DIGITAL 					0
#define IF_POSITION_SAMPLE_TRIGGER 	1
#define	IF_ENCODER_IN				2
#define IF_PULSE_IN					3
#define IF_DIR_IN					4
#define	IF_HOME_TRIGGER				5
#define IF_MOTOR_ENABLE				6

//DIO1 =  O = P0[20]	pin 58
//DIO2 =  I = P1[8]		pin 92
//DIO3 =  I = P0[29]	pin 29
//DIO4 =  O = P2[11]	pin 52
//DIO5 =  O = P2[12]	pin 51
//DIO6 =  I = P2[13]	pin 50

/*

//Digital I/O Shortcuts=========
//Digital I/O #1	P0[20]
#define DIO1_SET LPC_GPIO0->FIOSET2 |= 0x10
#define DIO1_CLR LPC_GPIO0->FIOCLR2 |= 0x10
#define DIO1_READ (LPC_GPIO0->FIOPIN2 & 0x10)>>4
#define DIO1_DIR_CLEAR LPC_GPIO0->FIODIR2 &= ~(1<<4)
#define DIO1_SET_INPUT LPC_GPIO0->FIODIR2 &= ~(1<<4)
#define DIO1_SET_OUTPUT LPC_GPIO0->FIODIR2 |= 0x10
#define DIO1_TOGGLE LPC_GPIO0->FIOPIN2 ^= 0x10

//Digital I/O #2	P1[8]
#define DIO2_SET LPC_GPIO1->FIOSET1 |= 0x01
#define DIO2_CLR LPC_GPIO1->FIOCLR1 |= 0x01
#define DIO2_READ (LPC_GPIO1->FIOPIN1 & 0x01)
#define DIO2_DIR_CLEAR LPC_GPIO1->FIODIR1 &= ~(1<<0)
#define DIO2_SET_INPUT LPC_GPIO1->FIODIR1 &= ~(1<<0)
#define DIO2_SET_OUTPUT LPC_GPIO1->FIODIR1 |= 0x01
#define DIO2_TOGGLE LPC_GPIO1->FIOPIN1 ^= 0x01

//Digital I/O #3	P0[29]
#define DIO3_SET LPC_GPIO0->FIOSET3 |= 0x20
#define DIO3_CLR LPC_GPIO0->FIOCLR3 |= 0x20
#define DIO3_READ (LPC_GPIO0->FIOPIN3 & 0x20)>>5
#define DIO3_DIR_CLEAR LPC_GPIO0->FIODIR3 &= ~(1<<5)
#define DIO3_SET_INPUT LPC_GPIO0->FIODIR3 &= ~(1<<5)
#define DIO3_SET_OUTPUT LPC_GPIO0->FIODIR3 |= 0x20
#define DIO3_TOGGLE LPC_GPIO0->FIOPIN3 ^= 0x20

//Digital I/O #4	P2[11]
#define DIO4_SET LPC_GPIO2->FIOSET1 |= 0x08
#define DIO4_CLR LPC_GPIO2->FIOCLR1 |= 0x08
#define DIO4_READ (LPC_GPIO2->FIOPIN1 & 0x08)>>3
#define DIO4_DIR_CLEAR LPC_GPIO2->FIODIR1 &= ~(1<<3)
#define DIO4_SET_INPUT LPC_GPIO2->FIODIR1 &= ~(1<<3)
#define DIO4_SET_OUTPUT LPC_GPIO2->FIODIR1 |= 0x08
#define DIO4_TOGGLE LPC_GPIO2->FIOPIN1 ^= 0x08
//DIO4 TriState Buffer Controller - P2[6]
#define DIO4_SET_T LPC_GPIO2->FIOSET0 |= 0x40
#define DIO4_CLR_T LPC_GPIO2->FIOCLR0 |= 0x40

//Digital I/O #5	P2[12]
#define DIO5_SET LPC_GPIO2->FIOSET1 |= 0x10
#define DIO5_CLR LPC_GPIO2->FIOCLR1 |= 0x10
#define DIO5_READ (LPC_GPIO2->FIOPIN1 & 0x10)>>4
#define DIO5_DIR_CLEAR LPC_GPIO2->FIODIR1 &= ~(1<<4)
#define DIO5_SET_INPUT LPC_GPIO2->FIODIR1 &= ~(1<<4)
#define DIO5_SET_OUTPUT LPC_GPIO2->FIODIR1 |= 0x10
#define DIO5_TOGGLE LPC_GPIO2->FIOPIN1 ^= 0x10
//DIO5 TriState Buffer Controller - P2[5]
#define DIO5_SET_T LPC_GPIO2->FIOSET0 |= 0x20
#define DIO5_CLR_T LPC_GPIO2->FIOSET0 |= 0x20

//Digital I/O #6	P2[13]
#define DIO6_SET LPC_GPIO2->FIOSET1 |= 0x20
#define DIO6_CLR LPC_GPIO2->FIOCLR1 |= 0x20
#define DIO6_READ (LPC_GPIO2->FIOPIN1 & 0x20)>>5
#define DIO6_DIR_CLEAR LPC_GPIO2->FIODIR1 &= ~(1<<5)
#define DIO6_SET_INPUT LPC_GPIO2->FIODIR1 &= ~(1<<5)
#define DIO6_SET_OUTPUT LPC_GPIO2->FIODIR1 |= 0x20
#define DIO6_TOGGLE LPC_GPIO2->FIOPIN1 ^= 0x20
//DIO6 TriState Buffer Controller - P2[4]
#define DIO6_SET_T_OUTPUT LPC_GPIO2->FIODIR0 |= 0x10
#define DIO6_SET_T LPC_GPIO2->FIOSET0 |= 0x10
#define DIO6_CLR_T LPC_GPIO2->FIOCLR0 |= 0x10

*/

//--Functions
void Init_IOFunctions();
void UpdateLimitState();
char ReadIO_State(uint8_t);
char UpdateIO_Func(uint8_t, uint8_t);
char UpdateIO_State(uint8_t, uint8_t);
uint8_t WaitForIO_Active();
void SetupWIO(uint8_t,uint8_t);
extern char (*UpdateIO_Dir[])(uint8_t);	//Array of function pointers.

extern volatile uint8_t startPulse[6];

#endif /* INC_DIO_H_ */

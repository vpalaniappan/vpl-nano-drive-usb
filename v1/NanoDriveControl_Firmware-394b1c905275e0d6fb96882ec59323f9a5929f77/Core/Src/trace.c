/*
 * trace.c
 *
 *  Created on: Aug 21, 2023
 *      Author: tjudge
 */

#include "main.h"
#include "motion.h"
#include "trace.h"

TraceData traceLog[MAX_TRACE_SAMPLES];

uint32_t traceDone = 0;		//Flag that indicates that the desired # of sample points have been taken and written to flash. Operation complete.
uint32_t traceSample = 0;		//# of samples taken in the current trace operation

uint16_t getPointsTaken()
{
	return traceSample;
}

void SetupTrace(unsigned int NumSamples, unsigned int SampleFreq, long long StartPos)
{
	//We are storing trace data in local RAM. STM32F405 has 192KB of SRAM available
	points = 0;
	traceDone = 0;
	traceSample = 0;

	T_StartPos = StartPos;
	T_Samples = NumSamples;
	T_ServosPerSample = SampleFreq;

	startTrace = 1;
}

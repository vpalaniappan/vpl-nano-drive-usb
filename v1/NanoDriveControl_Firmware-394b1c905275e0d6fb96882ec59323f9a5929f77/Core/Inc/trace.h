/*
 * trace.h
 *
 *  Created on: Aug 21, 2023
 *      Author: tjudge
 */

#ifndef INC_TRACE_H_
#define INC_TRACE_H_

#define MAX_TRACE_SAMPLES	1000

typedef struct
{
	int32_t realPosition;
	int32_t trajPosition;
	int32_t dacAdjustment;
	uint16_t motorADC;

}TraceData;

extern TraceData traceLog[MAX_TRACE_SAMPLES];
extern volatile uint8_t traceOn;
extern uint32_t traceDone;
extern uint32_t traceSample;
extern volatile uint32_t startTrace;

extern volatile int64_t  T_StartPos;
extern volatile uint32_t T_Samples;
extern volatile uint32_t T_ServosPerSample;

extern volatile uint32_t points;

uint16_t getPointsTaken();
uint8_t TraceComplete();
void SetupTrace(unsigned int NumSamples, unsigned int SampleFreq, long long StartPos);

#endif /* INC_TRACE_H_ */

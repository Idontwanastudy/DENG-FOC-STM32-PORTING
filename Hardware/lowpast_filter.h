#ifndef __LOWPAST_FILTER
#define __LOWPAST_FILTER
#include "system.h"

typedef struct
{
	float Tf;
	float y_prev;
	uint64_t timestamp_prev;
}LowPast_Filter;

float LowPassFilter_operation(float x , LowPast_Filter *FLT);
void LowPassFilter_Init(float time_constant, LowPast_Filter *FLT);

#endif

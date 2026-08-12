#include "lowpast_filter.h"
#include "REALTIME.h"
#include "stm32f10x.h"


void LowPassFilter_Init(float time_constant, LowPast_Filter *FLT)
{
	FLT->Tf=time_constant;
	FLT->y_prev=0;
	FLT->timestamp_prev = micros();
}

float LowPassFilter_operation(float x , LowPast_Filter *FLT)
{
	uint64_t timestamp = micros();
	float dt = ( timestamp - FLT->timestamp_prev)*1e-6f;
	
	if(dt < 0.0f) dt = 1e-3f;
	else if (dt > 0.5f)
	{
		FLT->y_prev = x;
		FLT->timestamp_prev=timestamp;
		return x;
	}
	float alpha = FLT->Tf / (FLT->Tf + dt);
	float y=alpha * FLT->y_prev + (1.0f - alpha) * x;
	FLT->y_prev = y;
	FLT->timestamp_prev = timestamp;
	return y;
}

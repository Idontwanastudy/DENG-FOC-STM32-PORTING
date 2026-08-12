#ifndef __PID_H
#define __PID_H
#include "system.h"

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Err0;
	float Err1;
	
	float Kp;
	float Ki;
	float Kd;
	
	float output_ramp;//速度限制
	
	float OutMax;//电压输出限制
	float OutMin;
	
	float integral_prev;
	float output_prev;
	
	uint64_t timestamp_prev;
} PID_t;

void PID_Clear(PID_t *p);
void PID_Update(PID_t *p);
void PID_SetUp(PID_t *p, float Kp, float Ki, float Kd, float ramp, float limit);

#endif

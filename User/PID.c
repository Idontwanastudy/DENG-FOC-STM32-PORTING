#include "PID.h"
#include "math.h"
#include "REALTIME.h"
#include "system.h"

void PID_Clear(PID_t *p)
{
	p->Err0 = 0;
	p->Err1 = 0;
}

void PID_Update(PID_t *p)
{
	uint64_t timestamp_now = micros();
	float Ts = (timestamp_now - p->timestamp_prev) * 1e-6f;
	if(Ts<=0 || Ts>0.5) Ts = 1e-3f;
	p->Err0 = p->Target - p->Actual;
	
	float Pout= p->Kp * p->Err0;
	float Iout = p->integral_prev + p->Ki * Ts * 0.5 * (p->Err0 + p->Err1);
	if (Iout > p->OutMax) {Iout = p->OutMax;}
	if (Iout < p->OutMin) {Iout = p->OutMin;}
	float Dout = p->Kd * (p->Err0 - p->Err1)/Ts;
	float output= Pout + Iout + Dout;
	if (output > p->OutMax) {output = p->OutMax;}
	if (output < p->OutMin) {output = p->OutMin;}
	
	if(p->output_ramp > 0)
	{
		float output_rate = (output - p->output_prev)/Ts;
		if(output_rate > p->output_ramp)
		{
			output = p->output_prev + p->output_ramp*Ts;
		}
		else if(output_rate < -p->output_ramp)
		{
			output = p->output_prev - p->output_ramp*Ts;
		}
	}
	p->integral_prev = Iout;
	p->output_prev = output;
	p->Err1 = p->Err0;
	p->timestamp_prev = timestamp_now;
	p->Out = output;
}

void PID_SetUp(PID_t *p, float Kp, float Ki, float Kd, float ramp, float limit)
{
	p->Kp = Kp;
	p->Ki = Ki;
	p->Kd = Kd;
	p->output_ramp = ramp;
	p->OutMin = -1*limit;
	p->OutMax = limit;
}
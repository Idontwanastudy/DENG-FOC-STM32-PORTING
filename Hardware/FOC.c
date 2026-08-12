#include "FOC.h"
#include "PID.h"
#include <math.h>
#include "AS5600.h"
#include "SysTick.h"
#include "Serial.h"
#include "AD.h"
#include "lowpast_filter.h"
#include "Motor.h"
#include "REALTIME.h"

#define PI 3.1415926f
#define _3PI_2 4.71238898038f
#define power_supply 12.0f
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float S0_zero_electric_angle=0;
float open_loop_timestamp=0;
int M0_PP=7,M0_DIR=1;
float shaft_angle;
float Cmd_Num_Array[5];
uint8_t Cmd_Stack[2]={0};
uint8_t Setpid_Cmd[3]={4,5,6};
uint8_t Show_Cmd[4]={7,8,9,10};
int flag=1;

float normalizeAngle(float angle)
{
	float a = fmod(angle , 2*PI);
	return a >= 0 ? a : (a + 2*PI);
}

void FOC_M0_SET_VEL_PID(PID_t *VEL,float P, float I, float D, float ramp, float limit)
{
	VEL->Kp = P;
	VEL->Ki = I;
	VEL->Kd = D;
	VEL->output_ramp = ramp;
	VEL->OutMax = limit;
}

void FOC_M0_SET_ANGLE_PID(PID_t *ANGLE, float P, float I, float D, float ramp, float limit)
{
	ANGLE->Kp = P;
	ANGLE->Ki = I;
	ANGLE->Kd = D;
	ANGLE->output_ramp = ramp;
	ANGLE->OutMax = limit;
}

void FOC_M0_SET_CURRENT_PID(PID_t *CURRENT, float P, float I, float D, float ramp, float limit)
{
	CURRENT->Kp = P;
	CURRENT->Ki = I;
	CURRENT->Kd = D;
	CURRENT->output_ramp = ramp;
	CURRENT->OutMax = limit;
}


void M0_setPWM(float Ua , float Ub, float Uc)
{
	Ua = _constrain(Ua , 0.0f , power_supply);
	Ub = _constrain(Ub , 0.0f , power_supply);
	Uc = _constrain(Uc , 0.0f , power_supply);
	
	float dc_a= _constrain(Ua/power_supply , 0.0f , 1.0f);
	float dc_b= _constrain(Ub/power_supply , 0.0f , 1.0f);
	float dc_c= _constrain(Uc/power_supply , 0.0f , 1.0f);
	
	Motor_SetPWM(1, dc_a);
	Motor_SetPWM(2, dc_b);
	Motor_SetPWM(3, dc_c);
}

void M0_setTorque(float Uq, float angle_el)//angle_el电角度
{
	Uq=_constrain(Uq, -(power_supply/2), power_supply/2);
	float Ud = 0;
	angle_el = normalizeAngle(angle_el);
	float Ualpha = -Uq*sin(angle_el);//帕克逆变换
	float Ubeta = Uq*cos(angle_el);
	
	float Ua = Ualpha + power_supply/2;
	float Ub = (sqrt(3)*Ubeta-Ualpha)/2 + power_supply/2;
	float Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + power_supply/2;
	M0_setPWM(Ua,Ub,Uc);
}

//开环相电压
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	angle_el = normalizeAngle(angle_el + S0_zero_electric_angle);
	float Ualpha = -Uq*sin(angle_el);//帕克逆变换
	float Ubeta = Uq*cos(angle_el);
	
	float Ua = Ualpha + power_supply/2;
	float Ub = (sqrt(3)*Ubeta-Ualpha)/2 + power_supply/2;
	float Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + power_supply/2;
	M0_setPWM(Ua,Ub,Uc);
}

float S0_electricalAngle()
{
	return normalizeAngle((float)(M0_DIR * M0_PP) * getMechanicalAngle() - S0_zero_electric_angle);
	
}
float _electricalAngle(float shaft_angle, int pole_pairs)
{
	return (shaft_angle * pole_pairs);
}

void FOC_M0_alignSensor(int _PP, int _DIR)
{
	M0_PP = _PP;
	M0_DIR = _DIR;
	M0_setTorque(3 , _3PI_2);//起劲
	delay_ms(1000);
	Sensor_update();
	S0_zero_electric_angle=S0_electricalAngle();
	M0_setTorque(0,_3PI_2);//松劲
//	Serial_Printf("M0 0电角度：");
//	Serial_Print_float_number(S0_zero_electric_angle);
}

float FOC_M0_Angle()
{
	return M0_DIR * getAngle();
}

float cal_Iq_Id(float current_a, float current_b, float angle_el)
{
	float I_alpha = current_a;
	float I_beta = (2*current_b + current_a)/sqrt(3.0f);
	
	float ct = cos(angle_el);
	float st = sin(angle_el);
	
	float I_q= ct * I_beta - st * I_alpha;
	return I_q;
}
//开环
float velocityOpenloop(float target_velocity)
{
	uint64_t now_us = micros();
	float Ts = (now_us -open_loop_timestamp) * 1e-6f;
	if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
	shaft_angle = normalizeAngle(shaft_angle + target_velocity*Ts);
	float Uq = 10;              
	setPhaseVoltage(Uq, 0,_electricalAngle(shaft_angle, 7));
	open_loop_timestamp = now_us;
	return Uq;
}

float FOC_M0_Current(float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt)
{
	Get_Phase_Currents(current_a,current_b);
	float current_A = *current_a;
	float current_B = *current_b;
	float I_q_M0_oringin = cal_Iq_Id(current_A, current_B, S0_electricalAngle());
	float I_q_M0_filter = LowPassFilter_operation(I_q_M0_oringin, M0_Curr_Flt);
	return I_q_M0_filter;
}

float FOC_M0_Velocity(LowPast_Filter *M0_Vel_Flt)
{
	float vel_M0_oringin = getVelocity();
	float vel_M0_filter = LowPassFilter_operation(M0_DIR * vel_M0_oringin, M0_Vel_Flt);
	return vel_M0_filter;
}
//电流力矩环
void FOC_M0_setTorque(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt, PID_t *M0_Current)
{
	M0_Current->Target = Target;
	M0_Current->Actual = FOC_M0_Current(current_a, current_b, M0_Curr_Flt);
	PID_Update(M0_Current);
	M0_setTorque(M0_Current->Out,S0_electricalAngle());
}
//力-速度-角度环
void FOC_M0_set_Velcocity_Angle(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_Angle, PID_t *M0_VEL, PID_t *M0_Current)
{
	M0_Angle->Target = Target /** 180 / PI*/;
	M0_Angle->Actual = FOC_M0_Angle() /** 180 / PI*/;
	PID_Update(M0_Angle);
	M0_VEL->Target = M0_Angle->Out;
	M0_VEL->Actual = FOC_M0_Velocity(M0_Vel_Flt);
	PID_Update(M0_VEL);
	FOC_M0_setTorque(M0_VEL->Out, current_a, current_b, M0_Curr_Flt, M0_Current);
}
//速度环
void FOC_M0_setVelocity(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_VEL, PID_t *M0_Current)
{
	M0_VEL->Target = Target /* * 180 / PI*/;
	M0_VEL->Actual = FOC_M0_Velocity(M0_Vel_Flt) /** 180 / PI*/;
	PID_Update(M0_VEL);
	FOC_M0_setTorque(M0_VEL->Out, current_a, current_b, M0_Curr_Flt, M0_Current);
}

void FOC_M0_set_Force_Angle(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt, PID_t *M0_Angle, PID_t *M0_Current)
{
	M0_Angle->Target = Target /** 180 / PI*/;
	M0_Angle->Actual = FOC_M0_Angle() /** 180 / PI*/;
	PID_Update(M0_Angle);
	FOC_M0_setTorque(M0_Angle->Out, current_a, current_b, M0_Curr_Flt, M0_Current);
}

void RUN_FOC(float *current_a, float *current_b)
{
	Sensor_update();
	Get_Phase_Currents(current_a, current_b);
}

void Cmd_operation(float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_VEL, PID_t *M0_Angle, PID_t *M0_Current)
{
	uint8_t cmd = Cmd_Compare();
	if(Usart_flag==1)
	{
		flag = 1;
		Usart_flag = 0;
	}
	else if(Usart_flag==0)
		flag = 0;
	//设置速度角度的时候
	if (IS_in(cmd)==0 && Cmd_Num_Array[0]!=114514)
	{
		if(cmd!=0)
		Get_real_num_array(Cmd_Num_Array);
		Cmd_Stack[0]=cmd;
		Start_cmd(Cmd_Stack[0], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
	}
	//主任务不为错误的时候，展示参数的时候，先展示参数，后回归执行，flag置0
	else if(IS_in(cmd)==2 && flag == 1 && Cmd_Stack[0]!=0 && Cmd_Num_Array[0]!=114514)
	{
		Cmd_Stack[1]=cmd;
		Start_cmd(Cmd_Stack[1], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
		Start_cmd(Cmd_Stack[0], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
	}
	//主任务不为错误的时候，设置参数的时候，先设置参数，再展示参数，后回归执行，flag置0
	else if(IS_in(cmd)==1 && flag == 1 && Cmd_Stack[0]!=0 && Cmd_Num_Array[0]!=114514)
	{
		Get_real_num_array(Cmd_Num_Array);
		Cmd_Stack[1]=cmd;
		Start_cmd(Cmd_Stack[1], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
		Start_cmd(7, current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
		Start_cmd(Cmd_Stack[0], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
	}
	//主任务为错误时，设置参数的时候，先设置参数，再展示参数，后flag置0，打印错误信息，USART停止刷新
	else if(IS_in(cmd)==1 && flag == 1 && Cmd_Stack[0]==0 && Cmd_Num_Array[0]!=114514)
	{
		Get_real_num_array(Cmd_Num_Array);
		Cmd_Stack[1]=cmd;
		Start_cmd(Cmd_Stack[1], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
		Start_cmd(7, current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
		Start_cmd(0, current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
	}
	//主任务为错误时，展示参数的时候，先展示参数，后flag置0，打印错误信息，USART停止刷新
	else if(IS_in(cmd)==2&& flag == 1 && Cmd_Stack[0]==0 && Cmd_Num_Array[0]!=114514)
	{
		Cmd_Stack[1]=cmd;
		Start_cmd(Cmd_Stack[1], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
		Start_cmd(0, current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
	}
	
	//若主任务不为错误时，flag为0时候一直运行主任务
	else if(IS_in(cmd)!=0 && flag == 0 && Cmd_Stack[0] != 0 && Cmd_Num_Array[0]!=114514)
	{
		Start_cmd(Cmd_Stack[0], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
	}
	//若主任务不为错误时，flag为0时候不做任何动作
	else if(IS_in(cmd)!=0 && flag == 0 && Cmd_Stack[0] == 0 && Cmd_Num_Array[0]!=114514);

	//参数错误时，打印错误标志
	else if(Cmd_Num_Array[0] == 114514)
	{
		Start_cmd(0, current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Angle, M0_Current);
	}
}


void Start_cmd(uint8_t cmd, float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_VEL, PID_t *M0_Angle, PID_t *M0_Current)
{
		switch(cmd){
				case 0:		Serial_SendString("ERRO!\n"); break;
				case 1:		FOC_M0_setVelocity(Cmd_Num_Array[0], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_VEL, M0_Current); break;
				case 2:		FOC_M0_set_Velcocity_Angle(Cmd_Num_Array[0], current_a, current_b, M0_Vel_Flt, M0_Curr_Flt, M0_Angle, M0_VEL, M0_Current);break;
				case 3:		setprog(Cmd_Num_Array[0]); break;
				case 4:		SetPID(M0_VEL, Cmd_Num_Array[0], Cmd_Num_Array[1], Cmd_Num_Array[2], Cmd_Num_Array[3], Cmd_Num_Array[4]); break;
				case 5:		SetPID(M0_Angle, Cmd_Num_Array[0], Cmd_Num_Array[1], Cmd_Num_Array[2], Cmd_Num_Array[3], Cmd_Num_Array[4]); break;
				case 6:		SetPID(M0_Current, Cmd_Num_Array[0], Cmd_Num_Array[1], Cmd_Num_Array[2], Cmd_Num_Array[3], Cmd_Num_Array[4]); break;
				case 7:		Show_Pid_Parameters(M0_VEL, M0_Angle, M0_Current); break;
				case 8:		Show_Vel(); break;
				case 9:		Show_Angle(); break;
				case 10:	Show_Current(M0_Current); break;
			}
}

uint8_t IS_in(uint8_t cmd)
{
	int i=0;
	for(i=0;i<3;i++)
	{
		if(cmd==Setpid_Cmd[i])
			return 1;
	}
	for(i=0;i<4;i++)
	{
		if(cmd==Show_Cmd[i])
			return 2;
	}
	return 0;
}

void setprog(float num)
{
	
}

void Show_Pid_Parameters(PID_t *M0_VEL, PID_t *M0_Angle, PID_t *M0_Current)
{
	Serial_SendString("Velocity_PID_Parameters:\n");
	Serial_SendString("Kp:"); Serial_Print_float_number(M0_VEL->Kp);Serial_SendString("; ");
	Serial_SendString("Ki:"); Serial_Print_float_number(M0_VEL->Ki);Serial_SendString("; ");
	Serial_SendString("Kd:"); Serial_Print_float_number(M0_VEL->Kd);Serial_SendString(";\n");
	Serial_SendString("Ramp:"); Serial_Print_float_number(M0_VEL->output_ramp);Serial_SendString("; ");
	Serial_SendString("Limit:"); Serial_Print_float_number(M0_VEL->OutMax);Serial_SendString(";\n");
	Serial_SendString("Angle_PID_Parameters:\n");
	Serial_SendString("Kp:"); Serial_Print_float_number(M0_Angle->Kp);Serial_SendString("; ");
	Serial_SendString("Ki:"); Serial_Print_float_number(M0_Angle->Ki);Serial_SendString("; ");
	Serial_SendString("Kd:"); Serial_Print_float_number(M0_Angle->Kd);Serial_SendString(";\n");
	Serial_SendString("Ramp:"); Serial_Print_float_number(M0_Angle->output_ramp);Serial_SendString("; ");
	Serial_SendString("Limit:"); Serial_Print_float_number(M0_Angle->OutMax);Serial_SendString(";\n");
	Serial_SendString("Current_PID_Parameters:\n");
	Serial_SendString("Kp:"); Serial_Print_float_number(M0_Current->Kp);Serial_SendString("; ");
	Serial_SendString("Ki:"); Serial_Print_float_number(M0_Current->Ki);Serial_SendString("; ");
	Serial_SendString("Kd:"); Serial_Print_float_number(M0_Current->Kd);Serial_SendString(";\n");
	Serial_SendString("Ramp:"); Serial_Print_float_number(M0_Current->output_ramp);Serial_SendString("; ");
	Serial_SendString("Limit:"); Serial_Print_float_number(M0_Current->OutMax);Serial_SendString(";\n\n");
}

void Show_Vel(void)
{
	Serial_SendString("Velocity:");Serial_Print_float_number(getVelocity());Serial_SendString(";\n");
}

void Show_Angle(void)
{
	Serial_SendString("Angle:");Serial_Print_float_number(Sensor_GetAngle());Serial_SendString(";\n");
}

void Show_Current(PID_t *M0_Current/*float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt*/)
{
//	Get_Phase_Currents(current_a,current_b);
//	float current_A = *current_a;
//	float current_B = *current_b;
//	float I_q_M0_oringin = cal_Iq_Id(current_A, current_B, S0_electricalAngle());
//	float I_q_M0_filter = LowPassFilter_operation(I_q_M0_oringin, M0_Curr_Flt);
	Serial_SendString("Current:");Serial_Print_float_number(M0_Current->Actual);Serial_SendString(";\n");
}

void SetPID(PID_t *PID, float Kp, float Ki, float Kd, float ramp , float limit)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->output_ramp = ramp;
	PID->OutMax = limit;
	PID->OutMin = -1*limit;
}

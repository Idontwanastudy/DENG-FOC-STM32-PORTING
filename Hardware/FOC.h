#ifndef __FOC
#define __FOC
#include "system.h"
#include "FOC.h"
#include "PID.h"
#include <math.h>
#include "AS5600.h"
#include "SysTick.h"
#include "Serial.h"
#include "AD.h"
#include "lowpast_filter.h"
#include "Motor.h"

extern float S0_zero_electric_angle;
extern float open_loop_timestamp;
extern int M0_PP,M0_DIR;
extern float shaft_angle;
extern float Cmd_Num_Array[5];
extern uint8_t Cmd_Stack[2];
extern uint8_t Setpid_Cmd[3];
extern uint8_t Show_Cmd[4];
extern int flag;


float normalizeAngle(float angle);
void FOC_M0_SET_VEL_PID(PID_t *VEL,float P, float I, float D, float ramp, float limit);
void FOC_M0_SET_ANGLE_PID(PID_t *ANGLE, float P, float I, float D, float ramp, float limit);
void FOC_M0_SET_CURRENT_PID(PID_t *CURRENT, float P, float I, float D, float ramp, float limit);
void M0_setPWM(float Ua , float Ub, float Uc);
void M0_setTorque(float Uq, float angle_el);
float S0_electricalAngle(void);
void FOC_M0_alignSensor(int _PP, int _DIR);
float FOC_M0_Angle(void);
float cal_Iq_Id(float current_a, float current_b, float angle_el);
float FOC_M0_Current(float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt);
float FOC_M0_Velocity(LowPast_Filter *M0_Vel_Flt);
void FOC_M0_setTorque(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt, PID_t *M0_Current);
void FOC_M0_set_Velcocity_Angle(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_Angle, PID_t *M0_VEL, PID_t *M0_Current);
void FOC_M0_setVelocity(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_VEL, PID_t *M0_Current);
void FOC_M0_set_Force_Angle(float Target, float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt, PID_t *M0_Angle, PID_t *M0_Current);
void RUN_FOC(float *current_a, float *current_b);
float velocityOpenloop(float target_velocity);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
float _electricalAngle(float shaft_angle, int pole_pairs);
void Cmd_operation(float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_VEL, PID_t *M0_Angle, PID_t *M0_Current);
void setprog(float num);
void SetPID(PID_t *PID,float Kp, float Ki, float Kd, float ramp , float limit);
void Start_cmd(uint8_t cmd, float *current_a, float *current_b, LowPast_Filter *M0_Vel_Flt, LowPast_Filter *M0_Curr_Flt, PID_t *M0_VEL, PID_t *M0_Angle, PID_t *M0_Current);
uint8_t IS_in(uint8_t cmd);
void Show_Pid_Parameters(PID_t *M0_VEL, PID_t *M0_Angle, PID_t *M0_Current);
void Show_Vel(void);
void Show_Angle(void);
void Show_Current(PID_t *M0_Current/*float *current_a, float *current_b, LowPast_Filter *M0_Curr_Flt*/);



#endif

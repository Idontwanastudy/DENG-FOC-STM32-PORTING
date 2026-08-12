#include "system.h"
#include "stm32f10x.h"                  // Device header
#include "SysTick.h"
#include "PWM.h"
#include "AD.h"
#include "REALTIME.h"
#include "Motor.h"
#include "AS5600.h"
#include "lowpast_filter.h"
#include "FOC.h"
#include "Serial.h"
#include "PID.h"

#define power_supply 12.0f
#define _PI 3.1415926535

float angle00;
long int i = 0;
float current_a;
float current_b;
float out_vel,out_angle,out_current;
float target_vel,target_angle,target_current;
//uint64_t TIME[6];
uint8_t cmd;

int main(void)
{
		SysTick_Init(72);
		Motor_Init();
		Serial_Init();
		Sensor_init();
		AD_Init();
	
	LowPast_Filter M0_Vel_Flt;
	LowPassFilter_Init(0.01 , &M0_Vel_Flt);
	LowPast_Filter M0_Curr_Flt;
	LowPassFilter_Init(0.05 , &M0_Curr_Flt);
	
	PID_t vel_loop_M0;
	 {
		vel_loop_M0.Kp = 0.1; vel_loop_M0.Ki = 2; vel_loop_M0.Kd = 0;//速度
		/*vel_loop_M0.Kp = 0.02; vel_loop_M0.Ki = 1; vel_loop_M0.Kd = 0;*///角度
	  vel_loop_M0.output_ramp = 100000; vel_loop_M0.OutMax = 2;
	  vel_loop_M0.OutMin = -2; vel_loop_M0.timestamp_prev = micros();
	  vel_loop_M0.output_prev=0;}
	PID_Clear(&vel_loop_M0);
	
	PID_t angle_loop_M0;
	 {
		angle_loop_M0.Kp = 10; angle_loop_M0.Ki = 1; angle_loop_M0.Kd=0.01;
	  angle_loop_M0.output_ramp = 100000; angle_loop_M0.OutMax = 30;
	  angle_loop_M0.OutMin = -30; angle_loop_M0.timestamp_prev = micros();
	  angle_loop_M0.output_prev=0;}
	PID_Clear(&angle_loop_M0);
	
	PID_t current_loop_M0;
		{
		 current_loop_M0.Kp = 0.5; current_loop_M0.Ki =50; current_loop_M0.Kd = 0;//速度
		 /*current_loop_M0.Kp = 5; current_loop_M0.Ki =200; current_loop_M0.Kd = 0;*///角度
		 current_loop_M0.output_ramp = 100000; current_loop_M0.OutMax = power_supply;
		 current_loop_M0.OutMin = -power_supply ;current_loop_M0.timestamp_prev = micros();
		 current_loop_M0.output_prev=0;}
	PID_Clear(&current_loop_M0); 

	
//	uint16_t i;
//	float vel_loop_M0_erro0,vel_loop_M0_erro1,vel_loop_M0_out,vel_loop_M0_actual,vel_loop_M0_target;
//	float current_loop_M0_erro0,current_loop_M0_erro1,current_loop_M0_out,current_loop_M0_actual,current_loop_M0_target;
	FOC_M0_alignSensor(7,1);
	while(1)
	{
		RUN_FOC(&current_a, &current_b);
		Cmd_operation(&current_a, &current_b, &M0_Vel_Flt, &M0_Curr_Flt, &vel_loop_M0, &angle_loop_M0, &current_loop_M0);
		//FOC_M0_set_Force_Angle(0, &current_a, &current_b, &M0_Curr_Flt, &angle_loop_M0, &current_loop_M0);
		//FOC_M0_set_Velcocity_Angle(_PI/3, &current_a, &current_b, &M0_Vel_Flt, &M0_Curr_Flt, &angle_loop_M0, &vel_loop_M0, &current_loop_M0);
		
		//FOC_M0_setVelocity(30, &current_a, &current_b, &M0_Vel_Flt, &M0_Curr_Flt, &vel_loop_M0, &current_loop_M0);
		//FOC_M0_set_Force_Angle(0.0f, &current_a, &current_b, &M0_Curr_Flt, &angle_loop_M0, &current_loop_M0);
		//FOC_M0_setTorque(2, &current_a, &current_b, &M0_Curr_Flt, &current_loop_M0);
		//velocityOpenloop(20);
//			Check_Raw_ADC();
//			Serial_SendString("\n");
		//Sensor_update();
//		i++;
//		if(i==30)
//		{
//		Serial_Print_float_number(getVelocity());
//		Serial_SendString("\r\n");
//		Serial_Print_float_number(current_loop_M0.Actual);
//		Serial_SendString("\r\n");
//		Serial_Print_float_number(Sensor_GetAngle());
//		Serial_SendString("\r\n");
//		i=0;
//		}
//		out_vel = vel_loop_M0.Out;
//		target_vel = vel_loop_M0.Target;
//		out_angle = angle_loop_M0.Out;
//		target_angle = angle_loop_M0.Target;
//		out_current = current_loop_M0.Out;
//		target_current = current_loop_M0.Target;
//		Get_Phase_Currents(&current_a, &current_b);
//			Serial_Print_float_number(current_a);
//			Serial_Print_float_number(current_b);
		  //delay_ms(100);
		//vel_loop_M0.Actual;
//		if(i<=5)
//		{
//		TIME[i] = micros();
//		i++;
//		}
		
 	}
}


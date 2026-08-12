#include "AS5600.h"
#include "iic.h"
#include "math.h"
#include "REALTIME.h" 
#include "SysTick.h"

#define _2PI 6.28318530718f

float angle_prev = 0;
uint64_t angle_prev_ts = 0;
float vel_angle_prev = 0;
uint64_t vel_angle_prev_ts = 0;
long int full_rotations = 0; 
long int vel_full_rotations = 0;
uint8_t readArray[2]={0x0000,0x0000};
uint16_t readValue = 0x0000;

void Sensor_init(void)
{
	REALTIME_INIT();
	IIC_Init();
	delay_ms(500);
	Sensor_GetAngle();
	delay_us(1);
	vel_angle_prev=Sensor_GetAngle();
	vel_angle_prev_ts = micros();
	delay_ms(1);
	Sensor_GetAngle();
	delay_us(1);
	angle_prev = Sensor_GetAngle();
	angle_prev_ts = micros();
}

double Sensor_GetAngle(void)
{
	u8 angle_reg_msb_hi = 0x0e;
	u8 angle_reg_msb_lo = 0x0f;
	
	IIC_Start();
	IIC_Send_Byte(0X6C);
	IIC_Wait_Ack();
	IIC_Send_Byte(angle_reg_msb_hi);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0X6D);
	IIC_Wait_Ack();
	readArray[0]=IIC_Read_Byte(0);
	IIC_Stop();
	
	IIC_Start();
	IIC_Send_Byte(0X6C);
	IIC_Wait_Ack();
	IIC_Send_Byte(angle_reg_msb_lo);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0X6D);
	IIC_Wait_Ack();
	readArray[1]=IIC_Read_Byte(0);
	IIC_Stop();
	
//	int _bit_resolution = 12;
//	int _bits_used_msb =4;
//	float cpr = pow(2, _bit_resolution);
//	int lsb_used = _bit_resolution - _bits_used_msb;
//	
//	uint8_t lsb_mask = (uint8_t)((2 << lsb_used) - 1); 
//	uint8_t msb_mask = (uint8_t)((2 << _bits_used_msb) - 1 );
//	
//	readValue = ( readArray[1] & lsb_mask );
//  readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
	readValue = (uint16_t)readArray[0] << 8 ; 
	readValue = readValue | (uint16_t)readArray[1];

  return (readValue/ 4096.0f) * _2PI;
}

void Sensor_update(void)
{
	float val = Sensor_GetAngle();
	angle_prev_ts = micros();
	float d_angle = val - angle_prev;
	if(fabs(d_angle) > (0.8f*_2PI)) full_rotations += (d_angle >0) ? -1:1;
	angle_prev = val;
}

float getMechanicalAngle(void)
{
	return angle_prev;
}

float getAngle(void)
{
	return (float)full_rotations * _2PI + angle_prev;
}

float getVelocity(void)//计算变化角度，vel实际上就是当前圈数+当前角度之和减去前全书+前角度之和，也就是真正的delta angle
{
	float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
	if(Ts <= 0) Ts = 1e-3f;
	float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;
	vel_angle_prev = angle_prev;
  vel_full_rotations = full_rotations;
  vel_angle_prev_ts = angle_prev_ts;
  return vel;
}

#ifndef _AS5600_H
#define _AS5600_H
#include "system.h"

extern uint8_t readArray[2];
extern uint16_t readValue;

double Sensor_GetAngle(void);
void Sensor_init(void);
void Sensor_update(void);
float getMechanicalAngle(void);
float getAngle(void);
float getVelocity(void);
extern float angle_prev;
extern uint64_t angle_prev_ts;
extern float vel_angle_prev;
extern uint64_t vel_angle_prev_ts;
extern long int full_rotations; 
extern long int vel_full_rotations;

#endif

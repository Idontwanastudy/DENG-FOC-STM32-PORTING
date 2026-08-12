#ifndef __AD_H
#define __AD_H
#include "system.h"

void AD_Init(void);
float AD_GetValue(uint8_t n);
void CurrSense(void);
void Take_offset(void);
void Get_Phase_Currents(float *current_a , float *current_b);
void Check_Raw_ADC(void);

extern float offset_ia;
extern float offset_ib;
extern float offset_ic;
extern float shunt_resistor;
extern float amp_gain;
extern float volts_2_amps_ratio;
extern float gain_a;
extern float gain_b;
extern uint16_t adc_buffer[2];

#endif

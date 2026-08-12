#ifndef __SERIAL_H
#define __SERIAL_H
#include "system.h"

#include <stdio.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
void Serial_Print_float_number(float number);
void Array_Cut_Operation(char Serial_RxPacket[]);
uint8_t Serial_GetRxFlag(void);
float Serial_GetRxData(void);
uint8_t Cmd_Compare(void);
float Get_real_num(char num[]);
void Get_real_num_array(float *num);
void Clear_Array(void);

extern char Serial_RxPacket[100];	
extern uint8_t Serial_RxFlag;		
extern char Cmd_Array[10];
extern char Num_Array[5][10];
extern float Common_Num[5];
extern uint8_t Usart_flag;

#endif

#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

char Serial_RxPacket[100];	
uint8_t Serial_RxFlag;		
char Cmd_Array[10];
char Num_Array[5][10]={"0.0","0.0","0.0","0.0","0.0"};
float Common_Num[5];
uint8_t Usart_flag=0;


/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//开启USART2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA2引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA3引脚初始化为上拉输入
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 115200;              //波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要rdwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择ode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;     //奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位8b;
	USART_Init(USART2, &USART_InitStructure);               //将结构体变量交给USART_Init，配置USART2
	
	/*中断输出配置*/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;       //选择配置NVIC的USART2线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//指定NVIC线路的抢占优先级为1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);                         //将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART2, ENABLE);								//使能USART2，串口开始运行
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

void Serial_Print_float_number(float number)
{
    char String[100], num;
    
    // 处理符号
    int is_negative = 0;
    if (number < 0) {
        is_negative = 1;
        number = -number;
    }
    
    int numberh = (int)number;  // 整数部分
    float numberm = number - (float)numberh;  // 小数部分
    
    // 计算需要显示的小数位数（最多4位）
    int decimal_digits = 0;
    float temp = numberm;
    while (temp > 0.00001 && decimal_digits < 6) {
        temp = temp * 10;
        temp = temp - (int)temp;
        decimal_digits++;
    }
    
    // 将小数部分转换为整数
    int multiplier = 1;
    for (int k = 0; k < decimal_digits; k++) {
        multiplier *= 10;
    }
    int numberl = (int)(numberm * multiplier + 0.5);  // 四舍五入
    
    // 特殊处理：如果小数部分为0，也要保留至少1位
    if (decimal_digits == 0 && numberm > 0.00001) {
        decimal_digits = 1;
        numberl = (int)(numberm * 10 + 0.5);
    }
    
    u8 flag = 1, n;
    u8 i = 0;
    
    // 处理小数部分
    if (numberl > 0) {
        flag = 1;
        while (flag != 0) {
            String[i] = (char)(numberl % 10) + '0';
            numberl = numberl / 10;
            i++;
            if (numberl == 0)
                flag = 0;
        }
        
        // 如果小数位数不足，补0
        while (i < decimal_digits) {
            String[i] = '0';
            i++;
        }
        
        String[i] = '.';
        i++;
    } else {
        // 没有小数部分，只显示整数
        String[i] = '.';
        i++;
    }
    
    // 处理整数部分
    flag = 1;
    // 处理0的情况
    if (numberh == 0) {
        String[i] = '0';
        i++;
        flag = 0;
    } else {
        while (flag != 0) {
            String[i] = (char)(numberh % 10) + '0';
            numberh = numberh / 10;
            i++;
            if (numberh == 0)
                flag = 0;
        }
    }
    
    // 添加负号
    if (is_negative) {
        String[i] = '-';
        i++;
    }
    
    // 反转字符串
    for (n = 0; n <= (i - 1) / 2; n++) {
        num = String[n];
        String[n] = String[i - 1 - n];
        String[i - 1 - n] = num;
    }
    
    String[i] = '\0';
    Serial_SendString(String);
}


/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的数据，范围：0~255
  */

/**
  * 函    数：USART2中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */

void Array_Cut_Operation(char RxPacket[])
{
	int i=0,j=0,l=0;
	Clear_Array();
	for(i=0;RxPacket[i]!=' '&&RxPacket[i]!='\0';i++ )
	{
		Cmd_Array[i] = RxPacket[i];
	}
	Cmd_Array[i] = '\0';
	i++;
	for(j=0,l=0; RxPacket[i]!='\0' && j<5 && l<10; l++,i++)
	{
		if(RxPacket[i] != ' ')
			Num_Array[j][l] = RxPacket[i];
		else if(RxPacket[i] == ' ')
		{
			Num_Array[j][l] = '\0';
			j++;
			l=-1;
		}
	}
	Num_Array[j][l] = '\0';
}

void Clear_Array(void)
{
	int i=0,j=0;
	for(i=0;i<10;i++)
	{
		Cmd_Array[i]='\0';
	}
	i=0;j=0;
	for(j=0;j<5;j++)
	{
		for(i=0;i<10;i++)
		{
			Num_Array[j][i] = '\0';
		}
	}
}

uint8_t Cmd_Compare(void)
{
	if(strcmp(Cmd_Array,"setvel")==0)
		return 1;
	else if(strcmp(Cmd_Array,"setang")==0)
		return 2;
	else if(strcmp(Cmd_Array,"setprog")==0)
		return 3;
	else if(strcmp(Cmd_Array,"setpidvel")==0)
		return 4;
	else if(strcmp(Cmd_Array,"setpidang")==0)
		return 5;
	else if(strcmp(Cmd_Array,"setpidcur")==0)
		return 6;
	else if(strcmp(Cmd_Array,"showpid")==0)
		return 7;
	else if(strcmp(Cmd_Array,"showvel")==0)
		return 8;
	else if(strcmp(Cmd_Array,"showang")==0)
		return 9;
	else if(strcmp(Cmd_Array,"showcur")==0)
		return 10;
	else
		return 0;
}

void Get_real_num_array(float num[])
{
	int i = 0;
	while(i<5)
	{
		if(Get_real_num(Num_Array[i])!=114514.0f)
		{
			num[i]=Get_real_num(Num_Array[i]);
			i++;
		}
		else if(Get_real_num(Num_Array[i])==114514.0f)
		{
			num[0]=114514;
			break;
		}
	}
}

float Get_real_num(char num[])
{
	float Real_Num=0;
	int i=0,j=1,dir=1,count_dop=0,count_minus=0;
		for(i=0,j=1; num[i]!='\0'; i++)
		{
			if((((num[i]-'0')>=0)&&((num[i]-'0')<=9)) && dir == 1)
			{
				Real_Num = Real_Num*10 + (num[i]-'0');
			}
			else if(num[i] == '.')
			{
				count_dop++;
				if(count_dop<2)
				{
					dir = 0;
				}
				else
					return 114514;
			}//整数部分
			else if((((num[i]-'0')>0)&&((num[i]-'0')<=9)) && dir == 0)
			{
				Real_Num = Real_Num + (num[i]-'0')*pow(0.1,j);
				j++;
			}
			else if((num[i]-'0')==0 && dir == 0)
			{
				j++;
			}
			else if(num[i] == '-')
			{
				count_minus++;
			}
			else
				return 114514;
		}//小数部分
		if(num[0] == '-' && (count_minus % 2) == 1)
			Real_Num = -1*Real_Num;
		return Real_Num;
}

void USART2_IRQHandler(void)
{
	static uint8_t RxState = 0;	
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)		
	{
		uint8_t RxData = USART_ReceiveData(USART2);		
		if(RxState == 0)
		{
			if(RxData == '/' /*&& Serial_RxFlag == 0*/)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		
		else if(RxState == 1)
		{
			if (RxData == '\r')	
				{
					RxState = 2;
				}
			else
			{
				Serial_RxPacket[pRxPacket] = RxData;
				pRxPacket ++;			
			}				

		}
		else if (RxState == 2)
		{
			if (RxData == '\n')			
			{
				RxState = 0;			
				Serial_RxPacket[pRxPacket] = '\0';			
				Serial_RxFlag = 1;
				Array_Cut_Operation(Serial_RxPacket);
				Usart_flag=1;
			}
		}									
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);																	
	}
}

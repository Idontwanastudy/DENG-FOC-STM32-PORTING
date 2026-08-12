#include "stm32f10x.h"                  // Device header
#include "AD.h"
#include "SysTick.h"
#include "Serial.h"

float shunt_resistor;
float amp_gain;
float volts_2_amps_ratio;
float gain_a;
float gain_b;
float offset_ia;
float offset_ib;
float offset_ic;
uint16_t adc_buffer[2];

#define ADC_VOL 3.3f
#define ADC_FENBIANLV 4095.0F
#define ADC_CONV ((ADC_VOL)/(ADC_FENBIANLV))

void AD_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//开启ADC1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*设置ADC时钟*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);						//选择时钟6分频，ADCCLK = 72MHz / 6 = 12MHz
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PB0和PB1引脚初始化为模拟输入
	
	/*ADC初始化*/
	ADC_InitTypeDef ADC_InitStructure;						//定义结构体变量
	ADC_DeInit(ADC1); 
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;      //模式，选择独立模式，即单独使用ADC1
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //数据对齐，选择右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//外部触发，使用软件触发，不需要外部触发gConv_None;gConv_None;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;     //连续转换，失能，每转换一次规则组序列后停止
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;           //扫描模式，失能，只转换规则组的序列1这一个位置
	ADC_InitStructure.ADC_NbrOfChannel = 2;                 //通道数，为1，仅在扫描模式下，才需要指定大于1的数，在非扫描模式下，只能是1
	ADC_Init(ADC1, &ADC_InitStructure); 
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);								//固定流程，内部有电路会自动执行校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
	
	delay_ms(10);
	
	DMA_InitTypeDef DMA_InitStruct;
	DMA_DeInit(DMA1_Channel1);
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&adc_buffer;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStruct.DMA_BufferSize = 2;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	delay_ms(10);
	
	Take_offset();
	CurrSense();
//	Check_Raw_ADC();
}


float AD_GetValue(uint8_t n)
{
	if (n == 1)				//指定电流传感器1
	{
		return (float)(adc_buffer[0]*ADC_CONV);	
	}
	else if (n == 2)		//指定电流传感器2
	{	
		return (float)(adc_buffer[1]*ADC_CONV);	
	}
}

void CurrSense()
{
		shunt_resistor = 0.01;
		amp_gain = 50;
		volts_2_amps_ratio = 1.0f / shunt_resistor / amp_gain;
		gain_a = volts_2_amps_ratio*(-1);
		gain_b = volts_2_amps_ratio*(-1);
}

void Take_offset()
{
	const int calibration_rounds = 1000;
	
	offset_ia = 0;
	offset_ib = 0;
	offset_ic = 0;
	
	int i=0;
	for(i=0; i<calibration_rounds; i++)
	{
		offset_ia+=AD_GetValue(1);
		offset_ib+=AD_GetValue(2);
		delay_ms(1);
	}
	offset_ia = offset_ia/calibration_rounds;
	offset_ib = offset_ib/calibration_rounds;
}

void Get_Phase_Currents(float *current_a , float *current_b)
{
	*current_a = (AD_GetValue(1)-offset_ia)*gain_a;
	*current_b = (AD_GetValue(2)-offset_ib)*gain_b;
}

void Check_Raw_ADC(void)
{
			Serial_SendNumber(adc_buffer[0],4);
			Serial_SendString("_");
			Serial_SendNumber(adc_buffer[1],4);
}

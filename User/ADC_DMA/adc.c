/*
Author:SHIELD_QI
Date:2018-06-30
************************************/

#include "adc.h"

__IO uint16_t  ADC_ConvertedValue[DMA_Data_size]={2048,1,1};

//-----------------------------ADC1��GPIO_Init------------------------------/

static void ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//--------------------ACC1_ͨ��1----------------------/
  RCC_AHB1PeriphClockCmd(ADC_GPIO_CLK1,ENABLE); 
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin=ADC_GPIO_PIN1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
	GPIO_Init(ADC_GPIO_PORT1, &GPIO_InitStructure);
	
	//--------------------ACC2_ͨ��2----------------------/
	RCC_AHB1PeriphClockCmd(ADC_GPIO_CLK2,ENABLE);
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin=ADC_GPIO_PIN2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(ADC_GPIO_PORT2, &GPIO_InitStructure);
	
	//--------------------DCC_ͨ��3-----------------------/
	RCC_AHB1PeriphClockCmd(ADC_GPIO_CLK3,ENABLE);
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin=ADC_GPIO_PIN3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(ADC_GPIO_PORT3, &GPIO_InitStructure);
	 
}

static void ADC_Mode_Config(void)
{
	DMA_InitTypeDef          DMA_InitStructure;
	ADC_InitTypeDef          ADC_InitStructure;
	ADC_CommonInitTypeDef    ADC_CommonInitStructure;
	
	RCC_AHB1PeriphClockCmd(ADC_DMA_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(ADC_CLK , ENABLE);	
	
	//--------------------------DMA��ʼ��---------------------------/	
	
	/*
	*ADC �Ĺ������ݼĴ���ֻ�е� 16 λ��Ч��ʵ�ʴ�ŵ�����
	*ֻ�� 12 λ���ѣ������������ݴ�СΪ���ִ�С��
	*/
	DMA_InitStructure.DMA_BufferSize=DMA_Data_size;
	DMA_InitStructure.DMA_Channel=ADC_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold=DMA_FIFOThreshold_HalfFull;     //FIFO�Ĵ�С
	DMA_InitStructure.DMA_Memory0BaseAddr=(u32)ADC_ConvertedValue;
	DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;  //���������ֽ�
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;            
	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;                      //ѭ������ģʽ
	DMA_InitStructure.DMA_PeripheralBaseAddr=ADC_DR_ADDR;
	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority=DMA_Priority_High;
	DMA_Init(ADC_DMA_STREAM, &DMA_InitStructure);
	DMA_Cmd(ADC_DMA_STREAM, ENABLE);
	
	//-------------------------ADC��ʼ��----------------------------/	
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_NbrOfConversion=DMA_Data_size;                     //ת��ͨ��
	ADC_InitStructure.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;//��ֹ�ⲿ��Ե����
	ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b;                     //ADC�ֱ���
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;                  
	ADC_Init(ADC_X, &ADC_InitStructure);
	
  //---------------------ADC_Common��ʼ��-------------------------/	
	
	ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;    // ��ֹ DMA ֱ�ӷ���ģʽ
	ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	//-------------���� ADCͨ��ת��˳��Ͳ���ʱ������----------------/	
	
	ADC_RegularChannelConfig(ADC_X, ADC_CHANNEL1, 1, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC_X, ADC_CHANNEL2, 2, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC_X, ADC_CHANNEL3, 3, ADC_SampleTime_28Cycles); 
	
	ADC_DMARequestAfterLastTransferCmd(ADC_X, ENABLE);                      // ʹ�� ADC DMA   
  ADC_DMACmd(ADC_X, ENABLE);
	
	ADC_Cmd(ADC_X, ENABLE);
	
	
	ADC_SoftwareStartConv(ADC_X);                                           //��ʼ adc ת�����������
	
}

void ADC_INIT(void)
{
		ADC_GPIO_Config();
		ADC_Mode_Config();
}

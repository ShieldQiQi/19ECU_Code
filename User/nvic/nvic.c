/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "asms.h"
#include "timer.h"
#include "adc.h"
#include "nvic.h"
#include "bckp_io.h"

void NVIC_Configure(void)
{
	
	NVIC_InitTypeDef 					NVIC_InitStructure;
	
	//----------------------��ʱ�ж�----------------------/
	NVIC_InitStructure.NVIC_IRQChannel=TIM_X_IRQn;                //��ʱ��5�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;    //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03;           //�����ȼ�5
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//-----------------�����ж�Դ��ASMS----------------/
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  NVIC_InitStructure.NVIC_IRQChannel = ASMS_EXTI_IRQ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

  //--------------�����ж�Դ����ʻ����----------------/
  NVIC_InitStructure.NVIC_IRQChannel = RUN_ENABLE_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
	//----------------�ⲿ�ж�EXTI����--------------------/
	
	EXTI_InitTypeDef EXTI_InitStructure;
	// ʹ�� SYSCFG ʱ�� ��ʹ��GPIO�ⲿ�ж�ʱ����ʹ��SYSCFGʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	// ���� EXTI �ж�Դ �� ASMS PE2����
//  SYSCFG_EXTILineConfig(ASMS_EXTI_PORTSOURCE,ASMS_EXTI_PINSOURCE);
//	EXTI_InitStructure.EXTI_Line = ASMS_EXTI_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
	// ���� EXTI �ж�Դ �� ��ʻ����
  SYSCFG_EXTILineConfig(RUN_ENABLE_EXTI_PORTSOURCE,RUN_ENABLE_EXTI_PINSOURCE);
	EXTI_InitStructure.EXTI_Line = RUN_ENABLE_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

}

//void DCC_IRQHandler(void)
//{

//	if(ADC_GetFlagStatus( ADC_X, ADC_FLAG_AWD)==SET)
//	{
//		//�����ʻģʽ
//		if(GPIO_ReadInputDataBit(RUN_ENABLE_GPIO_PORT, RUN_ENABLE_PIN))
//			ToDriveFlag=1;
//		//�ƶ�
//		CAN1_Ctrl_Send(Disable_Msg,0x04,0x00);
//		CAN1_LED_TOGGLE;
//		
//		
//		ADC_ClearITPendingBit( ADC_X, ADC_IT_AWD);
//	}
//	
//}

//	//--------------�����ж�Դ��ADC-DCC----------------/
//	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//	
//  NVIC_InitStructure.NVIC_IRQChannel = DCC_IRQ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//	
//	//��ͨ�� ADC_Channel_12
//  ADC_AnalogWatchdogSingleChannelConfig( ADC_X, ADC_CHANNEL3);
//	//���øߵ;���ֵ
//	ADC_AnalogWatchdogThresholdsConfig( ADC_X,HighThreshold, LowThreshold);   
//	//����ģ�⿴�Ź�
//	ADC_AnalogWatchdogCmd(ADC_X, ADC_AnalogWatchdog_SingleRegEnable);
//  //�����жϣ�ģ�⿴�Ź���
//  ADC_ITConfig( ADC_X, ADC_IT_AWD, ENABLE);

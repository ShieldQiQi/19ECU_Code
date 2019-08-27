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
	
	//----------------------定时中断----------------------/
	NVIC_InitStructure.NVIC_IRQChannel=TIM_X_IRQn;                //定时器5中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;    //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03;           //子优先级5
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//-----------------配置中断源：ASMS----------------/
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  NVIC_InitStructure.NVIC_IRQChannel = ASMS_EXTI_IRQ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

  //--------------配置中断源：待驶开关----------------/
  NVIC_InitStructure.NVIC_IRQChannel = RUN_ENABLE_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
	//----------------外部中断EXTI配置--------------------/
	
	EXTI_InitTypeDef EXTI_InitStructure;
	// 使能 SYSCFG 时钟 ，使用GPIO外部中断时必须使能SYSCFG时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	// 连接 EXTI 中断源 到 ASMS PE2引脚
//  SYSCFG_EXTILineConfig(ASMS_EXTI_PORTSOURCE,ASMS_EXTI_PINSOURCE);
//	EXTI_InitStructure.EXTI_Line = ASMS_EXTI_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
	// 连接 EXTI 中断源 到 待驶开关
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
//		//进入待驶模式
//		if(GPIO_ReadInputDataBit(RUN_ENABLE_GPIO_PORT, RUN_ENABLE_PIN))
//			ToDriveFlag=1;
//		//制动
//		CAN1_Ctrl_Send(Disable_Msg,0x04,0x00);
//		CAN1_LED_TOGGLE;
//		
//		
//		ADC_ClearITPendingBit( ADC_X, ADC_IT_AWD);
//	}
//	
//}

//	//--------------配置中断源：ADC-DCC----------------/
//	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//	
//  NVIC_InitStructure.NVIC_IRQChannel = DCC_IRQ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//	
//	//单通道 ADC_Channel_12
//  ADC_AnalogWatchdogSingleChannelConfig( ADC_X, ADC_CHANNEL3);
//	//设置高低警戒值
//	ADC_AnalogWatchdogThresholdsConfig( ADC_X,HighThreshold, LowThreshold);   
//	//开启模拟看门狗
//	ADC_AnalogWatchdogCmd(ADC_X, ADC_AnalogWatchdog_SingleRegEnable);
//  //开启中断（模拟看门狗）
//  ADC_ITConfig( ADC_X, ADC_IT_AWD, ENABLE);

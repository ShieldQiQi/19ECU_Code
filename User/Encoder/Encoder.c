/*
Author:SHIELD_QI
Date:2018-06-30
************************************/

#include "Encoder.h"

//----------------------------------------------ENCODER3_TIM8----------------------------------------------/
void TIM8_Encoder_Init(u32 arr,u16 psc)  
{ 
	GPIO_InitTypeDef          GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_ICInitTypeDef         TIM8_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	     //TIM8时钟使能    
	RCC_AHB1PeriphClockCmd(ENCODER3_GPIO_Clk1, ENABLE); 	     //使能PORTA时钟	
	
	//------------------------------GPIO输入通道初始化------------------------------/
	GPIO_InitStructure.GPIO_Pin = ENCODER3_GPIO_Pin1|ENCODER3_GPIO_Pin2;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //速度100MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //输入功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //开路输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //上拉	
	GPIO_Init(ENCODER3_GPIO_Port1,&GPIO_InitStructure); 
	
	GPIO_PinAFConfig(ENCODER3_GPIO_Port1,GPIO_PinSource6,GPIO_AF_TIM8);     
  GPIO_PinAFConfig(ENCODER3_GPIO_Port2,GPIO_PinSource7,GPIO_AF_TIM8);     
	
	//------------------------------时基机构体初始化------------------------------/  
	TIM_DeInit(TIM8);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //定时器分频
  TIM_TimeBaseStructure.TIM_Period=arr;                      //自动重装载值 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //时钟分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	
  //--------------------------捕获通道 & 编码器模式配置-------------------------/
	TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
	TIM_ICStructInit(&TIM8_ICInitStructure);
  TIM8_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	TIM8->CNT = 0;
  TIM_Cmd(TIM8,ENABLE ); 	                                   //使能定时器5
		
}

//----------------------------------------------ENCODER4_TIM1----------------------------------------------/
void TIM1_Encoder_Init(u32 arr,u16 psc)  
{ 
	GPIO_InitTypeDef          GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_ICInitTypeDef         TIM1_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	     //TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(ENCODER4_GPIO_Clk1, ENABLE); 	     //使能PORTA时钟	
	
	//------------------------------GPIO输入通道初始化------------------------------/
	GPIO_InitStructure.GPIO_Pin = ENCODER4_GPIO_Pin1|ENCODER4_GPIO_Pin2;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //速度100MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //输入功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //开路输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //上拉	
	GPIO_Init(ENCODER4_GPIO_Port1,&GPIO_InitStructure); 
	
	GPIO_PinAFConfig(ENCODER4_GPIO_Port1,GPIO_PinSource9,GPIO_AF_TIM1);     
  GPIO_PinAFConfig(ENCODER4_GPIO_Port2,GPIO_PinSource11,GPIO_AF_TIM1);     
	
	//------------------------------时基机构体初始化------------------------------/  
	TIM_DeInit(TIM1);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //定时器分频
  TIM_TimeBaseStructure.TIM_Period=arr;                      //自动重装载值 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //时钟分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
  //--------------------------捕获通道 & 编码器模式配置-------------------------/
	TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
	TIM_ICStructInit(&TIM1_ICInitStructure);
  TIM1_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM1->CNT = 0;
  TIM_Cmd(TIM1,ENABLE ); 	                                   //使能定时器5
		
}


void ENCODER_INIT(void)
{
//	TIM3_Encoder_Init(60000,0);
//	TIM2_Encoder_Init(60000,0);
	TIM8_Encoder_Init(60000,0);
	TIM1_Encoder_Init(60000,0);
}

/***************************END OF FILE**************************/


//----------------------------------------------ENCODER1_TIM3-----------------------------------------------/

//void TIM3_Encoder_Init(u32 arr,u16 psc)  
//{ 
//	GPIO_InitTypeDef          GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
//	TIM_ICInitTypeDef         TIM3_ICInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	     //TIM3时钟使能    
//	RCC_AHB1PeriphClockCmd(ENCODER1_GPIO_Clk1, ENABLE); 	     //使能PORTA时钟	
//	
//	//------------------------------GPIO输入通道初始化------------------------------/
//	GPIO_InitStructure.GPIO_Pin = ENCODER1_GPIO_Pin1|ENCODER1_GPIO_Pin2;  
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //速度100MHz
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //输入功能
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //开路输出 
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //上拉	
//	GPIO_Init(ENCODER1_GPIO_Port1,&GPIO_InitStructure); 
//	 
//	GPIO_PinAFConfig(ENCODER1_GPIO_Port1,GPIO_PinSource6,GPIO_AF_TIM3);      //PA6复用位定时器3
//  GPIO_PinAFConfig(ENCODER1_GPIO_Port2,GPIO_PinSource7,GPIO_AF_TIM3);      //PA7复用位定时器3
//	
//	//------------------------------时基机构体初始化------------------------------/  
//	TIM_DeInit(TIM3);
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //定时器分频
//  TIM_TimeBaseStructure.TIM_Period=arr;                      //自动重装载值 
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数模式
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //时钟分频
//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//	
//  //--------------------------捕获通道 & 编码器模式配置-------------------------/
//	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
//	TIM_ICStructInit(&TIM3_ICInitStructure);
//  TIM3_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 配置输入滤波器 不滤波
//  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
//	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	TIM3->CNT = 0;
//  TIM_Cmd(TIM3,ENABLE ); 	                                   //使能定时器3
//		
//}

////-----------------------------------------------ENCODER2_TIM2-----------------------------------------------/
//void TIM2_Encoder_Init(u32 arr,u16 psc)  
//{ 
//	
//	GPIO_InitTypeDef          GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
//	TIM_ICInitTypeDef         TIM2_ICInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	     //TIM2时钟使能    
//	RCC_AHB1PeriphClockCmd(ENCODER2_GPIO_Clk1, ENABLE); 	     //使能PORTA时钟	
//	
//	//------------------------------GPIO输入通道初始化------------------------------/
//	GPIO_InitStructure.GPIO_Pin = ENCODER2_GPIO_Pin1|ENCODER2_GPIO_Pin2;  
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //速度100MHz
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //输入功能
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //开路输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //上拉	
//	GPIO_Init(ENCODER2_GPIO_Port1,&GPIO_InitStructure); 
//	
//	GPIO_PinAFConfig(ENCODER2_GPIO_Port1,GPIO_PinSource2,GPIO_AF_TIM2);     
//  GPIO_PinAFConfig(ENCODER2_GPIO_Port2,GPIO_PinSource3,GPIO_AF_TIM2);     
//	
//	//------------------------------时基机构体初始化------------------------------/  
//	TIM_DeInit(TIM2);
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //定时器分频
//  TIM_TimeBaseStructure.TIM_Period=arr;                      //自动重装载值 
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数模式
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //时钟分频
//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
//	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
//	
//  //--------------------------捕获通道 & 编码器模式配置-------------------------/
//	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
//	TIM_ICStructInit(&TIM2_ICInitStructure);
//  TIM2_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 配置输入滤波器 不滤波
//  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//	TIM2->CNT = 0;
//  TIM_Cmd(TIM2,ENABLE ); 	                                   //使能定时器5
//		
//}

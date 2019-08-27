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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	     //TIM8ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(ENCODER3_GPIO_Clk1, ENABLE); 	     //ʹ��PORTAʱ��	
	
	//------------------------------GPIO����ͨ����ʼ��------------------------------/
	GPIO_InitStructure.GPIO_Pin = ENCODER3_GPIO_Pin1|ENCODER3_GPIO_Pin2;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //�ٶ�100MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //���빦��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //��·���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //����	
	GPIO_Init(ENCODER3_GPIO_Port1,&GPIO_InitStructure); 
	
	GPIO_PinAFConfig(ENCODER3_GPIO_Port1,GPIO_PinSource6,GPIO_AF_TIM8);     
  GPIO_PinAFConfig(ENCODER3_GPIO_Port2,GPIO_PinSource7,GPIO_AF_TIM8);     
	
	//------------------------------ʱ���������ʼ��------------------------------/  
	TIM_DeInit(TIM8);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_Period=arr;                      //�Զ���װ��ֵ 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	
  //--------------------------����ͨ�� & ������ģʽ����-------------------------/
	TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
	TIM_ICStructInit(&TIM8_ICInitStructure);
  TIM8_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	TIM8->CNT = 0;
  TIM_Cmd(TIM8,ENABLE ); 	                                   //ʹ�ܶ�ʱ��5
		
}

//----------------------------------------------ENCODER4_TIM1----------------------------------------------/
void TIM1_Encoder_Init(u32 arr,u16 psc)  
{ 
	GPIO_InitTypeDef          GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_ICInitTypeDef         TIM1_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	     //TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(ENCODER4_GPIO_Clk1, ENABLE); 	     //ʹ��PORTAʱ��	
	
	//------------------------------GPIO����ͨ����ʼ��------------------------------/
	GPIO_InitStructure.GPIO_Pin = ENCODER4_GPIO_Pin1|ENCODER4_GPIO_Pin2;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //�ٶ�100MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //���빦��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //��·���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //����	
	GPIO_Init(ENCODER4_GPIO_Port1,&GPIO_InitStructure); 
	
	GPIO_PinAFConfig(ENCODER4_GPIO_Port1,GPIO_PinSource9,GPIO_AF_TIM1);     
  GPIO_PinAFConfig(ENCODER4_GPIO_Port2,GPIO_PinSource11,GPIO_AF_TIM1);     
	
	//------------------------------ʱ���������ʼ��------------------------------/  
	TIM_DeInit(TIM1);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_Period=arr;                      //�Զ���װ��ֵ 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
  //--------------------------����ͨ�� & ������ģʽ����-------------------------/
	TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
	TIM_ICStructInit(&TIM1_ICInitStructure);
  TIM1_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM1->CNT = 0;
  TIM_Cmd(TIM1,ENABLE ); 	                                   //ʹ�ܶ�ʱ��5
		
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
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	     //TIM3ʱ��ʹ��    
//	RCC_AHB1PeriphClockCmd(ENCODER1_GPIO_Clk1, ENABLE); 	     //ʹ��PORTAʱ��	
//	
//	//------------------------------GPIO����ͨ����ʼ��------------------------------/
//	GPIO_InitStructure.GPIO_Pin = ENCODER1_GPIO_Pin1|ENCODER1_GPIO_Pin2;  
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //�ٶ�100MHz
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //���빦��
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //��·��� 
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //����	
//	GPIO_Init(ENCODER1_GPIO_Port1,&GPIO_InitStructure); 
//	 
//	GPIO_PinAFConfig(ENCODER1_GPIO_Port1,GPIO_PinSource6,GPIO_AF_TIM3);      //PA6����λ��ʱ��3
//  GPIO_PinAFConfig(ENCODER1_GPIO_Port2,GPIO_PinSource7,GPIO_AF_TIM3);      //PA7����λ��ʱ��3
//	
//	//------------------------------ʱ���������ʼ��------------------------------/  
//	TIM_DeInit(TIM3);
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //��ʱ����Ƶ
//  TIM_TimeBaseStructure.TIM_Period=arr;                      //�Զ���װ��ֵ 
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���ģʽ
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //ʱ�ӷ�Ƶ
//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//	
//  //--------------------------����ͨ�� & ������ģʽ����-------------------------/
//	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
//	TIM_ICStructInit(&TIM3_ICInitStructure);
//  TIM3_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 ���������˲��� ���˲�
//  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
//	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	TIM3->CNT = 0;
//  TIM_Cmd(TIM3,ENABLE ); 	                                   //ʹ�ܶ�ʱ��3
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
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	     //TIM2ʱ��ʹ��    
//	RCC_AHB1PeriphClockCmd(ENCODER2_GPIO_Clk1, ENABLE); 	     //ʹ��PORTAʱ��	
//	
//	//------------------------------GPIO����ͨ����ʼ��------------------------------/
//	GPIO_InitStructure.GPIO_Pin = ENCODER2_GPIO_Pin1|ENCODER2_GPIO_Pin2;  
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;         //�ٶ�100MHz
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;               //���빦��
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;             //��·���
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;               //����	
//	GPIO_Init(ENCODER2_GPIO_Port1,&GPIO_InitStructure); 
//	
//	GPIO_PinAFConfig(ENCODER2_GPIO_Port1,GPIO_PinSource2,GPIO_AF_TIM2);     
//  GPIO_PinAFConfig(ENCODER2_GPIO_Port2,GPIO_PinSource3,GPIO_AF_TIM2);     
//	
//	//------------------------------ʱ���������ʼ��------------------------------/  
//	TIM_DeInit(TIM2);
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //��ʱ����Ƶ
//  TIM_TimeBaseStructure.TIM_Period=arr;                      //�Զ���װ��ֵ 
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���ģʽ
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //ʱ�ӷ�Ƶ
//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
//	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
//	
//  //--------------------------����ͨ�� & ������ģʽ����-------------------------/
//	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
//	TIM_ICStructInit(&TIM2_ICInitStructure);
//  TIM2_ICInitStructure.TIM_ICFilter = 0;                     //IC1F=0000 ���������˲��� ���˲�
//  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//	TIM2->CNT = 0;
//  TIM_Cmd(TIM2,ENABLE ); 	                                   //ʹ�ܶ�ʱ��5
//		
//}

/****
Project:ECU
Author:SHIELD_QI
Date:2018-08-12
**************************************************/

#include "stm32f4xx.h"
#include "can.h"
#include "adc.h"
#include "timer.h"
#include "debug_usart.h"
#include "led.h"
#include "brake_led.h"
#include "beep.h"
#include "asms.h"
#include "assi.h"
#include "bckp_io.h"
#include "msg_queue.h"
#include "circult.h"
#include "nvic.h"
#include "assi.h"
#include "rtc.h"

int main(void)
{
	CAN1_Config();											//CAN1 for Motor
	CAN2_Config();											//CAN2 for computer��EBS��etc
	TIM_Timer_Init(10000-1,84-1);				//TIM5=84M
	ADC_INIT();													//����̤�塢�ƶ�̤��ADC
	Debug_USART_Config();							  //���ڡ�������
	LED_GPIO_Config();									//״ָ̬ʾ��
	BRAKE_LED_GPIO_Config();						//�ƶ�ָʾ��
	BEEP_GPIO_Config();									//������
	ASMS_GPIO_Config();									//���˼�ʻϵͳ������
	BCKP_IO_GPIO_Config();							//�����������IO��
	ASSI_GPIO_Config();									//���˼�ʻ״ָ̬ʾ��
	SC_GPIO_Config();										//��ȫ��·IO
	NVIC_Configure();										//�ж�����
	Init_CanMsg_Queue();							  //��ʼ��CAN��Ϣ����
	RTC_CLK_Config();										//��ʼ��ʵʱʱ��
	
	SC(ON);															
	BRAKE_LED_OFF;
	NoneLight;
	
  while(1){}

}


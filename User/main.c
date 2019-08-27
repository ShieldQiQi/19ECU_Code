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
	CAN2_Config();											//CAN2 for computer，EBS，etc
	TIM_Timer_Init(10000-1,84-1);				//TIM5=84M
	ADC_INIT();													//加速踏板、制动踏板ADC
	Debug_USART_Config();							  //串口、调试用
	LED_GPIO_Config();									//状态指示灯
	BRAKE_LED_GPIO_Config();						//制动指示灯
	BEEP_GPIO_Config();									//蜂鸣器
	ASMS_GPIO_Config();									//无人驾驶系统主开关
	BCKP_IO_GPIO_Config();							//备用输入输出IO口
	ASSI_GPIO_Config();									//无人驾驶状态指示灯
	SC_GPIO_Config();										//安全回路IO
	NVIC_Configure();										//中断配置
	Init_CanMsg_Queue();							  //初始化CAN消息队列
	RTC_CLK_Config();										//初始化实时时钟
	
	SC(ON);															
	BRAKE_LED_OFF;
	NoneLight;
	
  while(1){}

}


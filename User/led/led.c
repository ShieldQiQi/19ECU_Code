/*
Date:2018-07-17
Author:SHIELD_QI
*********************************************/
  
#include "led.h"   

void LED_GPIO_Config(void)
{		

		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd ( RUNNING_LED_GPIO_CLK|WORKING_CAN_GPIO_CLK|CAR_STATU_GPIO_CLK, ENABLE); 	
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	
		//CAN接收
		GPIO_InitStructure.GPIO_Pin = RUNNING_LED_PIN;
    GPIO_Init(RUNNING_LED_GPIO_PORT, &GPIO_InitStructure);	
	
		//CAN1发送
		GPIO_InitStructure.GPIO_Pin = WORKING_CAN_PIN;
    GPIO_Init(WORKING_CAN_GPIO_PORT, &GPIO_InitStructure);
	
		//CAN发送
		GPIO_InitStructure.GPIO_Pin = CAR_STATU_PIN;
    GPIO_Init(CAR_STATU_GPIO_PORT, &GPIO_InitStructure);
	
}

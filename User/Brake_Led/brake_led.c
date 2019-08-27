/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "brake_led.h"

void BRAKE_LED_GPIO_Config(void)
{		

		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd ( BRAKE_LED_GPIO_CLK, ENABLE); 	
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
		GPIO_InitStructure.GPIO_Pin = BRAKE_LED_PIN;
    GPIO_Init(BRAKE_LED_GPIO_PORT, &GPIO_InitStructure);	
	
}

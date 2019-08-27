/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "beep.h"

void BEEP_GPIO_Config(void)
{		

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd ( BEEP_GPIO_CLK, ENABLE); 	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
	GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStructure);	
	
	GPIO_SetBits(BEEP_GPIO_PORT,BEEP_PIN);
}

/*
Date:2018-08-12
Author:SHIELD_QI
*********************************************/
  
#include "circult.h"   

void SC_GPIO_Config(void)
{		

		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd ( SC_GPIO_CLK, ENABLE); 	
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	
		GPIO_InitStructure.GPIO_Pin = SC_PIN;
    GPIO_Init(SC_GPIO_PORT, &GPIO_InitStructure);	
	
}

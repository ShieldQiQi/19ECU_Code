/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "bckp_io.h"

//备用IO口-输出与输入

void BCKP_IO_GPIO_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure_out;
	GPIO_InitTypeDef GPIO_InitStructure_in;

	RCC_AHB1PeriphClockCmd ( BCKP_OUT1_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd ( RUN_ENABLE_GPIO_CLK|BCKP_IN2_GPIO_CLK|BCKP_IN3_GPIO_CLK|BCKP_IN4_GPIO_CLK|isTS_ON_GPIO_CLK, ENABLE);  	

  //----------------------------备用输出-------------------------/
	GPIO_InitStructure_out.GPIO_Mode = GPIO_Mode_OUT;   
	GPIO_InitStructure_out.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_out.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure_out.GPIO_Speed = GPIO_Speed_50MHz; 

	//待驶LED指示灯
	GPIO_InitStructure_out.GPIO_Pin = BCKP_OUT1_PIN;
	GPIO_Init(BCKP_OUT1_GPIO_PORT, &GPIO_InitStructure_out);	

	GPIO_ResetBits(BCKP_OUT1_GPIO_PORT, BCKP_OUT1_PIN);

	
	//----------------------------备用输入-------------------------/
	GPIO_InitStructure_in.GPIO_Mode = GPIO_Mode_IN;   
	GPIO_InitStructure_in.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure_in.GPIO_Speed = GPIO_Speed_50MHz; 

	//待驶模式开关
	GPIO_InitStructure_in.GPIO_Pin = RUN_ENABLE_PIN;
	GPIO_Init(RUN_ENABLE_GPIO_PORT, &GPIO_InitStructure_in);	
	
	//安全回路判断高压是否开启
	GPIO_InitStructure_in.GPIO_Pin = isTS_ON_PIN;
	GPIO_Init(isTS_ON_GPIO_PORT, &GPIO_InitStructure_in);
	
}





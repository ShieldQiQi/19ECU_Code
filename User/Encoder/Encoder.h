#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "stm32f4xx.h"	
void TIM2_Encoder_Init(u32 arr,u16 psc);
//--------------------------ENCODER1_TIM3----------------------------/       //FL_A FL_B
#define ENCODER1_GPIO_Port1          GPIOA
#define ENCODER1_GPIO_Pin1           GPIO_Pin_6
#define ENCODER1_GPIO_Clk1           RCC_AHB1Periph_GPIOA

#define ENCODER1_GPIO_Port2          GPIOA
#define ENCODER1_GPIO_Pin2           GPIO_Pin_7   
#define ENCODER1_GPIO_Clk2           RCC_AHB1Periph_GPIOA
//--------------------------ENCODER2_TIM2----------------------------/			 //FR_A FR_B    //≤ªø…”√
#define ENCODER2_GPIO_Port1          GPIOA
#define ENCODER2_GPIO_Pin1           GPIO_Pin_2
#define ENCODER2_GPIO_Clk1           RCC_AHB1Periph_GPIOA

#define ENCODER2_GPIO_Port2          GPIOA
#define ENCODER2_GPIO_Pin2           GPIO_Pin_3
#define ENCODER2_GPIO_Clk2           RCC_AHB1Periph_GPIOA
//--------------------------ENCODER3_TIM8----------------------------/				//BL_A BL_B
#define ENCODER3_GPIO_Port1          GPIOC
#define ENCODER3_GPIO_Pin1           GPIO_Pin_6
#define ENCODER3_GPIO_Clk1           RCC_AHB1Periph_GPIOC

#define ENCODER3_GPIO_Port2          GPIOC
#define ENCODER3_GPIO_Pin2           GPIO_Pin_7   
#define ENCODER3_GPIO_Clk2           RCC_AHB1Periph_GPIOC
//--------------------------ENCODER4_TIM1----------------------------/				//BR_A BR_B
#define ENCODER4_GPIO_Port1          GPIOE
#define ENCODER4_GPIO_Pin1           GPIO_Pin_9
#define ENCODER4_GPIO_Clk1           RCC_AHB1Periph_GPIOE

#define ENCODER4_GPIO_Port2          GPIOE
#define ENCODER4_GPIO_Pin2           GPIO_Pin_11
#define ENCODER4_GPIO_Clk2           RCC_AHB1Periph_GPIOE

void ENCODER_INIT(void);

#endif	/* __BSP_ENCODER_H */

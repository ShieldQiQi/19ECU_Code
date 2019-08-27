#ifndef __ASSI_H
#define __ASSI_H

#include "stm32f4xx.h"

//无人驾驶状态信号指示灯

//---------------ASSI_CH1-----------------/
#define ASSI1_PIN                 GPIO_Pin_13                 
#define ASSI1_GPIO_PORT           GPIOE                
#define ASSI1_GPIO_CLK            RCC_AHB1Periph_GPIOE

//---------------ASSI_CH2-----------------/
#define ASSI2_PIN                 GPIO_Pin_9                 
#define ASSI2_GPIO_PORT           GPIOD                
#define ASSI2_GPIO_CLK            RCC_AHB1Periph_GPIOD

//---------------ASSI_CH3-----------------/
#define ASSI3_PIN                 GPIO_Pin_7                 
#define ASSI3_GPIO_PORT           GPIOG                
#define ASSI3_GPIO_CLK            RCC_AHB1Periph_GPIOG


#define NoneLight									GPIO_ResetBits(ASSI1_GPIO_PORT,ASSI1_PIN);\
																		GPIO_ResetBits(ASSI2_GPIO_PORT,ASSI2_PIN);\
																		GPIO_ResetBits(ASSI3_GPIO_PORT,ASSI3_PIN)
//YellowLight
#define BlueFlash								  GPIO_ResetBits(ASSI1_GPIO_PORT,ASSI1_PIN);\
																		GPIO_SetBits(ASSI2_GPIO_PORT,ASSI2_PIN);\
																		GPIO_SetBits(ASSI3_GPIO_PORT,ASSI3_PIN)

#define YellowFlash								GPIO_SetBits(ASSI1_GPIO_PORT,ASSI1_PIN);\
																		GPIO_SetBits(ASSI2_GPIO_PORT,ASSI2_PIN);\
																		GPIO_SetBits(ASSI3_GPIO_PORT,ASSI3_PIN)

#define BlueLight									GPIO_ResetBits(ASSI1_GPIO_PORT,ASSI1_PIN);\
																		GPIO_SetBits(ASSI2_GPIO_PORT,ASSI2_PIN);\
																		GPIO_ResetBits(ASSI3_GPIO_PORT,ASSI3_PIN)

#define YellowLight									GPIO_SetBits(ASSI1_GPIO_PORT,ASSI1_PIN);\
																		GPIO_SetBits(ASSI2_GPIO_PORT,ASSI2_PIN);\
																		GPIO_ResetBits(ASSI3_GPIO_PORT,ASSI3_PIN)
																		
void ASSI_GPIO_Config(void);

#endif  /*__ASSI_H*/

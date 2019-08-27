#ifndef __BRAKE_LED_H
#define __BRAKE_LED_H

#include "stm32f4xx.h"

//制动指示灯IO口宏定义

#define BRAKE_LED_PIN                  GPIO_Pin_8                 
#define BRAKE_LED_GPIO_PORT            GPIOG                 
#define BRAKE_LED_GPIO_CLK             RCC_AHB1Periph_GPIOG

#define BRAKE_LED_ON					  			 GPIO_ResetBits(BRAKE_LED_GPIO_PORT,BRAKE_LED_PIN);
#define BRAKE_LED_OFF					  			 GPIO_SetBits(BRAKE_LED_GPIO_PORT,BRAKE_LED_PIN);

void BRAKE_LED_GPIO_Config(void);

#endif  /*__BRAKE_LED_H*/

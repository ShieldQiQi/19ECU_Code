#ifndef __CIRCULT_H
#define __CIRCULT_H

#include "stm32f4xx.h"


#define SC_PIN                  GPIO_Pin_0                 
#define SC_GPIO_PORT            GPIOD                 
#define SC_GPIO_CLK             RCC_AHB1Periph_GPIOD

#define BREAK  	1
#define CLOSE 	0



#define SC(a)		if (a)	\
					GPIO_SetBits(SC_GPIO_PORT,SC_PIN);\
					else		\
					GPIO_ResetBits(SC_GPIO_PORT,SC_PIN)
					
void SC_GPIO_Config(void);
					
#endif /*__CIRCULT_H*/

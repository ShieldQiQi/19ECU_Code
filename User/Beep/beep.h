#ifndef __BEEP_H
#define __BEEP_H

#include "stm32f4xx.h"

//·äÃùÆ÷IO¿ÚÊä³ö

#define BEEP_PIN                  GPIO_Pin_3                 
#define BEEP_GPIO_PORT            GPIOD                
#define BEEP_GPIO_CLK             RCC_AHB1Periph_GPIOD

#define BEEP_IO_TOGGLE					  {GPIOD->ODR ^=GPIO_Pin_3;}


void BEEP_GPIO_Config(void);


#endif  /*__BEEP_H*/


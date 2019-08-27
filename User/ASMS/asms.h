#ifndef __ASMS_H
#define __ASMS_H

#include "stm32f4xx.h"

//无人驾驶系统主开关

#define ASMS_PIN                 GPIO_Pin_2                 
#define ASMS_GPIO_PORT           GPIOE                 
#define ASMS_GPIO_CLK            RCC_AHB1Periph_GPIOE
#define ASMS_EXTI_PORTSOURCE 		 EXTI_PortSourceGPIOE
#define ASMS_EXTI_PINSOURCE  		 EXTI_PinSource2
#define ASMS_EXTI_LINE        	 EXTI_Line2
#define ASMS_EXTI_IRQ            EXTI2_IRQn
#define ASMS_IRQHandler          EXTI2_IRQHandler

void ASMS_GPIO_Config(void);

#endif  /*__ASMS*/

#ifndef __BCKP_IO_H
#define __BCKP_IO_H

#include "stm32f4xx.h"

//备用IO口输出输入

//---------------BCKP_OUT1-----------------/
#define BCKP_OUT1_PIN                 GPIO_Pin_6                 
#define BCKP_OUT1_GPIO_PORT           GPIOD                
#define BCKP_OUT1_GPIO_CLK            RCC_AHB1Periph_GPIOD

//---------------BCKP_OUT2-----------------/
#define BCKP_OUT2_PIN                 GPIO_Pin_7                 
#define BCKP_OUT2_GPIO_PORT           GPIOD                
#define BCKP_OUT2_GPIO_CLK            RCC_AHB1Periph_GPIOD

//---------------BCKP_OUT3-----------------/
#define BCKP_OUT3_PIN                 GPIO_Pin_0                 
#define BCKP_OUT3_GPIO_PORT           GPIOF              
#define BCKP_OUT3_GPIO_CLK            RCC_AHB1Periph_GPIOF

//---------------BCKP_OUT4-----------------/
#define BCKP_OUT4_PIN                 GPIO_Pin_14                 
#define BCKP_OUT4_GPIO_PORT           GPIOG                
#define BCKP_OUT4_GPIO_CLK            RCC_AHB1Periph_GPIOG

//---------------BCKP_OUT5-----------------/
#define BCKP_OUT5_PIN                 GPIO_Pin_11                 
#define BCKP_OUT5_GPIO_PORT           GPIOG                
#define BCKP_OUT5_GPIO_CLK            RCC_AHB1Periph_GPIOG

//---------------BCKP_OUT6-----------------/
#define BCKP_OUT6_PIN                 GPIO_Pin_7                 
#define BCKP_OUT6_GPIO_PORT           GPIOA              
#define BCKP_OUT6_GPIO_CLK            RCC_AHB1Periph_GPIOA

//---------------RUN_ENABLE-----------------/
#define RUN_ENABLE_PIN                    GPIO_Pin_1                 
#define RUN_ENABLE_GPIO_PORT              GPIOC                
#define RUN_ENABLE_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define RUN_ENABLE_EXTI_PORTSOURCE 			  EXTI_PortSourceGPIOC
#define RUN_ENABLE_EXTI_PINSOURCE  			  EXTI_PinSource1
#define RUN_ENABLE_EXTI_LINE        			EXTI_Line1
#define RUN_ENABLE_EXTI_IRQ               EXTI1_IRQn
#define RUN_ENABLE_IRQHandler             EXTI1_IRQHandler

//---------------TS detect-----------------/
#define isTS_ON_PIN                  GPIO_Pin_5                
#define isTS_ON_GPIO_PORT           GPIOA               
#define isTS_ON_GPIO_CLK            RCC_AHB1Periph_GPIOA

//---------------BCKP_IN2-----------------/
#define BCKP_IN2_PIN                 GPIO_Pin_3                
#define BCKP_IN2_GPIO_PORT           GPIOB               
#define BCKP_IN2_GPIO_CLK            RCC_AHB1Periph_GPIOB

//---------------BCKP_IN3-----------------/
#define BCKP_IN3_PIN                 GPIO_Pin_4                
#define BCKP_IN3_GPIO_PORT           GPIOB                
#define BCKP_IN3_GPIO_CLK            RCC_AHB1Periph_GPIOB

//---------------BCKP_IN4-----------------/
#define BCKP_IN4_PIN                 GPIO_Pin_5                 
#define BCKP_IN4_GPIO_PORT           GPIOB                
#define BCKP_IN4_GPIO_CLK            RCC_AHB1Periph_GPIOB


#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态
					
#define ToDrive_LED_ON	 		  GPIO_SetBits(BCKP_OUT1_GPIO_PORT, BCKP_OUT1_PIN);
#define ToDrive_LED_OFF	 		  GPIO_ResetBits(BCKP_OUT1_GPIO_PORT, BCKP_OUT1_PIN);

void BCKP_IO_GPIO_Config(void);

#endif  /*__BCKP_IO_H*/





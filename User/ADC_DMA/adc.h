#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"

#define DMA_Data_size          3

//------------------------ACC1_通道1---------------------------/
#define ADC_GPIO_PORT1         GPIOC
#define ADC_GPIO_PIN1          GPIO_Pin_0
#define ADC_GPIO_CLK1          RCC_AHB1Periph_GPIOC
#define ADC_CHANNEL1           ADC_Channel_10
//------------------------ACC2_通道2---------------------------/
#define ADC_GPIO_PORT2         GPIOA
#define ADC_GPIO_PIN2          GPIO_Pin_4
#define ADC_GPIO_CLK2          RCC_AHB1Periph_GPIOA
#define ADC_CHANNEL2           ADC_Channel_4
//------------------------DCC_通道3---------------------------/
#define ADC_GPIO_PORT3         GPIOC
#define ADC_GPIO_PIN3          GPIO_Pin_2
#define ADC_GPIO_CLK3          RCC_AHB1Periph_GPIOC
#define ADC_CHANNEL3           ADC_Channel_12
#define DCC_IRQ            		 ADC_IRQn
#define DCC_IRQHandler   			 ADC_IRQHandler

#define HighThreshold    0xBA3
#define LowThreshold     0x000

//------------------------ADC----------------------------/
#define ADC_X                  ADC1
#define ADC_CLK                RCC_APB2Periph_ADC1
#define ADC_DR_ADDR            ((u32)&ADC1->DR)             //DR寄存器地址
//------------------------DMA----------------------------/
#define ADC_DMA_CLK            RCC_AHB1Periph_DMA2
#define ADC_DMA_CHANNEL        DMA_Channel_0
#define ADC_DMA_STREAM         DMA2_Stream0

void ADC_INIT(void);

#endif  /*__ADC_H*/ 

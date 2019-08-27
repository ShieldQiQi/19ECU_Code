#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

//״ָ̬ʾ�ơ���˸��ʾ��������
#define RUNNING_LED_PIN                  GPIO_Pin_3                 
#define RUNNING_LED_GPIO_PORT            GPIOC                 
#define RUNNING_LED_GPIO_CLK             RCC_AHB1Periph_GPIOC

//CAN1��CAN2 ��ʾ�շ�����
#define WORKING_CAN_PIN                  GPIO_Pin_7                 
#define WORKING_CAN_GPIO_PORT            GPIOF                 
#define WORKING_CAN_GPIO_CLK             RCC_AHB1Periph_GPIOF

//�������ơ���ʾ�����ת��
#define CAR_STATU_PIN                 	 GPIO_Pin_8                 
#define CAR_STATU_GPIO_PORT           	 GPIOF                 
#define CAR_STATU_GPIO_CLK            	 RCC_AHB1Periph_GPIOF

#define ON  0
#define OFF 1

//-----------------------------���κ�

#define RUNNING_LED(a)		if (a)	\
					GPIO_SetBits(RUNNING_LED_GPIO_PORT,RUNNING_LED_PIN);\
					else		\
					GPIO_ResetBits(RUNNING_LED_GPIO_PORT,RUNNING_LED_PIN)

#define CAN_LED(a)				if (a)	\
					GPIO_SetBits(WORKING_CAN_GPIO_PORT,WORKING_CAN_PIN);\
					else		\
					GPIO_ResetBits(WORKING_CAN_GPIO_PORT,WORKING_CAN_PIN)
					
#define CAR_STATU_LED(a)	if (a)	\
					GPIO_SetBits(CAR_STATU_GPIO_PORT,CAR_STATU_PIN);\
					else		\
					GPIO_ResetBits(CAR_STATU_GPIO_PORT,CAR_STATU_PIN)

//-----------------------------�������IO�ĺ�
		
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//�����ת״̬
					
#define CAN_RECEIVE_TOGGLE		digitalToggle(RUNNING_LED_GPIO_PORT,RUNNING_LED_PIN)

#define CAN2_LED_TOGGLE				digitalToggle(WORKING_CAN_GPIO_PORT,WORKING_CAN_PIN)

#define CAN1_LED_TOGGLE				digitalToggle(CAR_STATU_GPIO_PORT,CAR_STATU_PIN)

void LED_GPIO_Config(void);

#endif /* __LED_H */


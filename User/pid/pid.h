#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

#define Kp		800.0f
#define Ki		1000.0f
#define Kd		0.0f

uint16_t	Get_PID_Output(float Va,float Vt,uint16_t ACC2_Value);


#endif  /*__PID_H*/

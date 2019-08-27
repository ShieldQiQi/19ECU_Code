/**
Date:2018-08-17
Author:SHIELD_QI
Project:增量式PID控制、驱动防滑系统（ASR）
**************************/

#include "pid.h"

uint16_t  v_f=0;

float bias_now=0;
float bias_last=0;
float bias_last_before=0;

uint16_t Get_PID_Output(float Va,float Vt,uint16_t ACC2_Value)
{
	//误差赋值
	bias_last_before=bias_last;
	bias_last=bias_now;
	//当前滑转率、控制在16%
	bias_now=(Vt-Va)/Vt-0.16f;
	
	v_f=Kp*(bias_now-bias_last)+Ki*bias_now+Kd*(bias_now-2*bias_last+bias_last_before);
	//滑转率控制不超过车手控制的15%
	v_f = v_f>(ACC2_Value-120)/1980.0*0X7332*0.15
	?(uint16_t)(ACC2_Value-120)/1980.0*0X7332*0.15:v_f;
	
	//返回pid控制器输出、滑转率小于16%不进行滑转率控制
//	if(v_f>0&&bias_now>0)
//		return v_f;
//	else
		return 0;
}

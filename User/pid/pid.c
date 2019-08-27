/**
Date:2018-08-17
Author:SHIELD_QI
Project:����ʽPID���ơ���������ϵͳ��ASR��
**************************/

#include "pid.h"

uint16_t  v_f=0;

float bias_now=0;
float bias_last=0;
float bias_last_before=0;

uint16_t Get_PID_Output(float Va,float Vt,uint16_t ACC2_Value)
{
	//��ֵ
	bias_last_before=bias_last;
	bias_last=bias_now;
	//��ǰ��ת�ʡ�������16%
	bias_now=(Vt-Va)/Vt-0.16f;
	
	v_f=Kp*(bias_now-bias_last)+Ki*bias_now+Kd*(bias_now-2*bias_last+bias_last_before);
	//��ת�ʿ��Ʋ��������ֿ��Ƶ�15%
	v_f = v_f>(ACC2_Value-120)/1980.0*0X7332*0.15
	?(uint16_t)(ACC2_Value-120)/1980.0*0X7332*0.15:v_f;
	
	//����pid�������������ת��С��16%�����л�ת�ʿ���
//	if(v_f>0&&bias_now>0)
//		return v_f;
//	else
		return 0;
}

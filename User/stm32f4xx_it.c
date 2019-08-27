/*
* Date:2019-08-15
* Author:SHIELD_QI
* Project:ECU_HRT-19D
************************************/

#include "stm32f4xx_it.h"
#include "can.h"
#include "debug_usart.h"
#include "adc.h"
#include "led.h"
#include "bckp_io.h"
#include "beep.h"
#include "brake_led.h"
#include "msg_queue.h"
#include "circult.h"
#include "pid.h"
#include "assi.h"
#include "asms.h"
#include <math.h>
/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
																												变量及函数声明
*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-**/

//------------Show the statu------------/

uint8_t 			 ToDriveFlag=0;										//待驶模式标志位、1表示进入待驶模式、电机可对加速踏板做出响应
uint8_t        RES_Flag=1;											//急停置0【无人】
uint8_t				 EBS_Flag=3;
uint8_t				 E_Switch_Flag=1;
uint8_t				 RES_LOSE_Flag=0;		
uint8_t				 ROS_LOSE_Flag=0;	
uint8_t				 Motor_LOSE_Flag=0;
uint8_t				 ESC_LOSE_Flag=0;	
uint8_t				 STEERING_LOSE_Flag=0;	
uint8_t        Steer_Flag = 2;
uint8_t        isSteer_Enable = 0;
uint8_t				 EBS_LOSE_Flag=0;	
uint8_t				 ESC_Flag=0;
uint8_t				 ADC_Flag=1;											//错误置0
uint8_t				 Sudden_Flag=1;										//油门与制动踏板冲突标志位
uint8_t        ACC_ErrorFlag=0;									//两路加速踏板信号一致检测、置1进入100ms检测
uint8_t				 Stop_Flag=1;											//总制动标志位、制动置0
uint8_t				 Statu_Flag=0;										//0有人(无人驾驶关闭)、1准备、2行驶、3完成、4制动
uint16_t			 Beep_man=0;											//进入待驶模式
uint8_t				 Beep_ros=3;											//启动鸣笛为0、制动鸣笛为1
uint16_t			 Beep_ros_count=0;								//蜂鸣时间计数
uint8_t				 Brake_Flag=0;										//制动标志位
uint8_t				 Match_Mode=0;										//比赛模式，0有人、1直线加速、2八字、3高速循迹、4制动测试、5无人检查
uint8_t				 Error_Flag=200;									//用于仪表盘显示错误源
uint8_t				 ROS_FastBrake=0;									//用于直线加速结束后在规定时间内停车
uint8_t				 isTS_ON=0;												//高压状态位、0表示高压未激活
uint8_t				 isFinshed=0;											//比赛是否完成
uint16_t			 BMS_TIME_Flag=0;									//检测高压是否开启
uint16_t			 r=89;
uint16_t			 HV_voltage=0;
uint8_t 			 tmp =0;
uint8_t        oilPressure_enough = 1;
uint8_t        oilPressure_bl = 0;
uint8_t        oilPressure_br = 0;
uint8_t        oilPressure_fl = 0;
uint8_t        oilPressure_fr = 0;
uint8_t        gasPressure_enough = 1;
uint8_t 			 SC_breaker =3;
uint8_t 			 temp_pump =0;
uint8_t    		 Bms_State = 0;

//-------------For Steering-------------/

float			 		 steering_angle;
uint8_t				 is_SteerEnable = 0;

//---------------For ADC---------------/

uint16_t			 DCC_Value=0;
uint16_t			 ACC2_0_Value=0;
uint16_t			 ACC1_Value=0;
uint16_t			 ACC2_Value=0;
uint16_t			 ACC1_meanValue=0;
uint16_t			 ACC2_meanValue=0;
uint16_t			 ADC_ValueLocal[5][3];
extern 	 	 		 __IO uint16_t  ADC_ConvertedValue[DMA_Data_size];

//---------------For CAN1---------------/

extern		 		 LinkQueue Can1Msg_Queue;
extern		 		 CanTxMsg	 ROS_TxMsg;
extern		 		 CanRxMsg	 CAN1_RxMessage;
//extern		 		 CanTxMsg	 Disable_Msg;
//extern		 		 CanRxMsg	 Statu_Get_Msg;
//extern		 		 CanTxMsg  StatuGet_Command_Msg;
//extern		 		 CanTxMsg  SpeedGet_Command_Msg;
//extern		 		 CanTxMsg  Position_Cntrol_Msg;
//extern		 		 CanTxMsg  Torque_Cntrol_Msg;
//extern 		     CanTxMsg  Speed_Cntrol_Msg;
extern		 		 CanTxMsg	 AMK_SETPOINT_RF;
extern		 		 CanTxMsg	 AMK_SETPOINT_LF;
extern		 		 CanTxMsg	 AMK_SETPOINT_RB;
extern		 		 CanTxMsg	 AMK_SETPOINT_LB;

extern				 CanRxMsg  AMK_RF_V1;
extern				 CanRxMsg  AMK_RF_V2;
extern				 CanRxMsg  AMK_LF_V1;
extern				 CanRxMsg  AMK_LF_V2;
extern				 CanRxMsg  AMK_RB_V1;
extern				 CanRxMsg  AMK_RB_V2;
extern				 CanRxMsg  AMK_LB_V1;
extern				 CanRxMsg  AMK_LB_V2;

AMK_ActualValue1	Amk_actualVel_rf_1;
AMK_ActualValue1	Amk_actualVel_lf_1;
AMK_ActualValue1	Amk_actualVel_rb_1;
AMK_ActualValue1	Amk_actualVel_lb_1;

AMK_ActualValue2	Amk_actualVel_rf_2;
AMK_ActualValue2	Amk_actualVel_lf_2;
AMK_ActualValue2	Amk_actualVel_rb_2;
AMK_ActualValue2	Amk_actualVel_lb_2;

AMK_SetPoint      Amk_setPoint_rf;
AMK_SetPoint      Amk_setPoint_lf;
AMK_SetPoint      Amk_setPoint_rb;
AMK_SetPoint      Amk_setPoint_lf;

//---------------For CAN2---------------/

extern		 		 LinkQueue Can2Msg_Queue;
extern	    	 CanTxMsg RTC_Msg;
extern	    	 CanRxMsg CAN2_RxMessage;
extern 		  	 CanTxMsg HRT_Ctrl_1;
extern 		  	 CanRxMsg Screen_Rx_Message;
extern 		  	 CanRxMsg RES_Rx_Message;
extern 		  	 CanTxMsg ESC_Tx_Message;
extern 		  	 CanRxMsg ESC_Rx_Message;
extern 		  	 CanTxMsg Steering_Tx_Message0;
extern 		  	 CanTxMsg Steering_Tx_Message;
extern 		  	 CanRxMsg Steering_Rx_Message;
extern 		  	 CanTxMsg EBS_Tx_Message;
extern 		  	 CanRxMsg EBS_Rx_Message;

//--------For the speed Encoder--------/

u16  					 Count_fr=0;
u16 					 Count_fl=0;
double 				 frv[5]={0,0,0,0,0};
double 				 flv[5]={0,0,0,0,0};										//滤波数组
double 				 encoder_flv;
double 				 encoder_frv;														//编码器速度

//-------------电机控制器逻辑-------------/
uint32_t 	 		 time0=0;
uint8_t 	 		 time1=0;

float 	       motor_vel_v=0;
uint16_t 	     motor_torque=160;
uint8_t		     motor_num=9;

float   	     motor_ctrl_v[10]={0,0,0,0,0,0,0,0,0,0};	//滤波数组
uint16_t       motor_ctrl_V=0;

//-----------------RTC-------------------/

RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;

//-------------------------CAN1发送函数-------------------------/

u8 CAN1_Ctrl_Send(CanTxMsg txmsg,uint16_t ACC1 ,uint16_t Angle)
{
	int i=0;

	txmsg.Data[2] = (uint8_t)((ACC1-100)/2000.0*0.4*0X3FFF);
	txmsg.Data[3] = (uint8_t)(((uint16_t)((ACC1-100)/2000.0*0.4*0X3FFF)) >> 8);
	
	txmsg.Data[4] = ACC1 == 100 ?0:0xFF;//(uint8_t)((ACC2-700)/200.0*0X3FFF);
	txmsg.Data[5] = ACC1 == 100 ?0:0x2F;//uint8_t(((uint16_t)((ACC2-120)/200.0*0X3FFF)) >> 8);
	
	txmsg.Data[6] = ACC1 == 100 ?0:0;//ACC2 == 700 ?0:0x14;//(uint8_t)((ACC2-700)/200.0*0X3FFF);
	txmsg.Data[7] = ACC1 == 100 ?0:0;//uint8_t(((uint16_t)((ACC2-120)/200.0*0X3FFF)) >> 8);
	
	uint8_t mbox=CAN_Transmit(CAN1, &txmsg);
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
		i++;	
  if(i>=0XFFF)
		return 0;
  return 1;
}

u8 ROS_Ctrl_Send(CanTxMsg txmsg)
{
	int i=0;
	uint8_t mbox=CAN_Transmit(CAN1, &txmsg);
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
		i++;	
  if(i>=0XFFF)
		return 0;
  return 1;
}

//-------------------------CAN2发送函数-------------------------/
u8 CAN2_Ctrl_Send(CanTxMsg txmsg)
{
	int i=0;
	uint8_t mbox=CAN_Transmit(CAN2, &txmsg);
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
		i++;	
  if(i>=0XFFF)
		return 0;
  return 1;
}
//------------------------滞回曲线-制动踏板----------------------/
void isBrake(void)
{
	if(DCC_Value>1250){				//踩下制动
		Brake_Flag=1;
		BRAKE_LED_ON;
	}
	else if(DCC_Value<1000&&Stop_Flag!=0){		//松下制动
		Brake_Flag=0;
		BRAKE_LED_OFF;
	}
}

/*----*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
																												中断服务函数
*----*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/


//------------------------------------------定时中断TIM5：100 HZ == 10ms------------------------------------------/

void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	{
//		if(BMS_TIME_Flag!=0&&BMS_TIME_Flag<=300&&GPIO_ReadOutputDataBit(SC_GPIO_PORT,SC_PIN)==0)
//			BMS_TIME_Flag++;
//		else if(BMS_TIME_Flag==301&&isTS_ON==0){
//			BMS_TIME_Flag=0;
//			isTS_ON=0;
//			//高压断开、退出待驶模式
//			ToDriveFlag=0;
//			Beep_man=0;
//			ToDrive_LED_OFF;
//			//SC(BREAK);
//		}
		
//		if(GPIO_ReadInputDataBit(isTS_ON_GPIO_PORT, isTS_ON_PIN)==1){
//			isTS_ON = 1;
//		}
//		else{
//			isTS_ON = 0;
//			ToDriveFlag=0;
//			ToDrive_LED_OFF;
//		}
		
		if(Amk_actualVel_lf_1.Amk_bQuitDcOn == 0)
		{
			//ToDriveFlag=0;
			//Beep_man = 0;
			//ToDrive_LED_OFF;
			HRT_Ctrl_1.Data[7] = 0;
		}else
			HRT_Ctrl_1.Data[7] = 1;
		
		if(Stop_Flag) //防止标志位无限加导致的溢出
		{
			if(!motor_num)
			{
				Motor_LOSE_Flag++;
			}
			EBS_LOSE_Flag++;
			STEERING_LOSE_Flag++;
			//ESC_LOSE_Flag++;
			if(Statu_Flag)//【无人】
			{
				ROS_LOSE_Flag++;
				RES_LOSE_Flag++;
			}
		}
		
		//----------------------------------编码器、AD-------------------------------------/
		switch(time0%2)
		{
			case 0:		//--------Decoder The Amk data--------/
				Amk_actualVel_rf_1.Amk_actualVelocity = ((uint16_t)(AMK_RF_V1.Data[3] & 0x7F))<<8 | AMK_RF_V1.Data[2];
				Amk_actualVel_rf_1.Amk_bDcOn = 				(AMK_RF_V1.Data[1] >> 4) & 1;
				Amk_actualVel_rf_1.Amk_bError = 			(AMK_RF_V1.Data[1] >> 1) & 1;
				Amk_actualVel_rf_1.Amk_bQuitDcOn = 		(AMK_RF_V1.Data[1] >> 3) & 1;
				Amk_actualVel_rf_1.Amk_bSystemReady = (AMK_RF_V1.Data[1] >> 0) & 1;
				Amk_actualVel_rf_1.Amk_bWarn = 				(AMK_RF_V1.Data[1] >> 2) & 1;
				Amk_actualVel_rf_1.bDerating = 				(AMK_RF_V1.Data[1] >> 7) & 1;
				Amk_actualVel_rf_1.bInverterOn = 			(AMK_RF_V1.Data[1] >> 6) & 1;
				Amk_actualVel_rf_1.bQuitInverterOn = 	(AMK_RF_V1.Data[1] >> 5) & 1;
				Amk_actualVel_rf_2.Amk_TempIGBT = 		0.1*(((uint16_t)(AMK_RF_V2.Data[7] & 0x7F))<<8 | AMK_RF_V2.Data[6]);
				Amk_actualVel_rf_2.Amk_TempInverter = 0.1*(((uint16_t)(AMK_RF_V2.Data[3] & 0x7F))<<8 | AMK_RF_V2.Data[2]);
				Amk_actualVel_rf_2.Amk_TempMotor = 		0.1*(((uint16_t)(AMK_RF_V2.Data[1] & 0x7F))<<8 | AMK_RF_V2.Data[0]);
				Amk_actualVel_rf_2.ErrorInfo = 				((uint16_t)(AMK_RF_V2.Data[5] & 0x7F))<<8 | AMK_RF_V2.Data[4];
				
				Amk_actualVel_lf_1.Amk_actualVelocity = ((uint16_t)(AMK_LF_V1.Data[3] & 0x7F))<<8 | AMK_LF_V1.Data[2];
				Amk_actualVel_lf_1.Amk_bDcOn = 				(AMK_LF_V1.Data[1] >> 4) & 1;
				Amk_actualVel_lf_1.Amk_bError = 			(AMK_LF_V1.Data[1] >> 1) & 1;
				Amk_actualVel_lf_1.Amk_bQuitDcOn = 		(AMK_LF_V1.Data[1] >> 3) & 1;
				Amk_actualVel_lf_1.Amk_bSystemReady = (AMK_LF_V1.Data[1] >> 0) & 1;
				Amk_actualVel_lf_1.Amk_bWarn = 				(AMK_LF_V1.Data[1] >> 2) & 1;
				Amk_actualVel_lf_1.bDerating = 				(AMK_LF_V1.Data[1] >> 7) & 1;
				Amk_actualVel_lf_1.bInverterOn = 			(AMK_LF_V1.Data[1] >> 6) & 1;
				Amk_actualVel_lf_1.bQuitInverterOn = 	(AMK_LF_V1.Data[1] >> 5) & 1;
				Amk_actualVel_lf_2.Amk_TempIGBT = 		0.1*(((uint16_t)(AMK_LF_V2.Data[7] & 0x7F))<<8 | AMK_LF_V2.Data[6]);
				Amk_actualVel_lf_2.Amk_TempInverter = 0.1*(((uint16_t)(AMK_LF_V2.Data[3] & 0x7F))<<8 | AMK_LF_V2.Data[2]);
				Amk_actualVel_lf_2.Amk_TempMotor = 		0.1*(((uint16_t)(AMK_LF_V2.Data[1] & 0x7F))<<8 | AMK_LF_V2.Data[0]);
				Amk_actualVel_lf_2.ErrorInfo = 				((uint16_t)(AMK_LF_V2.Data[5] & 0x7F))<<8 | AMK_LF_V2.Data[4];
				
				Amk_actualVel_rb_1.Amk_actualVelocity = ((uint16_t)(AMK_RB_V1.Data[3] & 0x7F))<<8 | AMK_RB_V1.Data[2];
				Amk_actualVel_rb_1.Amk_bDcOn = 				(AMK_RB_V1.Data[1] >> 4) & 1;
				Amk_actualVel_rb_1.Amk_bError = 			(AMK_RB_V1.Data[1] >> 1) & 1;
				Amk_actualVel_rb_1.Amk_bQuitDcOn = 		(AMK_RB_V1.Data[1] >> 3) & 1;
				Amk_actualVel_rb_1.Amk_bSystemReady = (AMK_RB_V1.Data[1] >> 0) & 1;
				Amk_actualVel_rb_1.Amk_bWarn = 				(AMK_RB_V1.Data[1] >> 2) & 1;
				Amk_actualVel_rb_1.bDerating = 				(AMK_RB_V1.Data[1] >> 7) & 1;
				Amk_actualVel_rb_1.bInverterOn = 			(AMK_RB_V1.Data[1] >> 6) & 1;
				Amk_actualVel_rb_1.bQuitInverterOn = 	(AMK_RB_V1.Data[1] >> 5) & 1;
				Amk_actualVel_rb_2.Amk_TempIGBT = 		0.1*(((uint16_t)(AMK_RB_V2.Data[7] & 0x7F))<<8 | AMK_RB_V2.Data[6]);
				Amk_actualVel_rb_2.Amk_TempInverter = 0.1*(((uint16_t)(AMK_RB_V2.Data[3] & 0x7F))<<8 | AMK_RB_V2.Data[2]);
				Amk_actualVel_rb_2.Amk_TempMotor = 		0.1*(((uint16_t)(AMK_RB_V2.Data[1] & 0x7F))<<8 | AMK_RB_V2.Data[0]);
				Amk_actualVel_rb_2.ErrorInfo = 				((uint16_t)(AMK_RB_V2.Data[5] & 0x7F))<<8 | AMK_RB_V2.Data[4];
				
				Amk_actualVel_lb_1.Amk_actualVelocity = ((uint16_t)(AMK_LB_V1.Data[3] & 0x7F))<<8 | AMK_LB_V1.Data[2];
				Amk_actualVel_lb_1.Amk_bDcOn = 				(AMK_LB_V1.Data[1] >> 4) & 1;
				Amk_actualVel_lb_1.Amk_bError = 			(AMK_LB_V1.Data[1] >> 1) & 1;
				Amk_actualVel_lb_1.Amk_bQuitDcOn = 		(AMK_LB_V1.Data[1] >> 3) & 1;
				Amk_actualVel_lb_1.Amk_bSystemReady = (AMK_LB_V1.Data[1] >> 0) & 1;
				Amk_actualVel_lb_1.Amk_bWarn = 				(AMK_LB_V1.Data[1] >> 2) & 1;
				Amk_actualVel_lb_1.bDerating = 				(AMK_LB_V1.Data[1] >> 7) & 1;
				Amk_actualVel_lb_1.bInverterOn = 			(AMK_LB_V1.Data[1] >> 6) & 1;
				Amk_actualVel_lb_1.bQuitInverterOn = 	(AMK_LB_V1.Data[1] >> 5) & 1;
				Amk_actualVel_lb_2.Amk_TempIGBT = 		0.1*(((uint16_t)(AMK_LB_V2.Data[7] & 0x7F))<<8 | AMK_LB_V2.Data[6]);
				Amk_actualVel_lb_2.Amk_TempInverter = 0.1*(((uint16_t)(AMK_LB_V2.Data[3] & 0x7F))<<8 | AMK_LB_V2.Data[2]);
				Amk_actualVel_lb_2.Amk_TempMotor = 		0.1*(((uint16_t)(AMK_LB_V2.Data[1] & 0x7F))<<8 | AMK_LB_V2.Data[0]);
				Amk_actualVel_lb_2.ErrorInfo = 				((uint16_t)(AMK_LB_V2.Data[5] & 0x7F))<<8 | AMK_LB_V2.Data[4];
				break;
			case 1:		//-----------Solve The ADC raw data------------/
				for(int j=3;j>=0;j--)
				{
					ADC_ValueLocal[j+1][0]=ADC_ValueLocal[j][0];
					ADC_ValueLocal[j+1][1]=ADC_ValueLocal[j][1];
					ADC_ValueLocal[j+1][2]=ADC_ValueLocal[j][2];
				}
				ADC_ValueLocal[0][0]=ADC_ConvertedValue[0];
				ADC_ValueLocal[0][1]=ADC_ConvertedValue[1];
				ADC_ValueLocal[0][2]=ADC_ConvertedValue[2];
				
				ACC1_meanValue=(ADC_ValueLocal[0][0]+ADC_ValueLocal[1][0]+ADC_ValueLocal[2][0]+ADC_ValueLocal[3][0]+ADC_ValueLocal[4][0])/5;
				ACC2_meanValue=(ADC_ValueLocal[0][1]+ADC_ValueLocal[1][1]+ADC_ValueLocal[2][1]+ADC_ValueLocal[3][1]+ADC_ValueLocal[4][1])/5;
				
				//ACC2角位移,ACC1线位移、加权滤波
				ACC1_Value=1/3.0*(0.8*ADC_ValueLocal[0][0]+0.7*ADC_ValueLocal[1][0]+0.6*ADC_ValueLocal[2][0]+0.5*ADC_ValueLocal[3][0]+0.4*ADC_ValueLocal[4][0])>100?\
									 1/3.0*(0.8*ADC_ValueLocal[0][0]+0.7*ADC_ValueLocal[1][0]+0.6*ADC_ValueLocal[2][0]+0.5*ADC_ValueLocal[3][0]+0.4*ADC_ValueLocal[4][0]):100;
				ACC2_Value=1/3.0*(0.8*ADC_ValueLocal[0][1]+0.7*ADC_ValueLocal[1][1]+0.6*ADC_ValueLocal[2][1]+0.5*ADC_ValueLocal[3][1]+0.4*ADC_ValueLocal[4][1])>700?\
									 1/3.0*(0.8*ADC_ValueLocal[0][1]+0.7*ADC_ValueLocal[1][1]+0.6*ADC_ValueLocal[2][1]+0.5*ADC_ValueLocal[3][1]+0.4*ADC_ValueLocal[4][1]):700;

				//三路AD断线检测
				if(ACC1_Value>2500||ACC2_Value>2500||DCC_Value>2700)
				{
					Error_Flag=0;
					ADC_Flag=0;
				}
				
				ACC1_Value=1/3.0*(0.8*ADC_ValueLocal[0][0]+0.7*ADC_ValueLocal[1][0]+0.6*ADC_ValueLocal[2][0]+0.5*ADC_ValueLocal[3][0]+0.4*ADC_ValueLocal[4][0])<2100?\
									 ACC1_Value:2100;
				ACC2_Value=1/3.0*(0.8*ADC_ValueLocal[0][1]+0.7*ADC_ValueLocal[1][1]+0.6*ADC_ValueLocal[2][1]+0.5*ADC_ValueLocal[3][1]+0.4*ADC_ValueLocal[4][1])<1200?\
									 ACC2_Value:1200;
				
				DCC_Value=1/3.0*(0.8*ADC_ValueLocal[0][2]+0.7*ADC_ValueLocal[1][2]+0.6*ADC_ValueLocal[2][2]+0.5*ADC_ValueLocal[3][2]+0.4*ADC_ValueLocal[4][2])>170?\
									 1/3.0*(0.8*ADC_ValueLocal[0][2]+0.7*ADC_ValueLocal[1][2]+0.6*ADC_ValueLocal[2][2]+0.5*ADC_ValueLocal[3][2]+0.4*ADC_ValueLocal[4][2]):170;
				
				//判断是否制动
				isBrake();
				//当加速踏板位置传感器的信号输出超过 25%并踩下制动踏板时，电机的目标转矩必须为 0N*m//36%
				if(ACC2_Value>813&&Brake_Flag)
				{
					//BRAKE_LED_ON;
					//Sudden_Flag=0;
					//CAN1_Ctrl_Send(Torque_Cntrol_Msg,0x00,0x00);		//目标转矩设为0
				}else if(ACC2_Value<215)
				{
					Sudden_Flag=1;
				}
				
				//触发检测 踏板不可靠信号
				if(pow((0.004297*ACC2_meanValue*ACC2_meanValue-0.4238*ACC2_meanValue-3585-ACC1_meanValue)/(0.004297*ACC2_meanValue*ACC2_meanValue-0.4238*ACC2_meanValue-3585),2)>0.01)
				{
					if(ACC_ErrorFlag<10)
						;//ACC_ErrorFlag++;
					//printf("%d \n",ACC_ErrorFlag);
				}else
				{
					//运行正常
					ACC_ErrorFlag=0;
				}
				break;
		}
		
		//----------------------------------------CAN1----------------------------------------/
		
		//每隔20ms发送电机控制报文
		if((time0)%2==0)
		{
			if(Stop_Flag&&(Statu_Flag==0))//----------------------------【有人】
			{
				if(Brake_Flag||ACC_ErrorFlag==10)//踏板信号超差、关闭动力输出
				{
					CAN1_Ctrl_Send(AMK_SETPOINT_RF,100,0);
					CAN1_Ctrl_Send(AMK_SETPOINT_LF,100,0);
					CAN1_LED_TOGGLE;
					BRAKE_LED_ON;
				}else
				{
					BRAKE_LED_OFF;
					if(ToDriveFlag==1 && Sudden_Flag)
					{
						CAN1_LED_TOGGLE;
						if(ACC2_Value!=100){
							CAN1_Ctrl_Send(AMK_SETPOINT_LF,ACC1_Value,steering_angle);
							CAN1_Ctrl_Send(AMK_SETPOINT_RF,ACC1_Value,steering_angle);
						}else{
							CAN1_Ctrl_Send(AMK_SETPOINT_LF,100,0);
							CAN1_Ctrl_Send(AMK_SETPOINT_RF,100,0);
						}
					}else
					{
						CAN1_Ctrl_Send(AMK_SETPOINT_LF,100,0);
						CAN1_Ctrl_Send(AMK_SETPOINT_RF,100,0);
						CAN1_LED_TOGGLE;
					}
				}
			}else if(Stop_Flag&&Statu_Flag)//---------------------------【无人】
			{
				if(RES_Flag==2)
				{
					//CAN1_Ctrl_Send(Speed_Cntrol_Msg,0,0);
				}else if(RES_Flag>=4&&DCC_Value<1400)		//RES按下启动且EBS不再制动制动踏板回位
				{
					if(RES_Flag!=6&&Statu_Flag==2&&ROS_FastBrake==0)
					{
						for(int j=8;j>=0;j--)
						{
							motor_ctrl_v[j+1]=motor_ctrl_v[j];
						}
						//无人模式速度控制，理论实际最大速度6000rpm、传动系统传动比40/13、1m/s对应701.95
//						if(/*Speed_Cntrol_Msg.Data[1]<20*/1)
//							motor_ctrl_v[0]=(Speed_Cntrol_Msg.Data[1]+Speed_Cntrol_Msg.Data[2]/100.0)*701.95*0.5;
						motor_ctrl_V=0;
						for(int j=0;j<10;j++)
							motor_ctrl_V+=motor_ctrl_v[j];
						motor_ctrl_V=(uint16_t)(1/10.0*motor_ctrl_V);
						
						CAN1_LED_TOGGLE;
						//CAN1_Ctrl_Send(Speed_Cntrol_Msg,0xFF-(uint8_t)(motor_ctrl_V)&0XFF,0xFF-(uint8_t)(((uint16_t)(motor_ctrl_V))>>8)|0x80);
					}else if(RES_Flag==6||Statu_Flag==1||Statu_Flag==3||Statu_Flag==4||ROS_FastBrake)
					{
						switch(ROS_FastBrake)
						{
							case 0://正常缓停
								for(int j=8;j>=0;j--)
								{
									motor_ctrl_v[j+1]=motor_ctrl_v[j];
								}
								motor_ctrl_v[0]=0;
								motor_ctrl_V=0;
								for(int j=0;j<10;j++)
									motor_ctrl_V+=motor_ctrl_v[j];
								motor_ctrl_V=(uint16_t)(1/10.0*motor_ctrl_V);
								
								//CAN1_Ctrl_Send(Speed_Cntrol_Msg,0xFF-(uint8_t)(motor_ctrl_V)&0XFF,0xFF-(uint8_t)(((uint16_t)(motor_ctrl_V))>>8)|0x80);
								CAN1_LED_TOGGLE;
								break;
							case 100://速度较高且要求在一定距离下停车
								//反扭矩制动同时配合EBS制动
								//-0.5*(encoder_flv+encoder_frv)<0.1?\
								//CAN1_Ctrl_Send(Torque_Cntrol_Msg,(uint8_t)(0&0X00FF),(uint8_t)(0>>8)):
								//CAN1_Ctrl_Send(Torque_Cntrol_Msg,(uint8_t)((uint16_t)(-1332.0/4000*pow((-0.5*(encoder_flv+encoder_frv)),3)+9.99*pow((-0.5*(encoder_flv+encoder_frv)),2))&0X00FF),\
									(uint8_t)((uint16_t)(-1332.0/4000*pow((-0.5*(encoder_flv+encoder_frv)),3)+9.99*pow((-0.5*(encoder_flv+encoder_frv)),2))>>8));
								break;
						}
					}
				}
			}else//【电机失能】
			{
				CAN1_Ctrl_Send(AMK_SETPOINT_RF,100,0);
				CAN1_LED_TOGGLE; 
				BRAKE_LED_ON;
				for(int j=9;j>=0;j--)
				{
					motor_ctrl_v[j]=0;
				}
			}
		}

		//每隔100ms发送工作站报文
		if(time0%10==0)
		{
			ROS_TxMsg.Data[0]=Match_Mode;																			//比赛模式，制动测试，八字等
			if(-0.5*(encoder_flv+encoder_frv)<0)															//速度小于0，车正在后退
			{	
				ROS_TxMsg.Data[7]=RES_Flag==4?3:2;															//RES_Flag==4表示遥控发来行驶信号
				ROS_TxMsg.Data[1]=(int)(0.5*(encoder_flv+encoder_frv));					//整车速度整数部分
				ROS_TxMsg.Data[2]=((int)(0.5*(encoder_flv+encoder_frv)*100))%100;//整车速度小数部分
			}else																															//速度大于0
			{
				ROS_TxMsg.Data[7]=RES_Flag==4?0:1;
				ROS_TxMsg.Data[1]=(int)(-0.5*(encoder_flv+encoder_frv));				//整车速度整数部分
				ROS_TxMsg.Data[2]=((int)(-0.5*(encoder_flv+encoder_frv)*100))%100;//整车速度小数部分		
			}
			ROS_TxMsg.Data[3]=Steering_Rx_Message.Data[3];						  			//前轮转角高八位
			ROS_TxMsg.Data[4]=Steering_Rx_Message.Data[4];								    //前轮转角低八位
			//电机实时转速
			//ROS_TxMsg.Data[5]=Speed_Get_Msg.Data[2];													//电机转速高八位
			//ROS_TxMsg.Data[6]=Speed_Get_Msg.Data[1];													//电机转速低八位
			
			ROS_Ctrl_Send(ROS_TxMsg);
		}
		
		//----------------------------------------CAN2----------------------------------------/
		
		//每隔100ms即10HZ频率发送整车控制报文
		if(time0%10==0)
		{
			CAN2_LED_TOGGLE;
			HRT_Ctrl_1.Data[0] = (uint8_t)(Statu_Flag+is_SteerEnable<<3+Error_Flag<<4);		
			
			//仪表盘显示错误源 0  ADC错误 1  RES_Flag 2  EBS_Flag 3  Motor_LOSE 4  ROS_LOSE 5  RES_LOSE 6  EBS_LOSE 7  ESC_LOSE 8  STEERING_LOSE
			CAN2_Ctrl_Send(HRT_Ctrl_1);
		}
		
		//每隔60ms即16.7HZ的频率发送时间基准
		if((time0-3)%23==0)
		{
			// 获取时间
			RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

			RTC_Msg.Data[0]=RTC_DateStructure.RTC_Year/100;
			RTC_Msg.Data[1]=RTC_DateStructure.RTC_Year%100;
			RTC_Msg.Data[2]=RTC_DateStructure.RTC_Date;
			RTC_Msg.Data[3]=RTC_DateStructure.RTC_WeekDay;
			RTC_Msg.Data[4]=RTC_TimeStructure.RTC_Hours;
			RTC_Msg.Data[5]=RTC_TimeStructure.RTC_Minutes;
			RTC_Msg.Data[6]=RTC_TimeStructure.RTC_Seconds;

			(void)RTC->DR;
			
			CAN2_LED_TOGGLE;
			CAN2_Ctrl_Send(RTC_Msg);
		}

		//每隔50ms 20HZ发送转向报文 //第一位115清零
		if((time0%5==0)&&Statu_Flag)
		{
			CAN2_LED_TOGGLE;
			CAN2_Ctrl_Send(Steering_Tx_Message);
		}
		
		//---------------------------------------逻辑模块---------------------------------------/
		
		//无人模式按下急停开关进入紧急制动状态
		if((Statu_Flag==1||Statu_Flag==2)&&isTS_ON==0&&RES_Flag!=0)
		{
			if(E_Switch_Flag!=0)
				Beep_ros=1;
			E_Switch_Flag=0;
			BlueFlash;
			Statu_Flag=4;
		}
		
//		//踏板不可靠信号超过100ms、关闭电机动力输出
//		if(ACC_ErrorFlag!=0)
//		{
//			if(ACC_ErrorFlag==11)
//			{ 
//				ADC_Flag=0;
//				Error_Flag=0;
//				BRAKE_LED_ON;
//				CAN1_Ctrl_Send(Speed_Cntrol_Msg,0x02,0x00);
//			}
//		}
		//ASMS
		if(GPIO_ReadInputDataBit(ASMS_GPIO_PORT, ASMS_PIN)==0)
		{
			if(Statu_Flag==0&&time0%5==0)
			{
				Steering_Tx_Message0.Data[0]=0;
				CAN2_Ctrl_Send(Steering_Tx_Message0);
			}
			Statu_Flag=0;
			//NoneLight;
		}else if(Statu_Flag==0&&Match_Mode!=0&&isFinshed==0)
		{
			Steering_Tx_Message0.Data[0]=1;
			CAN2_Ctrl_Send(Steering_Tx_Message0);
			Statu_Flag=1;
		}
		/*蜂鸣器--进入待驶模式*/
		if(ToDriveFlag&&(Beep_man<=150)) 
		{
			Beep_man++;
			if((Beep_man%40==0)||((Beep_man+20)%40==0))
				BEEP_IO_TOGGLE;
		}else if((Beep_man>=151||ToDriveFlag==0)&&Beep_ros==3)
		{
			GPIO_SetBits(BEEP_GPIO_PORT,BEEP_PIN);
		}
		/*蜂鸣器--无人启动与紧急制动鸣笛*/
		switch(Beep_ros)
		{
			case 0://启动鸣笛
				if(Beep_ros_count<150)
				{
					Beep_ros_count++;
					if(Beep_ros_count%10==0)
						BEEP_IO_TOGGLE;
				}else
				{
					GPIO_SetBits(BEEP_GPIO_PORT,BEEP_PIN);
					Beep_ros_count=0;
					Beep_ros=3;
				}
				break;
			case 1://紧急制动鸣笛、响18s、通断频率5HZ、占空比50%
				if(Beep_ros_count<1800)
				{
					Beep_ros_count++;
					if(Beep_ros_count%100==0)
						BEEP_IO_TOGGLE;
				}else
				{
					GPIO_SetBits(BEEP_GPIO_PORT,BEEP_PIN);
					Beep_ros_count=0;
					Beep_ros=3;
				}
				break;
		}
		
		//更新总状态标志位//
		if(!ADC_Flag||!RES_Flag||!EBS_Flag||(Motor_LOSE_Flag>30)||(ROS_LOSE_Flag>40)||(RES_LOSE_Flag>40)
			||(EBS_LOSE_Flag>40)||(ESC_LOSE_Flag>40)||(STEERING_LOSE_Flag>100))
		{
			//Stop_Flag=0;
			//SC(BREAK);
			//CAN1_Ctrl_Send(Torque_Cntrol_Msg,0,0);
			//ToDriveFlag=0;
		}else{
			Error_Flag=200;
		}
		
		//发送串口蓝牙至上位机
		if(time1==3)
		{
			//printf("%d %d %d\n",ACC1_meanValue,ACC2_meanValue,DCC_meanValue);ToDriveFlag
			//printf("%d %d %d\n",ToDriveFlag, Brake_Flag,DCC_Value);
			if(tmp%2==0)
			{
				USART_SendData(DEBUG_USART,tmp/2);
				tmp++;
			}else{
				switch((tmp-1)/2){
					case 0:
						USART_SendData(DEBUG_USART,Amk_actualVel_rf_1.Amk_bQuitDcOn+37);
						break;
					case 1:
						USART_SendData(DEBUG_USART,Match_Mode+37);
						break;
					case 2:
						USART_SendData(DEBUG_USART,Error_Flag+37);
						break;
					case 3:
						USART_SendData(DEBUG_USART,oilPressure_enough+37);
						break;
					case 4:
						USART_SendData(DEBUG_USART,RES_Flag+37);
						break;
					case 5:
						USART_SendData(DEBUG_USART,EBS_Flag+37);
						break;
					case 6:
						USART_SendData(DEBUG_USART,Steer_Flag+37);
						break;
					case 7:
						USART_SendData(DEBUG_USART,gasPressure_enough+37);
						break;
					case 8:
						USART_SendData(DEBUG_USART,Brake_Flag+37);
						break;
					case 9:
						USART_SendData(DEBUG_USART,isSteer_Enable+37);
						break;
					case 10:
						USART_SendData(DEBUG_USART,SC_breaker+37);
						break;
					case 11:
						USART_SendData(DEBUG_USART,Error_Flag+37);
						break;
					case 12:
						USART_SendData(DEBUG_USART,Statu_Flag+37);
						break;
					case 13:
						USART_SendData(DEBUG_USART,(uint8_t)((steering_angle-10+108)/2)+37);
						break;
					case 14:
						USART_SendData(DEBUG_USART,(uint8_t)(ACC2_Value/10.0)+37);
						break;
					case 15:
						USART_SendData(DEBUG_USART,(uint8_t)(ACC1_Value/10.0)+37);
						break;
					case 16:
						USART_SendData(DEBUG_USART,(uint8_t)(DCC_Value/10.0)+37);
						break;
					case 17:
						USART_SendData(DEBUG_USART,(uint8_t)(HV_voltage/3.0)+37);
						break;
					case 18:
						USART_SendData(DEBUG_USART,temp_pump+37);
						break;	
					case 19:
						USART_SendData(DEBUG_USART,Bms_State+37);
						break;
					case 20:
						USART_SendData(DEBUG_USART,(Amk_actualVel_lf_1.bQuitInverterOn&&Amk_actualVel_rf_1.bQuitInverterOn)+37);
						break;
					case 21:
						USART_SendData(DEBUG_USART,oilPressure_bl+37);
						break;
					case 22:
						USART_SendData(DEBUG_USART,oilPressure_br+37);
						break;
					case 23:
						USART_SendData(DEBUG_USART,oilPressure_fl+37);
						break;
					case 24:
						USART_SendData(DEBUG_USART,oilPressure_fr+37);
						break;
					case 25:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_lb_2.Amk_TempMotor+37);
						break;
					case 26:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_rb_2.Amk_TempMotor+37);
						break;
					case 27:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_lf_2.Amk_TempMotor+37);
						break;
					case 28:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_rf_2.Amk_TempMotor+37);
						break;
					case 29:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_lb_2.Amk_TempInverter+37);
						break;
					case 30:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_rb_2.Amk_TempInverter+37);
						break;
					case 31:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_lf_2.Amk_TempInverter+37);
						break;
					case 32:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_rf_2.Amk_TempInverter+37);
						break;
					case 33:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_lb_2.Amk_TempIGBT+37);
						break;
					case 34:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_rb_2.Amk_TempIGBT+37);
						break;
					case 35:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_lf_2.Amk_TempIGBT+37);
						break;
					case 36:
						USART_SendData(DEBUG_USART,(uint8_t)Amk_actualVel_rf_2.Amk_TempIGBT+37);
						break;
				}
				tmp++;
				if(tmp == 74)
					tmp=0;
			}
			time1 = 0;
		}
		
		if(time0==30)
		{
			if(Motor_LOSE_Flag>30)
				Error_Flag=3;
			if(ROS_LOSE_Flag>40)
				Error_Flag=4;
			if(RES_LOSE_Flag>40)
				Error_Flag=5;
			if(EBS_LOSE_Flag>40)
				Error_Flag=6;
			if(ESC_LOSE_Flag>40)
				Error_Flag=7;
			if(STEERING_LOSE_Flag>40)
				Error_Flag=8;		
			//printf("Statu_Flag %d Beep_ros %d\n",Statu_Flag,Beep_ros);
			//第一次电机掉线检测失能
			motor_num=motor_num!=0?motor_num-1:0;
			
			//USART_SendData(DEBUG_USART,0);
			
//			printf("%d %d %d \n%d %d %d %d %f %d\n ----- \n",Amk_actualVel_lf_1.Amk_actualVelocity, Amk_actualVel_lf_1.Amk_bDcOn, Amk_actualVel_lf_1.Amk_bQuitDcOn, 
//								Amk_actualVel_lf_1.Amk_bSystemReady, Amk_actualVel_lf_1.bInverterOn, Amk_actualVel_lf_1.bQuitInverterOn, Brake_Flag,steering_angle,ACC2_Value);
			//printf("%d \n",(uint8_t)((steering_angle-10)/2+108)+40);
			time0=0;
		}

		//清楚中断标志位
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); 
		time0++;
		time1++;
	}

}

//----------------------------------------------CAN1接收中断----------------------------------------------/

void CAN1_RX_IRQHandler(void)
{
	CAN_RECEIVE_TOGGLE;
	CAN_Receive(CANx1, CAN_FIFO0, &CAN1_RxMessage);
	
	//接收电机控制器消息
	switch(CAN1_RxMessage.StdId)
	{
		//right front
		case 0x284:
			AMK_RF_V1 = CAN1_RxMessage;
			break;
		case 0x286:
			AMK_RF_V2 = CAN1_RxMessage;
			break;
		//left front
		case 0x283:
			AMK_LF_V1 = CAN1_RxMessage;
			break;
		case 0x285:
			AMK_LF_V2 = CAN1_RxMessage;
			break;
		//left behind
		case 0x287:
			AMK_LB_V1 = CAN1_RxMessage;
			break;
		case 0x289:
			AMK_LB_V2 = CAN1_RxMessage;
			break;
		//right behind
		case 0x288:
			AMK_RB_V1 = CAN1_RxMessage;
			break;
		case 0x290:
			AMK_RB_V2 = CAN1_RxMessage;
			break;
	}	
//	if(CAN1_RxMessage.StdId==0x180)
//		switch(CAN1_RxMessage.Data[0])
//		{
//			case 0x30://电机传回的速度消息
//				Motor_LOSE_Flag=0;
//				Speed_Get_Msg.Data[1]=CAN1_RxMessage.Data[1];
//				Speed_Get_Msg.Data[2]=CAN1_RxMessage.Data[2];
//				break;
//			case 0x00:
//				break;
//		}
	
	
	//接收工作站消息
	if(CAN1_RxMessage.ExtId==0x124)
	{
		ROS_LOSE_Flag=0;
		if(Statu_Flag)
		{
			//ASSI 状态
			if(RES_Flag&&E_Switch_Flag)
			{
				switch(CAN1_RxMessage.Data[0])
				{
					case 1:
						YellowLight;
						break;
					case 2:
						YellowFlash;	
						break;
					case 3:
						BlueLight;
						break;
					case 4:
						BlueFlash;
						if(Statu_Flag!=4)
							Beep_ros=1;
						break;
				}
				Statu_Flag=CAN1_RxMessage.Data[0];									//1准备、2行驶、3完成、4制动
				isFinshed=(Statu_Flag==3);
			}
//			Speed_Cntrol_Msg.Data[1]=CAN1_RxMessage.Data[1];		//速度整数部分
//			Speed_Cntrol_Msg.Data[2]=CAN1_RxMessage.Data[2];		//速度小数部分
			Steering_Tx_Message.Data[1]=CAN1_RxMessage.Data[3];	//转角高八位
			Steering_Tx_Message.Data[2]=CAN1_RxMessage.Data[4];	//转角低八位
			ESC_Flag=CAN1_RxMessage.Data[5];										//制动等级
			ROS_FastBrake=CAN1_RxMessage.Data[7];								//直线加速紧急制动标志位
		}
	}
	CAN1_RxMessage.ExtId=CAN1_RxMessage.StdId=0x00;
}

//----------------------------------------------CAN2接收中断----------------------------------------------/

void CAN2_RX_IRQHandler(void)
{
	CAN_Receive(CANx2, CAN_FIFO0, &CAN2_RxMessage);
	switch(CAN2_RxMessage.StdId)
	{
		case 0x564://仪表盘消息
			//接收比赛模式
			Match_Mode=CAN2_RxMessage.Data[0];
			break;
		case 0x11://RES消息
			RES_LOSE_Flag=0;
			if(Statu_Flag)
			{
				switch(CAN2_RxMessage.Data[0])
				{
					case 0xAB:	//制动
						if(RES_Flag!=0)
							Beep_ros=1;
						RES_Flag=0;
						Statu_Flag=4;
						Error_Flag=1;
						BlueFlash;
						//Stop_Flag=0;
						break;
					case 0xCC:	//准备
						RES_Flag=2;
						break;
					case 0x3F:	//行驶
						if(RES_Flag!=4)
							Beep_ros=0;
						RES_Flag=4;
						break;
					case 0x66:	//缓停
						RES_Flag=6;
						break;
				}
			}
			break;
		case 0x254://转向消息
			STEERING_LOSE_Flag=0;
			steering_angle = ((uint16_t)CAN2_RxMessage.Data[3]<<8 | CAN2_RxMessage.Data[4]) >2048? 
				((uint16_t)CAN2_RxMessage.Data[3]<<8 | CAN2_RxMessage.Data[4]-4096)/2048.0*180:((uint16_t)CAN2_RxMessage.Data[3]<<8 | CAN2_RxMessage.Data[4])/2048.0*180;
			break;
		case 0x407://ESC消息
			ESC_LOSE_Flag=0;
			if(CAN2_RxMessage.Data[0]!=ESC_Flag)
				;//DoSomething
			break;
		case 0x134://EBS消息
			EBS_LOSE_Flag=0;
			//气瓶气压不足或EBS未准备就绪
			r=CAN2_RxMessage.Data[3];
			if(CAN2_RxMessage.Data[1]==0&&Statu_Flag)
			{
//				Error_Flag=2;
//				EBS_Flag = 0;
			}
			break;
	}
	//接收BMS扩展帧信号
	if(CAN2_RxMessage.ExtId==0x186340F3)
	{
//		if((CAN2_RxMessage.Data[0]==16)&&isTS_ON==1&&Statu_Flag<3)
//		{
//			isTS_ON=0;
//			//高压断开、退出待驶模式
//			//ToDriveFlag=0;
//			//Beep_man=0;
//			//ToDrive_LED_OFF;
//			//SC(BREAK);
//		}else if(CAN2_RxMessage.Data[0]==81)
//		{
//			isTS_ON=1;
//		}
	}
	CAN2_RxMessage.ExtId=CAN2_RxMessage.StdId=0x00;
}

//----------------------------------------------进入待驶模式----------------------------------------------/

void RUN_ENABLE_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET) 
	{
		//激活驱动系统后进入待驶模式
		if(/*Brake_Flag&& Amk_actualVel_lf_1.bQuitInverterOn && */ADC_Flag)
		{
			ToDriveFlag=1;
			ToDrive_LED_ON;
		}else
		{
			ToDriveFlag=0;
			Beep_man=0;
			ToDrive_LED_OFF;
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

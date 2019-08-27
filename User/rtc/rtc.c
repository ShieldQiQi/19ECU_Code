/*
Date:2018-12-20
Author:SHIELD_QI
Project:RTC �������ɰ��ʱ�Լ� CAN ʱ���Ż�
*********************************************/

#include "stm32f4xx.h"
#include "rtc.h"
#include "can.h"

// ���������Լ�ʱ��
void RTC_TimeAndDate_Set(void)
{
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	
	// ��ʼ��ʱ��
	RTC_TimeStructure.RTC_H12 = RTC_H12_AMorPM;
	RTC_TimeStructure.RTC_Hours = HOURS;        
	RTC_TimeStructure.RTC_Minutes = MINUTES;      
	RTC_TimeStructure.RTC_Seconds = SECONDS;    
	RTC_SetTime(RTC_Format_BINorBCD, &RTC_TimeStructure);
	RTC_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);
	
  // ��ʼ������	
	RTC_DateStructure.RTC_WeekDay = WEEKDAY;       
	RTC_DateStructure.RTC_Date = DATE;         
	RTC_DateStructure.RTC_Month = MONTH;         
	RTC_DateStructure.RTC_Year = YEAR;        
	RTC_SetDate(RTC_Format_BINorBCD, &RTC_DateStructure);
	RTC_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);
}

// RTC���ã�ѡ��RTCʱ��Դ������RTC_CLK�ķ�Ƶϵ��
uint8_t RTC_CLK_Config(void)
{
	RTC_InitTypeDef RTC_InitStructure;
	
	//ʹ�� PWR ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  // PWR_CR:DBF��1��ʹ��RTC��RTC���ݼĴ����ͱ���SRAM�ķ���
  PWR_BackupAccessCmd(ENABLE);

  // ʹ��LSE
  RCC_LSEConfig(RCC_LSE_ON);
   // �ȴ�LSE�ȶ� 
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }
  // ѡ��LSE��ΪRTC��ʱ��Դ
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);    

  // ʹ��RTCʱ��
  RCC_RTCCLKCmd(ENABLE);

  // �ȴ� RTC APB �Ĵ���ͬ��
  RTC_WaitForSynchro();
   
	// ��ʼ��ͬ��/�첽Ԥ��Ƶ����ֵ
	// ����������ʱ��ck_spare = LSE/[(255+1)*(127+1)] = 1HZ 
	
	// �����첽Ԥ��Ƶ����ֵ
	RTC_InitStructure.RTC_AsynchPrediv = ASYNCHPREDIV;
	// ����ͬ��Ԥ��Ƶ����ֵ
	RTC_InitStructure.RTC_SynchPrediv = SYNCHPREDIV;	
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24; 
	// ��RTC_InitStructure�����ݳ�ʼ��RTC�Ĵ���
	if (RTC_Init(&RTC_InitStructure) == ERROR)
	{
		return 0;
	}
	
	//���ڳ�ʼ��
	RTC_TimeAndDate_Set();
	return 1;
}


/**********************************END OF FILE*************************************/

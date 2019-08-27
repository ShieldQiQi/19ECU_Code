/*
Date:2018-12-20
Author:SHIELD_QI
Project:RTC 用于数采板计时以及 CAN 时序优化
*********************************************/

#include "stm32f4xx.h"
#include "rtc.h"
#include "can.h"

// 设置日期以及时间
void RTC_TimeAndDate_Set(void)
{
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	
	// 初始化时间
	RTC_TimeStructure.RTC_H12 = RTC_H12_AMorPM;
	RTC_TimeStructure.RTC_Hours = HOURS;        
	RTC_TimeStructure.RTC_Minutes = MINUTES;      
	RTC_TimeStructure.RTC_Seconds = SECONDS;    
	RTC_SetTime(RTC_Format_BINorBCD, &RTC_TimeStructure);
	RTC_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);
	
  // 初始化日期	
	RTC_DateStructure.RTC_WeekDay = WEEKDAY;       
	RTC_DateStructure.RTC_Date = DATE;         
	RTC_DateStructure.RTC_Month = MONTH;         
	RTC_DateStructure.RTC_Year = YEAR;        
	RTC_SetDate(RTC_Format_BINorBCD, &RTC_DateStructure);
	RTC_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);
}

// RTC配置：选择RTC时钟源，设置RTC_CLK的分频系数
uint8_t RTC_CLK_Config(void)
{
	RTC_InitTypeDef RTC_InitStructure;
	
	//使能 PWR 时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  // PWR_CR:DBF置1，使能RTC、RTC备份寄存器和备份SRAM的访问
  PWR_BackupAccessCmd(ENABLE);

  // 使能LSE
  RCC_LSEConfig(RCC_LSE_ON);
   // 等待LSE稳定 
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }
  // 选择LSE做为RTC的时钟源
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);    

  // 使能RTC时钟
  RCC_RTCCLKCmd(ENABLE);

  // 等待 RTC APB 寄存器同步
  RTC_WaitForSynchro();
   
	// 初始化同步/异步预分频器的值
	// 驱动日历的时钟ck_spare = LSE/[(255+1)*(127+1)] = 1HZ 
	
	// 设置异步预分频器的值
	RTC_InitStructure.RTC_AsynchPrediv = ASYNCHPREDIV;
	// 设置同步预分频器的值
	RTC_InitStructure.RTC_SynchPrediv = SYNCHPREDIV;	
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24; 
	// 用RTC_InitStructure的内容初始化RTC寄存器
	if (RTC_Init(&RTC_InitStructure) == ERROR)
	{
		return 0;
	}
	
	//日期初始化
	RTC_TimeAndDate_Set();
	return 1;
}


/**********************************END OF FILE*************************************/

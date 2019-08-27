/*
Date:2018-12-20
Author:SHIELD_QI
Project:RTC �������ɰ��ʱ�Լ� CAN ʱ���Ż�
*********************************************/

#ifndef __RTC_H__
#define __RTC_H__

#include "stm32f4xx.h"


// ʱ��Դ�궨��
#define RTC_CLOCK_SOURCE_LSE      

// �첽��Ƶ����
#define ASYNCHPREDIV         0X7F
// ͬ����Ƶ����
#define SYNCHPREDIV          0XFF

// ʱ��궨��
#define RTC_H12_AMorPM			 RTC_H12_AM  
#define HOURS                9          // 0~23
#define MINUTES              16          // 0~59
#define SECONDS              0          // 0~59

// ���ں궨��
#define WEEKDAY              2         // 1~7
#define DATE                 25         // 1~31
#define MONTH                12         // 1~12
#define YEAR                 19         // 0~99

// ʱ���ʽ�궨��
#define RTC_Format_BINorBCD  RTC_Format_BIN

// ������Ĵ����궨��
#define RTC_BKP_DRX          RTC_BKP_DR0
// д�뵽���ݼĴ��������ݺ궨��
#define RTC_BKP_DATA         0X32F2
 
                                  
uint8_t RTC_CLK_Config(void);
void RTC_TimeAndDate_Set(void);

#endif // __RTC_H__

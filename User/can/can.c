/*
Author:SHIELD_QI
Date:2018-06-30
************************************/

#include "can.h"

//------------------------------------Defination of Msg Structure------------------------------------/


//---------------CAN2_无线急停、转向控制、工作站---------------/
/*接收结构体*/
CanRxMsg CAN2_RxMessage=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,3}
};
/*串口屏*/
CanRxMsg Screen_Rx_Message=
{
	0x7FF,0x7FF,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
CanTxMsg HRT_Ctrl_1=//0x156
{
	0x156,0x156,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*RES*/
CanRxMsg RES_Rx_Message=
{
	0x336,0x336,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*EBS*/
CanRxMsg EBS_Rx_Message=
{
	0x7FF,0x7FF,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
CanTxMsg EBS_Tx_Message=
{
	0x566,0x566,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0x02,0x00,0x00,1,0,0,0}
};
/*转向控制器*///data[6]==1编码器不正常为0正常
CanRxMsg Steering_Rx_Message=
{
	0x232,0x232,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
//转向使能
CanTxMsg Steering_Tx_Message0=
{
	0x439,0x439,CAN_Id_Standard,CAN_RTR_Data,8,
	{1,0x02,0x79,0x00,0,0,0,0}
};
//转向控制
CanTxMsg Steering_Tx_Message=
{
	0x439,0x439,CAN_Id_Standard,CAN_RTR_Data,8,
	{2,0x03,0x00,0x00,0,0,0,0}
};
/*ESC泵*/
CanRxMsg ESC_Rx_Message=
{
	0x7FF,0x7FF,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
CanTxMsg ESC_Tx_Message=
{
	0x221,0x221,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0x02,0x00,0x00,1,0,0,0}
};
//---------------CAN1_控制消息—发送结构体-----------------/
/*ROS发送*/
CanTxMsg ROS_TxMsg=
{
	0x439,0x439,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x51,0x04,0x00,0,0,0,0,0}
};
/*电机控制器*/
CanTxMsg AMK_SETPOINT_RF=
{
	0x185,0x185,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,7,0,0,0,0,0,0}
};
CanTxMsg AMK_SETPOINT_LF=
{
	0x184,0x184,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,7,0,0,0,0,0,0}
};
CanTxMsg AMK_SETPOINT_RB=
{
	0x189,0x189,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
CanTxMsg AMK_SETPOINT_LB=
{
	0x188,0x188,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*RTC实时时钟*/
CanTxMsg RTC_Msg=
{
	0x386,0x386,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x14,0x12,0x00,0,0,0,0,0}
};

//-----------------CAN1_电机状态读取—接收结构体-----------------/

/*AMK RF actual value1*/
CanRxMsg AMK_RF_V1=
{
	0x284,0x284,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*AMK RF actual value2*/
CanRxMsg AMK_RF_V2=
{
	0x286,0x286,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*AMK LF actual value1*/
CanRxMsg AMK_LF_V1=
{
	0x283,0x283,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*AMK LF actual value2*/
CanRxMsg AMK_LF_V2=
{
	0x285,0x285,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*AMK RB actual value1*/
CanRxMsg AMK_RB_V1=
{
	0x288,0x288,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*AMK RB actual value2*/
CanRxMsg AMK_RB_V2=
{
	0x290,0x290,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*AMK LB actual value1*/
CanRxMsg AMK_LB_V1=
{
	0x287,0x287,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*AMK LB actual value2*/
CanRxMsg AMK_LB_V2=
{
	0x289,0x289,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*接收结构体*/
CanRxMsg CAN1_RxMessage=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,5,6,0,0,3}
};
//------------------------------------CAN1 Configure------------------------------------/

void CAN1_Config(void)
{
	
	//------------------ GPIO初始化 ------------------/
 	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK|CAN1_RX_GPIO_CLK, ENABLE);
	
	//引脚复用
  GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
  GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT); 

  //Config CAN TX pins
  GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);
	
	//Configure CAN RX  pins
  GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);

  //接收中断配置
	NVIC_InitTypeDef 			NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	//------------------ CAN1初始化 ------------------/
	CAN_InitTypeDef        CAN_InitStructure;
  RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);

	//CAN寄存器初始化
	CAN_DeInit(CANx1);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   		//MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			  	  //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   		//MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			  	  //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;				    //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			  	  //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		    //BTR-SJW 重新同步跳跃宽度 2个时间单元
	 
	// ss=1 bs1=7 bs2=6 位时间宽度为(1+7+6) 波特率即为时钟周期tq*(1+7+6)
	
	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;		    //BTR-TS1 时间段1 占用了7个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;		    //BTR-TS1 时间段2 占用了6个时间单元	
	
	// CAN Baudrate = 500 kBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) 
	
	CAN_InitStructure.CAN_Prescaler =6;		        //BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+7+6)/6=500 kbps
	CAN_Init(CANx1, &CAN_InitStructure);

  //---------------- 配置CAN1筛选器 ----------------/
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	CAN_FilterInitStructure.CAN_FilterNumber=1;												  //筛选器组1
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;				//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;			//筛选器位宽为单个32位。
  
	CAN_FilterInitStructure.CAN_FilterIdHigh= 													//要筛选的ID高位 			
	((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		
	CAN_FilterInitStructure.CAN_FilterIdLow= 														//要筛选的ID低位 
	(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; 
	
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0;										//筛选器高16位每位无须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0;											//筛选器低16位每位无须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;	//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;								//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CANx1, CAN_IT_FMP0, ENABLE);														//CAN通信中断使能
	
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	
}

//------------------------------------CAN2 Configure------------------------------------/

void CAN2_Config(void)
{
	//------------------ GPIO初始化 ------------------/
 	GPIO_InitTypeDef			 GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK|CAN2_RX_GPIO_CLK, ENABLE);
	
	//引脚复用
  GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
  GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT); 

	//Config CAN TX pins
  GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
	
	//Config CAN RX pins
  GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);


	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/*中断设置*/
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX_IRQ;	   				//CAN RX0中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	  //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			    //子优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);

	/*CAN寄存器初始化*/
	CAN_DeInit(CANx2);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		 //BTR-SJW 重新同步跳跃宽度 2个时间单元
	 
	/* ss=1 bs1=7 bs2=6 位时间宽度为(1+7+6) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;		 //BTR-TS1 时间段1 占用了4个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;		 //BTR-TS1 时间段2 占用了2个时间单元	
	
	/* CAN Baudrate = 500k Bps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =6;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+4+2)/6=1 Mbps
	CAN_Init(CANx2, &CAN_InitStructure);


	CAN_FilterInitTypeDef  CAN_FilterInitStructure;


	CAN_FilterInitStructure.CAN_FilterNumber=14;																//筛选器组14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;								//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;							//筛选器位宽为单个32位。

	CAN_FilterInitStructure.CAN_FilterIdHigh=
	((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;								//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow=
	(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;												  //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0;														//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0;															//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;					//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;												//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/*CAN通信中断使能*/	
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	
	CAN_ITConfig(CANx2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CANx2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CANx2, CAN_IT_FMP0, ENABLE);
}



void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t ubCounter = 0;

	/*把接收结构体清零*/
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}

void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
	uint8_t ubCounter = 0;

  //TxMessage.StdId=0x00;						 
  TxMessage->ExtId=0x1314;						 //使用的扩展ID
  TxMessage->IDE=CAN_ID_EXT;					 //扩展模式
  TxMessage->RTR=CAN_RTR_DATA;				 //发送的是数据
  TxMessage->DLC=8;										 //数据长度为8字节
	
	/*设置要发送的数据0-7*/
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    TxMessage->Data[ubCounter] = ubCounter;
  }
}

/************************END OF FILE***********************/




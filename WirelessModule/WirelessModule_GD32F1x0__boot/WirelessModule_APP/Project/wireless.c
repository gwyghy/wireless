/**
  ******************************************************************************
  * File Name          : WIRELESS.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rfid_driver.h"
#include "usartapp.h"
/********************************************************************************
* #define宏定义
*********************************************************************************/

/*****************************************************************
** 函数名：Wl_RxDataProc
** 输　入：None
** 输　出：None
** 功能描述：无线接收数据处理。以下函数应该在任务中周期执行。
******************************************************************/
void Wl_RxDataProc(void)
{
	uint32_t	i,j;
	uint8_t buf[CC1101_TX_FIFO_SIZE];
	static uint32_t RFID_StateInquireIntv = 0;
	static uint16_t RFID_Sleep = 0;
	static uint32_t RFID_Init_Num = 0;
	stUARTDATA stSendFrame;	
	// 读取无线接收数据
	i = WL_ReceiveData((unsigned char *)buf, CC1101_TX_FIFO_SIZE);
	if (!i)
	{
		if((RFID_Sleep != 0xFFFF) && (RFID_Sleep != 0))
			RFID_Sleep--;
		if(RFID_Sleep == 0)
		{
			RFID_Init();
			RfidStrobe(CC1101_SIDLE);
			RFID_Sleep = 250*4;			
		}
		RFID_StateInquireIntv++;				//周期检查无线是否处于正确状态
		if (RFID_StateInquireIntv >= 5)			//100ms =5*WL_RX_TASK_INTERVAL
		{
			RFID_StateInquireIntv = 0;
			//获取RDID状态
			j = GetRfidCurStatus();
			switch(j&0xf0)
			{
				case CC1101_STATE_IDLE:
					SetRfidSRX();				//设置RFID进入接收状态
					RFID_Init_Num = 0;
				break;
				case CC1101_STATE_RX:
				case CC1101_STATE_TX:
				case CC1101_STATE_FSTXON:
				case CC1101_STATE_CALIBRATE:
				case CC1101_STATE_SETTLING:
					RFID_Init_Num = 0;
				break;
				case CC1101_STATE_RX_OVERFLOW:
					RfidStrobe(CC1101_SFRX);	//冲洗 RX FIFO buffer.
					RfidStrobe(CC1101_SRX);		//进入接收状态	
					RFID_Init_Num = 0;
				break;
				case CC1101_STATE_TX_UNDERFLOW:
					RfidStrobe(CC1101_SFTX);    //冲洗 TX FIFO buffer.
					RfidStrobe(CC1101_SRX);		//进入接收状态	
					RFID_Init_Num = 0;
				break;
				default:
					if (j == TIME_OVER && RFID_Init_Num < 5)	//重新初始化RFID
					{
						RFID_Init_Num++;
						RFID_Init();
					}
				break;			
			}
		}
		return;
	}
	else
	{
		RFID_Sleep = 50;
		RFID_StateInquireIntv = 0;
		RFID_Init_Num = 0;
	}	
	if (i < 5)
		return;
	memset(&stSendFrame,0x00,sizeof(stUARTDATA));
	memcpy(&(stSendFrame.SendBuf), &(buf), i);
	if (i < stSendFrame.SendBuf[4]+5)
		return;
	SendWirelesstData(stSendFrame, i);
	
}

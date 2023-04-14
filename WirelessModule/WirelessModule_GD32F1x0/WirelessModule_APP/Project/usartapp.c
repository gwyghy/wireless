#include "usartapp.h"
#include "rfid_driver.h"
#include "iapupdate.h"

#define STFRAME_LS_MAX						255//流水号的最大值
#define WL_INFO_LENGTH		128		//经无线传输的扩展CAN数据字节数
#define  SENDTIMES   3
uint8_t Version[]="V3.0.0";

uint8_t SC_RFID_ADDR = 0;
stUARTWIRELESS  USARTWIRLELESSTxBuf[USARTWIRELESS_TX_BUF_MAX];						//发送缓存
uint8_t	 USARTWIRELESSTxWrtPtr = 0;			   						    //发送写指针
uint8_t	 USARTWIRELESSTxRdPtr =  0;	

uint32_t u32RecvBaseAddr,u32RecvOffsetAddr;//基地址,偏移地址
uint32_t u32RecvTotalPacketNumb;//总包数
uint32_t u32NowRecvPacketNumbVERSION;//当前版本信息包数
uint32_t u32NowRecvPacketNumbCODE;//当前程序包数
// u8 	u8NowRecvPacketNumbTimes;//当前包数接收的次数
uint32_t u32RecvProgLength;
uint32_t u32DevType;//u32DevType在传输程序时为程序类型
uint32_t u32TargetType;//u32TargetType在传输程序时为目标板类型
uint32_t u32timeover = 0;


void UpdataInit(void)
{
	u32RecvOffsetAddr = 0;
	u32RecvBaseAddr = 0;
	u32RecvTotalPacketNumb = 0;
	u32NowRecvPacketNumbVERSION = 0;
	u32NowRecvPacketNumbCODE = 0;
	u32RecvProgLength = 0;
	u32DevType = 0;
	u32TargetType =0;
}
void timeoverset(uint32_t value)
{
	u32timeover = value;
}
void TimeFun(void)
{
	if((u32timeover != 0xFFFFFFFF) && (u32timeover != 0x00))
	{
		u32timeover--;
		if(u32timeover == 0)
		{
//			OSTaskResume(REOMNT_TASK_PRIO);
			timeoverset(0xFFFFFFFF);
			UpdataInit();
//			ReUpdata();
		}
	}
}


/********************************************************************************
* 功能描述： 插入usart发送队列
* 入口参数：
* 返回值：无
********************************************************************************/
static uint8_t InsUsartTrsQueue(stUARTDATA *pFrame)
{
	static uint8_t u8LiuShuiNumb = 0x00;
	static uint8_t u8TxBufTxWrtPtr = 0x00;
	uint16_t u16CrcTemp = 0x00;
	uint8_t u8ReturnFlag = 0x01;

	/**防止队列满的情况出现**/
	u8TxBufTxWrtPtr = USARTWIRELESSTxWrtPtr;
	u8TxBufTxWrtPtr++;
	u8TxBufTxWrtPtr %= USARTWIRELESS_TX_BUF_MAX;		
	if((u8TxBufTxWrtPtr != USARTWIRELESSTxRdPtr))
	{
		u8LiuShuiNumb++;
		u8LiuShuiNumb %= STFRAME_LS_MAX;
		memset(&(USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr].stFrame), 0x00  , sizeof(stUARTDATA));
		memcpy(&(USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr].stFrame), pFrame, sizeof(stUARTDATA));
		USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr].Address = WIRELESSDEVIC;
		USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr].Serial = u8LiuShuiNumb;		
 		Crc16UsartReceiveChick((uint8_t*)&(USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr]),USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr].stFrame.u8DLC+5,&u16CrcTemp);
		USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr].stFrame.CRCHigh = (uint8_t)((u16CrcTemp&0xFF00)>>8);
		USARTWIRLELESSTxBuf[USARTWIRELESSTxWrtPtr].stFrame.CRCLow = (uint8_t)(u16CrcTemp&0x00FF);
		USARTWIRELESSTxWrtPtr ++;
		USARTWIRELESSTxWrtPtr %= USARTWIRELESS_TX_BUF_MAX;
	}
	else
	{
		u8ReturnFlag =  FALSE;//缓冲区已满，不能再加入队列中
	}	
	
	return u8ReturnFlag;
}
/***************************************************************************************
** 功能描述：发送应答数据
** 函数输入：无
** 函数输出：None
** 注意事项：
******************************************************************************************/
void SendResponseData(stUARTDATA pFrame, uint8_t Data)
{
	stUARTDATA stSendFrame;
	stSendFrame = pFrame;
	stSendFrame.ACK = 0x00;
	stSendFrame.u8DLC = 0x01;
	stSendFrame.SendBuf[0] = Data;
	InsUsartTrsQueue(&stSendFrame);
}
/********************************************************************************
* 功能描述： 发送心跳信息
* 入口参数：
* 返回值：无
********************************************************************************/
void SendHeartBeatData(void)
{
	uint8_t i;
	stUARTDATA stSendFrame;
	memset(&stSendFrame,0x00,sizeof(stUARTDATA));
	stSendFrame.FrameType = HEARTBEAT;
	stSendFrame.ACK = NOACK;
	stSendFrame.u8DLC = 8;
	for(i = 0x30 ; i<0x3A; i++)
	{
		if(Version[1] == i)
		{
			Version[1] = (i-0x30);
		}
		if(Version[3] == i)
		{
			Version[3] = (i-0x30);
		}	
		if(Version[5] == i)
		{
			Version[5] = (i-0x30);
		}			
	}
	stSendFrame.SendBuf[4] = (uint8_t)(Version[0]);
	stSendFrame.SendBuf[5] = (uint8_t)(Version[1]);
	stSendFrame.SendBuf[6] = (uint8_t)(Version[3]);
	stSendFrame.SendBuf[7] = (uint8_t)(Version[5]);
	
	
	InsUsartTrsQueue(&stSendFrame);
}
/********************************************************************************
* 功能描述： 发送上电信息
* 入口参数：
* 返回值：无
********************************************************************************/
void ReUpdata(void)
{
//	uint8_t i;
	stUARTDATA stSendFrame;
	memset(&stSendFrame,0x00,sizeof(stUARTDATA));
	stSendFrame.FrameType = REUPDATAPROGRAMWL;
	stSendFrame.ACK = ACK;
	stSendFrame.u8DLC = 8;
	InsUsartTrsQueue(&stSendFrame);
}
/********************************************************************************
* 功能描述： 发送上电信息
* 入口参数：
* 返回值：无
********************************************************************************/
void SendStartData(void)
{
//	uint8_t i;
	stUARTDATA stSendFrame;
	memset(&stSendFrame,0x00,sizeof(stUARTDATA));
	stSendFrame.FrameType = STARTHINT;
	stSendFrame.ACK = ACK;
	stSendFrame.u8DLC = 8;
	InsUsartTrsQueue(&stSendFrame);
}
/********************************************************************************
* 功能描述： 发送无线数据
* 入口参数：
* 返回值：无
********************************************************************************/
void SendWirelesstData(stUARTDATA data,uint8_t Length)
{
//	uint8_t i;
	stUARTDATA stSendFrame;
	memset(&stSendFrame,0x00,sizeof(stUARTDATA));
	memcpy(&(stSendFrame.SendBuf), &(data.SendBuf), Length);
	
	stSendFrame.FrameType = WIRELESSDATA;
	stSendFrame.ACK = NOACK;
	stSendFrame.u8DLC = Length;
	InsUsartTrsQueue(&stSendFrame);
}
/********************************************************************************
* 功能描述： 无线数据
* 入口参数：
* 返回值：无
********************************************************************************/
void GetWirelesstData(stUARTDATA data)
{
	uint32_t To;
	uint8_t buf[WL_INFO_LENGTH];
//	stUARTDATA stSendFrame;
	memset(buf,0x00,WL_INFO_LENGTH);
	memcpy(buf, &(data.SendBuf[1]), data.u8DLC-1);
	To = data.SendBuf[0];
	WL_SendData((unsigned char *)buf, (unsigned int)buf[4]+5, To);
}
/********************************************************************************
* 功能描述： 解析信息
* 入口参数：
* 返回值：无
********************************************************************************/
void GetSetdata(stUARTDATA data)
{
	uint8_t SC_RFID_SYNC0 = 0;
	uint8_t SC_RFID_SYNC1 = 0;
	stUARTDATA stSendFrame;
	memset(&stSendFrame,0x00,sizeof(stUARTDATA));
	memcpy(&(stSendFrame.SendBuf), &(data.SendBuf), sizeof(stUARTDATA));
	SC_RFID_ADDR = stSendFrame.SendBuf[0];
	RfidWriteReg(CC1101_ADDR, SC_RFID_ADDR);
	SC_RFID_SYNC0 = stSendFrame.SendBuf[1];
	RfidWriteReg(CC1101_SYNC0, SC_RFID_SYNC0); // SYNC0
	SC_RFID_SYNC1 = stSendFrame.SendBuf[2];
	RfidWriteReg(CC1101_SYNC1, SC_RFID_SYNC1); // SYNC
	

}
/***************************************************************************************
** 功能描述：功能设置
** 函数输入：无
** 函数输出：None
** 注意事项：
******************************************************************************************/
void FunctionSet(uint16_t data)
{
	if (data & WIRELESSRESET)
		NVIC_SystemReset();
		vIapJumpToBoot(IN_FLASH_BOOTLOADER_ADDR);
}
/***************************************************************************************
** 功能描述：解析设置信息
** 函数输入：无
** 函数输出：None
** 注意事项：
******************************************************************************************/
void GetSetInformationData(stUARTDATA pFrame)
{
	uint16_t temp;	
	temp = pFrame.SendBuf[0];
	temp |= pFrame.SendBuf[1] << 8;
	temp |= pFrame.SendBuf[2] << 16;
	temp |= pFrame.SendBuf[3] << 24;
	FunctionSet(temp);
}
/***************************************************************************************
** 功能描述：解析升级程序
** 函数输入：无
** 函数输出：None
** 注意事项：
******************************************************************************************/
void UsartRecvProgProc(stUARTDATA pFrame)
{
	const uint8_t MULTIPLE = 8;
	static uint16_t u16CrcCalculate = 0x00;
	static uint16_t u16Crc = 0x00;
	uint32_t u32Addr = 0x00;
	uint32_t u32Temp;
//	OS_TCB  pdata;
	uint64_t zeor = 0;
	timeoverset(1250);
	//接收到第一针数据
	if(!u32NowRecvPacketNumbVERSION)
	{
		//计算程序类型
		u32DevType = pFrame.SendBuf[0];
		u32DevType |= pFrame.SendBuf[1]<<1*MULTIPLE;
		u32DevType |= pFrame.SendBuf[2]<<2*MULTIPLE;
		u32DevType |= pFrame.SendBuf[3]<<3*MULTIPLE;
		
		
		u32TargetType = pFrame.SendBuf[4];
		u32TargetType|= pFrame.SendBuf[5]<<1*MULTIPLE;
		u32TargetType|= pFrame.SendBuf[6]<<2*MULTIPLE;
		u32TargetType|= pFrame.SendBuf[7]<<3*MULTIPLE;
		
		if(u32DevType != THE_DEV_TYPE || u32TargetType != TAGET_GD32F1_MCU)
			return;
		
		//计算程序大小
		u32RecvProgLength =  pFrame.SendBuf[12];
		u32RecvProgLength |=  pFrame.SendBuf[13]<<1*MULTIPLE;
		u32RecvProgLength |=  pFrame.SendBuf[14]<<2*MULTIPLE;
		u32RecvProgLength |=  pFrame.SendBuf[15]<<3*MULTIPLE;
		
		if((u8IapGetPrgStorageAddr(u32DevType,&u32RecvBaseAddr)  == 0) || (u8IapGetPrgSize(u32DevType,&u32RecvOffsetAddr) == 0))
		{
			u32DevType = 0x00;
			return ;
		}
		__disable_fault_irq();
//		FLASH_SetLatency(FLASH_LATENCY_2);
		fmc_unlock();
		/**擦除此部分的所有块**/				
		u8IapEraserSector(u32RecvBaseAddr,(u32RecvBaseAddr+u32RecvOffsetAddr));//擦除块	
		__enable_fault_irq();	
	}
	if((u32NowRecvPacketNumbVERSION < 0x02))
	{
		/*判断写入地址是否在程序存储范围内*/
//		__disable_fault_irq();
//		FLASH_SetLatency(FLASH_LATENCY_2);
//		HAL_FLASH_Unlock();		
	
		u32Addr = u32RecvBaseAddr+(u32NowRecvPacketNumbVERSION)*0x80;			
		if((u32Addr >= THE_DEV_PRG_STORAGE_BASEADDR) && (u32Addr < MY_DEV_TYPE_ADDRESS))
			u8IapWriteBuf(&(pFrame.SendBuf[0]), u32Addr,(uint8_t)sizeof(pFrame.SendBuf)-1);//数据		
//		__enable_fault_irq();						
	}
	if(u32NowRecvPacketNumbVERSION <= 0x02)
	{
		if(u32NowRecvPacketNumbVERSION == 0x02)
		{
			u32NowRecvPacketNumbCODE = 1;//
			if( u32RecvProgLength%0x80)
				u32Temp = u32RecvProgLength/0x80+0x01;
			else
				u32Temp = u32RecvProgLength/0x80;
			u32RecvTotalPacketNumb = u32Temp;	
		}
		u32NowRecvPacketNumbVERSION++;
	}
	if(u32NowRecvPacketNumbCODE >= 0x01)// 开始传输代码
	{
		if(u32NowRecvPacketNumbCODE && u32RecvTotalPacketNumb && (u32NowRecvPacketNumbCODE < u32RecvTotalPacketNumb))
		{
			if(u32NowRecvPacketNumbCODE !=  pFrame.SendBuf[128])
			{
				123;
			}
			u32Addr = (u32NowRecvPacketNumbCODE-0x01)*0x80;
			u32Addr += u32RecvBaseAddr+PROG_CODE_OFFSET_ADDRESS;
			if((u32Addr >= THE_DEV_PRG_STORAGE_BASEADDR) && (u32Addr < MY_DEV_TYPE_ADDRESS))
			{
//				__disable_fault_irq();
//				FLASH_SetLatency(FLASH_LATENCY_2);
//				HAL_FLASH_Unlock();		
				u8IapWriteBuf(&(pFrame.SendBuf[0]), u32Addr, (uint8_t)sizeof(pFrame.SendBuf)-1);//程序
//				__enable_fault_irq();		
			}
			u32NowRecvPacketNumbCODE++;
		}
		else//最后一帧
		{
			/*排除非正常进入的条件*/					
 			if(!(u32NowRecvPacketNumbCODE&& u32RecvTotalPacketNumb && (u32NowRecvPacketNumbCODE == u32RecvTotalPacketNumb)))
			{			
				return;
			}
			u32Addr = (u32NowRecvPacketNumbCODE-0x01)*0x80;
			u32Addr += u32RecvBaseAddr+PROG_CODE_OFFSET_ADDRESS;
			if((u32Addr >= THE_DEV_PRG_STORAGE_BASEADDR) && (u32Addr < MY_DEV_TYPE_ADDRESS))
			{
//				__disable_fault_irq();
//				FLASH_SetLatency(FLASH_LATENCY_2);
//				HAL_FLASH_Unlock();		
				
				u8IapWriteBuf(&(pFrame.SendBuf[0]), u32Addr, (uint8_t)sizeof(pFrame.SendBuf)-1);//程序

//				__enable_fault_irq();		
			}
			delay_1ms(10);
			u16CrcCalculate = 0x00;
			u32Temp = 0x00;
			for(u32Addr = u32RecvBaseAddr+PROG_CODE_OFFSET_ADDRESS; u32Addr < u32RecvBaseAddr+PROG_CODE_OFFSET_ADDRESS+u32RecvProgLength;u32Addr+= 0x20000)
			{
				if((u32RecvBaseAddr+PROG_CODE_OFFSET_ADDRESS+u32RecvProgLength-u32Addr) >= 0x20000)
				{
					u16CrcCalculate = u16IapExFlashCrc(u32Addr,0x20000,&u16CrcCalculate);//临时屏蔽	
					u32Temp++;
				}
				else
					u16CrcCalculate = u16IapExFlashCrc(u32Addr,(u32RecvProgLength - u32Temp*0x2000),&u16CrcCalculate);//临时屏蔽
					WDGT_Feed();
//					OSTimeDly(30/TICK_TIME);//延时1500ms，以便调度看门狗任务

			}
			u16CrcCalculate = 0x00;
			u16CrcCalculate = u16IapExFlashCrc(u32RecvBaseAddr+PROG_CODE_OFFSET_ADDRESS ,u32RecvProgLength,&u16CrcCalculate);

			u16Crc = 0x00;
			__disable_fault_irq();
			u8IapReadBuf((uint8_t *)&u16Crc,u32RecvBaseAddr+PROG_CRCL_OFFSET_ADDRESS,0x02);
			__enable_fault_irq();	
			if((u16CrcCalculate == u16Crc))//校验成功
			{
				u16CrcCalculate = 0x00;
				u16CrcCalculate = u16IapExFlashCrc(u32RecvBaseAddr+PROG_DEVTYPE_OFFSET_ADDRESS,DEV_PROGRAM_VERSION_SIZE-0x04,&u16CrcCalculate);//计算CRC校验
				u32Temp = (uint32_t)u16CrcCalculate;
				/*判断写入地址是否在程序存储范围内*/
				u32Addr = u32RecvBaseAddr+PROG_VER_CRCL_OFFSET_ADDRESS;			
				if((u32Addr >= THE_DEV_PRG_STORAGE_BASEADDR) && (u32Addr < MY_DEV_TYPE_ADDRESS) )	
				{		
// 					__disable_fault_irq();
// 					FLASH_SetLatency(THE_DEV_FLASH_Latency);
// 					FLASH_Unlock();	
// 					u8IapWriteBuf((u8 *)&u32Temp, u32RecvBaseAddr+PROG_VER_CRCL_OFFSET_ADDRESS,0x04);
// 					FLASH_Lock();	
// 					__enable_fault_irq();	
				}
				

				if(u32DevType ==THE_DEV_TYPE)//设备类型相符，写入相关标识，立即更新
				{
					__disable_fault_irq();
//					FLASH_SetLatency(FLASH_LATENCY_2);
					fmc_unlock();
					
					//写入设备类型字
					u8IapEraserSector(MY_DEV_TYPE_ADDRESS,(MY_DEV_TYPE_ADDRESS+0xFFF));//擦除一个扇区
					u8IapWriteBuf((uint8_t *)&u32DevType, MY_DEV_TYPE_ADDRESS, 0x08);
					/* 程序启动首地址*/
					
					u8IapReadBuf((uint8_t *)&u32Temp,  u32RecvBaseAddr+PROG_WRITE_BASE_ADDRESS, 0x08);
					
					u8IapWriteBuf((uint8_t *)&u32Temp, APP_WRITE_BASEADDRESS, 0x08);//写入启动首地址	

					u32Temp = RROG_DOWNLOAD_FLAG;//写入下载完成标志
					/*判断写入地址是否在程序存储范围内*/
					u32Addr = APP_DOWNLOAD_OFFSET_ADDRESS;			
					if((u32Addr >= MY_DEV_TYPE_ADDRESS) && (u32Addr < (APP_NOT_DWNL_UPDATE_ADDRESS+0x08)))					
						u8IapWriteBuf((uint8_t *)&u32Temp, u32Addr,0x08);

					u32Temp = RROG_UPDATE_FLAG;//写入立即更新标志
					/*判断写入地址是否在程序存储范围内*/
					u32Addr = APP_UPDATE_OFFSET_ADDRESS;			
					if((u32Addr >= MY_DEV_TYPE_ADDRESS) && (u32Addr < (APP_NOT_DWNL_UPDATE_ADDRESS+0x08)))						
						u8IapWriteBuf((uint8_t *)&u32Temp, u32Addr,0x08);
					zeor = 0x00;
						u8IapWriteBuf((uint8_t *)&zeor, APP_PRGUPDATE_SUCCEED_ADDRESS,0x08);
					fmc_lock();
					__enable_fault_irq();
					

					delay_1ms(100);

					NVIC_SystemReset();
					//执行跳转
//					vIapJumpToBoot(IN_FLASH_BOOTLOADER_ADDR);
				}
			}
			else //校验不正确
			{
				__disable_fault_irq();
				fmc_lock();
				__enable_fault_irq();
				UpdataInit();
//				OSTaskQuery(REOMNT_TASK_PRIO, &pdata);			//查询处理任务是否挂起
//				if(pdata.OSTCBStat == OS_STAT_SUSPEND)
//				{
//					OSTaskResume(REOMNT_TASK_PRIO);				//唤醒任务
//				}
				timeoverset(0xFFFFFFFF);
//				ReUpdata();
			}
		}
	}
}
extern uint16_t time;
/********************************************************************************
** 功能描述：发送串口数据
** 输　入：  无
** 输　出：  无
*********************************************************************************/
uint8_t SendUsartDataProc(void)
{
	uint8_t u8ReturnFlag = 0x01;
	if((USARTWIRELESSTxWrtPtr != USARTWIRELESSTxRdPtr))
	{
		time = 0;
		memcpy(&USARTWIRLELESSTxBuf[USARTWIRELESSTxRdPtr].stFrame.SendBuf[USARTWIRLELESSTxBuf[USARTWIRELESSTxRdPtr].stFrame.u8DLC],&USARTWIRLELESSTxBuf[USARTWIRELESSTxRdPtr].stFrame.CRCHigh,2);
		usart_dma_send((uint8_t *)&USARTWIRLELESSTxBuf[USARTWIRELESSTxRdPtr],USARTWIRLELESSTxBuf[USARTWIRELESSTxRdPtr].stFrame.u8DLC+7);
		USARTWIRELESSTxRdPtr ++;
		USARTWIRELESSTxRdPtr %= USARTWIRELESS_TX_BUF_MAX;	
	}
	else
	{
		u8ReturnFlag =  0x00;//缓冲区已满，不能再加入队列中
	}	
	return u8ReturnFlag;
}
/********************************************************************************
* 功能描述： 接收数据处理
* 入口参数：
* 返回值：无
********************************************************************************/
uint8_t RecvUsartDataProc(void)
{
	stUARTWIRELESS  UsartRcvFrm;
	stUARTDATA stSendFrame;
	static uint8_t u8DestScNumbBackup = 1;
	if(UsartFetchData(&UsartRcvFrm) == FALSE)
		return 0x00;	
	if(u8DestScNumbBackup == UsartRcvFrm.Serial)
		return 0x00;	
	u8DestScNumbBackup = UsartRcvFrm.Serial;
	memset(&(stSendFrame), 0x00  , sizeof(stUARTDATA));
 	memcpy(&(stSendFrame), &(UsartRcvFrm.stFrame), sizeof(stUARTDATA));
	switch(stSendFrame.FrameType)
	{
		case WIRELESSDATA:
			 GetWirelesstData(stSendFrame);
		break;
		case PARAMETERSETTING:
			GetSetdata(stSendFrame);	
		break;
		case UPDATAPROGRAM:		
			UsartRecvProgProc(stSendFrame);
		break;
		case RESTORATIONWL:
 			GetSetInformationData(stSendFrame);
		break;
		default:
		break;
	}
	return 1;
}


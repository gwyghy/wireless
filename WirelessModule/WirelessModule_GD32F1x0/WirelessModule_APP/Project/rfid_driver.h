/*********************************************************************************************************************************
** 文件名:  rfid_driver.h
** 描　述:  RFID模块对外提供的接口
** 创建人: 	沈万江
** 日　期:  2014-12-26
** 修改人:	
** 日　期:	
**
** 版　本:	V1.0.0.0
** 更新记录:
** 更新记录	：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
**--------------------------------------------------------------------------
**************************Copyright (c) 1998-1999 天津华宁电子技术有限公司技术开发部*************************************************/
#ifndef __RFID_DRIVER_H__
#define __RFID_DRIVER_H__

#include "rfid_board.h"
#include "rfid_config.h"
#include "rfid_cc1101.h"

#define SS_RFID_ADDR	255						// 支架服务器RFID的地址
#define ALL_RFID_ADDR	0						// RFID群发地址

typedef void (*RCVED_BACK_CALL_FUNC)(void);		// 接收数据后，回调函数类型

/*****************************************************************
** 函数名：RFID_Init
** 输　入：
** 输　出：
** 功能描述：完成MCU驱动CC1101的硬件初始化以及CC1101的配置工作
******************************************************************/
void RFID_Init(void);
/*****************************************************************
** 函数名：SetRcvedBackCallFunc
** 输　入：BackCallFunc：数据接收后的回调函数
** 输　出：无
** 功能描述：设置RFID数据接收后的回调函数
******************************************************************/
void SetRcvedBackCallFunc(RCVED_BACK_CALL_FUNC BackCallFunc);
/*****************************************************************
** 函数名：WL_ReceiveData
** 输　入：buf：数据接收缓冲区
**         num：接收字节数，不符合此值放弃
** 输　出：实际读出数据字节数
** 功能描述：读取接收的无线数据
******************************************************************/
uint32_t WL_ReceiveData(unsigned char *buf, unsigned int num);
/*****************************************************************
** 函数名：WL_SendData
** 输　入：buf：发送数据缓冲区
**         num：发送数据字节数
**         ToAddr：接收地址
** 输　出：实际发送字节数
** 功能描述：无线发射数据
******************************************************************/
uint32_t WL_SendData(unsigned char *buf, unsigned int num, unsigned int ToAddr);
/*****************************************************************
** 功能：检查是否允许无线发射
** 输入：无
** 返回：TRUE:可以发送
******************************************************************/
uint32_t WlEmitEnabled(void);
/*****************************************************************
** 函数名：RFID_FetchData
** 输　入：*RfidRcvFrm，取出数据保存的位置
** 输　出：返回RET_OK，成功；RET_ERR,没有取到数据；
** 功能描述：读取接收的无线数据
******************************************************************/
uint32_t RFID_FetchData(st_RFIDRcvFrame *RfidRcvFrm);
/*****************************************************************
** 函数名：		RFID_SendData
** 输　入：		pu8Buf：发送数据缓冲区
**         		u32Length：发送数据字节数
** 输　出：		返回值：TX_OK，发送完成；
**						其余的返回状态可以参考rfid_config.h文件中的错误状态定义
**			
** 功能描述：	无线发射数据
******************************************************************/
uint32_t RFID_SendData(uint8_t u8DestAddr,uint8_t *pu8Buf, uint32_t *u32Length);

/*****************************************************************
** 功能：设置CC1101进入空闲状态
** 输入：无
** 返回：RET_OK:设置成功；其余的返回状态可以参考rfid_config.h文件中的错误状态定义
******************************************************************/
uint32_t SetRfidSIDLE(void);
/*****************************************************************
** 功能：设置CC1101进入接收状态
** 输入：无
** 返回：RET_OK:设置成功；其余的返回状态可以参考rfid_config.h文件中的错误状态定义
******************************************************************/
uint32_t SetRfidSRX(void);
/*****************************************************************
** 功能：获取CC1101是否允许发送
** 输入：无
** 返回：RET_OK = 0:允许发送；RET_ERR = 1：不允许发送
******************************************************************/
uint32_t GetRfidSendEnable(void);
/*****************************************************************
** 功能：	获取CC1101当前的TX FIFO状态
** 输入：	无
** 返回：	Bit7 & CC1101_STATUS_CHIP_RDYn_BM = 0说明CC1101已经装备好；
**			Bit6-4 & CC1101_STATUS_STATE_BM = CC1101_STATE_IDLE：		空闲状态；
**											= CC1101_STATE_RX：			接收状态
**											= CC1101_STATE_TX：			发送状态
**											= CC1101_STATE_FSTXON：		快速TX开启
**											= CC1101_STATE_CALIBRATE
**											= CC1101_STATE_SETTLING
**											= CC1101_STATE_RX_OVERFLOW
**											= CC1101_STATE_TX_UNDERFLOW
**			Bit3-0：RX FIFO中可用的字节数，或TX FIFO中剩余的空间	
******************************************************************/
uint8_t GetRfidCurStatus(void);
// /*****************************************************************
// ** 功能：设置CC1101地址滤波器的值
// ** 输入：Addr:地址过滤器的值
// ** 返回：RET_OK:设置成功；其余的返回状态可以参考rfid_config.h文件中的错误状态定义
// ******************************************************************/
uint8_t SetRxAddrFilter(uint8_t Addr);
void RfidSpiInit(void);
#endif	//__RFID_DRIVER_H__

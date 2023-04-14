#ifndef __USART_H
#define __USART_H

#include "gd32f1x0.h"
#include "string.h"
#define USART_TDATA_ADDRESS      ((uint32_t)0x40013828)
#define USART_RDATA_ADDRESS      ((uint32_t)0x40013824)

#define  DMA_TX   DMA_CH1
#define  DMA_RX   DMA_CH2


//#define  DMA_SPI_TX   DMA_CH1
//#define  DMA_SPI_RX   DMA_CH2


#define EVAL_COM1                        USART0
#define EVAL_COM1_CLK                    RCU_USART0

#define EVAL_COM1_TX_PIN                 GPIO_PIN_6
#define EVAL_COM1_RX_PIN                 GPIO_PIN_7

#define EVAL_COM_GPIO_PORT               GPIOB
#define EVAL_COM_GPIO_CLK                RCU_GPIOB
#define EVAL_COM_AF                      GPIO_AF_0


#define TRUE								1
#define FALSE								0
#define ENABLED								1
#define DISABLED							0

#define  SendHeartTimerMAX     500
#define  WIRELESSDEVIC        3
#define  USARTSENDBUFSIZE          129
#define  WIRELESSRESET   1


#define  USARTWIRELESS_TMP_BUF_MAX            140   //临时接收数组最大值
#define  USARTWIRELESS_RX_BUF_MAX              10   //接收队列最大值
#define  USARTWIRELESS_TX_BUF_MAX              10   //发送队列最大值

/**应答定义**/
enum
{
	NOACK=0,
	ACK
};
/**帧类别枚举**/
enum
{
	HEARTBEAT,//心跳
	WIRELESSDATA=1,//无线数据
	STARTHINT,//上电提示
	RESERVED,
	PARAMETERSETTING,//参数设置
	UPDATAPROGRAM,//更新程序
	REUPDATAPROGRAMWL,//CHONGXINSHENGJI
	RESTORATIONWL //FUWEI
};
//串口数据结构体8
typedef  struct
{
	uint8_t  FrameType;     //帧类别
	uint8_t  ACK;           //应答
	uint8_t  u8DLC;         //数据长度
	uint8_t  SendBuf[USARTSENDBUFSIZE];      //数据
	uint8_t  CRCHigh;         //CRC校验 高字节
	uint8_t  CRCLow;       //CRC校验  低字节
} stUARTDATA;
//串口发送结构体
typedef  struct
{
	uint8_t  Address;       //设备地址
 	uint8_t  Serial;        //流水号
    stUARTDATA  stFrame;	
} stUARTWIRELESS;

void gpio_init(void);
void usart_init(void);
void dma_config(void);
void usart_dma_send(uint8_t *buffer,uint16_t len);
void Crc16UsartReceiveChick(const uint8_t *u8Buf, uint32_t u32Len, uint16_t *u16CheckOld);
uint32_t UsartFetchData(stUARTWIRELESS *UsartRcvFrm);
#endif

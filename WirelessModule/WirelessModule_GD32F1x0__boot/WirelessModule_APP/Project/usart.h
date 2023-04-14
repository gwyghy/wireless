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


#define  USARTWIRELESS_TMP_BUF_MAX            140   //��ʱ�����������ֵ
#define  USARTWIRELESS_RX_BUF_MAX              10   //���ն������ֵ
#define  USARTWIRELESS_TX_BUF_MAX              10   //���Ͷ������ֵ

/**Ӧ����**/
enum
{
	NOACK=0,
	ACK
};
/**֡���ö��**/
enum
{
	HEARTBEAT,//����
	WIRELESSDATA=1,//��������
	STARTHINT,//�ϵ���ʾ
	RESERVED,
	PARAMETERSETTING,//��������
	UPDATAPROGRAM,//���³���
	REUPDATAPROGRAMWL,//CHONGXINSHENGJI
	RESTORATIONWL //FUWEI
};
//�������ݽṹ��8
typedef  struct
{
	uint8_t  FrameType;     //֡���
	uint8_t  ACK;           //Ӧ��
	uint8_t  u8DLC;         //���ݳ���
	uint8_t  SendBuf[USARTSENDBUFSIZE];      //����
	uint8_t  CRCHigh;         //CRCУ�� ���ֽ�
	uint8_t  CRCLow;       //CRCУ��  ���ֽ�
} stUARTDATA;
//���ڷ��ͽṹ��
typedef  struct
{
	uint8_t  Address;       //�豸��ַ
 	uint8_t  Serial;        //��ˮ��
    stUARTDATA  stFrame;	
} stUARTWIRELESS;

void gpio_init(void);
void usart_init(void);
void dma_config(void);
void usart_dma_send(uint8_t *buffer,uint16_t len);
void Crc16UsartReceiveChick(const uint8_t *u8Buf, uint32_t u32Len, uint16_t *u16CheckOld);
uint32_t UsartFetchData(stUARTWIRELESS *UsartRcvFrm);
#endif

/***********************************************************************************************
** �ļ���:  rfid_board.h
** �衡��:  RFIDӦ�ýӿ�ͷ�ļ�
*******************Copyright (c) 1998-1999 ��������Ӽ������޹�˾����������*********************/
#ifndef __RFID_BOARD_H__
#define __RFID_BOARD_H__

#include "rfid_config.h"

#if WL_SPI_TYPE > 0			//���ؼ�������

#define RCC_RFID_AF_APBxCmd()			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

#if WL_SPI_TYPE == 1				//���ؼ������ߺ��⣨SPI�ӿڣ�
/***********************************************************************************************/
//���ú��޸��Ե�Һ�ؿ���v1.0��SPI2������RFIDͨ��
/***********************************************************************************************/
/*** SPI RFID�ӿ� **************/
/**SPI RFID��ʹ�õ�Ӳ������***/
#define RFID_SPI						SPI1//rfid��ʹ�õ�SPI�ڶ���
#define GPIO_RFID_AF_DEFINE				GPIO_AF_SPI1//���ù��ܶ���
//#define RCC_RFID_APBxCmd()				RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE)//SPIʹ��ʱ�ӣ��������ڵ�APB1\APB2�����޸�

/*******************�������ö���*******************/
/***********************************************************************************************/

/*
 * RFID - SPI ��ӦMCU�ܽź궨��
 */
#define RFID_SPI_MCU_CS_PIN				PIN_RFID_CS
#define RFID_SPI_MCU_SCK_PIN			PIN_RFID_SCLK
#define RFID_SPI_MCU_MISO_PIN			PIN_RFID_MISO
#define RFID_SPI_MCU_MOSI_PIN			PIN_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PIN			PIN_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PIN			PIN_RFID_GDO0
/*
 * RFID - SPI ��Ӧ�˿ں궨��
 */
#define RFID_SPI_MCU_CS_PORT			GPIO_RFID_CS			
#define RFID_SPI_MCU_SCK_PORT			GPIO_RFID_SCLK
#define RFID_SPI_MCU_MISO_PORT			GPIO_RFID_MISO
#define RFID_SPI_MCU_MOSI_PORT			GPIO_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PORT			GPIO_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PORT			GPIO_RFID_GDO0
/*
 * RFID - SPI ��Ӧ�˿�ʱ�Ӻ궨��
 */
#define RFID_SPI_MCU_PORT_RCC			RCC_AHB1Periph_GPIOC
#define RFID_SPI_MCU_RCC				RCC_APB1Periph_SPI2
#define RFID_SPI_MCU_AFIO_RCC										//RCC_APB2Periph_AFIO
#define RFID_GDO0_MCU_RCC				RCC_RFID_GDO0
#define RFID_GDO2_MCU_RCC				RCC_RFID_GDO2
#define RFID_SPI_PORT					RFID_SPI
/*
 * GDOx�ж�����
 */
#define RFID_CC1101_GDO0_PORT_SOURCE	RFID_GDO0_EXTI_PORT_SOURCE
#define RFID_CC1101_GDO2_PORT_SOURCE	RFID_GDO2_EXTI_PORT_SOURCE
#define RFID_CC1101_GDO0_PIN_SOURCE		RFID_GDO0_EXTI_PIN_SOURCE
#define RFID_CC1101_GDO2_PIN_SOURCE		RFID_GDO2_EXTI_PIN_SOURCE
#define RFID_CC1101_GDO0_EXTI_LINE		RFID_GDO0_EXTI_LINE
#define RFID_CC1101_GDO2_EXTI_LINE		RFID_GDO2_EXTI_LINE
#define RFID_CC1101_GDOx_EXTI_IRQn		RFID_GDOx_EXTI_IRQn
/*
 * ����ʹ��λ����
 */
//#define	PAC_PORT						GPIOF
//#define PAC_GPIO_RCC					RCU_GPIOF
//#define PAC_PIN							GPIO_PIN_12

#else	//#if WL_SPI_TYPE == 1
#if WL_SPI_TYPE == 2
/***********************************************************************************************/
//WL_SPI_TYPE=2��USART������RFIDͨ��
/***********************************************************************************************/

/*** RFID��SPI�ӿ���USERT���� ***/
/**����˿ڡ����š���������(GPIO_Pin_sources)��ʱ�Ӷ���*****/
#define GPIO_RFID_SCLK		  			WL_PORT
#define PIN_RFID_SCLK					WL_CK_PIN

#define GPIO_RFID_MOSI		  			WL_PORT
#define PIN_RFID_MOSI					WL_TX_PIN

#define GPIO_RFID_MISO		  			WL_PORT
#define PIN_RFID_MISO					WL_RX_PIN

#define GPIO_RFID_CS	 				GPIOD
#define PIN_RFID_CS						GPIO_Pin_4
#define RCC_RFID_CS						RCC_AHB1Periph_GPIOD

/**����˿ڡ����š�ʱ�Ӷ���*****/
#define GPIO_RFID_GDO2					GPIOD
#define PIN_RFID_GDO2					GPIO_Pin_3
#define RCC_RFID_GDO2					RCC_AHB1Periph_GPIOD

#define GPIO_RFID_GDO0					GPIOD
#define PIN_RFID_GDO0					GPIO_Pin_0
#define RCC_RFID_GDO0					RCC_AHB1Periph_GPIOD

/*******************�ж϶���***********************/
#define RFID_GDO0_EXTI_PORT_SOURCE		EXTI_PortSourceGPIOD
#define RFID_GDO2_EXTI_PORT_SOURCE		EXTI_PortSourceGPIOD
#define RFID_GDO0_EXTI_PIN_SOURCE		EXTI_PinSource0
#define RFID_GDO2_EXTI_PIN_SOURCE		EXTI_PinSource3
#define RFID_GDO0_EXTI_LINE				EXTI_Line0
#define RFID_GDO2_EXTI_LINE				EXTI_Line3
#define RFID_GDO0_EXTI_IRQn				EXTI0_IRQn
#define RFID_GDO2_EXTI_IRQn				EXTI3_IRQn
#define RFID_GDO0_EXTI_IRQHandler		EXTI0_IRQHandler
#define RFID_GDO2_EXTI_IRQHandler		EXTI3_IRQHandler
/*******************�������ö���*******************/
/***********************************************************************************************/
/*
 * RFID - SPI ��ӦMCU�ܽź궨��
 */
#define RFID_SPI_MCU_CS_PIN				PIN_RFID_CS
#define RFID_SPI_MCU_SCK_PIN			PIN_RFID_SCLK
#define RFID_SPI_MCU_MISO_PIN			PIN_RFID_MISO
#define RFID_SPI_MCU_MOSI_PIN			PIN_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PIN			PIN_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PIN			PIN_RFID_GDO0
/*
 * RFID - SPI ��Ӧ�˿ں궨��
 */
#define RFID_SPI_MCU_CS_PORT			GPIO_RFID_CS			
#define RFID_SPI_MCU_SCK_PORT			GPIO_RFID_SCLK
#define RFID_SPI_MCU_MISO_PORT			GPIO_RFID_MISO
#define RFID_SPI_MCU_MOSI_PORT			GPIO_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PORT			GPIO_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PORT			GPIO_RFID_GDO0
/*
 * GDOx�ж�����
 */
#define RFID_CC1101_GDO0_PORT_SOURCE	RFID_GDO0_EXTI_PORT_SOURCE
#define RFID_CC1101_GDO2_PORT_SOURCE	RFID_GDO2_EXTI_PORT_SOURCE
#define RFID_CC1101_GDO0_PIN_SOURCE		RFID_GDO0_EXTI_PIN_SOURCE
#define RFID_CC1101_GDO2_PIN_SOURCE		RFID_GDO2_EXTI_PIN_SOURCE
#define RFID_CC1101_GDO0_EXTI_LINE		RFID_GDO0_EXTI_LINE
#define RFID_CC1101_GDO2_EXTI_LINE		RFID_GDO2_EXTI_LINE
#define RFID_CC1101_GDO0_EXTI_IRQn		RFID_GDO0_EXTI_IRQn
#define RFID_CC1101_GDO2_EXTI_IRQn		RFID_GDO2_EXTI_IRQn

#endif
#endif

/**RFID SPI���֧��10M***/
#define RFID_SPI_BAUNDRATE_PRESCALER	SPI_BaudRatePrescaler_64//ʱ�ӷ�Ƶϵ��.APB1:42M/32 = 1.3125 MHz

#endif	//#if WL_SPI_TYPE > 0

#endif /* __RFID_BOARD_H__*/
/*********************************������������޹�˾*******************************************/

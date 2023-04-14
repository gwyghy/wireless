/***********************************************************************************************
** 文件名:  rfid_board.h
** 描　述:  RFID应用接口头文件
*******************Copyright (c) 1998-1999 天津华宁电子技术有限公司技术开发部*********************/
#ifndef __RFID_BOARD_H__
#define __RFID_BOARD_H__

#include "rfid_config.h"

#if WL_SPI_TYPE > 0			//本地集成无线

#define RCC_RFID_AF_APBxCmd()			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

#if WL_SPI_TYPE == 1				//本地集成无线红外（SPI接口）
/***********************************************************************************************/
//借用和修改自电液控开发v1.0。SPI2用于与RFID通信
/***********************************************************************************************/
/*** SPI RFID接口 **************/
/**SPI RFID所使用的硬件定义***/
#define RFID_SPI						SPI1//rfid所使用的SPI口定义
#define GPIO_RFID_AF_DEFINE				GPIO_AF_SPI1//复用功能定义
//#define RCC_RFID_APBxCmd()				RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE)//SPI使能时钟，依据属于的APB1\APB2进行修改

/*******************结束借用定义*******************/
/***********************************************************************************************/

/*
 * RFID - SPI 对应MCU管脚宏定义
 */
#define RFID_SPI_MCU_CS_PIN				PIN_RFID_CS
#define RFID_SPI_MCU_SCK_PIN			PIN_RFID_SCLK
#define RFID_SPI_MCU_MISO_PIN			PIN_RFID_MISO
#define RFID_SPI_MCU_MOSI_PIN			PIN_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PIN			PIN_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PIN			PIN_RFID_GDO0
/*
 * RFID - SPI 对应端口宏定义
 */
#define RFID_SPI_MCU_CS_PORT			GPIO_RFID_CS			
#define RFID_SPI_MCU_SCK_PORT			GPIO_RFID_SCLK
#define RFID_SPI_MCU_MISO_PORT			GPIO_RFID_MISO
#define RFID_SPI_MCU_MOSI_PORT			GPIO_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PORT			GPIO_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PORT			GPIO_RFID_GDO0
/*
 * RFID - SPI 对应端口时钟宏定义
 */
#define RFID_SPI_MCU_PORT_RCC			RCC_AHB1Periph_GPIOC
#define RFID_SPI_MCU_RCC				RCC_APB1Periph_SPI2
#define RFID_SPI_MCU_AFIO_RCC										//RCC_APB2Periph_AFIO
#define RFID_GDO0_MCU_RCC				RCC_RFID_GDO0
#define RFID_GDO2_MCU_RCC				RCC_RFID_GDO2
#define RFID_SPI_PORT					RFID_SPI
/*
 * GDOx中断配置
 */
#define RFID_CC1101_GDO0_PORT_SOURCE	RFID_GDO0_EXTI_PORT_SOURCE
#define RFID_CC1101_GDO2_PORT_SOURCE	RFID_GDO2_EXTI_PORT_SOURCE
#define RFID_CC1101_GDO0_PIN_SOURCE		RFID_GDO0_EXTI_PIN_SOURCE
#define RFID_CC1101_GDO2_PIN_SOURCE		RFID_GDO2_EXTI_PIN_SOURCE
#define RFID_CC1101_GDO0_EXTI_LINE		RFID_GDO0_EXTI_LINE
#define RFID_CC1101_GDO2_EXTI_LINE		RFID_GDO2_EXTI_LINE
#define RFID_CC1101_GDOx_EXTI_IRQn		RFID_GDOx_EXTI_IRQn
/*
 * 功率使能位配置
 */
//#define	PAC_PORT						GPIOF
//#define PAC_GPIO_RCC					RCU_GPIOF
//#define PAC_PIN							GPIO_PIN_12

#else	//#if WL_SPI_TYPE == 1
#if WL_SPI_TYPE == 2
/***********************************************************************************************/
//WL_SPI_TYPE=2：USART用于与RFID通信
/***********************************************************************************************/

/*** RFID的SPI接口由USERT控制 ***/
/**输出端口、引脚、复用引脚(GPIO_Pin_sources)、时钟定义*****/
#define GPIO_RFID_SCLK		  			WL_PORT
#define PIN_RFID_SCLK					WL_CK_PIN

#define GPIO_RFID_MOSI		  			WL_PORT
#define PIN_RFID_MOSI					WL_TX_PIN

#define GPIO_RFID_MISO		  			WL_PORT
#define PIN_RFID_MISO					WL_RX_PIN

#define GPIO_RFID_CS	 				GPIOD
#define PIN_RFID_CS						GPIO_Pin_4
#define RCC_RFID_CS						RCC_AHB1Periph_GPIOD

/**输入端口、引脚、时钟定义*****/
#define GPIO_RFID_GDO2					GPIOD
#define PIN_RFID_GDO2					GPIO_Pin_3
#define RCC_RFID_GDO2					RCC_AHB1Periph_GPIOD

#define GPIO_RFID_GDO0					GPIOD
#define PIN_RFID_GDO0					GPIO_Pin_0
#define RCC_RFID_GDO0					RCC_AHB1Periph_GPIOD

/*******************中断定义***********************/
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
/*******************结束借用定义*******************/
/***********************************************************************************************/
/*
 * RFID - SPI 对应MCU管脚宏定义
 */
#define RFID_SPI_MCU_CS_PIN				PIN_RFID_CS
#define RFID_SPI_MCU_SCK_PIN			PIN_RFID_SCLK
#define RFID_SPI_MCU_MISO_PIN			PIN_RFID_MISO
#define RFID_SPI_MCU_MOSI_PIN			PIN_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PIN			PIN_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PIN			PIN_RFID_GDO0
/*
 * RFID - SPI 对应端口宏定义
 */
#define RFID_SPI_MCU_CS_PORT			GPIO_RFID_CS			
#define RFID_SPI_MCU_SCK_PORT			GPIO_RFID_SCLK
#define RFID_SPI_MCU_MISO_PORT			GPIO_RFID_MISO
#define RFID_SPI_MCU_MOSI_PORT			GPIO_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PORT			GPIO_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PORT			GPIO_RFID_GDO0
/*
 * GDOx中断配置
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

/**RFID SPI最大支持10M***/
#define RFID_SPI_BAUNDRATE_PRESCALER	SPI_BaudRatePrescaler_64//时钟分频系数.APB1:42M/32 = 1.3125 MHz

#endif	//#if WL_SPI_TYPE > 0

#endif /* __RFID_BOARD_H__*/
/*********************************天津华宁电子有限公司*******************************************/

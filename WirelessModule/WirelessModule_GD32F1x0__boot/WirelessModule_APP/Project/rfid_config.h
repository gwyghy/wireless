/*********************************************************************************************************************************
** �ļ���:  rfid_config.h
** �衡��:  RFID����ģ��ͷ�ļ�
** ������: 	����
** �ա���:  2014-12-26
** �޸���:	
** �ա���:	
**
** �桡��:	V1.0.0.0
** ���¼�¼:
** ���¼�¼	��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
**--------------------------------------------------------------------------
**************************Copyright (c) 1998-1999 ��������Ӽ������޹�˾����������*************************************************/
#ifndef __RFID_CTRL_H__
#define __RFID_CTRL_H__

#include <stdint.h>

// #if 0	//��rfid_board.h�����¶��壨�����ڵ�Һ�ؿ���v2.0��
// #include "stm32l1xx.h"

// /*
//  * RFID - SPI ��Ӧ�ܽź궨��
//  */
// #define RFID_SPI_MCU_CS_PIN				GPIO_Pin_12
// #define RFID_SPI_MCU_SCK_PIN			GPIO_Pin_13
// #define RFID_SPI_MCU_MISO_PIN			GPIO_Pin_14
// #define RFID_SPI_MCU_MOSI_PIN			GPIO_Pin_15
// #define RFID_SPI_MCU_GDO2_PIN			GPIO_Pin_7
// #define RFID_SPI_MCU_GDO0_PIN			GPIO_Pin_6
// /*
//  * RFID - SPI ��Ӧ�˿ں궨��
//  */
// #define RFID_SPI_MCU_CS_PORT			GPIOB			
// #define RFID_SPI_MCU_SCK_PORT			GPIOB
// #define RFID_SPI_MCU_MISO_PORT			GPIOB
// #define RFID_SPI_MCU_MOSI_PORT			GPIOB
// #define RFID_SPI_MCU_GDO2_PORT			GPIOC
// #define RFID_SPI_MCU_GDO0_PORT			GPIOC
// /*
//  * RFID - SPI ��Ӧ�˿�ʱ�Ӻ궨��
//  */
// #define RFID_SPI_MCU_PORT_RCC			RCC_AHBPeriph_GPIOB			//RCC_AHBPeriph_GPIOB,�޸�Ϊʹ��MCU��SPI�˿�
// #define RFID_SPI_MCU_RCC				RCC_APB1Periph_SPI2			//RCC_APB1Periph_SPI2
// #define RFID_SPI_MCU_AFIO_RCC										//RCC_APB2Periph_AFIO
// #define RFID_GDO_MCU_RCC				RCC_AHBPeriph_GPIOC
// #define RFID_SPI_PORT					SPI2
// /*
//  * GDOx�ж�����
//  */
// #define RFID_CC1101_GDOx_PORT_SOURCE	EXTI_PortSourceGPIOC		//GPIO_PortSourceGPIOC
// #define RFID_CC1101_GDO0_PIN_SOURCE		EXTI_PinSource6				//GPIO_PinSource6
// #define RFID_CC1101_GDO2_PIN_SOURCE		EXTI_PinSource7				//GPIO_PinSource7
// #define RFID_CC1101_GDO0_EXTI_LINE		EXTI_Line6
// #define RFID_CC1101_GDO2_EXTI_LINE		EXTI_Line7
// #define RFID_CC1101_GDOx_EXTI_IRQn		EXTI9_5_IRQn

// #endif		//#if 0
#define	WL_SPI_TYPE						1				//���ؼ������ߺ��⣨SPI�ӿڣ�
/*
 * ��������
 */
#define CC1101_ADDR_FILTER				1	// =1��ʹ�ܵ�ַ���˹��ܣ�=0��ʧ�ܵ�ַ���˹���
/*
 * ״̬�Ĵ����Ļ���ַ
 */
#define StatusRegBaseAddress			(0x30)

/*
 * CC1101-TX-FIFO Size
 */
#define CC1101_TX_FIFO_SIZE				(64)
#define CC1101_RX_FIFO_SIZE				(256)		//(64)

/*
 * RFID - �ز�����,��λ hz
 */
#define CC1101_CARRIER_FREQUENCY_DEFAULT	(433000000)
#define CC1101_CARRIER_FREQUENCY_MAX		(464000000)
#define CC1101_CARRIER_FREQUENCY_MIN		(387000000)

/*
 * CC1101 CCA����ʹ�ܻ�ʧ�ܶ���
 */
#define CC1101_CCA_ENABLED				1
#define CC1101_CCA_DISABLED				0
/*
 * CC1101�Ĵ������ýṹ�嶨��
 */
#define DEFAULT_RF_PWRIDX   			(7)    		//Default RF Output Power Index
#define DEFAULT_RF_ATNIDX    			(0)			//(2)			//Default RF Attenuation Index
#define DEFAULT_RF_VELOCITY  			(38)		//(250)  		//kbit/s
#define DEFAULT_RES_INTERVAL_TIME  		(0)			//s
#define DEFAULT_SLOT_TIME  				(20)	    //ms
#define DEFAULT_SLOT_SND_NUM  			(0)
#define	DEFAULT_CCA_USE_FLAG			CC1101_CCA_ENABLED

#define	DEFAULT_CHANNEL_SPACING			400000//hz,�ŵ���ȣ������400��������ֵ���������޸ģ�����������й�(rfid_cc1101.c)
/*
 * ����״̬
 */
#define RET_OK							(0)
#define RET_ERR							(1)
#define TX_LENGTH_ERR					(2)
#define CHIP_STATUS_ABNORMAL			(3)
#define RX_LENGTH_VIOLATION				(4)
#define RX_CRC_MISMATCH					(5)
#define RX_FRM_POOL_EMPTY				(6)
#define TX_OK							RET_OK
#define TX_ERR							RET_ERR
#define RX_OK							RET_OK
#define RX_ERR							RET_ERR
#define SPI_WR_ERROR					(7)
#define TIME_OVER						(0xff)
/*
 * ���ȴ�״̬��ʱʱ��
 */
#define MAX_STATUS_DELAY				(5)			//(100)
#define MAX_TX_DELAY					(50)
#define MAX_SPI_WR_TIMEOUT				(10000)
/*
 * ���ù���ģʽ����
 */
#define OBUMODE_ONROAD       			(0xfe01)	//·��ģʽ
#define OBUMODE_SLEEP        			(0xfe00)	//����ģʽ
/*
 * RFID��ر�ʶλ����
 */
typedef enum 
{
    RFCHIP_IDLE_POSITIVE = 0,
    RFCHIP_RESET_NECESSARILY = 1
}mRfChipRstFlag;

typedef enum 
{
    RFCHIP_RX_CLEARUP = 0,
    RFCHIP_TX_CLEARUP = 1
}mRFChipLastState;


//----------------------------------------------------------------------------------------
// 		�ṹ������
//----------------------------------------------------------------------------------------
/*
 * ����˥����������
 */
typedef struct
{
    uint8_t u8RFPwrIdx;    					//Output Power Table Index
    uint8_t u8RFAtnIdx;    					//Attenuation Table Index
    uint8_t u8RFVelocity;					//Velocity Table Index
    uint8_t u8ResIntervalTime;
    uint8_t u8SlotTime;
    uint8_t u8SlotSndNum;
} Tag_RFChipPar;

/* 
 * T_DRV_RF_CONFIG is a data structure which contains all relevant CC1101 registers
 */

typedef struct tagT_RF_CONFIG
{
	//u8 u8FSCTRL2;   // Frequency synthesizer control.
    uint8_t u8FSCTRL1;   // Frequency synthesizer control.
    uint8_t u8FSCTRL0;   // Frequency synthesizer control.
    uint8_t u8FREQ2;     // Frequency control word, high byte.
    uint8_t u8FREQ1;     // Frequency control word, middle byte.
    uint8_t u8FREQ0;     // Frequency control word, low byte.

    uint8_t u8MDMCFG4;   // Modem configuration.
    uint8_t u8MDMCFG3;   // Modem configuration.
    uint8_t u8MDMCFG2;   // Modem configuration.
    uint8_t u8MDMCFG1;   // Modem configuration.
    uint8_t u8MDMCFG0;   // Modem configuration.

    uint8_t u8CHANNR;    // Channel number.
    uint8_t u8DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    uint8_t u8FREND1;    // Front end RX configuration.
    uint8_t u8FREND0;    // Front end RX configuration.
    uint8_t u8MCSM0;     // Main Radio Control State Machine configuration.

    uint8_t u8FOCCFG;    // Frequency Offset Compensation Configuration.
    uint8_t u8BSCFG;     // Bit synchronization Configuration.
    uint8_t u8AGCCTRL2;  // AGC control.
    uint8_t u8AGCCTRL1;  // AGC control.
    uint8_t u8AGCCTRL0;  // AGC control.

    uint8_t u8FSCAL3;    // Frequency synthesizer calibration.
    uint8_t u8FSCAL2;    // Frequency synthesizer calibration.
    uint8_t u8FSCAL1;    // Frequency synthesizer calibration.
    uint8_t u8FSCAL0;    // Frequency synthesizer calibration.
    uint8_t u8FSTEST;    // Frequency synthesizer calibration control

    uint8_t u8TEST2;     // Various test settings.
    uint8_t u8TEST1;     // Various test settings.
    uint8_t u8TEST0;     // Various test settings.
    uint8_t u8IOCFG2;    // GDO2 output pin configuration
    uint8_t u8IOCFG0;    // GDO0 output pin configuration

    uint8_t u8PKTCTRL1;  // Packet automation control.
    uint8_t u8PKTCTRL0;  // Packet automation control.
    uint8_t u8ADDR;      // Device address.
    uint8_t u8PKTLEN;    // Packet length.
} Tag_RF_CONFIG;

#endif /* __RFID_CTRL_H__*/
/*********************************������������޹�˾*************************************************************/

/*********************************************************************************************************************************
** �ļ���:  rfid_cc1101.h
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
#include <stdint.h>
#include "rfid_config.h"
#ifndef __RFID_CC1101_H__
#define __RFID_CC1101_H__


#define BYTES_IN_RXFIFO					0x7f
#define CRC_OK							0x80
#define NULL							0

#define GPIO_RFID_SCLK		  			GPIOA
#define PIN_RFID_SCLK					GPIO_PIN_5
#define GPIO_RFID_MOSI		  			GPIOA
#define PIN_RFID_MOSI					GPIO_PIN_7
#define GPIO_RFID_MISO		  			GPIOA
#define PIN_RFID_MISO					GPIO_PIN_6



#define GPIO_RFID_CS	 				GPIOB
#define PIN_RFID_CS						GPIO_PIN_0
#define GPIO_RFID_GDO0                  GPIOB
#define PIN_RFID_GDO0                   GPIO_PIN_1
#define GPIO_RFID_GDO2                  GPIOA
#define PIN_RFID_GDO2                   GPIO_PIN_4

#define GPIO_WAKE_UP                    GPIOB
#define PIN_WAKE_UP                     GPIO_PIN_3

/*
 * RFID - SPI ��ӦMCU�ܽź궨��
 */
#define RFID_SPI_MCU_CS_PIN				PIN_RFID_CS
#define RFID_SPI_MCU_SCK_PIN			PIN_RFID_SCLK
#define RFID_SPI_MCU_MISO_PIN			PIN_RFID_MISO
#define RFID_SPI_MCU_MOSI_PIN			PIN_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PIN			PIN_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PIN			PIN_RFID_GDO0
#define RFID_SPI_MCU_WAKE_UP_PIN		PIN_RFID_GDO0
/*
 * RFID - SPI ��Ӧ�˿ں궨��
 */
#define RFID_SPI_MCU_CS_PORT			GPIO_RFID_CS			
#define RFID_SPI_MCU_SCK_PORT			GPIO_RFID_SCLK
#define RFID_SPI_MCU_MISO_PORT			GPIO_RFID_MISO
#define RFID_SPI_MCU_MOSI_PORT			GPIO_RFID_MOSI
#define RFID_SPI_MCU_GDO2_PORT			GPIO_RFID_GDO2
#define RFID_SPI_MCU_GDO0_PORT			GPIO_RFID_GDO0
#define RFID_SPI_MCU_WAKE_UP_PORT		GPIO_RFID_GDO0
/*******************�ж϶���***********************/
#define RFID_GDO0_EXTI_PORT_SOURCE		EXTI_PortSourceGPIOD
#define RFID_GDO2_EXTI_PORT_SOURCE		EXTI_PortSourceGPIOD
#define RFID_GDO0_EXTI_PIN_SOURCE		EXTI_PinSource1
#define RFID_GDO2_EXTI_PIN_SOURCE		EXTI_PinSource4
#define RFID_GDO0_EXTI_LINE				EXTI_LINE_1
#define RFID_GDO2_EXTI_LINE				EXTI_LINE_4
#define RFID_GDO0_EXTI_IRQn				EXTI1_IRQn
#define RFID_GDO2_EXTI_IRQn				EXTI4_IRQn
#define RFID_GDO0_EXTI_IRQHandler		EXTI1_IRQHandler
#define RFID_GDO2_EXTI_IRQHandler		EXTI4_IRQHandler
/*******************�������ö���*******************/

/*
 * GPIO�ܽŲ����궨��
 */
#define GPIO_SET(port, pin)          	GPIO_SetBits(port,pin)
#define GPIO_RESET(port, pin)          	GPIO_ResetBits(port,pin)
//#define GPIO_GET_PIN_STATUS(port, pin)	while(GPIO_ReadInputDataBit(port, pin))
#define GPIO_GET_PIN_STATUS(port, pin)	(WaitGPIOReset(port, pin))
/*
 * RFID - SPI �����궨��
 */
#define RFID_MCU_CS_DEASSERT  			gpio_bit_set(RFID_SPI_MCU_CS_PORT, RFID_SPI_MCU_CS_PIN)
#define RFID_MCU_CS_ASSERT  			gpio_bit_reset(RFID_SPI_MCU_CS_PORT, RFID_SPI_MCU_CS_PIN)
//#define RFID_MCU_CS_DEASSERT  			GPIO_SET(RFGPIO_PIN_SETID_SPI_MCU_CS_PORT, RFID_SPI_MCU_CS_PIN)

#define RFID_MCU_SCK_SET				gpio_bit_set(RFID_SPI_MCU_SCK_PORT, RFID_SPI_MCU_SCK_PIN)
#define RFID_MCU_SI_RESET				gpio_bit_reset(RFID_SPI_MCU_MOSI_PORT, RFID_SPI_MCU_MOSI_PIN)

#define RFID_SPI_MCU_WAIT_MISO			GPIO_GET_PIN_STATUS(RFID_SPI_MCU_MISO_PORT, RFID_SPI_MCU_MISO_PIN)

#define st(x)                   		do { x } while (0)
#define RFID_SPI_MCU_BEGIN        		st( RFID_MCU_CS_ASSERT; RFID_SPI_MCU_WAIT_MISO;)
#define RFID_SPI_MCU_END        		st( RFID_MCU_CS_DEASSERT;)

#define RFID_GDO2_EVENT					GPIO_ReadInputDataBit(RFID_SPI_MCU_GDO2_PORT, RFID_SPI_MCU_GDO2_PIN)


/************************************CC1101�Ĵ�����ַ����**********************************************/
/*
 * ���üĴ�����ַ
 */
#define CC1101_IOCFG2       0x00        // GDO2 ����ܽ�����
#define CC1101_IOCFG1       0x01        // GDO1 ����ܽ�����
#define CC1101_IOCFG0       0x02        // GDO0 ����ܽ�����
#define CC1101_FIFOTHR      0x03        // RX FIFO �� TX FIFO ��ֵ����
#define CC1101_SYNC1        0x04        // ͬ���֣����ֽ�
#define CC1101_SYNC0        0x05        // ͬ���֣����ֽ�
#define CC1101_PKTLEN       0x06        // ���ݰ��������üĴ�����ַ
#define CC1101_PKTCTRL1     0x07        // ���ݰ��Զ�����
#define CC1101_PKTCTRL0     0x08        // ���ݰ��Զ�����
#define CC1101_ADDR         0x09        // ������ַ
#define CC1101_CHANNR       0x0A        // ͨ����
#define CC1101_FSCTRL1      0x0B        // Ƶ�ʷ����ǿ���
#define CC1101_FSCTRL0      0x0C        // Ƶ�ʷ����ǿ���
#define CC1101_FREQ2        0x0D        // Ƶ�ʿ�����, ���ֽ�
#define CC1101_FREQ1        0x0E        // Ƶ�ʿ�����, �м��ֽ�
#define CC1101_FREQ0        0x0F        // Ƶ�ʿ�����, ���ֽ�
#define CC1101_MDMCFG4      0x10        // Modem configuration
#define CC1101_MDMCFG3      0x11        // Modem configuration
#define CC1101_MDMCFG2      0x12        // Modem configuration
#define CC1101_MDMCFG1      0x13        // Modem configuration
#define CC1101_MDMCFG0      0x14        // Modem configuration
#define CC1101_DEVIATN      0x15        // Modem deviation setting
#define CC1101_MCSM2        0x16        // �����߿���״̬������
#define CC1101_MCSM1        0x17        // �����߿���״̬������
#define CC1101_MCSM0        0x18        // �����߿���״̬������
#define CC1101_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CC1101_BSCFG        0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#define CC1101_WOREVT1      0x1E        // High byte Event 0 timeout
#define CC1101_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CC1101_WORCTRL      0x20        // Wake On Radio control
#define CC1101_FREND1       0x21        // Front end RX configuration
#define CC1101_FREND0       0x22        // Front end TX configuration
#define CC1101_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27        // RC oscillator configuration
#define CC1101_RCCTRL0      0x28        // RC oscillator configuration
#define CC1101_FSTEST       0x29        // Frequency synthesizer calibration control
#define CC1101_PTEST        0x2A        // Production test
#define CC1101_AGCTEST      0x2B        // AGC test
#define CC1101_TEST2        0x2C        // Various test settings
#define CC1101_TEST1        0x2D        // Various test settings
#define CC1101_TEST0        0x2E        // Various test settings

//Status Registers
#define CC1101_PARTNUM          0x30
#define CC1101_VERSION          0x31
#define CC1101_FREQEST          0x32
#define CC1101_LQI              0x33
#define CC1101_RSSI             0x34
#define CC1101_MARCSTATE        0x35
#define CC1101_WORTIME1         0x36
#define CC1101_WORTIME0         0x37
#define CC1101_PKTSTATUS        0x38
#define CC1101_VCO_VC_DAC       0x39
#define CC1101_TXBYTES          0x3A
#define CC1101_RXBYTES          0x3B
#define CC1101_RCCTRL1_STATUS   0x3C
#define CC1101_RCCTRL0_STATUS   0x3D

// Multi byte memory locations
#define CC1101_PATABLE      	0x3E
#define CC1101_TXFIFO       	0x3F
#define CC1101_RXFIFO       	0x3F

/*
 * Definitions for burst/single access to registers
 */
#define CC1101_WRITE_BURST  	0x40
#define CC1101_READ_SINGLE  	0x80
#define CC1101_READ_BURST   	0xC0

/* 
 * Strobe ����
 */
#define CC1101_SRES         	0x30        // ��λоƬ.
#define CC1101_SFSTXON      	0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
											// If in RX/TX: Go to a wait state where only the synthesizer is
											// running (for quick RX / TX turnaround).
#define CC1101_SXOFF        	0x32        // �ص�����
#define CC1101_SCAL         	0x33        // У׼Ƶ�ʷ����ǲ��ص���ʹ�ܿ���������                                 
#define CC1101_SRX          	0x34        // ʹ�� ��RX���� �����һ��״̬��SIDLE����MCSM0.FS_AUTOCAL=1������ִ��У׼
#define CC1101_STX          	0x35        // �ڿ���״̬: ʹ�� ��TX���������һ��״̬��SIDLE����MCSM0.FS_AUTOCAL=1������ִ��У׼
											// �������RX״̬����CCA��ʹ�ܣ������ʱͨ����࣬������TX�� 
#define CC1101_SIDLE        	0x36        // Exit RX / TX, turn off frequency synthesizer and exit
											// Wake-On-Radio mode if applicable.
#define CC1101_SAFC         	0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR         	0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD         	0x39        // ��CSn����ʱ���������ģʽ
#define CC1101_SFRX         	0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         	0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      	0x3C        // ��λʵʱʱ��.
#define CC1101_SNOP         	0x3D        // No operation. May be used to pad strobe commands to two
											// bytes for simpler software.

//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

// оƬ״̬�ֽ��ж�Ӧ��λ��
#define CC1101_STATUS_CHIP_RDYn_BM             0x80				// оƬ�Ƿ�׼��������
#define CC1101_STATUS_STATE_BM                 0x70				// оƬ��ǰ�Ĺ���״̬λ����
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F				// TX/RX FIFO��ʣ����ֽ�������

// оƬ��״̬
#define CC1101_STATE_IDLE                      0x00
#define CC1101_STATE_RX                        0x10
#define CC1101_STATE_TX                        0x20
#define CC1101_STATE_FSTXON                    0x30
#define CC1101_STATE_CALIBRATE                 0x40
#define CC1101_STATE_SETTLING                  0x50
#define CC1101_STATE_RX_OVERFLOW               0x60
#define CC1101_STATE_TX_UNDERFLOW              0x70


//----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
#define CC1101_LQI_CRC_OK_BM                   0x80
#define CC1101_LQI_EST_BM                      0x7F

//----------------------------------------------------------------------------------
// MARCSTATE �C Main Radio Control State Machine State
//----------------------------------------------------------------------------------
#define CC1101_MARCSTATE_SLEEP             0
#define CC1101_MARCSTATE_IDLE              1
#define CC1101_MARCSTATE_XOFF              2
#define CC1101_MARCSTATE_VCOON_MC          3
#define CC1101_MARCSTATE_REGON_MC          4
#define CC1101_MARCSTATE_MANCAL            5
#define CC1101_MARCSTATE_VCOON             6
#define CC1101_MARCSTATE_REGON             7
#define CC1101_MARCSTATE_STARTCAL          8
#define CC1101_MARCSTATE_BWBOOST           9
#define CC1101_MARCSTATE_FS_LOCK           10
#define CC1101_MARCSTATE_IFADCON           11
#define CC1101_MARCSTATE_ENDCAL            12
#define CC1101_MARCSTATE_RX                13
#define CC1101_MARCSTATE_RX_END            14
#define CC1101_MARCSTATE_RX_RST            15
#define CC1101_MARCSTATE_TXRX_SWITCH       16
#define CC1101_MARCSTATE_RXFIFO_OVERFLOW   17
#define CC1101_MARCSTATE_FSTXON            18
#define CC1101_MARCSTATE_TX                19
#define CC1101_MARCSTATE_TX_END            20
#define CC1101_MARCSTATE_RXTX_SWITCH       21
#define CC1101_MARCSTATE_TXFIFO_UNDERFLOW  22

/*
 * ����SPI�����ӹ���ģʽ
 */
enum {
	Master = 0,//
	Slave
};

typedef struct{
	uint8_t u8DataLen;			//���ݰ�����,��������ַ�ֽ�
	uint8_t u8DestAddr;
	uint8_t u8Data[CC1101_TX_FIFO_SIZE];//[64]
	uint8_t u8AppendStatus[2];
} st_RFIDRcvFrame;
/*
 * ��������
 */
void InitRfidIntIO(void);
void RfidChipReset(void);						// ����RFIDоƬ��λ
uint32_t SpiResetChipSingle(void);
void RFID_SpiPortInit(uint8_t u8OptMode);
void RFID_HarewreInit(void);
void RFID_SpiInit(void);
uint8_t RFID_SPI_SendByte(uint8_t u8Byte);
uint8_t RfidStrobe(uint8_t u8StrobeCmd);
 uint8_t SpiMcuWrite(uint8_t u8Addr, const uint8_t* pu8Data, uint16_t u16Length);
uint8_t RfidSpiWrite(uint8_t u8Addr, const uint8_t* pu8Data, uint16_t u16Length);
uint8_t RfidWriteReg(uint8_t u8Addr, uint8_t u8Data);
 uint8_t SpiMcuRead(uint8_t u8Addr, uint8_t* pu8Data, uint16_t u16Length);
uint8_t RfidReadStatusReg(uint8_t u8Addr);
uint8_t RfidWriteFifo(const uint8_t *pu8Data, uint8_t u8Length);
uint8_t RfidReadFifo(uint8_t *pu8Data, uint8_t u8Length);
void RfidRxRegInit(void);
void SetRFChipSleep(void);
uint8_t RfidReadFifoProc(uint8_t *pu8Data, uint8_t *pu8Length);
uint8_t RFTxSendPacket(uint8_t* pu8Data, uint8_t u8Length);
//void EXTI9_5_IRQHandler(void);
void SetRFChipSleepMode(void);
void SetRxMode(void);
uint8_t RfidGetTxStatus(void);
uint8_t MCU_SpiSendByte(uint8_t u8Data);
void MCU_WriteRfidConfig(void);
uint32_t RFID_GetFrame(st_RFIDRcvFrame *RfidRcvFrm);
uint8_t RFCtrlSetIDLE(void);
void RfidReadAllStatusReg(uint8_t* pu8Data, uint16_t u16Length);
void ClrRxFifo(void);
void delay_init(uint8_t SYSCLK);
void DelayUs(uint32_t u32Counter);
#endif /* __RFID_CC1101_H__*/
/*********************************������������޹�˾*************************************************************/

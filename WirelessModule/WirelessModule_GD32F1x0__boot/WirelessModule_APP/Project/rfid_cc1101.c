/*********************************************************************************************************************************
** �ļ���:  rfid_cc1101.h
** �衡��:  RFID����ģ��ʵ���ļ�
** 		
**			RFIDģ����ʹ�õ�һЩ��������˵����
**
** 				Chipcon
** 				Product = CC1101
** 				Chip version = A   (VERSION = 0x04)
**				Crystal accuracy = 10 ppm
** 				X-tal frequency = 26 MHz
** 				RF output power = 0 dBm
** 				RX filterbandwidth = 541.666667 kHz
** 				Deviation = 127 kHz
** 				Data rate = 250 kBaud or 38.4K(Only two)
** 				Modulation = 2-FSK
** 				Manchester enable = (0) Manchester disabled
** 				RF Frequency = 432.999817 MHz
** 				Channel spacing = 199.951172 kHz
** 				Channel number = 0
** 				Optimization = Sensitivity
** 				Sync mode = (3) 30/32 sync word bits detected
** 				Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
** 				CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
** 				Forward Error Correction = (0) FEC disabled
** 				Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
** 				Packetlength = 255
** 				Preamble count = (2)  4 bytes
** 				Append status = 1
** 				Address check = (0) No address check
** 				FIFO autoflush = 0
** 				Device address = 0
** 				GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
** 				GDO2 signal selection = (41) CHIP_RDY
**
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

#include "rfid_cc1101.h"
#include "rfid_driver.h"
#include "systick.h"
#include "usart.h"
#include "gd32f1x0_gpio.h"
#if WL_SPI_TYPE > 0	//��Һ��2.0��RFID

 uint32_t vu32GDO2IntFlag = 0;	// GDO�����ش�����־������һ�����ݽ���У����ȷ
 uint32_t vu32GDO0IntFlag = 0;	// GDO�½��ش�����־������һ�����ݷ������
const uint8_t uc8Patabel[8] = {0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0};
#if 1
/*const*/ Tag_RF_CONFIG st_RfConfig = {
    0x0C,	//0x06,   // FSCTRL1   	Ƶ�ʺϳ������� 1
    0x00,   // FSCTRL0   	Ƶ�ʺϳ������� 0
    0x10,   // FREQ2     	Ƶ�ʿ�����, ���ֽ�
    0xA7,   // FREQ1     	Ƶ�ʿ�����, �θ��ֽ�
    0x62,   // FREQ0     	Ƶ�ʿ�����, ���ֽ�

    0x2D,   // MDMCFG4   	��������
    0x3B,   // MDMCFG3   	��������
    0x03,   // MDMCFG2   	���ƽ��������
			// 				bit7 = 0,�ڽ��֮ǰ���ر�����ֱ���˲�ģ��
			//				bit6-4 = 0,ʹ��2-FSK���Ƹ�ʽ
			//				bit3 = 0,ʧ������˹�ر����
			//				bit2-0 = 3,30/32ͬ����λ���
    0x22,   // MDMCFG1   	���ƽ��������
			//				bit7 = 0,ʧ��ǰ�����룬ֻ�Թ̶����ݳ�������
			//				bit6-4 = 010,�����ֽڸ���Ϊ4
			//				bit1-0 = 10��ͨ���ռ��2λ��
    0xF8,   // MDMCFG0   	��������

    0x00,   // CHANNR    	ͨ����
    0x62,   // DEVIATN   	���ƽ����ƫ������ (when FSK modulation is enabled).
    0xB6,   // FREND1    	ǰ��RX ����
    0x10,   // FREND0    	ǰ��TX ����
    0x18,   // MCSM0     	CC1101�����������߼�״̬����

    0x1D,   // FOCCFG    	Ƶ��ƫ�Ʋ�������
    0x1C,   // BSCFG     	λͬ������
    0xC7,   // AGCCTRL2  	AGC ����
    0x00,   // AGCCTRL1  	AGC ����
    0xB0,   // AGCCTRL0  	AGC ����

    0xEA,   // FSCAL3    	Ƶ�ʺϳ���У׼
    0x2A,   // FSCAL2    	Ƶ�ʺϳ���У׼
    0x00,   // FSCAL1    	Ƶ�ʺϳ���У׼
    0x1F,   // FSCAL0    	Ƶ�ʺϳ���У׼
    0x59,   // FSTEST    	Ƶ�ʺϳ���У׼

    0x88,   // TEST2     	��ͬ��������
    0x31,   // TEST1     	��ͬ��������
    0x09,   // TEST0     	��ͬ��������
    0x07,   // IOCFG2    	GDO2 ����ܽ����ã��������ݽ��գ������յ�����У����ȷ��MCUһ���ⲿ�ж��ź�
    0x06,   // IOCFG0    	GDO0 ����ܽ����ã��������ݷ��ͣ���������ɺ�����ж��ź� 
#if (CC1101_ADDR_FILTER > 0)
    0x0E,   // PKTCTRL1  	���ݰ��Զ����ƼĴ���1��0x00001100,
			//				bit3 = 1,CRCУ�鲻��ȷ�Զ���ˢRX FIFO����ʧ��
			//				bit2 = 1,���ݺ����ٸ�������״̬�ֽ�
			//				bit1-0 = 00���޵�ַ��飻 =10,��ַ��飬00Ϊ�㲥��ַ��
#else
	0x0C,
#endif
    0x45,   // PKTCTRL0  	���ݰ��Զ����ƼĴ���0��0x01000101��
			// 				bit6 = 1�����ݼ��ܿ�����
			// 				bit5-4 = 00�����������շ�ģʽ��
			// 				bit2 = 1��CRCʹ�ܣ�
			// 				bit1-0 = 01���ɱ����ݰ�����ģʽ,length ��sync��ߵ� 1 ���ֽڣ�
			
    0xFE,	//0x55,   // ADDR      	������ַ��Ĭ��0x00
    0x40,	//0x0E,   // PKTLEN    	���ݰ����ȣ�length = 61;
};
#endif
#if 0
/*const*/ Tag_RF_CONFIG st_RfConfig = {
    0x0C,   // FSCTRL1   	Ƶ�ʺϳ������� 1
    0x00,   // FSCTRL0   	Ƶ�ʺϳ������� 0
    0x10,   // FREQ2     	Ƶ�ʿ�����, ���ֽ�
    0xA7,   // FREQ1     	Ƶ�ʿ�����, �θ��ֽ�
    0x62,   // FREQ0     	Ƶ�ʿ�����, ���ֽ�

    0x2D,   // MDMCFG4   	��������
    0x3B,   // MDMCFG3   	��������
    0x03,   // MDMCFG2   	���ƽ��������
			// 				bit7 = 0,�ڽ��֮ǰ���ر�����ֱ���˲�ģ��
			//				bit6-4 = 0,ʹ��2-FSK���Ƹ�ʽ
			//				bit3 = 0,ʧ������˹�ر����
			//				bit2-0 = 3,30/32ͬ����λ���
    0x22,   // MDMCFG1   	���ƽ��������
			//				bit7 = 0,ʧ��ǰ�����룬ֻ�Թ̶����ݳ�������
			//				bit6-4 = 010,�����ֽڸ���Ϊ4
			//				bit1-0 = 10��ͨ���ռ��2λ��
    0xF8,   // MDMCFG0   	��������

    0x00,   // CHANNR    	ͨ����
    0x62,   // DEVIATN   	���ƽ����ƫ������ (when FSK modulation is enabled).
    0x56,   // FREND1    	ǰ��RX ����
    0x10,   // FREND0    	ǰ��TX ����
    0x18,   // MCSM0     	CC1101�����������߼�״̬����

    0x1D,   // FOCCFG    	Ƶ��ƫ�Ʋ�������
    0x1C,   // BSCFG     	λͬ������
    0xC7,   // AGCCTRL2  	AGC ����
    0x00,   // AGCCTRL1  	AGC ����
    0xB0,   // AGCCTRL0  	AGC ����

    0xEA,   // FSCAL3    	Ƶ�ʺϳ���У׼
    0x2A,   // FSCAL2    	Ƶ�ʺϳ���У׼
    0x00,   // FSCAL1    	Ƶ�ʺϳ���У׼
    0x1F,   // FSCAL0    	Ƶ�ʺϳ���У׼
    0x59,   // FSTEST    	Ƶ�ʺϳ���У׼

    0x88,   // TEST2     	��ͬ��������
    0x31,   // TEST1     	��ͬ��������
    0x09,   // TEST0     	��ͬ��������
    0x07,   // IOCFG2    	GDO2 ����ܽ����ã��������ݽ��գ������յ�����У����ȷ��MCUһ���ⲿ�ж��ź�
    0x06,   // IOCFG0    	GDO0 ����ܽ����ã��������ݷ��ͣ���������ɺ�����ж��ź� 
#if (CC1101_ADDR_FILTER > 0)
    0x0E,   // PKTCTRL1  	���ݰ��Զ����ƼĴ���1��0x00001100,
			//				bit3 = 0,CRCУ�鲻��ȷ�Զ���ˢRX FIFO����ʹ��
			//				bit2 = 1,���ݺ����ٸ�������״̬�ֽ�
			//				bit1-0 = 00���޵�ַ��飻 =10,��ַ��飬00Ϊ�㲥��ַ��
#else
	0x0C,
#endif

#if (CC1101_PKTLEN_MODE > 0)
	0x44,
#else
    0x45,   // PKTCTRL0  	���ݰ��Զ����ƼĴ���0��0x01000101��
			// 				bit6 = 1�����ݼ��ܿ�����
			// 				bit5-4 = 00�����������շ�ģʽ��
			// 				bit2 = 1��CRCʹ�ܣ�
			// 				bit1-0 = 01���ɱ����ݰ�����ģʽ,length ��sync��ߵ� 1 ���ֽڣ�
#endif			
    0xFE,   // ADDR      	������ַ��Ĭ��0x00
    0x40,   // PKTLEN    	���ݰ����ȣ�length = 61;
};
#endif
const uint8_t st_RfConfigIndex[]={
    CC1101_FSCTRL1,     // FSCTRL1   Frequency synthesizer control.
    CC1101_FSCTRL0,     // FSCTRL0   Frequency synthesizer control.
    CC1101_FREQ2,       // FREQ2     Frequency control word, high byte.
    CC1101_FREQ1,       // FREQ1     Frequency control word, middle byte.
    CC1101_FREQ0,       // FREQ0     Frequency control word, low byte.
 
    CC1101_MDMCFG4,     // MDMCFG4   Modem configuration.
    CC1101_MDMCFG3,     // MDMCFG3   Modem configuration.
    CC1101_MDMCFG2,     // MDMCFG2   Modem configuration.
    CC1101_MDMCFG1,     // MDMCFG1   Modem configuration.
    CC1101_MDMCFG0,     // MDMCFG0   Modem configuration.
 
    CC1101_CHANNR,      // CHANNR    Channel number.
    CC1101_DEVIATN,     // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    CC1101_FREND1,      // FREND1    Front end RX configuration.
    CC1101_FREND0,      // FREND0    Front end TX configuration.
    CC1101_MCSM0,       // MCSM0     Main Radio Control State Machine configuration.
 
    CC1101_FOCCFG,      // FOCCFG    Frequency Offset Compensation Configuration.
    CC1101_BSCFG,       // BSCFG     Bit synchronization Configuration.
    CC1101_AGCCTRL2,    // AGCCTRL2  AGC control.
    CC1101_AGCCTRL1,    // AGCCTRL1  AGC control.
    CC1101_AGCCTRL0,    // AGCCTRL0  AGC control.
 
    CC1101_FSCAL3,      // FSCAL3    Frequency synthesizer calibration.
    CC1101_FSCAL2,      // FSCAL2    Frequency synthesizer calibration.
    CC1101_FSCAL1,      // FSCAL1    Frequency synthesizer calibration.
    CC1101_FSCAL0,      // FSCAL0    Frequency synthesizer calibration.
    CC1101_FSTEST,      // FSTEST    Frequency synthesizer calibration.
 
    CC1101_TEST2,       // TEST2     Various test settings.
    CC1101_TEST1,       // TEST1     Various test settings.
    CC1101_TEST0,       // TEST0     Various test settings.
    CC1101_IOCFG2,      // IOCFG2    GDO2 output pin configuration.
    CC1101_IOCFG0,      // IOCFG0    GDO0 output pin configuration.
 
    CC1101_PKTCTRL1,    // PKTCTRL1  Packet automation control.
    CC1101_PKTCTRL0,    // PKTCTRL0  Packet automation control.
    CC1101_ADDR,        // ADDR      Device address.
    CC1101_PKTLEN,      // PKTLEN    Packet length.
};
//----------------------------------------------------------------------------------------
// 		���鳣������
//----------------------------------------------------------------------------------------
/*
 * Optimum PATABLE Settings for Various Output Power Levels
 */
const uint8_t s_u8RFOutputPowerCfgTab[8] =
{
/*  PATABLE Setings     Output Power    comment*/
    0x12,               //-30dBm
    0x0E,               //-20dBm
    0x1D,               //-15dBm
    0x34,               //-10dBm
    0x60,               //0dBm
    0x84,               //+5dBm
    0xC8,               //+7dBm
    0xC0,               //+10dBm
};
/*
 * FIFOTHR.CLOSE_IN_RX - RX Attenuation, Typical Values
 */
static const uint8_t s_u8RFAttenuationCfgTab[4] =
{
/*  FIFOTHR Setings     	CLOSE_IN_RX     RX Attenuation*/
    0x07,                	// 0(00)         	0dB
    0x17,               	// 1(01)         	6dB
    0x27,               	// 2(10)         	12dB
    0x37                	// 3(11)         	18dB        
};
/* 
 * FIFOTHR.ADC_RETENTION - RX FIFO �� TX FIFO ��ֵ
 */
static const uint8_t s_u8RFCReg_FIFOTHR_BIT6[2] = 
{
    0x00,       			//250KBaud
    0x40        			//38.4KBaud
};
const uint8_t s_u8RFCReg_MCSM2[2] = 
{
    0x06,       			//250KBaud
    0x04       				//38.4KBaud
};
//Data Rate �Ĵ�������
/* 
 * MDMCFG4 - Modem Configuration: Sets the decimation ratio for 
 * the delta-sigma ADC input stream and thus the channel bandwidth. 
 */
const uint8_t s_u8RFCReg_MDMCFG4[2] = 
{
    0x2D,       //250KBaud
    0xCA        //38.4KBaud
};
/* MDMCFG3 - Modem Configuration: The mantissa of the user specified symbol rate.*/
const uint8_t s_u8RFCReg_MDMCFG3[2] = 
{
    0x3B,       //250KBaud
    0x83        //38.4KBaud
};
/* FSCAL3 - Frequency Synthesizer Calibration */
const uint8_t s_u8RFCReg_FSCAL3[2] = 
{
    0xEA,       //250KBaud
    0xE9        //38.4KBaud
};

/* TEST2 - Various Test Settings */
const uint8_t s_u8RFCReg_TEST2[2] = 
{
    0x88,       //250KBaud
    0x81        //38.4KBaud
};
/* TEST1 - Various Test Settings */
const uint8_t s_u8RFCReg_TEST1[2] = 
{
    0x31,       //250KBaud
    0x35        //38.4KBaud
};
/* DEVIATN - Modem Deviation Setting */
const uint8_t s_u8RFCReg_DEVIATN[2] = 
{
    0x62,       //250KBaud
    0x34        //38.4KBaud
};
uint8_t g_u8RFVelocityIdx = 1;		//38.4kbps	//0;	//250kbps
uint16_t g_u16RunningMode = OBUMODE_ONROAD;

Tag_RFChipPar st_RFChipPar = 
{
	DEFAULT_RF_PWRIDX, 
	DEFAULT_RF_ATNIDX, 
	DEFAULT_RF_VELOCITY, 
	DEFAULT_RES_INTERVAL_TIME, 
	DEFAULT_SLOT_TIME, 
	DEFAULT_SLOT_SND_NUM
};
void RfidConfig(/*const*/ Tag_RF_CONFIG* ptRfConfig, const uint8_t u8RfPaTable);

uint8_t u8RcvLength = 64;										// ���ڴ洢��CC1101��RX FIFO�н��յ����ݵĳ���
uint8_t u8RcvData[CC1101_RX_FIFO_SIZE+8];						// ���ڴ洢��CC1101��RX FIFO�н��յ�һ������
#define RFID_RCV_FRM_SIZE			10						// �洢�س���
st_RFIDRcvFrame st_RFIDRcvFrmPool[RFID_RCV_FRM_SIZE];		// RFID�������ݴ洢��
uint32_t u32RcvFrmPoolWritePtr = 0;
uint32_t u32RcvFrmPoolReadPtr = 0;
uint32_t u32RcvFrmPoolCnt = 0;

 uint32_t vu32CC1100State;										// 1=CC1100���ڷ���״̬
RCVED_BACK_CALL_FUNC RcvedBackCallFunc=NULL;				// ���պ�Ӧ�ò�ʵ�ֵĻص�����



/***********************************************************************************************
** �� �� ����	RFID_HarewreInit()
** ����������	
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void RFID_HarewreInit(void)
{
//	uint16_t i = 0;
//	HAL_NVIC_DisableIRQ(RFID_GDO0_EXTI_IRQn);
//	__HAL_GPIO_EXTI_CLEAR_IT(RFID_SPI_MCU_GDO0_PIN);
//	__HAL_GPIO_EXTI_CLEAR_IT(RFID_SPI_MCU_GDO0_PIN);
//	__HAL_GPIO_EXTI_CLEAR_FLAG(RFID_SPI_MCU_GDO0_PIN);
//	__HAL_GPIO_EXTI_CLEAR_FLAG(RFID_SPI_MCU_GDO0_PIN);
//	HAL_NVIC_ClearPendingIRQ(RFID_GDO0_EXTI_IRQn);
//	HAL_NVIC_ClearPendingIRQ(RFID_GDO2_EXTI_IRQn);
//#if OS_CRITICAL_METHOD == 3u
//    OS_CPU_SR  cpu_sr = 0u;
//#endif	
//	OS_ENTER_CRITICAL();							//��ֹȫ���ж�
	RfidChipReset();
	delay_1us(10);
    /*
	 * оƬ����
	 */ 	 
	RfidConfig(&st_RfConfig,s_u8RFOutputPowerCfgTab[st_RFChipPar.u8RFPwrIdx]);
//	OS_EXIT_CRITICAL();								//�ظ�ȫ���ж�
//	delay_1us(10);
	
	/*
	 * Additional chip configuration
	 */ 
	#if (DEFAULT_CCA_USE_FLAG == CC1101_CCA_ENABLED)
		RfidWriteReg(CC1101_MCSM1, 0x3F); //Use CCA, RX after TX and RX
		//RfidWriteReg(CC1101_MCSM1, 0x0C); // No CCA, IDLE after TX and RX			
	#else	
		RfidWriteReg(CC1101_MCSM1, 0x0F); // No CCA, RX after TX and RX
	#endif	
	//RfidWriteReg(CC1101_MCSM1, 0x30); // CCA enabled, IDLE after TX and RX
	RfidWriteReg(CC1101_SYNC0, 0x33); // SYNC0
	RfidWriteReg(CC1101_SYNC1, 0x33); // SYNC
	RfidWriteReg(CC1101_IOCFG0,0x06);
	RfidWriteReg(CC1101_IOCFG2,0x07);
}
/***********************************************************************************************
** �� �� ����	WaitGPIOReset
** ����������	�ȴ�ָ��GPIO����Reset�ź�
** �䡡  �룺	port: GPIO PORT
**         ��	pin: GPIO PIN
** �䡡  ����	FALSE=�ȴ���ʱ
************************************************************************************************/
uint32_t WaitGPIOReset(uint32_t Portx, uint32_t Pin)
{
	uint32_t i;
 	for (i = 0; i < 20000; i++)
	{
		if (gpio_output_bit_get(Portx, Pin) == RESET)
		{
			return (TRUE);
		}
	}
	return (FALSE);
}
/***********************************************************************************************
** �� �� ����	RfidRxRegInit
** ����������	����CC1101�Ĵ���ΪRXģʽ
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void RfidRxRegInit(void)
{
    uint8_t u8FifoThr = s_u8RFAttenuationCfgTab[st_RFChipPar.u8RFAtnIdx] + s_u8RFCReg_FIFOTHR_BIT6[g_u8RFVelocityIdx];
    delay_1us(10);	
	RfidWriteReg(CC1101_FIFOTHR, u8FifoThr);
   delay_1us(10);
	//RfidWriteReg(CC1101_FIFOTHR, s_aucRFAttenuationCfgTab[s_tRFChipPar.ucRFAtnIdx]);  
	RfidWriteReg(CC1101_AGCCTRL2, 0xC7);
   delay_1us(10);
	RfidWriteReg(CC1101_AGCCTRL1, 0x00);
   delay_1us(10);
	RfidWriteReg(CC1101_MCSM0, 0x18);
  delay_1us(10);
	RfidWriteReg(CC1101_WORCTRL, 0xF8);
  delay_1us(10);	
	RfidWriteReg(CC1101_MCSM2, 0x07);
  delay_1us(10);
}
/***********************************************************************************************
** �� �� ����	RfidWORRegInit
** ����������	����CC1101�Ĵ���ΪRXģʽ��Ӧ����
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void RfidWORRegInit(void)
{
	uint8_t u8MCSM2;
	
#define WOR_1200_MS
    uint8_t u8FifoThr = s_u8RFAttenuationCfgTab[st_RFChipPar.u8RFAtnIdx] + s_u8RFCReg_FIFOTHR_BIT6[g_u8RFVelocityIdx];
    RfidWriteReg(CC1101_FIFOTHR, u8FifoThr);
   delay_1us(10);
	//RfidWriteReg(CC1101_FIFOTHR, s_aucRFAttenuationCfgTab[s_tRFChipPar.ucRFAtnIdx] );
    RfidWriteReg(CC1101_AGCCTRL2, 0x03);
   delay_1us(10);
	RfidWriteReg(CC1101_AGCCTRL1, 0x40);
   delay_1us(10);
	RfidWriteReg(CC1101_MCSM0, 0x30);
    //RfidWriteReg(CC1101_MCSM0, 0x10);
   delay_1us(10);
	RfidWriteReg(CC1101_WORCTRL, 0x18);
   delay_1us(10);
	u8MCSM2 = s_u8RFCReg_MCSM2[g_u8RFVelocityIdx];
   
	RfidWriteReg(CC1101_MCSM2, u8MCSM2);//0x16
#ifdef WOR_1200_MS
  delay_1us(10);
   RfidWriteReg(CC1101_WOREVT1, 0xA2);
  delay_1us(10); 
  RfidWriteReg(CC1101_WOREVT0, 0x80);
  delay_1us(10);
#else
    RfidWriteReg(CC1101_WOREVT1, 0x87);
    RfidWriteReg(CC1101_WOREVT0, 0x6B);
#endif
}

/***********************************************************************************************
** �� �� ����	RfidConfig()
** ����������	CC1101��������
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void RfidConfig(/*const*/ Tag_RF_CONFIG* ptRfConfig, const uint8_t u8RfPaTable)
{
	uint8_t u8Idx = 0;
    /*const*/ uint8_t *pu8RfConfig = (/*const*/ uint8_t *)ptRfConfig;

    switch (st_RFChipPar.u8RFVelocity)
    {
      case 38:/*38.4k*/
      {
		g_u8RFVelocityIdx = 1;
        break;
      }
      default:
        break;     
    }

    if (g_u8RFVelocityIdx) /*��250k��Ҫ�޸ļĴ���*/
    {
        *(pu8RfConfig + 5) = s_u8RFCReg_MDMCFG4[g_u8RFVelocityIdx]; /*MDMCFG4*/
        *(pu8RfConfig + 6) = s_u8RFCReg_MDMCFG3[g_u8RFVelocityIdx]; /*MDMCFG3*/
        *(pu8RfConfig + 11) = s_u8RFCReg_DEVIATN[g_u8RFVelocityIdx]; /*DEVIATN*/
        *(pu8RfConfig + 20) = s_u8RFCReg_FSCAL3[g_u8RFVelocityIdx];/*FSCAL3*/
        *(pu8RfConfig + 25) = s_u8RFCReg_TEST2[g_u8RFVelocityIdx];/*TEST2*/
        *(pu8RfConfig + 26) = s_u8RFCReg_TEST1[g_u8RFVelocityIdx];/*TEST1*/
    }
    
    for(u8Idx = 0; u8Idx < 34; u8Idx++)
    {
        RfidWriteReg(st_RfConfigIndex[u8Idx],  pu8RfConfig[u8Idx]);
    }
    RfidWriteReg(CC1101_PATABLE | CC1101_WRITE_BURST, u8RfPaTable);
}
/***********************************************************************************************
** �� �� ����	uint8_t B0itReverse(uint8_t byte)
** ����������	����һ���ֽڣ��ֽ����ݷ�ת�����磺����01010101�����10101010
** �䡡  �룺	�ֽ�
** �䡡  ����	�ֽ����ݷ�ת
** ����  �ߣ�	�̺���
** �ա�  �ڣ�	2016.7.12
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
#if	WL_SPI_TYPE == 2	//=1�����ؼ������ߺ��⣨SPI�ӿڣ�
uint8_t BitReverse(uint8_t byte)
{
	byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
	byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
	byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
	return byte;
}
#endif
/***********************************************************************************************
** �� �� ����	DrvRfChipReset
** ����������	
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void RfidChipReset(void)
{
//	uint8_t i;
    SpiResetChipSingle();
	delay_1us(10);
	RfidStrobe(CC1101_SRES);
	delay_1us(10);
}
/***********************************************************************************************
** �� �� ����	DrvSpiResetChipSingle
** ����������	
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t SpiResetChipSingle(void)
{
    /* оƬƬѡ�ź� */
	RFID_MCU_SCK_SET;
	RFID_MCU_SI_RESET;
    RFID_MCU_CS_DEASSERT;
    delay_1us(30);
    RFID_MCU_CS_ASSERT;
    delay_1us(100);
    RFID_MCU_CS_DEASSERT;
    delay_1us(45);
    return RET_OK;
}


/*!
    \brief      use spi bus write a byte to sd card
    \param[in]  data: data to write
    \param[out] none
    \retval     the data read from the SPI bus
*/
uint8_t spi_write(uint8_t data)
{
	uint8_t value = 0;
    while(RESET == spi_i2s_flag_get(SPI0,SPI_FLAG_TBE));
    /* send the data */
    spi_i2s_data_transmit(SPI0, data);
    while(RESET == spi_i2s_flag_get(SPI0,SPI_FLAG_RBNE));
    /* return the data read from the SPI bus */ 
    value =  spi_i2s_data_receive(SPI0);
	return value;
}

///*!
//    \brief      use spi bus read a byte from sd card
//    \param[in]  none
//    \param[out] none
//    \retval     the data read from the SPI bus
//*/
//uint8_t spi_read(void)
//{
//    uint8_t value = 0;
//    while(RESET == spi_i2s_flag_get(SPI0,SPI_FLAG_TBE));
//    /* send the dummy byte to generate clock */
//    spi_i2s_data_transmit(SPI0, SD_DUMMY_BYTE);
//    while(RESET == spi_i2s_flag_get(SPI0,SPI_FLAG_RBNE));
//    /* get the data from the SPI bus */
//    value = spi_i2s_data_receive(SPI0);
//    /* return the data value*/
//    return value;
//}
/***********************************************************************************************
** �� �� ����	DrvSpiResetChipSingle
** ����������	
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
 uint8_t SpiMcuRead(uint8_t u8Addr, uint8_t* pu8Data, uint16_t u16Length)
{
	uint8_t buf;
	uint16_t u16Index;
    uint8_t u8Status;
#if OS_CRITICAL_METHOD == 3u
    OS_CPU_SR  cpu_sr = 0u;
#endif
    /*
     * SPI׼������
     */
	//RFID_SPI_MCU_BEGIN;
	RFID_MCU_CS_ASSERT;
	delay_1us(50);		//delay_1us(300);
	u8Status = RFID_SPI_MCU_WAIT_MISO;
	if (u8Status == FALSE)
		return(TIME_OVER);
//	OS_ENTER_CRITICAL();							//��ֹȫ���ж�
	spi_write(u8Addr);
//	OS_EXIT_CRITICAL();								//�ظ�ȫ���ж�
	delay_1us(10);
    /*
     *  ������
     */
	
	for (u16Index = 0; u16Index < u16Length; u16Index++)
    {

		buf = (uint8_t) spi_write(0);
		(pu8Data[u16Index]) = buf;
		delay_1us(10);
    }
	
    /*
     *  SPI�����ݽ���
     */
    RFID_SPI_MCU_END;
	delay_1us(10);
    /*
     *  ���ض�д�Ƿ�ɹ�״̬
     */
    return(u8Status);
}
/***********************************************************************************************
** �� �� ����	RfidReadStatusReg
** ����������	
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ע�����	��ͨ��SPI��ȡ�ӿڶ�ȡһ��״̬�Ĵ�����ʱ��ͬʱ��ƵӲ��Ҳ�ڸ��¼Ĵ������ͻ���һ����С
**				�ģ����޵Ŀ��ܵ��½������ȷ��CC1100��CC2500��������ж�˵������һ���⣬���Ƽ��˼���
**				���ʵĹ�����
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidReadStatusReg(uint8_t u8Addr)
{
    uint8_t u8Reg;
	
    SpiMcuRead(u8Addr | CC1101_READ_BURST, &u8Reg, 1);
	
    return(u8Reg);
}
/***********************************************************************************************
** �� �� ����	RfidSpiWrite
** ����������	��CC1101д������
** �䡡  �룺	uint8_t u8Addr��д����׵�ַ
**				const uint8_t* pu8Data��д�����ݵĵ�ַ
**				uint16_t u16Length��д�����ݳ���	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidSpiWrite(uint8_t u8Addr, const uint8_t* pu8Data, uint16_t u16Length)
{
	uint8_t u8RetSta;
	
    u8RetSta = SpiMcuWrite(u8Addr, pu8Data, u16Length);

    return (u8RetSta);
}
/***********************************************************************************************
** �� �� ����	SpiMcuWrite
** ����������	��CC1101д������
** �䡡  �룺	uint8_t u8Addr��д����׵�ַ
**				const uint8_t* pu8Data��д�����ݵĵ�ַ
**				uint16_t u16Length��д�����ݳ���
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
 uint8_t SpiMcuWrite(uint8_t u8Addr, const uint8_t* pu8Data, uint16_t u16Length)
{
    uint16_t u16Index;
    uint8_t u8Status;
#if OS_CRITICAL_METHOD == 3u
    OS_CPU_SR  cpu_sr = 0u;
#endif
    //RFID_SPI_MCU_BEGIN;

	RFID_MCU_CS_ASSERT;
	delay_1us(50);
	u8Status = RFID_SPI_MCU_WAIT_MISO;
	if (u8Status == FALSE)
		return(TIME_OVER);
//	OS_ENTER_CRITICAL();							//��ֹȫ���ж�
	spi_write(u8Addr);
//	OS_EXIT_CRITICAL();		
	delay_1us(10);
//	OS_ENTER_CRITICAL();							//��ֹȫ���ж�
	
	for (u16Index = 0; u16Index < u16Length; u16Index++)
    {
		spi_write(pu8Data[u16Index]);
//		spi_write((pu8Data[u16Index]));
		delay_1us(10);
   }	
	RFID_SPI_MCU_END;
   delay_1us(10);
    return(u8Status);
}
/***********************************************************************************************
** �� �� ����	RfidStrobe
** ����������	��CC1101���� strobe commands,����״̬
** �䡡  �룺	u8StrobeCmd��Ҫ���͵�����
** �䡡  ����	u8Status, ����״̬��Ϣ
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidStrobe(uint8_t u8StrobeCmd)
{
	uint8_t u8Status;
	#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
	OS_CPU_SR  cpu_sr = 0;
	#endif


    if (CC1101_STX == u8StrobeCmd)
		vu32CC1100State = 1;
	else if(CC1101_SRX == u8StrobeCmd)
		vu32CC1100State = 2;
	else
		vu32CC1100State = 0;
    //RFID_SPI_MCU_BEGIN;
	RFID_MCU_CS_ASSERT;
	delay_1us(50);		//delay_1us(300);
	u8Status = RFID_SPI_MCU_WAIT_MISO;
	if (u8Status == FALSE)
	{
		return(TIME_OVER);
	}

	spi_write(u8StrobeCmd);
	delay_1us(50);
//    u8Status = spi_i2s_data_receive(SPI0);

	RFID_SPI_MCU_END;
	delay_1us(10);
    return(u8Status);
}
/***********************************************************************************************
** �� �� ����	RFID_SpiPortInit
** ����������	
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void RfidReadAllStatusReg(uint8_t* pu8Data, uint16_t u16Length)
{
	uint8_t u8i;
	
	for(u8i = 0; u8i < u16Length; u8i++)
	{
		pu8Data[u8i] = RfidReadStatusReg(StatusRegBaseAddress + u8i);
		delay_1us(1);
	}
}
/***********************************************************************************************
** �� �� ����	RfidWriteReg
** ����������	��CC1101оƬд�Ĵ�����ֵ
** �䡡  �룺	uint8_t u8Addr��д��Ĵ�����ַ��
** 				uint8_t u8Data��д�����ݣ�
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidWriteReg(uint8_t u8Addr, uint8_t u8Data)
{
    return(RfidSpiWrite(u8Addr, &u8Data, 1));
}
/***********************************************************************************************
** �� �� ����	RfidWriteFifo
** ����������	��TX-FIFOд��ָ�����ȵ�����
** �䡡  �룺	const uint8_t *pu8Data��д������Ҫ����ĵ�ַ��
** 				uint8_t u8Length��д�����ݳ��ȣ�
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidWriteFifo(const uint8_t *pu8Data, uint8_t u8Length)
{
    return(SpiMcuWrite(CC1101_TXFIFO | CC1101_WRITE_BURST, pu8Data, u8Length));
}
/***********************************************************************************************
** �� �� ����	RfidReadFifo
** ����������	��RX-FIFO��ȡָ�����ȵ�����
** �䡡  �룺	uint8_t *pu8Data����ȡ����Ҫ����ĵ�ַ��
** 				uint8_t u8Length����ȡ�����ݳ��ȣ�
** �䡡  ����	����״̬��Ϣ
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidReadFifo(uint8_t *pu8Data, uint8_t u8Length)
{
    return(SpiMcuRead(CC1101_RXFIFO | CC1101_READ_BURST, pu8Data, u8Length));
}
/***********************************************************************************************
** �� �� ����	RFCtrlSetIDLE
** ����������	
** �䡡  ����	����״̬��Ϣ
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RFCtrlSetIDLE(void)
{
    uint16_t u16TimeoutCnt = 0;
	
    uint8_t u8ChipStatus;
	
    /* 
	 * ��֤���ͽ�������֮ǰ�Ǵ���IDLE״̬
	 */
    RfidStrobe(CC1101_SIDLE);
    do
    {
        u8ChipStatus = RfidReadStatusReg(CC1101_MARCSTATE);
        u16TimeoutCnt ++;        
        delay_1us(10);
        if(u16TimeoutCnt > MAX_STATUS_DELAY)
        {
            return(CHIP_STATUS_ABNORMAL);
        }
    }while(u8ChipStatus != CC1101_MARCSTATE_IDLE);
    return RET_OK;
}

/***********************************************************************************************
** �� �� ����	SetRFChipIDLE
** ����������	��λ��ֱ��������ƵоƬ״̬ΪIDLE
** �䡡  �룺	uint8_t u8ProcPreSet��Ԥ����״̬��
** 				uint8_t u8ClearUpMode��Ҫ�����״̬��
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t SetRFChipIdle(uint8_t u8ProcPreSet, uint8_t u8ClearUpMode)
{
    uint8_t u8RetVal = 0;
	
    /*
	 * ����ǰ��λRFChip�����SIDLE
	 */
    if (RFCHIP_RESET_NECESSARILY == u8ProcPreSet)
    {
        RFID_Init();
		delay_1us(10);
        RfidStrobe(CC1101_SIDLE);/* �˳�RX/TX���ص�Ƶ�ʷ����� */
		delay_1us(10);
		RfidStrobe(CC1101_SCAL);
		delay_1us(10);
    }
    else
    {
        u8RetVal = RFCtrlSetIDLE();
        if(RET_OK != u8RetVal)
        {
             return CHIP_STATUS_ABNORMAL;
        }
    }
	
    if (RFCHIP_TX_CLEARUP == u8ClearUpMode)
    {
        RfidStrobe(CC1101_SFTX);    // ��ϴ TX FIFO buffer.
		//delay_1us(10);
    }
    else
    {
        RfidStrobe(CC1101_SFRX);    // ��ϴ RX FIFO buffer.
		//delay_1us(10);
    }
    
    return RET_OK;
}
/***********************************************************************************************
** �� �� ����	RFTxSendPacket
** ����������	��CC1101оƬд���ݰ�
** �䡡  �룺	uint8_t* pu8Data��д�����ݵ�ַ��
** 				uint8_t u8Length��д�����ݳ��ȣ�
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RFTxSendPacket(uint8_t* pu8Data, uint8_t u8Length)
{	
	// 	uint32_t u32i;
	//static uint8_t u8LastLen=0;
	uint32_t u32i = 0x00;
	#if (DEFAULT_CCA_USE_FLAG == CC1101_CCA_ENABLED)
		uint8_t u8ChipStatus = 0x00;
		uint16_t u16TimeoutCnt= 0x00;
	#endif
	#if (OS_CRITICAL_METHOD == 3)                      /* Allocate storage for CPU status register           */
		OS_CPU_SR  cpu_sr = 0;
	#endif

    if (u8Length >= CC1101_TX_FIFO_SIZE)
    {
        return (TX_LENGTH_ERR);
    }
    
//	OS_ENTER_CRITICAL();//��ȫ���ж�
    /* ��֤���ͽ�������֮ǰ�Ǵ���IDLE״̬*/
    if(RET_OK != SetRFChipIdle(RFCHIP_IDLE_POSITIVE, RFCHIP_TX_CLEARUP))
    {
//		OS_EXIT_CRITICAL();//��ȫ���ж�
        return CHIP_STATUS_ABNORMAL;
    }
	delay_1us(10);

	#if (DEFAULT_CCA_USE_FLAG == CC1101_CCA_ENABLED)
		RfidStrobe(CC1101_SFTX);//Flush the TX FIFO buffer
		delay_1us(10);     
	#endif	                       
	vu32GDO0IntFlag = 0;
	// ������ݰ����ȷ����仯������д������ȼĴ���
	//if((u8LastLen != u8Length) && u8Length)
	//{
		RfidWriteReg(CC1101_PKTLEN,u8Length);
	//	u8LastLen = u8Length;
		delay_1us(40);
	//}
	// ��TX FIFO��д�����ݣ�ֻ���ڿ���״̬�ſ���
    RfidWriteFifo(pu8Data, u8Length+1);
	
	#if (DEFAULT_CCA_USE_FLAG != CC1101_CCA_ENABLED)
		delay_1us(10);
	#endif
	
#ifdef PAC_PORT
	//ʹ�ܸ߹��ʷ���
	gpio_bit_set(PAC_PORT, PAC_PIN);
#endif

	#if (DEFAULT_CCA_USE_FLAG == CC1101_CCA_ENABLED)
		RfidStrobe(CC1101_SRX);
		u16TimeoutCnt = 0x00;
		while(u16TimeoutCnt <= 250)
		{			
			u8ChipStatus = RfidReadStatusReg(CC1101_MARCSTATE);
			if(u8ChipStatus == CC1101_MARCSTATE_RX)
			{
				break;
			}
			u16TimeoutCnt++;
			delay_1us(1);
		}
		if(u16TimeoutCnt == 251)
		{
//			OS_EXIT_CRITICAL();//��ȫ���ж�
			return(CHIP_STATUS_ABNORMAL);
		}		
		
		u16TimeoutCnt = 0x00;
		while(u16TimeoutCnt <= 10)
		{
			// ����CC1101�ķ���״̬	
			RfidStrobe(CC1101_STX);	
			u8ChipStatus = RfidReadStatusReg(CC1101_MARCSTATE);
			if(u8ChipStatus == CC1101_MARCSTATE_TX)	
			{
				break;
			}
//			if(u8ChipStatus == CC1101_MARCSTATE_TXFIFO_UNDERFLOW)	
//				RfidStrobe(CC1101_SFTX);
			u16TimeoutCnt++;
			delay_1us(1);	
		}
		delay_1us(10);		
//		OS_EXIT_CRITICAL();//��ȫ���ж�
		
		// �ж�һ�������Ƿ������
		u32i = 0;
		while(u32i++<0x20000)
		{
			if(vu32GDO0IntFlag)
			{
				vu32GDO0IntFlag = 0;
				delay_1us(10);
				break;
			}
		}
		RfidStrobe(CC1101_SFTX);
		if (u32i<0x20000)
		{
			return TX_OK;
		}
	#else
		// ����CC1101�ķ���״̬
	    RfidStrobe(CC1101_STX);
		delay_1us(10);

		OS_EXIT_CRITICAL();//��ȫ���ж�	
		// �ж�һ�������Ƿ������
		u32i = 0;
		while(u32i++<0x20000)
		{
			if(vu32GDO0IntFlag)
			{
				vu32GDO0IntFlag = 0;
				delay_1us(10);
				break;
			}
		}
		RfidStrobe(CC1101_SFTX);
		if (u32i<0x20000)
		{
			return TX_OK;
		}	
	#endif

	return(TIME_OVER);
}
/***********************************************************************************************
** �� �� ����	RfidReadFifoProc
** ����������	��CC1101 FIFO����ȡһ�����������ݰ�
** �䡡  �룺	uint8_t *pu8Data�����뱣��������ݵĵ�ַ��
** 				uint8_t *pu8Length�����뱣��������ݳ��ȵĵ�ַ��
** �䡡  ����	return 	RX_CRC_MISMATCH����ʾCRCУ�鲻��ȷ��
**						RX_OK,��ʾ�������ݳɹ�
**						ע�⣬���յ����ݻᱣ������Ӧ�Ļ��棬�Լ����ݳ��ȣ�*pu8Length = �������� + 1(*pu8Length����ռ��һ���ֽ�)
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidReadFifoProc(uint8_t *pu8Data, uint8_t *pu8Length)
{
#if 1
    uint8_t u8DataLen;
// 	uint8_t u8CurrentRxBytes, u8LastRxBytes,u8LastDataLen;
    uint8_t u8AppendStatus[2];
	delay_1us(1);
	//��ȡ����
	RfidReadFifo(&u8DataLen, 1);				// ��ȡ���ݰ������ֽ�
	delay_1us(1);
	RfidReadFifo(pu8Data, u8DataLen);			// ��ȡ��������	
	delay_1us(1);
	//*pu8Length = u8DataLen;					// �������ݳ���	
	RfidReadFifo(u8AppendStatus, 2);			// CRCУ��
	delay_1us(1);
	
	pu8Data[u8DataLen] = u8AppendStatus[0];
	pu8Data[u8DataLen + 1] = u8AppendStatus[1];
	
	*pu8Length = u8DataLen + 2;					// �������ݳ���
	// ���������ݣ�Ҳ������ս��ջ��棬ֻ����SIDLE��SRX�����ʱ������
	//delay_1us(50);
	RfidStrobe(CC1101_SIDLE);
	delay_1us(1);
	RfidStrobe(CC1101_SFRX);
	delay_1us(1);
	SetRxMode();
	
	// �ж�CRCУ���Ƿ���ȷ������ȷ����RX_CRC_MISMATCH;���򷵻�RX_OK;
    if ((u8AppendStatus[1] & CC1101_LQI_CRC_OK_BM) != CC1101_LQI_CRC_OK_BM)
    {
        return(RX_CRC_MISMATCH);
    }
    return(RX_OK);
#endif // 0
}
/***********************************************************************************************
** �� �� ����	RfidGetTxStatus
** ����������	This function transmits a No Operation Strobe (SNOP) to get the status of
**				the radio and the number of free bytes in the TX FIFO
**				״̬�ֽڣ�
**      		---------------------------------------------------------------------------
**      		|          |            |                                                 |
**      		| CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
**      		|          |            |                                                 |
**      		---------------------------------------------------------------------------				
** �䡡  �룺	
** �䡡  ����	
** ע�����	������Ӳ�����ڸ��¼Ĵ�����ʱ��ͬʱ����ͨ��SPI�ӿڶ�ȡ״̬�Ĵ�����������һ���ǳ�С�ģ�
**				���޵Ŀ��ܵ��½���ǲ���ȷ�ġ�ͬ�������Ҳ����оƬ״̬�ֽڡ�CC1100��CC2500���������
**				������������⣬���ṩ�˼��ֹ�����
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidGetTxStatus(void)
{
    return(RfidStrobe(CC1101_SNOP));
}

//---------------------------------------------------------------------------------------------
//---			CC1101������ز���
//---------------------------------------------------------------------------------------------
/***********************************************************************************************
** �� �� ����	SetRFChipSleep
** ����������	������ƵоƬ����
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRFChipSleep(void)
{
	
    RfidRxRegInit();
	//delay_1us(10);
    /*
     * ��CSn����ʱ������˯��ģʽ�����رյ�Դģʽ
     */
    RfidStrobe(CC1101_SPWD);
	//delay_1us(10);
}
/***********************************************************************************************
** �� �� ����	SetRFChipWOR
** ����������	������ƵоƬWOR
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRFChipWOR(void)
{
    RfidWORRegInit();
	
    /*
	 * ������ڲ��������붨ʱ���ѽ���״̬
	 */
    RfidStrobe(CC1101_SWORRST);    			// Reset real time clock.
	//delay_1us(10);
    RfidStrobe(CC1101_SWOR);    			// Start automatic RX polling sequence (Wake-on-Radio).
	//delay_1us(10);
}
/***********************************************************************************************
** �� �� ����	SetRFChipSleepMode
** ����������	������ƵоƬ˯��ģʽ
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRFChipSleepMode(void)
{
    uint16_t u16CCR;
	
    /*����ʵ���������CC1101����״̬*/
    if(g_u16RunningMode == OBUMODE_SLEEP)
    {
        SetRFChipSleep();
		//delay_1us(10);
		if (st_RFChipPar.u8ResIntervalTime < 3)
		{
			u16CCR = 125;				/*100ms*//* WOR_TIMER_CLK */
		}
		else
		{
			 /*Ŀǰ���52s��������������ACLK��Ƶ����*/
			u16CCR = st_RFChipPar.u8ResIntervalTime * 1250;/* WOR_TIMER_CLK */
		}
		//WORTimerStart(u16CCR);
    }
    else
    {
        SetRFChipWOR();
		//delay_1us(10);
		//g_wRunningMode = OBUMODE_SLEEP;
    }
}
/***********************************************************************************************
** �� �� ����	ClrRxFifo
** ����������	������ջ���
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void ClrRxFifo(void)
{	
    RfidStrobe(CC1101_SFRX);		//�������״̬
	delay_1us(1);
}
/***********************************************************************************************
** �� �� ����	SetRxMode
** ����������	EXTI INTERRUPT �����Ӻ���
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRxMode(void)
{	
	uint32_t	i;
	#if OS_CRITICAL_METHOD == 3                 /* Allocate storage for CPU status register           */
	OS_CPU_SR  cpu_sr = 0;
	#endif

//	OS_ENTER_CRITICAL();//��ȫ���ж�

#ifdef PAC_PORT
	//�رո߹��ʷ���
	gpio_bit_reset(PAC_PORT, PAC_PIN);
#endif
	
	RfidRxRegInit();
	for (i = 0;i < 20;)
		i++;
    RfidStrobe(CC1101_SRX);		//�������״̬	
	i = RfidReadStatusReg(CC1101_MARCSTATE);
	for (i = 0;i < 20;)
		i++;
//	OS_EXIT_CRITICAL();//��ȫ���ж�
}

/***********************************************************************************************
** �� �� ����	RFID_GetFrame
** ����������	�ӽ��ջ�����ȡ��һ�����ݰ�
** �䡡  �룺	
** �䡡  ����	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t RFID_GetFrame(st_RFIDRcvFrame *RfidRcvFrm)
{
	if(u32RcvFrmPoolCnt)
	{
		#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
		OS_CPU_SR  cpu_sr = 0;
		#endif

//		OS_ENTER_CRITICAL();//��ȫ���ж�
		*RfidRcvFrm = st_RFIDRcvFrmPool[u32RcvFrmPoolReadPtr++];
		u32RcvFrmPoolReadPtr %= RFID_RCV_FRM_SIZE;
		
		u32RcvFrmPoolCnt--;
//		OS_EXIT_CRITICAL();//��ȫ���ж�
		return RET_OK;
	}else
	{
		return RET_ERR;
	}
}
//---------------------------------------------------------------------------------------------
//---			EXTI �����Ӻ���
//---------------------------------------------------------------------------------------------

#if WL_SPI_TYPE == 1	//����ģ��ӿ�ʹ��USARTģ��SPI
/***********************************************************************************************
** �� �� ����	EXTI0_IRQHandler
** ����������	EXTI INTERRUPT �����Ӻ���
** �䡡  �룺	
** �䡡  ����	
************************************************************************************************/
void EXTI0_1_IRQHandler(void)
{
	/* Get the status of RFID_GDO0_EXTI_LINE */
	if (exti_interrupt_flag_get(GPIO_PIN_1) != RESET)
	{
		/* Clear the RFID_GDO0_EXTI_LINE pending flag */
		exti_interrupt_flag_clear(GPIO_PIN_1);
		if (vu32CC1100State == 1)
		{

#ifdef PAC_PORT
			//�رո߹��ʷ���
			gpio_bit_reset(PAC_PORT, PAC_PIN);
#endif

			vu32GDO0IntFlag = 1;
			vu32CC1100State = 2;			//��������Զ�ת�����״̬���Ĵ������ã�
		}
		else if (vu32CC1100State == 2)
		{
			uint8_t u8CurrentRxBytes;
			u8CurrentRxBytes = RfidReadStatusReg(CC1101_RXBYTES);
			// �������������������ս��ջ���
			if(u8CurrentRxBytes & 0x80)
			{
				RfidStrobe(CC1101_SFRX);	//������ջ�����
				RfidStrobe(CC1101_SRX);		//�������״̬	
			}
		}
	}
}
/***********************************************************************************************
** �� �� ����	EXTI3_IRQHandler
** ����������	EXTI INTERRUPT �����Ӻ���
** �䡡  �룺	
** �䡡  ����	
************************************************************************************************/
void EXTI4_15_IRQHandler(void)
{
	uint32_t u32i;
	st_RFIDRcvFrame RfidRcvFrmTmp;
	/* Get the status of RFID_GDO2_EXTI_LINE */
	if (exti_interrupt_flag_get(GPIO_PIN_4) != RESET)
	{
		/* Clear the RFID_GDO2_EXTI_LINE pending flag */
		exti_interrupt_flag_clear(GPIO_PIN_4);

		//��ȫ���жϣ�
		//OS_ENTER_CRITICAL();
		u32i = RfidReadFifoProc(u8RcvData, &u8RcvLength);
		//��ȫ���жϣ�
		//OS_EXIT_CRITICAL();
			
		if (u32i == RX_OK)
		{
#if (CC1101_ADDR_FILTER > 0)
			if (u8RcvLength > 3)
			{
				RfidRcvFrmTmp.u8DataLen = u8RcvLength - 3;		// u8RcvLength = 1��ַ�ֽ� + N�����ֽ� + 2��״̬�ֽ�
				RfidRcvFrmTmp.u8DestAddr = u8RcvData[0];
				for(u32i = 0;u32i < RfidRcvFrmTmp.u8DataLen && u32i < 64;u32i++)
				{
					RfidRcvFrmTmp.u8Data[u32i] = u8RcvData[u32i + 1];
				}
#else
			if (u8RcvLength > 2)
			{
				RfidRcvFrmTmp.u8DataLen = u8RcvLength - 2;		// u8RcvLength = N�����ֽ� + 2��״̬�ֽ�
				RfidRcvFrmTmp.u8DestAddr = 0x00;
				for(u32i = 0;u32i < RfidRcvFrmTmp.u8DataLen && u32i < 64;u32i++)
				{
					RfidRcvFrmTmp.u8Data[u32i] = u8RcvData[u32i];
				}
#endif	
				RfidRcvFrmTmp.u8AppendStatus[0] = u8RcvData[u8RcvLength-2];
				RfidRcvFrmTmp.u8AppendStatus[1] = u8RcvData[u8RcvLength-1];
			
				if (u32RcvFrmPoolCnt < RFID_RCV_FRM_SIZE)
				{
					// �����ݰ����뵽�������
					st_RFIDRcvFrmPool[u32RcvFrmPoolWritePtr++] = RfidRcvFrmTmp;
					u32RcvFrmPoolWritePtr %= RFID_RCV_FRM_SIZE;
					
					u32RcvFrmPoolCnt++;
					// ���յ����ݺ�Ļص�����
//					OSSemPost(RfidRxSem);	//RFID���յ�����
//					if (RcvedBackCallFunc != NULL)
//						(*RcvedBackCallFunc)();
					Wl_RxDataProc();
				}
			}
		}
	}
}
#endif

#endif	//#if WL_SPI_TYPE > 0
/*********************************������������޹�˾*************************************************************/

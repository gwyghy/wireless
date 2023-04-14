/*********************************************************************************************************************************
** 文件名:  rfid_cc1101.h
** 描　述:  RFID驱动模块实现文件
** 		
**			RFID模块中使用的一些技术参数说明：
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

#include "rfid_cc1101.h"
#include "rfid_driver.h"
#include "systick.h"
#include "usart.h"
#include "gd32f1x0_gpio.h"
#if WL_SPI_TYPE > 0	//电液控2.0有RFID

 uint32_t vu32GDO2IntFlag = 0;	// GDO上升沿触发标志，代表一包数据接收校验正确
 uint32_t vu32GDO0IntFlag = 0;	// GDO下降沿触发标志，代表一包数据发送完成
const uint8_t uc8Patabel[8] = {0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0};
#if 1
/*const*/ Tag_RF_CONFIG st_RfConfig = {
    0x0C,	//0x06,   // FSCTRL1   	频率合成器控制 1
    0x00,   // FSCTRL0   	频率合成器控制 0
    0x10,   // FREQ2     	频率控制字, 高字节
    0xA7,   // FREQ1     	频率控制字, 次高字节
    0x62,   // FREQ0     	频率控制字, 低字节

    0x2D,   // MDMCFG4   	调制配置
    0x3B,   // MDMCFG3   	调制配置
    0x03,   // MDMCFG2   	调制解调器配置
			// 				bit7 = 0,在解调之前不关闭数字直流滤波模块
			//				bit6-4 = 0,使用2-FSK调制格式
			//				bit3 = 0,失能曼切斯特编解码
			//				bit2-0 = 3,30/32同步字位检测
    0x22,   // MDMCFG1   	调制解调器配置
			//				bit7 = 0,失能前纠错码，只对固定数据长度适用
			//				bit6-4 = 010,序列字节个数为4
			//				bit1-0 = 10，通道空间的2位幂
    0xF8,   // MDMCFG0   	调制配置

    0x00,   // CHANNR    	通道号
    0x62,   // DEVIATN   	调制解调器偏差设置 (when FSK modulation is enabled).
    0xB6,   // FREND1    	前端RX 配置
    0x10,   // FREND0    	前端TX 配置
    0x18,   // MCSM0     	CC1101无线主控制逻辑状态配置

    0x1D,   // FOCCFG    	频率偏移补偿配置
    0x1C,   // BSCFG     	位同步配置
    0xC7,   // AGCCTRL2  	AGC 控制
    0x00,   // AGCCTRL1  	AGC 控制
    0xB0,   // AGCCTRL0  	AGC 控制

    0xEA,   // FSCAL3    	频率合成器校准
    0x2A,   // FSCAL2    	频率合成器校准
    0x00,   // FSCAL1    	频率合成器校准
    0x1F,   // FSCAL0    	频率合成器校准
    0x59,   // FSTEST    	频率合成器校准

    0x88,   // TEST2     	不同测试设置
    0x31,   // TEST1     	不同测试设置
    0x09,   // TEST0     	不同测试设置
    0x07,   // IOCFG2    	GDO2 输入管脚配置，用于数据接收，当接收的数据校验正确给MCU一个外部中断信号
    0x06,   // IOCFG0    	GDO0 输入管脚配置，用于数据发送，当发送完成后产生中断信号 
#if (CC1101_ADDR_FILTER > 0)
    0x0E,   // PKTCTRL1  	数据包自动控制寄存器1，0x00001100,
			//				bit3 = 1,CRC校验不正确自动冲刷RX FIFO功能失能
			//				bit2 = 1,数据后面再附加两个状态字节
			//				bit1-0 = 00，无地址检查； =10,地址检查，00为广播地址；
#else
	0x0C,
#endif
    0x45,   // PKTCTRL0  	数据包自动控制寄存器0，0x01000101，
			// 				bit6 = 1，数据加密开启；
			// 				bit5-4 = 00，正常数据收发模式；
			// 				bit2 = 1，CRC使能；
			// 				bit1-0 = 01，可变数据包长度模式,length 在sync后边第 1 个字节；
			
    0xFE,	//0x55,   // ADDR      	器件地址，默认0x00
    0x40,	//0x0E,   // PKTLEN    	数据包长度，length = 61;
};
#endif
#if 0
/*const*/ Tag_RF_CONFIG st_RfConfig = {
    0x0C,   // FSCTRL1   	频率合成器控制 1
    0x00,   // FSCTRL0   	频率合成器控制 0
    0x10,   // FREQ2     	频率控制字, 高字节
    0xA7,   // FREQ1     	频率控制字, 次高字节
    0x62,   // FREQ0     	频率控制字, 低字节

    0x2D,   // MDMCFG4   	调制配置
    0x3B,   // MDMCFG3   	调制配置
    0x03,   // MDMCFG2   	调制解调器配置
			// 				bit7 = 0,在解调之前不关闭数字直流滤波模块
			//				bit6-4 = 0,使用2-FSK调制格式
			//				bit3 = 0,失能曼切斯特编解码
			//				bit2-0 = 3,30/32同步字位检测
    0x22,   // MDMCFG1   	调制解调器配置
			//				bit7 = 0,失能前纠错码，只对固定数据长度适用
			//				bit6-4 = 010,序列字节个数为4
			//				bit1-0 = 10，通道空间的2位幂
    0xF8,   // MDMCFG0   	调制配置

    0x00,   // CHANNR    	通道号
    0x62,   // DEVIATN   	调制解调器偏差设置 (when FSK modulation is enabled).
    0x56,   // FREND1    	前端RX 配置
    0x10,   // FREND0    	前端TX 配置
    0x18,   // MCSM0     	CC1101无线主控制逻辑状态配置

    0x1D,   // FOCCFG    	频率偏移补偿配置
    0x1C,   // BSCFG     	位同步配置
    0xC7,   // AGCCTRL2  	AGC 控制
    0x00,   // AGCCTRL1  	AGC 控制
    0xB0,   // AGCCTRL0  	AGC 控制

    0xEA,   // FSCAL3    	频率合成器校准
    0x2A,   // FSCAL2    	频率合成器校准
    0x00,   // FSCAL1    	频率合成器校准
    0x1F,   // FSCAL0    	频率合成器校准
    0x59,   // FSTEST    	频率合成器校准

    0x88,   // TEST2     	不同测试设置
    0x31,   // TEST1     	不同测试设置
    0x09,   // TEST0     	不同测试设置
    0x07,   // IOCFG2    	GDO2 输入管脚配置，用于数据接收，当接收的数据校验正确给MCU一个外部中断信号
    0x06,   // IOCFG0    	GDO0 输入管脚配置，用于数据发送，当发送完成后产生中断信号 
#if (CC1101_ADDR_FILTER > 0)
    0x0E,   // PKTCTRL1  	数据包自动控制寄存器1，0x00001100,
			//				bit3 = 0,CRC校验不正确自动冲刷RX FIFO功能使能
			//				bit2 = 1,数据后面再附加两个状态字节
			//				bit1-0 = 00，无地址检查； =10,地址检查，00为广播地址；
#else
	0x0C,
#endif

#if (CC1101_PKTLEN_MODE > 0)
	0x44,
#else
    0x45,   // PKTCTRL0  	数据包自动控制寄存器0，0x01000101，
			// 				bit6 = 1，数据加密开启；
			// 				bit5-4 = 00，正常数据收发模式；
			// 				bit2 = 1，CRC使能；
			// 				bit1-0 = 01，可变数据包长度模式,length 在sync后边第 1 个字节；
#endif			
    0xFE,   // ADDR      	器件地址，默认0x00
    0x40,   // PKTLEN    	数据包长度，length = 61;
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
// 		数组常量声明
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
 * FIFOTHR.ADC_RETENTION - RX FIFO 和 TX FIFO 阈值
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
//Data Rate 寄存器配置
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

uint8_t u8RcvLength = 64;										// 用于存储从CC1101的RX FIFO中接收的数据的长度
uint8_t u8RcvData[CC1101_RX_FIFO_SIZE+8];						// 用于存储从CC1101的RX FIFO中接收的一包数据
#define RFID_RCV_FRM_SIZE			10						// 存储池长度
st_RFIDRcvFrame st_RFIDRcvFrmPool[RFID_RCV_FRM_SIZE];		// RFID接收数据存储池
uint32_t u32RcvFrmPoolWritePtr = 0;
uint32_t u32RcvFrmPoolReadPtr = 0;
uint32_t u32RcvFrmPoolCnt = 0;

 uint32_t vu32CC1100State;										// 1=CC1100处于发送状态
RCVED_BACK_CALL_FUNC RcvedBackCallFunc=NULL;				// 接收后应用层实现的回调函数



/***********************************************************************************************
** 函 数 名：	RFID_HarewreInit()
** 功能描述：	
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
//	OS_ENTER_CRITICAL();							//禁止全局中断
	RfidChipReset();
	delay_1us(10);
    /*
	 * 芯片配置
	 */ 	 
	RfidConfig(&st_RfConfig,s_u8RFOutputPowerCfgTab[st_RFChipPar.u8RFPwrIdx]);
//	OS_EXIT_CRITICAL();								//回复全局中断
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
** 函 数 名：	WaitGPIOReset
** 功能描述：	等待指定GPIO引脚Reset信号
** 输　  入：	port: GPIO PORT
**         ：	pin: GPIO PIN
** 输　  出：	FALSE=等待超时
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
** 函 数 名：	RfidRxRegInit
** 功能描述：	设置CC1101寄存器为RX模式
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
** 函 数 名：	RfidWORRegInit
** 功能描述：	设置CC1101寄存器为RX模式对应配置
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
** 函 数 名：	RfidConfig()
** 功能描述：	CC1101参数配置
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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

    if (g_u8RFVelocityIdx) /*非250k需要修改寄存器*/
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
** 函 数 名：	uint8_t B0itReverse(uint8_t byte)
** 功能描述：	输入一个字节，字节数据翻转，例如：输入01010101，输出10101010
** 输　  入：	字节
** 输　  出：	字节数据翻转
** 作　  者：	程海超
** 日　  期：	2016.7.12
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
#if	WL_SPI_TYPE == 2	//=1：本地集成无线红外（SPI接口）
uint8_t BitReverse(uint8_t byte)
{
	byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
	byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
	byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
	return byte;
}
#endif
/***********************************************************************************************
** 函 数 名：	DrvRfChipReset
** 功能描述：	
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
** 函 数 名：	DrvSpiResetChipSingle
** 功能描述：	
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t SpiResetChipSingle(void)
{
    /* 芯片片选信号 */
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
** 函 数 名：	DrvSpiResetChipSingle
** 功能描述：	
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
     * SPI准备就绪
     */
	//RFID_SPI_MCU_BEGIN;
	RFID_MCU_CS_ASSERT;
	delay_1us(50);		//delay_1us(300);
	u8Status = RFID_SPI_MCU_WAIT_MISO;
	if (u8Status == FALSE)
		return(TIME_OVER);
//	OS_ENTER_CRITICAL();							//禁止全局中断
	spi_write(u8Addr);
//	OS_EXIT_CRITICAL();								//回复全局中断
	delay_1us(10);
    /*
     *  读数据
     */
	
	for (u16Index = 0; u16Index < u16Length; u16Index++)
    {

		buf = (uint8_t) spi_write(0);
		(pu8Data[u16Index]) = buf;
		delay_1us(10);
    }
	
    /*
     *  SPI读数据结束
     */
    RFID_SPI_MCU_END;
	delay_1us(10);
    /*
     *  返回读写是否成功状态
     */
    return(u8Status);
}
/***********************************************************************************************
** 函 数 名：	RfidReadStatusReg
** 功能描述：	
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 注意事项：	当通过SPI读取接口读取一个状态寄存器的时候，同时射频硬件也在更新寄存器，就会有一个很小
**				的，有限的可能导致结果不正确。CC1100和CC2500错误纠正中都说明了这一问题，且推荐了几种
**				合适的工作区
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
** 函 数 名：	RfidSpiWrite
** 功能描述：	向CC1101写入数据
** 输　  入：	uint8_t u8Addr，写入的首地址
**				const uint8_t* pu8Data，写入数据的地址
**				uint16_t u16Length，写入数据长度	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
** 函 数 名：	SpiMcuWrite
** 功能描述：	向CC1101写入数据
** 输　  入：	uint8_t u8Addr，写入的首地址
**				const uint8_t* pu8Data，写入数据的地址
**				uint16_t u16Length，写入数据长度
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
//	OS_ENTER_CRITICAL();							//禁止全局中断
	spi_write(u8Addr);
//	OS_EXIT_CRITICAL();		
	delay_1us(10);
//	OS_ENTER_CRITICAL();							//禁止全局中断
	
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
** 函 数 名：	RfidStrobe
** 功能描述：	向CC1101发送 strobe commands,返回状态
** 输　  入：	u8StrobeCmd，要发送的命令
** 输　  出：	u8Status, 返回状态信息
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
** 函 数 名：	RFID_SpiPortInit
** 功能描述：	
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
** 函 数 名：	RfidWriteReg
** 功能描述：	向CC1101芯片写寄存器的值
** 输　  入：	uint8_t u8Addr，写入寄存器地址；
** 				uint8_t u8Data，写入数据；
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidWriteReg(uint8_t u8Addr, uint8_t u8Data)
{
    return(RfidSpiWrite(u8Addr, &u8Data, 1));
}
/***********************************************************************************************
** 函 数 名：	RfidWriteFifo
** 功能描述：	向TX-FIFO写入指定长度的数据
** 输　  入：	const uint8_t *pu8Data，写入数据要保存的地址；
** 				uint8_t u8Length，写入数据长度；
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidWriteFifo(const uint8_t *pu8Data, uint8_t u8Length)
{
    return(SpiMcuWrite(CC1101_TXFIFO | CC1101_WRITE_BURST, pu8Data, u8Length));
}
/***********************************************************************************************
** 函 数 名：	RfidReadFifo
** 功能描述：	从RX-FIFO读取指定长度的数据
** 输　  入：	uint8_t *pu8Data，读取数据要保存的地址；
** 				uint8_t u8Length，读取的数据长度；
** 输　  出：	返回状态信息
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidReadFifo(uint8_t *pu8Data, uint8_t u8Length)
{
    return(SpiMcuRead(CC1101_RXFIFO | CC1101_READ_BURST, pu8Data, u8Length));
}
/***********************************************************************************************
** 函 数 名：	RFCtrlSetIDLE
** 功能描述：	
** 输　  出：	返回状态信息
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RFCtrlSetIDLE(void)
{
    uint16_t u16TimeoutCnt = 0;
	
    uint8_t u8ChipStatus;
	
    /* 
	 * 保证发送接收命令之前是处于IDLE状态
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
** 函 数 名：	SetRFChipIDLE
** 功能描述：	复位或直接设置射频芯片状态为IDLE
** 输　  入：	uint8_t u8ProcPreSet，预处理状态；
** 				uint8_t u8ClearUpMode，要清除的状态；
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t SetRFChipIdle(uint8_t u8ProcPreSet, uint8_t u8ClearUpMode)
{
    uint8_t u8RetVal = 0;
	
    /*
	 * 设置前复位RFChip或进入SIDLE
	 */
    if (RFCHIP_RESET_NECESSARILY == u8ProcPreSet)
    {
        RFID_Init();
		delay_1us(10);
        RfidStrobe(CC1101_SIDLE);/* 退出RX/TX，关掉频率分析仪 */
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
        RfidStrobe(CC1101_SFTX);    // 冲洗 TX FIFO buffer.
		//delay_1us(10);
    }
    else
    {
        RfidStrobe(CC1101_SFRX);    // 冲洗 RX FIFO buffer.
		//delay_1us(10);
    }
    
    return RET_OK;
}
/***********************************************************************************************
** 函 数 名：	RFTxSendPacket
** 功能描述：	向CC1101芯片写数据包
** 输　  入：	uint8_t* pu8Data，写入数据地址；
** 				uint8_t u8Length，写入数据长度；
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
    
//	OS_ENTER_CRITICAL();//关全局中断
    /* 保证发送接收命令之前是处于IDLE状态*/
    if(RET_OK != SetRFChipIdle(RFCHIP_IDLE_POSITIVE, RFCHIP_TX_CLEARUP))
    {
//		OS_EXIT_CRITICAL();//开全局中断
        return CHIP_STATUS_ABNORMAL;
    }
	delay_1us(10);

	#if (DEFAULT_CCA_USE_FLAG == CC1101_CCA_ENABLED)
		RfidStrobe(CC1101_SFTX);//Flush the TX FIFO buffer
		delay_1us(10);     
	#endif	                       
	vu32GDO0IntFlag = 0;
	// 如果数据包长度发生变化，重新写入包长度寄存器
	//if((u8LastLen != u8Length) && u8Length)
	//{
		RfidWriteReg(CC1101_PKTLEN,u8Length);
	//	u8LastLen = u8Length;
		delay_1us(40);
	//}
	// 往TX FIFO中写入数据，只有在空闲状态才可以
    RfidWriteFifo(pu8Data, u8Length+1);
	
	#if (DEFAULT_CCA_USE_FLAG != CC1101_CCA_ENABLED)
		delay_1us(10);
	#endif
	
#ifdef PAC_PORT
	//使能高功率发射
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
//			OS_EXIT_CRITICAL();//开全局中断
			return(CHIP_STATUS_ABNORMAL);
		}		
		
		u16TimeoutCnt = 0x00;
		while(u16TimeoutCnt <= 10)
		{
			// 开启CC1101的发送状态	
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
//		OS_EXIT_CRITICAL();//开全局中断
		
		// 判断一包数据是否发送完成
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
		// 开启CC1101的发送状态
	    RfidStrobe(CC1101_STX);
		delay_1us(10);

		OS_EXIT_CRITICAL();//开全局中断	
		// 判断一包数据是否发送完成
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
** 函 数 名：	RfidReadFifoProc
** 功能描述：	读CC1101 FIFO，获取一个完整的数据包
** 输　  入：	uint8_t *pu8Data，传入保存接收数据的地址；
** 				uint8_t *pu8Length，传入保存接收数据长度的地址；
** 输　  出：	return 	RX_CRC_MISMATCH，表示CRC校验不正确；
**						RX_OK,表示接收数据成功
**						注意，接收的数据会保存在相应的缓存，以及数据长度；*pu8Length = 数据内容 + 1(*pu8Length本身占用一个字节)
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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
	//读取数据
	RfidReadFifo(&u8DataLen, 1);				// 读取数据包长度字节
	delay_1us(1);
	RfidReadFifo(pu8Data, u8DataLen);			// 读取数据内容	
	delay_1us(1);
	//*pu8Length = u8DataLen;					// 返回数据长度	
	RfidReadFifo(u8AppendStatus, 2);			// CRC校验
	delay_1us(1);
	
	pu8Data[u8DataLen] = u8AppendStatus[0];
	pu8Data[u8DataLen + 1] = u8AppendStatus[1];
	
	*pu8Length = u8DataLen + 2;					// 返回数据长度
	// 清空这包数据，也就是清空接收缓存，只有在SIDLE、SRX（溢出时）才能
	//delay_1us(50);
	RfidStrobe(CC1101_SIDLE);
	delay_1us(1);
	RfidStrobe(CC1101_SFRX);
	delay_1us(1);
	SetRxMode();
	
	// 判断CRC校验是否正确，不正确返回RX_CRC_MISMATCH;否则返回RX_OK;
    if ((u8AppendStatus[1] & CC1101_LQI_CRC_OK_BM) != CC1101_LQI_CRC_OK_BM)
    {
        return(RX_CRC_MISMATCH);
    }
    return(RX_OK);
#endif // 0
}
/***********************************************************************************************
** 函 数 名：	RfidGetTxStatus
** 功能描述：	This function transmits a No Operation Strobe (SNOP) to get the status of
**				the radio and the number of free bytes in the TX FIFO
**				状态字节：
**      		---------------------------------------------------------------------------
**      		|          |            |                                                 |
**      		| CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
**      		|          |            |                                                 |
**      		---------------------------------------------------------------------------				
** 输　  入：	
** 输　  出：	
** 注意事项：	当无线硬件正在更新寄存器的时候，同时我们通过SPI接口读取状态寄存器，将会有一个非常小的，
**				有限的可能导致结果是不正确的。同样的情况也存在芯片状态字节。CC1100和CC2500错误纠正中
**				解释了这个问题，并提供了几种工作区
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t RfidGetTxStatus(void)
{
    return(RfidStrobe(CC1101_SNOP));
}

//---------------------------------------------------------------------------------------------
//---			CC1101休眠相关操作
//---------------------------------------------------------------------------------------------
/***********************************************************************************************
** 函 数 名：	SetRFChipSleep
** 功能描述：	设置射频芯片休眠
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRFChipSleep(void)
{
	
    RfidRxRegInit();
	//delay_1us(10);
    /*
     * 当CSn拉高时，进入睡眠模式，即关闭电源模式
     */
    RfidStrobe(CC1101_SPWD);
	//delay_1us(10);
}
/***********************************************************************************************
** 函 数 名：	SetRFChipWOR
** 功能描述：	设置射频芯片WOR
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRFChipWOR(void)
{
    RfidWORRegInit();
	
    /*
	 * 经过入口操作，进入定时唤醒接收状态
	 */
    RfidStrobe(CC1101_SWORRST);    			// Reset real time clock.
	//delay_1us(10);
    RfidStrobe(CC1101_SWOR);    			// Start automatic RX polling sequence (Wake-on-Radio).
	//delay_1us(10);
}
/***********************************************************************************************
** 函 数 名：	SetRFChipSleepMode
** 功能描述：	设置射频芯片睡眠模式
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRFChipSleepMode(void)
{
    uint16_t u16CCR;
	
    /*根据实际情况设置CC1101工作状态*/
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
			 /*目前最大52s，如需增大设置ACLK分频即可*/
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
** 函 数 名：	ClrRxFifo
** 功能描述：	清除接收缓存
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
void ClrRxFifo(void)
{	
    RfidStrobe(CC1101_SFRX);		//进入接收状态
	delay_1us(1);
}
/***********************************************************************************************
** 函 数 名：	SetRxMode
** 功能描述：	EXTI INTERRUPT 服务子函数
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
void SetRxMode(void)
{	
	uint32_t	i;
	#if OS_CRITICAL_METHOD == 3                 /* Allocate storage for CPU status register           */
	OS_CPU_SR  cpu_sr = 0;
	#endif

//	OS_ENTER_CRITICAL();//关全局中断

#ifdef PAC_PORT
	//关闭高功率发射
	gpio_bit_reset(PAC_PORT, PAC_PIN);
#endif
	
	RfidRxRegInit();
	for (i = 0;i < 20;)
		i++;
    RfidStrobe(CC1101_SRX);		//进入接收状态	
	i = RfidReadStatusReg(CC1101_MARCSTATE);
	for (i = 0;i < 20;)
		i++;
//	OS_EXIT_CRITICAL();//开全局中断
}

/***********************************************************************************************
** 函 数 名：	RFID_GetFrame
** 功能描述：	从接收缓存中取出一个数据包
** 输　  入：	
** 输　  出：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
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

//		OS_ENTER_CRITICAL();//关全局中断
		*RfidRcvFrm = st_RFIDRcvFrmPool[u32RcvFrmPoolReadPtr++];
		u32RcvFrmPoolReadPtr %= RFID_RCV_FRM_SIZE;
		
		u32RcvFrmPoolCnt--;
//		OS_EXIT_CRITICAL();//开全局中断
		return RET_OK;
	}else
	{
		return RET_ERR;
	}
}
//---------------------------------------------------------------------------------------------
//---			EXTI 服务子函数
//---------------------------------------------------------------------------------------------

#if WL_SPI_TYPE == 1	//无线模块接口使用USART模拟SPI
/***********************************************************************************************
** 函 数 名：	EXTI0_IRQHandler
** 功能描述：	EXTI INTERRUPT 服务子函数
** 输　  入：	
** 输　  出：	
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
			//关闭高功率发射
			gpio_bit_reset(PAC_PORT, PAC_PIN);
#endif

			vu32GDO0IntFlag = 1;
			vu32CC1100State = 2;			//发送完毕自动转入接收状态（寄存器设置）
		}
		else if (vu32CC1100State == 2)
		{
			uint8_t u8CurrentRxBytes;
			u8CurrentRxBytes = RfidReadStatusReg(CC1101_RXBYTES);
			// 如果发生溢出，则可以清空接收缓存
			if(u8CurrentRxBytes & 0x80)
			{
				RfidStrobe(CC1101_SFRX);	//清除接收缓冲区
				RfidStrobe(CC1101_SRX);		//进入接收状态	
			}
		}
	}
}
/***********************************************************************************************
** 函 数 名：	EXTI3_IRQHandler
** 功能描述：	EXTI INTERRUPT 服务子函数
** 输　  入：	
** 输　  出：	
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

		//关全局中断？
		//OS_ENTER_CRITICAL();
		u32i = RfidReadFifoProc(u8RcvData, &u8RcvLength);
		//开全局中断？
		//OS_EXIT_CRITICAL();
			
		if (u32i == RX_OK)
		{
#if (CC1101_ADDR_FILTER > 0)
			if (u8RcvLength > 3)
			{
				RfidRcvFrmTmp.u8DataLen = u8RcvLength - 3;		// u8RcvLength = 1地址字节 + N数据字节 + 2个状态字节
				RfidRcvFrmTmp.u8DestAddr = u8RcvData[0];
				for(u32i = 0;u32i < RfidRcvFrmTmp.u8DataLen && u32i < 64;u32i++)
				{
					RfidRcvFrmTmp.u8Data[u32i] = u8RcvData[u32i + 1];
				}
#else
			if (u8RcvLength > 2)
			{
				RfidRcvFrmTmp.u8DataLen = u8RcvLength - 2;		// u8RcvLength = N数据字节 + 2个状态字节
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
					// 将数据包插入到缓存池中
					st_RFIDRcvFrmPool[u32RcvFrmPoolWritePtr++] = RfidRcvFrmTmp;
					u32RcvFrmPoolWritePtr %= RFID_RCV_FRM_SIZE;
					
					u32RcvFrmPoolCnt++;
					// 接收到数据后的回调函数
//					OSSemPost(RfidRxSem);	//RFID接收到数据
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
/*********************************天津华宁电子有限公司*************************************************************/

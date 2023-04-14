/*********************************************************************************************************************************
** 文件名:  rfid_driver.c
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

//#include <string>
#include "rfid_driver.h"
#include "gd32f1x0_gpio.h"
#if WL_SPI_TYPE > 0	//电液控2.0有RFID

extern RCVED_BACK_CALL_FUNC RcvedBackCallFunc;			// 接收后应用层实现的回调函数。定义在rfid_cc1101.c
extern uint8_t SC_RFID_ADDR;
uint8_t ReciveRSSIData = 0;		
void RfidSpiInit(void);
void RfidGDOxIntInit(void);
uint32_t RFID_FetchData(st_RFIDRcvFrame *RfidRcvFrm);
//----------------------------------------------------------------------------------------------------
// 对外提供的函数接口
//----------------------------------------------------------------------------------------------------
/***********************************************************************************************
** 函 数 名：	RFID_Init
** 功能描述：	从数据存储池中取出数据
** 输　  入：	void;
** 输　  出：	st_RFIDRcvFrame：一包数据
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
void RFID_Init(void)
{
	RfidSpiInit();
	
	__disable_irq();  
                                                                                                                                                             	RfidGDOxIntInit();
	RFID_HarewreInit();
	RfidGDOxIntInit();
	__enable_irq();  
	
//	__disable_fault_irq();                                                                                                                                                                    	RfidGDOxIntInit();
//	RFID_HarewreInit();
//	RfidGDOxIntInit();
//	__enable_fault_irq();
	//设置CC1100接收地址滤波器的值
#ifndef DYK_SS
	RfidWriteReg(CC1101_ADDR,SC_RFID_ADDR);
	if(SC_RFID_ADDR == 0)
	{
		;
	}
#else
	RfidWriteReg(CC1101_ADDR, SS_RFID_ADDR);
#endif
	
	//设置接收模式
	SetRxMode();
}

/********************************************************************************
**函数名称：RfidSpiConfig
**函数作用：配置RFID SPI总线的硬件接口（时钟、管脚），以及SPI总线的参数和电气特性
**函数参数：无
**函数输出：无
**注意事项：无
*********************************************************************************/
#if	WL_SPI_TYPE == 1	//=1：本地集成无线红外（SPI接口）
void RfidSpiConfig(void)
{
    spi_parameter_struct spi_init_struct;
	
    rcu_periph_clock_enable(RCU_SPI0);
	
	rcu_periph_clock_enable(RCU_GPIOA);

    /* GPIOB config, PB3(LCD_SPI_CLK), PB4(SPI0_MISO), PB5(LCD_SPI_MOSI) */
    gpio_af_set(GPIOA, GPIO_AF_0,GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5);
  
	spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);
    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);

    /* set crc polynomial */
    spi_crc_polynomial_set(SPI0, 7);
    spi_enable(SPI0);
}

#endif
/********************************************************************************
**函数名称：RfidUsartSpiConfig
**函数作用：配置RFIDSPI(USART2模拟)总线的硬件接口（时钟、管脚）和硬件参数
**函数参数：无
**函数输出：无
**注意事项：无
*********************************************************************************/
#if	WL_SPI_TYPE == 2	//=2：本地集成无线红外（USART模拟SPI接口）
void RfidUsartSpiConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;   
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
    RCC_ClocksTypeDef  rcc_clocks; 

	RCC_AHB1PeriphClockCmd(WL_GPIO_RCC, ENABLE);			//串口端口时钟使能
	RCC_RFID_AF_APBxCmd();

	/* IR Periph clock enable */
#if	WL_APB == APB1
 	RCC_APB1PeriphClockCmd(WL_RCC, ENABLE);					//串口时钟使能
#else
	RCC_APB2PeriphClockCmd(WL_RCC, ENABLE);					//串口时钟使能
#endif
		
	/* Configure WL_TX_PIN in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = WL_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(WL_PORT, &GPIO_InitStructure);

	/* Configure WL_RX_PIN in input no pull mode */
	GPIO_InitStructure.GPIO_Pin = WL_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(WL_PORT, &GPIO_InitStructure);

	/* Configure WL_CK_PIN in input no pull mode */
	GPIO_InitStructure.GPIO_Pin = WL_CK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(WL_PORT, &GPIO_InitStructure);

	//设置引脚复用功能
	GPIO_PinAFConfig(WL_PORT, WL_RX_SOURCE, WL_GPIO_AF);
	GPIO_PinAFConfig(WL_PORT, WL_TX_SOURCE, WL_GPIO_AF);
	GPIO_PinAFConfig(WL_PORT, WL_CK_SOURCE, WL_GPIO_AF);
	
    RCC_GetClocksFreq(&rcc_clocks);							//调用标准库函数，获取系统时钟。
	
	USART_InitStructure.USART_BaudRate = rcc_clocks.PCLK1_Frequency>>((RFID_SPI_BAUNDRATE_PRESCALER>>3)+1);	//与SPI接口一致
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;		//无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(WL_USART, &USART_InitStructure);

	/* Configure USART_ClockInitStruct members */
	USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInit(WL_USART, &USART_ClockInitStructure);
	
	USART_ITConfig(WL_USART, USART_IT_RXNE, DISABLE);		//中断禁止
	USART_ITConfig(WL_USART, USART_IT_TXE, DISABLE);
	USART_ITConfig(WL_USART, USART_IT_TC, DISABLE);

	USART_Cmd(WL_USART, ENABLE);							//串口使能
}
#endif
/********************************************************************************
**函数名称：RfidSpiInit
**函数作用：配置RFID SPI总线的硬件接口（时钟、管脚）和硬件参数
**函数参数：无
**函数输出：无
**注意事项：无
*********************************************************************************/
void RfidSpiInit(void)
{	
#if	WL_SPI_TYPE != 2	//=1：本地集成无线红外（SPI接口）
	RfidSpiConfig();
#else					//=2：本地集成无线红外（USART模拟SPI接口）
	RfidUsartSpiConfig();
#endif
}

/********************************************************************************
**函数名称：RfidGDOxIntInit
**函数作用：配置RFID的中断
**函数参数：无
**函数输出：无
**注意事项：无
*********************************************************************************/
void RfidGDOxIntInit(void)
{
	rcu_periph_clock_enable(RCU_CFGCMP);	
	rcu_periph_clock_enable(RCU_GPIOA);//GPIOA时钟使能
	rcu_periph_clock_enable(RCU_GPIOB);//GPIOA时钟使能
	gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
	gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
	nvic_irq_enable(EXTI0_1_IRQn, 3U,0U);
	nvic_irq_enable(EXTI4_15_IRQn, 2U,0U);
	/* connect key EXTI line to key GPIO pin */
	syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN4);
	syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN1);
	/* configure key EXTI line */
	exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
	exti_interrupt_flag_clear(EXTI_1);
	exti_init(EXTI_4, EXTI_INTERRUPT, EXTI_TRIG_RISING);
	exti_interrupt_flag_clear(EXTI_4);

}
/*****************************************************************
** 函数名：SetRcvedBackCallFunc
** 输　入：BackCallFunc：数据接收后的回调函数
** 输　出：无
** 功能描述：设置RFID数据接收后的回调函数
******************************************************************/
void SetRcvedBackCallFunc(RCVED_BACK_CALL_FUNC BackCallFunc)
{
	RcvedBackCallFunc = BackCallFunc;
}
/*****************************************************************
** 函数名：WL_ReceiveData
** 输　入：buf：数据接收缓冲区
**         num：接收字节数，不符合此值放弃
** 输　出：实际读出数据字节数
** 功能描述：读取接收的无线数据
******************************************************************/
uint32_t WL_ReceiveData(unsigned char *buf, unsigned int num)
{
	st_RFIDRcvFrame RxFrm;

	if (RFID_GetFrame(&RxFrm) == RET_OK)
	{
//		StateLed(LED_WL, 0);
		if ((RxFrm.u8DestAddr == ALL_RFID_ADDR 				//群发地址
#ifndef DYK_SS
			|| RxFrm.u8DestAddr == SC_RFID_ADDR	//本架地址
#else
			|| RxFrm.u8DestAddr == SS_RFID_ADDR				//本架地址
#endif
			) && (num >= RxFrm.u8DataLen))					//长度过滤
		{
			uint32_t i=RxFrm.u8DataLen;		//数据长度
			uint8_t *pt=RxFrm.u8Data;
			ReciveRSSIData = RxFrm.u8AppendStatus[0x00];
			if (i > 0)
			{
				memmove(buf, pt, i);	//取数据
				return (i);
			}
		}
	}else
//		StateLed(0, LED_WL);
	return(0);
}
/*****************************************************************
** 函数名：WL_SendData
** 输　入：buf：发送数据缓冲区
**         num：发送数据字节数
**         ToAddr：接收地址
** 输　出：实际发送字节数
** 功能描述：无线发射数据
******************************************************************/
uint32_t WL_SendData(unsigned char *buf, unsigned int num, unsigned int ToAddr)
{
	uint32_t Len=num;
	
	if (RFID_SendData(ToAddr, buf, &Len) == RET_OK)
		return(num);
	else
		return(0);
}
/*****************************************************************
** 功能：检查是否允许无线发射
** 输入：无
** 返回：TRUE:可以发送
******************************************************************/
uint32_t WlEmitEnabled(void)
{
	//if (GetRfidSendEnable() == RET_OK)
		return (1);
	//else
	//	return (FALSE);
}
/***********************************************************************************************
** 函 数 名：	RFID_FetchData
** 功能描述：	从数据存储池中取出数据
** 输　  入：	void;
** 输　  出：	st_RFIDRcvFrame：一包数据
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t RFID_FetchData(st_RFIDRcvFrame *RfidRcvFrm)
{
	return (RFID_GetFrame(RfidRcvFrm));
}
/***********************************************************************************************
** 函 数 名：	RFID_SendData
** 功能描述：	向FIFO中写入数据；
** 输　  入：	u8DestAddr,数据的接收地址；
**				pu8Buf，数据缓存指针；
**				u32Length，数据长度指针；
** 输　  出：	
** 返 回 值：	
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t RFID_SendData(uint8_t u8DestAddr,uint8_t *pu8Buf, uint32_t *u32Length)
{
	uint8_t pu8DataTmp[64],u8Length,u8Ret;
	uint32_t u32i;
	
#if (CC1101_ADDR_FILTER > 0)
	pu8DataTmp[1] = u8DestAddr;
	for(u32i = 2; u32i < (*u32Length + 2); u32i++)
	{
		pu8DataTmp[u32i] = pu8Buf[u32i-2];
	}
	u8Length = (uint8_t)(*u32Length) + 1;
#else
	for(u32i = 1; u32i < (*u32Length + 1); u32i++)
	{
		pu8DataTmp[u32i] = pu8Buf[u32i-1];
	}
	u8Length = (uint8_t)(*u32Length);
#endif
	pu8DataTmp[0] = u8Length;
		
	// 加入发送函数
	u8Ret = RFTxSendPacket(pu8DataTmp, u8Length);
	
	//if(u8Ret == TX_OK)
		//SetRxMode();			//进入接收模式
		//RFCtrlSetIDLE();
	SetRfidSRX();	
	
	return((uint32_t)u8Ret);
	
}
/***********************************************************************************************
** 函 数 名：	SetRfidSIDLE
** 功能描述：	设置CC1101进入空闲状态
** 输　  入：	无
** 输　  出：	
** 返 回 值：	返回状态
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t SetRfidSIDLE(void)
{
	return(RFCtrlSetIDLE());
}
/***********************************************************************************************
** 函 数 名：	SetRfidSRX
** 功能描述：	设置CC1101进入接收状态
** 输　  入：	无
** 输　  出：	
** 返 回 值：	返回状态
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t SetRfidSRX(void)
{
	SetRxMode();
	return 0;
}
/***********************************************************************************************
** 函 数 名：	GetRfidSendEnable
** 功能描述：	获取CC1101是否允许发送
** 输　  入：	无
** 输　  出：	
** 返 回 值：	RET_OK = 0:允许发送；RET_ERR = 1：不允许发送
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t GetRfidSendEnable(void)
{
	uint8_t u8ChipStatus;
	
	u8ChipStatus = RfidReadStatusReg(CC1101_PKTSTATUS);
	if(u8ChipStatus & 0x10)
		return RET_OK;
	else
		return RET_ERR;
}

/***********************************************************************************************
** 函 数 名：	GetRfidCurStatus
** 功能描述：	获取CC1101当前状态
** 输　  入：	无
** 输　  出：	
** 返 回 值：	返回状态
** 作　  者：	沈万江
** 日　  期：	2014.12.26
** 版    本：	V1.0.0
** 更新记录：
** 更新记录：
** 					日    期      姓    名                    描      述
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t GetRfidCurStatus(void)
{
	return(RfidGetTxStatus());
}
/***********************************************************************************************
** 函 数 名：	SetRxAddrFilter
** 功能描述：	设置CC1101接收地址滤波器的值
** 输　  入：	Addr:地址过滤器的值
** 返 回 值：	
************************************************************************************************/
uint8_t SetRxAddrFilter(uint8_t Addr)
{
	uint8_t i;
	#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
	    OS_CPU_SR  cpu_sr = 0;
	#endif
	
//	OS_ENTER_CRITICAL(); 	
#ifndef DYK_SS
    i = RfidWriteReg(CC1101_ADDR,  Addr);
#else
    i = RfidWriteReg(CC1101_ADDR,  0xff);
#endif
// 	OS_EXIT_CRITICAL(); 	
    return(i);
}





#endif	//#if WL_SPI_TYPE > 0
/*********************************天津华宁电子有限公司*************************************************************/

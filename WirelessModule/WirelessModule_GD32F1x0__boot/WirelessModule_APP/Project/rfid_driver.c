/*********************************************************************************************************************************
** �ļ���:  rfid_driver.c
** �衡��:  RFIDģ������ṩ�Ľӿ�
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

//#include <string>
#include "rfid_driver.h"
#include "gd32f1x0_gpio.h"
#if WL_SPI_TYPE > 0	//��Һ��2.0��RFID

extern RCVED_BACK_CALL_FUNC RcvedBackCallFunc;			// ���պ�Ӧ�ò�ʵ�ֵĻص�������������rfid_cc1101.c
extern uint8_t SC_RFID_ADDR;
uint8_t ReciveRSSIData = 0;		
void RfidSpiInit(void);
void RfidGDOxIntInit(void);
uint32_t RFID_FetchData(st_RFIDRcvFrame *RfidRcvFrm);
//----------------------------------------------------------------------------------------------------
// �����ṩ�ĺ����ӿ�
//----------------------------------------------------------------------------------------------------
/***********************************************************************************************
** �� �� ����	RFID_Init
** ����������	�����ݴ洢����ȡ������
** �䡡  �룺	void;
** �䡡  ����	st_RFIDRcvFrame��һ������
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
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
	//����CC1100���յ�ַ�˲�����ֵ
#ifndef DYK_SS
	RfidWriteReg(CC1101_ADDR,SC_RFID_ADDR);
	if(SC_RFID_ADDR == 0)
	{
		;
	}
#else
	RfidWriteReg(CC1101_ADDR, SS_RFID_ADDR);
#endif
	
	//���ý���ģʽ
	SetRxMode();
}

/********************************************************************************
**�������ƣ�RfidSpiConfig
**�������ã�����RFID SPI���ߵ�Ӳ���ӿڣ�ʱ�ӡ��ܽţ����Լ�SPI���ߵĲ����͵�������
**������������
**�����������
**ע�������
*********************************************************************************/
#if	WL_SPI_TYPE == 1	//=1�����ؼ������ߺ��⣨SPI�ӿڣ�
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
**�������ƣ�RfidUsartSpiConfig
**�������ã�����RFIDSPI(USART2ģ��)���ߵ�Ӳ���ӿڣ�ʱ�ӡ��ܽţ���Ӳ������
**������������
**�����������
**ע�������
*********************************************************************************/
#if	WL_SPI_TYPE == 2	//=2�����ؼ������ߺ��⣨USARTģ��SPI�ӿڣ�
void RfidUsartSpiConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;   
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
    RCC_ClocksTypeDef  rcc_clocks; 

	RCC_AHB1PeriphClockCmd(WL_GPIO_RCC, ENABLE);			//���ڶ˿�ʱ��ʹ��
	RCC_RFID_AF_APBxCmd();

	/* IR Periph clock enable */
#if	WL_APB == APB1
 	RCC_APB1PeriphClockCmd(WL_RCC, ENABLE);					//����ʱ��ʹ��
#else
	RCC_APB2PeriphClockCmd(WL_RCC, ENABLE);					//����ʱ��ʹ��
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

	//�������Ÿ��ù���
	GPIO_PinAFConfig(WL_PORT, WL_RX_SOURCE, WL_GPIO_AF);
	GPIO_PinAFConfig(WL_PORT, WL_TX_SOURCE, WL_GPIO_AF);
	GPIO_PinAFConfig(WL_PORT, WL_CK_SOURCE, WL_GPIO_AF);
	
    RCC_GetClocksFreq(&rcc_clocks);							//���ñ�׼�⺯������ȡϵͳʱ�ӡ�
	
	USART_InitStructure.USART_BaudRate = rcc_clocks.PCLK1_Frequency>>((RFID_SPI_BAUNDRATE_PRESCALER>>3)+1);	//��SPI�ӿ�һ��
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;		//��У��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(WL_USART, &USART_InitStructure);

	/* Configure USART_ClockInitStruct members */
	USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInit(WL_USART, &USART_ClockInitStructure);
	
	USART_ITConfig(WL_USART, USART_IT_RXNE, DISABLE);		//�жϽ�ֹ
	USART_ITConfig(WL_USART, USART_IT_TXE, DISABLE);
	USART_ITConfig(WL_USART, USART_IT_TC, DISABLE);

	USART_Cmd(WL_USART, ENABLE);							//����ʹ��
}
#endif
/********************************************************************************
**�������ƣ�RfidSpiInit
**�������ã�����RFID SPI���ߵ�Ӳ���ӿڣ�ʱ�ӡ��ܽţ���Ӳ������
**������������
**�����������
**ע�������
*********************************************************************************/
void RfidSpiInit(void)
{	
#if	WL_SPI_TYPE != 2	//=1�����ؼ������ߺ��⣨SPI�ӿڣ�
	RfidSpiConfig();
#else					//=2�����ؼ������ߺ��⣨USARTģ��SPI�ӿڣ�
	RfidUsartSpiConfig();
#endif
}

/********************************************************************************
**�������ƣ�RfidGDOxIntInit
**�������ã�����RFID���ж�
**������������
**�����������
**ע�������
*********************************************************************************/
void RfidGDOxIntInit(void)
{
	rcu_periph_clock_enable(RCU_CFGCMP);	
	rcu_periph_clock_enable(RCU_GPIOA);//GPIOAʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOB);//GPIOAʱ��ʹ��
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
** ��������SetRcvedBackCallFunc
** �䡡�룺BackCallFunc�����ݽ��պ�Ļص�����
** �䡡������
** ��������������RFID���ݽ��պ�Ļص�����
******************************************************************/
void SetRcvedBackCallFunc(RCVED_BACK_CALL_FUNC BackCallFunc)
{
	RcvedBackCallFunc = BackCallFunc;
}
/*****************************************************************
** ��������WL_ReceiveData
** �䡡�룺buf�����ݽ��ջ�����
**         num�������ֽ����������ϴ�ֵ����
** �䡡����ʵ�ʶ��������ֽ���
** ������������ȡ���յ���������
******************************************************************/
uint32_t WL_ReceiveData(unsigned char *buf, unsigned int num)
{
	st_RFIDRcvFrame RxFrm;

	if (RFID_GetFrame(&RxFrm) == RET_OK)
	{
//		StateLed(LED_WL, 0);
		if ((RxFrm.u8DestAddr == ALL_RFID_ADDR 				//Ⱥ����ַ
#ifndef DYK_SS
			|| RxFrm.u8DestAddr == SC_RFID_ADDR	//���ܵ�ַ
#else
			|| RxFrm.u8DestAddr == SS_RFID_ADDR				//���ܵ�ַ
#endif
			) && (num >= RxFrm.u8DataLen))					//���ȹ���
		{
			uint32_t i=RxFrm.u8DataLen;		//���ݳ���
			uint8_t *pt=RxFrm.u8Data;
			ReciveRSSIData = RxFrm.u8AppendStatus[0x00];
			if (i > 0)
			{
				memmove(buf, pt, i);	//ȡ����
				return (i);
			}
		}
	}else
//		StateLed(0, LED_WL);
	return(0);
}
/*****************************************************************
** ��������WL_SendData
** �䡡�룺buf���������ݻ�����
**         num�����������ֽ���
**         ToAddr�����յ�ַ
** �䡡����ʵ�ʷ����ֽ���
** �������������߷�������
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
** ���ܣ�����Ƿ��������߷���
** ���룺��
** ���أ�TRUE:���Է���
******************************************************************/
uint32_t WlEmitEnabled(void)
{
	//if (GetRfidSendEnable() == RET_OK)
		return (1);
	//else
	//	return (FALSE);
}
/***********************************************************************************************
** �� �� ����	RFID_FetchData
** ����������	�����ݴ洢����ȡ������
** �䡡  �룺	void;
** �䡡  ����	st_RFIDRcvFrame��һ������
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t RFID_FetchData(st_RFIDRcvFrame *RfidRcvFrm)
{
	return (RFID_GetFrame(RfidRcvFrm));
}
/***********************************************************************************************
** �� �� ����	RFID_SendData
** ����������	��FIFO��д�����ݣ�
** �䡡  �룺	u8DestAddr,���ݵĽ��յ�ַ��
**				pu8Buf�����ݻ���ָ�룻
**				u32Length�����ݳ���ָ�룻
** �䡡  ����	
** �� �� ֵ��	
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
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
		
	// ���뷢�ͺ���
	u8Ret = RFTxSendPacket(pu8DataTmp, u8Length);
	
	//if(u8Ret == TX_OK)
		//SetRxMode();			//�������ģʽ
		//RFCtrlSetIDLE();
	SetRfidSRX();	
	
	return((uint32_t)u8Ret);
	
}
/***********************************************************************************************
** �� �� ����	SetRfidSIDLE
** ����������	����CC1101�������״̬
** �䡡  �룺	��
** �䡡  ����	
** �� �� ֵ��	����״̬
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t SetRfidSIDLE(void)
{
	return(RFCtrlSetIDLE());
}
/***********************************************************************************************
** �� �� ����	SetRfidSRX
** ����������	����CC1101�������״̬
** �䡡  �룺	��
** �䡡  ����	
** �� �� ֵ��	����״̬
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint32_t SetRfidSRX(void)
{
	SetRxMode();
	return 0;
}
/***********************************************************************************************
** �� �� ����	GetRfidSendEnable
** ����������	��ȡCC1101�Ƿ�������
** �䡡  �룺	��
** �䡡  ����	
** �� �� ֵ��	RET_OK = 0:�����ͣ�RET_ERR = 1����������
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
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
** �� �� ����	GetRfidCurStatus
** ����������	��ȡCC1101��ǰ״̬
** �䡡  �룺	��
** �䡡  ����	
** �� �� ֵ��	����״̬
** ����  �ߣ�	����
** �ա�  �ڣ�	2014.12.26
** ��    ����	V1.0.0
** ���¼�¼��
** ���¼�¼��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
************************************************************************************************/
uint8_t GetRfidCurStatus(void)
{
	return(RfidGetTxStatus());
}
/***********************************************************************************************
** �� �� ����	SetRxAddrFilter
** ����������	����CC1101���յ�ַ�˲�����ֵ
** �䡡  �룺	Addr:��ַ��������ֵ
** �� �� ֵ��	
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
/*********************************������������޹�˾*************************************************************/

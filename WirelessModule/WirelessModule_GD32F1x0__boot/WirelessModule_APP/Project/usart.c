#include "usart.h"

#include "usartapp.h"


uint8_t  UsartReveiveDataBuf[USARTWIRELESS_TMP_BUF_MAX];   //临时接收数组
uint8_t  tmpTxWrtPtr  = 0;  //临时变量写指针

extern stUARTWIRELESS  USARTWIRLELESSTxBuf[USARTWIRELESS_TX_BUF_MAX];						//发送缓存
extern  uint8_t	 USARTWIRELESSTxRdPtr;			   							//发送读指针
stUARTWIRELESS  USARTWIRELESSRxBuf[USARTWIRELESS_RX_BUF_MAX];						//接收缓存
uint8_t	 USARTWIRELESSRxWrtPtr = 0;			   						    //接收写指针
uint8_t	 USARTWIRELESSRxRdPtr =  0;			   							//接收读指针
uint16_t const u16CrcUsarttab[256] = 
{
0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};




void gpio_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOB);
    /* configure led GPIO port */ 
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN_2);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0);
	gpio_bit_reset(GPIOB,GPIO_PIN_2);
	gpio_bit_reset(GPIOB,GPIO_PIN_8);
	
}


void usart_init(void)
{
    rcu_periph_clock_enable( EVAL_COM_GPIO_CLK);
    /* enable USART clock */
    rcu_periph_clock_enable(EVAL_COM1_CLK);

    /* connect port to USARTx_Tx */
    gpio_af_set(EVAL_COM_GPIO_PORT, EVAL_COM_AF, EVAL_COM1_TX_PIN);

    /* connect port to USARTx_Rx */
    gpio_af_set(EVAL_COM_GPIO_PORT, EVAL_COM_AF, EVAL_COM1_RX_PIN);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(EVAL_COM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,EVAL_COM1_TX_PIN);
    gpio_output_options_set(EVAL_COM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,EVAL_COM1_RX_PIN);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(EVAL_COM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,EVAL_COM1_TX_PIN);
    gpio_output_options_set(EVAL_COM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,EVAL_COM1_RX_PIN);

    /* USART configure */
    usart_deinit(EVAL_COM1);
    usart_baudrate_set(EVAL_COM1,115200U);
    usart_transmit_config(EVAL_COM1, USART_TRANSMIT_ENABLE);
    usart_receive_config(EVAL_COM1, USART_RECEIVE_ENABLE);
    usart_enable(EVAL_COM1);
	
	/* enable USART0 IDLEIE interrupt */
	usart_interrupt_enable(EVAL_COM1, USART_INT_IDLE);
	
	usart_interrupt_flag_clear(EVAL_COM1,USART_INT_FLAG_IDLE);
	usart_data_receive(EVAL_COM1);  //清楚标志位
	/* USART interrupt configuration */
	nvic_irq_enable(USART0_IRQn, 1, 1);

}

void dma_config(void)
{
    dma_parameter_struct dma_init_struct;
    /* enable the DMA clock */
    rcu_periph_clock_enable(RCU_DMA);
    /* configure the USART TX DMA channel */
    dma_deinit(DMA_TX );
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = 0;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 0;
    dma_init_struct.periph_addr = USART_TDATA_ADDRESS;
	
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_TX , &dma_init_struct);
    dma_circulation_disable(DMA_TX );
    dma_memory_to_memory_disable(DMA_TX );
    /* configure DMA mode */
    dma_circulation_disable(DMA_TX );
    dma_memory_to_memory_disable(DMA_TX );
    
    /* configure the USART RX DMA channel */
    dma_deinit(DMA_RX );
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)UsartReveiveDataBuf;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = USARTWIRELESS_TMP_BUF_MAX;
    dma_init_struct.periph_addr = USART_RDATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_RX , &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA_RX );
    dma_memory_to_memory_disable(DMA_RX );
	
	/* USART DMA enable for reception */
    usart_dma_receive_config(EVAL_COM1, USART_DENR_ENABLE);
	
	
    /* enable DMA channel2 transfer complete interrupt */
    dma_interrupt_enable(DMA_RX, DMA_INT_FTF);
    /* enable DMA channel2 */
    dma_channel_enable(DMA_RX);
	
	//DMA  spi
//	dma_parameter_struct dma_init_struct;
    /* enable the DMA clock */
//    rcu_periph_clock_enable(RCU_DMA);
    /* configure the USART TX DMA channel */
//    dma_deinit(DMA_TX );
//    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
//    dma_init_struct.memory_addr = 0;
//    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
//    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
//    dma_init_struct.number = 0;
//    dma_init_struct.periph_addr = USART_TDATA_ADDRESS;
//    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
//    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
//    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
//    dma_init(DMA_TX , &dma_init_struct);
//    dma_circulation_disable(DMA_TX );
//    dma_memory_to_memory_disable(DMA_TX );
//    /* configure DMA mode */
//    dma_circulation_disable(DMA_TX );
//    dma_memory_to_memory_disable(DMA_TX );
//    
//    /* configure the USART RX DMA channel */
//    dma_deinit(DMA_RX );
//    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
//    dma_init_struct.memory_addr = (uint32_t)UsartReveiveDataBuf;
//    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
//    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
//    dma_init_struct.number = USARTWIRELESS_TMP_BUF_MAX;
//    dma_init_struct.periph_addr = USART_RDATA_ADDRESS;
//    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
//    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
//    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
//    dma_init(DMA_RX , &dma_init_struct);
//    /* configure DMA mode */
//    dma_circulation_disable(DMA_RX );
//    dma_memory_to_memory_disable(DMA_RX );
//	
//	/* USART DMA enable for reception */
//    usart_dma_receive_config(EVAL_COM1, USART_DENR_ENABLE);
//    /* enable DMA channel2 transfer complete interrupt */
//    dma_interrupt_enable(DMA_RX, DMA_INT_FTF);
//    /* enable DMA channel2 */
//    dma_channel_enable(DMA_RX);
}
//
void usart_dma_send(uint8_t *buffer,uint16_t len)
{
	dma_channel_disable(DMA_TX);
	
	dma_memory_address_config(DMA_TX,(uint32_t)buffer);//设置要发送数据的内存地址
	dma_transfer_number_config(DMA_TX,len);//一共发多少个数据
	
	dma_channel_enable(DMA_TX);
	
  	usart_dma_transmit_config(USART0, USART_DENT_ENABLE);//使能串口DMA发送
	while(RESET == dma_flag_get(DMA_TX, DMA_FLAG_FTF));
}
/*******************************************************************************************
**函数作用：计算CRC16校验值
**函数参数：u8Buf:需要校验的数据起始地址，u32Len:需要校验数据的长度，*u16CheckOld:计算后的CRC校验值
**函数输出：无
**注意事项：无
*******************************************************************************************/
void Crc16UsartReceiveChick(const uint8_t *u8Buf, uint32_t u32Len, uint16_t *u16CheckOld)
{
	uint32_t u32Cnt = 0x00;
	
	uint16_t u16Crc = *u16CheckOld;
	
	for( u32Cnt = 0; u32Cnt < u32Len; u32Cnt++)
		u16Crc = (u16Crc<<8) ^ u16CrcUsarttab[((u16Crc>>8) ^ *(uint8_t *)u8Buf++)&0x00FF];
	*u16CheckOld = u16Crc;
}
/***********************************************************************************************
** 功能描述：	从串口接收队列中取数据
** 输　  入：	无
** 输　  出：	无
************************************************************************************************/
uint32_t UsartFetchData(stUARTWIRELESS *UsartRcvFrm)
{
	if(USARTWIRELESSRxWrtPtr != USARTWIRELESSRxRdPtr) 
	{
		memset((UsartRcvFrm), 0x00  , sizeof(stUARTWIRELESS));
		memcpy((UsartRcvFrm),&USARTWIRELESSRxBuf[USARTWIRELESSRxRdPtr],sizeof(stUARTWIRELESS) );	
		USARTWIRELESSRxRdPtr++;
		USARTWIRELESSRxRdPtr %= USARTWIRELESS_RX_BUF_MAX;
		return TRUE;
	}
	else
	{
		return FALSE;
	}	
}
void USART0_IRQHandler(void)
{
//	uint8_t len;
	uint16_t u16CrcTemp = 0x00;
    //DMA接收，只触发空闲中断
    if(RESET != usart_flag_get(EVAL_COM1, USART_FLAG_IDLE))
    {
		usart_interrupt_flag_clear(EVAL_COM1,USART_INT_FLAG_IDLE);
        usart_data_receive(EVAL_COM1);  //清楚标志位
		dma_channel_disable(DMA_RX);	
		if((UsartReveiveDataBuf[3] != 1) && ((UsartReveiveDataBuf[4] == 1) || (UsartReveiveDataBuf[4] == 0)))
		{
			;
		}
		else
		{
			Crc16UsartReceiveChick(UsartReveiveDataBuf,UsartReveiveDataBuf[4]+5,&u16CrcTemp);	
			//校验接收的数据是否正确
			if(UsartReveiveDataBuf[UsartReveiveDataBuf[4]+5] == (uint8_t)((u16CrcTemp&0xFF00)>>8)  
			   && UsartReveiveDataBuf[UsartReveiveDataBuf[4]+6] ==  (uint8_t)(u16CrcTemp&0x00FF))
			{
				memset(&(USARTWIRELESSRxBuf[USARTWIRELESSRxWrtPtr]), 0x00  , sizeof(stUARTWIRELESS));
				memcpy(&(USARTWIRELESSRxBuf[USARTWIRELESSRxWrtPtr]), UsartReveiveDataBuf,UsartReveiveDataBuf[4]+5 );
				USARTWIRELESSRxWrtPtr ++;
				USARTWIRELESSRxWrtPtr %= USARTWIRELESS_RX_BUF_MAX;
				//发应答
				if(UsartReveiveDataBuf[3] == 1)
				{
					SendResponseData((USARTWIRELESSRxBuf[USARTWIRELESSRxWrtPtr].stFrame),1);
				}
			}
		}
		/* 重新设置DMA传输 */
		dma_memory_address_config(DMA_RX,(uint32_t)UsartReveiveDataBuf);
		dma_transfer_number_config(DMA_RX,sizeof(UsartReveiveDataBuf));
		dma_channel_enable(DMA_RX);		/* 开启DMA传输 */
    }
}

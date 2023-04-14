/*!
    \file    WirelessModule_APP/main.c
    \brief   UART0 and SPI0 conversion  
    \version 2022-8-23 for GD32F1x0
*/

#include "gd32f1x0.h"
#include "systick.h"
#include "usart.h"
#include "usartapp.h"
#include "rfid_driver.h"

#define   DELAY   4 


//重装载值
#define WDGT_RELOAD_VALUE       156 
//分频系数
#define WDGT_PRESCALER_DIV      FWDGT_PSC_DIV256




static void irc40k_config(void)
{
    /* enable IRC40K */
    rcu_osci_on(RCU_IRC40K);
    /* wait till IRC40K is ready */
    while(ERROR == rcu_osci_stab_wait(RCU_IRC40K));
}
void WDGT_Init(void)
{
    //时钟初始化
    irc40k_config();    
    fwdgt_config(WDGT_RELOAD_VALUE, WDGT_PRESCALER_DIV);
    fwdgt_enable();    
}
//喂狗
void WDGT_Feed(void)
{
    fwdgt_counter_reload();
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t time;

int main(void)
{
    uint32_t i = 0,j = 0;	
	systick_config();
	gpio_init();
	usart_init();
	dma_config();
	RFID_Init();
	SendStartData();

	for(i = 0; i<1;i++)
	{
		for(j = 0; j<2000000;j++)
		;
	}
	SendStartData();
	WDGT_Init();	
	while(1)
	{	
		if(time > 1580/DELAY)
		{
			SendHeartBeatData();
		}
		SendUsartDataProc();
		RecvUsartDataProc();
		Wl_RxDataProc();
		TimeFun();
		delay_1ms(DELAY);
		WDGT_Feed();
		time++;
	}

}

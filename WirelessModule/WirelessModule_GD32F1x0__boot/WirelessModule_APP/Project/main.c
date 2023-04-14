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
#include "iapupdate.h"

uint8_t UpdateVersion(void)
{
	uint32_t u32DevTemp =0x00;
	uint32_t u32TargetTemp =0x00;
	u8IapReadBuf((uint8_t *)&u32DevTemp,WL_DEV_PROGRAM_BASE_ADDRESS+PROG_DEVTYPE_OFFSET_ADDRESS,4);
	u8IapReadBuf((uint8_t *)&u32TargetTemp,WL_DEV_PROGRAM_BASE_ADDRESS+PROG_TARGETYPE_OFFSET_ADDRESS,4);
	if(u32DevTemp != THE_DEV_TYPE)
	{
		return 0x00;
	}
	if(u32TargetTemp!= THE_TARGET_TYPE)
	{
		return 0x00;
	}
	return 0x01;
}




void upgrade(void)
{
	uint8_t buf[32];
	uint32_t UPDATE;
	uint32_t PRGUPDATE;
	uint32_t size;
	uint32_t read;
	uint32_t write;
	uint64_t flag;
	
	u8IapReadBuf((uint8_t *)&UPDATE,APP_UPDATE_OFFSET_ADDRESS,4);
	u8IapReadBuf((uint8_t *)&PRGUPDATE,APP_PRGUPDATE_SUCCEED_ADDRESS,4);
	if((UPDATE == 1) && (PRGUPDATE == 0))
	{
		if(UpdateVersion())
		{
			u8IapReadBuf((uint8_t *)&size,WL_DEV_PROGRAM_BASE_ADDRESS+PROG_LENGTH_OFFSET_ADDRESS,4);
			if(size < (20*1024))
			{
				u8IapEraserSector(0x08004000,0x08008C00-1);
				read = 0x08008C00 + 0x100;
				write = 0x08004000;
				size += read;
				while(read < size)
				{
					u8IapReadBuf(buf,read,32);
					u8IapWriteBuf(buf,write,32);
					read += 32;
					write += 32;
				}
			}
			u8IapEraserSector(0x0800DC00,0x0800DC00);
		}
	}
	vIapJumpToApp(0x08004000);
}
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
	int i;
	for(i=0;i>10000;i++)
		i = i;
	
	while(1)
	{
		upgrade();
	}
}

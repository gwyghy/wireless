/********************************************************************************
* �ļ����ƣ�	iapupdate.c
* ��	�ߣ�	������   
* ��ǰ�汾��   	V1.0
* ������ڣ�    2014.02.08
* ��������: 	��ɳ�������ʱ�����Ӳ���Ĳ�������Ҫ��������FLASH��LCD��ʾ�ȵĲ�����������Ŀʵ�����������ʹ�õĵײ㺯�������滻��
* ��ʷ��Ϣ��   
*           	�汾��Ϣ     ���ʱ��      ԭ����        ע��
*
*       >>>>  �ڹ����е�λ��  <<<<
*          	  3-Ӧ�ò�
*           �� 2-Э���
*             1-Ӳ��������
*********************************************************************************
* Copyright (c) 2014,������������޹�˾ All rights reserved.
*********************************************************************************/
/********************************************************************************
* .hͷ�ļ�
*********************************************************************************/
#include "iapupdate.h"
#include <gd32f1x0.h>
#include "string.h"
/********************************************************************************
* #define�궨��
*********************************************************************************/
 #define ERROR_UP_OVERFLOW     1
 #define ERROR_DOWN_OVERFLOW   2
 #define ERROR_OUTOFFRAME      3
 #define ERROR_OVERTIME		   4
 #define ERROR_DATA_NUM		   5


 #define FLASH_SIZE    0x0000FFFF
 #define PAGE_SIZE    0x400
/********************************************************************************
* ��������
*********************************************************************************/

/********************************************************************************
* ��������
*********************************************************************************/
/**�����豸������洢��ַ�Ķ�Ӧ��ϵ***//**��Ӧ�ڲ�ͬ��ģ�飬�˱���Ҫ�޸�***/
static PRG_STORAGE_BASEADDR_TYPE const sDevPrgWriteBase[(uint8_t)(DYK_DEV_TYPE_MAX&0xFF)]=
{
	{THE_DEV_TYPE,		THE_DEV_PRG_STORAGE_BASEADDR},
	{DYK_DEV_TYPE_MAX,	0x00}
};
/**�����豸����������С�Ķ�Ӧ��ϵ***//**��Ӧ�ڲ�ͬ��ģ�飬�˱���Ҫ�޸�***/
static PRG_STORAGE_BASEADDR_TYPE const	sDevPrgSize[(uint8_t)(DYK_DEV_TYPE_MAX&0xFF)]=	
{
	{THE_DEV_TYPE,		THE_DEV_PRG_STORAGE_SIZE},
	{DYK_DEV_TYPE_MAX,	0x00}
};

typedef  void (*pFunction)(void);
/*����jumpToAppΪָ�뺯��*/
pFunction jumpToApp; 
static uint8_t u8Buf[LOAD_CODE_TEMPBUF_MAX_LEN] = {0x00};//�������ݼ������ʱ����
/********************************************************************************
* ��������
*********************************************************************************/
/*******************************************************************************************
**�������ƣ�IapGetPrgStorageAddr
**��������: ��ȡ��������/��Flash�еĴ洢��ַ
**������������
**ע�������
*******************************************************************************************/
uint8_t u8IapGetPrgStorageAddr(uint32_t u32DevType,uint32_t *pStorageAddr)
{
	uint8_t u8I = 0x00;
	uint8_t u8ReturnValue = 0x00;
	
	for(u8I = 0x00; u8I < (uint8_t)(DYK_DEV_TYPE_MAX&0xFF); u8I++)
	{
		if(sDevPrgWriteBase[u8I].u32DevType == DYK_DEV_TYPE_MAX)
		{
			u8ReturnValue = 0x00;
			break;
		}
		if(sDevPrgWriteBase[u8I].u32DevType == u32DevType)
		{
			*pStorageAddr = sDevPrgWriteBase[u8I].u32NorPrgBaseAddr;
			u8ReturnValue = 0x01;
			break;
		}
			
	}
	
	return u8ReturnValue;
}
/*******************************************************************************************
**�������ƣ�IapGetPrgStorageAddr
**��������: ��ȡ��������/��Flash�еĳ���洢��С
**������������
**ע�������
*******************************************************************************************/
uint8_t u8IapGetPrgSize(uint32_t u32DevType,uint32_t *pSize)
{
	uint8_t u8I = 0x00;
	uint8_t u8ReturnValue = 0x00;
	
	for(u8I = 0x00; u8I < (uint8_t)(DYK_DEV_TYPE_MAX&0xFF); u8I++)
	{
		if(sDevPrgSize[u8I].u32DevType == DYK_DEV_TYPE_MAX)
		{
			u8ReturnValue = 0x00;
			break;
		}
		if(sDevPrgSize[u8I].u32DevType == u32DevType)
		{
			*pSize = sDevPrgSize[u8I].u32NorPrgBaseAddr;
			u8ReturnValue = 0x01;
			break;
		}
			
	}
	
	return u8ReturnValue;
}
/*******************************************************************************************
**�������ƣ�NVIC_DeInit
**��������: NVIC��λ
**������������
**ע�������
*******************************************************************************************/
void NVIC_DeInit(void)
{
 #if (BSP_PLATFORM	==	BSP_PLATFORM_M4)
	uint32_t u32I = 0x00;
	
	/*�رն�ʱ���ж�*/
	SysTick->VAL   = 0x00;                                          /* Load the SysTick Counter Value */
	SysTick->CTRL  = 0x00;
	/*����ж�����λ��cortex_m4�ں˹���81���ж�*/
	NVIC->ICER[0] = 0xFFFFFFFF;//д��1�������ж�ʧ��
	NVIC->ICER[1] = 0xFFFFFFFF;
	NVIC->ICER[2] = 0x0001FFFF;
	/*����жϹ���λ*/
	NVIC->ICPR[0] = 0xFFFFFFFF;//д��1������жϹ���״̬
	NVIC->ICPR[1] = 0xFFFFFFFF;	
	NVIC->ICPR[2] = 0x0001FFFF;
	
	for(u32I = 0; u32I < 81; u32I++)
	{
	 NVIC->IP[u32I] = 0x00;	
	} 
#elif (BSP_PLATFORM	==	BSP_PLATFORM_M3)
	uint32_t index = 0;
	
	NVIC->ICER[0] = 0xFFFFFFFF;
	NVIC->ICER[1] = 0x000007FF;
	NVIC->ICPR[0] = 0xFFFFFFFF;
	NVIC->ICPR[1] = 0x000007FF;
	
	for(index = 0; index < 0x0B; index++)
	{
	 NVIC->IP[index] = 0x00000000;	
	}  
#endif

}

/*******************************************************************************************
**�������ƣ�vIapJumpToApp
**�������ã���ת��APP���� 
**������������
**�����������
**ע�������
*******************************************************************************************/
void vIapJumpToApp(uint32_t u32AppAddr)
{
	/*���ջ����ַ�Ƿ�Ϸ�.*/
	if(((*(uint32_t*)u32AppAddr) & 0x2FFE0000) == 0x20000000)
	{ 	
		//NVIC��λ
		NVIC_DeInit();
		/*�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)*/
		jumpToApp = (pFunction)(*(uint32_t*)(u32AppAddr+4));

		/*��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)*/				
		__set_MSP(*(uint32_t*)u32AppAddr);

		/* Vector Table Relocation in Internal FLASH. */
  		SCB->VTOR = FLASH_BASE | VTOR_OFFSET; 
		//SCB->VTOR = appAddr; 

		
		/*��ת��APP*/						
		jumpToApp();											
	}
}

/*********************************************************************************************
** ��������iapJumpToBoot
** �䡡�룺bootAddr --- BOOT�����ַ  
** �䡡����None
** ������������ת��bootloader���� 
** ȫ�ֱ�����None
** ����ģ�飺None
** ������:	������
** �ա��ڣ� 2013.07.18
*********************************************************************************************/
void vIapJumpToBoot(uint32_t u32bootAddr)
{
	/*���ջ����ַ�Ƿ�Ϸ�.*/
	if(((*(uint32_t*)u32bootAddr) & 0x2FFE0000) == 0x20000000)
	{ 	
		__set_FAULTMASK(1);
		NVIC_SystemReset(); //�ָ�NVICΪ��λ״̬.ʹ�жϲ��ٷ���.�����⣬2013.07.18����
		//NVIC��λ
		//NVIC_DeInit(); //�ڶ��ַ����д�����֤������֤�ɹ�
		/*�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)*/
		jumpToApp = (pFunction)(*(uint32_t*)(u32bootAddr+4));
		

		/*��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)*/				
		__set_MSP(*(uint32_t*)u32bootAddr);

		/* Vector Table Relocation in Internal FLASH. */
		//SCB->VTOR = IN_FLASH_BOOTLOADER_ADDR | VECT_BL_TAB_OFFSET; 
		SCB->VTOR = u32bootAddr; 
		
		/*��ת��Boot*/						
		jumpToApp();											
	}
}
/*******************************************************************************************
**�������ƣ�u16InFlashCrc
**�������ã�����CRCУ��
**����������u32SrcAddr:��ʼ��ַ��u32Len:��ҪУ�����ݵĳ���
**���������uint16_t����CRCУ��ֵ
**ע�������
*******************************************************************************************/
/* CRC16 implementation acording to CCITT standards */
uint16_t const u16Crc16tab[256] = 
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
uint16_t u16InFlashCrc(uint32_t u32SrcAddr, uint32_t u32Len)
{
	uint32_t u32Counter = 0x00;
	uint16_t u16Crc = 0x00;
	
	for( u32Counter = 0; u32Counter < u32Len; u32Counter++)
	{
		u16Crc = (u16Crc<<8) ^ u16Crc16tab[((u16Crc>>8) ^ *(uint8_t *)u32SrcAddr++)&0x00FF];
	}
	return u16Crc;
}

/*******************************************************************************************
**�������ƣ�vCrc16Ccitt
**�������ã�����CRC16У��ֵ
**����������u8Buf:��ҪУ���������ʼ��ַ��u32Len:��ҪУ�����ݵĳ��ȣ�*u16CheckOld:������CRCУ��ֵ
**�����������
**ע�������
*******************************************************************************************/
void vCrc16Ccitt(const uint8_t *u8Buf, uint32_t u32Len, uint16_t *u16CheckOld)
{
	uint32_t u32Cnt = 0x00;
	uint16_t u16Crc = *u16CheckOld;
	
	for( u32Cnt = 0; u32Cnt < u32Len; u32Cnt++)
		u16Crc = (u16Crc<<8) ^ u16Crc16tab[((u16Crc>>8) ^ *(uint8_t *)u8Buf++)&0x00FF];
	*u16CheckOld = u16Crc;
}

/*******************************************************************************************
**�������ƣ�vDelay
**�������ã��򵥵���ʱ(�Ǿ�ȷ)
**����������u32Count��ʱ��ʱ�����
**�����������
**ע�������
*******************************************************************************************/
void vDelay(uint32_t u32Count)
{
	uint32_t u32I = 0x00, u32Y = 0x00;
	
	for (u32I = 0; u32I < 5000 ; ++u32I)
	{
		for (u32Y = 0; u32Y < u32Count ; ++u32Y)
			;
	}
}
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
//uint32_t page = 0;
//static uint32_t GetPage(uint32_t Addr)
//{
//  if (Addr < (IN_FLASH_BOOTLOADER_ADDR + FLASH_SIZE))
//  {
//    /* Bank 1 */
//    page = Addr ;
//	page = page - IN_FLASH_BOOTLOADER_ADDR;
//	page = page  / PAGE_SIZE;
//  } 
//  return page;
//}
/*******************************************************************************************
**�������ƣ�vInFlashEraseAllFromPage �� Sector
**�������ã���������ʼҳ��ʼ��flash���һҳ ��	��������ʼ������ʼ��flash���һ����
**����������u32StartPageAddr����Ҫ��������ʼҳ�ĵ�ַ
**�����������
**ע�������
*******************************************************************************************/
void vInFlashErasePage(uint32_t u32StartPageAddr,uint32_t u32EndPageAddr)
{
	uint32_t erase_counter;	
	/* unlock the flash program/erase controller */	
	uint32_t page_num;	
	
	page_num = ((u32EndPageAddr - u32StartPageAddr)/PAGE_SIZE)+1;
	fmc_unlock();
    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_END);
    fmc_flag_clear(FMC_FLAG_WPERR);
    fmc_flag_clear(FMC_FLAG_PGERR);    
    /* erase the flash pages */
    for(erase_counter = 0; erase_counter < page_num; erase_counter++)
	{
        fmc_page_erase(u32StartPageAddr + (PAGE_SIZE * erase_counter));		
		fmc_flag_clear(FMC_FLAG_END);
		fmc_flag_clear(FMC_FLAG_WPERR);
		fmc_flag_clear(FMC_FLAG_PGERR);   
    }
    /* lock the main FMC after the erase operation */
    fmc_lock();
}
/*******************************************************************************************
**�������ƣ�u8InFlashReadBuf
**�������ã���ȡ�ڲ�FLASH������
**����������u32StartPageAddr����Ҫ��������ʼҳ�ĵ�ַ
**�����������
**ע�������
*******************************************************************************************/
#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
	uint8_t u8InFlashReadBuf(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
	{
		while (NumByteToRead--)
		{
//			__disable_fault_irq();
			*pBuffer++ = *(uint8_t *)ReadAddr++;
//			__enable_fault_irq();	
		}
	}
#endif
/*******************************************************************************************
**�������ƣ�u8InFlashWriteBuf
**�������ã�д�ڲ�FLASH������
**����������u32StartPageAddr����Ҫ��������ʼҳ�ĵ�ַ
**�����������
**ע�������
*******************************************************************************************/
#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
	uint8_t u8InFlashWriteBuf(uint8_t* pBuffer, uint32_t u32WriteAddr, uint16_t NumByteToRead)
	{
		uint8_t u8Flag = 1;
		uint8_t tmp_buf[4];
		uint8_t tmp_over = 0;
		uint16_t count = 0;
		uint16_t i;
		tmp_over = NumByteToRead % 4;
		count = NumByteToRead / 4;
		/* unlock the flash program/erase controller */
		fmc_unlock();
		for(i = 0; i < count; i++)
		{
			if(fmc_word_program(u32WriteAddr, *(uint32_t *)pBuffer) == FMC_READY)
			{
				u32WriteAddr += 4;
				pBuffer += 4;
				fmc_flag_clear(FMC_FLAG_END);
				fmc_flag_clear(FMC_FLAG_WPERR);
				fmc_flag_clear(FMC_FLAG_PGERR);
			}
			else
				u8Flag = 0;
		}
		if(tmp_over)
		{
			memset(tmp_buf,0xff,4);
			memcpy(tmp_buf,pBuffer,tmp_over);
			if(fmc_word_program(u32WriteAddr, *(uint32_t *)tmp_buf) == FMC_READY)
			{
				fmc_flag_clear(FMC_FLAG_END);
				fmc_flag_clear(FMC_FLAG_WPERR);
				fmc_flag_clear(FMC_FLAG_PGERR);

				u8Flag = 1;
			}
			else
				u8Flag = 0;
		}
		
	   /* lock the main FMC after the program operation */
		fmc_lock();
		return  u8Flag;
	}
#endif
/*******************************************************************************************
**�������ƣ�u16IapExFlashCrc
**�������ã������ srcAddr��ʼ����Ϊlen�����ݵ�crc
**����������u32SrcAddr:�ⲿFLASH�������׵�ַ,u32Len:�ⲿFLASH�����ݳ���
**���������uint16_t����CRCУ��ֵ
**ע�������
*******************************************************************************************/
uint16_t u16IapExFlashCrc(uint32_t u32SrcAddr, uint32_t u32Len,uint16_t *pCrc)
{
	uint16_t u16i = 0x00;
	uint16_t u16Crc = *pCrc;
	uint16_t u16CopyLen = 0x00;
	uint32_t u32ExFlashAppAddr = u32SrcAddr;
	
	while (u32Len > 0)
	{
		/*�ж�ÿ�ο������ݵĴ�С*/	
		u16CopyLen = (u32Len > LOAD_CODE_TEMPBUF_MAX_LEN) ? LOAD_CODE_TEMPBUF_MAX_LEN : u32Len;			
		
		/*��ȡÿ�ο���������*/
		#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)
			W25QXX_ReadBuffer(u8Buf, u32ExFlashAppAddr, u16CopyLen);	
		#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
			u8InFlashReadBuf(u8Buf, u32ExFlashAppAddr, u16CopyLen);	
		#else
			return 0x00;
		#endif
		u32ExFlashAppAddr += u16CopyLen;
		
		/*�����ݽ���CRCУ��*/
		for (u16i = 0; u16i < u16CopyLen; u16i++)
		{
			u16Crc = (u16Crc<<8) ^ u16Crc16tab[((u16Crc>>8) ^ u8Buf[u16i])&0x00FF];	
		}
		/*ʣ�����ݵĳ���*/
		 if (u16CopyLen != u32Len)
		{
			u32Len = u32Len - LOAD_CODE_TEMPBUF_MAX_LEN;
		}
		else
		{
			return u16Crc;
		}
	} 
	return 0;
}




/*******************************************************************************************
**�������ƣ�u32IapReadCodeLen
**�������ã�������Ҫ��������Ĵ�С
**������������
**�����������
**ע�������
*******************************************************************************************/
uint32_t u32IapReadCodeLen(uint32_t u32Addr)
{
	uint32_t u32Len = 0x00;
	
	/*��Ŀ���ַ�������������������*/
	#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)
		W25QXX_ReadBuffer((uint8_t *)&u32Len, u32Addr, 0x04);
	#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
		u8InFlashReadBuf((uint8_t *)&u32Len, u32Addr, 0x04);
	#else
		return 0x00;
	#endif
	return u32Len;
}


/*******************************************************************************************
**�������ƣ�vIapReadCrc
**�������ã������ⲿFLASH��CRC
**������������
**���������CRCУ��ֵ��Ϊuint16_t���͡�
**ע�������
*******************************************************************************************/
uint32_t u32IapReadCrc(uint32_t u32Addr)
{
	uint32_t u32Crc = 0x00;

	/*��Ŀ���ַ�������������������*/
	#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)	
		W25QXX_ReadBuffer((uint8_t *)&u32Crc, u32Addr, 0x04);
	#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
		u8InFlashReadBuf((uint8_t *)&u32Crc, u32Addr, 0x04);
	#else
		return 0x00;
	#endif
	return u32Crc;
}

/*******************************************************************************************
**�������ƣ�u32IapReadFlag
**�������ã���ȡ������־״̬
**����������u32Addr:��Ҫ�����ı�־λ��ַ 
**��������������Ƿ�ɹ���UCCESS:��־���óɹ�(0x00);ERROR_UP_OVERFLOW:��ַ��������ַ�����(0x01)
**ע�������
*******************************************************************************************/
uint32_t u32IapReadFlag(uint32_t u32Addr)
{
 	/*��Ŀ���ַ�ϵı�־����*/
 	uint32_t u32FlagStats = 0x00;
	
	#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)		
 		W25QXX_ReadBuffer((uint8_t *)&u32FlagStats, u32Addr, 0x04);
	#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
		u8InFlashReadBuf((uint8_t *)&u32FlagStats, u32Addr, 0x04);
	#else
		return 0x00;
	#endif
	
 	return u32FlagStats;
}


/*******************************************************************************************
**�������ƣ�u8IapReadBuf
**�������ã���ȡFLASH���ض����ȵ����ݣ���д�뵽ָ��λ��
**����������pBuffer��д���ַ,u32Addr:��Ҫ��������ʼ��ַ��u16NumByteToRead���ֽ��� 
**��������������Ƿ�ɹ���SUCCESS:��ȡ�ɹ�(0);ERROR_UP_OVERFLOW:��ַ��������ַ�����(0x01)
**ע�������
*******************************************************************************************/
uint8_t u8IapReadBuf(uint8_t *pBuffer, uint32_t u32ReadAddr, uint16_t u16NumByteToRead)
 {
 	uint8_t u8Flag = 0x00;

	#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)	
		u8Flag = W25QXX_ReadBuffer(pBuffer,u32ReadAddr,u16NumByteToRead);
	#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
		u8Flag = u8InFlashReadBuf(pBuffer,u32ReadAddr,u16NumByteToRead);
	#else
		u8Flag =  0x00;
	#endif
 	return u8Flag;
}

/*******************************************************************************************
**�������ƣ�u8IapWriteBuf
**�������ã����ض����ȵ�����д�뵽flash��ָ��λ��
**����������pBuffer�����ݶ�ȡ��ַ��u32Addr:��Ҫ��������ʼ��ַ��u16NumByteToRead���ֽ���  
**��������������Ƿ�ɹ���SUCCESS:д��ɹ�(0x00);ERROR_UP_OVERFLOW:��ַ��������ַ�����(0x01)
**ע�����u16NumByteToRead:���д���ֽڷ�ΧΪ64K��
*******************************************************************************************/
uint8_t u8IapWriteBuf(uint8_t *pBuffer, uint32_t u32ReadAddr, uint16_t u16NumByteToRead)
 {
 	uint8_t u8Flag = 0x00;
	 
	#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)
		u8Flag = W25QXX_WriteBuffer(pBuffer,u32ReadAddr,u16NumByteToRead);
	#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
		u8Flag = u8InFlashWriteBuf(pBuffer,u32ReadAddr,u16NumByteToRead);
	#else
		u8Flag =  0x00;
	#endif	
 	return u8Flag;
}

/*******************************************************************************************
**�������ƣ�u8IapEraserBulk
**�������ã�ɾ��һ����
**����������u32BulkAddr:��Ҫ��������ʼ��ַ
**��������������Ƿ�ɹ���SUCCESS:�����ɹ�(0x00);ERROR_UP_OVERFLOW:��ַ��������ַ�����(0x01)
**ע�����һ����Ĭ��Ϊ64K.
                              ������ʵ��Ϊ64K.
                              �ڲ�:1xx��ҳΪ��λ��С����������Ϊ1K��������������Ϊ2K��
                                          4xx������Ϊ��λ,Ϊ16K~128K
*******************************************************************************************/
uint8_t u8IapEraserBulk(uint32_t u32BulkAddr)
{
	uint8_t u8Flag = 0x00;
	
	#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)
		u8Flag = W25QXX_EraseBulk(u32BulkAddr);
	#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
		#if (BSP_PLATFORM	==	BSP_PLATFORM_M3)
			 vInFlashErasePage(u32BulkAddr,u32BulkAddr+THE_DEV_PRG_BULK_SIZE);
		#elif (BSP_PLATFORM	==	BSP_PLATFORM_M4)
			vInFlashEraseSector(u32BulkAddr,u32BulkAddr+THE_DEV_PRG_BULK_SIZE,THE_DEV_CPU_VOLTAGE);
		#else
			u8Flag = 0x00;
		#endif
	#else
		u8Flag = 0x00;
	#endif
	
	return u8Flag;
}

/*******************************************************************************************
**�������ƣ�u8IapEraserSector
**�������ã�ɾ��һ������
**����������u32BulkAddr:��Ҫ��������ʼ��ַ
**��������������Ƿ�ɹ���SUCCESS:�����ɹ�(0x00);ERROR_UP_OVERFLOW:��ַ��������ַ�����(0x01)
**ע�������
*******************************************************************************************/
uint8_t u8IapEraserSector(uint32_t u32StartPageAddr,uint32_t u32EndPageAddr)
{
	uint8_t u8Flag = 0x00;

	#if (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_EXFLASH_USE)
		u8Flag = W25QXX_EraseSector(u32SectorAddr);
	#elif (BSP_PRG_STORAGE_FLASH_FLAG	==	BSP_PRG_STORAGE_INFLASH_USE)
		#if (BSP_PLATFORM	==	BSP_PLATFORM_M3)
			 vInFlashErasePage(u32StartPageAddr,u32EndPageAddr);
		#elif (BSP_PLATFORM	==	BSP_PLATFORM_M4)
			vInFlashEraseSector(u32SectorAddr,u32SectorAddr+THE_DEV_PRG_SECTOR_SIZE,THE_DEV_CPU_VOLTAGE);
		#else
			u8Flag = 0x00;
		#endif
	#else
		u8Flag = 0x00;
	#endif	
	
	return u8Flag;
}

/*******************************************************************************************
**�������ƣ�LcdClear
**�������ã���������
**������������
**�����������
**ע�������
*******************************************************************************************/
#if (BSP_LCD_FLAG == BSP_LCD_USE)
	void vIapLcdClear(void)
	{
		LcdClr(0);
	}
#endif

/*******************************************************************************************
**�������ƣ�vShowStr
**�������ã���ʾ�ַ���
**������������
**�����������
**ע�������
*******************************************************************************************/
#if (BSP_LCD_FLAG == BSP_LCD_USE)
	void vIapShowStr(uint16_t u16Inv, const uint8_t *cu8Str, uint16_t u16Row, uint16_t u16Col)
	{
		ShowStr_16b(u16Inv, cu8Str, strlen((char *)cu8Str), u16Row, u16Col);
	}
#endif



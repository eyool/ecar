/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : flash_if.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Header for flash_if.c file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_MAL_H
#define __FLASH_IF_MAL_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32L1XX_MD
 #include "stm32l1xx.h"
#else
 #include "stm32f10x.h"
#endif /* STM32L1XX_MD */
 
#define MAL_OK   0
#define MAL_FAIL 1
#define bMaxPacketSize0             0x40     /* bMaxPacketSize0 = 64 bytes   */
#define wTransferSize               0x0400   /* wTransferSize   = 1024 bytes */

#define PAGE_SIZE										0x400
#define SYS_FLASHADDR_START         0x0800c000
#define SYS_FLASHADDR_END           0x0800c3ff
#define CFGSYS_FLASHADDR_START         0x0800c400
#define CFGSYS_FLASHADDR_END           0x0800ffff

#define CFGSYS_FLASH_CFG_MD5        CFGSYS_FLASHADDR_START
//#define CFGSYS_FLASH_CFG_N          CFGSYS_FLASH_CFG_MD5+16 /*2�ֽ�*/
#define CFGSYS_FLASH_CFG_NP         CFGSYS_FLASH_CFG_MD5+16 /*2�ֽڵ�ַ��1�ֽڳ��� */
#define CFGSYS_FLASH_CFG_BODY       CFGSYS_FLASHADDR_START+0x600

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint16_t FLASH_If_Init(void);
uint16_t FLASH_If_Erase (uint32_t SectorAddress);
uint16_t FLASH_If_Write(uint32_t SectorAddress, uint8_t *MAL_Buffer,uint32_t DataLength);
uint8_t *FLASH_If_Read (uint32_t SectorAddress, uint32_t DataLength);

#endif /* __FLASH_IF_MAL_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

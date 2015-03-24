/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "bsp_int.h"
#include "bsp.h"
#include <os_cpu.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
 // OS_CPU_PendSVHandler();
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
 // OS_CPU_SysTickHandler();
}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void EXTI9_5_IRQHandler(void)
{

}
*/
/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void EXTI15_10_IRQHandler(void)
{
}*/


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : PPP_IRQHandler
* Description    : This function handles PPP interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

void  BSP_IntDisAll (void)
{
    CPU_IntDis();
}

void USART1_IRQHandler(void)
{
    OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	 if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
	 {//先读USART_SR，然后读USART_DR）
		USART_ReceiveData(USART1);
		Wifi_Istr();
	 }
	OSIntExit(); 
}
void USART2_IRQHandler(void)
{
    OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	 if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
	 {//先读USART_SR，然后读USART_DR）
		USART_ReceiveData(USART2);
		Zigbee_Istr();
	 }
	OSIntExit();
}
void USART3_IRQHandler(void)
{
    OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	 if(USART_GetITStatus(USART3,USART_IT_IDLE)!=RESET)
	 {//先读USART_SR，然后读USART_DR）
		USART_ReceiveData(USART3);
		Uhfid_Istr();
	 }

	OSIntExit();
}

void SPI2_IRQHandler(void)
{
    OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	Spi2_Istr();

	OSIntExit();
}
 /*void TIM3_IRQHandler(void)
{
 if(TIM_GetITStatus(RMT_TIM, RMT_IT_CC2))
  {
  	TIM_ClearITPendingBit(RMT_TIM, RMT_IT_CC2);
	StepMotorITProc(MOTOR_R);
  }

  if(TIM_GetITStatus(RMT_TIM, RMT_IT_CC2))
  {
  	TIM_ClearITPendingBit(RMT_TIM, RMT_IT_CC2);
	StepMotorITProc(MOTOR_L);
  }
}  */
void TIM4_IRQHandler(void)	 //统计3个电机速度
{
    OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	if(TIM_GetITStatus(SPDCAP_TIM, SPDCAP_T_IT) == SET) 
	{
	  	TIM_ClearITPendingBit(SPDCAP_TIM, SPDCAP_T_IT);
		SpdCap_Istr(SPDCAP_T_CHL,TIM_GetCapture1(SPDCAP_TIM));
	
	}
	if(TIM_GetITStatus(SPDCAP_TIM, SPDCAP_L_IT) == SET) 
	{
	  	TIM_ClearITPendingBit(SPDCAP_TIM, SPDCAP_L_IT);
		SpdCap_Istr(SPDCAP_L_CHL,TIM_GetCapture2(SPDCAP_TIM));
	
	}
	if(TIM_GetITStatus(SPDCAP_TIM, SPDCAP_R_IT) == SET) 
	{
	  	TIM_ClearITPendingBit(SPDCAP_TIM, SPDCAP_R_IT);
		SpdCap_Istr(SPDCAP_R_CHL,TIM_GetCapture3(SPDCAP_TIM));	
	}

	OSIntExit();
}
void EXTI4_IRQHandler(void)
{
    OS_CPU_SR  cpu_sr;
    OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

		if(EXTI_GetFlagStatus(EXTI_Line4)==SET)
		{
			EXTI_ClearITPendingBit(EXTI_Line4);
			SetCenter();
		}

	OSIntExit();
}	

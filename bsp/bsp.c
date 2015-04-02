#include  <bsp.h>
#include "app.h"
#include "flash_if.h"

extern   OS_EVENT      *App_UARTMbox;
INT8U const Uhfid_open[]=UHFID_OPENDEV;
INT8U const Uhfid_close[]=UHFID_CLOSEDEV;
INT8U const Uhf_ft[]=UHFID_FILTER;

INT8U  WifiTxBuf[WIFI_DMA_TX_BUFSIZE],WifiRxBuf[WIFI_DMA_RX_BUFSIZE];
INT8U  ZigbeeTxBuf[ZIGBEE_DMA_TX_BUFSIZE],ZigbeeRxBuf[ZIGBEE_DMA_RX_BUFSIZE];
INT8U  UhfidTxBuf[UHFID_DMA_TX_BUFSIZE],UhfidRxBuf[UHFID_DMA_RX_BUFSIZE];
INT16U Spd_Motor[4];
INT32U Spd_Motor_cn[4];
INT8U DevAddr,IsCenter;
INT16U ADC_CAL;
INT32U WifiLinkTime=0,ZigbeeLinkTime=0;
INT8U Status_Wifi=0,NRst_Wifi=0;
SYSSET m_sysset;
//INT16U  ADC_RegularConvertedValueTab[ADC_N_CH];
//INT16U  ADC_samplebuf[ADC_N_CH];
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

void uDelay(INT32U us)
{
  uint8_t t;
  while(us--)
  {
    t=4;
    while(t--)
    {
      __NOP();
      __NOP();  
      __NOP();      
    }
     __NOP();  
     __NOP();
     __NOP();
     __NOP();
     __NOP();   
     __NOP();    
  }
}
/*
*********************************************************************************************************
*                                         OS_CPU_SysTickClkFreq()
*
* Description : Get system tick clock frequency.
*
* Argument(s) : none.
*
* Return(s)   : Clock frequency (of system tick).
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

INT32U  OS_CPU_SysTickClkFreq (void)
{
    INT32U  freq;
    freq = BSP_CPU_ClkFreq();
    return (freq);
}
/*
*********************************************************************************************************
*                                            BSP_CPU_ClkFreq()
*
* Description : Read CPU registers to determine the CPU clock frequency of the chip.
*
* Argument(s) : none.
*
* Return(s)   : The CPU clock frequency, in Hz.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

INT32U  BSP_CPU_ClkFreq (void)
{
    RCC_ClocksTypeDef  rcc_clocks;


    RCC_GetClocksFreq(&rcc_clocks);

    return ((INT32U)rcc_clocks.HCLK_Frequency);
}


void  BSP_Init (void)
{
	INT8U i=4;
	while(i--){
		Spd_Motor[i]=0xffff;
		Spd_Motor_cn[i]=0;
	}
	IsCenter=0;
	memcpy((INT8U *)&m_sysset,(INT8U *)SYSSET_ADDR,sizeof(m_sysset));
	
	GPIO_Configuration();
	WIFI_init();
	ZIGBEE_init();
	UHFID_init();
	SpdCap_init();
	SpdPwm_init();
	ADCInit();

	SPI_init(); 
	ADXL_init();
	HMC_init();
	PS_init();
}
void SetCenter(void)
{
	IsCenter=1;
}
INT8U CheckCenter(void)
{
	if(IsCenter){
		IsCenter=0;
		return 1;
	}
	return 0;
}
/////////////
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;


//
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //2,3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 	//1
  GPIO_Init(GPIOB, &GPIO_InitStructure); 	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 	//0
  GPIO_Init(GPIOA, &GPIO_InitStructure); 		
  //led config PC10 safeline	
  RCC_APB2PeriphClockCmd(SAFELINE_APB2, ENABLE); 	
  GPIO_InitStructure.GPIO_Pin = SAFELINEPS_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SAFELINEPS_PORT, &GPIO_InitStructure);  
  GPIO_SetBits(SAFELINEPS_PORT,SAFELINEPS_PIN);
	//得到地址
  uDelay(150);
  DevAddr=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)|((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2))<<1)|((GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4))<<2)|((GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5))<<3);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //2,3
  GPIO_Init(GPIOC, &GPIO_InitStructure); 	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //运行刹车
  GPIO_Init(GPIOB, &GPIO_InitStructure); 		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //旋转刹车
  GPIO_Init(GPIOA, &GPIO_InitStructure); 	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //方向控制
  GPIO_Init(GPIOB, &GPIO_InitStructure); 		
	CloseUpRelay();
	CloseDownRelay();	
	//超声测距
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
  GPIO_Init(GPIOC, &GPIO_InitStructure); 	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	CloseUS();
	//霍尔传感器
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 		
	
	/* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI4 Line to PA.04 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI4 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void WIFI_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure; 
   USART_InitTypeDef USART_InitStructure;
   DMA_InitTypeDef           DMA_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure; 
  //rcc
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   RCC_APB2PeriphClockCmd(WIFI_RCC_APB2, ENABLE);
   //TX
   GPIO_InitStructure.GPIO_Pin = WIFI_TX_PIN; 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(WIFI_TR_PORT, &GPIO_InitStructure);  
   //RX
   GPIO_InitStructure.GPIO_Pin = WIFI_RX_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(WIFI_TR_PORT, &GPIO_InitStructure); 
   //RST
   GPIO_InitStructure.GPIO_Pin = WIFI_RST_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(WIFI_RST_PORT, &GPIO_InitStructure); 
   GPIO_SetBits(WIFI_RST_PORT,WIFI_RST_PIN);
   /* Configure the USARTx */ 
  USART_InitStructure.USART_BaudRate=WIFI_BAUND;
  USART_InitStructure.USART_WordLength=USART_WordLength_8b; // 
  USART_InitStructure.USART_StopBits=USART_StopBits_1;
  USART_InitStructure.USART_Parity=USART_Parity_No ;    
  USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;  
  USART_Init(WIFI_AF, &USART_InitStructure);

 //--------------------- 
  /* DMA1 Channel1 Configuration ----------------------------------------------*/
  //tx
  DMA_DeInit(WIFI_DMA_TX_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&WIFI_AF->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)WifiTxBuf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(WIFI_DMA_TX_Channel, &DMA_InitStructure);
  //rx
  DMA_DeInit(WIFI_DMA_RX_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&WIFI_AF->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)WifiRxBuf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = WIFI_DMA_RX_BUFSIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(WIFI_DMA_RX_Channel, &DMA_InitStructure);
  //

  /* Enable the Uart Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = WIFI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  

  /* Enable the USARTx */
  USART_DMACmd(WIFI_AF,USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
  //DMA_Cmd(WIFI_DMA_TX_Channel, ENABLE);
  DMA_Cmd(WIFI_DMA_RX_Channel, ENABLE);
  USART_Cmd(WIFI_AF, ENABLE);  
   //打开接收中断
  //DMA_ITConfig(WIFI_DMA_RX_Channel,DMA_IT_TC,ENABLE);
  USART_ITConfig(WIFI_AF, USART_IT_IDLE, ENABLE); 	//空闲中断，指示数据接受完成    
	WifiDisable();
}
void ZIGBEE_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure; 
   USART_InitTypeDef USART_InitStructure;
   DMA_InitTypeDef           DMA_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure; 
  //rcc
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   RCC_APB1PeriphClockCmd(ZIGBEE_AF_RCC_APB1, ENABLE);
   RCC_APB2PeriphClockCmd(ZIGBEE_PORT_RCC_APB2, ENABLE);
   //TX
   GPIO_InitStructure.GPIO_Pin = ZIGBEE_TX_PIN; 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(ZIGBEE_PORT, &GPIO_InitStructure);  
   //RX
   GPIO_InitStructure.GPIO_Pin = ZIGBEE_RX_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(ZIGBEE_PORT, &GPIO_InitStructure); 
   //RST
   GPIO_InitStructure.GPIO_Pin = ZIGBEE_RST_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(ZIGBEE_PORT, &GPIO_InitStructure); 
   GPIO_SetBits(ZIGBEE_PORT,ZIGBEE_RST_PIN);
   //ACK
   GPIO_InitStructure.GPIO_Pin = ZIGBEE_ACK_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(ZIGBEE_PORT, &GPIO_InitStructure); 
   /* Configure the USARTx */ 
  USART_InitStructure.USART_BaudRate=ZIGBEE_BAUND;
  USART_InitStructure.USART_WordLength=USART_WordLength_8b; // 
  USART_InitStructure.USART_StopBits=USART_StopBits_1;
  USART_InitStructure.USART_Parity=USART_Parity_No ;    
  USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;  
  USART_Init(ZIGBEE_AF, &USART_InitStructure);


 //--------------------- 
  /* DMA1 Channel1 Configuration ----------------------------------------------*/
  //tx
  DMA_DeInit(ZIGBEE_DMA_TX_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ZIGBEE_AF->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ZigbeeTxBuf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(ZIGBEE_DMA_TX_Channel, &DMA_InitStructure);
  //rx
  DMA_DeInit(ZIGBEE_DMA_RX_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ZIGBEE_AF->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ZigbeeRxBuf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ZIGBEE_DMA_RX_BUFSIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(ZIGBEE_DMA_RX_Channel, &DMA_InitStructure);
  //

  /* Enable the Uart Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = ZIGBEE_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  

  /* Enable the USARTx */
  USART_DMACmd(ZIGBEE_AF,USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
  //DMA_Cmd(ZIGBEE_DMA_TX_Channel, ENABLE);
  DMA_Cmd(ZIGBEE_DMA_RX_Channel, ENABLE);
  USART_Cmd(ZIGBEE_AF, ENABLE);  
   //打开接收中断
  //DMA_ITConfig(ZIGBEE_DMA_RX_Channel,DMA_IT_TC,ENABLE);
  USART_ITConfig(ZIGBEE_AF, USART_IT_IDLE, ENABLE); 	//空闲中断，指示数据接受完成 
	ZigbeeDisable();
}
void UHFID_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure; 
   USART_InitTypeDef USART_InitStructure;
   DMA_InitTypeDef           DMA_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure; 
  //rcc
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   RCC_APB1PeriphClockCmd(UHFID_AF_RCC_APB1, ENABLE);
   RCC_APB2PeriphClockCmd(UHFID_PORT_RCC_APB2, ENABLE);
   //TX
   GPIO_InitStructure.GPIO_Pin = UHFID_TX_PIN; 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(UHFID_PORT, &GPIO_InitStructure);  
   //RX
   GPIO_InitStructure.GPIO_Pin = UHFID_RX_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(UHFID_PORT, &GPIO_InitStructure); 
   //bp
   GPIO_InitStructure.GPIO_Pin = UHFID_BP_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(UHFID_PORT, &GPIO_InitStructure); 
   /* Configure the USARTx */ 
  USART_InitStructure.USART_BaudRate=UHFID_BAUND;
  USART_InitStructure.USART_WordLength=USART_WordLength_8b; // 
  USART_InitStructure.USART_StopBits=USART_StopBits_1;
  USART_InitStructure.USART_Parity=USART_Parity_No ;    
  USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;  
  USART_Init(UHFID_AF, &USART_InitStructure);


 /* DMA1 Channel1 Configuration ----------------------------------------------*/
  //tx
  DMA_DeInit(UHFID_DMA_TX_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UHFID_AF->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UhfidTxBuf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(UHFID_DMA_TX_Channel, &DMA_InitStructure);
  //rx
  DMA_DeInit(UHFID_DMA_RX_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UHFID_AF->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UhfidRxBuf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = UHFID_DMA_RX_BUFSIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(UHFID_DMA_RX_Channel, &DMA_InitStructure);
  //

  /* Enable the Uart Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = UHFID_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  

  /* Enable the USARTx */
  USART_DMACmd(UHFID_AF,USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
  //DMA_Cmd(UHFID_DMA_TX_Channel, ENABLE);
  DMA_Cmd(UHFID_DMA_RX_Channel, ENABLE);
  USART_Cmd(UHFID_AF, ENABLE); 
   //打开接收中断
  //DMA_ITConfig(UHFID_DMA_RX_Channel,DMA_IT_TC,ENABLE);
  USART_ITConfig(UHFID_AF, USART_IT_IDLE, ENABLE); 	//空闲中断，指示数据接受完成 

}
void SPI_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure; 
   SPI_InitTypeDef SPI_InitStructure;
//   NVIC_InitTypeDef NVIC_InitStructure; 
  //rcc
   RCC_APB1PeriphClockCmd(SPI_AF_RCC_APB1, ENABLE);
   RCC_APB2PeriphClockCmd(SPI_PORT_RCC_APB2, ENABLE);
   //SDO
   GPIO_InitStructure.GPIO_Pin = SPI_SDO_PIN; 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(SPI_AF_PORT, &GPIO_InitStructure);  
   //SDI
   GPIO_InitStructure.GPIO_Pin = SPI_SDI_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(SPI_AF_PORT, &GPIO_InitStructure); 
   //SCK
   GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(SPI_AF_PORT, &GPIO_InitStructure); 
   //CS
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Pin = SPI_CS0_PIN; 
   GPIO_Init(SPI_CS0_PORT, &GPIO_InitStructure); 
   GPIO_InitStructure.GPIO_Pin = SPI_CS1_PIN|SPI_CS2_PIN; 
   GPIO_Init(SPI_CS12_PORT, &GPIO_InitStructure); 
   GPIO_SetBits(SPI_CS0_PORT,SPI_CS0_PIN);
   GPIO_SetBits(SPI_CS12_PORT,SPI_CS1_PIN);
   GPIO_SetBits(SPI_CS12_PORT,SPI_CS2_PIN);
   //RDY INT
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_InitStructure.GPIO_Pin = SPI_RDY0_PIN|SPI_RDY1_PIN; 
   GPIO_Init(SPI_RDY01_PORT, &GPIO_InitStructure); 
   GPIO_InitStructure.GPIO_Pin = SPI_INT_PIN; 
   GPIO_Init(SPI_INT_PORT, &GPIO_InitStructure); 
   //spi
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_AF, &SPI_InitStructure);

  /* Enable SPI2 */
  SPI_Cmd(SPI_AF, ENABLE);
}
void SpdCap_init(void)
{
//   uint16_t	   PrescalerValue;
   GPIO_InitTypeDef GPIO_InitStructure; 
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   TIM_ICInitTypeDef  TIM_ICInitStructure;
   NVIC_InitTypeDef NVIC_InitStructure; 
  //rcc
   RCC_APB1PeriphClockCmd(SPDCAP_RCC_APB1, ENABLE);
   RCC_APB2PeriphClockCmd(SPDCAP_PORT_RCC_APB2, ENABLE);
  //gpio
   GPIO_InitStructure.GPIO_Pin = SPDCAP_L_PIN|SPDCAP_R_PIN|SPDCAP_T_PIN; 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(SPDCAP_PORT, &GPIO_InitStructure); 
   //  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / SPDCAP_FREQ) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(SPDCAP_TIM, &TIM_TimeBaseStructure);

  TIM_ICInitStructure.TIM_Channel = SPDCAP_T_CHL;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x3;	//ck_in N=8
  TIM_ICInit(SPDCAP_TIM, &TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = SPDCAP_L_CHL;
  TIM_ICInit(SPDCAP_TIM, &TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = SPDCAP_R_CHL;
  TIM_ICInit(SPDCAP_TIM, &TIM_ICInitStructure);

   /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SPDCAP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* TIM enable counter */
  TIM_Cmd(SPDCAP_TIM, ENABLE);
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(SPDCAP_TIM, SPDCAP_T_IT, ENABLE);
  TIM_ITConfig(SPDCAP_TIM, SPDCAP_L_IT, ENABLE);
  TIM_ITConfig(SPDCAP_TIM, SPDCAP_R_IT, ENABLE);

}

void SpdPwm_init(void)
{
 // uint16_t PrescalerValue;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  //rcc
  RCC_APB1PeriphClockCmd(SPDPWM_RCC_APB1,ENABLE);
  RCC_APB2PeriphClockCmd(SPDPWM_PORT_RCC_APB2, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SPDPWM_TY_PIN;
  GPIO_Init(SPDPWM_TY_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = SPDPWM_LR_PIN;
  GPIO_Init(SPDPWM_LR_PORT, &GPIO_InitStructure);

  //------------------------------------
  /* Compute the prescaler value */
 //PrescalerValue = (uint16_t) (SystemCoreClock / SPDPWM_CLK) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = SPDPWM_CLK/SPDPWM_FREQ-1;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / SPDPWM_CLK) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(SPDPWM_TIM, &TIM_TimeBaseStructure);
  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC1Init(SPDPWM_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(SPDPWM_TIM, TIM_OCPreload_Enable);
  TIM_OC2Init(SPDPWM_TIM, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(SPDPWM_TIM, TIM_OCPreload_Enable);
  TIM_OC3Init(SPDPWM_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(SPDPWM_TIM, TIM_OCPreload_Enable);
  TIM_OC4Init(SPDPWM_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(SPDPWM_TIM, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(SPDPWM_TIM, ENABLE);
  TIM_Cmd(SPDPWM_TIM, ENABLE);
}

//#define ADC1_DR_Address    ((uint32_t)0x4001244C)
void ADCInit(void)
{
//  NVIC_InitTypeDef NVIC_InitStructure;
  ADC_InitTypeDef           ADC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(ADC_RCC_APB2|ADC_PORT_APB2 ,ENABLE ); //使能ADC1通道时钟，各个管脚时钟
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //72M/6=12,ADC最大时间不能超过14M
  //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = ADC_XV_PIN;
  GPIO_Init(ADC_PORT, &GPIO_InitStructure);
  

    /* Configure and enable DMA interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);*/
  //----------------
/* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ADC_ExternalTrigConv_T4_CC4;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC_AF, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ //ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
  ADC_RegularChannelConfig(ADC_AF, ADC_XV_CHL, 1, ADC_SampleTime_28Cycles5 );
 
  /* Enable ADC1 */
  ADC_Cmd(ADC_AF, ENABLE);
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC_AF);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC_AF));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC_AF);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC_AF)); 
  ADC_CAL=ADC_GetConversionValue(ADC_AF);
  //------------  
  ADC_SoftwareStartConvCmd(ADC_AF,ENABLE);
}
void PS_init(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure; 
  //rcc
   RCC_APB2PeriphClockCmd(PS_APB2, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = PS_PSCK_PIN;
  GPIO_Init(PS_PORT, &GPIO_InitStructure);	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = PS_POUT_PIN;
  GPIO_Init(PS_PORT, &GPIO_InitStructure);	
	GPIO_ResetBits(PS_PORT,PS_PSCK_PIN);
	
}
/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  
  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  
  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    //IntToUnicode (Device_Serial0, &CustomHID_StringSerial[2] , 8);
    //IntToUnicode (Device_Serial1, &CustomHID_StringSerial[18], 4);
  }
}
/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}


void Reset_Device(void)
{
  NVIC_SystemReset();
}

void Wifi_Istr(void)
{
   static INT32U wifimsg[2];//	lastp=WIFI_DMA_RX_BUFSIZE;
   wifimsg[0]=(WIFI_DMA_RX_BUFSIZE-DMA_GetCurrDataCounter(WIFI_DMA_RX_Channel))|(UART_CHL_WIFI<<8);
   wifimsg[1]=(INT32U)WifiRxBuf;	
   OSMboxPost(App_UARTMbox, (void *)wifimsg);
   USART_ClearFlag(WIFI_AF,USART_FLAG_RXNE);
   USART_DMACmd(WIFI_AF,USART_DMAReq_Rx ,DISABLE);
   DMA_Cmd(WIFI_DMA_RX_Channel, DISABLE);
   DMA_SetCurrDataCounter(WIFI_DMA_RX_Channel,WIFI_DMA_RX_BUFSIZE);

	if((wifimsg[0]&0xff)>=8&&*(INT32U *)WifiRxBuf==*(INT32U *)&"AT+C"){	
			if((wifimsg[0]&0xff)<WIFI_DMA_RX_BUFSIZE)
					 WifiRxBuf[wifimsg[0]&0xff]=0;	
			if(*(INT32U *)(WifiRxBuf+4)==*(INT32U *)&"IPST"){	
				if(strstr((char *)(WifiRxBuf+8),"OK")||strstr((char *)(WifiRxBuf+8),"CONNECT"))
					Status_Wifi=1;
				else
					Status_Wifi=0;	
			}else if(*(INT32U *)(WifiRxBuf+4)==*(INT32U *)&"IPSE"){	
					if(strstr((char *)(WifiRxBuf+8),"OK"))
						Status_Wifi=2;
					else
						Status_Wifi=0;				
			}				
		}	
   DMA_Cmd(WIFI_DMA_RX_Channel, ENABLE);
   USART_DMACmd(WIFI_AF,USART_DMAReq_Rx ,ENABLE);	
	/*	switch(Status_Wifi)
		{
			case 0:
					if(strstr((char *)WifiRxBuf,"CONNECT"))
						Status_Wifi++;
				}
			break;
			case 1:
	//		if(*(INT32U *)WifiRxBuf==*(INT32U *)&"AT+C")	
	//			if((wifimsg[0]&0xff)<WIFI_DMA_RX_BUFSIZE){
	//				 WifiRxBuf[wifimsg[0]&0xff]=0;			
						if(WifiRxBuf[(wifimsg[0]&0xff)-1]=='>')
							Status_Wifi++;	
				}			
				break;				
		}
	}*/
}
void Zigbee_Istr(void)
{
   static INT32U zigbeemsg[2];//	lastp=WIFI_DMA_RX_BUFSIZE;
   zigbeemsg[0]=(ZIGBEE_DMA_RX_BUFSIZE-DMA_GetCurrDataCounter(ZIGBEE_DMA_RX_Channel))|(UART_CHL_ZIGBEE<<8);
   zigbeemsg[1]=(INT32U)ZigbeeRxBuf;
   OSMboxPost(App_UARTMbox, (void *)zigbeemsg);
   USART_ClearFlag(ZIGBEE_AF,USART_FLAG_RXNE);
   USART_DMACmd(ZIGBEE_AF,USART_DMAReq_Rx ,DISABLE);
   DMA_Cmd(ZIGBEE_DMA_RX_Channel, DISABLE);
   DMA_SetCurrDataCounter(ZIGBEE_DMA_RX_Channel,ZIGBEE_DMA_RX_BUFSIZE);
   DMA_Cmd(ZIGBEE_DMA_RX_Channel, ENABLE);
   USART_DMACmd(ZIGBEE_AF,USART_DMAReq_Rx ,ENABLE);
}
void Uhfid_Istr(void)
{
    static INT32U  uhfidmsg[2];//	lastp=WIFI_DMA_RX_BUFSIZE;
		uhfidmsg[0]=UHFID_DMA_RX_BUFSIZE-DMA_GetCurrDataCounter(UHFID_DMA_RX_Channel);
		if(uhfidmsg[0]<2)
			return;
		else if(UhfidRxBuf[0]==Uhf_ft[0]){
			if(uhfidmsg[0]<=4)
				return;		
		}
		if(*(INT32U *)UhfidRxBuf==UHFID_FINDID_ERR){
			if(uhfidmsg[0]>=8){
				 USART_ClearFlag(UHFID_AF,USART_FLAG_RXNE);
				 USART_DMACmd(UHFID_AF,USART_DMAReq_Rx ,DISABLE);
				 DMA_Cmd(UHFID_DMA_RX_Channel, DISABLE);
				 DMA_SetCurrDataCounter(UHFID_DMA_RX_Channel,UHFID_DMA_RX_BUFSIZE);
				 DMA_Cmd(UHFID_DMA_RX_Channel, ENABLE);
				 USART_DMACmd(UHFID_AF,USART_DMAReq_Rx ,ENABLE);				
			}
			return;			
		}
		if(*(INT32U *)UhfidRxBuf==UHFID_FINDID_OK){
			if(uhfidmsg[0]<UhfidRxBuf[4]+7)
				return;
		}		
//----------------
//		if(*(INT32U *)Uhf_ft!=*(INT32U *)UhfidRxBuf||*(INT32U *)(Uhf_ft+4)!=*(INT32U *)(UhfidRxBuf+4)){
		 uhfidmsg[0]|=(UART_CHL_UHFID<<8);
		 uhfidmsg[1]=(INT32U)UhfidRxBuf;	
		 OSMboxPost(App_UARTMbox, (void *)uhfidmsg);
	//	}
		
   USART_ClearFlag(UHFID_AF,USART_FLAG_RXNE);
   USART_DMACmd(UHFID_AF,USART_DMAReq_Rx ,DISABLE);
   DMA_Cmd(UHFID_DMA_RX_Channel, DISABLE);
   DMA_SetCurrDataCounter(UHFID_DMA_RX_Channel,UHFID_DMA_RX_BUFSIZE);
   DMA_Cmd(UHFID_DMA_RX_Channel, ENABLE);
   USART_DMACmd(UHFID_AF,USART_DMAReq_Rx ,ENABLE);
}
void Spi2_Istr(void)
{

}
void SpdCap_Istr(INT8U chl,INT16U cn)
{
	static INT32U lasttick[4]={0,0,0,0};
	static INT16U lastcap[4]={0,0,0,0};
	INT32U tick=OSTimeGet();
	chl=(chl>>2)&3;
	if(tick>lasttick[chl]+4000)	//4s 无脉冲认为停止
		Spd_Motor[chl]=0xffff;
	else if(cn>=lastcap[chl])
			Spd_Motor[chl]=cn-lastcap[chl];
		else
			Spd_Motor[chl]=0x10000-lastcap[chl]+cn;
	lasttick[chl]=tick;
	lastcap[chl]=cn;
	Spd_Motor_cn[chl]++;
}

//-------------------
void Wifi_AT(INT8U *buf,INT8U n)
{
		 DMA_Cmd(WIFI_DMA_TX_Channel, DISABLE);
		 MemCpy(buf,WifiTxBuf,n>WIFI_DMA_TX_BUFSIZE?WIFI_DMA_TX_BUFSIZE:n);
		 DMA_SetCurrDataCounter(WIFI_DMA_TX_Channel,n>WIFI_DMA_TX_BUFSIZE?WIFI_DMA_TX_BUFSIZE:n);
		 DMA_Cmd(WIFI_DMA_TX_Channel, ENABLE);
}
void Wifi_send(INT8U *buf,INT8U n)
{
	if(Status_Wifi>=WIFI_ST_OK)
		Wifi_AT(buf,n);
}
void Zigbee_send(INT8U *buf,INT8U n)
{
   DMA_Cmd(ZIGBEE_DMA_TX_Channel, DISABLE);
   MemCpy(buf,ZigbeeTxBuf,n>ZIGBEE_DMA_TX_BUFSIZE?ZIGBEE_DMA_TX_BUFSIZE:n);
   DMA_SetCurrDataCounter(ZIGBEE_DMA_TX_Channel,n>ZIGBEE_DMA_TX_BUFSIZE?ZIGBEE_DMA_TX_BUFSIZE:n);
   DMA_Cmd(ZIGBEE_DMA_TX_Channel, ENABLE);
}
void Uhfid_send(INT8U *buf,INT8U n)
{
   DMA_Cmd(UHFID_DMA_TX_Channel, DISABLE);
   MemCpy(buf,UhfidTxBuf,n>UHFID_DMA_TX_BUFSIZE?UHFID_DMA_TX_BUFSIZE:n);
   DMA_SetCurrDataCounter(UHFID_DMA_TX_Channel,n>UHFID_DMA_TX_BUFSIZE?UHFID_DMA_TX_BUFSIZE:n);
   DMA_Cmd(UHFID_DMA_TX_Channel, ENABLE);
}
void MemCpy(INT8U *src,INT8U *dst,INT8U n)
{
	INT8U cn=n>>2;
	while(cn--)
		*(INT32U *)(dst+(cn<<2))=*(INT32U *)(src+(cn<<2));
	cn=n&3;
	while(cn--){
		n--;
		dst[n]=src[n];
	}
}
//api
/*******************************************
*	chl=0,1,2
*	return 0-1000 每秒脉冲数放大8倍
*******************************************/
INT16U GetMotorSpd(INT8U chl)
{
	static INT16U lastre[4];
	INT16U re=0,tmp;

	if(chl==3)//速度和捕获通道不一致调整
		chl=1;
	if(chl>=4)
		chl=(chl>>2)&3;
	if(Spd_Motor[chl])
		re=(SPDCAP_FREQ<<3)/Spd_Motor[chl];
	if(re>10000)
		re=10000;
	tmp=(re+lastre[chl])>>1;
	lastre[chl]=re;
    //这里的数度需要处理成0-100
    //tmp>>=2;
	return tmp;
}
/*******************************************
*	chl=0,1,2
*	return 上电以来的脉冲计数
*******************************************/
INT32U GetMotorPoscn(INT8U chl)
{
	if(chl==3)//速度和捕获通道不一致调整
		chl=1;
	return Spd_Motor_cn[chl];
}
/*******************************************
*	chl=0,1,2,3
*	spd 速度百分百0-100
*******************************************/
void SetMotorSpd(INT8U chl,INT16U spd)
{
 	if(chl>=4)
		chl=(chl>>2)&3;
	switch(chl){
		case 0:	TIM_SetCompare1(SPDPWM_TIM,(spd*(SPDPWM_CLK/SPDPWM_FREQ))/100);break;
		case 1:	TIM_SetCompare2(SPDPWM_TIM,(spd*(SPDPWM_CLK/SPDPWM_FREQ))/100);break;
		case 2:	TIM_SetCompare3(SPDPWM_TIM,(spd*(SPDPWM_CLK/SPDPWM_FREQ))/100);break;
		case 3:	TIM_SetCompare4(SPDPWM_TIM,(spd*(SPDPWM_CLK/SPDPWM_FREQ))/100);break;
	}
}
void AdjustMotorSpd(INT8U chl,INT16S dspd,INT16S sspd)
{
  int ss;
 	if(chl>=4)
		chl=(chl>>2)&3;
	if(sspd>100)
		sspd=100;
	if(sspd<0)
		sspd=0;	
	sspd=
	sspd=MOTOR_ZERO_OFF+sspd*(SPDPWM_CLK/SPDPWM_FREQ-MOTOR_ZERO_OFF)/100;

	switch(chl){
    case 0:	
        ss=(INT16S)TIM_GetCapture1(SPDPWM_TIM)+dspd;
				if(ss>0&&ss<MOTOR_ZERO_OFF)
					ss=dspd>0?MOTOR_ZERO_OFF:0;
      TIM_SetCompare1(SPDPWM_TIM,ss<0?0:(ss>sspd?sspd:ss)); 
        break;
    case 1:	
        ss=(INT16S)TIM_GetCapture2(SPDPWM_TIM)+dspd;
				if(ss>0&&ss<MOTOR_ZERO_OFF)
					ss=dspd>0?MOTOR_ZERO_OFF:0;
        TIM_SetCompare2(SPDPWM_TIM,ss<0?0:(ss>sspd?sspd:ss));
        break;
    case 2:	
        ss=(INT16S)TIM_GetCapture3(SPDPWM_TIM)+dspd;
				if(ss>0&&ss<MOTOR_ZERO_OFF)
					ss=dspd>0?MOTOR_ZERO_OFF:0;
        TIM_SetCompare3(SPDPWM_TIM,ss<0?0:(ss>sspd?sspd:ss));
        break;
    case 3:	
        ss=(INT16S)TIM_GetCapture4(SPDPWM_TIM)+dspd;
				if(ss>0&&ss<MOTOR_ZERO_OFF)
					ss=dspd>0?MOTOR_ZERO_OFF:0;
        TIM_SetCompare4(SPDPWM_TIM,ss<0?0:(ss>sspd?sspd:ss));
        break;
	}
}
/*******************************************
*	得到外部电压
*******************************************/
INT16U GetXV(void)
{
	if(ADC_GetFlagStatus(ADC_AF,ADC_FLAG_EOC)==SET){
		ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    	return ADC_GetConversionValue(ADC_AF);
	}
	else
		return 0xffff;
}										  
void SpiRead(INT8U addr,INT8U *buf,INT8U n,INT8U sel)
{
	static INT8U lock=0;
	INT8U i;	
	while(lock)
		OSTimeDly(2);
	lock++;

	SPI_I2S_ReceiveData(SPI_AF);//clear
	switch(sel){
		case SPI_CS_HMC0:	 GPIO_ResetBits(SPI_CS0_PORT,SPI_CS0_PIN);  break;
		case SPI_CS_HMC1:	 GPIO_ResetBits(SPI_CS12_PORT,SPI_CS1_PIN); break;
		case SPI_CS_ADXL:	 GPIO_ResetBits(SPI_CS12_PORT,SPI_CS2_PIN); break;
	 }
	 if(n>1)
	 	addr|=SPI_MB;
	 while(SPI_I2S_GetFlagStatus(SPI_AF,SPI_I2S_FLAG_TXE)==RESET);
	 SPI_I2S_SendData(SPI_AF,SPI_READ|addr);
	 for(i=0;i<n;i++){
		while(SPI_I2S_GetFlagStatus(SPI_AF,SPI_I2S_FLAG_TXE)==RESET);
	 	SPI_I2S_SendData(SPI_AF,0);
		while(SPI_I2S_GetFlagStatus(SPI_AF,SPI_I2S_FLAG_RXNE)==RESET);
		if(i>0)
			buf[i-1]=SPI_I2S_ReceiveData(SPI_AF);
		else
			SPI_I2S_ReceiveData(SPI_AF);
	}
	while(SPI_I2S_GetFlagStatus(SPI_AF,SPI_I2S_FLAG_BSY)==SET);
	buf[i-1]=SPI_I2S_ReceiveData(SPI_AF);	
	
	switch(sel){
		case SPI_CS_HMC0:	 GPIO_SetBits(SPI_CS0_PORT,SPI_CS0_PIN);  break;
		case SPI_CS_HMC1:	 GPIO_SetBits(SPI_CS12_PORT,SPI_CS1_PIN); break;
		case SPI_CS_ADXL:	 GPIO_SetBits(SPI_CS12_PORT,SPI_CS2_PIN); break;
	 }
	lock--;
}
void SpiWrite(INT8U addr,INT8U *buf,INT8U n,INT8U sel)
{
	INT8U i;
	SPI_I2S_ReceiveData(SPI_AF);//clear
	switch(sel){
		case SPI_CS_HMC0:	 GPIO_ResetBits(SPI_CS0_PORT,SPI_CS0_PIN);  break;
		case SPI_CS_HMC1:	 GPIO_ResetBits(SPI_CS12_PORT,SPI_CS1_PIN); break;
		case SPI_CS_ADXL:	 GPIO_ResetBits(SPI_CS12_PORT,SPI_CS2_PIN); break;
	 }
	 if(n>1)
	 	addr|=SPI_MB;
	 while(SPI_I2S_GetFlagStatus(SPI_AF,SPI_I2S_FLAG_TXE)==RESET);
	 SPI_I2S_SendData(SPI_AF,SPI_WRITE|addr);
	 for(i=0;i<n;i++){
		while(SPI_I2S_GetFlagStatus(SPI_AF,SPI_I2S_FLAG_TXE)==RESET);
	 	SPI_I2S_SendData(SPI_AF,buf[i]);
	}
		while(SPI_I2S_GetFlagStatus(SPI_AF,SPI_I2S_FLAG_BSY)==SET);	 
		SPI_I2S_ReceiveData(SPI_AF);
	switch(sel){
		case SPI_CS_HMC0:	 GPIO_SetBits(SPI_CS0_PORT,SPI_CS0_PIN);  break;
		case SPI_CS_HMC1:	 GPIO_SetBits(SPI_CS12_PORT,SPI_CS1_PIN); break;
		case SPI_CS_ADXL:	 GPIO_SetBits(SPI_CS12_PORT,SPI_CS2_PIN); break;
	 }
}
void ADXL_init(void)
{
	INT8U buf[3];
	buf[0]=ADXL_RATE_800;
	buf[1]=ADXL_POWER_ON;
	buf[2]=ADXL_DATA_RD;	 //data reday 
	SpiWrite(ADXL_BW_RATE,buf,3,SPI_CS_ADXL);
	buf[0]=m_sysset.g_offx ;//
	buf[1]=m_sysset.g_offy ;//
	buf[2]=m_sysset.g_offz ;
	SpiWrite(ADXL_XOFF,buf,3,SPI_CS_ADXL);
}
void HMC_init(void)
{
	INT8U buf[3];
	buf[0]=0x7c;   //8n 220hz
	buf[1]=0;	   //0.88ga
	buf[2]=0;      //continue
	SpiWrite(HMC_CFGA,buf,3,SPI_CS_HMC0);
	SpiWrite(HMC_CFGA,buf,3,SPI_CS_HMC1);
}
INT8U ADXL_GetData(INT8U *buf,INT8U bufsize)
{
	 INT8U re=6;
	 if(bufsize<6)
	 	re=bufsize;
	 SpiRead(ADXL_DATAX0,buf,re,SPI_CS_ADXL);
	 return re;	 
}
INT8U HMC_GetData(INT8U *buf,INT8U bufsize)
{
	 INT8U re=12,t,i;
	 if(bufsize<12)
	 	re=bufsize;
	 re&=0xfe;
	 SpiRead(HMC_DOUTXM,buf,re>>1,SPI_CS_HMC0);
	 SpiRead(HMC_DOUTXM,(re>>1)+buf,re>>1,SPI_CS_HMC1);
	 
	 for(i=0;i<re;i+=2){
		 t=buf[i];
		 buf[i]=buf[i+1];
		 buf[i+1]=t;
	 }
	 return re;
}

INT8U GetPSData(INT8U chl,INT32S *data)
{
	INT8U i;
	if(GPIO_ReadInputDataBit(PS_PORT,PS_POUT_PIN)){
		GPIO_ResetBits(PS_PORT,PS_PSCK_PIN);
		return 0;
	}
	*data=0;
	if(chl)
		chl=2;
	else
		chl=1;
	for(i=0;i<24+chl;i++){
		GPIO_SetBits(PS_PORT,PS_PSCK_PIN);	
		*data<<=1;
		*data|=GPIO_ReadInputDataBit(PS_PORT,PS_POUT_PIN);				
		GPIO_ResetBits(PS_PORT,PS_PSCK_PIN);
		*data&=0xffffffff;//无效操作，延长时间
	}
	*data>>=chl;
	if(*data>>23)
		*data|=0xff000000;
	return chl;
}
void WifiLink(void)
{
#ifdef NOAUTOLINK
	return;
#endif
	if(OSTimeGet()>WIFI_OVTETIME+WifiLinkTime){
		if(NRst_Wifi++>WIFI_NRST){			
			NRst_Wifi=0;
			WifiDisable();
			OSTimeDly(10);	
			WifiEnable();
			OSTimeDly(200);				
		}		
		Wifi_SetReSrv();
		WifiLinkTime+=WIFI_OVTETIME;
	}
}
void ZigbeeLink(void)
{
	if(OSTimeGet()>ZIGBEE_OVTETIME+ZigbeeLinkTime){
			ZigbeeDisable();
			OSTimeDly(10);	
			ZigbeeEnable();
			OSTimeDly(100);				
			ZigbeeLinkTime+=ZIGBEE_OVTETIME;
	}
}
void SetWifiLinkTime(void)
{
	WifiLinkTime=OSTimeGet();
}
void SetZigbeeLinkTime(void)
{
	ZigbeeLinkTime=OSTimeGet();
}
void Wifi_SetReSrv(void)
{
	//INT8U buf[64]={'+','+','+'};
	Status_Wifi=0;
	Wifi_AT(WIFI_OUTSEND,sizeof(WIFI_OUTSEND)-1);//字符串不发结束符
	OSTimeDly(30);
	//strcpy(buf,WIFI_RESRV,sizeof(WIFI_RESRV);
	Wifi_AT(WIFI_SETRESRV,sizeof(WIFI_SETRESRV)-1);	
	OSTimeDly(50);	
//	Wifi_send(WIFI_INTOSEND,sizeof(WIFI_INTOSEND)-1);		
//	OSTimeDly(2);	
	Wifi_AT(WIFI_STARTSEND,sizeof(WIFI_STARTSEND)-1);			

}
INT8U IsNetData(INT8U *buf,INT8U len)
{
	INT8U i=0,n=0;
	while(i+2<len){
		if(buf[i]==FRAME_HEAD&&buf[i+1]+i+2<=len&&buf[buf[i+1]+i+1]==FRAME_END&&(buf[i+3]==DevAddr||buf[i+3]==FRAME_BROAD)){
			n++;
			i+=buf[i+1]+2;
			Status_Wifi=WIFI_ST_OK;
			NRst_Wifi=0;
		}
		else
			break;
	}
	return n;
}
INT8U WifiStatus(void)
{
	return Status_Wifi;
}
INT8U GetAddr(void)
{
	return DevAddr;
}
//-------------------------------
INT8U CheckSum8(INT8U *buf,INT32U len)
{
	uint8_t re=0;
	while(len--)
	{
		re+=buf[len];
	}
	return re;
}
INT8U GetCmd(INT8U *buf,INT8U **rbuf,INT8U *sp,INT8U len)
{
	INT8U re=0;
	if(*sp+6>=len)
		return 0;
	if(buf[*sp]==FRAME_HEAD&&buf[*sp+1]+*sp+2<=len&&buf[buf[*sp+1]+*sp+1]==FRAME_END&&(buf[*sp+3]==DevAddr||buf[*sp+3]==FRAME_BROAD)){
		if(buf[buf[*sp+1]+*sp]==CheckSum8(buf+*sp,buf[*sp+1])){
			*rbuf=buf+*sp+4;
			re=buf[*sp+1]-4;
			*sp+=buf[*sp+1]+2;
			return re;
		}
	}
	return 0;
}
void GetCfgMd5(INT32U *buf)
{
	buf[0]=*(INT32U *)CFGSYS_FLASH_CFG_MD5;
	buf[1]=*(INT32U *)(CFGSYS_FLASH_CFG_MD5+4);
	buf[2]=*(INT32U *)(CFGSYS_FLASH_CFG_MD5+8);
	buf[3]=*(INT32U *)(CFGSYS_FLASH_CFG_MD5+12);	
}
void SetCfgMd5(INT32U *buf)
{
	FLASH_Unlock();
	FLASH_If_Write(CFGSYS_FLASH_CFG_MD5,(INT8U *)buf,16);
	FLASH_Lock();	
}
INT32U CheckCfgSum(void)
{
	INT32U re=0,n=0,addr;
	for(addr=CFGSYS_FLASH_CFG_NP;addr<CFGSYS_FLASH_CFG_BODY-4;addr+=4){
		if(*(INT16U *)addr!=0xffff){
			re+=CheckSum8((INT8U *)(CFGSYS_FLASH_CFG_BODY+*(INT16U *)addr),*(INT8U *)(addr+2));	
			n++;
		}
		else	
			break;
	}
	return (re&0xfffff)|(n<<20);
}
void CfgClear(void)
{
	INT32U addr=CFGSYS_FLASHADDR_START;
	FLASH_Unlock();
	for(;addr<CFGSYS_FLASHADDR_END;addr+=PAGE_SIZE)
		FLASH_If_Erase(addr);
	FLASH_Lock();	
}
INT16U CfgWriteId(INT8U *buf,INT8U len)
{
	INT32U addr;
	INT8U tbuf[4];
	INT16U n=0;
	FLASH_Unlock();
	for(addr=CFGSYS_FLASH_CFG_NP;addr<CFGSYS_FLASH_CFG_BODY-4;addr+=4){
			tbuf[2]=len;
		if(*(INT16U *)addr==0xffff){
			if(addr==CFGSYS_FLASH_CFG_NP)
				*(INT16U *)tbuf=0;
			else
				*(INT16U *)tbuf=*(INT16U *)(addr-4)+*(INT8U *)(addr-2);				
			
			if(*(INT16U *)tbuf&1)
				(*(INT16U *)tbuf)++;
			if(CFGSYS_FLASH_CFG_BODY+*(INT16U *)tbuf+len<=(CFGSYS_FLASHADDR_END&0xfffffffc)){
				FLASH_If_Write(addr,tbuf,3);				
				FLASH_If_Write(CFGSYS_FLASH_CFG_BODY+*(INT16U *)tbuf,buf,len);

				n++;
			}
			break;
		}
		else
			n++;
	}
	FLASH_Lock();	
	return n;
}
RFID * GetRfidStruct(INT16U id)
{
	INT32U addr;
	RFID *rp=0;	
	for(addr=CFGSYS_FLASH_CFG_NP;addr<CFGSYS_FLASH_CFG_BODY;addr+=4){
		if(*(INT16U *)addr==0xffff)
			return 0;
		else if(CFGSYS_FLASH_CFG_BODY+*(INT16U *)addr+*(INT8U *)(addr+2)>CFGSYS_FLASHADDR_END)
				continue;
		else
		{
			rp=(RFID *)(CFGSYS_FLASH_CFG_BODY+*(INT16U *)addr);
			if(rp->id==id)
				return rp;
		}
	}
	return 0;	
}
void UhfidSwitch(INT8U cmd)
{
	if(cmd==UHFID_CMD_OPEN)
		Uhfid_send((INT8U *)Uhfid_open,sizeof(Uhfid_open));
	else if(cmd==UHFID_CMD_CLOSE)
			Uhfid_send((INT8U *)Uhfid_close,sizeof(Uhfid_close));	
}

/*
COM12, Read(1): AA  | a
COM12, Read(1): 02  | \#2
COM12, Read(1): 22  | "
COM12, Read(2): 00 11  | \#0\#17
COM12, Read(17): C0 30 00 E2 00 10 00 78 05 02 48 11 30 A6 A3 53 03  | à0\#0a\#0\#16\#0x\#5\#2H\#170|￡S\#3
COM12, Read(1): BE  | ?
COM12, Read(1): 8E  | ?
*/
INT32U ParseUhfid(INT8U *buf,INT8S *RSSI,INT8U len)
{
	INT8U n;
	if(*(INT32U *)buf==UHFID_FINDID_OK){
		n=buf[4]+7;
		if(n<=len&&buf[n-1]==UHFID_FRAME_END){
			*RSSI=buf[5];
			return *(INT32U *)(buf+8);
		}
	}
	return 0;
}
IDCMD *FindUhfidCmd(RFID *p_rfid)
{
	static INT8U n_cmd=0;
	IDCMD *	p_ic=&p_rfid->p_idcmd;
	p_ic+=n_cmd;
	if(p_rfid->n_idcmd<=++n_cmd)
		n_cmd=0;
	return p_ic;
}
void SaveSysSet(void)
{
	FLASH_Unlock();
	FLASH_If_Erase(SYSSET_ADDR);
	FLASH_If_Write(SYSSET_ADDR,(INT8U *)&m_sysset,sizeof(m_sysset));
	FLASH_Lock();	

}

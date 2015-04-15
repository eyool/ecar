/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/
#ifndef BSP_H
#define BSP_H

#include  <stm32f10x_conf.h>
#include  <app_cfg.h>
#include  <ucos_ii.h>


typedef struct _IDCMD{
	INT16S dis,tick;	/*tick��λ��0.1��*/
	INT8U cmd,type,spd,runtime;
}IDCMD;
typedef struct _RFID{
	INT32S pos;	
	INT16U id,safedis;	
	INT8U  area,n_idcmd;
	IDCMD  p_idcmd;
}RFID;
typedef struct _SYSSET{
	INT16S g_offx,g_offy,g_offz,rev;
	INT16S	 hmc_off;
	INT16U rfid;

}SYSSET;




#define PI      3.141592653
//wifi
#define WIFI_BAUND				 115200
#define WIFI_AF                  USART1
#define WIFI_RCC_APB2			 RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD 			 	
#define WIFI_TR_PORT		 	 GPIOA
#define WIFI_TX_PIN				 GPIO_Pin_9
#define WIFI_RX_PIN		 		 GPIO_Pin_10
#define WIFI_RST_PORT			 GPIOD
#define WIFI_RST_PIN			 GPIO_Pin_2

#define	WIFI_IRQn 				 USART1_IRQn
#define WIFI_DMA_TX_Channel		 DMA1_Channel4
#define WIFI_DMA_RX_Channel		 DMA1_Channel5
#define WIFI_DMA_TX_BUFSIZE		 256
#define WIFI_DMA_RX_BUFSIZE		 256


//zigbee
#define ZIGBEE_BAUND				 115200
#define ZIGBEE_AF                USART2
#define ZIGBEE_AF_RCC_APB1		 RCC_APB1Periph_USART2
#define ZIGBEE_PORT_RCC_APB2	 RCC_APB2Periph_GPIOA
#define ZIGBEE_PORT		  	 	 GPIOA
#define ZIGBEE_RST_PIN			 GPIO_Pin_0
#define ZIGBEE_ACK_PIN		     GPIO_Pin_1
#define ZIGBEE_TX_PIN			 GPIO_Pin_2
#define ZIGBEE_RX_PIN		     GPIO_Pin_3

#define	ZIGBEE_IRQn 			 USART2_IRQn
#define ZIGBEE_DMA_TX_Channel		 DMA1_Channel7
#define ZIGBEE_DMA_RX_Channel		 DMA1_Channel6
#define ZIGBEE_DMA_TX_BUFSIZE		 256
#define ZIGBEE_DMA_RX_BUFSIZE		 256
//uhfid
#define UHFID_BAUND			 	 115200
#define UHFID_AF                 USART3
#define UHFID_AF_RCC_APB1		 RCC_APB1Periph_USART3
#define UHFID_PORT_RCC_APB2 	 RCC_APB2Periph_GPIOB
#define UHFID_PORT		 	 	 GPIOB
#define UHFID_BP_PIN			 GPIO_Pin_9
#define UHFID_TX_PIN			 GPIO_Pin_10
#define UHFID_RX_PIN		     GPIO_Pin_11

#define	UHFID_IRQn 			 	 USART3_IRQn
#define UHFID_DMA_TX_Channel		 DMA1_Channel2
#define UHFID_DMA_RX_Channel		 DMA1_Channel3
#define UHFID_DMA_TX_BUFSIZE		 256
#define UHFID_DMA_RX_BUFSIZE		 256

#define UART_CHL_ZIGBEE		0
#define UART_CHL_WIFI			1
#define UART_CHL_UHFID		2
//SPI BUS
#define SPI_BAUND				1000000
#define SPI_AF					SPI2
#define SPI_AF_RCC_APB1			RCC_APB1Periph_SPI2 
#define SPI_PORT_RCC_APB2	    RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC
#define SPI_AF_PORT		    	GPIOB
#define SPI_SCK_PIN				GPIO_Pin_13
#define SPI_SDI_PIN				GPIO_Pin_14
#define SPI_SDO_PIN				GPIO_Pin_15
#define SPI_CS0_PORT			GPIOB
#define SPI_CS12_PORT			GPIOC
#define SPI_INT_PORT			GPIOA
#define SPI_RDY01_PORT			GPIOC
#define SPI_CS0_PIN				GPIO_Pin_12
#define SPI_CS1_PIN				GPIO_Pin_9
#define SPI_CS2_PIN				GPIO_Pin_8
#define SPI_RDY0_PIN			GPIO_Pin_6
#define SPI_RDY1_PIN			GPIO_Pin_7
#define SPI_INT_PIN		  		GPIO_Pin_8

#define	SPI_IRQn 			 	SPI2_IRQn
#define	SPI_READ				0x80
#define	SPI_WRITE				0x00
#define	SPI_MB					0x40
#define SPI_CS_HMC0				0
#define SPI_CS_HMC1				1
#define SPI_CS_ADXL				2
#define	ADXL_XOFF				0x1e
#define	ADXL_BW_RATE		0x2c
#define	ADXL_POWER_CTL		0x2d
#define	ADXL_INT_ENABLE		0x2e
#define	ADXL_DATA_FORMAT	0x31
#define	ADXL_DATAX0			0x32
//#define	ADXL_DATAX1			0x33
#define	ADXL_DATAY0			0x34
//#define	ADXL_DATAY1			0x35
#define	ADXL_DATAZ0			0x36
//#define	ADXL_DATAZ1			0x37
#define	ADXL_FIFO_CTL 			0x38
#define	ADXL_FIFO_STATUS 		0x39
#define	ADXL_RATE_800		0xd
#define	ADXL_POWER_ON		0x8
#define	ADXL_DATA_RD		0x80
#define	ADXL_FULL      0x8
#define	ADXL_4G        0x1

#define HMC_CFGA 	0x00  /*Configuration Register A rw*/
#define HMC_CFGB 	0x01  /*Configuration Register B rw*/
#define HMC_MODE 	0x02  /*Mode Register		   rw*/
#define HMC_DOUTXM 	0x03  /*Data Output X MSB Register r*/
#define HMC_DOUTXL 	0x04  /*Data Output X LSB Register r*/
#define HMC_DOUTZM 	0x05  /*Data Output Z MSB Register  r*/
#define HMC_DOUTZL 	0x06  /*Data Output Z LSB Register	 r*/
#define HMC_DOUTYM 	0x07  /*Data Output Y MSB Register 	 r*/
#define HMC_DOUTYL 	0x08  /*Data Output Y LSB Register 	 r*/
#define HMC_STATUS 	0x09  /*Status Register 			r	*/
#define HMC_IRA 	0x0A  /*Identification Register A  	 r*/
#define HMC_IRB 	0x0B  /*Identification Register B	r	*/
#define HMC_IRC 	0x0C  /*Identification Register C  	r	 */
#define HMC_TPOUTM 	0x31  /*Temperature Output MSB Register r	*/
#define HMC_TPOUTL 	0x32  /*Temperature Output LSB Register r	 */
//�ٶȲ���
#define SPDCAP_FREQ				8000
#define SPDCAP_TIM				TIM4
#define SPDCAP_RCC_APB1			RCC_APB1Periph_TIM4
#define SPDCAP_PORT_RCC_APB2	RCC_APB2Periph_GPIOB
#define SPDCAP_PORT				GPIOB
#define SPDCAP_L_PIN			GPIO_Pin_7
#define SPDCAP_R_PIN			GPIO_Pin_8
#define SPDCAP_T_PIN			GPIO_Pin_6
#define SPDCAP_T_CHL			TIM_Channel_1
#define SPDCAP_L_CHL			TIM_Channel_2
#define SPDCAP_R_CHL			TIM_Channel_3
#define SPDCAP_T_IT				TIM_IT_CC1
#define SPDCAP_L_IT				TIM_IT_CC2
#define SPDCAP_R_IT				TIM_IT_CC3

#define SPDCAP_IRQn				TIM4_IRQn

//pwm�ٶ����
#define SPDPWM_CLK				12000000	
#define SPDPWM_FREQ				50000
#define SPDPWM_TIM				TIM3
#define SPDPWM_RCC_APB1			RCC_APB1Periph_TIM3
#define SPDPWM_PORT_RCC_APB2	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB
#define SPDPWM_TY_PORT			GPIOA
#define SPDPWM_LR_PORT			GPIOB
#define SPDPWM_TY_PIN		    GPIO_Pin_6|GPIO_Pin_7
//#define SPDPWM_Y_PIN			GPIO_Pin_7
#define SPDPWM_LR_PIN			GPIO_Pin_0|GPIO_Pin_1
//#define SPDPWM_R_PIN			GPIO_Pin_0
#define SPDPWM_T_CHL			TIM_Channel_1
#define SPDPWM_Y_CHL			TIM_Channel_2
#define SPDPWM_R_CHL			TIM_Channel_3
#define SPDPWM_L_CHL			TIM_Channel_4



//#define ADCTS_FREQ              12000
//#define ADC_N_CH                10
#define ADC_AF					ADC1
#define	ADC_RCC_APB2            RCC_APB2Periph_ADC1
#define	ADC_PORT_APB2           RCC_APB2Periph_GPIOC
#define	ADC_PORT                GPIOC
#define	ADC_XV_PIN	            GPIO_Pin_0
#define	ADC_XV_CHL	            ADC_Channel_10
//ѹ��������
#define	PS_APB2           			 RCC_APB2Periph_GPIOC
#define PS_PORT                  GPIOC
#define PS_POUT_PIN              GPIO_Pin_11
#define PS_PSCK_PIN              GPIO_Pin_12

#define PS_CHL_A			0
#define PS_CHL_B			1
//safe line
#define	SAFELINE_APB2           			 RCC_APB2Periph_GPIOC
#define SAFELINEPS_PORT                  GPIOC
#define SAFELINEPS_PIN                 GPIO_Pin_10
//addr
//#define	DEVADDRB_APB2           			 RCC_APB2Periph_GPIOB
//#define	DEVADDRC_APB2           			 RCC_APB2Periph_GPIOC
//#define	DEVADDRB_PORT           			 GPIOB
//#define	DEVADDRC_PORT           			 GPIOC
//esp8266  
#define WIFI_LINK_OK		1
#define WIFI_ST_OK			2
#define WIFI_OVTETIME   10000
#define WIFI_NRST    		5
#define WIFI_SETRESRV "AT+CIPSTART=\"TCP\",\"192.168.100.3\",8880\r\n"
//#define WIFI_INTOSEND "AT+CIPMODE=\r\n"
#define WIFI_OUTSEND "+++"
#define WIFI_STARTSEND "AT+CIPSEND\r\n"

#define ZIGBEE_OVTETIME   2000

//AA 00 F5 00 01 00 F6 8E
//AA 00 27 00 02 01 F4 1E 8E
//AA 00 28 00 00 28 8E 
#define UHFID_CMD1 		 {0xAA, 00 ,0xF5 ,00, 0x01, 0x00, 0xF6 ,0x8E}
#define UHFID_OPENDEV  {0xAA, 00, 0x27, 00, 0x02, 0x01, 0xF4, 0x1E, 0x8E} 
#define UHFID_CLOSEDEV {0xAA, 00, 0x28, 00, 0x00, 0x28, 0x8E} 
#define UHFID_FILTER   {0xAA, 01,	0xFF, 00, 0x01, 0x15, 0x16, 0x8E}
#define UHFID_FINDID_OK   0x002202aa
#define UHFID_FINDID_ERR  0x00ff01aa
#define UHFID_FRAME_END   0x8e

#define MOTOR_ZERO_OFF  0x50
#define MOTOR_TILT_ZERO_OFF  0x28
//���� 
void SetCenter(void);


void WIFI_init(void);
void ZIGBEE_init(void);
void UHFID_init(void);
void SPI_init(void);
void SpdCap_init(void);
void SpdPwm_init(void);
void ADCInit(void);

// �жϷ������
void Wifi_Istr(void);
void Zigbee_Istr(void);
void Uhfid_Istr(void);
void Spi2_Istr(void);
void SpdCap_Istr(INT8U chl,INT16U cn);

//INT32U  OS_CPU_SysTickClkFreq(void);
void 		 uDelay(INT32U us);
void         BSP_Init(void);
void         BSP_IntDisAll (void);
INT32U       BSP_CPU_ClkFreq(void);
void         GPIO_Configuration(void);
void Get_SerialNum(void);
void Reset_Device(void);

void SpiRead(INT8U addr,INT8U *buf,INT8U n,INT8U sel);
void SpiWrite(INT8U addr,INT8U *buf,INT8U n,INT8U sel);


void ADXL_init(void);
void HMC_init(void);
void PS_init(void);

#define FRAME_HEAD 0xa5
#define FRAME_END 0x8f
#define FRAME_BROAD 0xff


#define KEY_GO		1
#define KEY_JOIN		2
#define KEY_REV			4
/*******************************************
*Wifi ����Զ�̷����� Ĭ��192.168.100.3
*******************************************/
void Wifi_SetReSrv(void);
//����WIFI�������ӵ�ϵͳʱ��
void SetWifiLinkTime(void);
//����zigbee�������ӵ�ϵͳʱ��
void SetZigbeeLinkTime(void);
//����ϵͳ�������������
INT8U GetCmd(INT8U *buf,INT8U **rbuf,INT8U *sp,INT8U len);
void GetCfgMd5(INT32U *buf);
void SetCfgMd5(INT32U *buf);
void CfgClear(void);
INT16U CfgWriteId(INT8U *buf,INT8U len);
INT32U CheckCfgSum(void);
void StartIWDG(void);
/**********************************************************************************************************
*api  ϵͳ�ṩAPI ����ӦӦ�ó������
*
********************************************************************************************************/



#define OpenUpRelay() 		GPIO_SetBits(GPIOC,GPIO_Pin_4)/*������Һѹ��ŷ�*/
#define OpenDownRelay() 	GPIO_SetBits(GPIOC,GPIO_Pin_5)/*���½�Һѹ��ŷ�*/
#define CloseUpRelay() 		GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define CloseDownRelay() 	GPIO_ResetBits(GPIOC,GPIO_Pin_5)
#define CloseAllRelay()   GPIO_ResetBits(GPIOC,GPIO_Pin_4);GPIO_ResetBits(GPIOC,GPIO_Pin_5);
#define OpenRunBreak()    GPIO_SetBits(GPIOB,GPIO_Pin_2)/*ɲ������*/
#define CloseRunBreak()   GPIO_ResetBits(GPIOB,GPIO_Pin_2)
#define OpenTurnBreak()   GPIO_SetBits(GPIOA,GPIO_Pin_5)/*ɲ������*/
#define CloseTurnBreak()  GPIO_ResetBits(GPIOA,GPIO_Pin_5)
#define GetTurnBreak()		GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5)
#define GetTurnDir()		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_5)
#define OpenUS() 					GPIO_SetBits(GPIOC,GPIO_Pin_2)  /*�򿪳������*/
#define CloseUS() 				GPIO_ResetBits(GPIOC,GPIO_Pin_2) 
#define GetUS() 					GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)/*������Ӧ��״̬*/
#define SetSafeLine() 			GPIO_SetBits(GPIOC,GPIO_Pin_10)
#define ResetSafeLine() 	  GPIO_ResetBits(GPIOC,GPIO_Pin_10)
#define GetKey()           (GPIO_ReadInputData(GPIOC)>>13)
//#define GetHCTrig() 			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)/*����λ�û�������*/
#define WifiEnable()         GPIO_SetBits(WIFI_RST_PORT,WIFI_RST_PIN)
#define WifiDisable()   		  GPIO_ResetBits(WIFI_RST_PORT,WIFI_RST_PIN)
#define ZigbeeEnable()       GPIO_SetBits(ZIGBEE_PORT,ZIGBEE_RST_PIN)
#define ZigbeeDisable()   		GPIO_ResetBits(ZIGBEE_PORT,ZIGBEE_RST_PIN)
//����Ƕȷ���-1800-1799 ����0.1��
#define GetArc(a,b) (INT16S)(atan2((float)(a),(float)(b))*1800.0/PI)
/*******************************************
*	�ڴ濽�� src->dst
*******************************************/
void MemCpy(INT8U *src,INT8U *dst,INT8U n);
INT8U CheckSum8(INT8U *buf,INT32U len);
/*******************************************
*	buf ���ͻ�����ָ��
*	n   �ֽ���
*******************************************/
void Wifi_AT(INT8U *buf,INT8U n);
void Wifi_send(INT8U *buf,INT8U n);
void Zigbee_send(INT8U *buf,INT8U n);
void Uhfid_send(INT8U *buf,INT8U n);


/*******************************************
*	chl=0,1,2
*	return 0-1000 ÿ���������Ŵ�10��
*******************************************/
INT16U GetMotorSpd(INT8U chl);
/*******************************************
*	chl=0,1,2
*	return �ϵ��������������
*******************************************/
INT32U GetMotorPoscn(INT8U chl);
/*******************************************
*	chl=0,1,2
*	spd �ٶȰٷְ�0-100
*******************************************/
void SetMotorSpd(INT8U chl,INT16U spd); 
/*******************************************
*	chl=0,1,2,3
*	dspd �ٶȵ��ڲ�ֵ
* sspd  �趨������
*******************************************/
void AdjustMotorSpd(INT8U chl,INT16S dspd,INT16S sspd); 
/*******************************************
*	�õ��ⲿ��ѹ
*******************************************/
INT16U GetXV(void);

/*******************************************
*	ADXL345��ȡ����	,bufsize=6,ȡȫ������
*******************************************/
INT8U ADXL_GetData(INT8U *buf,INT8U bufsize);
/*******************************************
*	hmc5983��ȡ����,bufsize=12,ȡȫ������ǰ6��Ϊһ��ģ�飬��6��Ϊ����һ��
*******************************************/
INT8U HMC_GetData(INT8U *buf,INT8U bufsize);
/*******************************************
*	ѹ����������ȡ����,
* chl=0��Aͨ��,1��Bͨ��
* return 0: ��Ч
*        ��0����Ч
*******************************************/
INT8U GetPSData(INT8U chl,INT32S *data);
/*******************************************
*	�жϳ����Ƿ�ص�����λ�ã�1�����ģ�0��������
*******************************************/
INT8U CheckCenter(void);
// �ж�wifi�Ƿ�����,��������� �Զ���������
void WifiLink(void);
// �ж�wifi״̬
INT8U WifiStatus(void);
// �ж�zigbee�Ƿ�����,��������� �Զ���������
void ZigbeeLink(void);


//�ж��ǲ������߲��ֵ����ݣ������м�֡����
INT8U IsNetData(INT8U *buf,INT8U len);
//�õ�������ַ
INT8U GetAddr(void);
//��ѯID�����ݣ������ҵ��Ľṹ���ַ��0��û�ҵ�
RFID * GetRfidStruct(INT16U id);
//uhfid command ���� �ر�
#define UHFID_CMD_OPEN	1
#define UHFID_CMD_CLOSE	0
void UhfidSwitch(INT8U cmd);

//�õ�������
#define RDID_BITS	0xffff
INT32U ParseUhfid(INT8U *buf,INT8S *RSSI,INT8U len);

IDCMD *FindUhfidCmd(RFID *p_rfid);

void SaveSysSet(void);
/*�������*/
void TurnLeft(void);  
void TurnRight(void);    
//��֪����׼������
void UARTMboxPost(INT8U n,INT8U chl);
//����������ת��ƽ��
void AdjustMotorBalance(INT32S dps);
//�������
void CalcUS(void);
//�õ�����
INT16U GetFrontSafeDis(void);
//api end
#endif

#ifndef __APP_H
#define __APP_H
#include "string.h"
  // *(unsigned int*)(0x1FFFF7EC);
  //*(unsigned int*)(0x1FFFF7F0);
//  Device_Serial0=Device_Serial0^Device_Serial1;
//  Device_Serial1=Device_Serial1^Device_Serial2;
//  Device_Serial2=Device_Serial2^0xf5125f78;   

typedef unsigned int   UINT;
extern OS_EVENT      *App_StartMbox;


typedef struct _SENSOR{
	INT32S		ps;		/*ѹ��ֵ*/
//	INT32S		pos;/*���������λ�� ��cm*/		
	INT32U 	r_np[2],t_np;/*ǰ������,�Ƕȼ���*/
	INT16S		tilt;/*���-1800-1799*/
	INT16S		rotation[2];/*ת��-1800-1799*/	
	INT16S		runspeed[2];/*����ת�٣�*/
	INT16S		turnspeed;	/*��תspeed,*/
	INT16S		usdis;/*ǰ�����о���*/
	INT16S		xv;	
	INT16U	b_hall;/*����������״̬*/

	
}SENSOR;

#define CAR_MAX	16
typedef struct _CAR{
	INT32U pos,rfid_np,g_tick,l_tick;/*g_tick��ÿ�����п�ʼʵ�ʣ�l_tick����ID���º���¼�ʱ */
	INT8U allcarid[CAR_MAX];
	INT32U allcarpos[CAR_MAX];
//	INT16S st_angle,st_turn,st_run;
	RFID *p_rfid[2];/*����2�ο�ID*/ 
	INT16U spd;
	INT8U cid,addr,status,err;
}CAR;

/*typedef struct _CARSET{
	INT16U runspd,turnspd;
	
	
}CARSET*/


#define CAR_CMD_RUN    'r'
#define CAR_CMD_TURN   't'
#define CAR_CMD_ANGLE  'a'
#define CAR_CMD_PLAY   'p'

#define B_HALL_C 		1

#define MOTOR_RL  	 2
#define MOTOR_RR  	 3
#define MOTOR_TURN  0
#define MOTOR_TILT    1
//#define PROC_DTIME  		50
#define SYS_TBUF_LEN			1040
//
#define NET_CHL_WIFI  		1
#define NET_CHL_ZIGBEE	  2
#define NET_CHL_ALL				3	

//
#define C_PC_POS				1 
#define C_PC_CFG_MD5		2 
//#define C_PC_CFG_MD5_OK		1 
#define C_PC_CFG_DL			3 
#define C_PC_CFG_JOIN		4 
#define C_CAR_REG				5 
#define C_CAR_UNREG			6 

#define C_PC_CFG_DL_ERR		0xffff 
#define C_PC_CFG_DL_OK		0xfffe 


#define PROGRAMM_KEY  0xa55a9527


#define SYS_STATUS_INIT_OK		0x0
#define SYS_STATUS_INIT_ERR		0x1

#define CAR_STATUS_NULL			0
#define CAR_STATUS_INIT			1
#define CAR_STATUS_JOIN			2
#define CAR_STATUS_WAITCMD		3
#define CAR_STATUS_RUN			4
#define CAR_STATUS_BACK			5

#define CAR_AREA_NULL			0
#define CAR_AREA_START			1
#define CAR_AREA_PLAY			2
#define CAR_AREA_GETOFF			3
#define CAR_AREA_BACK			4
#define CAR_AREA_STOP			5
#define CAR_AREA_GETON			6
//-----------------------
#define POSMODIFY(a)  (a*2>>8)/*��Ҫʵ�ʲ�������*/
#define SPDMODIFY(a)  (a*2>>8)
void App_init(void);
void MSDelay(UINT ms);//������ms
void UartRecvProc(uint8_t chl,uint8_t *buf,uint32_t len);
void MainTaskProc(void *p_msg);
void AppRunProc(void  *p_msg);
void SensorProc(void);
void nopf(void);
void Debug(INT8U *buf,INT8U len);
INT8U CheckSelf(void);
INT8U DowloadCFG(void);
void NetSend(INT8U len,INT8U chl);
void NetCmdProc(INT8U *buf,INT8U len);
void RunCmdProc(RFID *p_rfid);
void RunCtrl(IDCMD *p_ic);
void TurnCtrl(IDCMD *p_ic);
void TiltCtrl(IDCMD *p_ic);
void SW_Power(void);
int   RegisterCar(INT8U id,INT32U pos);
void UnRegisterCar(INT8U id);
INT32U GetFrontDis(void);
#endif

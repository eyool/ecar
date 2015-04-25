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
extern SYSSET m_sysset;


typedef struct _SENSOR{
	INT32S		ps;		/*压力值*/
//	INT32S		pos;/*车体离起点位置 ，cm*/		
	INT32U 	r_np[2],t_np,lastt_np;/*前进计数,角度计数,旋转上次记录计数*/
	INT16S		tilt;/*倾角-1800-1799*/
	INT16S		rotation[2];/*转角-1800-1799*/	
	INT16S		runspeed[2];/*轮子转速，*/
	INT16S		turnspeed;	/*旋转speed,*/
	INT16S		usdis;/*前方空闲距离*/
	INT16S		xv;	
	INT16U	b_hall;/*霍尔传感器状态*/

	
}SENSOR;

#define CAR_MAX	16
typedef struct _CAR{
	INT32U pos,rfid_np,g_tick,l_tick;/*g_tick：每次运行开始实际，l_tick：卡ID更新后从新计时 */
	INT32S turnangle_np;
	INT8U allcarid[CAR_MAX];
	INT32U allcarpos[CAR_MAX];
//	INT16S st_angle,st_turn,st_run;
	RFID *p_rfid[2];/*保存2次卡ID*/ 
	INT16U spd,frontsafedis;//,rfid;
	INT8U cid,status,laststatus,err;
	INT8S RSSI;
}CAR;




#define CAR_CMD_RUN    'r'
#define CAR_CMD_TURN   't'
#define CAR_CMD_TILT  'a'
#define CAR_CMD_PLAY   'p'

#define B_HALL_C 		1

#define MOTOR_RL  	 2
#define MOTOR_RR  	 3
#define MOTOR_TURN  0
#define MOTOR_TILT    1
//#define PROC_DTIME  		50
#define SYS_TBUF_LEN			248
#define SYS_RBUF_LEN			512
//
#define NET_CHL_WIFI  		1
#define NET_CHL_ZIGBEE	  2
#define NET_CHL_ALL				3	

//
#define C_PC_POS			1 
#define C_PC_CFG_MD5		2 
//#define C_PC_CFG_MD5_OK		1 
#define C_PC_CFG_DL			3 
#define C_PC_CFG_JOIN		4 
#define C_CAR_REGPOS		5 
#define C_CAR_UNREG			6 
#define C_PC_G_ADJUST		7
#define C_PC_ACT_RUN		8
#define C_PC_ACT_TURN		9
#define C_PC_ACT_TILT		10
#define C_PC_SENSOR			11
#define C_PC_MCMD			12/*手动控制*/
#define C_PC_SETADDR        13

#define C_PC_MCMD_IN			0x80
#define C_PC_MCMD_OUT			0x81
#define C_PC_MCMD_RUN			0x82


#define C_PC_SYSERR        0xfc
#define C_PC_SYSRST        0xfd
#define C_PC_MEM		   0xfe
#define C_PC_HEART		   0xff

#define TIMEOV_HEART	   2000

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

#define CAR_CHECK_POS			30
//-----------------------
#define POSMODIFY(a)  ((INT32U)a*259>>8)/*需要实际测试修正*/
#define SPDMODIFY(a)  ((INT32U)a*2>>6)
#define TURNANGLEMODIFY(a)  ((int)a<<2)

void App_init(void);
void MSDelay(UINT ms);//参数：ms
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
void ReportPos(void);
void MCmdProc(void);
void TrigUS(void);
void UartSendProc(INT8U n,INT8U chl);

#endif

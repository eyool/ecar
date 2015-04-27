#include <includes.h>
#include "app.h"
#include "flash_if.h"
#include "math.h"

INT8U LvSystbuf[SYS_TBUF_LEN+8];//系统临时缓存
INT8U Sysrbuf[SYS_RBUF_LEN];
INT8U Uarttbuf[SYS_TBUF_LEN];
//INT8U n_uartTX;
INT8U *Systbuf;
INT32U G_msg[2];
//INT32U SysStatus,
INT8U b_debug=0,sw_power=0;
SENSOR m_sensor[2];//0,当前，1，保留上次
INT32U lastsendtime=0;
IDCMD m_icmd;

//RFID *p_rfid;
CAR m_car;
//----------
float test(void)
{
	IDCMD * p_ic=FindUhfidCmd((RFID *)0x800c400);
		IDCMD *p2;
	p2=p_ic+1;
	
	if(p2->cmd==p_ic->cmd)
		return 1;
	else
		return 0;
}

void App_init(void)
{
	//SysStatus=0;
	Systbuf=LvSystbuf+8;
	memset((void *)m_sensor,0,sizeof(m_sensor));
	memset((void *)&m_car,0,sizeof(CAR));
	memset((void *)&m_icmd,0,sizeof(IDCMD));
	m_car.cid=m_sysset.cid;//GetAddr();
	SetAddr(m_car.cid);
	m_car.status=CAR_STATUS_NULL;
	//n_uartTX=0;
//	p_rfid=NULL;
	OSTimeDly(2);
	WifiEnable();
	ZigbeeEnable();
	OSTimeDly(500);	
//  Wifi_SetReSrv();
//	OSTimeDly(500);	

//#define TEST 1	
#ifndef	TEST	
	while(WifiStatus()<WIFI_LINK_OK){
		Wifi_SetReSrv();
		OSTimeDly(500);			
	}
	

	if(CheckSelf()==0){	
		while(!DowloadCFG()){
			WifiLink();
			OSTimeDly(200);			
		}				
	}
	//else
		//m_car.err=SYS_STATUS_INIT_ERR;			
#endif	
//	test();
	m_car.status=CAR_STATUS_INIT;
	UhfidSwitch(UHFID_CMD_OPEN);
	StartIWDG();
}
//--------------------
void nopf(void)
{
}

//由应用程序调用
void MSDelay(UINT ms)//参数：ms
{
  OSTimeDly(ms);
}
/*static UINT lasttick=0;
	UINT ct=OSTimeGet();   
        
        if(lasttick==0) 
          lasttick=ct;*/
//-------------------
//主任务函数1/4
void MainTaskProc(void *p_msg)
{
	UINT ct=OSTimeGet();  
	if(p_msg==0){
		SW_Power();
		if ((ct&0xfff)==0)
			UhfidSwitch(UHFID_CMD_OPEN);
		//----------------------------
		if(!b_debug)
			WifiLink();//让wifi断线从新连接
		ZigbeeLink();//zigbee断线从新连接
		OSTimeDly(20);	
	}
	else
	{
		OSTimeDly(10);		
	}
}
//任务函数 2/4
void UartRecvProc(uint8_t chl,uint8_t *buf,uint32_t len)
{
	static INT8U debugcmd=0;
	RFID *pid;
	if(chl==UART_CHL_UHFID){
		if(buf[0]=='D'&&buf[1]=='B'&&buf[2]=='G'&&buf[3]=='U')//debug uart
			if(buf[4]=='='&&buf[5]=='U'&&buf[6]=='2'){
				if(buf[7]=='W')
					debugcmd=buf[8]&1;
				else if(buf[7]=='Z')
					debugcmd=(buf[8]&1)<<1;	
				b_debug=debugcmd;
				return;
			}
	}	
	if(debugcmd&1){//u2w
		if(chl==UART_CHL_UHFID)
			 Wifi_AT(buf,len);
		if(chl==UART_CHL_WIFI)		
			 Uhfid_send(buf,len);
		SetWifiLinkTime();
	}
	if(debugcmd&2){//u2z
		if(chl==UART_CHL_UHFID)
			 Zigbee_send(buf,len);
		if(chl==UART_CHL_ZIGBEE)		
			 Uhfid_send(buf,len);	
		SetZigbeeLinkTime();		
	}	
	//----------------------------
	if(!debugcmd){
		if(chl==UART_CHL_WIFI){
			if(IsNetData(buf,len)){
				memcpy(Sysrbuf,buf,len>SYS_RBUF_LEN?SYS_RBUF_LEN:len);
				NetCmdProc(Sysrbuf,len);
				SetWifiLinkTime();						
			}
	
			//Debug(buf,len);//重复取消
		}		
		else if(chl==UART_CHL_ZIGBEE){
			if(IsNetData(buf,len)){
					memcpy(Sysrbuf,buf,len>SYS_RBUF_LEN?SYS_RBUF_LEN:len);
					NetCmdProc(Sysrbuf,len);
			}
			SetZigbeeLinkTime();					
		}
		else if(chl==UART_CHL_UHFID){
				*(INT32U *)buf=ParseUhfid(buf,(INT8S *)(buf+5),len);
				pid=GetRfidStruct((buf[0]<<8)|buf[1]);
				if(pid){
					if(pid!=m_car.p_rfid[0]&&pid!=m_car.p_rfid[1]){
						//m_car.rfid=*(INT32U *)buf&RDID_BITS;
						m_car.RSSI=*(INT8S *)(buf+5);
						m_car.p_rfid[1]=m_car.p_rfid[0];
						m_car.p_rfid[0]=pid;
						m_car.l_tick=OSTimeGet();  
						m_car.rfid_np=m_sensor[0].r_np[0]+m_sensor[0].r_np[1]>>1;
					}
				}
				//Wifi_send(buf,len);
		}
	}
}


//任务函数 3/4
void AppRunProc(void  *p_msg)
{
// 	INT32U tmp;
	if(!p_msg){
		OSTimeDly(50);
		switch(m_car.status){
			case CAR_STATUS_NULL:

				break;
			case CAR_STATUS_INIT:
				if(!m_car.err){
					Systbuf[0]=C_PC_CFG_JOIN;
					Systbuf[1]=GetKey()&KEY_JOIN;
					Get_SerialNum((INT32U *)(Systbuf+2));//12字节
					NetSend(14,NET_CHL_ALL);//INIT
					OSTimeDly(1000);
				}
				else{
					Systbuf[0]=C_PC_SYSERR;
					Systbuf[1]=m_car.status;
					Systbuf[2]=m_car.err;
					NetSend(3,NET_CHL_ALL);//INIT
					OSTimeDly(1000);
				}
				break;
			case CAR_STATUS_JOIN:
				sw_power=1;
				OSTimeDly(1000);
				CloseRunBreak();
				if(m_car.p_rfid[0]&&m_car.p_rfid[0]->id){
					m_car.status=CAR_STATUS_WAITCMD;
					SetMotorSpd(MOTOR_RL,0);
					SetMotorSpd(MOTOR_RR,0);
				}
				else{//找卡
					if(m_car.pos<CAR_CHECK_POS){
						AdjustMotorSpd(MOTOR_RL,1,20);
						AdjustMotorSpd(MOTOR_RR,1,20);
						AdjustMotorBalance(m_sensor[0].ps);
					}
					else{
						m_car.status=CAR_STATUS_WAITCMD;
						SetMotorSpd(MOTOR_RL,0);
						SetMotorSpd(MOTOR_RR,0);							
					}
					OSTimeDly(20);
				}
				break;
			case CAR_STATUS_WAITCMD:
				MCmdProc();
				break;
			case CAR_STATUS_RUN:
				if(m_car.p_rfid[0]){
					if((m_car.p_rfid[0]->area==CAR_AREA_GETOFF&&!(GetKey()&KEY_GO))||(m_car.p_rfid[0]->area==CAR_AREA_GETON&&(GetKey()&KEY_GO))){
						SetMotorSpd(MOTOR_TURN,0);
						SetMotorSpd(MOTOR_RL,0);
						SetMotorSpd(MOTOR_RR,0);
						SetMotorSpd(MOTOR_TILT,0);
						CloseAllRelay();
						sw_power=0;
					}
					else{
						if(!sw_power){
							sw_power=1;
							OSTimeDly(500);
						}
						RunCmdProc(m_car.p_rfid[0]);
					}
				}
				break;
		}
	}
	else{
		
	
	}
}
//任务函数 4/4
void SensorProc(void)
{
	static INT32S abuf[2]={0,0};
	static INT16S hbuf[4]={0,0,0,0};
//	static INT8U  bn=0;
	INT16S buf[9]; 
	SENSOR *p_sensor=&m_sensor[0];
	SYSSET *p_ss=(SYSSET *)SYSSET_ADDR;
	int tmp;
//	INT8U *bp=(INT8U *)buf; 	
	memcpy((void *)&m_sensor[1],(void *)&m_sensor[0],sizeof(SENSOR));
	//得到霍尔 ，加速度，压力传感器
	ADXL_GetData((INT8U *)buf,6);
	abuf[0]=((int)abuf[0]*31>>5)+buf[0];
	abuf[1]=((int)abuf[1]*31>>5)+buf[1];
	p_sensor->tilt=GetArc(abuf[0]-(p_ss->g_offx<<5),abuf[1]-(0x100-p_ss->g_offy<<5));	
	HMC_GetData((INT8U *)(buf+3),12);
	hbuf[0]=((int)hbuf[0]*7>>3)+buf[3];
	hbuf[1]=((int)hbuf[1]*7>>3)+buf[5];	
	p_sensor->rotation[0]=GetArc(hbuf[0],hbuf[1]);	
	hbuf[2]=((int)hbuf[2]*7>>3)+buf[6];
	hbuf[3]=((int)hbuf[3]*7>>3)+buf[8];		
	p_sensor->rotation[1]=GetArc(hbuf[2],hbuf[3]);	
	//------------更新偏移量
	m_sysset.g_offx=abuf[0]>>5;
	m_sysset.g_offy=abuf[1]>>5;
	m_sysset.hmc_off=p_sensor->rotation[0]-p_sensor->rotation[1];
	if(m_sysset.hmc_off>=1800)
		m_sysset.hmc_off-=3600;
	if(m_sysset.hmc_off<=-1800)
		m_sysset.hmc_off+=3600;	
	m_sysset.rfid=m_car.p_rfid[0]->id;
	//-压力传感器值
	  if(GetPSData(PS_CHL_A,(INT32S *)buf))
			p_sensor->ps=*(INT32S *)buf;
	//得到外部电压
		p_sensor->xv=GetXV();
	//得到电机数度和脉冲数
		p_sensor->turnspeed=GetMotorSpd(MOTOR_TURN);
		p_sensor->runspeed[0]=GetMotorSpd(MOTOR_RL);	
		p_sensor->runspeed[1]=GetMotorSpd(MOTOR_RR);
		p_sensor->t_np=(INT32S)GetMotorPoscn(MOTOR_TURN);
		p_sensor->r_np[0]=GetMotorPoscn(MOTOR_RL);	
		p_sensor->r_np[1]=GetMotorPoscn(MOTOR_RR);
	//---检查是否回正
		if(CheckCenter()){
			p_sensor->b_hall|=B_HALL_C;
			p_sensor->lastt_np=p_sensor->t_np;
			//m_car.turnangle_np=0;
		}
		else
			p_sensor->b_hall&=(~B_HALL_C);	
	//计算角度对应np
		m_car.turnangle_np=p_sensor->t_np-p_sensor->lastt_np;
		/*tmp=p_sensor->t_np-p_sensor->lastt_np;
		p_sensor->lastt_np=p_sensor->t_np;
		if(GetTurnDir())
			m_car.turnangle_np+=tmp;
		else
			m_car.turnangle_np-=tmp;*/


	//这里添加测距函数
		tmp=(m_sensor[0].runspeed[0]>>2)+(m_sensor[0].runspeed[1]>>2);
		tmp+=(m_sensor[1].runspeed[0]>>2)+(m_sensor[1].runspeed[1]>>2);
		m_car.spd=SPDMODIFY(tmp);
		tmp=(p_sensor->r_np[0]+p_sensor->r_np[1]>>1)-m_car.rfid_np;
		m_car.pos=POSMODIFY(tmp);
		if(m_car.p_rfid[0])
			m_car.pos+=m_car.p_rfid[0]->pos;
		//if(m_car.status==CAR_STATUS_RUN){
		if(m_car.p_rfid[0])			
			ReportPos();	
		//}
		else if(m_car.status!=CAR_STATUS_INIT&&OSTimeGet()-lastsendtime>TIMEOV_HEART){
						Uarttbuf[0]=C_PC_HEART;
						Uarttbuf[1]=m_car.status;
						UARTMboxPost(2,NET_CHL_ALL);
						//NetSend(2,NET_CHL_ALL);//heart
					}
	//碰撞检测
#define SAFE_DIS	300
		m_car.frontsafedis=m_car.frontsafedis*7+GetFrontSafeDis()>>3;
		TrigUS();
		if(m_car.frontsafedis<SAFE_DIS){
			AdjustMotorSpd(MOTOR_RL,0,m_car.frontsafedis>>3);
			AdjustMotorSpd(MOTOR_RR,0,m_car.frontsafedis>>3);
		//	OpenRunBreak();
		}
		//else
			//CloseRunBreak();
/*#define N_CKUS	10
		tmp=GetUSDis();
		if(tmp==0){
			if(bn++>N_CKUS)
				OpenRunBreak();
		}
		else if(tmp>0){
				bn=0;
				CloseRunBreak();
		}
	//Wifi_send((INT8U *)buf,18);*/
	OSTimeDly(5);
}
//-------------------------
void Debug(INT8U *buf,INT8U len)
{
			if(buf[0]=='D'&&buf[1]=='B'&&buf[2]=='G'){//debug spi
			if(buf[3]=='S'){//spi 总线数据
				if(buf[4]=='W'){//write
					//SpiWrite(buf[6],buf+8,buf[7]&7,buf[5]);//地址 ，数据区，要求长度，CS选择
					
				}		
				else if(buf[4]=='R'){//read
					//OSSchedLock();
					SpiRead(buf[6],buf+8,buf[7]&7,buf[5]);
					//OSSchedUnlock();
					Wifi_send(buf+8,buf[7]&7);
				}
			}
			else if(buf[3]=='P'){//压力传感器数据
				buf[9]=GetPSData(buf[4],(INT32S *)(buf+5));
				Wifi_send(buf+5,5);
			}
			else if(buf[3]=='M'){//内存数据
				if(buf[4]=='R'){//read
					Wifi_send((INT8U *)(0x20000000+(*(INT32U *)(buf+5)&0x4fff)),16);
				}
			}
			else if(buf[3]=='C'){//命令
				if(buf[4]=='S')
						Wifi_send((INT8U *)m_sensor,sizeof(m_sensor));
			}		
			else if(buf[3]=='U'){//uhfid
				UhfidSwitch(buf[4]&3); 
			}			
			else if(buf[3]=='D'){//debug open
				b_debug=buf[4]&1; 
			}				
			return;
		}
}
void NetSend(INT8U len,INT8U chl)
{
	INT8U *bp=Systbuf-4;
	bp[0]=FRAME_HEAD;
	bp[1]=len+4;
	bp[2]=m_car.cid;	
	bp[3]=FRAME_BROAD;	
	bp[len+4]=CheckSum8(bp,len+4);
	bp[len+5]=FRAME_END;	
	if(chl&NET_CHL_WIFI)
			Wifi_send(bp,len+6);
	if(chl&NET_CHL_ZIGBEE)
			Zigbee_send(bp,len+6);	
	lastsendtime=OSTimeGet();
}

INT8U DowloadCFG(void)
{
	INT32U *msg;
	INT8U err;
	if(WifiStatus()==WIFI_ST_OK){
		Systbuf[0]=C_PC_CFG_MD5;
		NetSend(1,NET_CHL_WIFI);//DL
		msg = OSMboxPend(App_StartMbox, 3000, &err);//等待3秒
		if(err==OS_ERR_NONE&&(msg[0]&0xff)==C_PC_CFG_DL){
			if(msg[1]==C_PC_CFG_DL_OK)
				return 1;
		}
	}

	return 0;
}



void NetCmdProc(INT8U *buf,INT8U len)
{
	static INT8U b_dl=0;
	static INT32U mbuf[4];	
	INT8U sp=0,sn;
	INT8U *rbuf;
	int i;
	sn=GetCmd(buf,&rbuf,&sp,len);
	while(sn){
		switch(*rbuf){
			case C_PC_POS:

				break;
			case C_PC_CFG_MD5://cmd(1)+data(n)
				GetCfgMd5(mbuf);
				if(memcmp((INT8U *)mbuf,rbuf+1,16)&&*(INT32U *)(rbuf+17)==PROGRAMM_KEY){//不一样，从新下载
					memcpy(mbuf,rbuf+1,16);
					CfgClear();
					b_dl=1;
					Systbuf[0]=C_PC_CFG_DL;
					Systbuf[1]=0;	
					Systbuf[2]=0;						
					NetSend(3,NET_CHL_WIFI);//DL	
				}
				else{
					G_msg[0]=C_PC_CFG_DL;
					G_msg[1]=C_PC_CFG_DL_OK;	
					OSMboxPost(App_StartMbox, (void *)G_msg);
				}
				break;
			case C_PC_CFG_DL://cmd(1)+index(2)+data(n)
					if(*(INT16U *)(rbuf+1)==C_PC_CFG_DL_ERR||*(INT16U *)(rbuf+1)==C_PC_CFG_DL_OK){
						if(*(INT16U *)(rbuf+1)==C_PC_CFG_DL_OK&&b_dl&&CheckCfgSum()==*(INT32U *)(rbuf+3))
							SetCfgMd5(mbuf);
						b_dl=0;
						G_msg[0]=C_PC_CFG_DL;
						G_msg[1]=*(INT16U *)(rbuf+1);
						OSMboxPost(App_StartMbox, (void *)G_msg);	
					}else{
						Systbuf[0]=C_PC_CFG_DL;		
						if(b_dl){
							if(*(INT16U *)(rbuf+1)+1==CfgWriteId(rbuf+3,sn-3))
								*(INT16U *)(Systbuf+1)=*(INT16U *)(rbuf+1)+1;											
							else{
								*(INT16U *)(Systbuf+1)=C_PC_CFG_DL_ERR;								
								b_dl=0;
							}								
						}else
								*(INT16U *)(Systbuf+1)=C_PC_CFG_DL_ERR;									
						NetSend(3,NET_CHL_WIFI);//DL	
						if(b_dl==0){
								G_msg[0]=C_PC_CFG_DL;
								G_msg[1]=C_PC_CFG_DL_ERR;	
								OSMboxPost(App_StartMbox, (void *)G_msg);
						}
						
					}											
				break;	
			case C_PC_CFG_JOIN:
				if(rbuf[1])
					m_car.status=CAR_STATUS_JOIN;
				break;
			case C_CAR_REGPOS:
				for (i=1;i<sn;i+=5)
					RegisterCar(rbuf[i],*(INT32U *)(rbuf+i+1));							
				break;
			case C_CAR_UNREG:
				for (i=1;i<sn;i++)
					UnRegisterCar(rbuf[i]);
				break;
			case C_PC_G_ADJUST:
					SaveSysSet();
				break;
			case C_PC_ACT_RUN:
				if((rbuf[2]&3)>=MOTOR_RL){
					if(rbuf[3])
						OpenRunBreak();
					else
						CloseRunBreak();
				}
				AdjustMotorSpd(rbuf[2]&3,(INT8S)rbuf[1],100);
				break;
			case C_PC_ACT_TURN:
				if((INT8S)rbuf[1]>0)
					TurnLeft();
				else{
					TurnRight();
					//rbuf[1]=-(INT8S)rbuf[1];
				}
				if(rbuf[2])
					OpenTurnBreak();
				else
					CloseTurnBreak();
				//SetMotorSpd(MOTOR_TURN,rbuf[1]>>3);
				break;
			case C_PC_ACT_TILT:
				if((INT8S)rbuf[1]>0){
					CloseDownRelay();
					OpenUpRelay();
				}
				else{
					CloseUpRelay();
					OpenDownRelay();
					//rbuf[1]=-(INT8S)rbuf[1];
				}
				//SetMotorSpd(MOTOR_TILT,rbuf[1]>>3);
				break;
			case C_PC_SENSOR:
				Systbuf[0]=C_PC_SENSOR;
				memcpy(Systbuf+1,(INT8U *)&m_sensor[0],sizeof(m_sensor)>>1);
				NetSend((sizeof(m_sensor)>>1)+1,NET_CHL_ALL);//sensor
				break;
			case C_PC_MEM:
				Systbuf[0]=C_PC_MEM;
				memcpy(Systbuf+1,(INT8U *)(0x20000000+(*(INT32U *)(rbuf+2)&0x3fff)),rbuf[1]&0x1f);
				NetSend((rbuf[1]&0x1f)+1,NET_CHL_ALL);//mem
				break;
			case C_PC_MCMD:
				if(m_car.status!=CAR_STATUS_WAITCMD){
					if(rbuf[1]==C_PC_MCMD_IN){
						m_car.laststatus=m_car.status;
						m_car.status=CAR_STATUS_WAITCMD;
						if(sn>=3)
							m_car.p_rfid[0]=(RFID *)(CFGSYS_FLASH_CFG_BODY+*(INT16U *)(CFGSYS_FLASH_CFG_NP+(rbuf[2]<<2)));
					}
				}
				else{
					if(rbuf[1]==C_PC_MCMD_OUT){
						if(m_car.laststatus)
							m_car.status=m_car.laststatus;
						else
							m_car.status=CAR_STATUS_RUN;
					}
					else if(rbuf[1]==C_PC_MCMD_RUN)
						 m_car.status=CAR_STATUS_RUN;
					else if(rbuf[1]!=C_PC_MCMD_IN)
						 memcpy((INT8U *)&m_icmd,rbuf+1,sn-1);
				}
				break;
			case C_PC_SETADDR:		//addr  id(12byte)
				Get_SerialNum((INT32U *)Systbuf);
				if(memcmp(Systbuf,rbuf+2,12)==0){
					m_car.cid=rbuf[1];
					m_sysset.cid=rbuf[1];
					SetAddr(rbuf[1]);
					SaveSysSet();
				}
				break;
			case C_PC_SYSRST:
				if(*(INT32U *)(rbuf+2)==PROGRAMM_KEY&&rbuf[1]==m_car.cid)//GetAddr()
					NVIC_SystemReset();
				break;
		}
		sn=GetCmd(buf,&rbuf,&sp,len);	
	}
}

#define TYPE_TILT_HALF	1
#define TYPE_TILT_ONE	2
void RunCmdProc(RFID *p_rfid)
{
//	static INT8U n_tilt=0;
	static IDCMD tilt_ic;
	static RFID *lp_rfid=0;
	int i;
	INT32U dt;
	if(p_rfid){
		for (i=0;i<p_rfid->n_idcmd;i++){
			IDCMD *p_ic = FindUhfidCmd(p_rfid);
			dt=OSTimeGet()-m_car.l_tick;
			//if(dt>p_ic->tick*100&&(dt<(p_ic->tick+p_ic->runtime)*100||p_ic->runtime==0))
			switch(p_ic->cmd){
					case  CAR_CMD_RUN:
						if(dt>p_ic->tick*100&&(dt<(p_ic->tick+p_ic->runtime)*100||p_ic->runtime==0))
							RunCtrl(p_ic);
						//else{
						//	SetMotorSpd(MOTOR_RL,0);
						//	SetMotorSpd(MOTOR_RR,0);
						//}
						break;
					case 	CAR_CMD_TURN:
						if(dt>p_ic->tick*100&&(dt<(p_ic->tick+p_ic->runtime)*100||p_ic->runtime==0))
							TurnCtrl(p_ic);
						//else
						//	SetMotorSpd(MOTOR_TURN,0);
						break;
					case 	CAR_CMD_TILT:
						if(dt>p_ic->tick*100&&(dt<(p_ic->tick+p_ic->runtime)*100||p_ic->runtime==0)){
							if(lp_rfid!=p_rfid||tilt_ic.tick!=p_ic->tick)
								memcpy((INT8U *)&tilt_ic,(INT8U *)p_ic,sizeof(IDCMD));
							
							if(TiltCtrl(&tilt_ic)){
								if(tilt_ic.type==TYPE_TILT_HALF)
									tilt_ic.dis=0;
								if(tilt_ic.type==TYPE_TILT_ONE){
									if(tilt_ic.dis==p_ic->dis)
										tilt_ic.dis=-p_ic->dis;
									else
										tilt_ic.dis=0;
								}

							}
							//b_tilt=1;
						}
						//else
							//n_tilt=0;
						//	SetMotorSpd(MOTOR_TILT,0);
						break;
					case 	CAR_CMD_PLAY:

						break;
			}
		}
	}
	lp_rfid=p_rfid;
}
#define SPD_DT_LMT 5
void RunCtrl(IDCMD *p_ic)
{
//		 static INT16S sspd=0;
 //    int dspd=0;//,rspd;
//		static INT16S ldt=0;
//		int dt;
    //rspd=;m_sensor[0].runspeed[0]+p_ic->spd-m_sensor[0].runspeed[1]>>1;
	/*	dt=((int)p_ic->spd-(int)m_car.spd<<4)-ldt*15-1;
		ldt=p_ic->spd-m_car.spd;	
			if(dt<0)
			dspd=0;
	if(dt>0)	
			dspd+=dt;
		else
			dspd+=(dt<<1);	
		dspd=p_ic->spd-m_car.spd;
    if (dspd>SPD_DT_LMT) 
        dspd=SPD_DT_LMT;
    if (dspd<-SPD_DT_LMT) 
        dspd=-SPD_DT_LMT;*/
	if(m_car.p_rfid[0]&&GetFrontDis()<m_car.p_rfid[0]->safedis){
	//		dspd=-SPD_DT_LMT;
			AdjustMotorSpd(MOTOR_RL,-5,p_ic->spd);
			AdjustMotorSpd(MOTOR_RR,-5,p_ic->spd);			
			return;
		}
	
	/*	if(dt>=0)
			dspd+=(dt&0x3f);
		else
			dspd-=((-dt)&0x3f);				
			if(dspd>=0x40){
				dt+=0x40;
				dspd-=0x40;
			}			
			else if(dspd<=-0x40){
				dt-=0x40;
				dspd+=0x40;	
			}
		if(dspd>0)
			sspd++;
		else if(dspd<0)
			sspd--;*/
		
    AdjustMotorSpd(MOTOR_RL,1,p_ic->spd);
    AdjustMotorSpd(MOTOR_RR,1,p_ic->spd);
    AdjustMotorBalance(m_sensor[0].ps);
		
}
#define ANGLE_CYCLE   3600
#define ANGLE_DT      20
#define ANGLE_MAX     3000
//#define LMIN_SPD      20
#define ZERO_SPD      2
void TurnCtrl(IDCMD *p_ic)
{
  //  static INT32U ltick=0;
    int dir=0;
    int sro=p_ic->dis,cro=TURNANGLEMODIFY(m_car.turnangle_np);//m_sensor[0].rotation[0]-m_sensor[0].rotation[1];
    while(cro>=ANGLE_CYCLE) cro-=ANGLE_CYCLE;
    while(cro<=-ANGLE_CYCLE) cro+=ANGLE_CYCLE;

    sro=sro-cro;
    //if(sro>=(ANGLE_CYCLE>>1)) sro-=ANGLE_CYCLE;
    //if(sro<=-(ANGLE_CYCLE>>1)) sro+=ANGLE_CYCLE;

    if (sro>=0)
        dir=1;//正转
    else
        sro=-sro;
    if (sro>ANGLE_MAX)
        sro=ANGLE_MAX;


    if (sro<ANGLE_DT) {
		OpenTurnBreak();
		SetMotorSpd(MOTOR_TURN,0);
		//ltick=OSTimeGet();
	}
	else {

        /*dspd=((p_ic->spd*sro)/ANGLE_MAX)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;*/
		if(!GetTurnBreak())
			if(sro<(ANGLE_DT<<4))
				AdjustMotorSpd(MOTOR_TURN,1,p_ic->spd>8+(sro>>4)?8+(sro>>4):p_ic->spd);
			else
				AdjustMotorSpd(MOTOR_TURN,1,p_ic->spd);
			/*if(sro<(ANGLE_DT<<2)) 
				AdjustMotorSpd(MOTOR_TURN,1,p_ic->spd>>1);
			else
				AdjustMotorSpd(MOTOR_TURN,1,p_ic->spd);*/

		if(m_sensor[0].turnspeed<ZERO_SPD)
		{
			if(dir) 
				TurnLeft();
			else
				TurnRight();
			CloseTurnBreak();
		}
    }
}
#define TILT_MAX    100
#define TILT_DT    10
int TiltCtrl(IDCMD *p_ic)
{
    int dir=0;
    int stilt=p_ic->dis-(m_sensor[0].tilt+m_sensor[1].tilt>>1);

    if (stilt>0) 
        dir=1;
    else   
        stilt=-stilt; 
    if (stilt>TILT_MAX) 
        stilt=TILT_MAX;

    if (stilt<TILT_DT) {
        SetMotorSpd(MOTOR_TILT,0); 
        CloseAllRelay();
		return 1;
    }
    else{
        if (dir){
            CloseDownRelay();
            OpenUpRelay();
        } else{
            CloseUpRelay();
            OpenDownRelay();
        }
       /* dspd=((p_ic->spd*stilt)/TILT_MAX)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;*/
		if(stilt<(TILT_DT<<2))
			AdjustMotorSpd(MOTOR_TILT,2,p_ic->spd>(stilt<<1)?(stilt<<1):p_ic->spd);
		else
			AdjustMotorSpd(MOTOR_TILT,2,p_ic->spd);
		return 0;
    }
}
void SW_Power(void)
{
	if(sw_power&&(OSTimeGet()&0x7f)<30)
		SetSafeLine();
	else
		ResetSafeLine();
	IWDG_ReloadCounter();
}
int RegisterCar(INT8U id,INT32U pos)
{	
	int i=0;
	for (;i<CAR_MAX;i++)
		if(m_car.allcarid[i]==id){
			m_car.allcarpos[i]=pos;
			return i;
		}
	if(i>=CAR_MAX)
		for (i=0;i<CAR_MAX;i++)
			if(!m_car.allcarid[i]){
				m_car.allcarid[i]=id;
				m_car.allcarpos[i]=pos;
				return i;
			}
	return -1;
}
void UnRegisterCar(INT8U id)
{
	int i=0;
	for (;i<CAR_MAX;i++)
		if(m_car.allcarid[i]==id)
			m_car.allcarid[i]=0;
}
INT32U GetFrontDis(void)
{
	INT32U dis=0xffffffff;
	int i=0;
	for (;i<CAR_MAX;i++)
		if(m_car.allcarid[i]&&m_car.pos<m_car.allcarpos[i]&&dis>m_car.allcarpos[i]-m_car.pos)
				dis=m_car.allcarpos[i]-m_car.pos;
	if(m_car.p_rfid[0]&&dis==0xffffffff&&m_car.p_rfid[0]->area==CAR_AREA_GETON){
		for (i=0;i<CAR_MAX;i++)
			if(m_car.allcarid[i]&&dis>m_car.allcarpos[i])
				dis=m_car.allcarpos[i];
	}
	return dis;
}

#define RP_DT	1000
#define RP_DP	50
void ReportPos(void)//INT32U pos,INT16U rdif
{
	static INT32U ltick=0;
	static INT32U lpos=0;
	INT32U ntick=OSTimeGet();
	if(ntick-ltick>RP_DT||m_car.pos-lpos>RP_DP){
		if(GetKey()&KEY_JOIN)
			Uarttbuf[0]=C_CAR_REGPOS;
		else
			Uarttbuf[0]=C_CAR_UNREG;
		//Systbuf[1]=m_car.cid;
		*(INT32U *)(Uarttbuf+1)=m_car.pos;
		*(INT16U *)(Uarttbuf+5)=m_car.p_rfid[0]->id;
		*(INT16U *)(Uarttbuf+7)=m_car.spd;
		Uarttbuf[9]=m_car.status;
		Uarttbuf[10]=GetKey();
//		if(ntick-ltick>RP_DT)
		ltick=ntick;
//		if(pos-lpos>RP_DP)
		lpos=m_car.pos;
		//NetSend(10,NET_CHL_ALL);//rep
		UARTMboxPost(11,NET_CHL_ALL);
	}
}

void MCmdProc(void)
{
	INT32U spos=m_car.pos;
	INT32U stime=OSTimeGet();
	OSTimeDly(m_icmd.tick&0xff);
	switch(m_icmd.cmd){
		case CAR_CMD_RUN:
			while(spos+(m_icmd.dis&0xff)>m_car.pos&&m_icmd.runtime*100+stime>OSTimeGet()){
				AdjustMotorSpd(MOTOR_RL,1,m_icmd.spd);
				AdjustMotorSpd(MOTOR_RR,1,m_icmd.spd);
				AdjustMotorBalance(m_sensor[0].ps);
				OSTimeDly(20);
			}
			SetMotorSpd(MOTOR_RL,0);
			SetMotorSpd(MOTOR_RR,0);
			break;
		case CAR_CMD_TURN:
			if(m_icmd.dis>=0)
				TurnRight();
			else{
				TurnLeft();
				m_icmd.dis=-m_icmd.dis;
			}
			while(spos+(m_icmd.dis&0xff)>m_car.pos&&m_icmd.runtime*100+stime>OSTimeGet()){
				AdjustMotorSpd(MOTOR_TURN,1,m_icmd.spd);
				OSTimeDly(20);
			}
			SetMotorSpd(MOTOR_TURN,0);
			break;
		case CAR_CMD_TILT:
			if(m_icmd.dis>=0){
				CloseDownRelay();
				OpenUpRelay();
			}
			else{
				CloseUpRelay();
				OpenDownRelay();
				m_icmd.dis=-m_icmd.dis;
			}
			while(spos+(m_icmd.dis&0xff)>m_car.pos&&m_icmd.runtime*100+stime>OSTimeGet()){
				AdjustMotorSpd(MOTOR_TILT,1,m_icmd.spd);
				OSTimeDly(20);
			}
			CloseUpRelay();
			CloseDownRelay();
			SetMotorSpd(MOTOR_TILT,0);
			break;
	}
	m_icmd.cmd=0;
	m_icmd.tick=0;
}
void TrigUS(void)
{
	if(GetUS())
		return;
	OpenUS();
	uDelay(30);
	CloseUS();
/*	static INT8U b_op=0;
	static INT32U stime;
	INT32U n=1000;
	if(b_op==0){
		OpenUS();
		uDelay(30);
		CloseUS();
		if(GetUS())
			return-1;
		while(n--){
			uDelay(1);
			if(GetUS()){
				stime=OSTimeGet();
				b_op=1;
				break;
			}
		}	
	}
	else{
		if(OSTimeGet()-stime<30){
			
			return GetUS()?OSTimeGet()-stime:0;
		}
		else
			b_op=0;
	}
	return -1;*/
}
void UartSendProc(INT8U n,INT8U chl)
{
	memcpy(Systbuf,Uarttbuf,n);
	NetSend(n,chl);
}

#define CHECK_TA	450
INT8U CheckSelf(void)
{
	INT32S tmp[2],n;//re=0x400;
	return 0;//debug 取消
	//1
	m_car.err=0xf;
	tmp[0]=m_sensor[0].r_np[0];
	tmp[1]=m_sensor[0].r_np[1];
	n=10;
	while(n--){
		AdjustMotorSpd(MOTOR_RL,2,20);
		AdjustMotorSpd(MOTOR_RR,2,20);
		OSTimeDly(100);
		if(m_sensor[0].r_np[0]>tmp[0]+2&&m_sensor[0].r_np[1]>tmp[1]+2){
			m_car.err&=0xe;
			break;
		}
	}
	SetMotorSpd(MOTOR_RL,0);
	SetMotorSpd(MOTOR_RR,0);
	//2
	tmp[0]=m_sensor[0].t_np;
	n=10;
	TurnLeft();
	while(n--){
		AdjustMotorSpd(MOTOR_TURN,2,20);
		OSTimeDly(100);
		if(m_sensor[0].t_np>tmp[0]+2||m_sensor[0].t_np+2<tmp[0]){
			m_car.err&=0xd;
			break;
		}
	}
	tmp[0]=0;
	while(TURNANGLEMODIFY(m_car.turnangle_np)<CHECK_TA){
		if(m_sensor[0].b_hall|B_HALL_C){
			tmp[0]++;
			break;
		}
		OSTimeDly(100);
	}
	if(tmp[0]==0){
		SetMotorSpd(MOTOR_TURN,0);
		OSTimeDly(3000);
		TurnRight();
		while(TURNANGLEMODIFY(m_car.turnangle_np)>-CHECK_TA){
			AdjustMotorSpd(MOTOR_TURN,2,20);
			if(m_sensor[0].b_hall|B_HALL_C){
				tmp[0]++;
				break;
			}
			OSTimeDly(100);
		}
	}
	if(tmp[0])
		m_car.err&=0xb;;
	SetMotorSpd(MOTOR_TURN,0);
	OSTimeDly(3000);
	while(m_car.turnangle_np>(ANGLE_DT>>2)){
		TurnRight();
		AdjustMotorSpd(MOTOR_TURN,2,10);
		OSTimeDly(100);
	}
	while(m_car.turnangle_np<-(ANGLE_DT>>2)){
		TurnLeft();
		AdjustMotorSpd(MOTOR_TURN,2,10);
		OSTimeDly(100);
	}
	SetMotorSpd(MOTOR_TURN,0);
	//4
	tmp[0]=m_sensor[0].tilt;
	n=10;
	while(n--){
		if(tmp[0]>TILT_DT){
			CloseDownRelay();
			OpenUpRelay();
		}
		else{
			CloseUpRelay();
			OpenDownRelay();
		}
		AdjustMotorSpd(MOTOR_TILT,2,20);
		OSTimeDly(100);
		if(m_sensor[0].tilt>tmp[0]+(TILT_DT<<1)||tmp[0]>m_sensor[0].tilt+(TILT_DT<<1)){
			m_car.err&=0x7;
			break;
		}
	}
	while(m_sensor[0].tilt>TILT_DT){
		CloseDownRelay();
		OpenUpRelay();
		OSTimeDly(200);
	}
	while(m_sensor[0].tilt<-TILT_DT){
		CloseUpRelay();
		OpenDownRelay();
		OSTimeDly(200);
	}
	CloseAllRelay();
	SetMotorSpd(MOTOR_TILT,0);	
	//

	return m_car.err;
}

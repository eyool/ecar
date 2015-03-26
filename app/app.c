#include <includes.h>
#include "app.h"
#include "flash_if.h"
#include "math.h"

INT8U LvSystbuf[SYS_TBUF_LEN+8];//系统临时缓存

INT8U *Systbuf;
INT32U G_msg[5];
//INT32U SysStatus,
INT8U b_debug=0,sw_power=0;
SENSOR m_sensor[2];//0,当前，1，保留上次
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
	m_car.addr=GetAddr();
	m_car.status=CAR_STATUS_NULL;
//	p_rfid=NULL;
	OSTimeDly(2);
	WifiEnable();
	ZigbeeEnable();
	OSTimeDly(500);	
//  Wifi_SetReSrv();
//	OSTimeDly(500);	
	
	while(WifiStatus()<WIFI_LINK_OK){
		Wifi_SetReSrv();
		OSTimeDly(50);			
	}
	if(CheckSelf()){	
		while(!DowloadCFG()){
			WifiLink();
			OSTimeDly(200);			
		}				
	}
	else
		m_car.err=SYS_STATUS_INIT_ERR;			
//	test();
	m_car.status=CAR_STATUS_INIT;
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
	if(p_msg==0){
		SW_Power();
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
				if(buf[7]=='Z')
					debugcmd=(buf[8]&1)<<1;	
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
					SetWifiLinkTime();
					NetCmdProc(buf,len);
			}
			Debug(buf,len);
		}		
		else if(chl==UART_CHL_ZIGBEE){
					if(IsNetData(buf,len)){
							SetZigbeeLinkTime();
							NetCmdProc(buf,len);
					}
		}
		else if(chl==UART_CHL_UHFID){
				*(INT32U *)buf=ParseUhfid(buf,(INT8S *)(buf+5),len);
				pid=GetRfidStruct(*(INT32U *)buf&RDID_BITS);
				if(pid){
					if(pid!=m_car.p_rfid[0]&&pid!=m_car.p_rfid[1]){
						m_car.p_rfid[1]=m_car.p_rfid[0];
						m_car.p_rfid[0]=pid;
						m_car.l_tick=OSTimeGet();   
					}
				}
				//Wifi_send(buf,len);
		}
	}
}


//任务函数 3/4
void AppRunProc(void  *p_msg)
{
//	INT8U buf[4];
	if(!p_msg){
		OSTimeDly(20);
		switch(m_car.status){
			case CAR_STATUS_NULL:

				break;
			case CAR_STATUS_INIT:
				if(!m_car.err&&(GetKey()&KEY_JOIN)){
					Systbuf[0]=C_PC_CFG_JOIN;
					NetSend(1,NET_CHL_ALL);
				}
				break;
			case CAR_STATUS_JOIN:
				sw_power=1;
				if(m_car.p_rfid[0])
					if(m_car.p_rfid[0]->id)
						m_car.status=CAR_STATUS_RUN;
				break;
			case CAR_STATUS_RUN:
				if(m_car.p_rfid[0]){
					if((m_car.p_rfid[0]->area==CAR_AREA_GETOFF&&!(GetKey()&KEY_PLAY))||(m_car.p_rfid[0]->area==CAR_AREA_GETON&&(GetKey()&KEY_PLAY))){
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
			case CAR_STATUS_WAITCMD:
				
				break;
		}
	}
	else{
		
	
	}
}
//任务函数 4/4
void SensorProc(void)
{
	INT16S buf[9]; 
	SENSOR *p_sensor=&m_sensor[0];
	int tmp;
//	INT8U *bp=(INT8U *)buf; 	
	memcpy((void *)&m_sensor[1],(void *)&m_sensor[0],sizeof(SENSOR));
	//得到霍尔 ，加速度，压力传感器
	ADXL_GetData((INT8U *)buf,6);
	HMC_GetData((INT8U *)(buf+3),12);
	p_sensor->tilt=GetArc(buf[0],buf[1]);
	p_sensor->rotation[0]=GetArc(buf[3],buf[5]);	
	p_sensor->rotation[1]=GetArc(buf[6],buf[8]);	
  if(GetPSData(PS_CHL_A,(INT32S *)buf))
		p_sensor->ps=*(INT32S *)buf;
	//---检查是否回正
	if(CheckCenter())
		p_sensor->b_hall|=B_HALL_C;
	else
		p_sensor->b_hall&=(~B_HALL_C);	
	//得到外部电压
		p_sensor->xv=GetXV();
	//得到电机数度和脉冲数
		p_sensor->turnspeed=GetMotorSpd(MOTOR_TURN);
		p_sensor->runspeed[0]=GetMotorSpd(MOTOR_RL);	
		p_sensor->runspeed[1]=GetMotorSpd(MOTOR_RR);
		p_sensor->t_np=GetMotorPoscn(MOTOR_TURN);
		p_sensor->r_np[0]=GetMotorPoscn(MOTOR_RL);	
		p_sensor->r_np[1]=GetMotorPoscn(MOTOR_RR);
	//这里添加测距函数
		tmp=(m_sensor[0].runspeed[0]>>2)+(m_sensor[0].runspeed[1]>>2);
		tmp+=(m_sensor[1].runspeed[0]>>2)+(m_sensor[1].runspeed[1]>>2);
		m_car.spd=SPDMODIFY(tmp);
		tmp=(p_sensor->r_np[0]>>1)+(p_sensor->r_np[1]>>1);
		m_car.pos=POSMODIFY(tmp);
		if(m_car.p_rfid[0])
			m_car.pos+=m_car.p_rfid[0]->pos;
		
	//Wifi_send((INT8U *)buf,18);
	OSTimeDly(10);
}
//-------------------------
void Debug(INT8U *buf,INT8U len)
{
			if(buf[0]=='D'&&buf[1]=='B'&&buf[2]=='G'){//debug spi
			if(buf[3]=='S'){//spi 总线数据
				if(buf[4]=='W'){//write
					SpiWrite(buf[6],buf+8,buf[7]&7,buf[5]);//地址 ，数据区，要求长度，CS选择
					
				}		
				else if(buf[4]=='R'){//read
					SpiRead(buf[6],buf+8,buf[7]&7,buf[5]);	
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
	bp[2]=GetAddr();	
	bp[3]=FRAME_BROAD;	
	bp[len+4]=CheckSum8(bp,len+4);
	bp[len+5]=FRAME_END;	
	if(chl&NET_CHL_WIFI)
			Wifi_send(bp,len+6);
	if(chl&NET_CHL_ZIGBEE)
			Zigbee_send(bp,len+6);	
	
}

INT8U DowloadCFG(void)
{
	INT32U *msg;
	INT8U err;
	if(WifiStatus()==WIFI_ST_OK){
		Systbuf[0]=C_PC_CFG_MD5;
		NetSend(1,NET_CHL_WIFI);
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

	sn=GetCmd(buf,&rbuf,&sp,len);
	while(sn){
		switch(*rbuf){
			case C_PC_CFG_MD5://cmd(1)+data(n)
				GetCfgMd5(mbuf);
				if(memcmp((INT8U *)mbuf,rbuf+1,16)&&*(INT32U *)(rbuf+17)==PROGRAMM_KEY){//不一样，从新下载
					memcpy(mbuf,rbuf+1,16);
					CfgClear();
					b_dl=1;
					Systbuf[0]=C_PC_CFG_DL;
					Systbuf[1]=0;	
					Systbuf[2]=0;						
					NetSend(3,NET_CHL_WIFI);	
				}
				else{
					G_msg[0]=C_PC_CFG_DL;
					G_msg[1]=C_PC_CFG_DL_OK;	
					OSMboxPost(App_StartMbox, (void *)G_msg);
				}
			break;
			case C_PC_CFG_DL://cmd(1)+index(2)+data(n)
					if(*(INT16U *)(rbuf+1)==C_PC_CFG_DL_ERR||*(INT16U *)(rbuf+1)==C_PC_CFG_DL_OK){
						if(*(INT16U *)(rbuf+1)==C_PC_CFG_DL_OK&&b_dl)
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
						NetSend(3,NET_CHL_WIFI);	
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
		}
		sn=GetCmd(buf,&rbuf,&sp,len);	
	}
}
INT8U CheckSelf(void)
{
	return 1;
	

	return 0;
}

void RunCmdProc(RFID *p_rfid)
{
	int i;
	if(p_rfid){
		for (i=0;i<p_rfid->n_idcmd;i++){
			IDCMD *p_ic = FindUhfidCmd(p_rfid);
			switch(p_ic->cmd){
					case  CAR_CMD_RUN:
						if(m_car.l_tick>p_ic->tick*100&&(m_car.l_tick<(p_ic->tick+p_ic->runtime)*100||p_ic->runtime==0))
							RunCtrl(p_ic);
						break;
					case 	CAR_CMD_TURN:
						if(m_car.l_tick>p_ic->tick*100&&(m_car.l_tick<(p_ic->tick+p_ic->runtime)*100||p_ic->runtime==0))
							TurnCtrl(p_ic);
						break;
					case 	CAR_CMD_ANGLE:
						if(m_car.l_tick>p_ic->tick*100&&(m_car.l_tick<(p_ic->tick+p_ic->runtime)*100||p_ic->runtime==0))
							TiltCtrl(p_ic);
						break;
					case 	CAR_CMD_PLAY:

						break;
			}
		}
	}
}
#define SPD_DT_LMT 10
void RunCtrl(IDCMD *p_ic)
{
    int dspd;//,rspd;
    //rspd=;m_sensor[0].runspeed[0]+p_ic->spd-m_sensor[0].runspeed[1]>>1;
    dspd=p_ic->spd-m_car.spd;
    if (dspd>SPD_DT_LMT) 
        dspd=SPD_DT_LMT;
    if (dspd<-SPD_DT_LMT) 
        dspd=-SPD_DT_LMT;
	if(m_car.p_rfid[0])
		if(GetFrontDis()<m_car.p_rfid[0]->safedis)
			dspd=-SPD_DT_LMT;
	
    AdjustMotorSpd(MOTOR_RL,dspd);
    AdjustMotorSpd(MOTOR_RR,dspd);
    
		
}
#define ANGLE_CYCLE   3600
#define ANGLE_DT      10
#define ANGLE_MAX     1350
void TurnCtrl(IDCMD *p_ic)
{
 //   static INT8U IsRun=0;
    int dspd,dir=0;
    int sro=p_ic->dis,cro=m_sensor[0].rotation[0]-m_sensor[0].rotation[1];
    if(cro>=(ANGLE_CYCLE>>1)) cro-=ANGLE_CYCLE;
    if(cro<=-(ANGLE_CYCLE>>1)) cro+=ANGLE_CYCLE;

    sro=sro-cro;
    if(sro>=(ANGLE_CYCLE>>1)) sro-=ANGLE_CYCLE;
    if(sro<=-(ANGLE_CYCLE>>1)) sro+=ANGLE_CYCLE;

    if (sro>=0)
        dir=1;//正转
    else
        sro=-sro;
    if (sro>ANGLE_MAX)
        sro=ANGLE_MAX;

    if (sro<ANGLE_DT) {
        OpenTurnBreak();
        SetMotorSpd(MOTOR_TURN,0);
    }
    else {
        CloseTurnBreak();
        if (dir) 
            TurnRight();
        else
            TurnLeft();
      
        dspd=((p_ic->spd*sro)/ANGLE_MAX)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TURN,dspd);
    }
}
#define TILT_MAX    100
#define TILT_DT    10
void TiltCtrl(IDCMD *p_ic)
{
    int dspd,dir=0;
    int stilt=p_ic->dis-m_sensor[0].tilt;

    if (stilt>0) 
        dir=1;
    else   
        stilt=-stilt; 
    if (stilt>TILT_MAX) 
        stilt=TILT_MAX;

    if (stilt<TILT_DT) {
        SetMotorSpd(MOTOR_TILT,0); 
        CloseAllRelay();
    }
    else{
        if (dir){
            CloseDownRelay();
            OpenUpRelay();
        } else{
            CloseUpRelay();
            OpenDownRelay();
        }
        dspd=((p_ic->spd*stilt)/TILT_MAX)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TILT,dspd);

    }
}
void SW_Power(void)
{
	if(sw_power&&(OSTimeGet()&0xff)<50)
		SetSafeLine();
	else
		ResetSafeLine();
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
	return dis;
}

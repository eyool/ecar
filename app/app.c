#include <includes.h>
#include "app.h"
#include "flash_if.h"
#include "math.h"

INT8U LvSystbuf[SYS_TBUF_LEN+8];//系统临时缓存

INT8U *Systbuf;
INT32U G_msg[5];
INT8U SysStatus,b_debug=0;
SENSOR m_sensor[2];//0,当前，1，保留上次
RFID *p_rfid;

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
	SysStatus=0;
	Systbuf=LvSystbuf+8;
	memset((void *)m_sensor,0,sizeof(m_sensor));
	p_rfid=NULL;
	OSTimeDly(2);
	WifiEnable();
	ZigbeeEnable();
	OSTimeDly(500);	

/*	while(WifiStatus()!=WIFI_ST_OK){
		WifiLink();
		OSTimeDly(50);			
	}
	if(CheckSelf()){	
			while(!DowloadCFG()){
				OSTimeDly(200);		
				WifiLink();
			}				
			SysStatus=SYS_STATUS_INIT_OK;	
	}
	else
			SysStatus=SYS_STATUS_INIT_ERR;	*/		
	test();

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

//-------------------
//主任务函数1/4
void MainTaskProc(void *p_msg)
{
/*static UINT lasttick=0;
	UINT ct=OSTimeGet();   
        
        if(lasttick==0) 
          lasttick=ct;*/

	if(p_msg==0){

	  if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_10)){
			 	GPIO_ResetBits(GPIOC,GPIO_Pin_10);
				 OSTimeDly(200);
				}
			else{
				GPIO_SetBits(GPIOC,GPIO_Pin_10);
				 OSTimeDly(300);
				} 
		//----------------------------
		if(!b_debug)
			WifiLink();//让wifi断线从新连接
//		ZigbeeLink();//zigbee断线从新连接
		OSTimeDly(20);	

	}
	else
	{
           //SetMsg((UINT)p_msg,MsgParam);
				 OSTimeDly(10);		
	}
}
//任务函数 2/4
void UartRecvProc(uint8_t chl,uint8_t *buf,uint32_t len)
{
	static INT8U debugcmd=0;
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
					CmdProc(buf,len);
			}
			Debug(buf,len);
		}		
		else if(chl==UART_CHL_ZIGBEE){
					if(IsNetData(buf,len)){
							SetZigbeeLinkTime();
							CmdProc(buf,len);
					}
		}
		else if(chl==UART_CHL_UHFID){
				*(INT32U *)buf=ParseUhfid(buf,buf+5,len);
				p_rfid=GetRfidStruct(*(INT32U *)buf&RDID_BITS);
				//Wifi_send(buf,len);
		}
	}
}


//任务函数 3/4
void AppRunProc(void  *p_msg)
{
    int i;
	if(!p_msg){
		OSTimeDly(20);
		if(p_rfid){
            for (i=0;i<p_rfid->n_idcmd;i++){
                IDCMD *p_ic = FindUhfidCmd(p_rfid);
    			switch(p_ic->cmd){
    				case  CAR_CMD_RUN:
    						RunCtrl(p_ic);
    					break;
    				case 	CAR_CMD_TURN:
    						TurnCtrl(p_ic);
    					break;
    				case 	CAR_CMD_ANGLE:
    						TiltCtrl(p_ic);
    					break;
    				case 	CAR_CMD_PLAY:
    					
    					break;
    			}
            }
		}
	}
	else{
		
	
	}
}
//任务函数 4/4
void SensorProc(void)
{
	INT16S buf[9]; 
//	INT8U *bp=(INT8U *)buf; 	
	memcpy((void *)&m_sensor[1],(void *)&m_sensor[0],sizeof(SENSOR));
	//得到霍尔 ，加速度，压力传感器
	ADXL_GetData((INT8U *)buf,6);
	HMC_GetData((INT8U *)(buf+3),12);
	m_sensor[0].tilt=GetArc(buf[0],buf[1]);
	m_sensor[0].rotation[0]=GetArc(buf[3],buf[5]);	
	m_sensor[0].rotation[1]=GetArc(buf[6],buf[8]);	
  if(GetPSData(PS_CHL_A,(INT32S *)buf))
		m_sensor[0].ps=*(INT32S *)buf;
	//---检查是否回正
	if(CheckCenter())
		m_sensor[0].b_hall|=B_HALL_C;
	else
		m_sensor[0].b_hall&=(~B_HALL_C);	
	//得到外部电压
		m_sensor[0].xv=GetXV();
	//这里添加测距函数
	
	//得到电机数度和脉冲数
		m_sensor[0].turnspeed=GetMotorSpd(MOTOR_TURN);
		m_sensor[0].runspeed[0]=GetMotorSpd(MOTOR_RL);	
		m_sensor[0].runspeed[1]=GetMotorSpd(MOTOR_RR);
		m_sensor[0].t_np=GetMotorPoscn(MOTOR_TURN);
		m_sensor[0].r_np[0]=GetMotorPoscn(MOTOR_RL);	
		m_sensor[0].r_np[1]=GetMotorPoscn(MOTOR_RR);
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



void CmdProc(INT8U *buf,INT8U len)
{
	static INT8U b_dl=0;
	INT8U sp=0,sn;
	INT8U *rbuf;
	INT32U mbuf[4];
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
		}
		sn=GetCmd(buf,&rbuf,&sp,len);	
	}
}
INT8U CheckSelf(void)
{
	return 1;
	

	return 0;
}
#define SPD_DT_LMT 10
void RunCtrl(IDCMD *p_ic)
{
    int dspd,rspd;
    rspd=m_sensor[0].runspeed[0]+p_ic->spd-m_sensor[0].runspeed[1]>>1;
    dspd=p_ic->spd-rspd;
    if (dspd>SPD_DT_LMT) 
        dspd=SPD_DT_LMT;
    if (dspd<-SPD_DT_LMT) 
        dspd=-SPD_DT_LMT;
    AdjustMotorSpd(MOTOR_RL,dspd);
    AdjustMotorSpd(MOTOR_RR,dspd);
    
		
}
void TurnCtrl(IDCMD *p_ic)
{
    int dspd;
    int sro=p_ic->dis,cro=m_sensor[0].rotation[0]-m_sensor[0].rotation[1];
    if (sro<0) sro=-sro;
    if (cro<0) cro=-cro;    
    if(sro>(cro<<2)){
        dspd=p_ic->spd-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TURN,dspd);
    }
    else if(sro>(cro<<1)){
        dspd=(p_ic->spd>>1)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TURN,dspd);    
    }
    else if((sro*3>>2)>cro){
        dspd=(p_ic->spd>>2)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TURN,dspd);    
    }
    else{
        OpenTurnBreak();
        SetMotorSpd(MOTOR_TURN,0);                                          
    }
}
void TiltCtrl(IDCMD *p_ic)
{
    int dspd;
    int stilt=p_ic->dis,ctilt=m_sensor[0].tilt;
    if (stilt<0) stilt=-stilt;
    if (ctilt<0) ctilt=-ctilt; 
    if (stilt>(ctilt<<2)) {
        dspd=p_ic->spd-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TILT,dspd); 
    }
    else if (stilt>(ctilt<<1)) {
        dspd=(p_ic->spd>>1)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TILT,dspd); 
    }
    else if (stilt*3>(ctilt<<2)) {
        dspd=(p_ic->spd>>2)-m_sensor[0].turnspeed;
        if (dspd>SPD_DT_LMT) 
            dspd=SPD_DT_LMT;
        if (dspd<-SPD_DT_LMT) 
            dspd=-SPD_DT_LMT;
        AdjustMotorSpd(MOTOR_TILT,dspd); 
    }
    else if(stilt*7>(ctilt<<3)){
        SetMotorSpd(MOTOR_TILT,0); 
    }
    else
        CloseAllRelay();
}

#include "mymath.h"

//#include "stm32f10x.h"

//#define AST 3000
u16 SN1 = 80;
u16 SN2 = 100;
//#define NDLY() 	_nop_();_nop_();_nop_();_nop_()
s32 avbuf[N_CHECK];
//u16 evbuf[N_CHECK][4];
//u16 evmin[N_CHECK]={0xffff,0xffff,0xffff,0xffff},evmax[N_CHECK]={0,0,0,0};
//u16 ev[N_CHECK][2];
long Qbuf1[N_CHECK][2];
long Qbuf2[N_CHECK][2];
long Mbuf1[N_CHECK];//freq 1:600
long Mbuf2[N_CHECK];//freq 2:750
u16 spp1,spp2;
//u16 am_trig=AM_TRIG;
/*floatN = (float) m_blockSize;   //这里选择16
    k = (int) (0.5 + ((floatN * m_targetFrequency) / m_sampleRate));   
    omega = (float)(2.0 * PI * k) / floatN;   
    sine = (float) sin(omega);   
    cosine = (float) cos(omega);   
    coeff = (float) 2.0 * cosine;   */
const int  coeff1=473;//N=16,1.84776;放大256倍
const int  coeff2=487;//N=20;放大256倍

const float  Lut[46]={
0.000000,0.017455,0.034921,0.052408,0.069927,0.087489,
0.105104,0.122785,0.140541,0.158384,0.176327,0.194380,
0.212557,0.230868,0.249328,0.267949,0.286745,0.305731,
0.324920,0.344328,0.363970,0.383864,0.404026,0.424475,
0.445229,0.466308,0.487733,0.509525,0.531709,0.554309,
0.577350,0.600861,0.624869,0.649408,0.674509,0.700208,
0.726543,0.753554,0.781286,0.809784,0.839100,0.869287,
0.900404,0.932515,0.965689,1.000000
};
/*
void ADC_ISP() interrupt 10
{
	switch(AMX0P)
	{
	case 0:
		AMX0P=1;
		adbuf0[adcid]=(ADC0H<<8)+ADC0L;
		if(IsPP)
			advp[0]+=(ADC0H<<8)+ADC0L;
		else
			advn[0]+=(ADC0H<<8)+ADC0L;
		ProcessSample((ADC0H<<8)+ADC0L,0);
		break;
	case 1:
		AMX0P=0;
		adbuf1[adcid]=(ADC0H<<8)+ADC0L;
		if(IsPP)
			advp[1]+=(ADC0H<<8)+ADC0L;
		else
			advn[1]+=(ADC0H<<8)+ADC0L;
		ProcessSample((ADC0H<<8)+ADC0L,1);
		if(++adcid>=SN)
		{	

			M0=GetMagnitude(0)+M0*15>>4;
			M1=GetMagnitude(1)+M1*15>>4;
			AM[0]=sqrt_16((u32)(M0+M1));
			AM[1]=CalAngle(M0,M1);


			if((advn[0]>>4)+(advp[0]>>4)>0x3ff) PCA0CPH1++;
			else PCA0CPH1--;
			if((advn[1]>>4)+(advp[1]>>4)>0x3ff) PCA0CPH0++;
			else PCA0CPH0--;

		}
		break;
		default:
			AMX0P=0;
		break;
	}
	test=!test;
}
void InitMos()
{

	ND=1;
	usDelay(3);
	ND=0;
	usDelay(100);

	PD=0;
	usDelay(3);
	PD=1;
	usDelay(1);
}
void OpenPMos()
{
	ND=0;
	usDelay(1);

	PD=0;
	usDelay(3);
	PD=1;
	//usDelay(100);
	AMX0P=0;
	adcid=0;
	IsPP=1;
	advp[0]=advp[1]=0;
	ResetGoertzel();
	TL1=0;
	TR1=1;
}
void OpenNMos()
{
	ND=1;
	AMX0P=0;
	adcid=0;
	IsPP=0;
	advn[0]=advn[1]=0;
	ResetGoertzel();
	TL1=0;
	TR1=1;
}*/
u16 CalAngle(int x,int y)
{
	if(x==0)
	{
		if(y>=0) return 90;
		else 	 return 270;
	}
	if(y==0)
	{
		if(x>=0) return 0;
		else 	 return 180;
	}
	if(x>0)
	{
		if(y>0)
		{
			if(x>=y) return ArcTan(x,y);
			else     return 90-ArcTan(y,x);
		}
		else
		{
			y=-y;
			if(x>=y) return 360-ArcTan(x,y);
			else     return 270+ArcTan(y,x);
		}
	}
	else
	{
		x=-x;
		if(y>0)
		{
			if(x>=y) return 180-ArcTan(x,y);
			else     return 90+ArcTan(y,x);
		}
		else
		{
			y=-y;
			if(x>=y) return 180+ArcTan(x,y);
			else     return 270-ArcTan(y,x);
		}
	}
}
u16 ArcTan(int x,int y)//0-45
{
	u8 sp,ep;
	float tan=(float)y/(float)x;
	sp=0;
	ep=45;
	while(sp+1<ep)
	{
		if(Lut[sp+ep>>1]>=tan) ep=sp+ep>>1;
		else	sp=sp+ep>>1;		
	}
	if(Lut[ep]-tan>tan-Lut[ep])	return sp;
	else	return ep;
	
}
long GetMagnitude(long *Q,u16 coe,u16 sn)
{
 long a=Q[0]/sn;//SN1
 long b=Q[1]/sn;//SN1
 long c=a*b;
 if(c>0x100000||c<-0x100000)
	return (a-b)*(a-b)+((c>>2)*(512-coe)>>6);//a*a+b*b-a*b*2+(a>>1)*(b>>1)*(512-coe))>>6;
 else
	return (a-b)*(a-b)+(c*(512-coe)>>8);
  //return Q[0]*Q[0]+ Q[1]*Q[1]-(Q[0]*Q[1]*coe>>8);  
}
void Goertzel(long *Q,s16 sample,u16 coe)
{
  long Qt;
  Qt=(coe * Q[0]>>8) - Q[1] + (long) sample;
  Q[1]= Q[0];
  Q[0]=Qt;
}
/*
void ProcessSample(u16 sample,u8 sel)   
{   
    long Q;   
	if(sel==0)
	{
	    Q = (coeff * Q1>>8) - Q2 + (long) sample;   
	    Q2 = Q1;   
	    Q1 = Q;  
	} 
	else
	{
	    Q = (coeff * Q3>>8) - Q4 + (long) sample;   
	    Q4 = Q3;   
	    Q3 = Q;  		
	}
} */
void ResetGoertzel(long *Q,u16 n)   
{   
  for(u32 i=0;i<n;i++)
    Q[i]=0;  
}  
u16 sqrt_16(u32 M) 
{
 u16 N, i; 
 u32 tmp, ttp; // 结果、循环计数 
 if (M == 0) // 被开方数，开方结果也为0 
     return 0; 
 N = 0;
 tmp = (M >> 30); // 获取最高位：B[m-1] 
 M <<= 2; 
 if (tmp >= 1) // 最高位为1 
 { 
   N ++; // 结果当前位为1，否则为默认的0 
   tmp -= N; 
 } 
 for (i=15; i>0; i--) // 求剩余的15位 
 { 
   N <<= 1; // 左移一位 
   tmp <<= 2; 
   tmp += (M >> 30); // 假设 
   ttp = N; 
   ttp = (ttp<<1)+1; 
   M <<= 2; 
   if (tmp >= ttp) // 假设成立
   { 
     tmp -= ttp; N ++; 
   }
 }
 return N;
} 
void avproc(s16 *sample,u16 n)
{
  for(int i=0;i<N_CHECK;i++){
    avbuf[i]=(s32)sample[i]+(avbuf[i]*255>>8);
  }
}
/*void evproc(s16 *sample,u16 n)
{
  u16 *ep;
  for(int i=0;i<n;i++)
  {
    ep=evbuf[i];
    ep[3]=ep[2];
    ep[2]=ep[1];
    ep[1]=ep[0];     
    ep[0]=Abs((s32)sample[i]-(avbuf[i]>>8));
    ev[i][1]=ev[i][0];
    ev[i][0]=ep[0]+ep[1]+ep[2]+ep[3];   
    if(ev[i][0]<evmin[i])
      evmin[i]=ev[i][0];
    if(ev[i][0]>evmax[i])
      evmax[i]=ev[i][0];    
    evmin[i]++;
  }
}
u32 evTrig(u32 d)
{
  u32 re=N_CHECK;
  for(int i=0;i<N_CHECK;i++)
  {
    if(ev[i][0]<evmin[i]+d&&ev[i][1]<evmin[i]+d)
      re--;
  }
  return re;
}*/
u32 Abs(s32 a)
{
  return (u32)(a>=0?a:-a);
}
void SetTrig(u16 a,u16 b)
{
  if(a>0)
    SN1=a;
  if(b>0) 
    SN2=b;  
}
int ProcessSample(s16 *sample,u16 n)
{
  int re=0;
  if(n>N_CHECK) return re;
  avproc(sample,n);
  /*evproc(sample,n);
  if(evTrig(am_trig)==0)
  {
    spp1=0;
    ResetGoertzel(Qbuf1[0],N_CHECK*2);  

    spp2=0;
    ResetGoertzel(Qbuf2[0],N_CHECK*2); 
    
    GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    return re;
  }
  GPIO_SetBits(GPIOA,GPIO_Pin_8);*/
  Goertzel(Qbuf1[0],sample[0],coeff1);
  Goertzel(Qbuf2[0],sample[0],coeff2); 
  Goertzel(Qbuf1[1],sample[1],coeff1);
  Goertzel(Qbuf2[1],sample[1],coeff2); 
  Goertzel(Qbuf1[2],sample[2],coeff1);
  Goertzel(Qbuf2[2],sample[2],coeff2); 
  Goertzel(Qbuf1[3],sample[3],coeff1);
  Goertzel(Qbuf2[3],sample[3],coeff2);   
           
  if(++spp1>=SN1){
    Mbuf1[0]=GetMagnitude(Qbuf1[0],coeff1,SN1);
    Mbuf1[1]=GetMagnitude(Qbuf1[1],coeff1,SN1);
    Mbuf1[2]=GetMagnitude(Qbuf1[2],coeff1,SN1);
    Mbuf1[3]=GetMagnitude(Qbuf1[3],coeff1,SN1);    
    spp1=0;
    ResetGoertzel(Qbuf1[0],N_CHECK*2);  
    re|=1;
  }

  if(++spp2>=SN2){
    Mbuf2[0]=GetMagnitude(Qbuf2[0],coeff2,SN2);
    Mbuf2[1]=GetMagnitude(Qbuf2[1],coeff2,SN2);
    Mbuf2[2]=GetMagnitude(Qbuf2[2],coeff2,SN2);
    Mbuf2[3]=GetMagnitude(Qbuf2[3],coeff2,SN2);    
    spp2=0;
    ResetGoertzel(Qbuf2[0],N_CHECK*2); 
    re|=2;
  }
 return re;
}
  
u32 GetDir()
{
  u32 ag;
  ag=CalAngle(avbuf[0],avbuf[1]);
  ag+=CalAngle(avbuf[2],avbuf[3])<<16; 
  return ag;
}
//0xab: a=0,1代表左右，b=0,1,2 0=xy合成幅度，1=x,2=y
u32 GetAM(u16 sel)
{
  u32 re;
  if((sel>>4)==0){
    if((sel&0xf)==1)
    {
      re=sqrt_16((u32)(Mbuf1[0]));
      re+=sqrt_16((u32)(Mbuf1[2]))<<16;       
    }
    else if((sel&0xf)==2)
    {
      re=sqrt_16((u32)(Mbuf1[1]));
      re+=sqrt_16((u32)(Mbuf1[3]))<<16;           
    }
    else
    {
      re=sqrt_16((u32)(Mbuf1[0]+Mbuf1[1]));
      re+=sqrt_16((u32)(Mbuf1[2]+Mbuf1[3]))<<16;  
    }
  }
  else{
    if((sel&0xf)==1)
    {
      re=sqrt_16((u32)(Mbuf2[0]));
      re+=sqrt_16((u32)(Mbuf2[2]))<<16;     
    }
     else if((sel&0xf)==2)
    {
      re=sqrt_16((u32)(Mbuf2[1]));
      re+=sqrt_16((u32)(Mbuf2[3]))<<16;         
    }
    else
    {
      re=sqrt_16((u32)(Mbuf2[0]+Mbuf2[1]));
      re+=sqrt_16((u32)(Mbuf2[2]+Mbuf2[3]))<<16;
    }
  }
   return re;  
}
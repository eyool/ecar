#ifndef __MYMATH_H
#define __MYMATH_H
typedef unsigned char u8;
typedef unsigned short u16;
typedef  short    s16;
typedef unsigned int u32;
typedef  int s32;


#define N_CHECK 4

#define AM_TRIG 0x30
u16 CalAngle(int x,int y);
u16 ArcTan(int x,int y);

long GetMagnitude(long *Q,u16 coe,u16 sn);
void Goertzel(long *Q,s16 sample,u16 coe);
void ResetGoertzel(long *Q,u16 n);
int ProcessSample(s16 *sample,u16 n);
u16 sqrt_16(u32 M);
void avproc(s16 *sample,u16 n);
//void evproc(s16 *sample,u16 n);
//u32 evTrig(u32 d);
u32 Abs(s32 a);
u32 GetDir();
u32 GetAM(u16 sel);
void SetTrig(u16 a,u16 b);
#endif
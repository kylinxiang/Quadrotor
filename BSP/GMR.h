#ifndef __GMR_H__
#define __GMR_H__

#include "stm32f10x.h"
#include "math.h"
#include "led.h"
#include "sysconfig.h"
#include "BSP.h"

void GMR_Init(void);
void GMR_GetRaw(void);
void GMR_Offset(void);
void GMR_GetData(T_int16_xyz *gmr);

extern T_int16_xyz	GMR_OFFSET;  
extern u8 GMR_flag;  //根据中断按键次数计数
extern float mx,my,mz,Xsf,Ysf,Zsf;
extern int mx_mid,my_mid,mz_mid;

#endif

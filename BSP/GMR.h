#ifndef __GMR_H__
#define __GMR_H__

#include "stm32f10x.h"
#include "math.h"
#include "led.h"

void ADC_Configuration(void);
void EXTI_Configuration(void);
void get_GMR_data_before_calibration(void);
void calibration_GMR(void);
void get_GMR_data_after_calibration(void);

   
extern u8 GMR_flag;  //根据中断按键次数计数
extern float mx_temp,my_temp,mz_temp,mx_mid,my_mid,mz_mid,Xsf,Ysf,Zsf;
extern float mx,my,mz,norm;

#endif

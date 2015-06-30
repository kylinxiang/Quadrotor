#ifndef _BSP_BSP_H_
#define _BSP_BSP_H_
#include "sysconfig.h"
#include "eeprom.h"
#include "MPU6050.h"
#include "control.h"
#include "GMR.h"   //保存偏移数据时可以直接使用数据

void Nvic_Init(void);

void EE_INIT(void);
void EE_SAVE_ACC_OFFSET(void);
void EE_READ_ACC_OFFSET(void);

void EE_SAVE_GYRO_OFFSET(void);
void EE_READ_GYRO_OFFSET(void);

//void EE_SAVE_COMPASS_OFFSET(void);
//void EE_READ_COMPASS_OFFSET(void);

void EE_SAVE_PID(void);
void EE_READ_PID(void);

#endif

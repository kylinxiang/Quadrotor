#include "data_transfer.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 Data_Check,Send_Status,Send_Senser,Send_RCData,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_PID4,Send_PID5,Send_PID6,Send_MotoPwm;
u8 data_to_send[50];
void Nrf_Check_Event(void)
{
	u8 sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);

	if(sta & (1<<RX_DR))
	{
		u8 rx_len = NRF_Read_Reg(R_RX_PL_WID);
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);// read receive payload from RX_FIFO buffer
			//LED1_ONOFF();
			Data_Receive_Anl(NRF24L01_RXDATA,rx_len);
		}
		else 
		{
			NRF_Write_Reg(FLUSH_RX,0xff);//清空缓冲区
		}
	}

	if(sta & (1<<TX_DS))
	{
		//LED1_ONOFF();
	}

	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}

	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}

void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头

	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			MPU6050_CalOff_Acc();
		if(*(data_buf+4)==0X02)
			MPU6050_CalOff_Gyr();
		if(*(data_buf+4)==0X03)
		{MPU6050_CalOff_Acc();MPU6050_CalOff_Gyr();}
//		if(*(data_buf+4)==0X04)
//			Cal_Compass();
//		if(*(data_buf+4)==0X05)
//			MS5611_CalOffset();
	}
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			Send_PID1 = 1;
			Send_PID2 = 1;
			Send_PID3 = 1;
			Send_PID4 = 1;
			Send_PID5 = 1;
			Send_PID6 = 1;
		}
		if(*(data_buf+4)==0X02)
			Send_Offset = 1;
	}
#ifndef CONTROL_USE_RC
	if(*(data_buf+2)==0X03)
	{
		Rc_D.THROTTLE = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
		Rc_D.YAW = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
		Rc_D.ROLL = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
		Rc_D.PITCH = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
		Rc_D.AUX1 = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
		Rc_D.AUX2 = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
		Rc_D.AUX3 = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
		Rc_D.AUX4 = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
		Rc_D.AUX5 = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
		Rc_D.AUX6 = (vs16)(*(data_buf+21)<<8)|*(data_buf+22);
		Rc_Fun(&Rc_D,&Rc_C);
	}
#endif
	if(*(data_buf+2)==0X10)								//PID1
	{
			PID_ROL.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_ROL.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			PID_ROL.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_PIT.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_PIT.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PID_PIT.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_YAW.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_YAW.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_YAW.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(*(data_buf+2),sum);
		  EE_SAVE_PID();
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
			PID_ALT.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_ALT.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			PID_ALT.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_POS.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_POS.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			PID_POS.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_PID_1.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_PID_1.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_PID_1.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(*(data_buf+2),sum);
		  EE_SAVE_PID();
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
			PID_PID_2.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_PID_2.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			PID_PID_2.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_PID_3.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_PID_3.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			PID_PID_3.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_PID_4.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_PID_4.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_PID_4.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(*(data_buf+2),sum);
		  EE_SAVE_PID();
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
			Data_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)							
	{
			Data_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)							
	{
			Data_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X16)								//OFFSET
	{
			AngleOffset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			AngleOffset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
	}
}

u8 Nrf_Get_FIFOSta(void)
{
	return NRF_Read_Reg(NRF_READ_REG + FIFO_STATUS);
}

void Data_Exchange(void)
{
#ifdef DATA_TRANSFER_USE_SPI_NRF
	Nrf_Check_Event();
	u8 sta = Nrf_Get_FIFOSta();
	if((sta & (1<<4))==0)
		return;
#endif
	if(Send_Status)
	{
		Send_Status = 0;
		Data_Send_Status();
	}
	else if(Send_Senser)
	{
		Send_Senser = 0;
		Data_Send_Senser();
	}
	else if(Send_PID1)
	{
		Send_PID1 = 0;
		Data_Send_PID1();
	}
	else if(Send_PID2)
	{
		Send_PID2 = 0;
		Data_Send_PID2();
	}
	else if(Send_PID3)
	{
		Send_PID3 = 0;
		Data_Send_PID3();
	}
	else if(Send_PID4)
	{
		Send_PID4 = 0;
		Data_Send_PID4();
	}
	else if(Send_PID5)
	{
		Send_PID5 = 0;
		Data_Send_PID5();
	}
	else if(Send_PID6)
	{
		Send_PID6 = 0;
		Data_Send_PID6();
	}
	else if(Send_RCData)
	{
		Send_RCData = 0;
		Data_Send_RCData();
	}
	else if(Send_Offset)
	{
		Send_Offset = 0;
		Data_Send_OFFSET();
	}
	else if(Send_MotoPwm)
	{
		Send_MotoPwm = 0;
		Data_Send_MotoPWM();
	}
}
void Data_Send_Status(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	vs16 _temp;
	_temp = (int)(Att_Angle.rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Att_Angle.pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Att_Angle.yaw*100);
	//_temp = (int)(Mag_Heading*100); //电子罗盘预留
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	vs32 _temp2 = Alt;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
		
	if(Rc_C.ARMED==0)			data_to_send[_cnt++]=0xA0;	//锁定
	else if(Rc_C.ARMED==1)		data_to_send[_cnt++]=0xA1;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}

void Data_Send_Senser(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Acc.X);
	data_to_send[_cnt++]=BYTE0(Acc.X);
	data_to_send[_cnt++]=BYTE1(Acc.Y);
	data_to_send[_cnt++]=BYTE0(Acc.Y);
	data_to_send[_cnt++]=BYTE1(Acc.Z);
	data_to_send[_cnt++]=BYTE0(Acc.Z);
	data_to_send[_cnt++]=BYTE1(Gyr.X);
	data_to_send[_cnt++]=BYTE0(Gyr.X);
	data_to_send[_cnt++]=BYTE1(Gyr.Y);
	data_to_send[_cnt++]=BYTE0(Gyr.Y);
	data_to_send[_cnt++]=BYTE1(Gyr.Z);
	data_to_send[_cnt++]=BYTE0(Gyr.Z);
	data_to_send[_cnt++]=BYTE1(mx_data);  //电子罗盘预留数据
	data_to_send[_cnt++]=BYTE0(mx_data);
	data_to_send[_cnt++]=BYTE1(my_data);
	data_to_send[_cnt++]=BYTE0(my_data);
	data_to_send[_cnt++]=BYTE1(mz_data);
	data_to_send[_cnt++]=BYTE0(mz_data);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}

void Data_Send_RCData(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Rc_D.THROTTLE);
	data_to_send[_cnt++]=BYTE0(Rc_D.THROTTLE);
	data_to_send[_cnt++]=BYTE1(Rc_D.YAW);
	data_to_send[_cnt++]=BYTE0(Rc_D.YAW);
	data_to_send[_cnt++]=BYTE1(Rc_D.ROLL);
	data_to_send[_cnt++]=BYTE0(Rc_D.ROLL);
	data_to_send[_cnt++]=BYTE1(Rc_D.PITCH);
	data_to_send[_cnt++]=BYTE0(Rc_D.PITCH);
	
	data_to_send[_cnt++]=BYTE1(Moto_PWM_1);  //显示PWM具体值 可用来测试ADC值（电子罗盘）
	data_to_send[_cnt++]=BYTE0(Moto_PWM_1);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_2);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_2);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_3);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_3);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_4);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_4);

//		data_to_send[_cnt++]=0;  //显示PWM时将这些屏蔽
//		data_to_send[_cnt++]=0;
//		data_to_send[_cnt++]=0;
//		data_to_send[_cnt++]=0;
//		data_to_send[_cnt++]=0;
//		data_to_send[_cnt++]=0;
//		data_to_send[_cnt++]=0;
//		data_to_send[_cnt++]=0;
		data_to_send[_cnt++]=BYTE1(PID_ROL.P);
		data_to_send[_cnt++]=BYTE0(PID_ROL.P);;
		data_to_send[_cnt++]=0;
		data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}


void Data_Send_MotoPWM(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Moto_PWM_1-1000);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_1-1000);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_2-1000);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_2-1000);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_3-1000);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_3-1000);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_4-1000);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_4-1000);
	data_to_send[_cnt++]=BYTE1(GMR_OFFSET.X);
	data_to_send[_cnt++]=BYTE0(GMR_OFFSET.X);
	data_to_send[_cnt++]=BYTE1(GMR_OFFSET.Y);
	data_to_send[_cnt++]=BYTE0(GMR_OFFSET.Y);
	data_to_send[_cnt++]=BYTE1(GMR_OFFSET.Z);
	data_to_send[_cnt++]=BYTE0(GMR_OFFSET.Z);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_8);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}


void Data_Send_OFFSET(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x16;
	data_to_send[_cnt++]=0;
	vs16 _temp = AngleOffset_Rol*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = AngleOffset_Pit*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}
void Data_Send_PID1(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	vs16 _temp;
	_temp = PID_ROL.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ROL.I * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ROL.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PIT.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PIT.I * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PIT.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_YAW.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_YAW.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_YAW.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}
void Data_Send_PID2(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;
	
	vs16 _temp;
	_temp = PID_ALT.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ALT.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ALT.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_POS.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_POS.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_POS.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;//PID_PID_1.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;//PID_PID_1.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;//PID_PID_1.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}
void Data_Send_PID3(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x12;
	data_to_send[_cnt++]=0;
	
	vs16 _temp;
	_temp = PID_PID_2.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_2.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_2.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_3.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_3.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_3.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_4.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_4.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_4.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}
void Data_Send_PID4(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x13;
	data_to_send[_cnt++]=0;
	
	vs16 _temp;
	_temp = PID_PID_5.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_5.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_5.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_6.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_6.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_6.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_7.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_7.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_7.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}
void Data_Send_PID5(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x14;
	data_to_send[_cnt++]=0;
	
	vs16 _temp;
	_temp = PID_PID_8.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_8.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_8.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_9.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_9.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_9.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_10.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_10.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_10.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}
void Data_Send_PID6(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x15;
	data_to_send[_cnt++]=0;
	
	vs16 _temp;
	_temp = PID_PID_11.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_11.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_11.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_12.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_12.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_12.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF_TxPacket(data_to_send,_cnt);
#endif
}
void Data_Send_Check(u8 head,u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	
	data_to_send[6]=sum;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,7);
#else
	NRF_TxPacket(data_to_send,7);
#endif
}

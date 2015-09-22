#include "GMR.h"

u16 AD_Value[10][3];
#define ADC1_DR_Address ((uint32_t)0x4001244C)  //定义ADC1转化结果寄存器

void ADC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);//使能ADC1时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;	//配置PA2,PA3为模拟输入端AIN1,AIN2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//配置PB1为模拟输入端口AIN3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	ADC_DeInit(ADC1);
	DMA_DeInit(DMA1_Channel1); 
	DMA_InitStructure.DMA_PeripheralBaseAddr=ADC1_DR_Address; 					 
	DMA_InitStructure.DMA_MemoryBaseAddr=(u32)&AD_Value; 
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_BufferSize=30; 
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord; 
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord; 
	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority=DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M=DMA_M2M_Disable; 
	
	DMA_Init(DMA1_Channel1,&DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1,ENABLE);

	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;	 
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None; 										
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfChannel=3; 
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 3, ADC_SampleTime_55Cycles5);
	
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void EXTI_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	 
  //KEY1(PB14) KEY2(PB15)
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);	//JTAG
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);   

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);		
  EXTI_InitStructure.EXTI_Line=EXTI_Line14; 
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;		
  EXTI_Init(&EXTI_InitStructure);	 

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource15);		
  EXTI_InitStructure.EXTI_Line=EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	  
	
	//NVIC
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;	
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  NVIC_Init(&NVIC_InitStructure);  
			
  NVIC_Init(&NVIC_InitStructure); 
}


int mx_max=0,mx_min=3300,my_max=0,my_min=3300,mz_max=0,mz_min=3300;
float Xsf=0.0,Ysf=0.0,Zsf=0.0;
int mx_data,my_data,mz_data,mx_mid,my_mid,mz_mid;
float mx,my,mz;
static int cal=1;  // 防止计算偏移的循环函数重入
T_int16_xyz	GMR_OFFSET={0,0,0};

void GMR_GetRaw(void)
{
	u8 i,j;
	u32 sum[3];
	for(j=0;j<3;j++)
	{
		sum[j]=0;
		for(i=0;i<10;i++)
		{
			sum[j]+=AD_Value[i][j];
		}
	}
	mx_data=(int)(sum[0]/10);//*3.3/4096;  //得到初步读取值
	my_data=(int)(sum[1]/10);//*3.3/4096; 
	mz_data=(int)(sum[2]/10)+1500;//*3.3/4096;		
}

void GMR_Offset(void)  //校准GMR读数  //可使用中断按键来完成校准工作
{	
	if(GMR_flag==3)  //取得中值
	{
			mx_mid = (int)(mx_max+mx_min)/2;
			my_mid = (int)(my_max+my_min)/2;
			mz_mid = (int)(mz_max+mz_min)/2;
			
			GMR_OFFSET.X = mx_mid;
			GMR_OFFSET.Y = my_mid;
			GMR_OFFSET.Z = mz_mid;
			
			Xsf = (mz_max-mz_min)/(mx_max-mx_min);
			Ysf = (mz_max-mz_min)/(my_max-my_min);
			Zsf = 1;	
			return;
	}	
	
	for(int i=0;i<800;i++)
	{
		GMR_GetRaw();  //均值滤波值
		Delay_ms_led(10);
		if(GMR_flag==1)  //双轴GMR的最大和最小值  可以使用static的全局数  按一次时为1 按第二次为2，第三次取中值 相应灯闪三次
		{
			if(mx_data>mx_max)
				mx_max=mx_data;
			if(mx_data<mx_min)
				mx_min=mx_data;
			if(my_data>my_max)
				my_max=my_data;
			if(my_data<my_min)
				my_min=my_data;
		}
		if(GMR_flag==2)  //单轴GMR的最大和最小值
		{
			if(mz_data>mz_max)
				mz_max=mz_data;
			if(mz_data<mz_min)
				mz_min=mz_data;
		}		
	}	
	LED_FLASH();	
}

void GMR_GetData(T_int16_xyz *gmr)  //得到GMR校正数据后的纠正读数
{
	GMR_GetRaw();
	
	mx = Xsf*(mx_data-mx_mid);
	my = Ysf*(my_data-my_mid);
	mz = Zsf*(mz_data-mz_mid);
	gmr->X = mx;
	gmr->Y = my;
	gmr->Z = mz;
		
}	


//按键中断处理函数
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		++GMR_flag;
		
		if(GMR_flag==1&&cal)
		{ 
			cal=0;
			LED_FLASH(); 
		  GMR_Offset(); //进行平面校准
		}
		if(GMR_flag==2)
		{
			LED_FLASH();
			LED_FLASH();
			GMR_Offset();
			//进行垂直校准
		}
		if(GMR_flag==3)
		{
			LED_FLASH();
			LED_FLASH();	
			LED_FLASH();	
			GMR_Offset();
			
			EE_SAVE_COMPASS_OFFSET();  	//将偏移值写入EEPROOM	
			GMR_flag=0;	
			cal=1;  //使计算偏移的函数能够再次进入	
	
			return;   //直接跳出整个程序
		}
		EXTI_ClearITPendingBit(EXTI_Line14);  //如果放在子程序中，处理函数会一直执行到中断标志清除

	}
 
	else if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		//可以进行MPU6050校准
		EXTI_ClearITPendingBit(EXTI_Line15); 
	} 
}

void GMR_Init(void)
{
	ADC_Configuration();
	EXTI_Configuration();
}


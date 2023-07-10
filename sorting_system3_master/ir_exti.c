#include "stm32f10x_exti.h"
#include "config.h"
#define PA4 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)
#define PA5 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)
#define PA6 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)
static unsigned int IRCheck=0;

//////////////////////////////////////////////////////////////////////////////////
							  
//////////////////////////////////////////////////////////////////////////////////
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

  //GPIOA.4
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;	//PA4
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//中断模式
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发 EXTI_Trigger_Rising  下降沿触发 EXTI_Trigger_Falling
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//初始化外设寄存器

 //GPIOA.5
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  	EXTI_Init(&EXTI_InitStructure);	  	//初始化外设寄存器


  //GPIOA.6	  
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6); 
		EXTI_InitStructure.EXTI_Line=EXTI_Line6;
//		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_Init(&EXTI_InitStructure);		


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//EXTI_Line0
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  	NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//EXTI_Line2
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  	NVIC_Init(&NVIC_InitStructure);


//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//EXTI_Line3
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级1
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
//  	NVIC_Init(&NVIC_InitStructure);  	  
 
}
int RetIRCheckFunc(void)
{
	return IRCheck;
}

void ClearIRCheck(void)
{
	IRCheck=0;
}

void IRCheckFunc(void)//检查传感器
{
	
 if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_SET || \
	 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_SET || \
 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_SET)){
	 IRCheck =1;
 }
}


void EXTI4_IRQHandler(void)
{
	if(PA4==(uint32_t)Bit_SET)
	{
		IRCheck =1;
		//debug_out1("PA5U",4);
	}else if(PA4==(uint32_t)Bit_RESET)
	{
		//IRCheck =1;
		//debug_out1("PA5D",4);
	}
	EXTI_ClearITPendingBit(EXTI_Line4); 
}
 

//void EXTI2_IRQHandler(void)
//{
//	debug_out1("2",1);
//	if(PA5==(uint32_t)Bit_SET)
//	{
//		//IRCheck =1;
//		debug_out1("2PA4U",5);
//	}else if(PA5==(uint32_t)Bit_RESET)
//	{
//		//IRCheck =1;
//		debug_out1("2PA4D",5);
//	}
//	EXTI_ClearITPendingBit(EXTI_Line2);
//}

//void EXTI3_IRQHandler(void)
//{
//	debug_out1("3",1);
//	if(PA6==(uint32_t)Bit_SET)
//	{
//		//IRCheck =1;
//		debug_out1("3PA6U",5);
//	}else if(PA6==(uint32_t)Bit_RESET){
//		//IRCheck =1;
//		debug_out1("3PA6D",5);
//	}
//	EXTI_ClearITPendingBit(EXTI_Line3);
//}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		if(PA5==(uint32_t)Bit_SET)
		{
			IRCheck =1;
			//debug_out1("PA4U",4);
		}else if(PA5==(uint32_t)Bit_RESET)
		{
			//IRCheck =1;
			//debug_out1("PA4D",4);
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		if(PA6==(uint32_t)Bit_SET)
		{
			IRCheck =1;
			//debug_out1("PA6U",4);
		}else if(PA6==(uint32_t)Bit_RESET){
			//IRCheck =1;
			//debug_out1("PA6D",4);
		}
		EXTI_ClearITPendingBit(EXTI_Line6); 
	}
}
 

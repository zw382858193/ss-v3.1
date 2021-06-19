
#include "device_can.h"
#include "uart.h"
#include "MotorDriver.h"
#include "config.h"
#include <stdarg.h>

int WorkStatus;
Recv_CAN_Data_Stu Recv_CAN_Data;
u8 CAN_Mode_Init(CANBaudRate BaudRate,u8 mode,unsigned int FilterNumber,unsigned int FilterId,unsigned int FilterMaskId)
{
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
 	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
 	NVIC_InitTypeDef  NVIC_InitStructure;
	if(FilterNumber>13) return 1;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);  	
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//can rx
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//can tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	CAN_ITConfig(CAN1,CAN_IT_FMP0|CAN_IT_FMP1,ENABLE);//FIFO0 MSG irq

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

	CAN_InitStructure.CAN_TTCM=DISABLE;//非时间触发通信模式
	CAN_InitStructure.CAN_ABOM=ENABLE;//软件自动离线管理
	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒
	CAN_InitStructure.CAN_NART=ENABLE;//禁止报文自动传送
	CAN_InitStructure.CAN_RFLM=DISABLE;//报文不锁定，新的覆盖旧的
	CAN_InitStructure.CAN_TXFP=DISABLE;//优先级由报文标识符决定
	CAN_InitStructure.CAN_Mode= mode;//0 普通模式 1 回环模式

	switch(BaudRate){
		case BaudRate5Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;
			CAN_InitStructure.CAN_Prescaler=600;
			break;
		case BaudRate10Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler=600;
			break;
		case BaudRate20Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
			CAN_InitStructure.CAN_Prescaler=100;
			break;
		case BaudRate25Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler=240;
			break;
		case BaudRate40Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler=150;
			break;
		case BaudRate50Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
			CAN_InitStructure.CAN_Prescaler=40;
			break;
		case BaudRate80Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler=75;
			break;
		case BaudRate100Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
			CAN_InitStructure.CAN_Prescaler=20;
			break;
		case BaudRate125Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler=48;
			break;
		case BaudRate200Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
			CAN_InitStructure.CAN_Prescaler=9;
			break;
		case BaudRate250Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler=24;
			break;
		case BaudRate400Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
			CAN_InitStructure.CAN_Prescaler=10;
			break;
		case BaudRate500Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;
			CAN_InitStructure.CAN_Prescaler=6;
			break;
		case BaudRate800Kbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
			CAN_InitStructure.CAN_Prescaler=5;
			break;
		case BaudRate1Mbps:	
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
			CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler=6;
			break;
		default: break;
	}
	CAN_Init(CAN1, &CAN_InitStructure);
	
	//过滤器参数配置
	CAN_FilterInitStructure.CAN_FilterNumber=FilterNumber;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=(FilterId>>16);
	CAN_FilterInitStructure.CAN_FilterIdLow=(FilterId&0xff);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(FilterMaskId>>16);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=(FilterMaskId&0xff);
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);//初始化过滤器组

	return 0;
}

u8 Can_Send_Msg(unsigned int id,u8* msg,u8 len)
{
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12; //标准标识符，低11位有效
  TxMessage.ExtId=id;//扩展标识符，低29位有效
  TxMessage.IDE=CAN_Id_Standard;//CAN_ID_EXT;//0;//标准帧
  TxMessage.RTR=CAN_RTR_DATA;//0;//数据帧
  TxMessage.DLC=len;//发送数据长度
  for(i=0;i<len;i++)
		TxMessage.Data[i]=msg[i];       
  mbox= CAN_Transmit(CAN1, &TxMessage);
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;
  if(i>=0XFFF)return 1;
  return 0;
}

#define CAN_DEBUG 0
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	int i=0;
 	CanRxMsg RxMessage;
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
//	Recv_CAN_Data.Recv_CAN_ID = RxMessage.ExtId;
//	Recv_CAN_Data.motor_error_code.car_error=RxMessage.Data[0];
//	Recv_CAN_Data.motor_error_code.servo_error=RxMessage.Data[1];
//	Recv_CAN_Data.goodsnumber=RxMessage.Data[2];
//	Recv_CAN_Data.sensor_position[Recv_CAN_Data.Recv_CAN_ID]=RxMessage.Data[3];
	
//	switch(RxMessage.Data[0]){//can comm mode
//		case 0x01://heartbeat
//			Slave_Device.Slave_Device_Car_ONLINE[RxMessage.StdId]=RxMessage.Data[1];
//			Slave_Device.Slave_Device_Servo_ONLINE[RxMessage.StdId]=RxMessage.Data[2];
//			break;
//		case 0x02://sensor input
//			Slave_Device.Sensor_Position[RxMessage.StdId]=RxMessage.Data[1];
//			if(Slave_Device.Sensor_Position[RxMessage.StdId]==0){
//				GPIO_ResetBits(GPIOC,GPIO_Pin_7);
//			}else if(Slave_Device.Sensor_Position[RxMessage.StdId]==1){
//				GPIO_SetBits(GPIOC,GPIO_Pin_7);
//			}
//			break;
//		default:break;
//	}

	for(i=0;i<CAN_RECV_BUF_LEN;i++){
		can_recv_buf.Recv_Buf_Len0++;
		can_recv_buf.CAN_Recv_Buf0[i]=RxMessage.Data[i];
	}
	if((can_recv_buf.CAN_Recv_Buf0[1]==0x73)&&(can_recv_buf.CAN_Recv_Buf0[2]==0x74)&&(can_recv_buf.CAN_Recv_Buf0[3]==0x6f)&&(can_recv_buf.CAN_Recv_Buf0[4]==0x70)){
		WorkStatus=can_recv_buf.CAN_Recv_Buf0[0];
	}else if((can_recv_buf.CAN_Recv_Buf0[1]==0x75)&&(can_recv_buf.CAN_Recv_Buf0[2]==0x6e)&&(can_recv_buf.CAN_Recv_Buf0[3]==0x73)&&(can_recv_buf.CAN_Recv_Buf0[4]==0x74)&&(can_recv_buf.CAN_Recv_Buf0[5]==0x6f)&&(can_recv_buf.CAN_Recv_Buf0[6]==0x70)){
		WorkStatus=0;
	}else if(can_recv_buf.CAN_Recv_Buf0[0]=='e'&&can_recv_buf.CAN_Recv_Buf0[1]=='n'&&can_recv_buf.CAN_Recv_Buf0[2]=='d'){
		debug_out1((char *)can_recv_buf.CAN_Recv_Buf0,can_recv_buf.Recv_Buf_Len0);//分拣完成上报
	}
	
}

u8 Can_Receive_Msg(u8 *buf)
{
 	u32 i;
	CanRxMsg RxMessage;
  if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  for(i=0;i<8;i++){
    buf[i]=RxMessage.Data[i];
		//my_sprintf_32("",buf[i]);
	}

	return RxMessage.DLC;	
}


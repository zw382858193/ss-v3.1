#include "device_can.h"
#include "uart.h"
#include "config.h"
#include "MotorDriver.h"

void can_comm(void)
{
	int i;
	unsigned int delayTick=0;
	unsigned char send_modbus_buf[8];
	uint16_t data_tmp;
	uchar CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	if(can_recv_buf.Recv_Buf_Len0==CAN_RECV_BUF_LEN){
		switch(can_recv_buf.CAN_Recv_Buf0[1]){//协议功能
			case 0x01://win cmd
				if(device.id<can_recv_buf.CAN_Recv_Buf0[0]){//读id
					device.goodsnumber++;
				}else if(device.id==can_recv_buf.CAN_Recv_Buf0[0]){
					int j;
					CanSendMsg[0]='o';
					CanSendMsg[1]='k';
					Can_Send_Msg(device.id,CanSendMsg,8);
					for(i=0;i<DEVICESNUM;i++){
						if(windows[i].Win==0){
							windows[i].Win=(can_recv_buf.CAN_Recv_Buf0[2]<<24)||(can_recv_buf.CAN_Recv_Buf0[3]<<16)|\
							(can_recv_buf.CAN_Recv_Buf0[4]<<8)|(can_recv_buf.CAN_Recv_Buf0[5]&0xff);
							windows[i].goodslist=device.goodsnumber;
							windows[i].BillCodeAndPortCode[0]='e';
							windows[i].BillCodeAndPortCode[1]='n';
							windows[i].BillCodeAndPortCode[2]='d';
							for(j=0;j<(can_recv_buf.Recv_Buf_Len0-6);j++){
								windows[i].BillCodeAndPortCode[j+3]=can_recv_buf.CAN_Recv_Buf0[j+7];//记录货物单号和格口码
								windows[i].BillCodeAndPortCodeLen++;
							}
//							CanSendMsg[0]=0xff;
//							CanSendMsg[7]=0xff;
							break;
						}
					}
				}
				break;
			case 0x02://heartbeat cmd
				if(device.id==can_recv_buf.CAN_Recv_Buf0[0]){
					CanSendMsg[0] = device.id;
					CanSendMsg[1] = 0x02;//heartbeat cmd
					CanSendMsg[2] = 0x03;//read byte number
					CanSendMsg[3] = device.motor_error_code.car_error;
					CanSendMsg[4] = device.motor_error_code.servo_error;
					CanSendMsg[5] = device.status;//0x10 0x20锁格口 0x01~0x0f传感器
					device.heartbeat=1;
					Can_Send_Msg(device.id,CanSendMsg,8);
					
				}
				break;
			case 0x03://send read modbus
				
				if(device.id == can_recv_buf.CAN_Recv_Buf0[0]){
					if(can_recv_buf.CAN_Recv_Buf0[2]==0x1){//car1
						//send
						send_modbus_buf[0]=0x01;
						send_modbus_buf[1]=0x03;
						send_modbus_buf[2]=can_recv_buf.CAN_Recv_Buf0[3];
						send_modbus_buf[3]=can_recv_buf.CAN_Recv_Buf0[4];
						data_tmp=(can_recv_buf.CAN_Recv_Buf0[5]<<8)|can_recv_buf.CAN_Recv_Buf0[6];
						set_mobus_crc(send_modbus_buf,data_tmp,2);
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x2){//car2 servo motor
						send_modbus_buf[0]=0x01;
						send_modbus_buf[1]=0x03;
						send_modbus_buf[2]=can_recv_buf.CAN_Recv_Buf0[3];
						send_modbus_buf[3]=can_recv_buf.CAN_Recv_Buf0[4];
						data_tmp=(can_recv_buf.CAN_Recv_Buf0[5]<<8)|can_recv_buf.CAN_Recv_Buf0[6];
						set_mobus_crc(send_modbus_buf,data_tmp,4);
					}
				}
				break;
			case 0x06://send write modbus
				
				if(device.id == can_recv_buf.CAN_Recv_Buf0[0]){
					if(can_recv_buf.CAN_Recv_Buf0[2]==0x1){//car1
						//send
						send_modbus_buf[0]=0x01;
						send_modbus_buf[1]=0x06;
						send_modbus_buf[2]=can_recv_buf.CAN_Recv_Buf0[3];
						send_modbus_buf[3]=can_recv_buf.CAN_Recv_Buf0[4];
						data_tmp=(can_recv_buf.CAN_Recv_Buf0[5]<<8)|can_recv_buf.CAN_Recv_Buf0[6];
						set_mobus_crc(send_modbus_buf,data_tmp,2);

					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x2){//car2 servo motor
						send_modbus_buf[0]=0x01;
						send_modbus_buf[1]=0x06;
						send_modbus_buf[2]=can_recv_buf.CAN_Recv_Buf0[3];
						send_modbus_buf[3]=can_recv_buf.CAN_Recv_Buf0[4];
						data_tmp=(can_recv_buf.CAN_Recv_Buf0[5]<<8)|can_recv_buf.CAN_Recv_Buf0[6];
						set_mobus_crc(send_modbus_buf,data_tmp,4);
					}
				}
				break;
			case 0x08://send write modbus
				
				if(device.id == can_recv_buf.CAN_Recv_Buf0[0] || can_recv_buf.CAN_Recv_Buf0[0]==0x00){
					if(can_recv_buf.CAN_Recv_Buf0[0]==device.id){//car1
						if(can_recv_buf.CAN_Recv_Buf0[2]==0x20&&can_recv_buf.CAN_Recv_Buf0[3]==0x08){//修改ID地址 01 08 20 08 00 02 00 00
							device.id = can_recv_buf.CAN_Recv_Buf0[5];
							WriteFlashOneWord(FLASH_DEVICE_ID_ADDR,device.id);
						}
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x00&&can_recv_buf.CAN_Recv_Buf0[3]==0x00){//小车运动状态
						if(can_recv_buf.CAN_Recv_Buf0[5]==0x00){
							treadmill.work_status=FOREWARD;
						}else if(can_recv_buf.CAN_Recv_Buf0[5]==0x01){
							treadmill.work_status=BACKWARD;
							
						}else if(can_recv_buf.CAN_Recv_Buf0[5]==0x05){
							treadmill.work_status=STOP;
						}
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x00&&can_recv_buf.CAN_Recv_Buf0[3]==0x01){//伺服运动状态
						if(can_recv_buf.CAN_Recv_Buf0[5]==0x00){
							servo.work_status=WIN1;
						}else if(can_recv_buf.CAN_Recv_Buf0[5]==0x01){
							servo.work_status=WIN2;
						}else if(can_recv_buf.CAN_Recv_Buf0[5]==0x05){
							servo.work_status=STOP;
						}
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x20&&can_recv_buf.CAN_Recv_Buf0[3]==0x01){//设置小车速度
						treadmill.speed = can_recv_buf.CAN_Recv_Buf0[4]<<8|can_recv_buf.CAN_Recv_Buf0[5];
						set_mobus_crc(treadmill.SET_SPEED,treadmill.speed,2);
						WriteFlashOneWord(FLASH_CAR_SPEED_DATA_ADDR,treadmill.speed);
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x01&&can_recv_buf.CAN_Recv_Buf0[3]==0x90){//设置伺服速度 01 08 01 90 05 dc 
						servo.speed = can_recv_buf.CAN_Recv_Buf0[4]<<8|can_recv_buf.CAN_Recv_Buf0[5];
						set_mobus_crc(servo.PosSpeed,servo.speed,4);
						WriteFlashOneWord(FLASH_SERVO_MOTOR_SPEED_ADDR,servo.speed);
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x01&&can_recv_buf.CAN_Recv_Buf0[3]==0x68){//设置posset
						servo.position = can_recv_buf.CAN_Recv_Buf0[4]<<24|can_recv_buf.CAN_Recv_Buf0[5]<<16|can_recv_buf.CAN_Recv_Buf0[6]<<8|can_recv_buf.CAN_Recv_Buf0[7];
						my_sprintf_32("pos",servo.position,1);
						WriteFlashOneWord(FLASH_SERVO_MOTOR_POSITION_ADDR,servo.position);
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x00&&can_recv_buf.CAN_Recv_Buf0[3]==0x50){//设置伺服加减速
						servo.adtime = can_recv_buf.CAN_Recv_Buf0[4]<<8|can_recv_buf.CAN_Recv_Buf0[5];
						set_mobus_crc(servo.ATime,servo.adtime,4);
						//delay
						delayTick = get_system_tick();
						while((get_system_tick()-delayTick)<100);
						set_mobus_crc(servo.DTime,servo.adtime,4);
						WriteFlashOneWord(FLASH_SERVO_MOTOR_ADTIME_ADDR,servo.adtime);
					}else if(can_recv_buf.CAN_Recv_Buf0[2]=='i'&&can_recv_buf.CAN_Recv_Buf0[3]=='n'&&can_recv_buf.CAN_Recv_Buf0[4]=='i'&&can_recv_buf.CAN_Recv_Buf0[5]=='t'){//初始化设备 00 08 init
						initServoLiChuang();
						Car1init();
					}
				}
				break;
			case 0x10://透传
				if(device.id == can_recv_buf.CAN_Recv_Buf0[0]){
					if(can_recv_buf.CAN_Recv_Buf0[2]==0x1){//car1
						debug_out2(((char *)can_recv_buf.CAN_Recv_Buf0)+3,can_recv_buf.CAN_Recv_Buf0[3]);
					}else if(can_recv_buf.CAN_Recv_Buf0[2]==0x2){//car2 servo motor
						debug_out4(((char *)can_recv_buf.CAN_Recv_Buf0)+3,can_recv_buf.CAN_Recv_Buf0[3]);
					}
				}
				break;
			default:break;
		}can_recv_buf.Recv_Buf_Len0=0;
	}
}

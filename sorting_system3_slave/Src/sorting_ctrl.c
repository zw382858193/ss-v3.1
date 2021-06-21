#include "device_can.h"
#include "uart.h"
#include "config.h"
#include "MotorDriver.h"

/****************************************
ASCLL码表
a 0x61 j 0x6a s 0x73 0 0x30 9 0x39
b 0x62 k 0x6b t 0x74 1 0x31
c 0x63 l 0x6c u 0x75 2 0x32
d 0x64 m 0x6d v 0x76 3 0x33
e 0x65 n 0x6e w 0x77 4 0x34
f 0x66 o 0x6f x 0x78 5 0x35
g 0x67 p 0x70 y 0x79 6 0x36
h 0x68 q 0x71 z 0x7a 7 0x37
i 0x69 r 0x72 ~ 0x7e 8 0x38
*****************************************/

void goods_export(unsigned int TimeOutTick)
{
	static int state=0;
	static unsigned int GoodsScanTick;
	int i;
	if(state==0){//感应到货物 传感器输出低电平 IO口被拉高
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_RESET){
//			device.status&=0xfe;
//		}
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_RESET){
//			device.status&=0xfd;
//		}
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_RESET){
//			device.status&=0xfb;
//		}
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_RESET){
//			device.status&=0xf7;
//		}
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_RESET){
//			device.status&=0xef;
//		}
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_RESET){
//			device.status&=0xdf;
//		}
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_SET){
//			device.status|=0x01;
//		}
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_SET){
//			device.status|=0x02;
//		}
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_SET){
//			device.status|=0x04;
//		}
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_SET){
//			device.status|=0x08;
//		}
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_SET){
//			device.status|=0x10;
//		}
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_SET){
//			device.status|=0x20;
//		}
		if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_SET)||(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_SET)||\
			(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_SET)||(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_SET)||\
		(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_SET)||(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_SET)){

		
			
			GoodsScanTick = get_system_tick();
			state = 1;
		}
	}else if(state == 1){
		if(retTickDiff(GoodsScanTick)>TimeOutTick){
			if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_SET)||(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_SET)||\
			(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_SET)||(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_SET)||\
		(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_SET)||(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_SET)){
			//if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7)==(uint32_t)Bit_SET){
				if(device.goodsnumber>0){
					//windows[i].goodslist--;
					device.goodsnumber--;
				}else{
					for(i=0;i<DEVICESNUM;i++){
						if(windows[i].Win!=0){
							if(windows[i].Win & 0x01){
								if(!(device.status&0x10)){//是否锁格口
									servo.work_status=WIN1;
									Can_Send_Msg(device.id,windows[i].BillCodeAndPortCode,windows[i].BillCodeAndPortCodeLen);
								}
								//send_servor_motor_pos(servo.SetPos,servo.position);
							}else{
								if(!(device.status&0x20)){//是否锁格口
									servo.work_status=WIN2;
									Can_Send_Msg(device.id,windows[i].BillCodeAndPortCode,windows[i].BillCodeAndPortCodeLen);
								}
								//send_servor_motor_pos(servo.SetPos,-servo.position);
							}
							windows[i].Win=0;
						}
					}
				}
			}
			state = 2;
			GoodsScanTick = get_system_tick();
		}
	}else if(state==2){
		if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_RESET)||(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_RESET)||\
			(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_RESET)||(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_RESET)||\
		(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_RESET)||(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_RESET)){//goods leave
			device.status&=0xfd;//解除货物卡住
			state = 0;
		}
		else if(retTickDiff(GoodsScanTick)>1000){//货物卡住
			state=3;
			device.status|=0x02;
		}
	}else if(state ==3){
		state = 0;

	}
}

void ServoCtrl(void)
{
	static unsigned int DelayTick=0;
	static int state=0;
	static int check=0;
	uchar *p;
	p = get_uart2_buf();
	if(servo.work_status==WIN1){
		state = 10;
		check=0;
		servo.work_status=WAIT;
	}else if(servo.work_status==WIN2){
		state = 20;
		check=0;
		servo.work_status=WAIT;
	}else if(servo.work_status==STOP){
		state = 30;
		check=0;
		servo.work_status=WAIT;
	}
	if(state == 0){
		DelayTick = get_system_tick();
		state=1;
	}else if(state ==1){
		if(retTickDiff(DelayTick)>1000){
			state=40;
		}
	}else if(state == 10){
		rs232_clear4();
		send_servor_motor_pos(servo.SetPos,servo.position);
		DelayTick = get_system_tick();
		state++;
	}else if(state == 11){
		if(retTickDiff(DelayTick)>30){
			if(memcmp(get_rs232_buf4(),servo.SetPos,strlen((char *)servo.SetPos))==0){
				state =0;
			}else{
				if(check < 2){
					check++;
					state--;
				}else{
					state=0;
				}
			}
		}
	}else if(state == 20){
		rs232_clear4();
		send_servor_motor_pos(servo.SetPos,-servo.position);
		DelayTick = get_system_tick();
		state++;
	}else if(state == 21){
		if(retTickDiff(DelayTick)>30){
			if(memcmp(get_rs232_buf4(),servo.SetPos,strlen((char *)servo.SetPos))==0){
				state =0;
			}else{
				if(check < 2){
					check++;
					state--;
				}else{
					state=0;
				}
			}
		}
	}else if(state == 30){
		rs232_clear4();
		//debug_out4((char *)servo.disable,8);
		send_servor_motor_data(servo.enable,0x10);//急停
		DelayTick = get_system_tick();
		state++;
	}else if(state == 31){
		if(retTickDiff(DelayTick)>30){
			if(memcmp(get_rs232_buf4(),servo.disable,strlen((char *)servo.enable))==0){
				state =0;
			}else{
				if(check < 10){
					check++;
					state--;
				}else{
					state=0;
				}
			}
		}
	}else if(state == 40){
		rs232_clear4();
		debug_out4((char *)servo.check,8);
		DelayTick = get_system_tick();
		state++;
	}else if(state == 41){
		if(retTickDiff(DelayTick)>30){
			if(memcmp(get_rs232_buf4(),servo.check,2)==0){
				device.motor_error_code.servo_error=p[3];
				state =0;
			}else{
				device.motor_error_code.servo_error=0xff;
				state =0;
			}
		}
	}
}

void CarCtrl(void)
{
	static unsigned int DelayTick=0;
	static int state=0;
	static int check=0;
	uchar *p;
	p = get_uart2_buf();
	
	if(treadmill.work_status==BACKWARD){
		treadmill.work_status=WAIT;
		state = 10;
		check =0;
	}else if(treadmill.work_status==FOREWARD){
		treadmill.work_status=WAIT;
		state = 40;
		check =0;
	}else if(treadmill.work_status==STOP){
		state = 20;
		check =0;
	}
	if(state == 0){
		DelayTick = get_system_tick();
		state = 1;
	}else if(state == 1){
		if(retTickDiff(DelayTick)>1000){
			state = 30;//检查小车状态
		}
	}else if(state == 10){
		rs232_clear2();
		id_run_ctrl(treadmill.BACKWARD,0x01);
		DelayTick = get_system_tick();
		state = 11;
	}else if(state == 11){
		if(retTickDiff(DelayTick)>50){
			if(memcmp(get_rs232_buf2(),treadmill.BACKWARD,8)==0){
				state = 0;
			}else{
				if(check <2){
					check++;
					state = 10;
				}else{
					state =0;
				}
			}
		}
	}else if(state == 20){
		rs232_clear2();
		id_run_ctrl(treadmill.STOP,0x01);
		DelayTick = get_system_tick();
		state = 21;
	}else if(state == 21){
		if(retTickDiff(DelayTick)>50){
			if(memcmp(get_rs232_buf2(),treadmill.STOP,8)==0){
				state = 0;
			}else{
				if(check <2){
					check++;
					state = 20;
				}else{
					state =0;
				}
			}
		}
	}else if(state == 30){
		rs232_clear2();
		id_run_ctrl(treadmill.CHECK_XIAOCHE_STATE,0x01);
		DelayTick = get_system_tick();
		state = 31;
	}else if(state == 31){
		if(retTickDiff(DelayTick)>50){
			if(memcmp(get_rs232_buf2(),treadmill.CHECK_XIAOCHE_STATE,2)==0){
				state = 0;
				device.motor_error_code.car_error = p[4];
			}else{
				device.motor_error_code.car_error = 0xff;
				state = 0;
			}
		}
	}else if(state == 40){
		rs232_clear2();
		id_run_ctrl(treadmill.FOREWARD,0x01);
		DelayTick = get_system_tick();
		state = 41;
	}else if(state == 41){
		if(retTickDiff(DelayTick)>50){
			if(memcmp(get_rs232_buf2(),treadmill.FOREWARD,8)==0){
				state = 0;
			}else{
				if(check <2){
					check++;
					state = 40;
				}else{
					state =0;
				}
			}
		}
	}
}

void StopKey(void)//pc6 急停
{
	static int state=0;
	static unsigned int tick;
	int i;
	uchar CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	if(state==0){//急停
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==(uint32_t)Bit_RESET){
			tick=get_system_tick();
			state=1;
		}
	}else if(state == 1){
		if(retTickDiff(tick)>10){
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==(uint32_t)Bit_RESET){
				state=2;
				
			}else{
				state=0;
			}
		}
	}else if(state == 2){//加锁
		CanSendMsg[0]=device.id;
		CanSendMsg[1]=0x73;
		CanSendMsg[2]=0x74;
		CanSendMsg[3]=0x6f;
		CanSendMsg[4]=0x70;
		Can_Send_Msg(0x00,CanSendMsg,8);//最高级别
		tick=get_system_tick();
		device.status|=0x01;
		servo.work_status=STOP;
		treadmill.work_status=STOP;
		for(i=0;i<DEVICESNUM;i++){
			windows[i].Win=0;
		}device.goodsnumber=0;
		state++;
	}else if(state == 3){
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==(uint32_t)Bit_SET){
			tick=get_system_tick();
			state++;
		}else if(retTickDiff(tick)>100){
			state--;
		}
	}else if(state == 4){
		if(retTickDiff(tick)>10){
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==(uint32_t)Bit_SET){//解锁
				state=0;
				CanSendMsg[0]=device.id;
				CanSendMsg[1]=0x75;
				CanSendMsg[2]=0x6e;
				CanSendMsg[3]=0x73;
				CanSendMsg[4]=0x74;
				CanSendMsg[5]=0x6f;
				CanSendMsg[6]=0x70;
				Can_Send_Msg(0x00,CanSendMsg,8);//最高级别
				device.status&=0xfe;
			}else{
				state=3;
			}
		}
	}
}

void LockKey(void)//pb0 pb1 锁口
{
	static int state=0;
	static unsigned int tick;
	if(state==0){
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET){
			tick=get_system_tick();
			state=1;
		}else{
			device.status&=0xef;
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET){
			tick=get_system_tick();
			state=10;
		}else{
			device.status&=0xdf;
		}
	}else if(state == 1){
		if(retTickDiff(tick)>10){
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET){
				state=2;
			}else{
				state=0;
			}
		}
	}else if(state == 2){
		device.status|=0x10;
		state=0;
	}else if(state == 10){
		if(retTickDiff(tick)>10){
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET){
				state=11;
			}else{
				state=0;
			}
		}
	}else if(state == 11){
		device.status|=0x20;
		state=0;
	}
}

#include "device_can.h"
#include "uart.h"
#include "config.h"
#include "MotorDriver.h"

/****************************************
ASCLL码表
a 0x61 j 0x6a s 0x73 0 0x30 9 0x39
b 0x62 k 0x6b t 0x74 1 0x31 ^ 0x5e
c 0x63 l 0x6c u 0x75 2 0x32
d 0x64 m 0x6d v 0x76 3 0x33
e 0x65 n 0x6e w 0x77 4 0x34
f 0x66 o 0x6f x 0x78 5 0x35
g 0x67 p 0x70 y 0x79 6 0x36
h 0x68 q 0x71 z 0x7a 7 0x37
i 0x69 r 0x72 ~ 0x7e 8 0x38
*****************************************/

void shengPaiDriverXOR(char *data)
{
	data[7] = data[1]^data[2]^data[3]^data[4]^data[5]^data[6];
	debug_out2(data,8);
}
void ir_test(void){//感应到货物 传感器输出低电平 IO口被拉高

	static unsigned int tick=0;
	if(retTickDiff(tick)>1000){
		tick=get_system_tick();
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_RESET){
			my_sprintf_32("PA4",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_SET){
			my_sprintf_32("PA4",1,1);
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_RESET){
			my_sprintf_32("PA5",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_SET){
			my_sprintf_32("PA5",1,1);
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_RESET){
			my_sprintf_32("PA6",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_SET){
			my_sprintf_32("PA6",1,1);
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_RESET){
			my_sprintf_32("PA7",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_SET){
			my_sprintf_32("PA7",1,1);
		}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_RESET){
			my_sprintf_32("PC4",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_SET){
			my_sprintf_32("PC4",1,1);
		}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_RESET){
			my_sprintf_32("PC5",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_SET){
			my_sprintf_32("PC5",1,1);
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET){
			my_sprintf_32("PB0",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET){
			my_sprintf_32("PB0",1,1);
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET){
			my_sprintf_32("PB1",0,1);
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET){
			my_sprintf_32("PB1",1,1);
		}
		
	}
}

void goods_export(unsigned int TimeOutTick)
{
	static int state=0;
	static unsigned int GoodsScanTick;
	char CanSendMsg[6]={0x00,0x01,0x03,0x00,0x00,0x00};//device.id wincmd(0x01) byte carCode servoCode lockStatus
	int ret;
	if(state==0){//感应到货物 传感器输出低电平 IO口被拉高
		//ir_test();
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
				CanSendMsg[0]=device.id;
				CanSendMsg[3] = device.motor_error_code.car_error;
				CanSendMsg[4] = device.motor_error_code.servo_error;
				CanSendMsg[5] = device.status;//0x10 0x20锁格口 0x01~0x0f传感器

				ret=Can_Send_Msg(device.id,(uchar *)CanSendMsg,6);
				if(ret == 1){
					debug_out1("ir send failed\r",strlen("ir send failed\r"));
				}else{
					state = 2;
					GoodsScanTick = get_system_tick();
				}
			}else{
				state = 2;
				GoodsScanTick = get_system_tick();
			}
		}
	}else if(state==2){
		if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_RESET)&&(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_RESET)&&\
			(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_RESET)&&(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==(uint32_t)Bit_RESET)&&\
		(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==(uint32_t)Bit_RESET)&&(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==(uint32_t)Bit_RESET)){//goods leave
			device.status&=0xfd;//解除货物卡住
			state = 0;
		}
		else if(retTickDiff(GoodsScanTick)>1000){//货物卡住
			state=3;
			device.status|=0x02;
		}
	}else if(state ==3){
		state=0;
	}
}

void ServoCtrl(int ms)
{
	static unsigned int DelayTick=0;
	static int state=0;
	static int check=0;
	static int setPosRevise;
	static int ForwordPosOffsetTimes_L=0,ForwordPosOffsetTimes_R=0;//根据偏移次数逐渐加大
	static int BackwordPosOffsetTimes_L=0,BackwordPosOffsetTimes_R=0;
	uchar *p;
	p = get_uart4_buf();
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
	}else if(servo.work_status==START){
		state = 50;
		check=0;
		servo.work_status=WAIT;
	}else if(servo.work_status==SETSPEED){
		state = 60;
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
		if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET)&&(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
			ForwordPosOffsetTimes_L+=500;
			ForwordPosOffsetTimes_R=0;
			setPosRevise=500+ForwordPosOffsetTimes_L;
		}else if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET)&&(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET)){
			ForwordPosOffsetTimes_R-=500;
			ForwordPosOffsetTimes_L=0;
			setPosRevise=-500+ForwordPosOffsetTimes_R;
		}else if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET)&&(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET)){
			ForwordPosOffsetTimes_L+=500;
			ForwordPosOffsetTimes_R=0;
			setPosRevise=500+ForwordPosOffsetTimes_L;
		}else{
			setPosRevise=0;
			ForwordPosOffsetTimes_L=0;
			ForwordPosOffsetTimes_R=0;
		}
		rs232_clear4();
		send_servor_motor_pos(servo.SetPos,(servo.position+setPosRevise));
		DelayTick = get_system_tick();
		state++;
	}else if(state == 11){
		if(retTickDiff(DelayTick)>ms){
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
		if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET)&&(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
			BackwordPosOffsetTimes_L-=500;
			BackwordPosOffsetTimes_R=0;
			setPosRevise=-500+BackwordPosOffsetTimes_L;
		}else if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET)&&(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET)){
			BackwordPosOffsetTimes_R+=500;
			BackwordPosOffsetTimes_L=0;
			setPosRevise=500+BackwordPosOffsetTimes_R;
		}else if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET)&&(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET)){
			BackwordPosOffsetTimes_R-=500;
			BackwordPosOffsetTimes_L=0;
			setPosRevise=-500+BackwordPosOffsetTimes_R;
		}else{
			BackwordPosOffsetTimes_L=0;
			BackwordPosOffsetTimes_R=0;
			setPosRevise=0;
		}
		rs232_clear4();
		send_servor_motor_pos(servo.SetPos,(-servo.position+setPosRevise));
		DelayTick = get_system_tick();
		state++;
	}else if(state == 21){
		if(retTickDiff(DelayTick)>ms){
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
		if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
			state++;//如果皮带归位了就正常流程
		}else{//否则就等待2秒后再休眠
			//ResetServoBelt(300);`
			DelayTick = get_system_tick();
			state=35;
		}
	}else if(state == 31){
		rs232_clear4();
		send_servor_motor_data(servo.enable,0x10);//急停
		DelayTick = get_system_tick();
		state++;
	}else if(state == 32){
		if(retTickDiff(DelayTick)>ms){
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
	}else if(state == 35){
		if(retTickDiff(DelayTick)>2000){//给正在运行的皮带一点运转时间
			state=31;
		}
	}else if(state == 40){
		rs232_clear4();
		//debug_out4((char *)servo.check,8);
		set_mobus_crc((uchar *)servo.check,0x01,4);
		DelayTick = get_system_tick();
		state++;
	}else if(state == 41){
		if(retTickDiff(DelayTick)>ms){
			if(memcmp(get_rs232_buf4(),servo.check,2)==0){
				device.motor_error_code.servo_error=p[4];//检测每bit上的数据 例如 报警在第二bit则 p[4]&0x02
				state =0;
			}else{
				device.motor_error_code.servo_error=0xff;
				state =0;
			}
		}
	}else if(state == 50){
		rs232_clear4();
		send_servor_motor_data(servo.enable,0x01);//enable
		DelayTick = get_system_tick();
		state++;
	}else if(state == 51){
		if(retTickDiff(DelayTick)>ms){
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
	}else if(state == 60){
		rs232_clear4();
		set_mobus_crc(servo.PosSpeed,servo.speed,4);
		DelayTick = get_system_tick();
		state++;
	}else if(state == 61){
		if(retTickDiff(DelayTick)>ms){
			if(memcmp(get_rs232_buf4(),servo.PosSpeed,strlen((char *)servo.PosSpeed))==0){
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
	}
}

void CarCtrl(void)
{
	static unsigned int DelayTick=0;
	static int state=0;
	static int check=0;
	uchar *p;
	#if SHENGPAI
		static char RUN[8] = {0x8A,0x01,0x00,0x00,0x00,0x00,0x01,0x00};
		static char DATA[8] = {0x85,0x01,0x53,0x00,0x00,0x00,0x00,0x52};
	#endif
	p = get_uart2_buf();
	//my_sprintf_32("state",state,1);
	if(treadmill.work_status==BACKWARD){
		treadmill.work_status=WAIT;
		state = 10;
		check =0;
	}else if(treadmill.work_status==FOREWARD){
		treadmill.work_status=WAIT;
		state = 40;
		check =0;
	}else if(treadmill.work_status==STOP){
		treadmill.work_status=WAIT;
		state = 20;
		check =0;
	}else if(treadmill.work_status==SETSPEED){
		treadmill.work_status=WAIT;
		state = 50;
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
		#if SHENGPAI
			DATA[1]=0x01;
			if((treadmill.speed/10) > 127)
			{
				DATA[2]=127;
			}else{
				DATA[2]=treadmill.speed/10;
			}
			shengPaiDriverXOR(DATA);
		#else
			id_run_ctrl(treadmill.BACKWARD,0x01);
		#endif
		
		DelayTick = get_system_tick();
		state = 11;
	}else if(state == 11){
		if(retTickDiff(DelayTick)>50){
			#if SHENGPAI
			if(get_rs232_len2()==4){
				if(p[0] == 0x99 && p[3] == (p[1] ^ p[2])){
					if(p[2]==0x00 || p[2] == 0x10){
						debug_out2(RUN,8);
					}else if((p[2] & 0x01)==0x01){
						device.motor_error_code.car_error = 0x01;//过流
					}else if((p[2] & 0x02)==0x02){
						device.motor_error_code.car_error = 0x05;//霍尔错误
					}else{
						device.motor_error_code.car_error = 0x80 | p[2];
					}
				}
				state =0;
			}else{
				if(check <2){
					check++;
					state = 10;
				}else{
					state =0;
					debug_out2(RUN,8);
				}
			}
			#else
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
		#endif
		}
	}else if(state == 20){
		rs232_clear2();
		#if SHENGPAI
			DATA[2]=0x00;
			shengPaiDriverXOR(DATA);
			debug_out1("stop\r",5);
		#else
			id_run_ctrl(treadmill.STOP,0x01);
		#endif
		DelayTick = get_system_tick();
		state = 21;
	}else if(state == 21){
		if(retTickDiff(DelayTick)>50){
			#if SHENGPAI
			if(get_rs232_len2()==4){
				if(p[0] == 0x99 && p[3] == (p[1] ^ p[2])){
					if(p[2]==0x00 || p[2] == 0x10){
						debug_out2(RUN,8);
						debug_out1("car send succ\r",strlen("car send succ\r"));
					}else if((p[2] & 0x01)==0x01){
						device.motor_error_code.car_error = 0x01;//过流
					}else if((p[2] & 0x02)==0x02){
						device.motor_error_code.car_error = 0x05;//霍尔错误
					}else{
						device.motor_error_code.car_error = 0x80 | p[2];
					}
				}
				state=0;
			}else{
				
				if(check <2){
					check++;
					state = 20;
				}else{
					debug_out1("car send fail\r",strlen("car send fail\r"));
					state =0;
					debug_out2(RUN,8);
				}
			}
			#else
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
		#endif
		}
	}else if(state == 30){
		rs232_clear2();
		#if SHENGPAI
			shengPaiDriverXOR(DATA);
		#else
			id_run_ctrl(treadmill.CHECK_XIAOCHE_STATE,0x01);
		#endif
		DelayTick = get_system_tick();
		state = 31;
	}else if(state == 31){
		if(retTickDiff(DelayTick)>50){
			#if SHENGPAI
			if(get_rs232_len2()>=4){
				if(p[0] == 0x99 && p[3] == (p[1] ^ p[2])){
					if(p[2]==0x00 || p[2] == 0x10){
						device.motor_error_code.car_error = 0x00;
					}else if((p[2] & 0x01)==0x01){
						device.motor_error_code.car_error = 0x01;//过流
					}else if((p[2] & 0x02)==0x02){
						device.motor_error_code.car_error = 0x05;//霍尔错误
					}else{
						device.motor_error_code.car_error = 0x80 | p[2];//添加0x80兼容代码 转换时需要去掉最高位0x80
					}
				}
			}else{
					device.motor_error_code.car_error = 0xff;
			}
			
			state=0;
			#else
				if(memcmp(get_rs232_buf2(),treadmill.CHECK_XIAOCHE_STATE,2)==0){
					state = 0;
					device.motor_error_code.car_error = p[4];
				}else{
					device.motor_error_code.car_error = 0xff;
					state = 0;
				}
			#endif
		}
	}else if(state == 40){
		rs232_clear2();
		#if SHENGPAI
			DATA[1]=0x41;
			if((treadmill.speed/10) > 127)
			{
				DATA[2]=127;
			}else{
				DATA[2]=treadmill.speed/10;
			}
			shengPaiDriverXOR(DATA);
			
		#else
			id_run_ctrl(treadmill.FOREWARD,0x01);
		#endif
		DelayTick = get_system_tick();
		state = 41;
	}else if(state == 41){
		if(retTickDiff(DelayTick)>50){
			#if SHENGPAI
				if(get_rs232_len2()==4){
					if(p[0] == 0x99 && p[3] == (p[1] ^ p[2])){
						if(p[2]==0x00 || p[2] == 0x10){
							debug_out2(RUN,8);
						}else if((p[2] & 0x01)==0x01){
							device.motor_error_code.car_error = 0x01;//过流
						}else if((p[2] & 0x02)==0x02){
							device.motor_error_code.car_error = 0x05;//霍尔错误
						}else{
							device.motor_error_code.car_error = 0x80 | p[2];
						}
					}state=0;
				}else{
					if(check <2){
						check++;
						state = 40;
					}else{
						state =0;
						debug_out2(RUN,8);
					}
				}
			#else
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
		  #endif
		}
	}else if(state == 50){
		rs232_clear2();
		#if SHENGPAI
			if((treadmill.speed/10) > 127)
			{
				DATA[2]=127;
			}else{
				DATA[2]=treadmill.speed/10;
			}
			shengPaiDriverXOR(DATA);
		#else
			set_mobus_crc(treadmill.SET_SPEED,treadmill.speed,2);
		#endif
		DelayTick = get_system_tick();
		state++;
	}else if(state == 51){
		if(retTickDiff(DelayTick)>50){
			#if SHENGPAI
			if(get_rs232_len2()==4){
				if(p[0] == 0x99 && p[3] == (p[1] ^ p[2])){
					if(p[2]==0x00 || p[2] == 0x10){
						//debug_out2(RUN,8);
					}else if((p[2] & 0x01)==0x01){
						device.motor_error_code.car_error = 0x01;//过流
					}else if((p[2] & 0x02)==0x02){
						device.motor_error_code.car_error = 0x05;//霍尔错误
					}else{
						device.motor_error_code.car_error = 0x80 | p[2];
					}
				}
				state=0;
			}else{
				if(check <2){
					check++;
					state--;
				}else{
					state =0;
				}
			}
			#else
			if(memcmp(get_rs232_buf2(),treadmill.SET_SPEED,8)==0){
					state = 0;
			}else{
				if(check <2){
					check++;
					state--;
				}else{
					state =0;
				}
			}
		#endif
		}
	}
}

void StopKey(void)//pc8 急停
{
	static int state=0;
	static unsigned int tick;
	int i;
	uchar CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	if(state==0){//急停
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==(uint32_t)Bit_RESET){
			tick=get_system_tick();
			state=1;
		}
	}else if(state == 1){
		if(retTickDiff(tick)>10){
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==(uint32_t)Bit_RESET){
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
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==(uint32_t)Bit_SET){
			tick=get_system_tick();
			state++;
		}else if(retTickDiff(tick)>100){
			state--;
		}
	}else if(state == 4){
		if(retTickDiff(tick)>10){
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==(uint32_t)Bit_SET){//解锁
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

void LockKey(void)//pc6 pc7 锁口
{
	static int state=0;
	static unsigned int tick;
	if(state==0){
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==(uint32_t)Bit_RESET){
			tick=get_system_tick();
			state=1;
		}else{
			device.status&=0xef;
		}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)==(uint32_t)Bit_RESET){
			tick=get_system_tick();
			state=10;
		}else{
			device.status&=0xdf;
		}
	}else if(state == 1){
		if(retTickDiff(tick)>10){
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==(uint32_t)Bit_RESET){
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
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)==(uint32_t)Bit_RESET){
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

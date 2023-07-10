#include "config.h"
#include "device_can.h"
#include "MotorDriver.h"
#include "flash.h"

//#include <stm32f10x.h>

unsigned int alarm_system_tick = 0;
treadmill_def treadmill;
window windows[DEVICESNUM];
#define INIT_TICK 50
void ir_init(void)
{
	GPIO_InitTypeDef ir_config;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// PA4 5 6 7sensor
	ir_config.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	ir_config.GPIO_Mode = GPIO_Mode_IPU;
	ir_config.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&ir_config);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE); 
	ir_config.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	ir_config.GPIO_Mode = GPIO_Mode_IPU;
	ir_config.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC,&ir_config);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	ir_config.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	ir_config.GPIO_Mode = GPIO_Mode_IPU;
	ir_config.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&ir_config);


}

uint8_t ret_ir(void)
{
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)){
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
		return 1;
	}else if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)){
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		return 1;
	}	
//	else if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)){
//		return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8);
//	}
	return 0;
}

void Car1init(void)
{
	int state = 0,check=0;
	unsigned int tick;

	#ifdef BLDC
	id_set_mobus_crc(treadmill.SET_485,0,0x00);
	#else
	id_run_ctrl(treadmill.EN_485,0x00);
	id_set_mobus_crc(treadmill.SET_485,5,0x00);
	#endif
	tick = get_system_tick();
	while((get_system_tick()-tick)<2000);//wait for system power up
	while(1){
		if(state == 0){
			
			rs232_clear2();
			id_run_ctrl(treadmill.STOP,0x01);
			tick = get_system_tick();
			state = 1;

		}else if(state == 1){
			if(retTickDiff(tick) > INIT_TICK){
				if(memcmp(get_uart2_buf(),treadmill.STOP,8)==0){
					check = 0;
					state = 2;
				}else{
					if(check<3){
						check++;
						state=0;
					}else{
						check=0;
						state = 2;
					}
				}
			}
		}else if(state == 2){
			rs232_clear2();
			state = 3;
			id_set_mobus_crc(treadmill.SET_SPEED,treadmill.speed,CAR1);
			tick = get_system_tick();

		}else if(state == 3){

			if ((get_system_tick()-tick) > INIT_TICK) {
				
				if(memcmp(get_uart2_buf(),treadmill.SET_SPEED,8)==0){//
					check=0;
					state = 4;  //
				}else{
					if(check<3){
						check++;
						state=2;
					}else{
						check=0;
						state = 4;
					}
				}
			}

		} else if(state == 4) {
			rs232_clear2();
			id_set_mobus_crc(treadmill.ADTIME_A,treadmill.adtime,CAR1);
			state = 5;
			tick = get_system_tick();

		} else if(state == 5) {
			
			if ((get_system_tick()-tick) > INIT_TICK) {
				if(memcmp(get_uart2_buf(),treadmill.ADTIME_A,8)==0){
					check=0;
					state = 6;  //      state = 6;
				}else{
					if(check<3){
						check++;
						state=4;
					}else{
						check=0;
						state = 6;
					}
				}
			}
		} else if(state == 6) {
			rs232_clear2();
			id_set_mobus_crc(treadmill.ADTIME_D,treadmill.adtime,CAR1);
			state = 7;
			tick = get_system_tick();

		}else if(state == 7){
					
			if((get_system_tick()-tick) > INIT_TICK) {
				if(memcmp(get_uart2_buf(),treadmill.ADTIME_D,8)==0){
					check=0;
					state = 100;
				}else{
					if(check<3){
						check++;
						state=6;
					}else{
						check=0;
						state = 100;
					}
				}
			}

		}else if(state == 100){
			//treadmill.work_onoff=ON;
			servo.work_status = WAIT;
			treadmill.work_status=WAIT;
			rs232_clear1();
			rs232_clear2();
			rs232_clear4();
			break;
		}
	}
}

void ShengPaiCarInit(void)
{
	int state=20,check=0;
	unsigned int DelayTick;
	uchar *p;
	char RUN[8] = {0x8A,0x01,0x00,0x00,0x00,0x00,0x01,0x00};
	char DATA[8] = {0x85,0x01,0x00,0x00,0x00,0x00,0x00,0x01};
	while(1){
		if(state == 20){
		rs232_clear2();
		debug_out2(DATA,8);
		DelayTick = get_system_tick();
		state = 21;
		}else if(state == 21){
			if(retTickDiff(DelayTick)>50){
				if(get_rs232_len2()==4){
					if(p[0] == 0x99 && p[3] == (p[1] ^ p[2])){
//						if(p[2]==0x00 || p[2] == 0x10){
//							debug_out2(RUN,8);
//						}else if((p[2] & 0x01)==0x01){
//							device.motor_error_code.car_error = 0x01;//过流
//						}else if((p[2] & 0x02)==0x02){
//							device.motor_error_code.car_error = 0x05;//霍尔错误
//						}else{
//							device.motor_error_code.car_error = 0x80 | p[2];
//						}
						debug_out2(RUN,8);
					}
					state=100;
				}else{
					if(check <2){
						check++;
						state = 20;
					}else{
						state =100;
						debug_out2(RUN,8);
					}
				}
			}
		}else if(state == 100){
			//treadmill.work_onoff=ON;
			servo.work_status = WAIT;
			treadmill.work_status=WAIT;
			rs232_clear1();
			rs232_clear2();
			rs232_clear4();
			break;
		}
	}
}

void device_init(void)
{
	device.motor_error_code.car_error=0xff;
	device.motor_error_code.servo_error=0xff;
	device.heartbeat=0;
}

void ResetServoBelt(int speed)
{
	int state =0;
	int init_tick,check;
	init_tick=get_system_tick();
	while(retTickDiff(init_tick)<1000);//等待设备上电完成
	while(1){
		if(state == 0){
			if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
				state++;
				init_tick = get_system_tick();
			}else{
				state = 8;
			}
		}else if(state == 1){
			if(retTickDiff(init_tick)>10){
				if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
					//break;
					state = 104;
				}else{
					state--;
				}
			}
		}else if(state==8){
			rs232_clear4();
			send_servor_motor_data(servo.enable,0x01);
			init_tick=get_system_tick();
			state++;
		}else if(state==9){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart4_buf(),servo.enable,8)){
					state++;
					check=0;
				}else{
					if(check<3){
						check++;
						state--;
					}else{
						check=0;
						state++;
					}
				}
			}
		}else if(state==10){
			rs232_clear4();
			set_mobus_crc(servo.PosSpeed,speed,4);
			init_tick=get_system_tick();
			state++;
		}else if(state==11){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart4_buf(),servo.PosSpeed,8)){
					state++;
					check=0;
				}else{
					if(check<3){
						check++;
						state--;
					}else{
						state++;
						check=0;
					}
				}
			}
		}else if(state == 12){
			if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_RESET)){
				state = 15;
			}if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_RESET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
				state = 13;
			}else{
				state = 15;
			}
		}else if(state == 13){
			rs232_clear4();
			send_servor_motor_pos(servo.SetPos,0-servo.position);
			init_tick = get_system_tick();
			state++;
		}else if(state == 14){
			if(retTickDiff(init_tick)>30){
				if(memcmp(get_rs232_buf4(),servo.SetPos,strlen((char *)servo.SetPos))==0){
					state=100;
					init_tick = get_system_tick();
				}else{
					if(check < 2){
						check++;
						state--;
					}else{
						state=100;
						init_tick = get_system_tick();
						//debug_out1("set servo speed fail\r",strlen("set servo speed fail\r"));
						//break;
					}
				}
			}
		}else if(state == 15){
			rs232_clear4();
			send_servor_motor_pos(servo.SetPos,servo.position);
			init_tick = get_system_tick();
			state++;
		}else if(state == 16){
			if(retTickDiff(init_tick)>30){
				if(memcmp(get_rs232_buf4(),servo.SetPos,strlen((char *)servo.SetPos))==0){
					state=100;
					init_tick = get_system_tick();
				}else{
					if(check < 2){
						check++;
						state--;
					}else{
						state=100;
						init_tick = get_system_tick();
					}
				}
			}
		}else if(state == 100){
			if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
				state++;
			}else if(retTickDiff(init_tick)>15000){
				//state++;
				debug_out1("time out\r",9);
				break;
			}
		}else if(state == 101){
			if(retTickDiff(init_tick)>10){
				if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
					state++;
				}else{
					state--;
				}
			}
		}else if(state == 102){
		rs232_clear4();
	  send_servor_motor_data(servo.stop,0x10);///急停 0x10
		init_tick = get_system_tick();
		state++;
	}else if(state == 103){
		if(retTickDiff(init_tick)>50){
			if(memcmp(get_rs232_buf4(),servo.stop,8)==0){
			//if(memcmp(get_rs232_buf4(),servo.SetPos,strlen((char *)servo.SetPos))==0){
				state++;
				//break;
			}else{
				if(check < 3){
					check++;
					state--;
				}else{
					state++;
					//break;
				}
			}
		}
	}else if(state==104){
			rs232_clear4();
			set_mobus_crc(servo.PosSpeed,servo.speed,4);
			init_tick=get_system_tick();
			state++;
	}else if(state==105){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart4_buf(),servo.PosSpeed,8)){
					
					check=0;
					state++;
				}else{
					if(check<3){
						check++;
						state--;
					}else{
						check=0;
						state++;
					}
				}
			}
		}else if(state==106){
			rs232_clear4();
			send_servor_motor_data(servo.enable,0x01);
			init_tick=get_system_tick();
			state++;
		}else if(state==107){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart4_buf(),servo.enable,8)){
					
					check=0;
					break;
				}else{
					if(check<3){
						check++;
						state--;
					}else{
						check=0;
						break;
					}
				}
			}
		}
	}
}


void my_sprintf_32(char *sbuf,int i,int COM)
{
	char buffer[32];
	memset(buffer,0,sizeof(buffer));
	sprintf(buffer,"%s=%d\r",sbuf,i);
	if(COM == 1){
		uart1_sendstr(buffer,strlen(buffer));
	}else if(COM==2){
		uart2_sendstr(buffer,strlen(buffer));
	}else if(COM==3){
		uart3_sendstr(buffer,strlen(buffer));
	}else if(COM==4){
		uart4_sendstr(buffer,strlen(buffer));
	}
}


int main(void)
{
  __enable_irq();	
	SysTick_Config(SystemCoreClock / 1000);
	rs232_uart_init1(9600);

	#if  SHENGPAI
		rs232_uart_init2(38400);
	#else
			#ifdef BLDC
			rs232_uart_init2(9600);//9600
			#else
			rs232_uart_init2(19200);
			#endif
	#endif
	rs232_uart_init4(19200);
	flash_init();
	flash_read();
	treadmill_init();
	servo_motor_rs485_init();
	ir_init();
	multiInit();
	#if SHENGPAI
		ShengPaiCarInit();
	#else
		Car1init();
	#endif
	ResetServoBelt(300);
	device_init();
	CAN_Mode_Init(BaudRate250Kbps,CAN_Mode_Normal,0,0x0000,0x0000);
	can_recv_buf.Recv_Buf_Len0=0;
 	while(1)
	{
		can_comm();
		ServoCtrl(50);
		CarCtrl();
		goods_export(10);
		StopKey();
		LockKey();
		rs232_process1();
 	}
}

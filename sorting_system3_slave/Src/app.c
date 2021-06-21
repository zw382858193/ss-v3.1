#include "config.h"
#include "device_can.h"
#include "MotorDriver.h"
#include "flash.h"

//#include <stm32f10x.h>

unsigned int alarm_system_tick = 0;
treadmill_def treadmill;
window windows[DEVICESNUM];
#define INIT_TICK 30
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

#if 1
void Car1init(void)
{
	int state = 0,check=0;
	unsigned int tick;
//	uchar *buf;
//	buf=get_uart1_buf();

	#ifdef BLDC
	id_set_mobus_crc(treadmill.SET_485,0,0x00);
	#else
	id_run_ctrl(treadmill.EN_485,0x00);
	id_set_mobus_crc(treadmill.SET_485,5,0x00);
	#endif
	tick = get_system_tick();
	while((get_system_tick()-tick)<2000);//wait for system power up
	while(1){
//		if(get_uart1_length()>0){
//			if (memcmp(buf, "car_tick=", strlen("car_tick="))==0) {
//				int value;
//				value = strtol((char *)&buf[strlen("car_tick=")],NULL,10);
//				treadmill.rs485_tick=value;
//				debug_out1("succ\r",5);
//			}else if(memcmp(buf, "setid", strlen("setid"))==0){
//				char *q;
//				int value;
//				treadmill.SETID[0] = strtol((char *)&buf[strlen("setid")],&q,10);
//				value = strtol((char *)&q[strlen("=")],NULL,10);
//				set_mobus_crc((uchar *)treadmill.SETID,value);
//				debug_out1("succ\r",5);
//			}rs232_clear1();
//		}
		if(state == 0){
			
			rs232_clear2();
			id_run_ctrl((uchar *)treadmill.STOP,CAR1);
			tick = get_system_tick();
			state = 1;

		}	else if(state == 1){
			if((get_system_tick()-tick) > INIT_TICK){
				if(memcmp(get_uart2_buf(),(uchar *)treadmill.STOP,8)==0){
					check=0;
					state = 2;//2
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
			id_set_mobus_crc((uchar *)treadmill.SET_SPEED,treadmill.speed,CAR1);
			tick = get_system_tick();

		}else if(state == 3){

			if ((get_system_tick()-tick) > INIT_TICK) {
				
				if(memcmp(get_uart2_buf(),(uchar *)treadmill.SET_SPEED,8)==0){//
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
			id_set_mobus_crc((uchar *)treadmill.ADTIME_A,treadmill.adtime,CAR1);
			state = 5;
			tick = get_system_tick();

		} else if(state == 5) {
			
			if ((get_system_tick()-tick) > INIT_TICK) {
				if(memcmp(get_uart2_buf(),(uchar *)treadmill.ADTIME_A,8)==0){
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
			id_set_mobus_crc((uchar *)treadmill.ADTIME_D,treadmill.adtime,CAR1);
			state = 7;
			tick = get_system_tick();

		}else if(state == 7){
					
			if((get_system_tick()-tick) > INIT_TICK) {
				if(memcmp(get_uart2_buf(),(uchar *)treadmill.ADTIME_D,8)==0){
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
			treadmill.work_onoff=ON;
			servo.work_status = WAIT;
			treadmill.work_status=WAIT;
			rs232_clear1();
			rs232_clear2();
			rs232_clear4();
			break;
		}
	}
}
#endif

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
	
	while(1){
		my_sprintf_32("a",state,1);
		if(state == 0){
			if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
				state++;
				init_tick = get_system_tick();
			}else{
				state = 10;
			}
		}else if(state == 1){
			if(retTickDiff(init_tick)>10){
				if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==(uint32_t)Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==(uint32_t)Bit_SET)){
					break;
				}else{
					state--;
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
			}else if(retTickDiff(init_tick)>5000){
				//
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
		//send_servor_motor_data(servo.stop,21);
	  send_servor_motor_data(servo.stop,0x10);///¼±Í£ 0x10
	  //send_servor_motor_pos(servo.SetPos,0);
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
  //SCB->VTOR = FLASH_BASE | 0x00000000;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  __enable_irq();	
	SysTick_Config(SystemCoreClock / 1000);
	rs232_uart_init1(9600);
	#ifdef BLDC
 	rs232_uart_init2(9600);//9600
	#else
	rs232_uart_init2(19200);
	#endif
	rs232_uart_init4(19200);
	treadmill_init();
	ir_init();
	Car1init();
	servo_motor_rs485_init();
	multiInit();
	flash_init();
	//flash_read();
	ResetServoBelt(100);
	device_init();
	CAN_Mode_Init(BaudRate250Kbps,CAN_Mode_Normal,0,0x0000,0x0000);
	can_recv_buf.Recv_Buf_Len0=0;
 	while(1)
	{
		can_comm();
		ServoCtrl();
		CarCtrl();
		goods_export(30);
		StopKey();
		LockKey();
		rs232_process1();
 	}
}

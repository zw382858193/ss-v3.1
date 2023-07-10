#include "device_can.h"
#include "MotorDriver.h"
#include "flash.h"
#include "config.h"

//#include <stm32f10x.h>

/*************************
Can协议
8位字节
id 功能码 数据
id 0x01 格口 补零 

功能码
01 win口命令
02 心跳
03 read 驱动器  驱动器地址 0x00 0x00 驱动器数据 0x00 0x00
06 写 驱动器 驱动器地址 0x00 0x00 驱动器数据 0x00 0x00

不足八个字节补零
*************************/

unsigned int alarm_system_tick = 0;
treadmill_def treadmill;
goodsStu goods[DEVICESNUM];
#define INIT_TICK 30
//void ir_init(void)
//{
//	GPIO_InitTypeDef ir_config;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// PA4 5 6 7sensor
//	ir_config.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//	ir_config.GPIO_Mode = GPIO_Mode_IPU;
//	ir_config.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_Init(GPIOA,&ir_config);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE); 
//	ir_config.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
//	ir_config.GPIO_Mode = GPIO_Mode_IPU;
//	ir_config.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_Init(GPIOC,&ir_config);
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
//	ir_config.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
//	ir_config.GPIO_Mode = GPIO_Mode_IPU;
//	ir_config.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_Init(GPIOB,&ir_config);

//}

void ir_init(void)
{
	GPIO_InitTypeDef ir_config;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// PA4 5 6 sensor
	ir_config.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	ir_config.GPIO_Mode = GPIO_Mode_IPU;
	//ir_config.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&ir_config);

}

void pb3_init(void)
{
	GPIO_InitTypeDef ir_config;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// PA4 5 6 7sensor
	ir_config.GPIO_Pin = GPIO_Pin_3;
	ir_config.GPIO_Mode = GPIO_Mode_Out_PP;
	ir_config.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&ir_config);
	GPIO_ResetBits(GPIOB,GPIO_Pin_3);
}

#if 1
void init(void)
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
	while((get_system_tick()-tick)<2000);//等系统上电
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
			id_set_mobus_crc((uchar *)treadmill.ADTIME_A,5,CAR1);
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
			id_set_mobus_crc((uchar *)treadmill.ADTIME_D,5,CAR1);
			state = 7;
			tick = get_system_tick();

		}else if(state == 7){
					
			if((get_system_tick()-tick) > INIT_TICK) {
				if(memcmp(get_uart2_buf(),(uchar *)treadmill.ADTIME_D,8)==0){
					check=0;
					state = 100;//8
				}else{
					if(check<3){
						check++;
						state=6;
					}else{
						check=0;
						state = 100;//8
					}
				}
			}

		}else if(state == 100){
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
	int i;
	for(i=0;i<DEVICESNUM;i++){
		Slave_Device.Slave_Device_Car_ONLINE[i]=0xff;
		Slave_Device.Slave_Device_Servo_ONLINE[i]=0xff;
	}
}




void my_sprintf_32(char *sbuf,int i)
{
	char buffer[32];
	memset(buffer,0,sizeof(buffer));
	sprintf(buffer,"%s=%d\r",sbuf,i);
	uart1_sendstr(buffer,strlen(buffer));
}

int main(void)
{
  //SCB->VTOR = FLASH_BASE | 0x00000000;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  __enable_irq();	
	SysTick_Config(SystemCoreClock / 1000);
	rs232_uart_init1(9600);

	#ifdef BLDC
 	rs232_uart_init2(9600);//9600
	#else
	rs232_uart_init2(19200);
	#endif

	#ifdef LICHUANG
	rs232_uart_init4(19200);
	#else
	rs232_uart_init4(57600);
	#endif
	ir_init();	 //	初始化IO
	flash_init();
	EXTIX_Init();
	GoodStructInit();
	treadmill_init();
	//ir_init();
	pb3_init();
	servo_motor_rs485_init();
	//flash_read();
	device_init();
	init();
	CAN_Mode_Init(BaudRate250Kbps,CAN_Mode_Normal,0,0x0000,0x0000);//波特率太快容易异常

 	while(1) 
	{
		IRCheckFunc();
		uart1RecvFinish(5);
		rs232_process1();
		SlaveDeviceStatus();
		check_car_and_servo(100);
		RunCar1(50);
 	}
}

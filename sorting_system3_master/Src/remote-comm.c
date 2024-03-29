#include "config.h"
#include "MotorDriver.h"
#include "device_can.h"
#include "MessageQue.h"
void Start(void);
void Stop(void);
unsigned short uart2_485_delay;
// 232 communication.

//void servo_motorconnectfailed_start(void);
//void servo_motorconnectfailed_stop(void);
//void treadmill5_cmd(int win);
//void treadmill2_cmd(int win);
//unsigned int delay_break;
//unsigned int tick;
/****************************************
ASCLL码表
a 0x61 j 0x6a s 0x73 0 0x30 9 0x39
b 0x62 k 0x6b t 0x74 1 0x31 del 0x7f
c 0x63 l 0x6c u 0x75 2 0x32
d 0x64 m 0x6d v 0x76 3 0x33
e 0x65 n 0x6e w 0x77 4 0x34
f 0x66 o 0x6f x 0x78 5 0x35
g 0x67 p 0x70 y 0x79 6 0x36
h 0x68 q 0x71 z 0x7a 7 0x37
i 0x69 r 0x72 ~ 0x7e 8 0x38
*****************************************/
Slave_Devices_Stu Slave_Device;
LiChuang servo;
CanCommStu CanComm;
char VERSION[]={"version v0.3.0"};
can_recv_buf_stu can_recv_buf;
can_recv_buf_stu can_send_buf;
char heartbeat[128];
static unsigned int goodsNumber=0;
static unsigned int DeviceStartOrStop=0;
//static unsigned int IRCheck = 0;

void CanCommInit(void)
{
	uchar SetCarSpeed[8] = {0x00,0x08,0x20,0x01,0x00,0x00,0x00,0x00};
	uchar SetCarStatus[8] = {0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00};
	uchar SetServoSpeed[8] = {0x00,0x08,0x01,0x90,0x00,0x00,0x00,0x00};
	uchar SetServoADTime[8] = {0x00,0x08,0x01,0x90,0x00,0x00,0x00,0x00};
	uchar SetServoStatus[8] = {0x00,0x08,0x00,0x01,0x00,0x00,0x00,0x00};
	uchar SetId[8] = {0x00,0x00,0x20,0x08,0x00,0x00,0x00,0x00};
	
	memcpy(CanComm.SetId,SetId,8);
	memcpy(CanComm.SetCarSpeed,SetCarSpeed,8);
	memcpy(CanComm.SetCarStatus,SetCarStatus,8);
	memcpy(CanComm.SetServoSpeed,SetServoSpeed,8);
	memcpy(CanComm.SetServoADTime,SetServoADTime,8);
	memcpy(CanComm.SetServoStatus,SetServoStatus,8);
	
}

void pb3_ctrl(int onoff)//控制流水线方向
{
	if(onoff==1){
		GPIO_SetBits(GPIOB,GPIO_Pin_3);
	}else if(onoff==0){
		GPIO_ResetBits(GPIOB,GPIO_Pin_3);
	}
}

void servo_motor_rs485_init(void)
{
	//#ifdef DAYONE
//		uchar ENABLE[8] = {0x01,0x06,0x00,0x00,0x00,0x01,0x48,0x0a};
//		uchar DISABLE[8] = {0x01,0x06,0x00,0x00,0x00,0x00,0x89,0xca};
//		uchar POS_MODE_PC[8] = {0x01,0x06,0x00,0x02,0x00,0xd0,0x29,0x96};
//		uchar Absolute_position[8] = {0x01,0x06,0x00,0x51,0x00,0x00,0xd8,0x1b};//绝对位置
//		uchar POS_MODE_H16[4] = {0x01,0x06,0x50,0x00};
//		uchar POS_MODE_L16[4] = {0x01,0x06,0x05,0x00};
//		uchar POS_SET[7] = {0x01,0x10,0x00,0x50,0x00,0x02,0x04};
//		uchar POS_SET_RET[8] = {0x01,0x10,0x00,0x50,0x00,0x02,0x41,0xd9};
//		uchar POS_MODE_SPEED[4] = {0x01,0x06,0x00,0x1d};
//		uchar READ_DATA_POS_BACK_H16[8] = {0x01,0x03,0x00,0xe8,0x00,0x01,0x04,0x3e};
//		uchar READ_DATA_POS_BACK_L16[8] = {0x01,0x03,0x00,0xe9,0x00,0x01,0x55,0xfe};
//		uchar READ_DATA_POS_BACK[8] = {0x01,0x03,0x00,0xe8,0x00,0x02,0x44,0x3f};
//		uchar CHECK_I[8] = {0x01,0x03,0x00,0xe2,0x00,0x01,0x24,0x3c};
//		uchar CHECK_STATUS[6] = {0x01,0x03,0x00,0xe3,0x00,0x01};//0xe3
//		uchar POS_MODE_ADTIME[6]={0x01,0x06,0x00,0x09,0x00,0x00};//位置模式的加减速时间
//		uchar Z_POS[6] = {0x01,0x06,0x00,0x53,0x00,0x00};
//		
//		memcpy(cmd.pos_ctrl.ENABLE,ENABLE,8);
//		memcpy(cmd.pos_ctrl.DISABLE,DISABLE,8);
//		memcpy(cmd.pos_ctrl.POS_MODE_PC,POS_MODE_PC,8);
//		memcpy(cmd.pos_ctrl.Absolute_position,Absolute_position,8);
//		memcpy(cmd.pos_ctrl.POS_MODE_H16,POS_MODE_H16,4);
//		memcpy(cmd.pos_ctrl.POS_MODE_L16,POS_MODE_L16,4);
//		memcpy(cmd.pos_ctrl.POS_MODE_SPEED,POS_MODE_SPEED,4);
//		memcpy(cmd.pos_ctrl.READ_DATA_POS_BACK_H16,READ_DATA_POS_BACK_H16,8);
//		memcpy(cmd.pos_ctrl.READ_DATA_POS_BACK_L16,READ_DATA_POS_BACK_L16,8);
//		memcpy(cmd.pos_ctrl.CHECK_I,CHECK_I,8);
//		memcpy(cmd.pos_ctrl.CHECK_STATUS,CHECK_STATUS,6);	
//		memcpy(cmd.pos_ctrl.POS_SET,POS_SET,7);
//		memcpy(cmd.pos_ctrl.POS_SET_RET,POS_SET_RET,8);
//		memcpy(cmd.pos_ctrl.READ_DATA_POS_BACK,READ_DATA_POS_BACK,8);
//		
//		memcpy(cmd.pos_ctrl.POS_MODE_ADTIME,POS_MODE_ADTIME,8);
//		memcpy(cmd.pos_ctrl.Z_POS,Z_POS,8);
	//#endif
	//#ifdef LICHUANG
		uchar SetPos[13] = {0x01,0x10,0x01,0x68,0x00,0x02,0x04,0x86,0xa0,0x00,0x01,0x11,0x1b};//位置指令
		uchar DisplayMotorSpeed[8] = {0x01,0x06,0x00,0x01,0x00,0x01,0x19,0xca};//显示速度
		uchar PosSpeed[8] = {0x01,0x06,0x01,0x90,0x01,0xf4,0x88,0x0c};//第0位置模式速度
		uchar RetSetPos[8] = {0x01,0x10,0x01,0x68,0x00,0x02,0xC1,0xE8};//绝对位置
		uchar PostionSpeedMode[8] = {0x01,0x06,0x00,0x02,0x00,0x03,0x68,0x0b};//位置速度模式切换
		uchar LitMode[8] = {0x01,0x06,0x00,0x04,0x00,0x00,0xc8,0x0b};//限位模式选择
		uchar ATime[8] = {0x01,0x10,0x00,0x50,0x00,0x02,0x04};//加速时间
		uchar DTime[8] = {0x01,0x10,0x00,0x50,0x00,0x02,0x41,0xd9};//减速时间
		uchar CommCtrl[8] = {0x01,0x06,0x00,0x90,0x00,0x01,0x48,0x27};//扩展模式 通讯控制
		uchar PosModeSelect[8] ={0x01,0x06,0x00,0x94,0x00,0x01,0x09,0xe6};//位置模式选择  相对位置模式
//		uchar PosLoadMode[8] ={0x01,0x06,0x00,0x96,0x00,0x02,0xe8,0x27};//位置模式装置模式 上升沿装载
		uchar Load[8] ={0x01,0x06,0x00,0x96,0x00,0x00,0x69,0xe6};//位置模式装载模式 一直装载
		uchar DI0Config[8] ={0x01,0x06,0x00,0x80,0x00,0x00,0x98,0x2c};
		uchar DI1Config[8] ={0x01,0x06,0x00,0x81,0x00,0x01,0x98,0x2c};
		uchar DI4Config[8] ={0x01,0x06,0x00,0x84,0x00,0x04,0xc8,0x20};//DI4控制模式切换
		uchar DI5Config[8] ={0x01,0x06,0x00,0x85,0x00,0x14,0x98,0x2c};//DI5位置装载信号
		uchar DI6Config[8] ={0x01,0x06,0x00,0x86,0x00,0x11,0xa8,0x2f};//DI6原点开关
		uchar DI7Config[8] ={0x01,0x06,0x00,0x87,0x00,0x10,0x38,0x2f};//DI7回零启动
		uchar CommCtrlDI[8]={0x01,0x06,0x01,0xa0,0x00,0xff,0xc8,0x54};//通讯控制DI口
		uchar ServoEnableSelection[8]={0x01,0x06,0x00,0x8f,0x00,0x01,0x79,0xe1};//伺服使能选择 自动使能
		uchar clean[8]={0x01,0x06,0x01,0xa4,0x00,0x02,0x48,0x14};//清除报警
		uchar check[8]={0x01,0x03,0x04,0x78,0x00,0x01,0x05,0x23};//检查设备
		uchar servoEnable[8]={0x01,0x06,0x01,0xa4,0x00,0x01,0x08,0x15};//使能
		uchar servoDisable[8]={0x01,0x06,0x01,0xa4,0x00,0x00,0xc9,0xd5};//失能
		uchar save[8] ={0x01,0x06,0x01,0xa7,0x08,0x01,0xff,0xd5};//保存参数
		
		memcpy(servo.SetPos,SetPos,13);
		memcpy(servo.DisplayMotorSpeed,DisplayMotorSpeed,8);
		memcpy(servo.PosSpeed,PosSpeed,8);
		memcpy(servo.RetSetPos,RetSetPos,8);
		memcpy(servo.PostionSpeedMode,PostionSpeedMode,8);
		memcpy(servo.LitMode,LitMode,8);
		memcpy(servo.ATime,ATime,8);
		memcpy(servo.DTime,DTime,8);
		memcpy(servo.CommCtrl,CommCtrl,8);
		memcpy(servo.PosModeSelect,PosModeSelect,8);
		memcpy(servo.PosLoadMode,Load,8);	
		memcpy(servo.DI0Config,DI0Config,8);
		memcpy(servo.DI1Config,DI1Config,8);
		memcpy(servo.DI4Config,DI4Config,8);
		memcpy(servo.DI5Config,DI5Config,8);
		memcpy(servo.DI6Config,DI6Config,8);
		memcpy(servo.DI7Config,DI7Config,8);
		memcpy(servo.CommCtrlDI,CommCtrlDI,8);
		memcpy(servo.ServoEnableSelection,ServoEnableSelection,8);
		memcpy(servo.clean,clean,8);
		memcpy(servo.check,check,8);
		memcpy(servo.enable,servoEnable,8);
		memcpy(servo.disable,servoDisable,8);
		memcpy(servo.save,save,8);
	//#endif
}

void treadmill_init(void)
{
	uchar SETID[4] = {0x00,0x06,0x20,0x08};
	uchar STOP[6] = {0x01,0x06,0x20,0x00,0x00,0x05};
	uchar FOREWARD[6] = {0x01,0x06,0x20,0x00,0x00,0x01};
	uchar BACKWARD[6] = {0x01,0x06,0x20,0x00,0x00,0x02};
	uchar SET_SPEED[6] = {0x01,0x06,0x20,0x01,0x05,0xDC};
	uchar ADTIME_A[4] = {0x01,0x06,0x20,0x03};
	uchar ADTIME_D[4] = {0x01,0x06,0x20,0x04};
	uchar R_ADTIME_A[6] = {0x01,0x03,0x20,0x03,0x00,0x01};
	uchar R_ADTIME_D[6] = {0x01,0x03,0x20,0x04,0x00,0x01};
	uchar ERR_RESET[6] = {0x01,0x06,0x20,0x00,0x00,0x07};
	uchar BAUDRATE[6] = {0x01,0x06,0x20,0x09,0x00,0x03};
	#ifdef BLDC
		uchar BLDC_CHECK_XIAOCHE_STATE[6] = {0x01,0x03,0x21,0x04,0x00,0x01};
		uchar BLDC_R_SET_SPEED[6] = {0x01,0x03,0x20,0x01,0x00,0x01};
		uchar BLDC_SET_485[6] = {0x00,0x06,0x20,0x0b,0x00,0x05};
		uchar BLDC_DIANJI_JIDUISHU[6] = {0x00,0x06,0x20,0x02,0x00,0x02};
		uchar BLDC_RUN_MODE_CHANNEL[6] = {0x00,0x06,0x20,0x06,0x00,0x02};
		uchar BLDC_RUN_DIR[6] = {0x00,0x06,0x20,0x0A,0x00,0x01};
	#else
		uchar EN_485[6] = {0x01,0x06,0x20,0x0E,0x00,0x01};
		uchar SET_485[6] = {0x01,0x06,0x08,0x03,0x00,0x05};
		uchar CHECK_XIAOCHE_STATE[6] = {0x00,0x03,0x21,0x02,0x00,0x01};
		uchar R_SET_SPEED[8] = {0x01,0x03,0x30,0x05,0x00,0x01,0xB9,0x0B};
		uchar R_UI[6] = {0x01,0x03,0x30,0x07,0x00,0x01};
	#endif
	
	memcpy(treadmill.SETID,SETID,4);
	memcpy(treadmill.STOP,STOP,6);
	memcpy(treadmill.BACKWARD,BACKWARD,6);
	memcpy(treadmill.FOREWARD,FOREWARD,6);
	memcpy(treadmill.SET_SPEED,SET_SPEED,6);
	memcpy(treadmill.ADTIME_A,ADTIME_A,4);
	memcpy(treadmill.ADTIME_D,ADTIME_D,4);
	memcpy(treadmill.R_ADTIME_A,R_ADTIME_A,6);
	memcpy(treadmill.R_ADTIME_D,R_ADTIME_D,6);
	#ifdef BLDC
		memcpy(treadmill.BLDC_RUN_DIR,BLDC_RUN_DIR,6);
		memcpy(treadmill.BLDC_DIANJI_JIDUISHU,BLDC_DIANJI_JIDUISHU,6);
		memcpy(treadmill.BLDC_RUN_MODE_CHANNEL,BLDC_RUN_MODE_CHANNEL,6);
		memcpy(treadmill.CHECK_XIAOCHE_STATE,BLDC_CHECK_XIAOCHE_STATE,6);
		memcpy(treadmill.SET_485,BLDC_SET_485,6);
		memcpy(treadmill.R_SET_SPEED,BLDC_R_SET_SPEED,6);
	#else
		memcpy(treadmill.CHECK_XIAOCHE_STATE,CHECK_XIAOCHE_STATE,6);
		memcpy(treadmill.EN_485,EN_485,6);
		memcpy(treadmill.SET_485,SET_485,6);
		memcpy(treadmill.R_SET_SPEED,R_SET_SPEED,6);
		memcpy(treadmill.R_UI,R_UI,6);
	#endif
	memcpy(treadmill.ERR_RESET,ERR_RESET,6);	
	memcpy(treadmill.BAUDRATE,BAUDRATE,6);
}
//int IRCheckFunc(void)//检查传感器
//{
//	
// if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==(uint32_t)Bit_SET || \
//	 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==(uint32_t)Bit_SET || \
// GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==(uint32_t)Bit_SET){
//	 //IRCheck =1;
//	return 1;
// }
// return 0;
//}

void SlaveDeviceStatus(void)//长时间没有货物就暂停分拣机前进滚筒
{
	static unsigned int tick =0;
	static int firstPower=1;
	//DeviceStartOrStop 0 关机
	//                  1 开机
	//                  ff 休眠
	if(firstPower){
		DeviceStartOrStop = Slave_Device.PowerUp_OnOrOff;
		if(DeviceStartOrStop == 0){//第一次上电停机
			Stop();
		}else if(DeviceStartOrStop == 1){//第一次上电开机
			//IRCheck=0;
			Start();
		}
		firstPower =0;
	}
	if(RetIRCheckFunc() == 1 || DeviceStartOrStop == 0){
		tick = get_system_tick();//刷新休眠时间
	}
	if(Slave_Device.SleepTime > 0 && retTickDiff(tick) > Slave_Device.SleepTime*1000){//超时没货物就休眠
		Stop();
		DeviceStartOrStop = 0xff;
		tick = get_system_tick();
	}
	if(DeviceStartOrStop == 0xff && RetIRCheckFunc() == 1){//如果设备软启动了 休眠了 检查到货物就重启
		Start();
		DeviceStartOrStop=1;
		tick = get_system_tick();
	}
}


void RunCar1(int tickDelay)//如果感应到货物就把货物推出去
{
	static int state =1,CheckNumber=0;
	static unsigned int tick =0;
	static int Car1Dir;
	static int Car1RunTime;
	if(DeviceStartOrStop == 0){
		return;//协议已包含停止命令 这里不再发送直接return
	}
	if(state == 1){
		if(RetIRCheckFunc() == 1){
			ClearIRCheck();
			CheckNumber=0;
			Car1Dir = Slave_Device.Car1DirAndRunTime>>12;
			Car1RunTime = Slave_Device.Car1DirAndRunTime&0xfff;
			tick = get_system_tick();
			state++;//state=3跳过延迟
		}
	}else if(state == 2){
		if(retTickDiff(tick) > Slave_Device.IRCheck_DelayTime){
			state++;
		}
	}else if(state == 3){
		rs232_clear2();
		if(Car1Dir == 0x00)
			id_run_ctrl(treadmill.BACKWARD,0x01);
		else if(Car1Dir == 0x01){
			id_run_ctrl(treadmill.FOREWARD,0x01);
		}
		tick = get_system_tick();
		state++;
	}else if(state == 4){
		if(retTickDiff(tick) > tickDelay){//等待tickDelay ms判断是否发送成功并且有数据返回
			if(Car1Dir == 0x00){
				if(memcmp(get_uart2_buf(),treadmill.BACKWARD,8)==0){
					tick = get_system_tick();
					CheckNumber=0;
					state++;
				}else{
					CheckNumber++;
					if(CheckNumber > 3){
						tick = get_system_tick();
						CheckNumber=0;
						state++;
					}else
						state--;
				}
			}else if(Car1Dir == 0x01){
				if(memcmp(get_uart2_buf(),treadmill.FOREWARD,8)==0){
					tick = get_system_tick();
					CheckNumber=0;
					state++;
				}else{
					CheckNumber++;
					if(CheckNumber > 3){
						tick = get_system_tick();
						CheckNumber=0;
						state++;
					}else
						state--;
				}
			}else{
				state = 1;
			}
		}
	}else if(state == 5){
		if(retTickDiff(tick) > Car1RunTime){//运转时间
			state++;
		}
	}else if(state == 6){
		rs232_clear2();
		id_run_ctrl(treadmill.STOP,0x01);
		tick = get_system_tick();
		state++;
	}else if(state == 7){
		if(retTickDiff(tick) > tickDelay){//等待tickDelay ms判断是否发送成功并且有数据返回
			if(memcmp(get_uart2_buf(),treadmill.STOP,8)==0){
				CheckNumber=0;
				state++;
				//state=1;
			}else{
				CheckNumber++;
				if(CheckNumber > 3){
					tick = get_system_tick();
					CheckNumber=0;
					state++;
					//state=1;
				}else
					state--;
			}
		}
	}
	else if(state == 8){
		ClearIRCheck();
		if(RetIRCheckFunc()==0){//货物送走后等待下一次货物 
			state=1;
		}
	}
}

void Start(void)//设备启动
{
	int ret;
	u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned int delay_tick=get_system_tick();
	id_set_mobus_crc((uchar *)treadmill.SET_SPEED,treadmill.speed,CAR1);//设置单台小车速度
	CanSendMsg[0]=0x00;//广播
	CanSendMsg[1]=0x08;//
	CanSendMsg[5]=0x00;
	ret=Can_Send_Msg(0,CanSendMsg,8);
	if(ret == 1){
		//debug_out1("Start send failed\r",strlen("send failed\r"));
	}
	GoodStructInit();
	goodsNumber=1;
	pb3_ctrl(1);
	ClearIRCheck();
}
void Stop(void)//设备停止
{
	int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[5]=0x05;//stop
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				//debug_out1("Stop send failed\r",strlen("send failed\r"));
			}
			GoodStructInit();
			goodsNumber=0;
			id_run_ctrl(treadmill.STOP,0x01);
			pb3_ctrl(0);
			ClearIRCheck();
}
void initServoLiChuang(void)
{
	int check=0,state=0;
	unsigned int init_tick=0;
	while(1){
		if(state==0){
			rs232_clear2();
			debug_out2((char *)servo.PostionSpeedMode,8);
			init_tick=get_system_tick();
			state=1;
		}else if(state==1){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.PostionSpeedMode,8)){
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
		}else if(state==2){
			rs232_clear2();
			debug_out2((char *)servo.LitMode,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==3){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.LitMode,8)){
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
		}else if(state==4){
			rs232_clear2();
			set_mobus_crc(servo.ATime,servo.adtime);
			init_tick=get_system_tick();
			state++;
		}else if(state==5){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.ATime,8)){
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
		}else if(state==6){
			rs232_clear2();
			set_mobus_crc(servo.DTime,servo.adtime);
			init_tick=get_system_tick();
			state++;
		}else if(state==7){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.DTime,8)){
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
		}else if(state==8){
			rs232_clear2();
			debug_out2((char *)servo.CommCtrl,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==9){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.CommCtrl,8)){
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
		}else if(state==10){
			rs232_clear2();
			debug_out2((char *)servo.PosModeSelect,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==11){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.PosModeSelect,8)){
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
		}else if(state==12){
			rs232_clear2();
			debug_out2((char *)servo.PosLoadMode,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==13){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.PosLoadMode,8)){
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
		}else if(state==14){
			rs232_clear2();
			debug_out2((char *)servo.DI4Config,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==15){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.DI4Config,8)){
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
		}else if(state==16){
			rs232_clear2();
			debug_out2((char *)servo.DI5Config,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==17){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.DI5Config,8)){
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
		}else if(state==18){
			rs232_clear2();
			debug_out2((char *)servo.DI6Config,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==19){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.DI6Config,8)){
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
		}else if(state==20){
			rs232_clear2();
			debug_out2((char *)servo.DI7Config,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==21){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.DI7Config,8)){
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
		}else if(state==22){
			rs232_clear2();
			debug_out2((char *)servo.ServoEnableSelection,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==23){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.ServoEnableSelection,8)){
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
		}else if(state==24){
			rs232_clear2();
			debug_out2((char *)servo.CommCtrlDI,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==25){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.CommCtrlDI,8)){
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
		}else if(state==26){
			rs232_clear2();
			debug_out2((char *)servo.PosSpeed,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==27){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.PosSpeed,8)){
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
		}else if(state==28){
			rs232_clear2();
			send_servor_motor_pos(servo.SetPos,0);
			init_tick=get_system_tick();
			state++;
		}else if(state==29){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.RetSetPos,8)){
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
		}else if(state==30){
			rs232_clear2();
			debug_out2((char *)servo.save,8);
			init_tick=get_system_tick();
			state++;
		}else if(state==31){
			if(retTickDiff(init_tick)>50){
				if(memcmp(get_uart2_buf(),servo.save,8)){
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
		}else if(state==32){
			break;
		}
	}
}



void GoodsStructClean(int i)
{
	goods[i].Win=0;
	goods[i].DeviceLockStatus=0;
	goods[i].goodspoition=0;
	memset(goods[i].info,0,sizeof(goods[i].info));
}
void GoodStructInit(void)
{
	int i=0;
	for(i=0;i<MaxGoodsNumber;i++){
		GoodsStructClean(i);
	}
}

void check_car_and_servo(unsigned int tick_delay)
{
	static int state = 0;
	static unsigned int DelayTick;
	static int id=2,abnormal=0;
	int j;
	static int i;
	static u8 CanSendMsg[8]={0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00};
	//int ret;
	u8 normal[8] = {0x02,0x00,0x00,0x04,0x00,0x00,0x00,0x00};//[0]heartbeat [1]maxid [2]errorNumber [3]ByteLen
	//my_sprintf_32("state",state);
	if(WorkStatus!=0x00){//大于0表示急停 表示第几台设备发送急停命令
		normal[1] = Slave_Device.maxId;
		normal[2] = 0x01;//1台设备异常
		normal[3] = 0x04;//后面字节长度
		normal[4] = WorkStatus;//第几台发送的急停命令
		normal[5] = 0x01;//stop 02 03 01 04 WorkStatus 01 00 00
		memset(heartbeat,0,sizeof(heartbeat));
		memcpy(heartbeat,normal,8);
		for(i=0;i<MaxGoodsNumber;i++)
		{
			GoodsStructClean(i);
		}
		goodsNumber=0;
		DelayTick = get_system_tick();
		while((get_system_tick()-DelayTick)<1000);
		state=0;
		abnormal=0;
		Slave_Device.status=1;

	}else{
		if(state == 0){
			CanSendMsg[0]=id;
			CanSendMsg[1]=0x02;
			can_recv_buf.Recv_Buf_Len0=0;
			memset(can_recv_buf.CAN_Recv_Buf0,0,CAN_RECV_BUF_LEN);
			Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			//if(ret==1)
			//debug_out1("heartbeat\r",strlen("heartbeat\r"));
			DelayTick = get_system_tick();
			state = 1;
		}else if(state == 1){
			if((get_system_tick()-DelayTick)>tick_delay){
				
				//if(can_recv_buf.Recv_Buf_Len0>=CAN_RECV_BUF_LEN){
				if(Adv_Data_Get(can_recv_buf.CAN_Recv_Buf0)==1){
					
					if(can_recv_buf.CAN_Recv_Buf0[1]==0x02&&can_recv_buf.CAN_Recv_Buf0[0]==id){
						Slave_Device.Slave_Device_ONLINE[id]=can_recv_buf.CAN_Recv_Buf0[5];//device online
						Slave_Device.Slave_Device_Car_ONLINE[id]=can_recv_buf.CAN_Recv_Buf0[3];
						Slave_Device.Slave_Device_Servo_ONLINE[id]=can_recv_buf.CAN_Recv_Buf0[4];
						//debug_out1((char *)can_recv_buf.CAN_Recv_Buf0,8);
						if((can_recv_buf.CAN_Recv_Buf0[3]!=0x00) || ((can_recv_buf.CAN_Recv_Buf0[4]&0x02) == 0x02)){
							Slave_Device.status=0;
							abnormal++;
						}
					}
				}else{//设备掉线
					Slave_Device.Slave_Device_ONLINE[id]=0xff;//device off
					Slave_Device.Slave_Device_Car_ONLINE[id]=0xff;
					Slave_Device.Slave_Device_Servo_ONLINE[id]=0xff;
					Slave_Device.status=0;
					abnormal++;
					//debug_out1((char *)can_recv_buf.CAN_Recv_Buf0,can_recv_buf.Recv_Buf_Len0);
				}state++;
			}
		}else if(state == 2){
			if(id > (Slave_Device.maxId)){//id MAX
				state++;
				id=2;
			}else{
				id++;
				state=0;
			}
			if(goodsNumber>0){
				can_recv_buf.Recv_Buf_Len0=0;
				memset(can_recv_buf.CAN_Recv_Buf0,0,sizeof(can_recv_buf.CAN_Recv_Buf0));
				Adv_Data_Clear();
				id=2;
				state=100;
			}
		}else if(state == 3){
			memset(heartbeat,0,sizeof(heartbeat));
			if(Slave_Device.status==0){//如果有异常
				heartbeat[0] = 2;
				heartbeat[1] = Slave_Device.maxId;
				heartbeat[2] = abnormal;
				heartbeat[3] = heartbeat[2]*4;
				j=2;
				for(i=2;i<(Slave_Device.maxId+2);i++){
					if((Slave_Device.Slave_Device_ONLINE[i]!=0x00) || (Slave_Device.Slave_Device_Car_ONLINE[i]!=0) || ((Slave_Device.Slave_Device_Servo_ONLINE[i]&0x02)==0x02)){
						heartbeat[(j-1)*4] = i;
						heartbeat[((j-1)*4)+1] = Slave_Device.Slave_Device_ONLINE[i];
						heartbeat[((j-1)*4)+2] = Slave_Device.Slave_Device_Car_ONLINE[i];
						heartbeat[((j-1)*4)+3] = Slave_Device.Slave_Device_Servo_ONLINE[i];
						j++;
					}
				}
				//debug_out1((char *)heartbeat,(abnormal+1)*4);
			}else{
				normal[1] = Slave_Device.maxId;
				memcpy(heartbeat,normal,8);
				//debug_out1((char *)normal,8);
			}
			state++;
			DelayTick = get_system_tick();
		}else if(state == 4){
			if(retTickDiff(DelayTick)>5000){//每五秒自检
				state=0;
				abnormal=0;
				Slave_Device.status=1;
			}
			if(goodsNumber>0){//如果有货物
				abnormal=0;
				Slave_Device.status=1;
				can_recv_buf.Recv_Buf_Len0=0;
				memset(can_recv_buf.CAN_Recv_Buf0,0,sizeof(can_recv_buf.CAN_Recv_Buf0));
				state=100;
			}
			Adv_Data_Clear();
		}else if(state==100){//分拣状态
//			if(can_recv_buf.Recv_Buf_Len0>0){
//				state++;
//			}else if(can_recv_buf.Recv_Buf_Len1>0){
//				len1test++;
//			}else if(goodsNumber==0){
//				state=0;
//			}
			if(Adv_Data_Get(can_recv_buf.CAN_Recv_Buf0)==1){
				//debug_out1((char *)can_recv_buf.CAN_Recv_Buf0,6);
				state++;
			}else if(goodsNumber==0){
				state=0;
			}else{
				if(i<MaxGoodsNumber){
					if(goods[i].Win>0){//如果分配格口
						if(goods[i].goodspoition>1){//货物已到分拣小车上
							if(retTickDiff(goods[i].TimeOutTick)>Slave_Device.GoodsTimeOut){//货物超时丢失
								state=103;
							}else{
								i++;
							}
						}else if(goods[i].goodspoition==1){
							if(retTickDiff(goods[i].TimeOutTick)>Slave_Device.firstPostionTimeOut){//判断货物是到达第一台小车 超时丢失
								state=103;
							}else{
								i++;
							}
						}
					}else{
						i++;
					}
				}else{
					i=0;
				}
			}
		}else if(state==101){//id cmdMode byte lockStatus
			if(can_recv_buf.CAN_Recv_Buf0[1]==0x01){//win cmd
				memset(CanSendMsg,0,sizeof(CanSendMsg));
				for(i=0;i<MaxGoodsNumber;i++){
					if(goods[i].goodspoition==can_recv_buf.CAN_Recv_Buf0[0]-1)
					{
						goods[i].goodspoition=can_recv_buf.CAN_Recv_Buf0[0];
						goods[i].DeviceLockStatus=can_recv_buf.CAN_Recv_Buf0[5];
						goods[i].TimeOutTick = get_system_tick();
						if(((goods[i].goodspoition-1)*2)==goods[i].Win){//是否到达指定格口
							if((goods[i].DeviceLockStatus & 0x10)==0x10){//是否锁格
								state=104;
								break;
							}else{
								state++;
								DelayTick=get_system_tick();
								CanSendMsg[0]=goods[i].goodspoition;
								CanSendMsg[1]=0x08;
								CanSendMsg[2]=0x00;
								CanSendMsg[3]=0x01;
								CanSendMsg[5]=0x00;
								break;
							}
						}
						else if(((goods[i].goodspoition-1)*2)==(goods[i].Win+1)){
							if((goods[i].DeviceLockStatus & 0x20)==0x20){
								state=104;
								break;
							}else{
								state++;
								DelayTick=get_system_tick();
								CanSendMsg[0]=goods[i].goodspoition;
								CanSendMsg[1]=0x08;
								CanSendMsg[2]=0x00;
								CanSendMsg[3]=0x01;
								CanSendMsg[5]=0x01;
								break;
							}
						}
						if(goods[i].goodspoition==(Slave_Device.maxId+1)){
							if(goods[i].Win>(Slave_Device.maxId*2)){//是否超越设备最大格口
								state=104;
								break;
							}
						}
//						can_recv_buf.Recv_Buf_Len0=0;
//						memset(can_recv_buf.CAN_Recv_Buf0,0,sizeof(can_recv_buf.CAN_Recv_Buf0));
						state--;
						break;
					}
				}
				if(i==MaxGoodsNumber){
					state=100;
				}
				if(can_recv_buf.CAN_Recv_Buf0[3]!=0x00 || (can_recv_buf.CAN_Recv_Buf0[4]&0x02)==0x02 || can_recv_buf.CAN_Recv_Buf0[5]!=0x00){
					heartbeat[1] = Slave_Device.maxId;
					heartbeat[2] = 0x01;//errorNumber
					heartbeat[3] = 0x04;//byte
					heartbeat[4] =  can_recv_buf.CAN_Recv_Buf0[0];//SlaveID
					heartbeat[5] =  can_recv_buf.CAN_Recv_Buf0[5];//Status 0x10 0x20锁格口 0x01~0x0f传感器
					heartbeat[6] =  can_recv_buf.CAN_Recv_Buf0[3];//CarError
					heartbeat[7] =  can_recv_buf.CAN_Recv_Buf0[4];//ServoError
				}else{
					
				}
				can_recv_buf.Recv_Buf_Len0=0;
				memset(can_recv_buf.CAN_Recv_Buf0,0,sizeof(can_recv_buf.CAN_Recv_Buf0));
			}
		}else if(state == 102){
			if(retTickDiff(DelayTick) > Slave_Device.WinDelayTime){
				//memset(can_recv_buf.CAN_Recv_Buf1,0,sizeof(can_recv_buf.CAN_Recv_Buf1));
				//can_recv_buf.Recv_Buf_Len1=0;
				Can_Send_Msg(Slave_Device.id,CanSendMsg,8);//发送分拣命令
				//state++;
				state=104;
				DelayTick=get_system_tick();
			}
		}else if(state == 103){
			//debug_out1("endTimeout",10);
			state++;
		}else if(state == 104){
			
			//debug_out1(goods[i].info,goods[i].infoLen);//分拣完成上报
			GoodsStructClean(i);
			if(goodsNumber>1){
				goodsNumber--;//分拣完成在分货物数减一
			}
			state=100;
		}
	}
}



void rs232_process1(void)//UART1
{
	unsigned int rx1_len;
	unsigned char *buf;
	int value;

	rx1_len = get_rs232_len1();

	if (rx1_len > 0) {
		buf = get_rs232_buf1();
		if (memcmp(buf, "win", strlen("win"))==0) {//77 69 6E 00 00 00 01 7F 31 32 33 7F 34 35 36 7E 0D
			int value;
			//int BillCodeAndPortCodeLen=3;
			//char BillCodeAndPortCode[64]={"end"};
			
			//char *p;
			//value=strtol((char *)&buf[strlen("win")],&p,10);
			value = buf[3]<<24|buf[4]<<16|buf[5]<<8|buf[6];
//			while(buf[BillCodeAndPortCodeLen+4]!=0x7e){
//				BillCodeAndPortCode[BillCodeAndPortCodeLen] = buf[BillCodeAndPortCodeLen+4];
//				BillCodeAndPortCodeLen++;
//			}
//			BillCodeAndPortCode[BillCodeAndPortCodeLen] = 0x7e;
			if(value>0 && goodsNumber>0){
				int i=0;
//				u8 CanSendMsg[64]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//				
//				//for(i=0;i<Slave_Device.goodsnumber;i++){
//					//if(windows[i].Win==0){
//						//can send
//						windows[i].Win=((value+1)/2)+1;//接收命令的设备ID号
//						CanSendMsg[0]=windows[i].Win;//接收命令的设备ID号
//						CanSendMsg[1]=0x01;//win 口命令
//						//CanSendMsg[2]=value;//格口
//						CanSendMsg[2]=buf[3];
//						CanSendMsg[3]=buf[4];
//						CanSendMsg[4]=buf[5];
//						CanSendMsg[5]=buf[6];
//						for(i=0;i<BillCodeAndPortCodeLen;i++){
//							CanSendMsg[i+6]=BillCodeAndPortCode[i];
//						}
//						if(can_send_buf.Recv_Buf_Len0==0){
//							memcpy(can_send_buf.CAN_Recv_Buf0,CanSendMsg,sizeof(CanSendMsg));
//							can_send_buf.Recv_Buf_Len0=BillCodeAndPortCodeLen+6;
//						}else if(can_send_buf.Recv_Buf_Len1==0){
//							memcpy(can_send_buf.CAN_Recv_Buf1,CanSendMsg,sizeof(CanSendMsg));
//							can_send_buf.Recv_Buf_Len1=BillCodeAndPortCodeLen+6;
//						}

						
						for(i=0;i<MaxGoodsNumber;i++){
							if(goods[i].Win==0){
								goods[i].Win=value;
								goods[i].goodspoition=1;
								goods[i].infoLen=3;
								goods[i].TimeOutTick=get_system_tick();
//								goods[i].info[0]='e';
//								goods[i].info[1]='n';
//								goods[i].info[2]='d';
//								if(rx1_len>5){
//									while(buf[goods[i].infoLen+4]!=0x7e){
//										goods[i].info[goods[i].infoLen++] = buf[goods[i].infoLen+4];
//									}
//									goods[i].info[goods[i].infoLen] = 0x7e;
//								}
								break;
							}
						}
						goodsNumber++;//在分货物数加一
						debug_out1("ok",2);
			}
		}else if (memcmp(buf, "addr", strlen("addr"))==0) {
			int value;
			char *q;
			uchar str[8]={0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00};
			value = strtol((char *)&buf[strlen("addr")],&q,10);
			set_addr_mobus_crc(str,value,strtol((&q[strlen("data")]),NULL,10));
			debug_out1("ok\r",3);
		}else if (memcmp(buf, "device_id", strlen("device_id"))==0) {
			char *q;
			Slave_Device.id = strtol((char *)&buf[strlen("device_id=")],&q,10);
			WriteFlashOneWord(FLASH_DEVICE_ID_ADDR,Slave_Device.id);
			debug_out1("ok\r",3);
		}else if (memcmp(buf, "start", strlen("start"))==0) {
			debug_out2((char *)servo.enable,8);
			debug_out1("ok\r",3);
		}else if (memcmp(buf, "stop", strlen("stop"))==0) {
			debug_out2((char *)servo.disable,8);
			debug_out1("ok\r",3);
		}else if (memcmp(buf, "pos_set=", strlen("pos_set="))==0) {
			char *q;
			servo.position = strtol((char *)&buf[strlen("pos_set=")],&q,10);
			debug_out1("ok\r",3);
		}else if (memcmp(buf, "pos_speed=", strlen("pos_speed="))==0){
			servo.speed = strtol((char *)&buf[strlen("pos_speed=")],NULL,10);
			
			send_servor_motor_data(servo.PosSpeed,value);
			debug_out1("ok\r",3);
		}else if (memcmp(buf, "pos_adtime=", strlen("pos_adtime="))==0){
			servo.adtime = strtol((char *)&buf[strlen("pos_adtime=")],NULL,10);
			set_mobus_crc(servo.ATime,servo.adtime);
			set_mobus_crc(servo.DTime,servo.adtime);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "setid", strlen("setid"))==0){
			char *q;
			treadmill.SETID[0] = strtol((char *)&buf[strlen("setid")],&q,10);
			value = strtol((char *)&q[strlen("=")],NULL,10);
			set_mobus_crc((uchar *)treadmill.SETID,value);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "car_speed=", strlen("car_speed="))==0){
			value = strtol((char *)&buf[strlen("car_speed=")],NULL,10);
			treadmill.speed = value;
			id_set_mobus_crc((uchar *)treadmill.SET_SPEED,value,CAR1);
			WriteFlashOneWord(FLASH_CAR_SPEED_DATA_ADDR,treadmill.speed);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qmaxid", strlen("qmaxid"))==0){//71 6d 61 78 69 64
			Slave_Device.maxId=buf[6];
			WriteFlashOneWord(FLASH_DEVICE_MAX_ID_ADDR,Slave_Device.maxId);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "version", strlen("version"))==0){
			debug_out1(VERSION,strlen(VERSION));
		}else if(memcmp(buf, "read_flash", strlen("read_flash"))==0){
			flash_read();
		}else if(memcmp(buf, "car_tick=",strlen("car_tick="))==0){
			uart2_485_delay = strtol((char *)&buf[strlen("car_tick=")],NULL,10);
			WriteFlashOneWord(FLASH_CAR_RS485_TICK_ADDR,uart2_485_delay);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcheck",6)==0){//heartbeat
			heartbeat[0] = DeviceStartOrStop;
			debug_out1(heartbeat,heartbeat[3]+4);
		}else if(memcmp(buf, "qsetid",6)==0){//set id 71 73 65 74 69 64 00 03 0d
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=buf[6];//广播
			CanSendMsg[1]=0x08;
			CanSendMsg[2]=0x20;
			CanSendMsg[3]=0x08;
			CanSendMsg[5]=buf[7];
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			//debug_out1((char *)CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qinit", strlen("qinit"))==0){//set init
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=0x00;//buf[5];//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[2]='i';
			CanSendMsg[3]='n';
			CanSendMsg[4]='i';
			CanSendMsg[5]='t';
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qsew1", strlen("qsew1"))==0){//set servi win 1 //00 08 00 01 00 01 00 00
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//device.id 08 00 01 00 01 00 00
			CanSendMsg[0]=0x00;//buf[5];//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[3]=0x01;
			CanSendMsg[5]=0x00;
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qsew2", strlen("qsew2"))==0){//set servo win 2
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=0x00;//buf[5];//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[3]=0x01;
			CanSendMsg[5]=0x01;
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qsestop", strlen("qsestop"))==0){//set servo stop
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[2]=0x00;
			CanSendMsg[3]=0x01;
			CanSendMsg[5]=0x05;
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qsestar", strlen("qsestar"))==0){//set servo start
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[2]=0x00;
			CanSendMsg[3]=0x01;
			CanSendMsg[5]=0x03;
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qsead", strlen("qsead"))==0){//set servo adtime
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=buf[5];//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[3]=0x50;
			CanSendMsg[5]=buf[6];
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qsesp", strlen("qsesp"))==0){//set servo speed 71 73 65 73 70
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			//value = strtol((char *)&buf[strlen("qsesp")],NULL,10);
			value = (buf[5]<<8)|buf[6];
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;
			CanSendMsg[2]=0x01;
			CanSendMsg[3]=0x90;
			CanSendMsg[4]=(value>>8)&0xff;
			CanSendMsg[5]=value&0xff;
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			servo.speed = value;
			WriteFlashOneWord(FLASH_SERVO_MOTOR_SPEED_ADDR,servo.speed);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "pos", strlen("pos"))==0){//set servo pos
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			value = strtol((char *)&buf[strlen("pos")],NULL,10);
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;
			CanSendMsg[2]=0x01;
			CanSendMsg[3]=0x68;
			CanSendMsg[4]=(value>>24)&0xff;
			CanSendMsg[5]=(value>>16)&0xff;
			CanSendMsg[6]=(value>>8)&0xff;
			CanSendMsg[7]=value&0xff;
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);	
			}
		}else if(memcmp(buf, "qscsp",5)==0){//set scan car speed qscsp 71 73 63 73 70 03 e8 1000RPM 皮带机
			//value = strtol((char *)&buf[strlen("qscsp")],NULL,10);
			value = (buf[5]<<8)|buf[6];
			treadmill.speed = value;
			id_set_mobus_crc((uchar *)treadmill.SET_SPEED,value,CAR1);
			WriteFlashOneWord(FLASH_CAR_SPEED_DATA_ADDR,treadmill.speed);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcsp",4)==0){//set car speed 71 63 73 70 01 2C 200RPM 前行滚筒
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			//value = strtol((char *)&buf[strlen("qcsp")],NULL,10);
			value = (buf[4]<<8)|buf[5];
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;
			CanSendMsg[2]=0x20;
			CanSendMsg[3]=0x01;
			CanSendMsg[4]=(value>>8)&0xff;
			CanSendMsg[5]=value&0xff;
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qinit",5)==0){//init servo
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;
			CanSendMsg[2]='i';
			CanSendMsg[3]='n';
			CanSendMsg[4]='i';
			CanSendMsg[5]='t';
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
		}else if(memcmp(buf, "qseton", strlen("qson"))==0){//设置上电默认启动
			WriteFlashOneWord(FLASH_DEFAULT_ON_OFF_DATA_ADDR,1);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qsetoff", strlen("qsetoff"))==0){//设置上电默认停止
			WriteFlashOneWord(FLASH_DEFAULT_ON_OFF_DATA_ADDR,0);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qirdt", strlen("qirdt"))==0){//q ir delaytime设置小车感应传感器后等待时间 qirdt 71 69 72 64 74 03 E8 1000ms
			Slave_Device.IRCheck_DelayTime = (buf[5]<<8)|buf[6];
			WriteFlashOneWord(FLASH_IRCHECKDELAYTIME_ADDR,Slave_Device.IRCheck_DelayTime);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qwidt", strlen("qwidt"))==0){//q ir delaytime设置分拣车感应传感器后等待时间 qwidt 71 77 69 64 74 03 E8 1000ms
			Slave_Device.WinDelayTime = (buf[5]<<8)|buf[6];
			WriteFlashOneWord(FLASH_WINDELAYTIME_ADDR,Slave_Device.WinDelayTime);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcarf", strlen("qcarf"))==0){//启动
//			int ret;
//			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//			unsigned int delay_tick=get_system_tick();
//			id_set_mobus_crc((uchar *)treadmill.SET_SPEED,treadmill.speed,CAR1);//设置单台小车速度
//			CanSendMsg[0]=0x00;//广播
//			CanSendMsg[1]=0x08;//
//			CanSendMsg[5]=0x00;
//			ret=Can_Send_Msg(0,CanSendMsg,8);
//			if(ret == 1){
//				debug_out1("send failed\r",strlen("send failed\r"));
//			}else{
//				debug_out1("ok\r",3);
//			}
//			GoodStructInit();
//			goodsNumber=1;
			//while(retTickDiff(delay_tick)<500);
			//id_run_ctrl(treadmill.FOREWARD,0x01);//看实际情况确认是否要启动第一台小车CAR1
			Start();
			//IRCheck=0;
			DeviceStartOrStop=1;
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcarb", strlen("qcarb"))==0){
			int ret;
			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			CanSendMsg[0]=0x00;//广播
			CanSendMsg[1]=0x08;//
			CanSendMsg[5]=0x01;//backfard
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
			//while(retTickDiff(delay_tick)<500);
			//id_run_ctrl(treadmill.BACKWARD,0x01);//看实际情况确认是否要启动第一台小车CAR1
			GoodStructInit();
			goodsNumber=1;
			DeviceStartOrStop=2;
			pb3_ctrl(1);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcars", strlen("qcars"))==0){//停止
//			int ret;
//			u8 CanSendMsg[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//			CanSendMsg[0]=0x00;//广播
//			CanSendMsg[1]=0x08;//
//			CanSendMsg[5]=0x05;//stop
//			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
//			if(ret == 1){
//				debug_out1("send failed\r",strlen("send failed\r"));
//			}else{
//				debug_out1("ok\r",3);
//			}
//			id_run_ctrl(treadmill.STOP,0x01);
//			GoodStructInit();
//			goodsNumber=0;
			Stop();
			DeviceStartOrStop = 0;
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcar2f", strlen("qcar2f"))==0){//控制流水线启动
			pb3_ctrl(1);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcar2s", strlen("qcar2s"))==0){//控制流水线停止
			pb3_ctrl(0);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qcar1", strlen("qcar1"))==0){//设置CAR1的运转时间和方向 第13位是方向 低12位是时间 11F4 ==> 1(反转 0 正转) 1f4(500) ,71 63 61 72 31 11 f4
			Slave_Device.Car1DirAndRunTime = buf[5] << 8 | buf[6];
			WriteFlashOneWord(FLASH_CAR1DIRANDRUNTIME_ADDR,Slave_Device.Car1DirAndRunTime);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qslp", strlen("qslp"))==0){//设休眠时间 0是不休眠 71 73 6c 70 00 00 00 0d 秒为单位
			Slave_Device.SleepTime = buf[5] << 8 | buf[6];
			WriteFlashOneWord(FLASH_SLEEPTIME_ADDR,Slave_Device.SleepTime);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "set_id", strlen("set_id"))==0){
			int value;
			char *q;
			value =  strtol((char *)&buf[strlen("set_id")],&q,10);
			treadmill.SETID[0] = value;
			set_mobus_crc((uchar *)treadmill.SETID,strtol((char *)&q[strlen("=")],NULL,10));
			//set_mobus_crc((uchar *)treadmill.SETID,strtol((char *)&q[strlen("=")],NULL,10),value);
		}else if(memcmp(buf, "clean", strlen("clean"))==0){

			debug_out2((char *)servo.clean,8);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "qtime", strlen("qtime"))==0){//71 74 69 6d 65
			Slave_Device.GoodsTimeOut=(buf[5]<<8)|buf[6];			
			WriteFlashOneWord(FLASH_DEVICE_TIMEOUT_ADDR,Slave_Device.GoodsTimeOut);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "ftime", strlen("ftime"))==0){//66 74 69 6d 65 13 88 5000MS
			Slave_Device.firstPostionTimeOut=(buf[5]<<8)|buf[6];			
			WriteFlashOneWord(FLASH_FIRSTPOSTIONTIMEOUT_ADDR,Slave_Device.firstPostionTimeOut);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "tcarf", strlen("tcarf"))==0){

			id_run_ctrl(treadmill.FOREWARD,0x01);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "tcarb", strlen("tcarb"))==0){

			id_run_ctrl(treadmill.BACKWARD,0x01);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "tcars", strlen("tcars"))==0){

			id_run_ctrl(treadmill.STOP,0x01);
			debug_out1("ok\r",3);
		}else if(memcmp(buf, "initServo", strlen("initServo"))==0){
			initServoLiChuang();
		}else if(memcmp(buf, "reset", strlen("reset"))==0){
			int ret;
			u8 CanSendMsg[8]={0x00,0x08,'r','e','s','e','t',0x00};
			ret=Can_Send_Msg(Slave_Device.id,CanSendMsg,8);
			if(ret == 1){
				debug_out1("send failed\r",strlen("send failed\r"));
			}else{
				debug_out1("ok\r",3);
			}
			reset();
		}else {
			//debug_out1("unknow cmd",strlen("unknow cmd"));
		}
		rs232_clear1();
	}
}





#include "flash.h"

#ifndef _UCHAR_
#define _UCHAR_
#define uchar unsigned char
#endif
#define BLDC
#define LICHUANG
#define DEVICESNUM 128
#define MAX_POS_SET 300000
#define MIN_POS_SET -300000

typedef struct {
	uchar SETID[8];
	uchar STOP[8];
	uchar BACKWARD[8];
	uchar FOREWARD[8];
	uchar SET_SPEED[8];
	uchar ADTIME_A[8];
	uchar ADTIME_D[8];
	uchar R_ADTIME_A[8];
	uchar R_ADTIME_D[8];
	uchar CHECK_XIAOCHE_STATE[8];
	uchar ERR_RESET[8];
	uchar R_SET_SPEED[8];
	uchar R_UI[8];
	uchar BAUDRATE[8];
	uchar SET_485[8];
	uchar EN_485[8];
	uchar BLDC_RUN_MODE_CHANNEL[8];
	uchar BLDC_DIANJI_JIDUISHU[8];
	uchar BLDC_RUN_DIR[8];
	uchar FIRST_XIAOCHE_FOREWARD[8];
	uchar FIRST_XIAOCHE_BACKWARD[8];
	int RUN_TICK;//单个设备运行时间
	int first_deltime;//第一个设备运行减少时间
	int next_state_flag;//退出当前运行函数或者跳过当前小车
	unsigned short first_xiaoche_dir;//改变第一个小车的运行方向
	int speed;
	unsigned int loop_i;
	unsigned short run_mode;//函数运行模式改变
	int work_status;
}treadmill_def;
extern treadmill_def treadmill;

typedef struct {
	uchar PostionSpeedMode[8];
	uchar LitMode[8];
	uchar ATime[8];
	uchar DTime[8];
	uchar CommCtrl[8];
	uchar PosModeSelect[8];
	uchar PosLoadMode[8];
	uchar DI0Config[8];
	uchar DI1Config[8];
	uchar DI4Config[8];
	uchar DI5Config[8];
	uchar DI6Config[8];
	uchar DI7Config[8];
	uchar CommCtrlDI[8];
	uchar ServoEnableSelection[8];
	uchar PosSpeed[8];
	uchar DisplayMotorSpeed[8];
	uchar SetPos[13];
	uchar RetSetPos[8];
	uchar clean[8];
	uchar check[8];
	uchar enable[8];
	uchar disable[8];
	uchar save[8];
	int speed;
	int adtime;
	int position;
	
	
}LiChuang;
extern LiChuang servo;

typedef struct {
	uchar SetCarSpeed[8];
	uchar SetServoSpeed[8];
	uchar SetCarStatus[8];
	uchar SetServoStatus[8];
	uchar SetId[8];
	uchar SetServoADTime[8];
	uchar SetServoPos[8];
	
	
}CanCommStu;
extern CanCommStu CanComm;

//typedef struct{
//	unsigned short Win;
//	unsigned short goodspoition;
//}window;
//extern window windows[DEVICESNUM];

typedef struct{
	unsigned short Win;//货物的目的格口
	char info[32];//货物的信息 不一定有
	unsigned short infoLen;//信息长度
	unsigned short goodspoition;//货物当前位置
	unsigned char DeviceLockStatus;//格口是否锁死
	unsigned int TimeOutTick;//用来记录货物流动的时间 
}goodsStu;
extern goodsStu goods[DEVICESNUM];//被分拣货物的相关结构体

typedef struct {
	unsigned short car_error;//错误码
	unsigned short servo_error;//错误码

}motor_error_code_stu;


typedef struct{
	unsigned short id;//本设备ID 一般是1
	//unsigned short goodsnumber;
	int GoodsTimeOut;//用来判断货物是否超时可能丢失 配合goods[i].TimeOutTick使用
	int firstPostionTimeOut;//货物到达第一个分拣小车的超时时间
	//unsigned short Sensor_Position[DEVICESNUM];
	unsigned short Slave_Device_Car_ONLINE[DEVICESNUM];//分拣小车前行滚筒是否正常
	unsigned short Slave_Device_Servo_ONLINE[DEVICESNUM];//分拣小车左右电机是否正常
	unsigned short Slave_Device_ONLINE[DEVICESNUM];//分拣小车的控制板是否正常
	int status;
	unsigned int WinDelayTime;//货物分拣的传感器感应到货物后延迟时间再分拣
	unsigned int IRCheck_DelayTime;//传感器感应后等待运行时间
	unsigned int PowerUp_OnOrOff;//上电默认启动还是停止
	unsigned int Car1DirAndRunTime;//非分拣小车的运动方向和运行时间
	unsigned int maxId;//子设备最大ID数 3台设备 6+1格口 情况下应当等于4
	unsigned int SleepTime;//设备启动后休眠时间
}Slave_Devices_Stu;
extern Slave_Devices_Stu Slave_Device;//所有子设备的相关结构体

typedef struct{
	motor_error_code_stu motor_error_code;
	unsigned short Recv_CAN_ID;
	unsigned short goodsnumber;
	unsigned short sensor_position[DEVICESNUM];

}Recv_CAN_Data_Stu;
extern Recv_CAN_Data_Stu Recv_CAN_Data;//CAN总线相关

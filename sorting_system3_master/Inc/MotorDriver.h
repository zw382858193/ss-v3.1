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
	int RUN_TICK;//�����豸����ʱ��
	int first_deltime;//��һ���豸���м���ʱ��
	int next_state_flag;//�˳���ǰ���к�������������ǰС��
	unsigned short first_xiaoche_dir;//�ı��һ��С�������з���
	int speed;
	unsigned int loop_i;
	unsigned short run_mode;//��������ģʽ�ı�
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
	unsigned short Win;//�����Ŀ�ĸ��
	char info[32];//�������Ϣ ��һ����
	unsigned short infoLen;//��Ϣ����
	unsigned short goodspoition;//���ﵱǰλ��
	unsigned char DeviceLockStatus;//����Ƿ�����
	unsigned int TimeOutTick;//������¼����������ʱ�� 
}goodsStu;
extern goodsStu goods[DEVICESNUM];//���ּ�������ؽṹ��

typedef struct {
	unsigned short car_error;//������
	unsigned short servo_error;//������

}motor_error_code_stu;


typedef struct{
	unsigned short id;//���豸ID һ����1
	//unsigned short goodsnumber;
	int GoodsTimeOut;//�����жϻ����Ƿ�ʱ���ܶ�ʧ ���goods[i].TimeOutTickʹ��
	int firstPostionTimeOut;//���ﵽ���һ���ּ�С���ĳ�ʱʱ��
	//unsigned short Sensor_Position[DEVICESNUM];
	unsigned short Slave_Device_Car_ONLINE[DEVICESNUM];//�ּ�С��ǰ�й�Ͳ�Ƿ�����
	unsigned short Slave_Device_Servo_ONLINE[DEVICESNUM];//�ּ�С�����ҵ���Ƿ�����
	unsigned short Slave_Device_ONLINE[DEVICESNUM];//�ּ�С���Ŀ��ư��Ƿ�����
	int status;
	unsigned int WinDelayTime;//����ּ�Ĵ�������Ӧ��������ӳ�ʱ���ٷּ�
	unsigned int IRCheck_DelayTime;//��������Ӧ��ȴ�����ʱ��
	unsigned int PowerUp_OnOrOff;//�ϵ�Ĭ����������ֹͣ
	unsigned int Car1DirAndRunTime;//�Ƿּ�С�����˶����������ʱ��
	unsigned int maxId;//���豸���ID�� 3̨�豸 6+1��� �����Ӧ������4
	unsigned int SleepTime;//�豸����������ʱ��
}Slave_Devices_Stu;
extern Slave_Devices_Stu Slave_Device;//�������豸����ؽṹ��

typedef struct{
	motor_error_code_stu motor_error_code;
	unsigned short Recv_CAN_ID;
	unsigned short goodsnumber;
	unsigned short sensor_position[DEVICESNUM];

}Recv_CAN_Data_Stu;
extern Recv_CAN_Data_Stu Recv_CAN_Data;//CAN�������

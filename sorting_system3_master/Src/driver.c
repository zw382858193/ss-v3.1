#include "config.h"
#include "MotorDriver.h"

void SensorDriver(void)
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)){
		send_servor_motor_pos((uchar *)servo.SetPos,servo.position);
	}else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)){
		send_servor_motor_pos((uchar *)servo.SetPos,-servo.position);
	}
}

#include <stm32f10x.h>
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>

#define FLASH_DEFAULT_ON_OFF_DATA_ADDR 0x04
#define FLASH_CAR_SPEED_DATA_ADDR FLASH_DEFAULT_ON_OFF_DATA_ADDR+0x04
#define FLASH_CAR_RS485_TICK_ADDR FLASH_CAR_SPEED_DATA_ADDR+0x04
#define FLASH_SERVO_MOTOR_SPEED_ADDR FLASH_CAR_RS485_TICK_ADDR+0x04
#define FLASH_SERVO_MOTOR_ADTIME_ADDR FLASH_SERVO_MOTOR_SPEED_ADDR+0x04
#define FLASH_SERVO_MOTOR_POSITION_ADDR FLASH_SERVO_MOTOR_ADTIME_ADDR+0x04
#define FLASH_DEVICE_ID_ADDR FLASH_SERVO_MOTOR_POSITION_ADDR+0x04
#define FLASH_DEVICE_MAX_ID_ADDR FLASH_DEVICE_ID_ADDR+0x04
#define FLASH_DEVICE_TIMEOUT_ADDR FLASH_DEVICE_MAX_ID_ADDR+0x04
#define FLASH_FIRSTPOSTIONTIMEOUT_ADDR FLASH_DEVICE_TIMEOUT_ADDR+0x04
#define FLASH_CAR1DIRANDRUNTIME_ADDR FLASH_FIRSTPOSTIONTIMEOUT_ADDR+0x04
#define FLASH_SLEEPTIME_ADDR FLASH_CAR1DIRANDRUNTIME_ADDR+0x04
#define FLASH_IRCHECKDELAYTIME_ADDR FLASH_SLEEPTIME_ADDR+0x04
#define FLASH_WINDELAYTIME_ADDR FLASH_IRCHECKDELAYTIME_ADDR+0x04


void flash_init(void);
void flash_read(void);
void flash_delay_data_get(void);
void flash_speed_data_get(void);
void flash_deltime_data_get(void);
void flash_powerup_on_off_data_get(void);
void flash_next_state_flag_data_get(void);
void flash_rs485_tick_data_get(void);
void flash_run_mode_data_get(void);
void flash_first_xiaoce_dir_data_get(void);
void flash_debug1_data_get(void);
void flash_device_id_data_get(void);
void flash_device_maxid_data_get(void);
void flash_device_timeOut_data_get(void);
void flash_firstPostionTimeOut_data_get(void);
void flash_SleepTime_data_get(void);;
void flash_Car1DirAndRunTime_data_get(void);
void flash_IRCheck_DelayTime_data_get(void);
void flash_Win_DelayTime_data_get(void);
void WriteFlashOneWord(uint32_t WriteAddress, uint32_t WriteData);


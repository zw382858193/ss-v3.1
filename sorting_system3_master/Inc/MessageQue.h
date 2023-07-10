#include <stm32f10x.h>
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>

#define ADV_DATA_DEPTH 128 //数据深度
#define ADV_DATA_LENGTH_MAX 32 //每帧数据长度

typedef struct{ 
    u8 ucHeadPtr; //头
    u8 ucTailPtr; //尾
    u8 ucAdvData[ADV_DATA_DEPTH][ADV_DATA_LENGTH_MAX];
}ADV_DATA_LIST;

extern ADV_DATA_LIST stAdvDataList;

u8 Adv_Data_Fill(u8 *buf,u8 length);
u8 Adv_Data_GetLength(void);
void Adv_Data_Clear(void);
//u8 Adv_Data_Get(u8 *buf,u8 *length);
u8 Adv_Data_Get(u8 *buf);

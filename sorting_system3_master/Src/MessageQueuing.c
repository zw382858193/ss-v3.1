#include "MessageQue.h"

ADV_DATA_LIST stAdvDataList;

//消息队列清空
void Adv_Data_Clear(void)
{ 
    stAdvDataList.ucHeadPtr = stAdvDataList.ucTailPtr =0;
}

//获取队列长度
u8 Adv_Data_GetLength(void)
{ 
    u8 ucLen = 0; 
    if((stAdvDataList.ucHeadPtr >= ADV_DATA_DEPTH)||(stAdvDataList.ucTailPtr >= ADV_DATA_DEPTH)) //超过数组长度清零
        Adv_Data_Clear(); 
    if( stAdvDataList.ucTailPtr >= stAdvDataList.ucHeadPtr)  //如果尾指针大于头指针 则长度正常取 尾位置-头位置
        ucLen = stAdvDataList.ucTailPtr - stAdvDataList.ucHeadPtr; 
    else //否则若头位置大于尾位置 则表明数据二次写入 头指针靠近消息队列出口 尾指针靠近消息队列入口 长度取 尾位置+数组深度-头位置
        ucLen = stAdvDataList.ucTailPtr + ADV_DATA_DEPTH - stAdvDataList.ucHeadPtr; 
        
    return ucLen;
}

//往队列里添加消息 返回0表示队列消息已满 无法再存储
u8 Adv_Data_Fill(u8 *buf,u8 length)
{ 
    u8 ucLen = 0; 
    ucLen = Adv_Data_GetLength(); 
    if(ucLen >= ADV_DATA_DEPTH-1) //队列满
        return 0; 
    
    if(length>(ADV_DATA_LENGTH_MAX-1))
        length=(ADV_DATA_LENGTH_MAX-1); 
    
    stAdvDataList.ucAdvData[stAdvDataList.ucTailPtr][0] = length; //转存时 每帧数据第一个字节用于储存数据长度信息
    
    for(ucLen=0;ucLen<length;ucLen++) 
        stAdvDataList.ucAdvData[stAdvDataList.ucTailPtr][1+ucLen] = buf[ucLen];

    stAdvDataList.ucTailPtr++; //尾指针数据下移 表示数据多了一帧
    
    if(stAdvDataList.ucTailPtr >= ADV_DATA_DEPTH)  //若尾指针位置超出队列长度 置零
        stAdvDataList.ucTailPtr = 0;

    return 1;
}

//从队列获取消息 0表示队列信息空 不能读取
u8 Adv_Data_Get(u8 *buf)//,u8 *length)
{ 
    u8 ucLen = 0,temp8; 
	
    ucLen = Adv_Data_GetLength(); 
    if(ucLen == 0) 
        return 0; 
    
    //*length = stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][0]; //获取数据长度
    //ucLen=*length;
		ucLen=stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][0];
    
    for(temp8=0;temp8<ucLen;temp8++){
      buf[temp8]=stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][1+temp8];
			stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][1+temp8]=0x00;
		}
    stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][0] =0;
    stAdvDataList.ucHeadPtr++; //头指针位置下移 表示数据少了一帧
    
    if(stAdvDataList.ucHeadPtr >= ADV_DATA_DEPTH) //若头指针位置超出队列长度 则置零
        stAdvDataList.ucHeadPtr = 0; 
    if(stAdvDataList.ucHeadPtr == stAdvDataList.ucTailPtr){//如果数据读完就清除队列
			Adv_Data_Clear();
		}
    return 1;
}

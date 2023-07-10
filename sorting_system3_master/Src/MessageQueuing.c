#include "MessageQue.h"

ADV_DATA_LIST stAdvDataList;

//��Ϣ�������
void Adv_Data_Clear(void)
{ 
    stAdvDataList.ucHeadPtr = stAdvDataList.ucTailPtr =0;
}

//��ȡ���г���
u8 Adv_Data_GetLength(void)
{ 
    u8 ucLen = 0; 
    if((stAdvDataList.ucHeadPtr >= ADV_DATA_DEPTH)||(stAdvDataList.ucTailPtr >= ADV_DATA_DEPTH)) //�������鳤������
        Adv_Data_Clear(); 
    if( stAdvDataList.ucTailPtr >= stAdvDataList.ucHeadPtr)  //���βָ�����ͷָ�� �򳤶�����ȡ βλ��-ͷλ��
        ucLen = stAdvDataList.ucTailPtr - stAdvDataList.ucHeadPtr; 
    else //������ͷλ�ô���βλ�� ��������ݶ���д�� ͷָ�뿿����Ϣ���г��� βָ�뿿����Ϣ������� ����ȡ βλ��+�������-ͷλ��
        ucLen = stAdvDataList.ucTailPtr + ADV_DATA_DEPTH - stAdvDataList.ucHeadPtr; 
        
    return ucLen;
}

//�������������Ϣ ����0��ʾ������Ϣ���� �޷��ٴ洢
u8 Adv_Data_Fill(u8 *buf,u8 length)
{ 
    u8 ucLen = 0; 
    ucLen = Adv_Data_GetLength(); 
    if(ucLen >= ADV_DATA_DEPTH-1) //������
        return 0; 
    
    if(length>(ADV_DATA_LENGTH_MAX-1))
        length=(ADV_DATA_LENGTH_MAX-1); 
    
    stAdvDataList.ucAdvData[stAdvDataList.ucTailPtr][0] = length; //ת��ʱ ÿ֡���ݵ�һ���ֽ����ڴ������ݳ�����Ϣ
    
    for(ucLen=0;ucLen<length;ucLen++) 
        stAdvDataList.ucAdvData[stAdvDataList.ucTailPtr][1+ucLen] = buf[ucLen];

    stAdvDataList.ucTailPtr++; //βָ���������� ��ʾ���ݶ���һ֡
    
    if(stAdvDataList.ucTailPtr >= ADV_DATA_DEPTH)  //��βָ��λ�ó������г��� ����
        stAdvDataList.ucTailPtr = 0;

    return 1;
}

//�Ӷ��л�ȡ��Ϣ 0��ʾ������Ϣ�� ���ܶ�ȡ
u8 Adv_Data_Get(u8 *buf)//,u8 *length)
{ 
    u8 ucLen = 0,temp8; 
	
    ucLen = Adv_Data_GetLength(); 
    if(ucLen == 0) 
        return 0; 
    
    //*length = stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][0]; //��ȡ���ݳ���
    //ucLen=*length;
		ucLen=stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][0];
    
    for(temp8=0;temp8<ucLen;temp8++){
      buf[temp8]=stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][1+temp8];
			stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][1+temp8]=0x00;
		}
    stAdvDataList.ucAdvData[stAdvDataList.ucHeadPtr][0] =0;
    stAdvDataList.ucHeadPtr++; //ͷָ��λ������ ��ʾ��������һ֡
    
    if(stAdvDataList.ucHeadPtr >= ADV_DATA_DEPTH) //��ͷָ��λ�ó������г��� ������
        stAdvDataList.ucHeadPtr = 0; 
    if(stAdvDataList.ucHeadPtr == stAdvDataList.ucTailPtr){//������ݶ�����������
			Adv_Data_Clear();
		}
    return 1;
}

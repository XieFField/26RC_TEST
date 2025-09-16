/**
 * @file position.cpp
 * @author Wu Jia
 * @brief position�����ļ�
 * @attention ���ļ�����position����action
 * @date 2025-06-25
 */

/*

  Reposition_SendData���������ض�λ��idΪ1����ض�λX,Y���ꣻid2����
  �����ض�λyaw, id3�ɽ�imu�ϵ�����
  
*/

// ���������ڽ�20�ֽڵĸ��������յ� float ������

#include "position.h"
#include <math.h>

RawPos RawPosData = {0};
RealPos RealPosData = {0};

#define NEW_OR_OLD 1

union
{
	uint8_t data[24];
	float ActVal[6];
} posture;

//���ջص�����
uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len)
{
    uint8_t count = 0;
	uint8_t i = 0;
	uint8_t CRC_check[2];//CRCУ��λ�����ļ�δ����
	
	
	
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			case 0:
			{
				if (buf[i] == FRAME_HEAD_POSITION_0)   //���հ�ͷ1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 1:
			{
				if (buf[i] == FRAME_HEAD_POSITION_1) //���հ�ͷ2
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 2://����֡ID�����ݳ���
			{
				if (buf[i] == 0x01) 
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 3:
			{
				if (buf[i] == 0x18) //0x0c
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 4://��ʼ��������
			{
				uint8_t j;
				
				#if NEW_OR_OLD
				if (i > len - 24)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 24; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
                
                #else
                if (i > len - 24)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 20; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
                
                #endif
				count++;
				break;
			}
			
			//����CRCУ����
			case 5:
			{
				uint8_t j;
				
				for(j = 0; j < 2; j++)
				{
					CRC_check[j] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			case 6:
			{
				if (buf[i] == FRAME_TAIL_POSITION_0)  //���հ�β1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 7:
			{
				if (buf[i] == FRAME_TAIL_POSITION_1)  //���հ�β2
				{	
					//�ڽ��հ�β2��ſ�ʼ�����ص�
					Update_RawPosition(posture.ActVal);
				}
				count = 0;
				
				break_flag = 0;
				
				break;
			}
			
			default:
			{
				count = 0;
				break;
			}
		}
		
	}
	return 0;
}

// ���ݸ��º��������������ֵ���� RawPos �� RealPos
void Update_RawPosition(float value[5])
{
//	//��ֵ
	// RawPosData.LAST_Pos_X = RawPosData.Pos_X;
	// RawPosData.LAST_Pos_Y = RawPosData.Pos_Y;

	// ��������
    // ��λ�õ�λ�� mm ת��Ϊ m������ 1000��
	RawPosData.Pos_X = value[0] / 1000.f; 
	RawPosData.Pos_Y = value[1] / 1000.f; 
	RawPosData.angle_Z = value[2];
	RawPosData.Speed_Yaw = value[3];
	RawPosData.Speed_Y = value[4];

//   //�������
	// RawPosData.DELTA_Pos_X = RawPosData.Pos_X - RawPosData.LAST_Pos_X;
	// RawPosData.DELTA_Pos_Y = RawPosData.Pos_Y - RawPosData.LAST_Pos_Y;

   //��������
	RealPosData.world_yaw = RawPosData.angle_Z;
    RealPosData.world_x   =  RawPosData.Pos_X + RealPosData.dx;
	RealPosData.world_y   =  RawPosData.Pos_Y + RealPosData.dy;

	RealPosData.dyaw = RawPosData.Speed_Yaw;


	//���밲װ���
    //�ۼ�λ��
	// RawPosData.REAL_X += (RawPosData.DELTA_Pos_X);
	// RawPosData.REAL_Y += (RawPosData.DELTA_Pos_Y);
	
    // ���迼�ǰ�װ����ȡ��ע���·����룺
    //���㰲װ���
	// RealPosData.world_x = RawPosData.REAL_X + INSTALL_ERROR_X * sinf(RealPosData.world_yaw * PI / 180.f);
	// RealPosData.world_y = RawPosData.REAL_Y + INSTALL_ERROR_Y * cosf(RealPosData.world_yaw * PI / 180.f);
}



void Reposition_SendData(float X, float Y)
{
	uint8_t txBuffer[16] = {0};

	union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;

	//��ͷ
	txBuffer[0] = FRAME_HEAD_POSITION_0;
	txBuffer[1] = FRAME_HEAD_POSITION_1;
    txBuffer[2]=0x01;

	//���ݳ���
	txBuffer[3] = 0x08;

	//����
	floatUnion.f = X;
	txBuffer[4] = floatUnion.bytes[0];
    txBuffer[5] = floatUnion.bytes[1];
    txBuffer[6] = floatUnion.bytes[2];
    txBuffer[7] = floatUnion.bytes[3];

    floatUnion.f = Y;
    txBuffer[8] = floatUnion.bytes[0];
    txBuffer[9] = floatUnion.bytes[1];
    txBuffer[10] = floatUnion.bytes[2];
    txBuffer[11] = floatUnion.bytes[3];

	//CRC
	txBuffer[12] = 0;
	txBuffer[13] = 0;
	//��β
	txBuffer[14] = FRAME_TAIL_POSITION_0;
	txBuffer[15] = FRAME_TAIL_POSITION_1;

	HAL_UART_Transmit(&huart3, txBuffer, 16, HAL_MAX_DELAY);
}

/** 
 * @brief position�ض�λ �������
 * @version 0.1
 */
void POS_Relocate_ByDiff(float X, float Y, float yaw)
{
	RealPosData.dx = X - RealPosData.world_x;
	RealPosData.dy = Y - RealPosData.world_y;
}


/**
 * @file ViewCommunication.c
 * @author Wu Jia / Luo Qi Ling
 * @brief ���Ӿ�ͨѶ�ӿ�
 */
#include <stdint.h>							// C ���Ա�׼��ͷ�ļ��������������Ͷ����
#include <string.h>							// C ���Ա�׼��ͷ�ļ��������ַ�����������
#include "stm32f4xx_hal.h"					// main.h ͷ�ļ�������ʵ�Ѿ����������ͷ�ļ�
#include "cmsis_os.h"						// FreeRTOS ͷ�ļ�
#include "FreeRTOS.h"						// FreeRTOS ͷ�ļ�
#include "main.h"							// HAL ��ͷ�ļ�
#include "task.h"							// FreeRTOS ͷ�ļ��������Ҳ�֪��Ҫ��Ҫ�������ͷ�ļ�
#include "drive_uart.h"						// ��������ͷ�ļ�
#include "ViewCommunication.h"		        // ���ͨ��ģ��ͷ�ļ�
#include "position.h"
#include <string.h>
#include "queue.h"

#define ViewCommunication_UartHandle &huart1	

static uint8_t DataPacket[17] = { 0 };		// ����������ֽ���Ч�غɵ����ݰ�

float receivex;		//�����ɰ�����һɾ�ö�ط�����Ҫɾ���Ȳ�����
float receivey;
float receiveyaw;

/**
 * @def �Ӿ�ͨѶ--����
 * @brief	5��float��������
 */




ReceiveRealData_S Pos_Now = {0};
ReceiveRealData_S Pos_Target = {0};

struct{
	ReceiveRealData_S Pos_Now ;
	ReceiveRealData_S Pos_Target
}ReceiveData_total;

/**
 * @brief ����궨�ؼ�λ��
 * @param ready ��Ϊ1 ��ʼ�궨 ��Ϊ0 �ͽ����궨
 * 				���͵�5��
 */
void Camera_Calibration(uint8_t ready)
{
	union 
	{
		/* data */
		//��ʱ����5��float����ֻ�ǵ�һ��float���ã��Ӿ���Ҫ��
		float data[5];
		uint8_t buffer[20]
	}camera_cmd;

	if(ready)
		camera_cmd.data[0] = 0;	//0Ϊ��ʼ�궨�ź�
	else if(!ready)
		camera_cmd.data[0] = 1; //1Ϊ�����궨���ź�

	for(int i = 1; i < 5; i++)
		camera_cmd.data[i] = 0;
	
	static uint8_t data_send[25];
	data_send[0] = 0x55;	//��ͷ1
	data_send[1] = 0xAA;	//��ͷ2
	data_send[2] = 0x14;	//���ݰ�����

	for(int i = 0; i < 20; i++)
		data_send[3 + i] = camera_cmd.buffer[i];

	data_send[23] = 0x0D;
	data_send[24] = 0x0A;
	HAL_UART_Transmit_DMA(ViewCommunication_UartHandle, data_send, 25);
}

extern xQueueHandle VISION_TO_REAL_Port;
extern xQueueHandle VISION_TO_TARGET_Port;
void Update_ReceiveData(ReceiveData_E data)
{
	static float signal_ = 0;
	signal_ = data.RealData[0];
	if(signal_ == 0)// Robot's pos
	{
		Pos_Now.x = data.RealData[1];
		Pos_Now.y = data.RealData[2];
		Pos_Now.z = data.RealData[3];
		Pos_Now.yaw = data.RealData[4];
	}
	else if(signal_ == 1) // target point
	{
		Pos_Target.x = data.RealData[1];
		Pos_Target.y = data.RealData[2];
		Pos_Target.z = data.RealData[3];
		Pos_Target.yaw = data.RealData[4];
	}

	// �жϰ�ȫ�Ķ��з��ͣ�xQueueSendFromISR
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(signal_ == 0)
		xQueueSendFromISR(VISION_TO_REAL_Port, &Pos_Now, &xHigherPriorityTaskWoken);
	else if(signal_ == 1)
		xQueueSendFromISR(VISION_TO_TARGET_Port, &Pos_Target, &xHigherPriorityTaskWoken);
}



uint32_t View_UART1_RxCallback(uint8_t *buf, uint16_t len)
{
	uint8_t cnt = 0;
	uint8_t n = 0;
	uint8_t break_flag = 1;
	static ReceiveData_E ReceiveData = {0};
	while(n < len && break_flag == 1)
	{
		switch (cnt)
		{
			case 0:
			{
				if (buf[n] == 0x55)   //���հ�ͷ1
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}
			case 1:
			{
				if (buf[n] == 0xAA) //���հ�ͷ2
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}

			case 2:
			{
				if (buf[n] == 0x14) //���ճ��� 20
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}
			case 3://��ʼ��������
			{
				uint8_t j;
				
								
				for(j = 0; j < 20 ; j++)
				{
					ReceiveData.rxbuff[j] = buf[n];
					n++;
				}
				cnt++;
				break;
			}
			

			case 4:
			{
				if (buf[n] == 0x0D)  //���հ�β1
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}
			
			case 5:
			{
				if (buf[n] == 0x0A)     //��β2	
					Update_ReceiveData(ReceiveData);
				
				cnt = 0;
				

				break_flag = 0;
				
				break;
			}
			
			default:
			{
				cnt = 0;
				break;
			}
		}
		
	}
	return 0;
}

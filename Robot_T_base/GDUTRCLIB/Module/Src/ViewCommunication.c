/**
 * @file ViewCommunication.c
 * @author Wu Jia / Luo Qi Ling
 * @brief 与视觉通讯接口
 */
#include <stdint.h>							// C 语言标准库头文件，包含整数类型定义等
#include <string.h>							// C 语言标准库头文件，包含字符串处理函数等
#include "stm32f4xx_hal.h"					// main.h 头文件里面其实已经包含了这个头文件
#include "cmsis_os.h"						// FreeRTOS 头文件
#include "FreeRTOS.h"						// FreeRTOS 头文件
#include "main.h"							// HAL 库头文件
#include "task.h"							// FreeRTOS 头文件，但是我不知道要不要包含这个头文件
#include "drive_uart.h"						// 串口驱动头文件
#include "ViewCommunication.h"		        // 板间通信模块头文件
#include "position.h"
#include <string.h>
#include "queue.h"

#define ViewCommunication_UartHandle &huart1	

static uint8_t DataPacket[17] = { 0 };		// 定义包含单字节有效载荷的数据包

float receivex;		//废弃旧案，但一删好多地方跟着要删，先不管了
float receivey;
float receiveyaw;

/**
 * @def 视觉通讯--接收
 * @brief	5个float数据如下
 */




ReceiveRealData_S Pos_Now = {0};
ReceiveRealData_S Pos_Target = {0};



static void ViewCommunication_BytePack(uint8_t* DataPacket);

void ViewCommunication_SendByte(void)
{
	//uint8_t DataPacket[11] = { 0 };		// 定义包含单字节有效载荷的数据包

	ViewCommunication_BytePack(DataPacket);

//	printf_DMA("%f\r\n", DataPacket);
	HAL_UART_Transmit_DMA(ViewCommunication_UartHandle, DataPacket,17);
}


/**
 * 旧的应该用不上力
 */
static void ViewCommunication_BytePack(uint8_t* DataPacket)
{
	union
	{
		float data[3];
		uint8_t buffer[12]
	}temp;
	temp.data[0]=RealPosData.world_x;
	temp.data[1]=RealPosData.world_y;
	temp.data[2]=RealPosData.world_yaw;
	DataPacket[0] = 0x55;					// 数据包头
	DataPacket[1] = 0xAA;					// 数据包头
	DataPacket[2] = 0x0C;					// 数据包长度
	DataPacket[3] = temp.buffer[0];	// 有效载荷
	DataPacket[4] = temp.buffer[1];
	DataPacket[5] = temp.buffer[2];
	DataPacket[6] = temp.buffer[3];					
	DataPacket[7] = temp.buffer[4];	
	DataPacket[8] = temp.buffer[5];
	DataPacket[9] = temp.buffer[6];	
	DataPacket[10] =temp.buffer[7];
	DataPacket[11] =temp.buffer[8];
	DataPacket[12] =temp.buffer[9];					
	DataPacket[13] =temp.buffer[10];	
	DataPacket[14] =temp.buffer[11];
	DataPacket[15] = 0x0D;
	DataPacket[16] = 0x0A;
}

/**
 * @brief 相机标定秘籍位置
 * @param ready 若为1 则开始标定 若为0 就结束标定
 * 				发送的5个
 */
void Camera_Calibration(uint8_t ready)
{
	union 
	{
		/* data */
		//暂时还是5个float，但只是第一个float有用，视觉的要求
		float data[5];
		uint8_t buffer[20]
	}camera_cmd;

	if(ready)
		camera_cmd.data[0] = 0;	//0为开始标定信号
	else if(!ready)
		camera_cmd.data[0] = 1; //1为结束标定的信号

	for(int i = 1; i < 5; i++)
		camera_cmd.data[i] = 0;
	
	static uint8_t data_send[25];
	data_send[0] = 0x55;	//包头1
	data_send[1] = 0xAA;	//包头2
	data_send[2] = 0x14;	//数据包长度

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

	xQueueSend(VISION_TO_REAL_Port, &Pos_Now, 0);
	xQueueSend(VISION_TO_TARGET_Port, &Pos_Target, 0);
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
				if (buf[n] == 0x55)   //接收包头1
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
				if (buf[n] == 0xAA) //接收包头2
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
				if (buf[n] == 0x14) //接收长度 20
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
			case 3://开始接收数据
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
				if (buf[n] == 0x0D)  //接收包尾1
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
				if (buf[n] == 0x0A)     //包尾2	
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

static void Camera_Calibration(uint8_t ready)
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
	//HAL_UART_Transmit_DMA(ViewCommunication_UartHandle, data_send);
    for(int i = 0; i < 25; i++)
        printf("%d  ", data_send[i]);
}
int main (void)
{
    Camera_Calibration(1);
    printf("\n");
    Camera_Calibration(0);
    return 0;
}
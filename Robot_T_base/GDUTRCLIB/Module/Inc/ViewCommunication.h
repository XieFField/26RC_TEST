#ifndef __ViewCommunication_H
#define __ViewCommunication_H


#include <stdint.h>


 // C”Ô—‘≤ø∑÷
#ifdef __cplusplus
extern "C" {
#endif

typedef union ReceiveData_E
{
	float RealData[5];
	uint8_t rxbuff[20];
}ReceiveData_E;

typedef struct ReceiveRealData_S{
	float signal; 	//0????? 1????????
	float x;		//??
	float y;		//??
	float z;		//??
	float yaw;
}ReceiveRealData_S;

static void ViewCommunication_BytePack(uint8_t* DataPacket);
void ViewCommunication_SendByte(void);

uint32_t View_UART1_RxCallback(uint8_t *buf, uint16_t len);

static void Camera_Calibration(uint8_t ready);

#ifdef __cplusplus
}
#endif


#endif
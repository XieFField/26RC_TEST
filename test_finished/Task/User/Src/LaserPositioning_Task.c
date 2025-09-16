/**
 * @file		LaserPositioning_Task.c | LaserPositioning_Task.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-19 (��������)
 * @date        2025-06-09 (����޸�����)
 * @platform	CubeMX����HAL��Ĵ���FreeRTOS v2����ϵͳ��STM32F407ZGT6��Ƭ��
 * @version     1.2.1
 * @note		�����ԣ����߲��Ƽ��ڵ�һ�����Ϲ��ض��������ģ��ʹ�ö����������Զ�����ģʽ���в�����ԭ��������
 *              1. ��ģʽ�£�������ģ����ĵ��β���ʱ���޷�ȷ����ֻ��ͨ��������ѯ�ķ�ʽ��ȡ���������������ʱ��Ŀ���
 *				2. ������ģ�����Ӧ��Ƶ�ʣ�ָģ���ڷ�����Ϣ���ÿ����ٴν���ָ���ʱ�䣩�ƺ��ϵͣ��ƺ���5ms���ң���δ��һ����֤
 * 				3. ��ʹ�ø�FreeRTOS����ϵͳʱ��δ֪ԭ����ʹ��osDelay()��HAL_Delay()�����Դ���DMA���ͽ��е���ʱ��׼ȷ�������޷������Ŀ��Ƶ�Ƭ����ģ�鷢��ָ���ʱ�������ᵼ��ģ���޷�������Ӧ��
 *				������δ���Թ���һ�����Ϲ��ض��������ģ��������е����Զ�����ģʽ�Ĳ���
 *				����ǽ���ʹ�öമ�ڷֱ���ص���������ģ��ķ�ʽ���в���
 *				������ʵҲ����������̫�ˣ����Ե��������ϲ�������Ĳ���
 * @warning		��ģ�鲻�����������裬�����ʵ����Ҫ����������裬����ģ���޷���������
 * @license     WTFPL License
 *
 * @par �汾�޶���ʷ
 * @{
 *  @li �汾��: 1.2.1
 *      - �޶�����: 2025-06-09
 *      - ��Ҫ���:
 *			- ������1.2.0�汾�ĳ��򣬽��Position��Yaw�����ݽ����˼��ⶨλ�Ķ�λ��׼�Ȳ���
 *			- ���Խ�����������ⶨλ�ľ��Ȼ�������Ԥ�ڣ��ڼ�������ľ��Ƕ�Ϊ+-3���ڣ�����������7.5m����ʱ���ڴ󲿷�������ܱ�֤X��Y��Ķ�λ������+-1cm����
 *		- ����֮��:
 *			- ��λ��ʼУ���㷨���һ���������Ľ����Լ������ڼ�����ģ�鰲װλ�úͽǶ�ƫ������ĳ�ʼ�̶����
 *			- ��λ�����㷨�����һ���������Ľ����Լ������ڼ�������д��������
 *      - ����: ZhangJiaJia
 * 
 *  @li �汾��: 1.2.0
 *      - �޶�����: 2025-06-08
 *      - ��Ҫ���:
 *			- �ṩ��Position����ϵ��ת���ת����ת������
 *		- ����֮��:
 *			- ����δʵ�⣬������ͨ��
 *      - ����: ZhangJiaJia
 * 
 *  @li �汾��: 1.1.0
 *      - �޶�����: 2025-06-04
 *      - ��Ҫ���:
 *			- �Ż��˼�����ģ����ĳ�ʼ������ LaserModuleGroup_Init()
 *				- ������ϵ��ģ���޷�������ʼ������Ҫ�ֶ���λ��Ƭ��һ�β����������������
 *				- �����˼�����ģ�����ʼ��ʧ�ܺ�����Ի���
 *			    - �����ִ���DMA���͸�Ϊ�˴����������ͣ�ԭ���Ǵ���DMA����ʱ��������Ʊ�����������ռDMA�̣߳����������޷��������ᷢ�ͣ�ʹģ���޷���������������
 *      - ����: ZhangJiaJia
 * 
 *  @li �汾��: 1.0.1
 *      - �޶�����: 2025-05-27
 *      - ��Ҫ���:
 *			- ��������ʹ��double�ĵط���Ϊʹ��float��ԭ����STM32F4ϵ�е�Ƭ����Ӳ���������㵥Ԫ��֧��double���͵�����
 *			- ��ԭ������.h�ļ��еĽṹ��Ķ��������.c�ļ��У��Լ��ٶ�����©
 *      - ����: ZhangJiaJia
 * 
 *  @li �汾��: 1.0.0
 *		- �޶�����: 2025-05-30
 *		- ��Ҫ���:
 *			- �������������ģ�������������
 *		- ����֮��:
 *			- ģ���״̬�����ò����ƣ���״̬���Ĵ���Ҳ��������
 *			- �����ڳ�ʼ�������еĽ�׳�Բ���
 * 			- �����Yaw������������������������
 *		- ����:
 *			- ����������������д
 *		- ����: ZhangJiaJia
 * 
 *  @li �汾��: 0.3.1
 *      - �޶�����: 2025-05-27
 *      - ��Ҫ���:
 *			- �޸���CubeMX�����е�һЩ�޺�bug
 *      - ����: ZhangJiaJia
 *
 *	@li �汾��: 0.3.0
 *		- �޶�����: 2025-05-24
 *		- ��Ҫ���:
 *			- ��2.0�汾δ��ɵ���������ģ������˲��Բ�ͨ��
 *			- ��д����˼򵥵Ķ�λ�㷨����
 *			- �޸���0.2.1�汾δ��ȫ�޸����޺�bug
 *		- ����֮��:
 *			- ������ʵ������
 *		- ����: ZhangJiaJia
 *
 *  @li �汾��: 0.2.1
 *		- �޶�����: 2025-05-23
 *		- ��Ҫ���:
 *			- ʹ��vTaskDelayUntil()������ osDelay() �����������Ż�
 *			- �޸���FreeRTOS�����ļ���������������޺�bug
 *		- ����: ZhangJiaJia
 *
 *	@li �汾��: 0.2.0
 *		- �޶�����: 2025-05-23
 *      - ��Ҫ���:
 *			- ʵ���˼�����ģ����Ķ����������Զ���������ȡ����״̬����ȡ��������ĺ���
 *			- ����˼��׵�ģ��״̬��
 *		- ����֮��:
 *			- Ŀǰֻ�Ե�������ģ������˲��ԣ�δͬʱ����������ģ����в���
 *			- ��ʱ���������û�������Ҫ��һ��ʹ��vTaskDelayUntil()���������Ż�
 *			- ģ���״̬�����ò����ƣ���״̬���Ĵ���Ҳ��������
 *      - ����: ZhangJiaJia
 *
 *	@li �汾��: 0.1.0
 *      - �޶�����: 2025-05-21
 *      - ��Ҫ���:
 *			- �½� LaserPositioning_Task ���������uart4��DMA�����жϽ��ճ��򣬲������յ����ݴ���FreeRTOS�Ķ�����
 *      - ����: ZhangJiaJia
 * @}
 */


// ��������ϵ���壺
// �������������������ص����ϽǶ���Ϊ����ԭ�㣬����ΪX�ᣬ����ΪY�ᣬ
// ��������ϵ��X�᷽��Ϊ0����ʱ��Ϊ������Ĭ�ϵ�λ���ȣ���Χ��-PI��PI֮��


// ״̬����0���������������쳣

// �Ժ�������ֵ LaserModuleGroupState ��˵����
// 0x00������ģ���鴦������״̬
// 0x01������ģ���鴦���쳣״̬

// �Ժ�������ֵ LaserPositioningState ��˵����
// 0x00�����ⶨλ��������״̬
// 0x01�����ⶨλ�����쳣״̬

// �� LaserModuleMeasurementDataTypedef �е� State ��˵����
// 0x00������ģ�鴦������״̬
// 0x01��������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
// 0x02��������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȶ�У�鲻ͨ��
// 0x04��������ģ��������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
// 0x08����


#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"		// main.h ͷ�ļ�������ʵ�Ѿ����������ͷ�ļ�
#include "cmsis_os.h"
#include "data_pool.h"
#include "FreeRTOS.h"
#include "main.h"
#include "LaserPositioning_Task.h"

typedef struct LaserModuleConfigurationData
{
	UART_HandleTypeDef* UartHandle;			// ���ھ��
	QueueHandle_t ReceiveQueue;		// ����DMA���ն��о��
	uint8_t Address;			// ����ģ��ԭʼ��ַ
	uint8_t ReadAddress;
	uint8_t WriteAddress;
}LaserModuleConfigurationDataTypedef;

typedef struct LaserModuleMeasurementData
{
	uint32_t Distance;
	uint16_t SignalQuality;
	uint16_t State;
}LaserModuleMeasurementDataTypedef;

typedef struct LaserModuleData
{
	LaserModuleConfigurationDataTypedef ConfigurationData;
	LaserModuleMeasurementDataTypedef MeasurementData;
}LaserModuleDataTypedef;

typedef struct LaserModuleDataGroup
{
	LaserModuleDataTypedef LaserModule1;
	LaserModuleDataTypedef LaserModule2;
}LaserModuleDataGroupTypedef;

typedef struct WorldXYCoordinates
{
	float X;		// ��λ��m
	float Y;		// ��λ��m
}WorldXYCoordinatesTypedef;


#define PI							3.14159265358979323846f			// ����Բ���ʳ���PI


#define LaserModule_1_UartHandle &huart6		// ������ģ��1���ھ��
#define LaserModule_2_UartHandle &huart4		// ������ģ��2���ھ��

#define LaserModule1Address				0x00							// ������ģ��1��ַ
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// ������ģ��1����ַ
#define LaserModule1WriteAddress		LaserModule1Address				// ������ģ��1д��ַ

#define LaserModule2Address				0x00							// ������ģ��2��ַ
#define LaserModule2ReadAddress			(LaserModule2Address | 0x80)	// ������ģ��2����ַ
#define LaserModule2WriteAddress		LaserModule2Address 			// ������ģ��2д��ַ


static uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];


static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserPositioning_YawJudgment(float* Yaw);
static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser);
static void LaserPositioning_GetYaw(float* Yaw);
static void GetPositionYaw(float* Yaw);
static uint8_t LaserPositioning_XYWorldCoordinatesVerification(const WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw);
static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
static void SendPositionXYCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
static uint8_t MyUART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);
static uint8_t MyUART_Receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout);
static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size);

uint32_t Laser_Y;
uint32_t Laser_X;
float Laser_Y_return;
float Laser_X_return;

//float delta_hoop_x = 3.884f;
//float delta_hoop_y = 0.746f;

float delta_hoop_x = 4.050f;
float delta_hoop_y = 0.967f;

uint16_t deltaX = 224;
uint16_t deltaY = 220;

float YYY;
float XXX;



void LaserPositioning_Task(void* argument)
{
	uint8_t LaserModuleGroupState = 0;	// ������ģ��״̬����
	uint8_t LaserPositioningState = 0;	// ���ⶨλ״̬����
	WorldXYCoordinatesTypedef WorldXYCoordinates;	// ��������ϵXY����������ڳ����������������������ϽǶ���Ϊ����ԭ�㣬����ΪX�ᣬ����ΪY��
	float Yaw = (3.0f / 2.0f) * PI;					// ƫ���Ǳ�������λ���ȣ�0��ʾ��������ϵ��X�᷽����ʱ��Ϊ�����򣬷�Χ��-PI��PI֮��
	TickType_t LastTimestamp = xTaskGetTickCount();			// �ϴ�ʱ�������������vTaskDelayUntil()�����ľ�����ʱ
	static LaserModuleDataGroupTypedef LaserModuleDataGroup;		// ������ģ�����������

	LaserModuleGroupState |= LaserModuleGroup_Init(&LaserModuleDataGroup);			// ������ģ�����ʼ��



	osDelay(100);	// ��ʱ100ms���ȴ�������ģ���һ�����ݽ������

	for(;;)
	{
		LaserModuleGroupState = 0;	// ������ģ��״̬����
		LaserPositioningState = 0;  // ���ⶨλ״̬����
		LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// ������ģ��1״̬����
		LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// ������ģ��2״̬����

		LaserModuleGroupState |= LaserModuleGroup_AnalysisModulesMeasurementResults(&LaserModuleDataGroup);			// ������ģ�����ȡ�������

         Laser_X = LaserModuleDataGroup.LaserModule2.MeasurementData.Distance;
        Laser_Y = LaserModuleDataGroup.LaserModule1.MeasurementData.Distance;

		if(Laser_X == 0 || Laser_Y == 0)
		{
			osDelay(100);
			LaserModuleGroup_Init(&LaserModuleDataGroup);			// ������ģ�����ʼ��
			osDelay(10000);
			continue;
		}
        
        // XXX = (float)Laser_X - XX ;
        // YYY = (float)Laser_Y - YY ;
        
       // Laser_Y_return = -((float)Laser_Y + deltaY) / 1000.f + delta_hoop_y;
        
        //Laser_Y_return = ((float)Laser_Y + deltaY) / 1000.f - delta_hoop_y;    		//�ֳ���������
        //Laser_X_return = -(((float)Laser_X - deltaX) / 1000.f) + delta_hoop_x;


		//Laser_Y_return = ((float)Laser_Y + deltaY) / 1000.f - delta_hoop_y;			//ѵ��������
		//Laser_X_return = -(((float)Laser_X - deltaX) / 1000.f) + delta_hoop_x;
		
        Laser_X_return = -(float)(Laser_X + 257) / 1000.f + delta_hoop_x;
        Laser_Y_return = (float)(Laser_Y + 374) / 1000.f - delta_hoop_y;

        // XXX = XXX / 1000.f;
        // YYY = YYY / 1000.f;
        
		//LaserPositioning_GetYaw(&Yaw);		// ��ȡƫ���ǣ���λ����


		
		vTaskDelayUntil(&LastTimestamp, pdMS_TO_TICKS(40));		// ÿ40msִ��һ������
	}
}

static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// ������ģ��״̬����

	// ������ģ��1��ʼ��
	LaserModuleDataGroup->LaserModule1.ConfigurationData.UartHandle = LaserModule_1_UartHandle;		// ���ü�����ģ��1�Ĵ��ھ��
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReceiveQueue = Receive_LaserModuleData_1_Port;	// ���ü�����ģ��1�Ĵ���DMA���ն���
	LaserModuleDataGroup->LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;
	LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = 0;	// ������ģ��1�������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = 0;	// ������ģ��1�ź��������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule1.MeasurementData.State = 0;	// ������ģ��1״̬���ݳ�ʼ��

	// ������ģ��2��ʼ��
	LaserModuleDataGroup->LaserModule2.ConfigurationData.UartHandle = LaserModule_2_UartHandle;		// ���ü�����ģ��2�Ĵ��ھ��
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReceiveQueue = Receive_LaserModuleData_2_Port;	// ���ü�����ģ��2�Ĵ���DMA���ն���
	LaserModuleDataGroup->LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;
	LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = 0;	// ������ģ��2�������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = 0;	// ������ģ��2�ź��������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule2.MeasurementData.State = 0;	// ������ģ��2״̬���ݳ�ʼ��

	TickType_t Timestamp = 0;
	vTaskDelayUntil(&Timestamp, pdMS_TO_TICKS(1000));	// ȷ�����ϵ������Ѿ���ʱ3000ms��ȷ��������ģ�������ģ���ڲ���ʼ��

	osDelay(100);

	LaserModuleGroupState |= LaserModule_StopContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule1);		// ֹͣ������ģ��1�������Զ�����
	LaserModuleGroupState |= LaserModule_StopContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule2);		// ֹͣ������ģ��2�������Զ�����
	
    osDelay(100);
    
	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);	// �򿪼�����ģ��1�ļ�����
	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);	// �򿪼�����ģ��2�ļ�����

	if(LaserModuleGroupState != 0)		// ���������ģ����״̬�쳣
	{
		uint8_t i = 1;		// ���Դ���������
		for (;; i++)
		{
			LaserModuleGroupState = 0;		// ���ü�����ģ����״̬

			LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);	// ������ģ��1�����Զ�����״̬����
			LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);	// ������ģ��2�����Զ�����״̬����

			if (LaserModuleGroupState == 0)		// ���������ģ����״̬����
			{
				break;	// ����ѭ��
			}

			if (i >= 3)
			{
				//break;
				return LaserModuleGroupState;		// �������3�δ򿪼�����ʧ�ܣ���ֹͣ������ģ�����ʼ�������ؼ�����ģ����״̬
			}

			osDelay(1000);		// ��ʱ1000ms������
		}
	}
    
    osDelay(100);

	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule1);	// ������ģ��1�����Զ�����״̬����
	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule2);	// ������ģ��2�����Զ�����״̬����

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleGroupState = 0;

	// ���ô򿪼�����������
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	LaserModuleGroupState |= MyUART_Transmit(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD), 10);		// ���ʹ򿪼�����������

	if (xQueueReceive(LaserModuleData->ConfigurationData.ReceiveQueue, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(50)) == pdPASS)	// �ȴ����ռ�����ģ���Ӧ������
	{
		if (strcmp(LaserPositionin_Rx_Buff, CMD) == 0)		// �ԱȽ��յ����ݺͷ��͵�����
		{
			return 0;	// ������ģ��״̬����
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x02;		// ������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȶ�У�鲻ͨ��
			return 1;	// ������ģ��״̬�쳣
		}
	}
	else
	{
		LaserModuleData->MeasurementData.State |= 0x01;	// ������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
		return 1;	// ������ģ��״̬�쳣
	}

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;	// ������ģ��״̬����

	// ���������Զ�����������
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	LaserModuleState |= MyUART_Transmit(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD), 10);		// �������ÿ�ʼ�����Զ�����ģ�������

	return LaserModuleState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;	// ������ģ��״̬����

	// ���������Զ�����������
	uint8_t CMD[1] = { 0x58 };

	LaserModuleState |= MyUART_Transmit(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD), 5);		// ��������ֹͣ�����Զ�����ģ�������

	return LaserModuleState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// ������ģ��״̬����

	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule1);	// ������ģ��1��ȡ�������
	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule2);	// ������ģ��2��ȡ�������

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;		// ������ģ��״̬����

	if (xQueueReceive(LaserModuleData->ConfigurationData.ReceiveQueue, LaserPositionin_Rx_Buff, pdFALSE) == pdPASS)
	{
		uint32_t Distance =
			(LaserPositionin_Rx_Buff[6] << 24) |
			(LaserPositionin_Rx_Buff[7] << 16) |
			(LaserPositionin_Rx_Buff[8] << 8) |
			(LaserPositionin_Rx_Buff[9] << 0);		// ���ղ��������
	
		uint16_t SignalQuality =
			(LaserPositionin_Rx_Buff[10] << 8) |
			(LaserPositionin_Rx_Buff[11] << 0);		// ���ղ������ź�����
	
		uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// ����У��ֵ
	
		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// ����У��ֵ
		}
	
		if (CheckValueReceive == CheckValueCalculation)
		{
			LaserModuleData->MeasurementData.Distance = Distance;				// ���¼�����ģ��1�ľ�������
			LaserModuleData->MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x04;		// ������ģ��������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
			LaserModuleState |= 0x01;							// ������ģ��״̬�쳣
		}
	}
	else
	{
		//LaserModuleData->MeasurementData.State |= 0x08;		// ������ģ��������󣬴���ԭ��δ���յ����ݰ�
		//LaserModuleState |= 0x01;							// ������ģ��״̬�쳣
	}

	return LaserModuleState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserPositioning_YawJudgment(float* Yaw)
{
	// TODO
}

static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser)
{
#define FrontLaserDistanceOffset_X	0							// ǰ����X�ᰲװ����ƫ��������λ��mm
#define FrontLaserDistanceOffset_Y	0							// ǰ����Y�ᰲװ����ƫ��������λ��mm
#define RightLaserDistanceOffset_X	0							// �Ҽ���X�ᰲװ����ƫ��������λ��mm
#define RightLaserDistanceOffset_Y	0							// �Ҽ���Y�ᰲװ����ƫ��������λ��mm
#define YawOffset					0.f						// ƫ����ƫ��������λ����
//#define FrontLaserAngleOffset_ActualDistance		0		// ǰ���ⰲװ�Ƕ�ƫ����_ʵ�ʾ��룬��λ��mm
#define FrontLaserAngleOffset_OffsetDistance		0			// ǰ���ⰲװ�Ƕ�ƫ����_ƫ�ƾ��룬��λ��mm
#define FrontLaserAngleOffset_MeasurementDistance	0			// ǰ���ⰲװ�Ƕ�ƫ����_�������룬��λ��mm
//#define RightLaserAngleOffset_ActualDistance		0			// �Ҽ��ⰲװ�Ƕ�ƫ����_ʵ�ʾ��룬��λ��mm
#define RightLaserAngleOffset_OffsetDistance		0			// �Ҽ��ⰲװ�Ƕ�ƫ����_ƫ�ƾ��룬��λ��mm
#define RightLaserAngleOffset_MeasurementDistance	0			// ǰ���ⰲװ�Ƕ�ƫ����_�������룬��λ��mm

	Yaw += ((float)YawOffset * PI / 180.0f);	// ƫ����ƫ����У��
	
	//float FrontLaserAngleOffset = atan((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_ActualDistance);
	//float RightLaserAngleOffset = atan(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_ActualDistance);

	float FrontLaserAngleOffset = asinf((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_MeasurementDistance);
	float RightLaserAngleOffset = asinf(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_MeasurementDistance);

	WorldXYCoordinates->Y = -(((float)FrontLaser * sinf(Yaw - FrontLaserAngleOffset)) / 1000.0f);
	WorldXYCoordinates->X = -(((float)RightLaser * sinf(Yaw - RightLaserAngleOffset)) / 1000.0f);

	WorldXYCoordinates->X += (float)RightLaserDistanceOffset_X / 1000.0f;
	WorldXYCoordinates->Y += (float)FrontLaserDistanceOffset_Y / 1000.0f;
}

static void LaserPositioning_GetYaw(float* Yaw)
{
#define PositionYaw_PositiveDirection 1		// Positionƫ����������1��ʾ�ͼ��ⶨλ��������ͬ��-1��ʾ�ͼ��ⶨλ�������෴
#define PositionYaw_Offset 0.0f				// Positionƫ����ƫ��������λ�Ƕȣ��Լ��ⶨλ������Ϊ0�ȣ���������ʱ��Ϊ�����򣬷�Χ��-180��180֮��

	float PositionYaw = 0.0f;		// ƫ���Ǳ�������λ����

	GetPositionYaw(&PositionYaw);		// ��ȡPosition��ƫ���ǣ���λ����

	// ����ϵת��
	*Yaw = (PositionYaw_PositiveDirection * PositionYaw) + (PositionYaw_Offset * PI / 180.0f);		// ��Position��ƫ����ת��Ϊ���ⶨλ��ƫ���ǣ���λ����
}

/**
 * @brief		���Position��ƫ����
 * @param[in]	float* Yaw ƫ����ָ�룬��λ����
 * @return		��
 * @note		ƫ���ǵĵ�λ�ǻ��ȣ���Χ��-PI��PI֮��
 */
static void GetPositionYaw(float* Yaw)
{
	// ��ʵ��
}

static uint8_t LaserPositioning_XYWorldCoordinatesVerification(const WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw)
{
	// TODO
}

static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
#define PositionXYCoordinates_Direction 1   // Position��XY����ϵ����1��ʾ�뼤�ⶨλ��ͬ������������ϵ��-1��ʾ�뼤�ⶨλ�෴������������ϵ
#define PositionXYCoordinates_XAngleOffset 0.0f	// Position��XY����ϵX��Ƕ�ƫ��������λ�Ƕȣ��Լ��ⶨλ������Ϊ0�ȣ���������ʱ��Ϊ�����򣬷�Χ��-180��180֮��
#define PositionXYCoordinates_OriginOffset_X 0.0f	// Position������ԭ��X����ƫ��������λ��m���Լ��ⶨλ����ԭ��Ϊ�ο���
#define PositionXYCoordinates_OriginOffset_Y 0.0f	// Position������ԭ��Y����ƫ��������λ��m���Լ��ⶨλ����ԭ��Ϊ�ο���

	float Position_X;		// ��λ��m
	float Position_Y;		// ��λ��m

	// �����ⶨλ����������ϵXY����ת��ΪPosition����������ϵXY����
	Position_X = cosf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->X + PositionXYCoordinates_OriginOffset_X) + sinf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->Y + PositionXYCoordinates_OriginOffset_Y);	// Position��X�������
	Position_Y = -PositionXYCoordinates_Direction * sinf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->X + PositionXYCoordinates_OriginOffset_X) + PositionXYCoordinates_Direction * cosf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->Y + PositionXYCoordinates_OriginOffset_Y);		// Position��Y�������

	// ����Position����������ϵXY��������
	SendPositionXYCoordinates(&(WorldXYCoordinatesTypedef){.X = Position_X, .Y = Position_Y});	// ����Position����������ϵXY��������
}

/**
 * @brief		����Position����������ϵXY��������
 * @param[in]	WorldXYCoordinatesTypedef* WorldXYCoordinates ��������ϵXY��������ָ��
 * @return		��
 * @note		��������ϵXY�������ݵĵ�λ��m
 */
static void SendPositionXYCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
	// ��ʵ��
}

static uint8_t MyUART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	UART_Status = HAL_UART_Transmit(huart, pData, Size, Timeout);

	if (UART_Status == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

static uint8_t MyUART_Receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	UART_Status = HAL_UART_Receive(huart, pData, Size, Timeout);

	if (UART_Status == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	UART_Status = HAL_UART_Transmit_DMA(huart, pData, Size);

	if (UART_Status == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
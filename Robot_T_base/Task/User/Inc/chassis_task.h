#pragma once

#include "data_pool.h"
#include "chassis_swerve.h"
#include "chassis_omni.h"
#include "launcher.h"
#include "speed_calculate.h"


#ifdef __cplusplus


typedef enum ROBOT_CRTL_E      
{
    OFF_MODE,   //����
    ROAD_AUTO_MODE, //����
    BALL_MODE,  //���˷�
}ROBOT_CRTL_E;

typedef enum RELOCATTION_E{
    NOT,       
    BY_LASAER, //ʹ�ü����ض�λ
    BY_VISION, //ʹ���Ӿ��ض�λ
}RELOCATTION_E;

/*---------------------------------------------------------*/

typedef enum CHASSIS_CRTL_E     //����
{
    CHASSIS_OFF,                //����
    CHASSIS_COM_MODE,           //��ͨ�ƶ� 
    CHASSIS_LOW_MODE,           //����ģʽ

    CHASSIS_LOCK_TARGET,
    CHASSIS_CAMERA_CALIBRA,   //����궨ģʽ
}CHASSIS_CRTL_E;




void PidParamInit(void);

typedef struct CONTROL_T
{
    Robot_Twist_t       twist;
    ROBOT_CRTL_E        robot_crtl;         
    
    CHASSIS_CRTL_E      chassis_ctrl;       
    uint8_t add_cnt=0;
}CONTROL_T;


typedef enum SHOOT_JUDGEMENT_E{
    VISION,
    POSITION,
}SHOOT_JUDGEMENT_E;

extern "C" {
#endif
void Chassis_Task(void *pvParameters);
void Shoot_JudgeTask(void *pvParameters);


#ifdef __cplusplus
}


extern Omni_Chassis chassis;

#endif

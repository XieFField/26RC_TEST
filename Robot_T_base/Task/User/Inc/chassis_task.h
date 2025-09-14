#pragma once

#include "data_pool.h"
#include "chassis_swerve.h"
#include "chassis_omni.h"
#include "launcher.h"
#include "speed_calculate.h"

#ifdef __cplusplus

#define MACHINE_VISION  1 //ֵΪ1�������Ӿ�Ͷ����ֵΪ0������positionͶ��

typedef enum ROBOT_CRTL_E      
{
    OFF_MODE,   //����
    SHOOT_MODE, //����
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

    CHASSIS_CALIBRA_MODE,       //У׼ģʽ������У׼ģʽ
    CHASSIS_DRIBBLE_LOW,

    CHASSIS_LOCK_TARGET,

}CHASSIS_CRTL_E;


/*---------------------------------------------------------*/

typedef enum PITCH_CRTL_E       //����
{
    PITCH_RESET_MODE,           //���ã��ŵ���ͽǶ�
    PITCH_LOCK_MODE,            //�����Ƕ�����
    PITCH_HAND_MODE,            //�ֲٸ���(����)  ûɶ��
    PITCH_AUTO_MODE,            //�Զ�����(����)  ûɶ��
    PITCH_CATCH_MODE,           
    PITCH_DRIBBLE_MODE,
    PITCH_DRIBBLE_RESET_MODE,
}PITCH_CRTL_E;

/*---------------------------------------------------------*/

/*---------------------------------------------------------*/

typedef enum FRICTION_CTRL_E    //Ħ����
{
    FRICTION_OFF_MODE,          //Ħ���ֹر�
    FRICTION_ON_MODE,           //Ħ���ֿ���
}FRICTION_CTRL_E;

typedef enum SHOOT_CTRL_E        //�������
{
    SHOOT_OFF,                   //��λ
    SHOOT_ON,                    //����
}SHOOT_CTRL_E;

typedef enum CATCH_BALL_E   //�������
{
    CATCH_ON,
    CATCH_OFF,
}CATCH_BALL_E;

typedef enum CAR_COMMUICA_E     //˫��ͨѶ   
{
    CAR_COMMUICA_OFF,       //
    CAR_COMMUICA_ON,        //
}CAR_COMMUICA_E;

/*---------------------------------------------------------*/

typedef enum LASER_CALIBRA_E    //����У׼
{
    LASER_CALIBRA_OFF,      //
    LASER_CALIBRA_ON,       //
}LASER_CALIBRA_E;

/*---------------------------------------------------------*/

/*---------------------------------------------------------*/

typedef enum DRIBBLE_E
{
    DRIBBLE_ON,
    DRIBBLE_CATCH_ON,
    DRIBBLE_OFF,
}DRIBBLE_E;


void PidParamInit(void);

typedef struct CONTROL_T
{
    Robot_Twist_t       twist;
    ROBOT_CRTL_E        robot_crtl;         
    
    CHASSIS_CRTL_E      chassis_ctrl;       
    PITCH_CRTL_E        pitch_ctrl;        
    FRICTION_CTRL_E     friction_ctrl;      
    SHOOT_CTRL_E        shoot_ctrl;            
    CATCH_BALL_E        catch_ball;
    CAR_COMMUICA_E      car_comm_ctrl;      
    LASER_CALIBRA_E     laser_ctrl;         
    DRIBBLE_E           dribble_ctrl;
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
extern Launcher launch;
#endif

#pragma once

#include "data_pool.h"
#include "chassis_swerve.h"
#include "chassis_omni.h"
#include "launcher.h"
#include "speed_calculate.h"


#ifdef __cplusplus


typedef enum ROBOT_CRTL_E      
{
    OFF_MODE,   //待机
    ROAD_AUTO_MODE, //发射
    BALL_MODE,  //接运放
}ROBOT_CRTL_E;

typedef enum RELOCATTION_E{
    NOT,       
    BY_LASAER, //使用激光重定位
    BY_VISION, //使用视觉重定位
}RELOCATTION_E;

/*---------------------------------------------------------*/

typedef enum CHASSIS_CRTL_E     //底盘
{
    CHASSIS_OFF,                //待机
    CHASSIS_COM_MODE,           //普通移动 
    CHASSIS_LOW_MODE,           //低速模式

    CHASSIS_LOCK_TARGET,
    CHASSIS_CAMERA_CALIBRA,   //相机标定模式
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

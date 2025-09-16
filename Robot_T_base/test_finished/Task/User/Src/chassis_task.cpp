/**
 * @file chassis_task.cpp
 * @author Wu Jia
 * @brief 机构任务调用
 * @version 1.8218319532
 * @attention 由于和视觉建图，所以代码里保留了不少测试功能，可视情况修改
 * 
 * @version 0.2 视觉与position数据采样抉择
 * 
 * @version 0.3 以后这份代码将包含挑战赛和竞技赛两套坐标系以及状态机，投篮赛我将把接球机构的控制移除，
 *              只保留投篮机构的控制，防止出现操作手，也就是我的紧张而引起的错误操作。
 *              更换挑战赛和竞技赛的模式选择需要前往speed_action.h文件修改CHANGE_MODE的值
 *              
 *              模式判断可以直接通过灯带判断，若为挑战赛模式则为常态黄灯
 *                                         若为竞技赛模式则为常态蓝灯
 * 
 *              当为挑战赛模式时候，lora的双车通讯会被关闭；改为竞技赛模式才会打开
 * 
 * @attention   当值为1的时候为挑战赛模式(篮筐坐标(0.0f,0.0f)，启动时候车屁股抵住底板，面向对面篮筐，
 *              启动时候需先用激光重定位).
 *              当值为0的时候为竞技赛模式(篮筐坐标(0.0f,13.096f)，启动时候车屁股抵住底板，面向对面篮筐，
 *              启动时需先试用激光重定位).
 * 
 *              position的坐标系为左手系，即站在车启动位置看，x轴正方向为左方，y轴正方向为前方。
 *              
 *              目前气压大和气压小的球的出射速度不同，后面可以考虑打两套表，通过改宏定义的值来选择
 *              之后上场时候的一分钟调试时间就好似绝佳的测试机会，应当好好把握。
 * 
 * @date    2025-8-13
 * @brief   删去一些先前的测试语句
 * 
 * 
 * @version Final
 * 
 * @version Restart
 * @brief   现在被改成测试车了，删掉了很多不需要的东西
 * @date    2025/9/14
 * 

 */
#include <cmath> // Add this at the top of the file if not already included
#include "chassis_task.h"
#include "speed_plan.h"
#include "shoot.h"
#include "position.h"
#include "drive_uart.h"
#include "LaserPositioning_Task.h"
#include "ViewCommunication.h"
#include "drive_uart.h"
#include "speed_action.h"

#define OLD_TEST 0
extern int yaw_flag;
extern float receivex;
extern float receivey;
extern float receiveyaw;
extern int init_flag;
PID_T yaw_pid = {0};
PID_T omega_pid = {0};
extern ReceiveRealData_S Pos_Now ;
extern ReceiveRealData_S Pos_Target ;
PID_T vision_pid = {0};
PID_T vision_yaw_pid = {0};
Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 3, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
CONTROL_T ctrl;

#if CHANGE_MODE
float HOOP_X = 0.000000000f;
float HOOP_Y = 0.000000000f;
#else

    #if TEST
        float HOOP_X = 0.000000000f;
        float HOOP_Y = 0.000000000f;
    #else

        float HOOP_X = 0.000000000f;
        float HOOP_Y = 13.096000000f;

    #endif

#endif

void Chassis_Task(void *pvParameters)
{
    static ReceiveRealData_S Robot_PosData = {0};
    static ReceiveRealData_S Target_PosData = {0};
    for(;;)
    {   
        /*用于测试*/

      if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
      {

        /*==底盘控制==*/
           if(ctrl.chassis_ctrl == CHASSIS_COM_MODE)
           {
               //普通控制模式
               chassis.Control(ctrl.twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_CALIBRA_MODE)
           {
               //激光校准模式
               //还没做
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_LOW_MODE) //低速模式
           {
            #if TEST
                = ctrl.twist.linear.x * (2.7 / 3.7)  * 0.2;
                ctrl.twist.linear.y = ctrl.twist.linear.y * (2.7 / 3.7) * 0.2;
            #else
                ctrl.twist.linear.x = ctrl.twist.linear.x * 0.7;
                ctrl.twist.linear.y = ctrl.twist.linear.y * 0.7;
            #endif
                ctrl.twist.angular.z = ctrl.twist.angular.z ;
                chassis.Control(ctrl.twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_OFF)
           {
               //底盘关闭
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_LOCK_TARGET)
           {
            //雷达的坐标轴和底盘坐标系不是一样的
               /*
                    Ladar_x -> robot_y
                    Ladar_y -> -1 * robot_x
               */
            #if OLD_TEST
              plan_global_speed(1.18f, 7.13f, receivey, receivex, &ctrl.twist.linear.x , &ctrl.twist.linear.y);
               
               
               
               
                speed_world_calculate(&ctrl.twist.linear.x,&ctrl.twist.linear.y); 
//               ctrl.twist.linear.x=-ctrl.twist.linear.x;
                ctrl.twist.linear.y=-ctrl.twist.linear.y; 
            #else
			   if(init_flag==1){
					plan_global_speed(-0.88f, 0.98f, Pos_Now.y, Pos_Now.x, &ctrl.twist.linear.x , &ctrl.twist.linear.y);
			   ChassisYaw_Control(0.45/M_PI*180,Pos_Now.yaw*180/M_PI,&ctrl.twist.linear.z);
			   }
			   else if(init_flag == 2)
					plan_global_speed(Pos_Target.y, Pos_Target.x, Pos_Now.y, Pos_Now.x, &ctrl.twist.linear.x , &ctrl.twist.linear.y);
                speed_world_calculate(&ctrl.twist.linear.x,&ctrl.twist.linear.y);
                ctrl.twist.linear.y=-ctrl.twist.linear.y;

			  
            #endif
               chassis.Control(ctrl.twist);
               
           }
           else if(ctrl.chassis_ctrl == CHASSIS_DRIBBLE_LOW)
           {
                ctrl.twist.linear.x = ctrl.twist.linear.x * 0.2;
                ctrl.twist.linear.y = ctrl.twist.linear.y * 0.2;
                chassis.Control(ctrl.twist);
           }
           else
           {
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           /*=================================================================*/


            chassis.Motor_Control();
       }
       
        osDelay(1);
    }
}

void PidParamInit(void)
{       
    chassis.Pid_Param_Init(0, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(1, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(2, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 

    chassis.Pid_Mode_Init(0, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(1, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(2, 0.1f, 0.0f, false, true);

//    //用于控制目标角度的角速度pid
    pid_param_init(&yaw_pid, PID_Position, 2.5, 0.0f, 0, 0.12f, 360, 0.7, 0, 0.1);
    pid_param_init(&vision_yaw_pid, PID_Position, 2.5, 0.0f, 0, 0.12f, 360, 0.82f, 0, 0.055f);
    pid_param_init(&omega_pid, PID_Incremental, 1.5, 0.0f, 0, 0.065f, 360, 0.3, 0.0008,0.02);
    pid_param_init(&vision_pid, PID_Incremental, 1.5, 0.0f, 0, 0.065f, 360, 0.60f, 0.0008f, 0);
    

    plan_global_init();
}



float error_read = 0.0f;

void Shoot_JudgeTask(void *pvParameters)
{

    for(;;)
    {
         osDelay(1);
    }
    osDelay(1);
}



